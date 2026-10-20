"""Simplified corridor prediction model (Milestone 1).

This is the kind of model an optimizer would roll forward over a short
horizon: much simpler than SUMO, but with the mechanisms that matter on
Corridor R-1. It is a mesoscopic, discrete-vehicle queue model in 1 s
steps:

* Each approach lane at the stop line holds a first-in-first-out queue, so
  a vehicle that cannot go blocks every vehicle behind it (head-of-line
  blocking). This matters most in I2's single shared side-street lane,
  where a permissive left-turner waiting for a gap holds up through and
  right-turning traffic.
* Left-turn bays have finite length. A left-turner that finds the bay full,
  or cannot reach it past the through queue, waits in the adjacent through
  lane and blocks it until it can enter the bay.
* Each link has finite storage (Document Zero usable storage). A vehicle
  cannot be discharged into a full link, so queues spill back.
* Discharge follows saturation headways per lane, with start-up lost time at
  the beginning of green and use of the first seconds of yellow.
* Permissive lefts (I2 side streets) need a gap: the opposing approach has
  no standing queue, no vehicle discharged in the last few seconds, and no
  vehicle arriving in the next few seconds.
* Between stop lines vehicles travel at free-flow speed and join the next
  queue when they reach its stop line (point queue in time).

The model needs: the current state (vehicles with their remaining routes),
future entries, and the signal states over the horizon. In the validation it
is given SUMO's true state, true future entries and the signal states SUMO
actually showed, so its errors are structural, not estimation errors.

`fifo=False` turns off head-of-line blocking in shared lanes (each movement
in a shared lane behaves as if it had its own lane) for comparison.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from cadence.paths import R1_DIR, load_spec
from cadence.simulation.demand import next_intersection
from cadence.simulation.network import (
    APPROACHES, DEPARTURE_DIRECTION, INTERSECTIONS, TURNS, approach_edge, approach_edges_upstream, departure_edge,
    movement_lanes,
)

OPPOSING = {"EB": "WB", "WB": "EB", "NB": "SB", "SB": "NB"}


@dataclass
class ModelParams:
    sat_headway_s: float = 2.0         # DZ 3.2: 1800 veh/h/lane
    startup_lost_s: float = 2.0
    yellow_use_s: float = 2.0
    free_speed_mps: float = 13.89
    jam_spacing_m: float = 7.5
    # permissive left: no opposing through/right departure in the last ... and no
    # arrival in the next ... seconds (calibrated on S1: 0 and 0, see
    # docs/validation/prediction_model_m1.md)
    gap_before_s: float = 0.0
    gap_after_s: float = 0.0
    entry_headway_s: float = 2.0       # insertion headway per entry lane
    fifo: bool = True


@dataclass
class Lane:
    movements: tuple[str, ...]
    queue: deque = field(default_factory=deque)
    next_ok: float = -math.inf         # earliest next departure time
    is_bay: bool = False
    split: dict = field(default_factory=dict)   # per-movement queues when FIFO is off

    def queues(self) -> list[deque]:
        return [self.queue] + list(self.split.values())

    def size(self) -> int:
        return len(self.queue) + sum(len(q) for q in self.split.values())


@dataclass
class Approach:
    key: tuple[str, str]
    lanes: list[Lane]
    bay: Lane | None
    inner: int                         # index of the through lane next to the bay
    bay_cap: int
    storage: int                       # through-lane storage of the link (vehicles)
    travel_s: float                    # free-flow time from link entry to stop line
    in_transit: list = field(default_factory=list)   # vehicles travelling toward the stop line
    last_departure: float = -math.inf
    last_conflicting_departure: float = -math.inf   # last through or right-turn departure
    entry_lanes: int = 0               # >0 for links fed by an entry (insertion lanes)

    def queued(self) -> int:
        return sum(l.size() for l in self.lanes) + (self.bay.size() if self.bay else 0)

    def count(self) -> int:
        return len(self.in_transit) + self.queued()

    def through_count(self) -> int:
        return len(self.in_transit) + sum(l.size() for l in self.lanes)


@dataclass
class Vehicle:
    vid: str
    moves: list[tuple[str, str, str]]  # remaining (intersection, approach, turn)
    t_arrive: float = 0.0              # time it reaches the current stop line
    exit_time: float | None = None     # for vehicles on an exit link


def _lane_lengths(net_path: Path) -> tuple[dict[str, list[float]], dict[tuple[str, str, int, int], float]]:
    root = ET.parse(net_path).getroot()
    lanes, internal = {}, {}
    ilen = {}
    for e in root.findall("edge"):
        ls = sorted(e.findall("lane"), key=lambda l: int(l.get("index")))
        if e.get("function") == "internal":
            for l in ls:
                ilen[l.get("id")] = float(l.get("length"))
        else:
            lanes[e.get("id")] = [float(l.get("length")) for l in ls]
    for c in root.findall("connection"):
        via = c.get("via")
        if via and not c.get("from").startswith(":"):
            key = (c.get("from"), c.get("to"), int(c.get("fromLane")), int(c.get("toLane")))
            internal[key] = ilen.get(via, 0.0)
    return lanes, internal


class CorridorQueueModel:
    def __init__(self, params: ModelParams | None = None, spec: dict | None = None,
                 net_path: Path = R1_DIR / "r1.net.xml"):
        self.p = params or ModelParams()
        self.spec = spec or load_spec()
        self.net_lanes, self.internal_len = _lane_lengths(net_path)
        self.approaches: dict[tuple[str, str], Approach] = {}
        self.cross_s: dict[tuple[str, str, str], float] = {}
        self.exit_s: dict[tuple[str, str], float] = {}
        self._build()
        self.t = 0.0
        self.backlog: dict[tuple[str, str], deque] = {}
        self.exiting: list[Vehicle] = []
        self.signals = None
        self.status: dict[tuple[str, str, str], str] = {}
        self.onset: dict[tuple[str, str, str], float] = {}
        self.before_yellow: dict[tuple[str, str, str], str] = {}
        self.discharges: list[tuple[float, tuple[str, str, str]]] = []
        self.system_count: list[tuple[float, int]] = []
        self.queue_count: list[tuple[float, dict[tuple[str, str], int]]] = []

    # ------------------------------------------------------------------ network
    def _build(self) -> None:
        s, v = self.spec, self.p.free_speed_mps
        jam = self.p.jam_spacing_m
        for i in INTERSECTIONS:
            for a in APPROACHES:
                edges = approach_edges_upstream(i, a, s)
                last = approach_edge(i, a, s)
                bay_lanes = {fl for fl, _ in movement_lanes(i, a, "L", s)}
                through_lanes = {fl for t in ("T", "R") for fl, _ in movement_lanes(i, a, t, s)}
                bay_idx = bay_lanes - through_lanes
                lanes = []
                for idx in sorted(through_lanes | (bay_lanes & through_lanes)):
                    mv = tuple(t for t in TURNS if idx in {fl for fl, _ in movement_lanes(i, a, t, s)})
                    lanes.append(Lane(mv))
                bay = Lane(("L",), is_bay=True) if bay_idx else None
                bay_cap = int(round(self.net_lanes[last][max(bay_idx)] / jam)) if bay_idx else 0
                storage = 0.0
                for e in edges:
                    n = len(self.net_lanes[e]) - (1 if (e == last and bay_idx) else 0)
                    storage += n * self.net_lanes[e][0] / jam
                length = sum(self.net_lanes[e][0] for e in edges)
                # junction lanes between consecutive approach edges (bay starts, lane drop)
                for e1, e2 in zip(edges, edges[1:]):
                    ks = [L for (f, t, _, _), L in self.internal_len.items() if f == e1 and t == e2]
                    length += max(ks) if ks else 0.0
                inner = max(range(len(lanes)), key=lambda k: k) if lanes else 0
                ap = Approach((i, a), lanes, bay, inner, bay_cap, int(math.floor(storage + 1e-9)), length / v)
                if a in ("EB", "WB") and edges[0] in ("EB0a", "WB4a"):
                    ap.entry_lanes = len(self.net_lanes[edges[0]])
                if a in ("NB", "SB"):
                    ap.entry_lanes = 1
                self.approaches[(i, a)] = ap
                for t in TURNS:
                    pairs = movement_lanes(i, a, t, s)
                    if not pairs:
                        continue
                    dep = departure_edge(i, DEPARTURE_DIRECTION[(a, t)])
                    L = max(self.internal_len.get((last, dep, fl, tl), 0.0) for fl, tl in pairs)
                    self.cross_s[(i, a, t)] = L / v
        for d, e in (("EB", "EB4"), ("WB", "WB0")):
            self.exit_s[("main", d)] = self.net_lanes[e][0] / v
        self.exit_s[("side", "")] = self.spec["cross_streets"]["leg_length_m"] / v

    def downstream(self, move: tuple[str, str, str]) -> tuple[str, str] | None:
        i, a, t = move
        d = DEPARTURE_DIRECTION[(a, t)]
        if d in ("EB", "WB"):
            nxt = next_intersection(i, d)
            return (nxt, d) if nxt else None
        return None

    def exit_travel(self, move: tuple[str, str, str]) -> float:
        i, a, t = move
        d = DEPARTURE_DIRECTION[(a, t)]
        return self.exit_s[("main", d)] if d in ("EB", "WB") else self.exit_s[("side", "")]

    # ------------------------------------------------------------------ state
    def reset(self, t0: float) -> None:
        self.t = float(t0)
        for ap in self.approaches.values():
            ap.in_transit.clear()
            for l in ap.lanes + ([ap.bay] if ap.bay else []):
                l.queue.clear(); l.split.clear(); l.next_ok = -math.inf
            ap.last_departure = -math.inf
            ap.last_conflicting_departure = -math.inf
        self.backlog = {k: deque() for k in self.approaches if self.approaches[k].entry_lanes}
        self._entry_next = {k: [-math.inf] * self.approaches[k].entry_lanes for k in self.backlog}
        self.exiting = []
        self.discharges, self.system_count, self.queue_count = [], [], []
        self.status, self.onset, self.before_yellow = {}, {}, {}

    def add_queued(self, veh: Vehicle) -> None:
        """Place a vehicle in the stop-line queues of its current approach, in
        call order (callers add vehicles nearest the stop line first)."""
        self._join_queue(self.approaches[veh.moves[0][:2]], veh)

    def add_in_transit(self, veh: Vehicle) -> None:
        self.approaches[veh.moves[0][:2]].in_transit.append(veh)

    def add_exiting(self, veh: Vehicle) -> None:
        self.exiting.append(veh)

    def add_entry(self, veh: Vehicle, scheduled: float) -> None:
        veh.t_arrive = scheduled
        self.backlog[veh.moves[0][:2]].append(veh)

    # ------------------------------------------------------------------ queues
    def _join_queue(self, ap: Approach, veh: Vehicle) -> None:
        turn = veh.moves[0][2]
        if turn == "L" and ap.bay is not None:
            inner = ap.lanes[ap.inner]
            if len(ap.bay.queue) < ap.bay_cap and len(inner.queue) < ap.bay_cap:
                ap.bay.queue.append(veh)
                return
            inner.queue.append(veh)   # bay full or out of reach: wait in the through lane
            return
        options = [l for l in ap.lanes if turn in l.movements]
        if not self.p.fifo and len(options) == 1 and len(options[0].movements) > 1:
            # without FIFO, each movement of a shared lane queues on its own
            options[0].split.setdefault(turn, deque()).append(veh)
            return
        min(options, key=lambda l: l.size()).queue.append(veh)

    def _transfer_to_bay(self, ap: Approach) -> None:
        if ap.bay is None:
            return
        inner = ap.lanes[ap.inner]
        for k, veh in enumerate(inner.queue):
            if k >= ap.bay_cap or len(ap.bay.queue) >= ap.bay_cap:
                break
            if veh.moves[0][2] == "L":
                del inner.queue[k]
                ap.bay.queue.append(veh)
                break

    # ------------------------------------------------------------------ signals
    def set_signals(self, fn) -> None:
        """fn(intersection, t) -> {(approach, turn): 'G'|'g'|'y'|'r'}."""
        self.signals = fn

    def _update_status(self, t: float) -> None:
        for i in INTERSECTIONS:
            for (a, tr), st in self.signals(i, t).items():
                m = (i, a, tr)
                prev = self.status.get(m)
                if st != prev:
                    if st == "y":
                        self.before_yellow[m] = prev or "G"
                    self.status[m] = st
                    self.onset[m] = t if prev is not None else -math.inf

    def _can_go(self, move, t: float) -> bool:
        st = self.status.get(move, "r")
        if st in "Gg":
            if t < self.onset[move] + self.p.startup_lost_s:
                return False
            return st == "G" or self._gap(move, t)
        if st == "y":
            # vehicles close to the stop line use the start of yellow; permissive
            # left-turners waiting at the stop line do not
            return self.before_yellow.get(move) == "G" and t < self.onset[move] + self.p.yellow_use_s
        return False

    def _gap(self, move, t: float) -> bool:
        """A permissive left needs a gap in opposing through and right-turn
        traffic (opposing lefts do not conflict)."""
        i, a, _ = move
        opp = self.approaches[(i, OPPOSING[a])]
        for lane in opp.lanes:
            for q in lane.queues():
                if q and q[0].moves[0][2] in ("T", "R") and self.status.get(q[0].moves[0], "r") in "Gg":
                    return False   # opposing queue is discharging
        if t - opp.last_conflicting_departure < self.p.gap_before_s:
            return False
        return not any(v.t_arrive <= t + self.p.gap_after_s and v.moves[0][2] in ("T", "R") for v in opp.in_transit)

    def _room(self, move) -> bool:
        down = self.downstream(move)
        if down is None:
            return True
        ap = self.approaches[down]
        return ap.through_count() < ap.storage

    # ------------------------------------------------------------------ step
    def step(self) -> None:
        t = self.t
        self._update_status(t)
        # 1. arrivals at stop lines
        for ap in self.approaches.values():
            arriving = [v for v in ap.in_transit if v.t_arrive <= t]
            if arriving:
                ap.in_transit = [v for v in ap.in_transit if v.t_arrive > t]
                for v in sorted(arriving, key=lambda v: v.t_arrive):
                    self._join_queue(ap, v)
            self._transfer_to_bay(ap)
        # 2. discharges
        for ap in self.approaches.values():
            lanes = ap.lanes + ([ap.bay] if ap.bay else [])
            for lane in lanes:
                for q in lane.queues():
                    if not q or t < lane.next_ok:
                        continue
                    veh = q[0]
                    move = veh.moves[0]
                    if move[2] == "L" and ap.bay is not None and not lane.is_bay:
                        continue  # left-turner stuck at the bay entrance blocks the lane
                    if not self._can_go(move, t) or not self._room(move):
                        continue
                    q.popleft()
                    lane.next_ok = t + self.p.sat_headway_s
                    ap.last_departure = t
                    if move[2] in ("T", "R"):
                        ap.last_conflicting_departure = t
                    self.discharges.append((t, move))
                    self._advance(veh, move, t)
        # 3. exits
        self.exiting = [v for v in self.exiting if v.exit_time > t]
        # 4. insertion from the backlog
        for key, q in self.backlog.items():
            ap = self.approaches[key]
            slots = self._entry_next[key]
            while q and q[0].t_arrive <= t and ap.through_count() < ap.storage:
                k = min(range(len(slots)), key=lambda j: slots[j])
                if slots[k] > t:
                    break
                veh = q.popleft()
                slots[k] = t + self.p.entry_headway_s
                veh.t_arrive = t + ap.travel_s
                ap.in_transit.append(veh)
        self.t = t + 1.0
        self.system_count.append((self.t, self.count_in_system()))

    def _advance(self, veh: Vehicle, move, t: float) -> None:
        veh.moves.pop(0)
        cross = self.cross_s.get(move, 0.0)
        if veh.moves:
            ap = self.approaches[veh.moves[0][:2]]
            veh.t_arrive = t + cross + ap.travel_s
            ap.in_transit.append(veh)
        else:
            veh.exit_time = t + cross + self.exit_travel(move)
            self.exiting.append(veh)

    def count_in_system(self) -> int:
        """Vehicles in the network plus scheduled vehicles waiting to enter."""
        n = len(self.exiting) + sum(ap.count() for ap in self.approaches.values())
        return n + sum(1 for q in self.backlog.values() for v in q if v.t_arrive <= self.t)

    def queued(self) -> dict[tuple[str, str], int]:
        return {k: ap.queued() for k, ap in self.approaches.items()}

    def run(self, until: float, record_queues_every: int = 0) -> None:
        k = 0
        while self.t < until:
            self.step()
            k += 1
            if record_queues_every and k % record_queues_every == 0:
                self.queue_count.append((self.t, self.queued()))
