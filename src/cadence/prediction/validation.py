"""Validate the corridor queue model against SUMO.

For each scenario, one SUMO run records, at regular start times, the true
state of every vehicle (lane, position, speed, remaining route) and the
insertion backlog, plus per-second queues by approach, the signal states it
showed, and SUMO's own record of when each vehicle left each approach. The
model is then initialized from each true state and rolled forward over the
horizon with the true future entries and the true signal states, and its
predictions are compared with what SUMO did.

Because the model receives true state, true demand and true signals, its
errors are structural: they measure what the simplification costs, not
estimation error.
"""

from __future__ import annotations

import bisect
import math
import pickle
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

from cadence.controllers.phasing import signal_links
from cadence.prediction.queue_model import CorridorQueueModel, ModelParams, Vehicle
from cadence.simulation import demand as dm
from cadence.simulation.network import (
    APPROACHES, DEPARTURE_DIRECTION, INTERSECTIONS, approach_edge, approach_edges_upstream, departure_edge,
)

HALTING = 0.1
QUEUED_SPEED = 1.0   # vehicles slower than this on an approach start in the model's stop-line queue


def edge_maps(spec: dict):
    """approach edge -> (intersection, approach); (approach edge, departure edge) -> turn;
    edge -> (intersection, approach) for every edge of an approach link."""
    last = {approach_edge(i, a, spec): (i, a) for i in INTERSECTIONS for a in APPROACHES}
    link = {e: (i, a) for i in INTERSECTIONS for a in APPROACHES for e in approach_edges_upstream(i, a, spec)}
    turn = {}
    for i in INTERSECTIONS:
        for a in APPROACHES:
            for (aa, t), d in DEPARTURE_DIRECTION.items():
                if aa == a:
                    turn[(approach_edge(i, a, spec), departure_edge(i, d))] = (i, a, t)
    return last, link, turn


def route_moves(edges: list[str], start: int, turn_map) -> list[tuple[str, str, str]]:
    """Movements a route makes from edge index `start` on."""
    return [turn_map[(e1, e2)] for e1, e2 in zip(edges[start:], edges[start + 1:]) if (e1, e2) in turn_map]


# ---------------------------------------------------------------------------
# Truth collection (one SUMO run per scenario)
# ---------------------------------------------------------------------------

@dataclass
class Truth:
    scenario: str
    begin_s: float
    end_s: float
    starts: list[float]
    snapshots: dict[float, dict] = field(default_factory=dict)
    system_count: dict[int, int] = field(default_factory=dict)        # second -> backlog + network
    queues: dict[int, dict] = field(default_factory=dict)             # second -> {(i, a): halted}
    state_changes: dict[str, list[tuple[int, str]]] = field(default_factory=dict)
    discharges: list[tuple[float, tuple[str, str, str]]] = field(default_factory=list)
    routes: dict[str, list[str]] = field(default_factory=dict)
    scheduled: dict[str, float] = field(default_factory=dict)


def collect_truth(scenario: str, workdir: Path, horizon_s: float = 300.0, every_s: float = 300.0,
                  warmup_s: float | None = None, replication: int = 0, max_starts: int | None = None) -> Truth:
    import libsumo

    from cadence.paths import load_spec
    from cadence.simulation.runner import CorridorSimulation

    spec = load_spec()
    w = dm.scenario_window(scenario, spec)
    warm = spec["scenarios"]["warmup_s"] if warmup_s is None else warmup_s
    starts = [w.start + warm + k * every_s for k in range(int((w.end - w.start - warm - horizon_s) // every_s) + 1)]
    starts = starts[:max_starts] if max_starts else starts
    vr = Path(workdir) / "vehroutes.xml"
    sim = CorridorSimulation(scenario, replication, workdir, end_s=min(w.end, starts[-1] + horizon_s + 1), extra_sumo_args=(
        "--vehroute-output", str(vr), "--vehroute-output.exit-times", "true",
        "--vehroute-output.write-unfinished", "true"))
    sim.start()
    _, link, _ = edge_maps(spec)
    lanes_of = defaultdict(list)
    for e, key in link.items():
        for k in range(len(sim.net_lanes[e])):
            lanes_of[key].append(f"{e}_{k}")
    truth = Truth(scenario, sim.begin_s, sim.end_s, starts)
    pending_starts = list(starts)
    try:
        while sim.tick < int(sim.end_s * 10):
            t = sim.tick / 10
            if pending_starts and t >= pending_starts[0] - 1e-9:
                truth.snapshots[pending_starts.pop(0)] = _snapshot(libsumo)
            sim.step()
            if sim.tick % 10 == 0:
                sec = sim.tick // 10
                last = sim.ledger.last
                truth.system_count[sec] = last.backlog + last.in_network
                truth.queues[sec] = {k: sum(libsumo.lane.getLastStepHaltingNumber(l) for l in ls)
                                     for k, ls in lanes_of.items()}
        truth.state_changes = {i: list(v) for i, v in sim.record.state_changes.items()}
    finally:
        sim.close()
    truth.scheduled = {v.get("id"): float(v.get("depart")) for v in ET.parse(sim.files["routes"]).getroot().iter("vehicle")}
    _, _, turn = edge_maps(spec)
    for v in ET.parse(vr).getroot().iter("vehicle"):
        r = v.find("route")
        edges = r.get("edges").split()
        truth.routes[v.get("id")] = edges
        exits = [float(x) for x in (r.get("exitTimes") or "").split()]
        for k, (e1, e2) in enumerate(zip(edges, edges[1:])):
            if (e1, e2) in turn and k < len(exits) and exits[k] >= 0:
                truth.discharges.append((exits[k], turn[(e1, e2)]))
    truth.discharges.sort()
    return ensure_routes(truth, sim.files["routes"])


def ensure_routes(truth: Truth, route_file: Path) -> Truth:
    """Planned routes from the route file for every vehicle (no rerouting, so
    they equal the driven routes). Vehicles still waiting to enter when the
    run ended are absent from SUMO's route output."""
    for v in ET.parse(route_file).getroot().iter("vehicle"):
        truth.routes.setdefault(v.get("id"), v.find("route").get("edges").split())
    return truth


def _snapshot(libsumo) -> dict:
    vehs = {}
    for v in libsumo.vehicle.getIDList():
        vehs[v] = (libsumo.vehicle.getLaneID(v), libsumo.vehicle.getLanePosition(v), libsumo.vehicle.getSpeed(v),
                   libsumo.vehicle.getRouteIndex(v))
    return {"vehicles": vehs, "pending": list(libsumo.simulation.getPendingVehicles())}


# ---------------------------------------------------------------------------
# Model initialization and prediction
# ---------------------------------------------------------------------------

def signal_lookup(state_changes: dict[str, list[tuple[int, str]]], spec: dict):
    links = {i: signal_links(i, spec=spec) for i in INTERSECTIONS}
    ticks = {i: [t for t, _ in state_changes[i]] for i in INTERSECTIONS}
    by_move = {i: {} for i in INTERSECTIONS}
    for i in INTERSECTIONS:
        for l in links[i]:
            by_move[i].setdefault((l.approach, l.turn), l.index)
    cache = {}

    def fn(i: str, t: float) -> dict:
        key = (i, int(round(t * 10)))
        if key not in cache:
            k = bisect.bisect_right(ticks[i], key[1]) - 1
            state = state_changes[i][max(k, 0)][1]
            cache[key] = {m: state[idx] for m, idx in by_move[i].items()}
        return cache[key]
    return fn


def initialize(model: CorridorQueueModel, truth: Truth, t0: float, horizon_s: float, spec: dict,
               net_lanes: dict[str, list[float]]) -> None:
    last, link, turn = edge_maps(spec)
    v_ff = model.p.free_speed_mps
    snap = truth.snapshots[t0]
    model.reset(t0)
    queued = defaultdict(list)
    for vid, (lane, pos, speed, ridx) in snap["vehicles"].items():
        edges = truth.routes[vid]
        if lane.startswith(":"):
            moves = route_moves(edges, ridx + 1, turn)
            veh = Vehicle(vid, moves)
            if moves:
                veh.t_arrive = t0 + model.approaches[moves[0][:2]].travel_s
                model.add_in_transit(veh)
            else:
                veh.exit_time = t0 + model.exit_s[("side", "")] if edges[-1][0] in "NS" else t0 + 10.0
                model.add_exiting(veh)
            continue
        edge = lane.rsplit("_", 1)[0]
        lane_idx = int(lane.rsplit("_", 1)[1])
        moves = route_moves(edges, ridx, turn)
        veh = Vehicle(vid, moves)
        if edge in link and moves and moves[0][:2] == link[edge]:
            key = link[edge]
            ap_edges = approach_edges_upstream(key[0], key[1], spec)
            rest = net_lanes[edge][lane_idx] - pos + sum(net_lanes[e][0] for e in ap_edges[ap_edges.index(edge) + 1:])
            if speed < QUEUED_SPEED:
                queued[key].append((rest, lane_idx if edge == ap_edges[-1] else None, veh))
            else:
                veh.t_arrive = t0 + rest / v_ff
                model.add_in_transit(veh)
        else:  # on an exit link
            veh.exit_time = t0 + max(0.0, net_lanes[edge][lane_idx] - pos) / v_ff
            model.add_exiting(veh)
    for key, items in queued.items():
        ap = model.approaches[key]
        for rest, lane_idx, veh in sorted(items, key=lambda x: x[0]):
            turn_ = veh.moves[0][2]
            if lane_idx is not None and ap.bay is not None and turn_ == "L" and lane_idx == len(ap.lanes):
                ap.bay.queue.append(veh)
            elif lane_idx is not None and lane_idx < len(ap.lanes) and turn_ in ap.lanes[lane_idx].movements:
                ap.lanes[lane_idx].queue.append(veh) if (model.p.fifo or len(ap.lanes[lane_idx].movements) == 1) \
                    else ap.lanes[lane_idx].split.setdefault(turn_, __import__("collections").deque()).append(veh)
            else:
                model.add_queued(veh)
    for vid in snap["pending"]:
        edges = truth.routes[vid]
        model.add_entry(Vehicle(vid, route_moves(edges, 0, turn)), truth.scheduled[vid])
    for vid, sched in truth.scheduled.items():
        if t0 < sched <= t0 + horizon_s and vid in truth.routes:
            model.add_entry(Vehicle(vid, route_moves(truth.routes[vid], 0, turn)), sched)


@dataclass
class Comparison:
    t0: float
    vt_model: float
    vt_sumo: float
    queue_model: dict          # mean queue over the horizon, by approach
    queue_sumo: dict
    discharge_model: Counter
    discharge_sumo: Counter
    end_queue_model: dict = field(default_factory=dict)
    end_queue_sumo: dict = field(default_factory=dict)

    @property
    def vt_rel_error(self) -> float:
        return (self.vt_model - self.vt_sumo) / self.vt_sumo


def predict_and_compare(truth: Truth, t0: float, horizon_s: float, spec: dict, net_lanes,
                        params: ModelParams | None = None) -> Comparison:
    model = CorridorQueueModel(params, spec)
    model.set_signals(signal_lookup(truth.state_changes, spec))
    initialize(model, truth, t0, horizon_s, spec, net_lanes)
    model.run(t0 + horizon_s, record_queues_every=1)
    vt_model = float(sum(n for _, n in model.system_count))
    s0 = int(round(t0))
    secs = range(s0 + 1, s0 + int(horizon_s) + 1)
    vt_sumo = float(sum(truth.system_count.get(s, 0) for s in secs))
    keys = list(model.approaches)
    mean_model = {k: sum(q[k] for _, q in model.queue_count) / len(model.queue_count) for k in keys}
    mean_sumo = {k: sum(truth.queues.get(s, {}).get(k, 0) for s in secs) / len(secs) for k in keys}
    q_end = int(round(t0 + horizon_s))
    times = [d[0] for d in truth.discharges]
    lo, hi = bisect.bisect_right(times, t0), bisect.bisect_right(times, t0 + horizon_s)
    return Comparison(t0, vt_model, vt_sumo, mean_model, mean_sumo,
                      Counter(m for t, m in model.discharges), Counter(m for t, m in truth.discharges[lo:hi]),
                      model.queued(), truth.queues.get(q_end, {}))


def save_truth(truth: Truth, path: Path) -> None:
    with open(path, "wb") as fh:
        pickle.dump(truth, fh)


def load_truth(path: Path) -> Truth:
    with open(path, "rb") as fh:
        return pickle.load(fh)
