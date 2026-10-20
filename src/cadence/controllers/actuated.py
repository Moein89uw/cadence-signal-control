"""Dual-ring actuated-coordinated signal controller (NEMA style).

This is Cadence's own controller rather than SUMO's built-in NEMA program,
because the research needs pedestrian intervals, external calls, holds and
force-offs that take effect immediately, and state that can be saved and
restored exactly. It has no SUMO dependency: it consumes phase presence
calls and pedestrian push-button presses, and exposes vehicle and
pedestrian displays. All times are integer ticks of 0.1 s, so interval
durations are exact.

Behavior
--------
* Two rings, two barrier groups (main: 1 2 | 5 6, side: 3 4 | 7 8). Phases
  in different rings run concurrently only within the same group, and the
  controller crosses a barrier only when both rings have finished their
  clearance intervals.
* Coordination with a fixed cycle, offset, fixed force-offs and a fixed
  yield point. Cycle k starts at offset + k * cycle (main-group start).
  Coordinated phases 2 and 6 are on recall: they hold until the yield
  point, then terminate together if any other phase can be served, and
  otherwise rest in green.
* A non-coordinated phase starts only if its call can be served before its
  force-off: minimum green, plus the full pedestrian interval when a walk
  will be served. It ends on gap-out (passage timer expired after minimum
  green) or at force-off, but never before minimum green and pedestrian
  clearance are complete.
* Simultaneous gap-out: the last phase of a ring before a barrier gaps out
  only when the other ring is also ready to cross.
* A ring with nothing to serve rests in red at the barrier (no dual entry)
  and re-enters if a later phase in its sequence is called while the other
  ring is still serving the group.
* Pedestrians: walk starts with the vehicle phase (or a leading pedestrian
  interval before it), then pedestrian clearance, and both always finish
  before the vehicle phase's yellow. A pedestrian call that arrives during
  green is served the same green (recycle) only if the full pedestrian
  interval fits before the yield point or force-off.

Event codes follow the Indiana high-resolution enumeration where one exists;
codes 200 and above are Cadence diagnostics.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Iterable, Mapping

from cadence.controllers.phasing import MAIN, SIDE, PhaseDef
from cadence.controllers.timing import TimingPlan

TICKS_PER_S = 10

# vehicle intervals
GREEN, YELLOW, RED_CLEAR, REST = "GREEN", "YELLOW", "RED_CLEAR", "REST"
# pedestrian intervals
DONT_WALK, WALK, PED_CLEAR = "DONT_WALK", "WALK", "PED_CLEAR"

# Indiana high-resolution event codes
EV_PHASE_BEGIN_GREEN = 1
EV_PHASE_GAP_OUT = 4
EV_PHASE_FORCE_OFF = 6
EV_PHASE_GREEN_TERMINATION = 7
EV_PHASE_BEGIN_YELLOW = 8
EV_PHASE_END_YELLOW = 9
EV_PHASE_BEGIN_RED_CLEAR = 10
EV_PHASE_END_RED_CLEAR = 11
EV_PED_BEGIN_WALK = 21
EV_PED_BEGIN_CLEAR = 22
EV_PED_BEGIN_DONT_WALK = 23
EV_PHASE_CALL_REGISTERED = 43
EV_PHASE_CALL_DROPPED = 44
EV_PED_CALL_REGISTERED = 45
EV_COORD_YIELD_POINT = 151
# Cadence diagnostics
EV_FORCE_OFF_OVERRUN = 200      # green held past force-off to finish minimum green or pedestrian clearance
EV_BARRIER_CROSS = 201          # parameter 1 = into main group, 2 = into side group
EV_PHASE_BEGIN_LPI = 202        # leading pedestrian interval begins (vehicle display still red)
EV_HELD_PAST_FORCE_OFF = 203    # green held past force-off by an external hold command
EV_CMD_CALL = 210               # external commands (parameter = phase)
EV_CMD_HOLD = 211
EV_CMD_FORCE_OFF = 212
EV_CMD_OMIT = 213

# Writable commands by controller family (Document Zero 3.5)
FAMILY_COMMANDS = {"A": frozenset({"call", "hold", "force_off", "omit"}),
                   "B": frozenset({"call", "hold", "force_off"})}


def s2t(seconds: float) -> int:
    return int(round(seconds * TICKS_PER_S))


@dataclass(frozen=True)
class PhaseTiming:
    number: int
    ring: int
    group: str
    coordinated: bool
    min_green: int
    yellow: int
    red_clear: int
    passage: int
    ped: str | None
    walk: int
    ped_clear: int
    lpi: int

    @property
    def clearance(self) -> int:
        return self.yellow + self.red_clear

    @classmethod
    def from_def(cls, d: PhaseDef, passage_s: float) -> "PhaseTiming":
        return cls(d.number, d.ring, d.group, d.coordinated, s2t(d.min_green_s), s2t(d.yellow_s),
                   s2t(d.red_clear_s), s2t(passage_s), d.ped, s2t(d.walk_s), s2t(d.ped_clear_s), s2t(d.lpi_s))


@dataclass
class RingState:
    number: int
    phase: int | None = None
    interval: str = REST
    start: int = 0          # tick the current interval began
    green_start: int = 0    # tick vehicle green begins (later than start during an LPI)
    served: list[int] = field(default_factory=list)  # phases served in the current group
    overrun_logged: bool = False
    green_logged: bool = False


@dataclass
class PedState:
    name: str
    phase: int
    state: str = DONT_WALK
    start: int = 0
    pending: bool = False


class ActuatedCoordinatedController:
    def __init__(self, intersection: str, phases: Mapping[int, PhaseDef], plan: TimingPlan,
                 start_tick: int, passage_s: float = 2.0, simultaneous_gap_out: bool = True,
                 ped_recycle: bool = True, family: str = "A"):
        self.intersection = intersection
        self.family = family
        self.allowed_commands = FAMILY_COMMANDS[family]
        # external commands: phase -> last tick (inclusive) the command applies
        self.cmd_call: dict[int, int] = {}
        self.cmd_hold: dict[int, int] = {}
        self.cmd_omit: dict[int, int] = {}
        self.cmd_force_off: set[int] = set()
        self.ph = {p: PhaseTiming.from_def(d, passage_s) for p, d in phases.items()}
        self.seq = {g: {r: list(s) for r, s in plan.sequences[g].items()} for g in (MAIN, SIDE)}
        self.cycle = s2t(plan.cycle_s)
        self.offset = s2t(plan.offset_s)
        self.simultaneous_gap_out = simultaneous_gap_out
        self.ped_recycle = ped_recycle
        self.plan = plan
        self._fo_tau, self._yield_tau = self._programmed_points(plan)
        self.rings = {r: RingState(r) for r in sorted(self.seq[MAIN])}
        self.peds = {d.ped: PedState(d.ped, p) for p, d in self.ph.items() if d.ped}
        self.ped_of = {d.number: d.ped for d in self.ph.values() if d.ped}
        self.presence = {p: False for p in self.ph}
        self.gap = {p: 0 for p in self.ph}
        # start as if at the barrier at the end of the previous cycle's side group
        k = (start_tick - self.offset) // self.cycle
        self.group = SIDE
        self.k = k - 1
        self.t = start_tick
        self._last_yield_logged: int | None = None
        self.events: list[tuple[int, int, int]] = []
        self.overruns = 0

    # ------------------------------------------------------------------ commands
    def command(self, kind: str, phase: int, duration_ticks: int = 1) -> None:
        """Apply an external command at the current tick (it takes effect at
        the next step). call: place a call; hold: keep the phase green while
        it is green; force_off: end the phase as soon as minimum green and any
        pedestrian interval allow (non-coordinated phases); omit: do not start
        the phase. Commands never shorten minimum green, clearance or
        pedestrian intervals."""
        if kind not in self.allowed_commands:
            raise ValueError(f"{self.intersection} (family {self.family}) does not accept '{kind}'")
        if phase not in self.ph:
            raise ValueError(f"{self.intersection} has no phase {phase}")
        until = self.t + max(1, duration_ticks)
        if kind == "call":
            self.cmd_call[phase] = until
            self._log(EV_CMD_CALL, phase)
        elif kind == "hold":
            self.cmd_hold[phase] = until
            self._log(EV_CMD_HOLD, phase)
        elif kind == "force_off":
            if self.ph[phase].coordinated:
                raise ValueError("force-off of a coordinated phase is not supported in Milestone 1")
            self.cmd_force_off.add(phase)
            self._log(EV_CMD_FORCE_OFF, phase)
        elif kind == "omit":
            if self.ph[phase].coordinated:
                raise ValueError("coordinated phases cannot be omitted")
            self.cmd_omit[phase] = until
            self._log(EV_CMD_OMIT, phase)

    def _active(self, table: dict[int, int], p: int, t: int | None = None) -> bool:
        return table.get(p, -1) >= (self.t if t is None else t)

    # ------------------------------------------------------------------ plan
    def _programmed_points(self, plan: TimingPlan) -> tuple[dict[int, int], int]:
        fo: dict[int, int] = {}
        main_time = s2t(plan.group_time_s(MAIN))
        yield_tau = None
        for g, base in ((MAIN, 0), (SIDE, main_time)):
            for seq in plan.sequences[g].values():
                tau = base
                for p in seq:
                    tau += s2t(plan.splits_s[p])
                    fo[p] = tau - self.ph[p].clearance
                    if self.ph[p].coordinated:
                        y = tau - self.ph[p].clearance
                        if yield_tau is not None and y != yield_tau:
                            raise ValueError("coordinated phases must share one yield point")
                        yield_tau = y
        return fo, yield_tau

    def cycle_start(self, k: int) -> int:
        return self.offset + k * self.cycle

    def force_off(self, p: int, k: int | None = None) -> int:
        return self.cycle_start(self.k if k is None else k) + self._fo_tau[p]

    def yield_point(self, k: int | None = None) -> int:
        return self.cycle_start(self.k if k is None else k) + self._yield_tau

    # ------------------------------------------------------------------ logging
    def _log(self, code: int, param: int) -> None:
        self.events.append((self.t, code, param))

    # ------------------------------------------------------------------ displays
    def vehicle_display(self, p: int) -> str:
        """'G', 'Y' or 'R' for the interval beginning at the last stepped tick."""
        r = self.rings[self.ph[p].ring]
        if r.phase != p:
            return "R"
        if r.interval == GREEN:
            return "G" if self.t >= r.green_start else "R"
        if r.interval == YELLOW:
            return "Y"
        return "R"

    def ped_display(self, name: str) -> str:
        return self.peds[name].state

    # ------------------------------------------------------------------ state
    _STATIC = ("ph", "seq", "plan", "_fo_tau", "ped_of", "events")

    def snapshot(self) -> dict:
        """Complete controller state, for save and restore. The event log is
        recorded by length: restoring truncates it."""
        snap = copy.deepcopy({k: v for k, v in self.__dict__.items() if k not in self._STATIC})
        snap["n_events"] = len(self.events)
        return snap

    def restore(self, snap: dict) -> None:
        snap = copy.deepcopy(snap)
        del self.events[snap.pop("n_events"):]
        self.__dict__.update(snap)

    # ------------------------------------------------------------------ main step
    def step(self, t: int, presence: Mapping[int, bool], ped_presses: Iterable[str] = ()) -> None:
        """Advance the controller to tick t and set displays for [t, t + 1)."""
        self.t = t
        for name in ped_presses:
            ped = self.peds.get(name)
            if ped is None:
                continue
            if ped.state != WALK and not ped.pending:
                ped.pending = True
                self._log(EV_PED_CALL_REGISTERED, int(name[1:]))
        for p, d in self.ph.items():
            now = bool(presence.get(p, False))
            if now != self.presence[p] and not d.coordinated and self.vehicle_display(p) != "G":
                self._log(EV_PHASE_CALL_REGISTERED if now else EV_PHASE_CALL_DROPPED, p)
            self.presence[p] = now
            if self.presence[p]:
                self.gap[p] = d.passage
            elif self.gap[p] > 0:
                self.gap[p] -= 1
        self._advance_peds()
        for _ in range(16):
            if not self._advance_rings():
                break
        else:  # pragma: no cover - defensive
            raise RuntimeError(f"{self.intersection}: controller did not settle at tick {t}")

    def _advance_peds(self) -> None:
        t = self.t
        for ped in self.peds.values():
            if ped.state == WALK and t - ped.start >= self.ph[ped.phase].walk:
                ped.state, ped.start = PED_CLEAR, t
                self._log(EV_PED_BEGIN_CLEAR, int(ped.name[1:]))
            elif ped.state == PED_CLEAR and t - ped.start >= self.ph[ped.phase].ped_clear:
                ped.state, ped.start = DONT_WALK, t
                self._log(EV_PED_BEGIN_DONT_WALK, int(ped.name[1:]))

    # ------------------------------------------------------------------ helpers
    def _call(self, p: int) -> bool:
        name = self.ped_of.get(p)
        return self.presence[p] or self._active(self.cmd_call, p) or (name is not None and self.peds[name].pending)

    def _ped_need(self, p: int) -> int:
        d = self.ph[p]
        return max(d.lpi + d.min_green, d.walk + d.ped_clear)

    def _can_start(self, p: int, t_start: int, k: int | None = None) -> tuple[bool, bool]:
        """(can start, will serve a walk) for phase p starting at t_start."""
        d = self.ph[p]
        name = self.ped_of.get(p)
        ped_pending = name is not None and self.peds[name].pending
        if d.coordinated:
            return True, ped_pending and t_start + d.walk + d.ped_clear <= self.yield_point(k)
        if self._active(self.cmd_omit, p, t_start):
            return False, False
        fo = self.force_off(p, k)
        if ped_pending and t_start + self._ped_need(p) <= fo:
            return True, True
        if (self.presence[p] or self._active(self.cmd_call, p)) and t_start + d.min_green <= fo:
            return True, False
        return False, False

    def _later_phases(self, ring: RingState) -> list[int]:
        seq = self.seq[self.group][ring.number]
        if not ring.served:
            return list(seq)
        last = seq.index(ring.served[-1])
        return seq[last + 1:]

    def _next_phase(self, ring: RingState, t_start: int) -> tuple[int, bool] | None:
        for p in self._later_phases(ring):
            ok, walk = self._can_start(p, t_start)
            if ok:
                return p, walk
        return None

    def _start(self, ring: RingState, p: int, walk: bool) -> None:
        t = self.t
        d = self.ph[p]
        ring.phase, ring.interval, ring.start = p, GREEN, t
        ring.served.append(p)
        ring.overrun_logged = False
        lpi = 0
        if walk:
            self._begin_walk(p)
            lpi = d.lpi
        ring.green_start = t + lpi
        self.gap[p] = d.passage if self.presence[p] else 0
        ring.green_logged = not lpi
        self._log(EV_PHASE_BEGIN_LPI if lpi else EV_PHASE_BEGIN_GREEN, p)

    def _begin_walk(self, p: int) -> None:
        ped = self.peds[self.ped_of[p]]
        ped.state, ped.start, ped.pending = WALK, self.t, False
        self._log(EV_PED_BEGIN_WALK, int(ped.name[1:]))

    def _terminate(self, ring: RingState, reason: int | None) -> None:
        p = ring.phase
        if reason is not None:
            self._log(reason, p)
        self._log(EV_PHASE_GREEN_TERMINATION, p)
        self._log(EV_PHASE_BEGIN_YELLOW, p)
        ring.interval, ring.start = YELLOW, self.t

    def _green_done(self, ring: RingState) -> bool:
        """Minimum green and any pedestrian interval complete."""
        p = ring.phase
        if self.t < ring.green_start or self.t - ring.green_start < self.ph[p].min_green:
            return False
        name = self.ped_of.get(p)
        return name is None or self.peds[name].state == DONT_WALK

    def _heading_to_barrier(self, ring: RingState) -> bool:
        if ring.interval == REST:
            return True
        if ring.interval in (YELLOW, RED_CLEAR):
            left = self.ph[ring.phase].clearance - (self.t - ring.start) if ring.interval == YELLOW \
                else self.ph[ring.phase].red_clear - (self.t - ring.start)
            return self._next_phase(ring, self.t + max(left, 0)) is None
        return False

    def _is_barrier_phase(self, ring: RingState) -> bool:
        return self._next_phase(ring, self.t + self.ph[ring.phase].clearance) is None

    def _in_gap_out(self, ring: RingState) -> bool:
        p = ring.phase
        return self._green_done(ring) and (self.gap[p] == 0 or self.t >= self.force_off(p))

    def _other(self, ring: RingState) -> RingState:
        return next(r for n, r in self.rings.items() if n != ring.number)

    # ------------------------------------------------------------------ rings
    def _advance_rings(self) -> bool:
        t = self.t
        changed = False

        # LPI end: vehicle green begins
        for ring in self.rings.values():
            if ring.interval == GREEN and not ring.green_logged and t >= ring.green_start:
                ring.green_logged = True
                self._log(EV_PHASE_BEGIN_GREEN, ring.phase)

        # clearance intervals
        for ring in self.rings.values():
            if ring.interval == YELLOW and t - ring.start >= self.ph[ring.phase].yellow:
                self._log(EV_PHASE_END_YELLOW, ring.phase)
                self._log(EV_PHASE_BEGIN_RED_CLEAR, ring.phase)
                ring.interval, ring.start = RED_CLEAR, t
                changed = True
            if ring.interval == RED_CLEAR and t - ring.start >= self.ph[ring.phase].red_clear:
                self._log(EV_PHASE_END_RED_CLEAR, ring.phase)
                nxt = self._next_phase(ring, t)
                if nxt:
                    self._start(ring, *nxt)
                else:
                    ring.phase, ring.interval, ring.start = None, REST, t
                changed = True

        # barrier crossing
        if all(r.interval == REST for r in self.rings.values()):
            self._cross_barrier()
            return True

        # red-rest re-entry within the current group
        for ring in self.rings.values():
            other = self._other(ring)
            if ring.interval == REST and not self._heading_to_barrier(other):
                nxt = self._next_phase(ring, t)
                if nxt:
                    self._start(ring, *nxt)
                    changed = True

        # coordinated phases
        coord = [r for r in self.rings.values() if r.interval == GREEN and self.ph[r.phase].coordinated]
        if self.group == MAIN and len(coord) == len(self.rings):
            if t >= self.cycle_start(self.k + 1):
                self.k += 1  # resting in coordinated green into the next cycle
            yp = self.yield_point()
            if t >= yp and self._last_yield_logged != self.k:
                self._last_yield_logged = self.k
                self._log(EV_COORD_YIELD_POINT, self.k % 256)
            for ring in coord:
                self._maybe_recycle_walk(ring, yp)
            held = any(self._active(self.cmd_hold, r.phase) for r in coord)
            if t >= yp and not held and all(self._green_done(r) for r in coord) and self._go_around_demand():
                for ring in coord:
                    self._terminate(ring, None)
                changed = True
        elif self.group == MAIN:
            for ring in coord:
                self._maybe_recycle_walk(ring, self.yield_point())

        # non-coordinated phases
        for ring in self.rings.values():
            if ring.interval != GREEN or self.ph[ring.phase].coordinated or t < ring.green_start:
                continue
            p = ring.phase
            fo = self.force_off(p)
            self._maybe_recycle_walk(ring, fo)
            if self._active(self.cmd_hold, p):
                if t >= fo and not ring.overrun_logged:
                    ring.overrun_logged = True
                    self._log(EV_HELD_PAST_FORCE_OFF, p)
                continue
            if not self._green_done(ring):
                if t >= fo and not ring.overrun_logged:
                    ring.overrun_logged = True
                    self.overruns += 1
                    self._log(EV_FORCE_OFF_OVERRUN, p)
                continue
            if p in self.cmd_force_off:
                self.cmd_force_off.discard(p)
                self._terminate(ring, EV_PHASE_FORCE_OFF)
                changed = True
            elif t >= fo:
                self._terminate(ring, EV_PHASE_FORCE_OFF)
                changed = True
            elif self.gap[p] == 0:
                if self.simultaneous_gap_out and self._is_barrier_phase(ring):
                    other = self._other(ring)
                    ready = self._heading_to_barrier(other) or (
                        other.interval == GREEN and not self.ph[other.phase].coordinated
                        and self._is_barrier_phase(other) and self._in_gap_out(other))
                    if not ready:
                        continue
                self._terminate(ring, EV_PHASE_GAP_OUT)
                changed = True
        return changed

    def _maybe_recycle_walk(self, ring: RingState, latest_end: int) -> None:
        if not self.ped_recycle and self.t != ring.start:
            return
        name = self.ped_of.get(ring.phase)
        if name is None:
            return
        ped = self.peds[name]
        d = self.ph[ring.phase]
        if ped.pending and ped.state == DONT_WALK and self.t >= ring.green_start \
                and self.t + d.walk + d.ped_clear <= latest_end:
            self._begin_walk(ring.phase)

    def _go_around_demand(self) -> bool:
        """True if leaving coordinated green now lets another phase be served."""
        t_start = self.t + max(self.ph[r.phase].clearance for r in self.rings.values() if r.phase is not None)
        for seq in self.seq[SIDE].values():
            for p in seq:
                if self._can_start(p, t_start, self.k)[0]:
                    return True
        if self.t < self.cycle_start(self.k + 1):
            for seq in self.seq[MAIN].values():
                lead = seq[0]
                if not self.ph[lead].coordinated and self._can_start(lead, t_start, self.k + 1)[0]:
                    return True
        return False

    def _cross_barrier(self) -> None:
        if self.group == MAIN:
            self.group = SIDE
        else:
            self.group = MAIN
            self.k += 1
        self._log(EV_BARRIER_CROSS, 1 if self.group == MAIN else 2)
        for ring in self.rings.values():
            ring.served = []
            nxt = self._next_phase(ring, self.t)
            if nxt:
                self._start(ring, *nxt)
            else:
                ring.phase, ring.interval, ring.start = None, REST, self.t


# ---------------------------------------------------------------------------
# Signal-state strings for SUMO
# ---------------------------------------------------------------------------

def signal_state(ctrl: ActuatedCoordinatedController, link_map: list[tuple[int, bool]]) -> str:
    chars = []
    disp = {p: ctrl.vehicle_display(p) for p in ctrl.ph}
    for phase, permissive in link_map:
        s = disp[phase]
        chars.append(("g" if permissive else "G") if s == "G" else ("y" if s == "Y" else "r"))
    return "".join(chars)


def build_controllers(scenario: str, start_tick: int, spec: dict | None = None, ctrl_cfg: dict | None = None,
                      plans: dict | None = None) -> dict[str, ActuatedCoordinatedController]:
    from cadence.controllers.phasing import load_controller_config, phase_defs
    from cadence.controllers.timing import load_plans
    from cadence.paths import load_spec
    from cadence.simulation.network import INTERSECTIONS

    spec = spec or load_spec()
    ctrl_cfg = ctrl_cfg or load_controller_config()
    plans = plans or load_plans(spec=spec, ctrl=ctrl_cfg)
    opts = ctrl_cfg["options"]
    out = {}
    for i in INTERSECTIONS:
        out[i] = ActuatedCoordinatedController(
            i, phase_defs(i, spec, ctrl_cfg), plans[scenario][i], start_tick,
            passage_s=ctrl_cfg["actuation"]["passage_time_s"],
            simultaneous_gap_out=opts["simultaneous_gap_out"], ped_recycle=opts["pedestrian_recycle"],
            family=spec["intersections"][i]["family"])
    return out
