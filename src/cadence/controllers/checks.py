"""Independent safety and timing checks for signal operation.

The checks work from what SUMO was actually told (the sequence of signal
state strings) and from the controller's event log. Conflicts are judged
against SUMO's own junction conflict matrix (the `foes` of each link in the
network file), not against the controller's phase table, so a mistake in the
phase table cannot hide itself.

Each check returns a list of human-readable violations; an empty list
means the check passed.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from cadence.controllers.actuated import (
    EV_COORD_YIELD_POINT, EV_FORCE_OFF_OVERRUN, EV_PED_BEGIN_CLEAR, EV_PED_BEGIN_DONT_WALK,
    EV_PED_BEGIN_WALK, EV_PED_CALL_REGISTERED, EV_PHASE_BEGIN_GREEN, EV_PHASE_BEGIN_LPI,
    EV_PHASE_BEGIN_YELLOW, EV_PHASE_CALL_DROPPED, EV_PHASE_CALL_REGISTERED, EV_PHASE_FORCE_OFF,
    ActuatedCoordinatedController,
)

NON_RED = set("Ggy")


@dataclass(frozen=True)
class Run:
    char: str
    start: int
    end: int  # exclusive

    @property
    def length(self) -> int:
        return self.end - self.start


def link_runs(changes: list[tuple[int, str]], end_tick: int) -> list[list[Run]]:
    """Per-link runs of identical display, from a list of (tick, state) changes."""
    if not changes:
        return []
    n = len(changes[0][1])
    runs: list[list[Run]] = [[] for _ in range(n)]
    cur = [None] * n
    for k, (t, state) in enumerate(changes):
        for i, ch in enumerate(state):
            if cur[i] is None:
                cur[i] = (ch, t)
            elif cur[i][0] != ch:
                runs[i].append(Run(cur[i][0], cur[i][1], t))
                cur[i] = (ch, t)
    for i in range(n):
        if cur[i] is not None:
            runs[i].append(Run(cur[i][0], cur[i][1], end_tick))
    return runs


def _compatible(ctrl: ActuatedCoordinatedController, p: int, q: int) -> bool:
    a, b = ctrl.ph[p], ctrl.ph[q]
    return p == q or (a.group == b.group and a.ring != b.ring)


def check_conflicts(changes, link_map, foes, ctrl) -> list[str]:
    """No two conflicting links are ever non-red together, except a
    permissive movement and its opposing movement in a concurrent phase."""
    out = []
    seen: dict[str, int] = {}
    for t, s in changes:
        seen.setdefault(s, t)
    for s, t in seen.items():
        for i, ch in enumerate(s):
            if ch not in NON_RED:
                continue
            for j in foes[i]:
                if j <= i or s[j] not in NON_RED:
                    continue
                (pi, perm_i), (pj, perm_j) = link_map[i], link_map[j]
                if (perm_i or perm_j) and _compatible(ctrl, pi, pj):
                    continue
                out.append(f"{ctrl.intersection} t={t}: links {i}({s[i]}) and {j}({s[j]}) conflict in state {s}")
    return out


def check_barrier(changes, link_map, ctrl) -> list[str]:
    """Main-group and side-group movements are never non-red together."""
    out = []
    seen = {}
    for t, s in changes:
        seen.setdefault(s, t)
    for s, t in seen.items():
        groups = {ctrl.ph[link_map[i][0]].group for i, ch in enumerate(s) if ch in NON_RED}
        if len(groups) > 1:
            out.append(f"{ctrl.intersection} t={t}: both barrier groups active in {s}")
    return out


def check_link_sequences(changes, end_tick, link_map, ctrl) -> list[str]:
    """green -> yellow -> red on every link; exact yellow; minimum green;
    and red clearance before any conflicting green (checked in intergreen)."""
    out = []
    for i, runs in enumerate(link_runs(changes, end_tick)):
        d = ctrl.ph[link_map[i][0]]
        for a, b in zip(runs, runs[1:]):
            ok = (a.char in "Gg" and b.char == "y") or (a.char == "y" and b.char == "r") or \
                 (a.char == "r" and b.char in "Gg")
            if not ok:
                out.append(f"{ctrl.intersection} link {i}: illegal change {a.char}->{b.char} at t={b.start}")
        for r in runs[1:-1]:  # first and last runs may be cut by the run window
            if r.char == "y" and r.length != d.yellow:
                out.append(f"{ctrl.intersection} link {i}: yellow {r.length} ticks at t={r.start}, expected {d.yellow}")
            if r.char in "Gg" and r.length < d.min_green:
                out.append(f"{ctrl.intersection} link {i}: green {r.length} ticks at t={r.start} below minimum {d.min_green}")
    return out


def check_intergreen(changes, end_tick, link_map, foes, ctrl) -> list[str]:
    """After a link's yellow ends, no conflicting link turns green before the
    link's phase red clearance has elapsed."""
    out = []
    runs = link_runs(changes, end_tick)
    yellow_ends = [[r.end for r in rs if r.char == "y"] for rs in runs]
    green_starts = [[r.start for r in rs[1:] if r.char in "Gg"] for rs in runs]
    for i in range(len(runs)):
        ar = ctrl.ph[link_map[i][0]].red_clear
        for j in foes[i]:
            (pi, perm_i), (pj, perm_j) = link_map[i], link_map[j]
            if (perm_i or perm_j) and _compatible(ctrl, pi, pj):
                continue
            for b in green_starts[j]:
                prior = [a for a in yellow_ends[i] if a <= b]
                if prior and b - prior[-1] < ar:
                    out.append(f"{ctrl.intersection}: link {j} green at t={b} only {b - prior[-1]} ticks "
                               f"after conflicting link {i} yellow ended (red clearance {ar})")
    return out


def _by_param(events, code):
    d = defaultdict(list)
    for t, c, p in events:
        if c == code:
            d[p].append(t)
    return d


def check_pedestrians(events, ctrl) -> list[str]:
    """Walk and pedestrian clearance have exact durations and end before the
    vehicle phase's yellow; an LPI precedes vehicle green by exactly its length."""
    out = []
    walks = _by_param(events, EV_PED_BEGIN_WALK)
    clears = _by_param(events, EV_PED_BEGIN_CLEAR)
    dws = _by_param(events, EV_PED_BEGIN_DONT_WALK)
    yellows = _by_param(events, EV_PHASE_BEGIN_YELLOW)
    lpis = _by_param(events, EV_PHASE_BEGIN_LPI)
    greens = _by_param(events, EV_PHASE_BEGIN_GREEN)
    for name, ped in ctrl.peds.items():
        n = int(name[1:])
        d = ctrl.ph[ped.phase]
        for w in walks.get(n, []):
            c = next((x for x in clears.get(n, []) if x >= w), None)
            if c is None:
                continue  # run ended during walk
            if c - w != d.walk:
                out.append(f"{ctrl.intersection} {name}: walk {c - w} ticks at t={w}, expected {d.walk}")
            dw = next((x for x in dws.get(n, []) if x >= c), None)
            if dw is None:
                continue
            if dw - c != d.ped_clear:
                out.append(f"{ctrl.intersection} {name}: clearance {dw - c} ticks at t={c}, expected {d.ped_clear}")
            y = next((x for x in yellows.get(ped.phase, []) if x >= w), None)
            if y is not None and y < dw:
                out.append(f"{ctrl.intersection} {name}: phase {ped.phase} yellow at t={y} before don't walk at t={dw}")
    for p, starts in lpis.items():
        lpi = ctrl.ph[p].lpi
        for s in starts:
            g = next((x for x in greens.get(p, []) if x >= s), None)
            if g is not None and g - s != lpi:
                out.append(f"{ctrl.intersection} phase {p}: LPI {g - s} ticks at t={s}, expected {lpi}")
    return out


def check_coordination(events, ctrl) -> list[str]:
    """Coordinated phases yield only at or after the yield point; force-offs
    happen exactly at the programmed point; no force-off overruns."""
    out = []
    C = ctrl.cycle
    for t, c, p in events:
        tau = (t - ctrl.offset) % C
        if c == EV_PHASE_BEGIN_YELLOW and ctrl.ph[p].coordinated and not (ctrl._yield_tau <= tau < C):
            out.append(f"{ctrl.intersection}: coordinated phase {p} yellow at cycle time {tau / 10:.1f} s, "
                       f"before yield point {ctrl._yield_tau / 10:.1f} s")
        if c == EV_PHASE_FORCE_OFF and tau != ctrl._fo_tau[p] % C:
            out.append(f"{ctrl.intersection}: phase {p} forced off at cycle time {tau / 10:.1f} s, "
                       f"programmed {ctrl._fo_tau[p] / 10:.1f} s")
        if c == EV_FORCE_OFF_OVERRUN:
            out.append(f"{ctrl.intersection}: phase {p} held past force-off at t={t}")
    return out


def service_waits(events, ctrl, end_tick) -> tuple[dict[int, int], dict[int, int], list[str]]:
    """Longest wait (ticks) from a vehicle call to green, per phase, and from a
    pedestrian call to walk, per pedestrian phase. Calls still waiting at the
    end of the run are reported as violations only if they waited past the
    bound used by the caller."""
    veh: dict[int, int] = defaultdict(int)
    ped: dict[int, int] = defaultdict(int)
    open_calls: dict[int, int] = {}
    open_peds: dict[int, int] = {}
    for t, c, p in events:
        if c == EV_PHASE_CALL_REGISTERED:
            open_calls.setdefault(p, t)
        elif c == EV_PHASE_CALL_DROPPED:
            open_calls.pop(p, None)
        elif c in (EV_PHASE_BEGIN_GREEN, EV_PHASE_BEGIN_LPI) and p in open_calls:
            veh[p] = max(veh[p], t - open_calls.pop(p))
        elif c == EV_PED_CALL_REGISTERED:
            open_peds.setdefault(p, t)
        elif c == EV_PED_BEGIN_WALK and p in open_peds:
            ped[p] = max(ped[p], t - open_peds.pop(p))
    pending = [f"{ctrl.intersection}: phase {p} call waiting since t={t}" for p, t in open_calls.items()] + \
              [f"{ctrl.intersection}: P{p} call waiting since t={t}" for p, t in open_peds.items()]
    for p, t in open_calls.items():
        veh[p] = max(veh[p], end_tick - t)
    for p, t in open_peds.items():
        ped[p] = max(ped[p], end_tick - t)
    return dict(veh), dict(ped), pending


def check_all(changes, end_tick, link_map, foes, ctrl, max_wait_cycles: float = 2.0) -> list[str]:
    events = ctrl.events
    out = []
    out += check_conflicts(changes, link_map, foes, ctrl)
    out += check_barrier(changes, link_map, ctrl)
    out += check_link_sequences(changes, end_tick, link_map, ctrl)
    out += check_intergreen(changes, end_tick, link_map, foes, ctrl)
    out += check_pedestrians(events, ctrl)
    out += check_coordination(events, ctrl)
    veh, ped, _ = service_waits(events, ctrl, end_tick)
    bound = int(max_wait_cycles * ctrl.cycle)
    out += [f"{ctrl.intersection}: phase {p} call waited {w / 10:.1f} s" for p, w in veh.items() if w > bound]
    out += [f"{ctrl.intersection}: P{p} call waited {w / 10:.1f} s" for p, w in ped.items() if w > bound]
    return out
