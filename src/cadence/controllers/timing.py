"""Coordination timing plans (cycle, splits, offsets) for Corridor R-1.

Plans are derived deterministically from the spec and the controller
configuration, written to configs/timing_plans_r1.json for review, and
checked by a test that regenerates them. They are the Milestone 1 baseline
plan, not an optimized one.

Split method (M1)
-----------------
1. Flow ratio y_p = phase volume / (lanes feeding the phase x saturation flow),
   from expected movement flows (S2 uses the 17:00-17:45 core).
2. Group times: each barrier group gets its clearance time plus effective
   green in proportion to its critical flow ratio (the larger ring sum),
   but never less than the larger ring sum of minimum splits.
3. Within each ring, green is shared in proportion to y_p with minimum
   splits enforced (water-filling), then rounded to whole seconds so each
   ring sums exactly to its group time.

A phase's minimum split is its clearance plus the longer of (LPI + minimum
green) and (walk + pedestrian clearance) when it serves a pedestrian phase,
so a pedestrian interval always fits inside the programmed split.

Offsets (M1): cycle time 0 is the programmed start of the main group.
Offsets give eastbound progression at the posted speed, aligned on the
programmed start of phase 2.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from cadence.controllers.phasing import (
    MAIN, SIDE, PhaseDef, load_controller_config, phase_defs, ring_sequences,
)
from cadence.paths import REPO_ROOT, load_spec
from cadence.simulation.demand import SCENARIOS, expected_movement_flows
from cadence.simulation.network import INTERSECTIONS, approach_edge, movement_lanes

PLANS_PATH = REPO_ROOT / "configs" / "timing_plans_r1.json"


@dataclass
class TimingPlan:
    intersection: str
    cycle_s: int
    offset_s: int
    splits_s: dict[int, int]
    sequences: dict[str, dict[int, list[int]]] = field(default_factory=dict)
    flow_ratios: dict[int, float] = field(default_factory=dict)

    def group_time_s(self, group: str) -> int:
        times = {sum(self.splits_s[p] for p in seq) for seq in self.sequences[group].values() if seq}
        if len(times) != 1:
            raise ValueError(f"{self.intersection}: rings disagree on {group} group time {times}")
        return times.pop()

    def validate(self, phases: dict[int, PhaseDef]) -> None:
        if self.group_time_s(MAIN) + self.group_time_s(SIDE) != self.cycle_s:
            raise ValueError(f"{self.intersection}: group times do not add to the cycle")
        for p, d in phases.items():
            if self.splits_s[p] < min_split_s(d) - 1e-9:
                raise ValueError(f"{self.intersection}: split of phase {p} below its minimum")

    def to_json(self) -> dict:
        return {
            "cycle_s": self.cycle_s,
            "offset_s": self.offset_s,
            "splits_s": {str(p): s for p, s in sorted(self.splits_s.items())},
            "flow_ratios": {str(p): round(y, 4) for p, y in sorted(self.flow_ratios.items())},
        }

    @classmethod
    def from_json(cls, intersection: str, data: dict, sequences: dict) -> "TimingPlan":
        return cls(intersection, int(data["cycle_s"]), int(data["offset_s"]),
                   {int(p): int(s) for p, s in data["splits_s"].items()}, sequences,
                   {int(p): float(y) for p, y in data.get("flow_ratios", {}).items()})


def min_split_s(d: PhaseDef) -> float:
    service = d.min_green_s
    if d.ped:
        service = max(d.lpi_s + d.min_green_s, d.walk_s + d.ped_clear_s)
    return d.clearance_s + service


def design_flows(scenario: str, spec: dict) -> dict[tuple[str, str, str], float]:
    seg = None
    if scenario == "S2":
        seg = {"EB": spec["scenarios"]["S2"]["eb_entry_core"]["flow"]}
    return expected_movement_flows(scenario, seg, spec)


def flow_ratios(intersection: str, scenario: str, spec: dict, phases: dict[int, PhaseDef]) -> dict[int, float]:
    flows = design_flows(scenario, spec)
    sat = spec["global"]["saturation_flow_vphpl"]
    out = {}
    for p, d in phases.items():
        moves = d.movements + d.permissive
        vol = sum(flows.get((intersection, a, t), 0.0) for a, t in moves)
        lanes = {(approach_edge(intersection, a, spec), fl)
                 for a, t in moves for fl, _ in movement_lanes(intersection, a, t, spec)}
        out[p] = vol / (len(lanes) * sat) if lanes else 0.0
    return out


def _share(total: float, phases: list[int], y: dict[int, float], defs: dict[int, PhaseDef]) -> dict[int, float]:
    """Split `total` seconds among phases: clearance + green in proportion to
    y, never below the minimum split (water-filling)."""
    fixed: dict[int, float] = {}
    free = list(phases)
    while True:
        avail = total - sum(fixed.values()) - sum(defs[p].clearance_s for p in free)
        ysum = sum(y[p] for p in free)
        alloc = {p: defs[p].clearance_s + (avail * (y[p] / ysum if ysum > 0 else 1 / len(free))) for p in free}
        low = [p for p in free if alloc[p] < min_split_s(defs[p])]
        if not low:
            return {**fixed, **alloc}
        for p in low:
            fixed[p] = min_split_s(defs[p])
            free.remove(p)
        if not free:
            return fixed


def _round_ring(alloc: dict[int, float], order: list[int], total: int, defs: dict[int, PhaseDef]) -> dict[int, int]:
    out = {p: max(math.ceil(min_split_s(defs[p])), int(round(alloc[p]))) for p in order}
    # the last phase of the ring (coordinated phase, or side through) absorbs rounding
    out[order[-1]] += total - sum(out.values())
    if out[order[-1]] < min_split_s(defs[order[-1]]):
        raise ValueError("rounding left a phase below its minimum split")
    return out


def compute_splits(intersection: str, scenario: str, spec: dict, ctrl: dict) -> tuple[dict[int, int], dict[int, float]]:
    defs = phase_defs(intersection, spec, ctrl)
    seqs = ring_sequences(intersection, spec, ctrl)
    y = flow_ratios(intersection, scenario, spec, defs)
    cycle = spec["timing_constants"]["cycle_s"][scenario]

    def ring_sum(group, fn):
        return max(sum(fn(p) for p in seq) for seq in seqs[group].values())

    crit = {g: ring_sum(g, lambda p: y[p]) for g in (MAIN, SIDE)}
    lost = {g: ring_sum(g, lambda p: defs[p].clearance_s) for g in (MAIN, SIDE)}
    tmin = {g: ring_sum(g, lambda p: min_split_s(defs[p])) for g in (MAIN, SIDE)}
    green = cycle - lost[MAIN] - lost[SIDE]
    t_main = lost[MAIN] + green * crit[MAIN] / (crit[MAIN] + crit[SIDE])
    if t_main < tmin[MAIN]:
        t_main = tmin[MAIN]
    if cycle - t_main < tmin[SIDE]:
        t_main = cycle - tmin[SIDE]
    if t_main < tmin[MAIN]:
        raise ValueError(f"{intersection} {scenario}: cycle {cycle} s cannot hold the minimum splits")
    t_main_i = int(round(t_main))
    if cycle - t_main_i < math.ceil(tmin[SIDE]):
        t_main_i = cycle - math.ceil(tmin[SIDE])
    group_time = {MAIN: t_main_i, SIDE: cycle - t_main_i}
    splits: dict[int, int] = {}
    for g in (MAIN, SIDE):
        for seq in seqs[g].values():
            if seq:
                splits.update(_round_ring(_share(group_time[g], seq, y, defs), seq, group_time[g], defs))
    return splits, y


def compute_offsets(scenario: str, splits: dict[str, dict[int, int]], spec: dict) -> dict[str, int]:
    cycle = spec["timing_constants"]["cycle_s"][scenario]
    v = spec["global"]["posted_speed_mps"]
    x1 = spec["intersections"]["I1"]["x"]
    phase2_start_i1 = splits["I1"][1]
    out = {}
    for i in INTERSECTIONS:
        travel = (spec["intersections"][i]["x"] - x1) / v
        out[i] = int(round(phase2_start_i1 + travel - splits[i][1])) % cycle
    return out


def generate_plans(spec: dict | None = None, ctrl: dict | None = None) -> dict[str, dict[str, TimingPlan]]:
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    plans: dict[str, dict[str, TimingPlan]] = {}
    for sc in SCENARIOS:
        splits, ys = {}, {}
        for i in INTERSECTIONS:
            splits[i], ys[i] = compute_splits(i, sc, spec, ctrl)
        offsets = compute_offsets(sc, splits, spec)
        plans[sc] = {}
        for i in INTERSECTIONS:
            plan = TimingPlan(i, spec["timing_constants"]["cycle_s"][sc], offsets[i], splits[i],
                              ring_sequences(i, spec, ctrl), ys[i])
            plan.validate(phase_defs(i, spec, ctrl))
            plans[sc][i] = plan
    return plans


def plans_to_json(plans: dict[str, dict[str, TimingPlan]], spec: dict, ctrl: dict) -> dict:
    return {
        "_about": "Generated by `python -m cadence.controllers.timing` from networks/corridor_r1/spec.json "
                  "and configs/controller_r1.json. Do not edit by hand. Milestone 1 baseline plans (M1).",
        "spec_version": spec["version"],
        "controller_version": ctrl["version"],
        "plans": {sc: {i: p.to_json() for i, p in by_i.items()} for sc, by_i in plans.items()},
    }


def write_plans(path: Path = PLANS_PATH, spec: dict | None = None, ctrl: dict | None = None) -> Path:
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    data = plans_to_json(generate_plans(spec, ctrl), spec, ctrl)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return path


def load_plans(path: Path = PLANS_PATH, spec: dict | None = None, ctrl: dict | None = None) -> dict[str, dict[str, TimingPlan]]:
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {sc: {i: TimingPlan.from_json(i, d, ring_sequences(i, spec, ctrl)) for i, d in by_i.items()}
            for sc, by_i in data["plans"].items()}


if __name__ == "__main__":
    print(write_plans())
