"""Demand generation for Corridor R-1 scenarios S0-S3.

Arrivals are time-varying Poisson processes per entry stream (DZ 4.1). Each
vehicle's route is sampled turn by turn from the scenario turning table, so
cross-street vehicles that turn onto the arterial continue through later
intersections like any arterial vehicle.

Randomness is fully determined by (master seed, scenario, replication,
stream): each stream gets its own numpy Generator built from a SeedSequence
with a fixed spawn key, so adding or removing one stream never changes
another stream's draws.

Milestone 1 models pedestrians as push-button calls only (no walking agents).
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from cadence.paths import load_spec
from cadence.simulation.network import (
    DEPARTURE_DIRECTION,
    INTERSECTIONS,
    approach_edges_upstream,
    departure_edge,
)

SCENARIOS = ("S0", "S1", "S2", "S3")
PED_PHASES = ("P2", "P4", "P6", "P8")
# Crosswalk served by each pedestrian phase (M1 convention, see docs/methodology).
PED_CROSSWALK = {"P2": "north leg (side crossing)", "P6": "south leg (side crossing)",
                 "P4": "west leg (mainline crossing)", "P8": "east leg (mainline crossing)"}


def hhmm(s: str) -> int:
    h, m = s.split(":")
    return int(h) * 3600 + int(m) * 60


@dataclass(frozen=True)
class Window:
    start: int
    end: int


def scenario_window(scenario: str, spec: dict | None = None) -> Window:
    spec = spec or load_spec()
    sc = spec["scenarios"][scenario]
    return Window(hhmm(sc["start"]), hhmm(sc["end"]))


# ---------------------------------------------------------------------------
# Turning
# ---------------------------------------------------------------------------

def turn_fractions(intersection: str, approach: str, scenario: str, spec: dict | None = None) -> dict[str, float]:
    spec = spec or load_spec()
    turning = spec["turning"]
    if approach in ("NB", "SB"):
        return dict(turning["cross"][intersection])
    if approach == "EB" and intersection == "I4":
        return dict(turning["eb_at_I4"])
    if approach == "WB" and intersection == "I1":
        return dict(turning["wb_at_I1"])
    t = spec["scenarios"][scenario]["mainline_through_fraction"]
    side = (1.0 - t) / 2.0
    return {"L": side, "T": t, "R": side}


def next_intersection(intersection: str, direction: str) -> str | None:
    k = INTERSECTIONS.index(intersection)
    if direction == "EB":
        return INTERSECTIONS[k + 1] if k + 1 < len(INTERSECTIONS) else None
    if direction == "WB":
        return INTERSECTIONS[k - 1] if k > 0 else None
    raise ValueError(direction)


def exit_edge(direction: str) -> str:
    return {"EB": "EB4", "WB": "WB0"}[direction]


@dataclass(frozen=True)
class Stream:
    name: str
    intersection: str  # first intersection reached
    approach: str
    first_split: dict | None = None  # overrides the first turning decision (S3 pulse)


def entry_streams() -> list[Stream]:
    streams = [Stream("EB", "I1", "EB"), Stream("WB", "I4", "WB")]
    for i in INTERSECTIONS:
        k = INTERSECTIONS.index(i) + 1
        streams.append(Stream(f"N{k}", i, "SB"))
        streams.append(Stream(f"S{k}", i, "NB"))
    return streams


def sample_route(stream: Stream, scenario: str, rng: np.random.Generator, spec: dict) -> tuple[list[str], list[tuple[str, str, str]]]:
    """Return (edge list, [(intersection, approach, turn), ...])."""
    cur, direction = stream.intersection, stream.approach
    edges = list(approach_edges_upstream(cur, direction, spec))
    decisions = []
    first = True
    while True:
        fr = stream.first_split if (first and stream.first_split) else turn_fractions(cur, direction, scenario, spec)
        first = False
        turns = [t for t in ("L", "T", "R") if fr.get(t, 0.0) > 0]
        probs = np.array([fr[t] for t in turns], dtype=float)
        turn = turns[int(rng.choice(len(turns), p=probs / probs.sum()))]
        decisions.append((cur, direction, turn))
        dep = DEPARTURE_DIRECTION[(direction, turn)]
        if dep in ("NB", "SB"):
            edges.append(departure_edge(cur, dep))
            return edges, decisions
        nxt = next_intersection(cur, dep)
        if nxt is None:
            edges.append(exit_edge(dep))
            return edges, decisions
        edges += approach_edges_upstream(nxt, dep, spec)
        cur, direction = nxt, dep


# ---------------------------------------------------------------------------
# Arrival rates
# ---------------------------------------------------------------------------

def stream_rate_segments(stream: Stream, scenario: str, spec: dict | None = None) -> list[tuple[int, int, float]]:
    """Piecewise-constant rate segments (start, end, veh/h) over the scenario window."""
    spec = spec or load_spec()
    sc = spec["scenarios"][scenario]
    w = scenario_window(scenario, spec)
    if stream.name == "EB":
        core = sc.get("eb_entry_core")
        if core:
            cs, ce = hhmm(core["start"]), hhmm(core["end"])
            return [(w.start, cs, sc["eb_entry"]), (cs, ce, core["flow"]), (ce, w.end, sc["eb_entry"])]
        return [(w.start, w.end, sc["eb_entry"])]
    if stream.name == "WB":
        return [(w.start, w.end, sc["wb_entry"])]
    if stream.name.startswith("PULSE"):
        p = sc["pulse"]
        return [(hhmm(p["start"]), hhmm(p["end"]), p["added_flow"])]
    return [(w.start, w.end, sc["cross_per_approach"][stream.intersection])]


def scenario_streams(scenario: str, spec: dict | None = None) -> list[Stream]:
    spec = spec or load_spec()
    streams = entry_streams()
    pulse = spec["scenarios"][scenario].get("pulse")
    if pulse:
        approach = pulse["approach"]
        streams.append(Stream(f"PULSE_{pulse['intersection']}{approach}", pulse["intersection"], approach,
                              first_split=dict(pulse["split"])))
    return streams


def _poisson_times(segments: list[tuple[int, int, float]], rng: np.random.Generator) -> list[float]:
    times: list[float] = []
    for start, end, rate_vph in segments:
        if rate_vph <= 0:
            continue
        lam = rate_vph / 3600.0
        t = float(start)
        while True:
            t += float(rng.exponential(1.0 / lam))
            if t >= end:
                break
            times.append(t)
    return times


def stream_rng(scenario: str, replication: int, stream_index: int, spec: dict, kind: int = 0) -> np.random.Generator:
    """Independent generator per (scenario, replication, stream, kind)."""
    ss = np.random.SeedSequence(entropy=spec["scenarios"]["master_seed"],
                                spawn_key=(SCENARIOS.index(scenario), replication, kind, stream_index))
    return np.random.Generator(np.random.PCG64(ss))


@dataclass
class Vehicle:
    vid: str
    depart: float
    edges: list[str]
    decisions: list[tuple[str, str, str]]


def generate_vehicles(scenario: str, replication: int, spec: dict | None = None) -> list[Vehicle]:
    spec = spec or load_spec()
    vehicles: list[Vehicle] = []
    for idx, stream in enumerate(scenario_streams(scenario, spec)):
        rng = stream_rng(scenario, replication, idx, spec, kind=0)
        times = _poisson_times(stream_rate_segments(stream, scenario, spec), rng)
        for n, t in enumerate(times):
            edges, decisions = sample_route(stream, scenario, rng, spec)
            vehicles.append(Vehicle(f"{stream.name}.{n:05d}", round(t, 2), edges, decisions))
    vehicles.sort(key=lambda v: (v.depart, v.vid))
    return vehicles


def generate_ped_calls(scenario: str, replication: int, spec: dict | None = None) -> list[tuple[float, str, str]]:
    """Pedestrian push-button arrivals: (time, intersection, ped phase)."""
    spec = spec or load_spec()
    rates = spec["pedestrians"]["call_rate_per_h"][scenario]
    w = scenario_window(scenario, spec)
    calls = []
    idx = 0
    for i in INTERSECTIONS:
        for p in PED_PHASES:
            if p in ("P4", "P8"):
                rate = rates["I4_mainline_crossing"] if i == "I4" else rates["mainline_crossing"]
            else:
                rate = rates["side_crossing"]
            rng = stream_rng(scenario, replication, idx, spec, kind=1)
            idx += 1
            for t in _poisson_times([(w.start, w.end, rate)], rng):
                calls.append((round(t, 2), i, p))
    calls.sort()
    return calls


def write_routes(vehicles: list[Vehicle], path: Path, spec: dict | None = None) -> None:
    spec = spec or load_spec()
    v = spec["global"]["vehicle"]
    vmax = spec["global"]["posted_speed_mps"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("<routes>\n")
        fh.write(
            f'    <vType id="car" vClass="passenger" length="{v["length_m"]}" minGap="{v["min_gap_m"]}" '
            f'accel="{v["accel"]}" decel="{v["decel"]}" sigma="{v["sigma"]}" tau="{v["tau"]}" maxSpeed="{vmax}"/>\n')
        for veh in vehicles:
            fh.write(f'    <vehicle id="{veh.vid}" type="car" depart="{veh.depart:.2f}" '
                     f'departLane="best" departSpeed="max">\n')
            fh.write(f'        <route edges="{" ".join(veh.edges)}"/>\n')
            fh.write("    </vehicle>\n")
        fh.write("</routes>\n")


def write_ped_calls(calls: list[tuple[float, str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "intersection", "ped_phase"])
        for row in calls:
            w.writerow(row)


def read_ped_calls(path: Path) -> list[tuple[float, str, str]]:
    with open(path, newline="", encoding="utf-8") as fh:
        return [(float(r["time_s"]), r["intersection"], r["ped_phase"]) for r in csv.DictReader(fh)]


def generate(scenario: str, replication: int, outdir: Path, spec: dict | None = None) -> dict[str, Path]:
    spec = spec or load_spec()
    outdir = Path(outdir)
    rou = outdir / f"{scenario}_r{replication:02d}.rou.xml"
    ped = outdir / f"{scenario}_r{replication:02d}.ped.csv"
    write_routes(generate_vehicles(scenario, replication, spec), rou, spec)
    write_ped_calls(generate_ped_calls(scenario, replication, spec), ped)
    return {"routes": rou, "ped_calls": ped}


# ---------------------------------------------------------------------------
# Analytic expected movement flows (deterministic propagation of the tables)
# ---------------------------------------------------------------------------

def expected_movement_flows(scenario: str, segment_rates: dict[str, float] | None = None,
                            spec: dict | None = None) -> dict[tuple[str, str, str], float]:
    """Expected flow (veh/h) of each (intersection, approach, turn) given constant
    entry rates. ``segment_rates`` overrides entry rates by stream name (for
    example {"EB": 1700} for the S2 core). The S3 pulse is excluded."""
    spec = spec or load_spec()
    sc = spec["scenarios"][scenario]
    rates = {"EB": sc["eb_entry"], "WB": sc["wb_entry"]}
    for i in INTERSECTIONS:
        k = INTERSECTIONS.index(i) + 1
        rates[f"N{k}"] = sc["cross_per_approach"][i]
        rates[f"S{k}"] = sc["cross_per_approach"][i]
    if segment_rates:
        rates.update(segment_rates)
    flows: dict[tuple[str, str, str], float] = {}
    # arrivals on each (intersection, approach) accumulate as flows propagate
    arrive: dict[tuple[str, str], float] = {}
    for s in entry_streams():
        arrive[(s.intersection, s.approach)] = arrive.get((s.intersection, s.approach), 0.0) + rates[s.name]
    order_eb = list(INTERSECTIONS)
    order_wb = list(reversed(INTERSECTIONS))
    # cross approaches feed the mainline, so process each intersection's cross
    # approaches before the mainline direction that leaves it
    pending = True
    processed: set[tuple[str, str]] = set()
    while pending:
        pending = False
        for i in order_eb + order_wb:
            for a in ("NB", "SB", "EB", "WB"):
                if (i, a) in processed or (i, a) not in arrive:
                    continue
                # mainline approach is ready only when all upstream feeders are processed
                if a == "EB":
                    prev = [x for x in order_eb[:order_eb.index(i)]]
                    if any((p, q) not in processed for p in prev for q in ("NB", "SB", "EB") if (p, q) in arrive):
                        continue
                if a == "WB":
                    prev = [x for x in order_wb[:order_wb.index(i)]]
                    if any((p, q) not in processed for p in prev for q in ("NB", "SB", "WB") if (p, q) in arrive):
                        continue
                vol = arrive[(i, a)]
                for t, f in turn_fractions(i, a, scenario, spec).items():
                    if f <= 0:
                        continue
                    flows[(i, a, t)] = flows.get((i, a, t), 0.0) + vol * f
                    dep = DEPARTURE_DIRECTION[(a, t)]
                    if dep in ("EB", "WB"):
                        nxt = next_intersection(i, dep)
                        if nxt is not None:
                            arrive[(nxt, dep)] = arrive.get((nxt, dep), 0.0) + vol * f
                processed.add((i, a))
                pending = True
    return flows
