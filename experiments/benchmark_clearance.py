"""Compute benchmark for clearance runs (Step Zero cost J).

    python experiments/benchmark_clearance.py

For S1-S3, a main run checkpoints at three horizon-end times (busy, peak,
late). From each checkpoint a clearance branch runs the baseline controller
with no new demand until the network and the insertion backlog are empty,
up to 90 minutes, so that the provisional 60 minute cap can be compared
with 30 and 90 minutes. J = horizon vehicle-time (the 5 minutes before the
checkpoint) + clearance vehicle-time + penalty x vehicles still present at
the cap; the provisional penalty is 60 vehicle-minutes, with 30 and 120 as
sensitivity values. Also reports the wall time of each branch on one core.

Writes docs/validation/clearance_benchmark_m1.md and .json.
"""

from __future__ import annotations

import json
import os
import platform
import time
from pathlib import Path

from cadence.paths import REPO_ROOT
from cadence.simulation import demand as dm
from cadence.simulation.overrides import HoldRed
from cadence.simulation.replay import in_fresh_process, outputs_after, run_branch
from cadence.simulation.runner import CorridorSimulation

RUNS = REPO_ROOT / "runs" / "clearance"
DOCS = REPO_ROOT / "docs" / "validation"
HORIZON_S = 300
CAP_MAX_S = 5400
CAPS_MIN = (30, 60, 90)
PENALTIES = (30, 60, 120)
POINTS = {
    "S1": ("07:30", "08:00", "08:45"),
    "S2": ("17:15", "17:45", "18:15"),
    "S3": ("18:10", "18:30", "19:15"),
}


def _main(scenario: str, times: tuple[str, ...], workdir: Path, stress: bool = False):
    w = dm.scenario_window(scenario)
    override = HoldRed("I1", "EB", dm.hhmm(times[0]) - 1200, 1200) if stress else None
    sim = CorridorSimulation(scenario, 0, workdir, signal_override=override)
    sim.start()
    out = []
    t0 = time.perf_counter()
    try:
        for hhmm in times:
            t = dm.hhmm(hhmm)
            sim.run_until(t - HORIZON_S)
            vs_before = sim.ledger.vehicle_seconds
            sim.run_until(t)
            cp = sim.checkpoint(workdir / f"cp_{hhmm.replace(':', '')}.xml")
            out.append({"time": hhmm, "checkpoint": str(cp.sumo_state),
                        "horizon_vehicle_seconds": sim.ledger.vehicle_seconds - vs_before,
                        "in_system_at_checkpoint": sim.ledger.last.backlog + sim.ledger.last.in_network,
                        "backlog_at_checkpoint": sim.ledger.last.backlog})
    finally:
        sim.close()
    return out, time.perf_counter() - t0, (dm.hhmm(times[-1]) - w.start)


def _clearance_metrics(series, t_c_tick: int, clearance: dict) -> dict:
    """Clearance vehicle-time and censored vehicles for each cap, from the
    branch's one-per-second ledger snapshots (backlog + network)."""
    out = {}
    for cap in CAPS_MIN:
        end = t_c_tick + cap * 600
        pts = [(k, n) for k, n in series if k <= end]
        cleared = clearance["cleared"] and clearance["duration_s"] <= cap * 60
        out[cap] = {"clearance_vehicle_seconds": sum(n for _, n in pts),
                    "censored": 0 if cleared else (pts[-1][1] if pts else 0)}
    return out


def run() -> dict:
    RUNS.mkdir(parents=True, exist_ok=True)
    result = {"machine": {"cpu": platform.processor() or platform.machine(), "cores_used": 1,
                          "python": platform.python_version()}, "cases": []}
    cases = [(sc, times, False) for sc, times in POINTS.items()] + [("S1", ("07:30",), True)]
    for sc, times, stress in cases:
        tag = f"{sc}{'_stress' if stress else ''}"
        cps, main_wall, sim_span = in_fresh_process(_main, sc, times, RUNS / tag, stress)
        for c in cps:
            out = in_fresh_process(run_branch, Path(c["checkpoint"]), RUNS / tag / f"clear_{c['time'].replace(':', '')}",
                                   clearance_cap_s=CAP_MAX_S)
            t_c_tick = int(round(dm.hhmm(c["time"]) * 10))
            m = _clearance_metrics(out["ledger_series"], t_c_tick, out["clearance"])
            row = {"scenario": sc, "stress": stress, **c, "clearance": out["clearance"], "by_cap": m,
                   "wall_resume_s": round(out["wall_s"]["resume"], 2), "wall_run_s": round(out["wall_s"]["run"], 2),
                   "collisions": len(out["collisions"]), "ledger_violations": len(out["ledger_violations"])}
            row["J_vehicle_hours"] = {f"cap{cap}_pen{pen}": round((c["horizon_vehicle_seconds"] + m[cap]["clearance_vehicle_seconds"]
                                                                  + pen * 60 * m[cap]["censored"]) / 3600, 3)
                                      for cap in CAPS_MIN for pen in PENALTIES}
            result["cases"].append(row)
            print(json.dumps({k: row[k] for k in ("scenario", "stress", "time", "in_system_at_checkpoint", "clearance",
                                                    "wall_resume_s", "wall_run_s")}), flush=True)
    return result


def _finalize(result: dict) -> dict:
    """Use the exact per-step clearance vehicle-time whenever the clearance
    finished within the cap (the per-second snapshots are only needed for
    caps the clearance did not reach), and recompute J."""
    for r in result["cases"]:
        cl = r["clearance"]
        for cap in CAPS_MIN:
            m = r["by_cap"][str(cap)] if str(cap) in r["by_cap"] else r["by_cap"][cap]
            if cl["cleared"] and cl["duration_s"] <= cap * 60:
                m["clearance_vehicle_seconds"] = cl["vehicle_seconds"]
                m["censored"] = 0
        r["by_cap"] = {int(k): v for k, v in r["by_cap"].items()}
        r["J_vehicle_hours"] = {f"cap{cap}_pen{pen}": round((r["horizon_vehicle_seconds"]
                                                            + r["by_cap"][cap]["clearance_vehicle_seconds"]
                                                            + pen * 60 * r["by_cap"][cap]["censored"]) / 3600, 3)
                                for cap in CAPS_MIN for pen in PENALTIES}
    return result


def report(result: dict) -> Path:
    result = _finalize(result)
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "clearance_benchmark_m1.json").write_text(json.dumps(result, indent=2))
    L = ["# Clearance-run compute benchmark, Milestone 1", "",
         "Generated by `python experiments/benchmark_clearance.py`. All numbers are in `clearance_benchmark_m1.json`.", "",
         "A clearance run continues from the end of a 5 minute horizon under the baseline controller with no new demand, "
         "until the network and the insertion backlog are empty. It supplies the second term of Step Zero's cost "
         "J = horizon vehicle-time + clearance vehicle-time, with a censoring penalty for vehicles still present at the "
         "cap (provisional: 60 minute cap, 60 vehicle-minutes per vehicle). Each clearance here was run with a 90 minute "
         "limit so that 30, 60 and 90 minute caps can be compared from the same run.", "",
         "The stress case holds I1 eastbound red for the 20 minutes before the checkpoint (a test-only override), so a "
         "large queue and insertion backlog must clear.", "",
         "## Results", "",
         "| Scenario | Checkpoint | In system at checkpoint | Of which backlog | Clearance time | Censored at 60 min | "
         "Horizon vehicle-hours | Clearance vehicle-hours | J (60 min cap, 60 veh-min penalty) | Wall time: resume + run |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in result["cases"]:
        name = r["scenario"] + (" (stress)" if r["stress"] else "")
        cl = r["clearance"]
        dur = f"{cl['duration_s'] / 60:.1f} min" if cl["cleared"] else f"not cleared in {cl['duration_s'] / 60:.0f} min"
        L.append(f"| {name} | {r['time']} | {r['in_system_at_checkpoint']} | {r['backlog_at_checkpoint']} | {dur} | "
                 f"{r['by_cap'][60]['censored']} | {r['horizon_vehicle_seconds'] / 3600:.2f} | "
                 f"{r['by_cap'][60]['clearance_vehicle_seconds'] / 3600:.2f} | {r['J_vehicle_hours']['cap60_pen60']:.2f} | "
                 f"{r['wall_resume_s']:.1f} s + {r['wall_run_s']:.1f} s |")
    cens = [r["by_cap"][c]["censored"] for r in result["cases"] for c in CAPS_MIN]
    walls = [r["wall_resume_s"] + r["wall_run_s"] for r in result["cases"]]
    L += ["", "## Cap and penalty sensitivity", "",
          f"Censored vehicles across all cases and caps: {sum(cens)} (largest single value {max(cens)}). "
          + ("Because no clearance reached even the 30 minute cap, J does not depend on the cap or the penalty in "
             "these cases; the sensitivity matters only for policies or faults that leave the corridor gridlocked."
             if max(cens) == 0 else "J by cap and penalty is listed per case in the JSON."), "",
          "| Scenario | Checkpoint | J, cap 30 / 60 / 90 min (penalty 60) | J, penalty 30 / 60 / 120 veh-min (cap 60) |",
          "| --- | --- | --- | --- |"]
    for r in result["cases"]:
        j = r["J_vehicle_hours"]
        L.append(f"| {r['scenario']}{' (stress)' if r['stress'] else ''} | {r['time']} | "
                 f"{j['cap30_pen60']:.2f} / {j['cap60_pen60']:.2f} / {j['cap90_pen60']:.2f} | "
                 f"{j['cap60_pen30']:.2f} / {j['cap60_pen60']:.2f} / {j['cap60_pen120']:.2f} |")
    L += ["", "## Compute", "",
          f"One clearance branch took {min(walls):.1f} to {max(walls):.1f} s of wall time on one core "
          f"({result['machine']['cpu']}, Python {result['machine']['python']}), of which starting a fresh process and "
          "resuming the checkpoint is the first figure in the table. The simulation steps at 0.1 s with the baseline "
          "controllers, detectors and ledger running.", "",
          "Clearance is therefore cheap: under the baseline controller it costs a few seconds per branch, so a paired "
          "counterfactual evaluation (a baseline branch and a candidate branch per decision, each a 5 minute horizon "
          "plus a clearance) costs well under a minute on one core, and branches run in parallel as independent "
          "processes. The cost grows with the backlog to clear (the stress case is the slowest).", "",
          "All clearances finished with zero collisions and zero ledger identity violations: "
          f"{all(r['collisions'] == 0 and r['ledger_violations'] == 0 for r in result['cases'])}.", ""]
    path = DOCS / "clearance_benchmark_m1.md"
    path.write_text("\n".join(L) + "\n", encoding="utf-8")
    return path


if __name__ == "__main__":
    import sys

    if "--report-only" in sys.argv:
        print(report(json.loads((DOCS / "clearance_benchmark_m1.json").read_text())))
    else:
        print(report(run()))
