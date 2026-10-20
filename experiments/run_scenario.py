"""Run one Corridor R-1 scenario under the baseline controller and report
safety and timing checks.

    python experiments/run_scenario.py --scenario S1
    python experiments/run_scenario.py --scenario S3 --begin 18:00 --minutes 30

Writes runs/<scenario>_rNN/summary.json (runs/ is not committed).
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from cadence.controllers import actuated as A
from cadence.controllers.checks import check_all, service_waits
from cadence.controllers.phasing import junction_foes
from cadence.paths import REPO_ROOT
from cadence.simulation import demand as dm
from cadence.simulation.runner import CorridorSimulation


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True, choices=dm.SCENARIOS)
    ap.add_argument("--replication", type=int, default=0)
    ap.add_argument("--begin", help="start time HH:MM (default: scenario start)")
    ap.add_argument("--minutes", type=float, help="duration (default: to scenario end)")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    w = dm.scenario_window(args.scenario)
    begin = dm.hhmm(args.begin) if args.begin else w.start
    end = begin + args.minutes * 60 if args.minutes else w.end
    out = args.out or REPO_ROOT / "runs" / f"{args.scenario}_r{args.replication:02d}"
    t0 = time.time()
    sim = CorridorSimulation(args.scenario, args.replication, out, begin_s=begin, end_s=end)
    sim.start()
    try:
        rec = sim.run_until()
    finally:
        sim.close()
    summary = {
        "scenario": args.scenario, "replication": args.replication,
        "begin_s": begin, "end_s": end, "wall_s": round(time.time() - t0, 1),
        "departed": rec.departed, "arrived": rec.arrived,
        "collisions": len(rec.collisions), "teleports": rec.teleports,
        "sumo_warnings": {k: len(v) for k, v in rec.warnings.items()},
        "accounting": {
            "identity_violations": len(sim.ledger.violations),
            "system_vehicle_hours": round(sim.ledger.vehicle_seconds / 3600, 2),
            "network_vehicle_hours": round(sim.ledger.network_vehicle_seconds / 3600, 2),
            "queued_vehicle_hours": round(sim.ledger.queued_vehicle_seconds / 3600, 2),
            "max_backlog": max((s.backlog for s in sim.ledger.snapshots), default=0),
            "final": vars(sim.ledger.last),
        },
        "intersections": {},
    }
    for i, c in sim.controllers.items():
        violations = check_all(rec.state_changes[i], rec.end_tick, sim.signal_maps[i], junction_foes(i), c)
        veh, ped, _ = service_waits(c.events, c, rec.end_tick)
        greens = {}
        for _, k, p in c.events:
            if k == A.EV_PHASE_BEGIN_GREEN:
                greens[p] = greens.get(p, 0) + 1
        summary["intersections"][i] = {
            "violations": violations[:20], "violation_count": len(violations), "overruns": c.overruns,
            "greens_per_phase": dict(sorted(greens.items())),
            "max_call_wait_s": {p: w / 10 for p, w in sorted(veh.items())},
            "max_ped_wait_s": {f"P{p}": w / 10 for p, w in sorted(ped.items())},
        }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "intersections"}, indent=2))
    for i, s in summary["intersections"].items():
        print(i, "violations", s["violation_count"], "overruns", s["overruns"],
              "max call wait", max(s["max_call_wait_s"].values(), default=0), "s")


if __name__ == "__main__":
    main()
