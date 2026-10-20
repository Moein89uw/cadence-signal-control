"""Deterministic replay: resume a checkpoint, apply actions, run clearance.

A checkpoint is a SUMO state file plus the pickled Python state of the
controllers, observations, ledger and harness (see
CorridorSimulation.checkpoint). A branch resumes it in a fresh process, in
which SUMO is started exactly once, directly from the saved state.

Guarantees (each is tested):

* Exact: runs resumed from the same checkpoint with the same actions are
  identical, bit for bit. Compare a counterfactual branch only with a
  baseline branch resumed from the same checkpoint.
* Close: a resumed run tracks the uninterrupted run. It is exact at most
  checkpoints, but SUMO's state file does not store the junction
  "approaching vehicle" registrations that yielding vehicles consult, nor
  all lane-change memory, so the first steps after a load can differ by a
  fraction of a metre for a vehicle at a merge or lane change.

What does not work, and is guarded against: loading a state into a running
SUMO (stale internal records cause collisions), and saving or loading in a
process that has already run a SUMO session (libsumo does not fully reset
between sessions).

Two uses:

* one-action counterfactuals: from the same checkpoint, run the baseline
  branch and a branch with one external command, and compare;
* clearance: continue with no new demand until the network and the
  insertion backlog are empty, or a cap is reached (Step Zero cost J).
"""

from __future__ import annotations

import multiprocessing as mp
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from cadence.simulation.runner import TICKS_PER_S, Checkpoint, CorridorSimulation, load_checkpoint


@dataclass(frozen=True)
class Action:
    """An external controller command issued at the boundary before `at_s`."""
    at_s: float
    intersection: str
    kind: str          # call, hold, force_off, omit
    phase: int
    duration_s: float = 0.1


def in_fresh_process(fn, *args, **kwargs):
    """Run fn(*args, **kwargs) in a new interpreter and return its result."""
    with ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn")) as ex:
        return ex.submit(fn, *args, **kwargs).result()


def clearance_removals(cp: Checkpoint) -> tuple[str, ...]:
    """Vehicles in the saved state that are scheduled at or after the
    checkpoint time (SUMO reads routes ahead), so no new demand enters."""
    t_c = cp.tick / TICKS_PER_S
    sched = {v.get("id"): float(v.get("depart"))
             for v in ET.parse(cp.config["files"]["routes"]).getroot().iter("vehicle")}
    in_state = [v.get("id") for v in ET.parse(cp.sumo_state).getroot().iter("vehicle")]
    return tuple(v for v in in_state if sched.get(v, -1.0) >= t_c - 1e-9)


def outputs_after(sim: CorridorSimulation, cp: Checkpoint) -> dict:
    """Everything the run produced after the checkpoint, in a form that can be
    compared exactly between a continuing run and a resumed one."""
    import libsumo

    resumed = sim.resumed_from == cp.tick
    lens = cp.recorder["lens"]

    def tail(seq, n):
        return list(seq) if resumed else list(seq)[n:]

    obs = sim.recorder.obs
    radar = obs.radar.to_array()
    return {
        "tick": sim.tick,
        "vehicles": {v: (libsumo.vehicle.getLaneID(v), libsumo.vehicle.getLanePosition(v),
                         libsumo.vehicle.getSpeed(v)) for v in libsumo.vehicle.getIDList()},
        "pending": tuple(libsumo.simulation.getPendingVehicles()),
        "controller_events": {i: tail(c.events, cp.controllers[i]["n_events"]) for i, c in sim.controllers.items()},
        "signal_states": {i: tail(sim.record.state_changes.get(i, []), cp.record["state_changes"].get(i, 0))
                          for i in sim.controllers},
        "detector_events": {i: tail(obs.detector_events.get(i, []), lens["det"].get(i, 0)) for i in sim.controllers},
        "ped_presses": {i: tail(obs.ped_presses.get(i, []), lens["ped"].get(i, 0)) for i in sim.controllers},
        "radar": (radar if resumed else radar[lens["radar"]:]).tobytes(),
        "tmc": tail(obs.tmc, lens["tmc"]),
        "collisions": tail(sim.record.collisions, cp.record["collisions"]),
        "vehicle_seconds": sim.ledger.vehicle_seconds,
        "ledger_last": vars(sim.ledger.last),
        "ledger_violations": tail(sim.ledger.violations, cp.ledger["n_violations"]),
    }


def run_branch(checkpoint: Path, workdir: Path, until_s: float | None = None, actions: tuple[Action, ...] = (),
               clearance_cap_s: float | None = None) -> dict:
    """Resume `checkpoint` in this (fresh) process, apply `actions`, and run
    either to `until_s` or, with `clearance_cap_s`, as a clearance run with no
    new demand until everything has left or the cap is reached. Returns
    outputs_after(...) plus clearance results."""
    cp = load_checkpoint(checkpoint)
    t_c = cp.tick / TICKS_PER_S
    if clearance_cap_s is not None:
        sim = CorridorSimulation.resume(cp, workdir, routes=False, remove_vehicles=clearance_removals(cp),
                                        end_s=t_c + clearance_cap_s + 1)
        sim.ledger.demand_end_s = t_c
        end_tick = int(round((t_c + clearance_cap_s) * TICKS_PER_S))
    else:
        sim = CorridorSimulation.resume(cp, workdir)
        end_tick = int(round((sim.end_s if until_s is None else until_s) * TICKS_PER_S))
    pending = sorted(actions, key=lambda a: a.at_s)
    cleared_at = None
    vs0 = sim.ledger.vehicle_seconds
    try:
        while sim.tick < end_tick:
            while pending and int(round(pending[0].at_s * TICKS_PER_S)) <= sim.tick:
                a = pending.pop(0)
                sim.command(a.intersection, a.kind, a.phase, a.duration_s)
            sim.step()
            if clearance_cap_s is not None:
                last = sim.ledger.last
                if last.in_network == 0 and last.backlog == 0:
                    cleared_at = sim.tick
                    break
        out = outputs_after(sim, cp)
    finally:
        sim.close()
    if clearance_cap_s is not None:
        last = sim.ledger.last
        out["clearance"] = {
            "cleared": cleared_at is not None,
            "duration_s": (sim.tick - cp.tick) / TICKS_PER_S,
            "vehicle_seconds": sim.ledger.vehicle_seconds - vs0,
            "censored_vehicles": last.in_network + last.backlog,
        }
    out["warnings"] = {k: len(v) for k, v in sim.record.warnings.items()}
    return out
