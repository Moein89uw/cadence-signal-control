"""Increment 5 acceptance tests: save, resume and one-action counterfactual replay.

Guarantee under test: runs resumed from the same checkpoint with the same
actions are identical, bit for bit. A resumed run also tracks the
uninterrupted run (exactly at most checkpoints; see
cadence.simulation.replay for why not always), which is checked with a
tolerance. Every run here uses a fresh process in which SUMO starts once.
"""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from cadence.controllers import actuated as A
from cadence.controllers.checks import (
    check_barrier, check_conflicts, check_intergreen, check_link_sequences, check_pedestrians,
)
from cadence.controllers.phasing import junction_foes
from cadence.simulation import demand as dm
from cadence.simulation.network import INTERSECTIONS
from cadence.simulation.overrides import HoldRed
from cadence.simulation.replay import Action, in_fresh_process, outputs_after, run_branch
from cadence.simulation.runner import CorridorSimulation

from ctrl_helpers import QueueTraffic, drive, foes, link_map, make


def _main_line(workdir, scenario, checkpoint_s, continue_to_s, override=None):
    w = dm.scenario_window(scenario)
    sim = CorridorSimulation(scenario, 0, workdir / "main", begin_s=w.start, end_s=w.end, signal_override=override)
    sim.start()
    try:
        sim.run_until(checkpoint_s)
        cp = sim.checkpoint()
        sim.run_until(continue_to_s)
        return cp, outputs_after(sim, cp)
    finally:
        sim.close()


def _main_and_checkpoint(workdir, scenario, checkpoint_s, continue_to_s, override=None):
    """Run the main line in a fresh process: checkpoint, continue, and return
    (checkpoint, outputs after the checkpoint)."""
    return in_fresh_process(_main_line, workdir, scenario, checkpoint_s, continue_to_s, override)


def _same(a, b, keys=None):
    """Output keys that differ, ignoring wall-clock timings and warning counts."""
    keys = keys or [k for k in a if k not in ("warnings", "wall_s")]
    return [k for k in keys if a[k] != b[k]]


def _tracks(main, branch, cp, rel=1e-3):
    """Vehicle-time after the checkpoint agrees within `rel`."""
    vm = main["vehicle_seconds"] - cp.ledger["vehicle_seconds"]
    vb = branch["vehicle_seconds"] - cp.ledger["vehicle_seconds"]
    return abs(vb - vm) <= rel * vm


def test_branches_from_one_checkpoint_are_identical(tmp_path):
    w = dm.scenario_window("S1")
    cp, main = _main_and_checkpoint(tmp_path, "S1", w.start + 600, w.start + 900)
    b1 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "b1", until_s=w.start + 900)
    b2 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "b2", until_s=w.start + 900)
    assert _same(b1, b2) == []
    assert _tracks(main, b1, cp)
    assert len(b1["vehicles"]) > 150 and sum(len(v) for v in b1["controller_events"].values()) > 100


def test_resume_keeps_the_insertion_backlog(tmp_path):
    w = dm.scenario_window("S1")
    hold = HoldRed("I1", "EB", w.start + 120, 600)
    cp, main = _main_and_checkpoint(tmp_path, "S1", w.start + 600, w.start + 1000, override=hold)
    assert cp.ledger["last"].backlog > 20
    b1 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "b1", until_s=w.start + 1000)
    b2 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "b2", until_s=w.start + 1000)
    assert _same(b1, b2) == []
    assert _tracks(main, b1, cp)
    assert main["ledger_violations"] == [] and b1["ledger_violations"] == []


def test_one_action_counterfactual_is_reproducible_and_legal(tmp_path):
    w = dm.scenario_window("S1")
    t_cp, t_end = w.start + 900, w.start + 1200
    cp, main = _main_and_checkpoint(tmp_path, "S1", t_cp, t_end)
    action = (Action(t_cp, "I4", "hold", 2, duration_s=110),)  # hold EB coordinated green one cycle
    base = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "base", until_s=t_end)
    base2 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "base2", until_s=t_end)
    a1 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "a1", until_s=t_end, actions=action)
    a2 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "a2", until_s=t_end, actions=action)
    assert _same(base, base2) == []
    assert _same(a1, a2) == []
    assert _tracks(main, base, cp)
    assert a1["signal_states"]["I4"] != base["signal_states"]["I4"]
    assert a1["vehicles"] != base["vehicles"]
    assert any(code == A.EV_CMD_HOLD for _, code, _ in a1["controller_events"]["I4"])
    assert a1["collisions"] == []
    for i in INTERSECTIONS:
        c = make(i, "S1")
        lm, fs = link_map(i), foes(i)
        ch = a1["signal_states"][i]
        assert check_conflicts(ch, lm, fs, c) == []
        assert check_barrier(ch, lm, c) == []
        assert check_link_sequences(ch, a1["tick"], lm, c) == []
        assert check_intergreen(ch, a1["tick"], lm, fs, c) == []
        assert check_pedestrians(a1["controller_events"][i], c) == []


def test_clearance_runs_with_no_new_demand(tmp_path):
    w = dm.scenario_window("S1")
    horizon_end = w.start + 900
    cp, _ = _main_and_checkpoint(tmp_path, "S1", horizon_end, horizon_end)
    out = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "clear", clearance_cap_s=3600)
    again = in_fresh_process(run_branch, cp.sumo_state, tmp_path / "clear2", clearance_cap_s=3600)
    assert _same(out, again) == [] and out["clearance"] == again["clearance"]
    routes = cp.config["files"]["routes"]
    sched = {v.get("id"): float(v.get("depart")) for v in ET.parse(routes).getroot().iter("vehicle")}
    horizon_vehicles = sum(1 for d in sched.values() if d < horizon_end)
    clr = out["clearance"]
    assert clr["cleared"] and clr["censored_vehicles"] == 0
    assert out["ledger_last"]["exited"] == horizon_vehicles
    assert out["ledger_violations"] == [] and out["collisions"] == []
    assert 0 < clr["duration_s"] < 900 and clr["vehicle_seconds"] > 0


def test_checkpoint_and_resume_refuse_a_used_process(tmp_path):
    w = dm.scenario_window("S0")
    cp, _ = _main_and_checkpoint(tmp_path, "S0", w.start + 60, w.start + 60)
    # make sure this process has already run a SUMO session
    first = CorridorSimulation("S0", 0, tmp_path / "used", begin_s=w.start, end_s=w.start + 10)
    first.start(); first.run_until(); first.close()
    with pytest.raises(RuntimeError, match="fresh"):
        CorridorSimulation.resume(cp, tmp_path / "x")
    again = CorridorSimulation("S0", 0, tmp_path / "used2", begin_s=w.start, end_s=w.start + 10)
    again.start()
    try:
        with pytest.raises(RuntimeError, match="fresh"):
            again.checkpoint()
    finally:
        again.close()


def test_family_b_controllers_reject_omit():
    a, b = make("I1"), make("I2")
    a.command("omit", 3, 600)
    with pytest.raises(ValueError):
        b.command("omit", 4, 600)
    for c in (a, b):
        c.command("call", 4)
        c.command("hold", 2, 10)
        c.command("force_off", 4)
    with pytest.raises(ValueError):
        a.command("force_off", 2)


@pytest.mark.parametrize("i", INTERSECTIONS)
def test_random_commands_never_break_safety(i):
    """Calls, holds, force-offs and omits change timing but never shorten
    minimum green, clearance or pedestrian intervals."""
    c = make(i, "S1")
    traffic = QueueTraffic(c, seed=100 + INTERSECTIONS.index(i))
    rng = np.random.default_rng(INTERSECTIONS.index(i))
    kinds = sorted(c.allowed_commands)

    def presence(t, c):
        if rng.random() < 0.01:
            kind = kinds[rng.integers(len(kinds))]
            phases = [p for p in c.ph if not (kind in ("force_off", "omit") and c.ph[p].coordinated)]
            c.command(kind, int(phases[rng.integers(len(phases))]), int(rng.integers(10, 300)))
        return traffic.presence(t, c)
    end = 20 * c.cycle
    changes = drive(c, 0, end, presence=presence, peds=traffic.peds)
    lm, fs = link_map(i), foes(i)
    assert check_conflicts(changes, lm, fs, c) == []
    assert check_barrier(changes, lm, c) == []
    assert check_link_sequences(changes, end, lm, c) == []
    assert check_intergreen(changes, end, lm, fs, c) == []
    assert check_pedestrians(c.events, c) == []
    assert any(k in (A.EV_CMD_CALL, A.EV_CMD_HOLD, A.EV_CMD_FORCE_OFF) for _, k, _ in c.events)


@pytest.mark.slow
def test_resume_is_exact_at_many_checkpoints_under_congestion(tmp_path):
    """Ten checkpoints across the S2 build-up and core. From each, two resumed
    branches are identical, and they track the uninterrupted run."""
    cps, mains = in_fresh_process(_many_checkpoints, tmp_path)
    exact = 0
    for k, (cp, main) in enumerate(zip(cps, mains)):
        b1 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / f"b{k}a", until_s=cp.tick / 10 + 60)
        b2 = in_fresh_process(run_branch, cp.sumo_state, tmp_path / f"b{k}b", until_s=cp.tick / 10 + 60)
        assert _same(b1, b2) == [], f"checkpoint {k} at t={cp.tick / 10}"
        assert _tracks(main, b1, cp), f"checkpoint {k} at t={cp.tick / 10}"
        exact += _same(main, b1) == []
    assert exact >= 5


def _many_checkpoints(tmp_path):
    sim = CorridorSimulation("S2", 0, tmp_path / "main", begin_s=dm.hhmm("16:30"), end_s=dm.hhmm("17:45"))
    sim.start()
    cps, mains = [], []
    try:
        for k in range(10):
            sim.run_until(dm.hhmm("16:40") + k * 360)
            cp = sim.checkpoint()
            sim.run_until(cp.tick / 10 + 60)
            cps.append(cp)
            mains.append(outputs_after(sim, cp))
    finally:
        sim.close()
    return cps, mains
