"""Increment 2 acceptance tests: the controller driving SUMO.

The fast tests run 20 minutes of S1 (the AM peak). The slow tests run every
scenario over its full window.
"""

import pytest

from cadence.controllers import actuated as A
from cadence.controllers.checks import check_all
from cadence.controllers.phasing import junction_foes
from cadence.simulation import demand as dm
from cadence.simulation.network import INTERSECTIONS
from cadence.simulation.runner import CorridorSimulation


def _run(scenario, workdir, begin_s=None, end_s=None):
    sim = CorridorSimulation(scenario, 0, workdir, begin_s=begin_s, end_s=end_s)
    sim.start()
    try:
        rec = sim.run_until()
    finally:
        sim.close()
    return sim, rec


def _assert_clean(sim, rec):
    assert rec.collisions == []
    assert rec.teleports == 0
    assert rec.warnings["emergency_braking"] == []
    assert rec.warnings["collision"] == [] and rec.warnings["teleport"] == []
    for i, c in sim.controllers.items():
        assert check_all(rec.state_changes[i], rec.end_tick, sim.signal_maps[i], junction_foes(i), c) == [], i
        assert c.overruns == 0, i


@pytest.fixture(scope="module")
def s1_run(tmp_path_factory):
    w = dm.scenario_window("S1")
    return _run("S1", tmp_path_factory.mktemp("s1"), end_s=w.start + 1200)


def test_s1_run_is_safe_and_legal(s1_run):
    _assert_clean(*s1_run)


def test_s1_every_phase_and_pedestrian_phase_is_served(s1_run):
    sim, rec = s1_run
    for i, c in sim.controllers.items():
        greens = {p for _, k, p in c.events if k == A.EV_PHASE_BEGIN_GREEN}
        walks = {p for _, k, p in c.events if k == A.EV_PED_BEGIN_WALK}
        assert greens == set(c.ph), i
        assert walks == {2, 4, 6, 8}, i


def test_s1_traffic_flows(s1_run):
    sim, rec = s1_run
    assert rec.departed > 1500
    assert rec.arrived > 0.85 * rec.departed


def test_runs_are_deterministic(tmp_path):
    w = dm.scenario_window("S3")
    a_sim, a = _run("S3", tmp_path / "a", begin_s=w.start, end_s=w.start + 300)
    b_sim, b = _run("S3", tmp_path / "b", begin_s=w.start, end_s=w.start + 300)
    assert dict(a.state_changes) == dict(b.state_changes)
    assert a_sim.events() == b_sim.events()
    assert (a.departed, a.arrived) == (b.departed, b.arrived)


@pytest.mark.slow
@pytest.mark.parametrize("scenario", dm.SCENARIOS)
def test_full_scenario_is_safe_and_legal(scenario, tmp_path):
    sim, rec = _run(scenario, tmp_path)
    _assert_clean(sim, rec)
    for i in INTERSECTIONS:
        assert {p for _, k, p in sim.controllers[i].events if k == A.EV_PHASE_BEGIN_GREEN} == set(sim.controllers[i].ph)
