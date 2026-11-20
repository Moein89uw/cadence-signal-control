"""Increment 4 acceptance tests: vehicle accounting with teleporting disabled."""

import pytest

from cadence.accounting.ledger import read_tripinfo, vehicle_seconds_from_trips
from cadence.simulation import demand as dm
from cadence.simulation.runner import CorridorSimulation


def _run(workdir, scenario="S1", minutes=20, hold=None, audit_queue_every=0):
    w = dm.scenario_window(scenario)
    trip = workdir / "tripinfo.xml"
    sim = CorridorSimulation(scenario, 0, workdir, begin_s=w.start, end_s=w.start + minutes * 60,
                             extra_sumo_args=("--tripinfo-output", str(trip),
                                              "--tripinfo-output.write-unfinished", "true"),
                             signal_override=hold, queue_audit_every=audit_queue_every)
    sim.start()
    try:
        sim.run_until()
    finally:
        sim.close()
    return sim, trip


def hold_i1_eastbound(start_s, minutes):
    """Test-only override: keep every I1 eastbound link red for a while, so the
    entry link fills and vehicles back up outside the network."""
    def f(i, tick, state, sim):
        if i != "I1" or not (start_s * 10 <= tick < (start_s + minutes * 60) * 10):
            return state
        links = sim.signal_link_index["I1"]["EB"]
        return "".join("r" if k in links else ch for k, ch in enumerate(state))
    return f


@pytest.fixture(scope="module")
def normal(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("normal"), audit_queue_every=10)


@pytest.fixture(scope="module")
def stressed(tmp_path_factory):
    w = dm.scenario_window("S1")
    return _run(tmp_path_factory.mktemp("stress"), minutes=30,
                hold=hold_i1_eastbound(w.start + 300, 10))


@pytest.mark.parametrize("which", ["normal", "stressed"])
def test_identities_hold_every_tick(which, request):
    sim, _ = request.getfixturevalue(which)
    led = sim.ledger
    assert led.violations == []
    assert len(led.snapshots) > 1000


def test_backlog_forms_and_drains_under_a_hold(stressed):
    sim, _ = stressed
    snaps = sim.ledger.snapshots
    peak = max(s.backlog for s in snaps)
    assert peak > 50
    assert snaps[-1].backlog < 5


@pytest.mark.parametrize("which", ["normal", "stressed"])
def test_vehicle_time_matches_per_vehicle_records(which, request):
    sim, trip = request.getfixturevalue(which)
    trips = read_tripinfo(trip, sim.files["routes"])
    sched = {v: t.scheduled for v, t in trips.items()}
    # vehicles scheduled before the end that SUMO never loaded into tripinfo would be lost
    from cadence.accounting.ledger import route_departs
    assert len([d for d in route_departs(sim.files["routes"]) if d <= sim.end_s - 0.1 + 1e-6]) <= len(sched) + 0
    expect = vehicle_seconds_from_trips(trips, sched, sim.begin_s, sim.end_s)
    assert sim.ledger.vehicle_seconds == pytest.approx(expect, abs=1e-6)


@pytest.mark.parametrize("which", ["normal", "stressed"])
def test_no_vehicle_is_lost(which, request):
    sim, trip = request.getfixturevalue(which)
    trips = read_tripinfo(trip, sim.files["routes"])
    last = sim.ledger.last
    exited = sum(1 for t in trips.values() if t.exited is not None)
    assert exited == last.exited
    assert sim.record.teleports == 0 and sim.record.collisions == []


def test_queued_count_matches_lane_halting_counts(normal):
    sim, _ = normal
    assert sim.queue_audit["samples"] > 1000
    assert sim.queue_audit["mismatches"] == []
