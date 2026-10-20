"""Increment 6 acceptance tests: the corridor queue model's mechanisms.

These use scripted signals and hand-placed vehicles. The comparison with
SUMO is in experiments/validate_prediction.py (report in docs/validation);
a short version runs here as a slow test.
"""

import pytest

from cadence.prediction.queue_model import CorridorQueueModel, ModelParams, Vehicle
from cadence.simulation.network import APPROACHES, INTERSECTIONS, TURNS

# DZ 3.2-3.3: usable through storage per approach link and left-bay storage (vehicles)
STORAGE = {("I1", "EB"): 160, ("I2", "EB"): 100, ("I3", "EB"): 32, ("I4", "EB"): 58,
           ("I4", "WB"): 160, ("I3", "WB"): 74, ("I2", "WB"): 32, ("I1", "WB"): 100}
BAYS = {("I1", "EB"): 10, ("I1", "WB"): 10, ("I2", "EB"): 8, ("I2", "WB"): 8, ("I3", "EB"): 5,
        ("I3", "WB"): 8, ("I4", "EB"): 7, ("I4", "WB"): 8, ("I1", "SB"): 6, ("I1", "NB"): 6,
        ("I3", "SB"): 4, ("I3", "NB"): 4}


def signals(green: dict):
    """green: {(intersection, approach, turn): 'G'|'g'} -> everything else red."""
    def fn(i, t):
        return {(a, tr): green.get((i, a, tr), "r") for a in APPROACHES for tr in TURNS}
    return fn


def model(**kw):
    m = CorridorQueueModel(ModelParams(**kw))
    m.reset(0.0)
    return m


def test_storage_and_bays_match_document_zero():
    m = model()
    for key, n in STORAGE.items():
        assert m.approaches[key].storage == n, key
    for key, ap in m.approaches.items():
        assert ap.bay_cap == BAYS.get(key, 0), key
    for i in INTERSECTIONS:
        for a in ("NB", "SB"):
            assert m.approaches[(i, a)].storage == 40
    assert [l.movements for l in m.approaches[("I2", "SB")].lanes] == [("L", "T", "R")]


def test_no_discharge_on_red_and_saturation_headway_on_green():
    m = model()
    for k in range(60):
        m.add_queued(Vehicle(f"v{k}", [("I1", "EB", "T")]))
    m.set_signals(signals({}))
    m.run(20)
    assert m.discharges == []
    m.set_signals(signals({("I1", "EB", "T"): "G"}))
    m.run(60)
    times = [t for t, _ in m.discharges]
    # two through lanes, 2.0 s headway, first departures after the 2 s start-up loss
    assert min(times) == 22.0
    assert len(times) == 2 * len(range(22, 60, 2))


def test_vehicles_are_conserved():
    m = model()
    m.set_signals(signals({(i, a, t): "G" for i in INTERSECTIONS for a in ("EB", "WB") for t in TURNS}))
    for k in range(40):
        m.add_entry(Vehicle(f"e{k}", [("I1", "EB", "T"), ("I2", "EB", "T"), ("I3", "EB", "T"), ("I4", "EB", "T")]), k * 2.0)
    for t in (50, 100, 200, 400):
        m.run(t)
        left_network = sum(1 for _, mv in m.discharges if mv == ("I4", "EB", "T")) - len(m.exiting)
        scheduled = sum(1 for k in range(40) if k * 2.0 <= m.t)
        assert m.count_in_system() + left_network == scheduled, t
    assert m.count_in_system() == 0


def test_full_link_blocks_upstream_discharge():
    m = model()
    down = m.approaches[("I3", "EB")]
    for k in range(down.storage):
        m.approaches[("I3", "EB")].lanes[k % 2].queue.append(Vehicle(f"d{k}", [("I3", "EB", "T")]))
    for k in range(5):
        m.add_queued(Vehicle(f"u{k}", [("I2", "EB", "T"), ("I3", "EB", "T")]))
    m.set_signals(signals({("I2", "EB", "T"): "G"}))   # I3 stays red
    m.run(60)
    assert not [1 for _, mv in m.discharges if mv == ("I2", "EB", "T")]
    assert down.through_count() == down.storage


def test_shared_lane_head_of_line_blocking():
    """A permissive left-turner with no gap holds up the through vehicles
    behind it; without FIFO they would pass."""
    results = {}
    for fifo in (True, False):
        m = model(fifo=fifo)
        m.add_queued(Vehicle("left", [("I2", "SB", "L"), ("I3", "EB", "T"), ("I4", "EB", "T")]))
        for k in range(4):
            m.add_queued(Vehicle(f"thr{k}", [("I2", "SB", "T")]))
        # continuous opposing through traffic keeps the left-turner waiting
        for k in range(20):
            m.add_queued(Vehicle(f"opp{k}", [("I2", "NB", "T")]))
        m.set_signals(signals({("I2", "SB", "L"): "g", ("I2", "SB", "T"): "G", ("I2", "SB", "R"): "G",
                               ("I2", "NB", "T"): "G"}))
        m.run(30)
        results[fifo] = sum(1 for _, mv in m.discharges if mv == ("I2", "SB", "T"))
    assert results[True] == 0
    assert results[False] == 4


def test_left_turn_gap_after_opposing_queue_clears():
    m = model()
    m.add_queued(Vehicle("left", [("I2", "SB", "L"), ("I3", "EB", "T"), ("I4", "EB", "T")]))
    for k in range(3):
        m.add_queued(Vehicle(f"opp{k}", [("I2", "NB", "T")]))
    m.set_signals(signals({("I2", "SB", "L"): "g", ("I2", "NB", "T"): "G"}))
    m.run(30)
    t_left = next(t for t, mv in m.discharges if mv == ("I2", "SB", "L"))
    t_opp = max(t for t, mv in m.discharges if mv == ("I2", "NB", "T"))
    assert t_left >= t_opp + m.p.gap_before_s


def test_bay_overflow_blocks_the_inner_through_lane():
    m = model()
    ap = m.approaches[("I3", "EB")]          # 5-vehicle bay
    for k in range(ap.bay_cap + 1):          # one more left-turner than the bay holds
        m.add_queued(Vehicle(f"L{k}", [("I3", "EB", "L")]))
    ap.lanes[ap.inner].queue.append(Vehicle("T_behind", [("I3", "EB", "T")]))
    assert len(ap.bay.queue) == ap.bay_cap
    assert [v.vid for v in ap.lanes[ap.inner].queue] == [f"L{ap.bay_cap}", "T_behind"]
    m.set_signals(signals({("I3", "EB", "T"): "G"}))   # through green, left red
    m.run(30)
    # the overflow left-turner cannot enter the full bay and holds the through vehicle behind it
    assert [v.vid for v in ap.lanes[ap.inner].queue] == [f"L{ap.bay_cap}", "T_behind"]
    m.set_signals(signals({("I3", "EB", "L"): "G", ("I3", "EB", "T"): "G"}))
    m.run(60)
    assert not ap.lanes[ap.inner].queue and not ap.bay.queue


@pytest.mark.slow
def test_model_tracks_sumo_over_five_minutes(tmp_path, spec):
    from cadence.paths import R1_DIR
    from cadence.prediction.validation import collect_truth, predict_and_compare
    from cadence.simulation.runner import net_lane_lengths

    truth = collect_truth("S1", tmp_path, max_starts=3)
    nl = net_lane_lengths(R1_DIR / "r1.net.xml")
    for t0 in truth.starts:
        c = predict_and_compare(truth, t0, 300, spec, nl)
        assert abs(c.vt_rel_error) < 0.05, t0
        tot_m, tot_s = sum(c.discharge_model.values()), sum(c.discharge_sumo.values())
        assert abs(tot_m - tot_s) <= 0.05 * tot_s, t0
