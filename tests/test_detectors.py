"""Increment 3 acceptance tests: detector layout and fault-free observations."""

import xml.etree.ElementTree as ET
from collections import Counter

import pytest

from cadence.controllers import actuated as A
from cadence.controllers.checks import check_all
from cadence.controllers.phasing import junction_foes, phase_defs, phase_lanes
from cadence.detectors.layout import STOPBAR_KINDS, actuation_map, build_layout
from cadence.detectors.logs import FAMILY_B_CODES, detector_counts, hires_log, tmc_counts
from cadence.simulation import demand as dm
from cadence.simulation.network import APPROACHES, INTERSECTIONS, approach_edge, approach_edges_upstream
from cadence.simulation.runner import CorridorSimulation


@pytest.fixture(scope="module")
def layout(net_lanes, spec):
    return build_layout(net_lanes, spec)


def _of(dets, i, kind):
    return [d for d in dets if d.intersection == i and d.kind == kind]


def test_stopbar_detection_covers_every_lane(layout, net_lanes, spec):
    dets, _ = layout
    kinds = {"I1": "stopbar_loop", "I2": "stopbar_loop", "I3": "radar_stopbar", "I4": "video_presence"}
    for i in INTERSECTIONS:
        lanes = {(approach_edge(i, a, spec), l) for a in APPROACHES for l in range(len(net_lanes[approach_edge(i, a, spec)]))}
        assert {(d.edge, d.lane) for d in _of(dets, i, kinds[i])} == lanes, i
        others = {d.kind for d in dets if d.intersection == i and d.kind in STOPBAR_KINDS} - {kinds[i]}
        assert not others, i


def test_stopbar_zones_end_at_the_stop_line(layout, net_lanes, spec):
    dets, _ = layout
    zone = spec["detection"]["stopbar_zone_m"]
    for d in dets:
        if d.kind in STOPBAR_KINDS:
            assert d.end_m == pytest.approx(net_lanes[d.edge][d.lane])
            assert d.end_m - d.start_m == pytest.approx(min(zone, net_lanes[d.edge][d.lane]))


def test_i1_advance_and_system_loops(layout, net_lanes, spec):
    dets, _ = layout
    adv = _of(dets, "I1", "advance_loop")
    assert {d.approach for d in adv} == {"EB", "WB"}
    for d in adv:
        assert d.distance_m == 100.0
        # independently: distance from the loop's downstream edge to the I1 stop line
        edges = approach_edges_upstream("I1", d.approach, spec)
        after = sum(net_lanes[e][0] for e in edges[edges.index(d.edge) + 1:])
        assert net_lanes[d.edge][0] - d.end_m + after == pytest.approx(100.0)
    assert Counter(d.approach for d in adv) == {"EB": 2, "WB": 2}
    sysl = _of(dets, "I1", "system_loop")
    assert Counter(d.approach for d in sysl) == {"EB": 2, "WB": 2}
    eb_link = sum(net_lanes[e][0] for e in approach_edges_upstream("I2", "EB", spec))
    wb_link = sum(net_lanes[e][0] for e in approach_edges_upstream("I1", "WB", spec))
    assert {round(d.distance_m, 2) for d in sysl if d.approach == "EB"} == {round(eb_link / 2, 2)}
    assert {round(d.distance_m, 2) for d in sysl if d.approach == "WB"} == {round(wb_link / 2, 2)}
    assert not [d for d in dets if d.intersection != "I1" and d.kind in ("advance_loop", "system_loop")]


def test_i3_radar_covers_120_m_on_every_approach(layout, net_lanes, spec):
    _, radars = layout
    assert {(z.intersection, z.approach) for z in radars} == {("I3", a) for a in APPROACHES}
    for z in radars:
        edges = approach_edges_upstream("I3", z.approach, spec)
        total = sum(net_lanes[e][0] for e in edges)
        covered = {}
        for e, l, lo, hi, off in z.segments:
            covered.setdefault(e, (lo, hi))
            assert hi == pytest.approx(net_lanes[e][l])
        span = sum(hi - lo for lo, hi in covered.values())
        assert span == pytest.approx(min(120.0, total)), z.approach
        lanes = {(e, l) for e, l, *_ in z.segments}
        assert lanes == {(e, l) for e in covered for l in range(len(net_lanes[e]))}


def test_each_stopbar_detector_calls_its_lanes_phase(layout, spec):
    dets, _ = layout
    act = actuation_map(dets)
    for i in INTERSECTIONS:
        for p, d in phase_defs(i, spec).items():
            expected = {(e, l) for e, l in phase_lanes(i, d, spec)}
            got = {(x.edge, x.lane) for x in dets if x.id in act[i][p]}
            assert got == expected, (i, p)


def test_channels_are_unique_per_cabinet(layout):
    dets, _ = layout
    for i in INTERSECTIONS:
        chans = [d.channel for d in dets if d.intersection == i]
        assert sorted(chans) == list(range(1, len(chans) + 1))


def test_family_logs_follow_document_zero_resolution():
    ctrl = [(1000, A.EV_PHASE_BEGIN_GREEN, 4), (1003, A.EV_PHASE_GAP_OUT, 4), (1003, A.EV_PHASE_BEGIN_YELLOW, 4)]
    det = [(1001, 7, 1), (1002, 7, 0)]
    ped = [(1001, 4)]
    a = hires_log("A", ctrl, det, ped)
    b = hires_log("B", ctrl, det, ped)
    assert (100.3, A.EV_PHASE_GAP_OUT, 4) in a and (100.1, 82, 7) in a
    assert all(isinstance(t, int) for t, _, _ in b)
    assert {c for _, c, _ in b} <= FAMILY_B_CODES and A.EV_PHASE_GAP_OUT not in {c for _, c, _ in b}
    # detector event precedes the controller event in the same tick
    assert hires_log("A", [(5, 1, 2)], [(5, 3, 1)], []) == [(0.5, 82, 3), (0.5, 1, 2)]


# ---------------------------------------------------------------------------
# Observations from a SUMO run
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def obs_run(tmp_path_factory):
    w = dm.scenario_window("S1")
    work = tmp_path_factory.mktemp("obs")
    sim = CorridorSimulation("S1", 0, work, begin_s=w.start, end_s=w.start + 900,
                             extra_sumo_args=("--vehroute-output", str(work / "vehroutes.xml"),
                                              "--vehroute-output.exit-times", "true",
                                              "--vehroute-output.write-unfinished", "true"),
                             presence_audit_every=7)
    sim.start()
    try:
        rec = sim.run_until()
    finally:
        sim.close()
    return sim, rec, work


def test_controller_actuated_by_detectors_stays_legal(obs_run):
    sim, rec, _ = obs_run
    assert rec.collisions == [] and rec.teleports == 0
    for i, c in sim.controllers.items():
        assert check_all(rec.state_changes[i], rec.end_tick, sim.signal_maps[i], junction_foes(i), c) == []


def test_detector_states_match_vehicle_positions(obs_run):
    """Fault-free detectors report exactly the vehicles that overlap them,
    computed independently from vehicle positions and lengths."""
    sim, _, _ = obs_run
    audit = sim.presence_audit
    assert audit["samples"] > 1000
    assert audit["mismatches"] == []
    # rears overhanging into a junction after a last-moment lane change are rare
    assert audit["ambiguous"] <= 0.001 * audit["samples"] * len(sim.detectors)


def test_point_loops_count_every_vehicle(obs_run):
    sim, _, _ = obs_run
    seen = sim.presence_audit["vehicles_seen"]
    by_ch = {d.id: d for d in sim.detectors}
    on_events = Counter()
    for i, evs in sim.recorder.obs.detector_events.items():
        chans = {d.channel: d.id for d in sim.detectors if d.intersection == i}
        for t, ch, on in evs:
            if on:
                on_events[chans[ch]] += 1
    for did, d in by_ch.items():
        if d.kind in ("advance_loop", "system_loop"):
            assert on_events[did] == len(seen[did]) > 50, did


def test_video_turning_counts_match_sumo_routes(obs_run, spec):
    sim, _, work = obs_run
    root = ET.parse(work / "vehroutes.xml").getroot()
    app = {approach_edge("I3", a, spec): a for a in APPROACHES}
    truth = Counter()
    begin, end = sim.begin_s, sim.end_s
    for v in root.findall("vehicle"):
        r = v.find("route")
        edges = r.get("edges").split()
        exits = [float(x) for x in (r.get("exitTimes") or "").split()]
        for k, e in enumerate(edges[:-1]):
            if e in app and k < len(exits) and 0 <= exits[k] < end - 1:
                truth[(app[e], edges[k + 1])] += 1
    got = Counter()
    from cadence.simulation.network import DEPARTURE_DIRECTION, departure_edge
    for t, a, turn in sim.recorder.obs.tmc:
        got[(a, departure_edge("I3", DEPARTURE_DIRECTION[(a, turn)]))] += 1
    # a vehicle still inside the junction at the end of the run may differ by one
    for k in set(truth) | set(got):
        assert abs(truth[k] - got[k]) <= 2, (k, truth[k], got[k])
    assert sum(got.values()) > 500
    bins = tmc_counts(sim.recorder.obs.tmc, spec["detection"]["I3"]["video_tmc"]["bin_s"], sim.record.begin_tick)
    assert sum(bins.values()) == len(sim.recorder.obs.tmc)


def test_radar_tracks_are_physical_and_match_vehicles(obs_run):
    sim, _, _ = obs_run
    import numpy as np
    from cadence.detectors.logs import radar_array
    r = radar_array(sim.recorder.obs.radar)
    assert len(r) > 10000
    assert r["distance_m"].min() >= 0 and r["distance_m"].max() <= 120.0 + 1e-3
    assert r["speed_mps"].min() >= 0 and r["speed_mps"].max() <= 13.89 * 1.3
    for track in np.unique(r["track"])[:200]:
        rows = r[r["track"] == track]
        assert len(set(rows["approach"])) == 1
        # vehicles move toward the stop line: distance never increases
        assert (np.diff(rows["distance_m"]) <= 1e-3).all(), track
    # one track per vehicle
    vids = list(sim.recorder.truth_track_vehicle.values())
    assert len(vids) == len(set(vids))


def test_logs_written_for_every_cabinet(obs_run):
    sim, _, work = obs_run
    for i in INTERSECTIONS:
        path = work / "logs" / f"{i}_hires.csv"
        assert path.exists()
        rows = path.read_text().splitlines()[1:]
        fam = sim.spec["intersections"][i]["family"]
        times = [r.split(",")[0] for r in rows]
        if fam == "B":
            assert all("." not in t for t in times)
        assert len(rows) > 500
    counts = detector_counts(sim.recorder.obs.detector_events["I1"], 300, sim.record.begin_tick)
    assert sum(counts.values()) > 0


@pytest.mark.slow
def test_detectors_match_vehicle_positions_under_s2_congestion(tmp_path):
    w = dm.scenario_window("S2")
    sim = CorridorSimulation("S2", 0, tmp_path, begin_s=w.start, end_s=dm.hhmm("17:45"), presence_audit_every=11)
    sim.start()
    try:
        rec = sim.run_until()
    finally:
        sim.close()
    audit = sim.presence_audit
    assert rec.collisions == [] and rec.teleports == 0
    assert audit["mismatches"] == []
    assert audit["ambiguous"] <= 0.001 * audit["samples"] * len(sim.detectors)
    seen = audit["vehicles_seen"]
    for i, evs in sim.recorder.obs.detector_events.items():
        chans = {d.channel: d for d in sim.detectors if d.intersection == i}
        ons = Counter(chans[ch].id for _, ch, on in evs if on)
        for did, n in ons.items():
            if did in seen:
                assert n == len(seen[did]), did
