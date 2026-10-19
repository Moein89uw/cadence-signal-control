"""Increment 1 acceptance tests: scenario demand."""

import math

import pytest

from cadence.simulation import demand as d

# DZ 4.2 derived EB through demand arriving at the I4 bottleneck (veh/h)
DERIVED = {"S0": 430, "S1": 760, "S2": 1050, "S3": 720}


@pytest.mark.parametrize("scenario", d.SCENARIOS)
def test_derived_bottleneck_volume_matches_document_zero(scenario, spec):
    seg = {"EB": spec["scenarios"]["S2"]["eb_entry_core"]["flow"]} if scenario == "S2" else None
    f = d.expected_movement_flows(scenario, seg, spec)
    through_lane = f[("I4", "EB", "T")] + f[("I4", "EB", "R")]
    assert through_lane == pytest.approx(DERIVED[scenario], rel=0.03)


@pytest.mark.parametrize("scenario", d.SCENARIOS)
def test_turning_fractions_sum_to_one(scenario, spec):
    from cadence.simulation.network import INTERSECTIONS
    for i in INTERSECTIONS:
        for a in ("EB", "WB", "NB", "SB"):
            assert sum(d.turn_fractions(i, a, scenario, spec).values()) == pytest.approx(1.0)


def test_generation_is_deterministic(spec):
    a = d.generate_vehicles("S0", 3, spec)
    b = d.generate_vehicles("S0", 3, spec)
    assert [(v.vid, v.depart, v.edges) for v in a] == [(v.vid, v.depart, v.edges) for v in b]
    assert d.generate_ped_calls("S0", 3, spec) == d.generate_ped_calls("S0", 3, spec)


def test_replications_differ(spec):
    a = d.generate_vehicles("S0", 0, spec)
    b = d.generate_vehicles("S0", 1, spec)
    assert [v.depart for v in a[:50]] != [v.depart for v in b[:50]]


def test_streams_are_independent(spec):
    """Removing the S3 pulse stream must not change any other stream's draws."""
    s3 = {v.vid: (v.depart, v.edges) for v in d.generate_vehicles("S3", 0, spec)}
    no_pulse = {k: v for k, v in s3.items() if not k.startswith("PULSE")}
    import copy
    spec2 = copy.deepcopy(spec)
    del spec2["scenarios"]["S3"]["pulse"]
    without = {v.vid: (v.depart, v.edges) for v in d.generate_vehicles("S3", 0, spec2)}
    assert without == no_pulse


@pytest.mark.parametrize("scenario", d.SCENARIOS)
def test_entry_counts_consistent_with_rates(scenario, spec):
    vehicles = d.generate_vehicles(scenario, 0, spec)
    for s in d.scenario_streams(scenario, spec):
        n = sum(1 for v in vehicles if v.vid.split(".")[0] == s.name)
        mean = sum((e - b) * r / 3600 for b, e, r in d.stream_rate_segments(s, scenario, spec))
        assert abs(n - mean) <= 4 * math.sqrt(mean) + 1, (s.name, n, mean)


def test_s3_pulse_window_and_split(spec):
    vehicles = [v for v in d.generate_vehicles("S3", 0, spec) if v.vid.startswith("PULSE")]
    start, end = d.hhmm("18:05"), d.hhmm("18:15")
    assert all(start <= v.depart < end for v in vehicles)
    assert all(v.decisions[0][:2] == ("I3", "NB") for v in vehicles)
    rights = sum(1 for v in vehicles if v.decisions[0][2] == "R") / len(vehicles)
    assert rights == pytest.approx(0.70, abs=0.15)


@pytest.mark.parametrize("scenario", d.SCENARIOS)
def test_routes_follow_network_connections(scenario, spec, net_connections):
    edge_pairs = {(f, t) for f, t, _, _ in net_connections}
    for v in d.generate_vehicles(scenario, 0, spec):
        for a, b in zip(v.edges, v.edges[1:]):
            assert (a, b) in edge_pairs, (v.vid, a, b)
