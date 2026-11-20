"""Increment 2 acceptance tests: phase assignment and timing plans (static)."""

import json

import pytest

from cadence.controllers.phasing import (
    GROUPS, junction_foes, link_phase_map, phase_defs, ring_sequences, signal_links,
)
from cadence.controllers.timing import PLANS_PATH, generate_plans, min_split_s, plans_to_json
from cadence.controllers.phasing import load_controller_config
from cadence.simulation.demand import SCENARIOS
from cadence.simulation.network import INTERSECTIONS


@pytest.mark.parametrize("i", INTERSECTIONS)
def test_every_signal_link_has_one_phase(i, spec):
    links = signal_links(i, spec=spec)
    m = link_phase_map(i, phase_defs(i, spec), links)
    assert len(m) == len(links) == len(junction_foes(i))


@pytest.mark.parametrize("i", INTERSECTIONS)
def test_concurrent_phases_never_conflict(i, spec):
    """Against SUMO's conflict matrix: two protected movements that may show
    green together never conflict. Only permissive movements may have
    conflicting partners, and they yield in SUMO."""
    defs = phase_defs(i, spec)
    links = signal_links(i, spec=spec)
    m = link_phase_map(i, defs, links)
    foes = junction_foes(i)
    seq = ring_sequences(i, spec)
    concurrent = {(p, p) for p in defs}
    for g in GROUPS:
        for a in seq[g][1]:
            for b in seq[g][2]:
                concurrent |= {(a, b), (b, a)}
    for x in range(len(links)):
        for y in foes[x]:
            (px, perm_x), (py, perm_y) = m[x], m[y]
            if (px, py) in concurrent:
                assert perm_x or perm_y, f"{i}: protected links {x} and {y} conflict in phases {px}/{py}"


def test_permissive_movements_only_at_i2_side_lefts(spec):
    for i in INTERSECTIONS:
        perm = {m for d in phase_defs(i, spec).values() for m in d.permissive}
        assert perm == ({("SB", "L"), ("NB", "L")} if i == "I2" else set()), i


def test_phase_structure_matches_document_zero(spec):
    for i in INTERSECTIONS:
        defs = phase_defs(i, spec)
        eight = spec["intersections"][i]["phasing"] == "8-phase"
        assert set(defs) == ({1, 2, 3, 4, 5, 6, 7, 8} if eight else {1, 2, 4, 5, 6, 8})
        tc = spec["timing_constants"]
        for p, d in defs.items():
            main = p in (1, 2, 5, 6)
            assert d.yellow_s == (tc["yellow_mainline_s"] if main else tc["yellow_side_s"])
            assert d.red_clear_s == (tc["all_red_mainline_s"] if main else tc["all_red_side_s"])
        assert defs[2].coordinated and defs[6].coordinated
        assert defs[4].ped == "P4" and defs[4].ped_clear_s == tc["ped_clear_mainline_crossing_s"]
        assert defs[2].ped == "P2" and defs[2].ped_clear_s == tc["ped_clear_side_crossing_s"]
        assert (defs[4].lpi_s, defs[8].lpi_s) == ((5.0, 5.0) if i == "I4" else (0.0, 0.0))
    # I4 side streets carry no lefts
    assert all(t != "L" for p in (4, 8) for _, t in phase_defs("I4", spec)[p].movements)


def test_plans_file_reproduces_from_spec(spec):
    ctrl = load_controller_config()
    fresh = plans_to_json(generate_plans(spec, ctrl), spec, ctrl)
    assert json.loads(PLANS_PATH.read_text()) == json.loads(json.dumps(fresh))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_plans_are_consistent(scenario, spec):
    plans = generate_plans(spec)
    for i, plan in plans[scenario].items():
        defs = phase_defs(i, spec)
        assert plan.cycle_s == spec["timing_constants"]["cycle_s"][scenario]
        assert plan.group_time_s("main") + plan.group_time_s("side") == plan.cycle_s
        for p, d in defs.items():
            assert plan.splits_s[p] >= min_split_s(d), (i, p)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_offsets_give_eastbound_progression(scenario, spec):
    plans = generate_plans(spec)[scenario]
    v = spec["global"]["posted_speed_mps"]
    x = {i: spec["intersections"][i]["x"] for i in INTERSECTIONS}
    C = plans["I1"].cycle_s
    start2 = {i: (p.offset_s + p.splits_s[1]) % C for i, p in plans.items()}
    for i in INTERSECTIONS[1:]:
        lag = (start2[i] - start2["I1"]) % C
        assert lag == pytest.approx((x[i] - x["I1"]) / v, abs=1.0), i
