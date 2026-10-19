"""Increment 1 acceptance tests: Corridor R-1 geometry and connectivity.

Expected values are taken from Document Zero (DZ) section 3 unless a test
says otherwise.
"""

from pathlib import Path

import pytest

from cadence.paths import R1_DIR
from cadence.simulation.network import (
    APPROACHES, DEPARTURE_DIRECTION, INTERSECTIONS, TURNS,
    approach_edge, build_network, departure_edge, movement_lanes,
)

JAM = 7.5


def _strip_header(text: str) -> str:
    return "\n".join(ln for ln in text.splitlines() if not ln.startswith("<!-- generated on"))


def test_network_rebuild_reproduces_committed_files(tmp_path, spec):
    build_network(spec, tmp_path)
    for name in ("r1.nod.xml", "r1.edg.xml", "r1.con.xml", "r1.net.xml"):
        committed = (R1_DIR / name).read_text(encoding="utf-8")
        rebuilt = (tmp_path / name).read_text(encoding="utf-8")
        assert _strip_header(rebuilt) == _strip_header(committed), f"{name} differs from a fresh build"


@pytest.mark.parametrize("direction", ["EB", "WB"])
def test_mainline_edges_match_spec(direction, spec, net_lanes):
    for link in spec["mainline_links"][direction]:
        for e in link["edges"]:
            lanes = net_lanes[e["id"]]
            assert len(lanes) == e["lanes"], e["id"]
            assert all(abs(L - e["length_m"]) < 1e-6 for L in lanes), e["id"]


# DZ 3.2 usable storage, vehicles per through lane
STORAGE = {
    ("EB", "I1"): 80, ("EB", "I2"): 50, ("EB", "I3"): 16,
    ("WB", "I4"): 80, ("WB", "I3"): 37, ("WB", "I2"): 16, ("WB", "I1"): 50,
}


@pytest.mark.parametrize("key", sorted(STORAGE))
def test_usable_storage_per_through_lane(key, spec, net_lanes):
    direction, to = key
    link = next(l for l in spec["mainline_links"][direction] if l["to"] == to)
    through_len = sum(net_lanes[e["id"]][0] for e in link["edges"])
    assert through_len / JAM == pytest.approx(STORAGE[key], abs=1e-6)


def test_i3_i4_eastbound_lane_drop(net_lanes, net_root):
    # DZ 3.2: 21 veh/lane x 2 to the lane drop, then 16 veh x 1 over the last 120 m
    assert len(net_lanes["EB3a"]) == 2 and net_lanes["EB3a"][0] / JAM == pytest.approx(21)
    assert len(net_lanes["EB3b"]) == 1
    one_lane_section = net_lanes["EB3b"][0] + net_lanes["EB3c"][0]
    assert one_lane_section / JAM == pytest.approx(16)
    node = next(j for j in net_root.findall("junction") if j.get("id") == "D")
    assert node.get("type") == "priority"
    merges = {(int(c.get("fromLane")), int(c.get("toLane"))) for c in net_root.findall("connection")
              if c.get("from") == "EB3a" and c.get("to") == "EB3b"}
    assert merges == {(0, 0), (1, 0)}, "both lanes must merge into the single lane"


# DZ 3.3 left-turn bay lengths (m)
BAYS = {
    ("I1", "EB"): 75, ("I1", "WB"): 75, ("I2", "EB"): 60, ("I2", "WB"): 60,
    ("I3", "EB"): 40, ("I3", "WB"): 60, ("I4", "EB"): 50, ("I4", "WB"): 60,
    ("I1", "SB"): 45, ("I1", "NB"): 45, ("I3", "SB"): 30, ("I3", "NB"): 30,
}


@pytest.mark.parametrize("key", sorted(BAYS))
def test_left_bays(key, spec, net_lanes):
    i, a = key
    edge = approach_edge(i, a, spec)
    left_lanes = {fl for fl, _ in movement_lanes(i, a, "L", spec)}
    through_lanes = {fl for fl, _ in movement_lanes(i, a, "T", spec)}
    assert left_lanes and not (left_lanes & through_lanes), "left turns need an exclusive bay lane"
    assert net_lanes[edge][max(left_lanes)] == pytest.approx(BAYS[key])


def test_lane_groups(spec):
    # DZ 3.3: I4 EB has a single through lane; I2 side streets are one shared lane;
    # I4 side streets are TH/RT only.
    assert {fl for fl, _ in movement_lanes("I4", "EB", "T", spec)} == {0}
    for a in ("NB", "SB"):
        assert movement_lanes("I2", a, "L", spec) and {fl for fl, _ in movement_lanes("I2", a, "L", spec)} == {0}
        assert {fl for fl, _ in movement_lanes("I2", a, "T", spec)} == {0}
        assert movement_lanes("I4", a, "L", spec) == []
    for i in ("I1", "I2", "I3"):
        assert {fl for fl, _ in movement_lanes(i, "EB", "T", spec)} == {0, 1}


def test_signalized_connections_match_movements(spec, net_connections):
    expected = set()
    for i in INTERSECTIONS:
        for a in APPROACHES:
            for t in TURNS:
                for fl, tl in movement_lanes(i, a, t, spec):
                    expected.add((approach_edge(i, a, spec), departure_edge(i, DEPARTURE_DIRECTION[(a, t)]), fl, tl))
    approach_edges = {approach_edge(i, a, spec) for i in INTERSECTIONS for a in APPROACHES}
    actual = {c for c in net_connections if c[0] in approach_edges}
    assert actual == expected


OPPOSITE = {"EB": "WB", "WB": "EB", "NB": "SB", "SB": "NB"}


def test_no_u_turns(spec, net_connections):
    for i in INTERSECTIONS:
        for a in APPROACHES:
            frm = approach_edge(i, a, spec)
            back = departure_edge(i, OPPOSITE[a])
            assert not any(c[0] == frm and c[1] == back for c in net_connections), (i, a)


def test_tls_link_counts(net_root, spec):
    expected = {}
    for i in INTERSECTIONS:
        expected[i] = sum(len(movement_lanes(i, a, t, spec)) for a in APPROACHES for t in TURNS)
    for tl in net_root.findall("tlLogic"):
        assert len(tl.find("phase").get("state")) == expected[tl.get("id")]
    assert {tl.get("id") for tl in net_root.findall("tlLogic")} == set(INTERSECTIONS)
