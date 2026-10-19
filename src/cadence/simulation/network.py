"""Build the Corridor R-1 SUMO network from networks/corridor_r1/spec.json.

The spec is the single source of truth. This module writes SUMO plain-XML
files (nodes, edges, lane-level connections) and runs netconvert. Lane-level
turning permissions are written explicitly rather than guessed, so every
movement in the network is one the spec allows.

Geometry conventions
--------------------
* x increases eastbound along Alder Avenue; y increases northbound.
* Approaches are named by travel direction: EB, WB, SB (arrives from the
  north leg), NB (arrives from the south leg).
* SUMO lane 0 is the rightmost lane. On bay edges the left-turn bay is the
  highest lane index.
* Edge lengths use SUMO's ``length`` override, so the usable storage of each
  link equals the spec value exactly (storage = length / jam spacing).
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import quoteattr

from cadence.paths import R1_DIR, load_spec, sumo_bin, sumo_env

INTERSECTIONS = ("I1", "I2", "I3", "I4")
APPROACHES = ("EB", "WB", "SB", "NB")
TURNS = ("L", "T", "R")

# Departure direction for each (approach, turn).
DEPARTURE_DIRECTION = {
    ("EB", "L"): "NB", ("EB", "T"): "EB", ("EB", "R"): "SB",
    ("WB", "L"): "SB", ("WB", "T"): "WB", ("WB", "R"): "NB",
    ("SB", "L"): "EB", ("SB", "T"): "SB", ("SB", "R"): "WB",
    ("NB", "L"): "WB", ("NB", "T"): "NB", ("NB", "R"): "EB",
}

GEOM_JUNCTION_OFFSET_M = 12.0  # visual spacing only; lengths are overridden


@dataclass(frozen=True)
class Movement:
    intersection: str
    approach: str
    turn: str

    @property
    def key(self) -> str:
        return f"{self.intersection}:{self.approach}{self.turn}"


def k_of(intersection: str) -> int:
    return INTERSECTIONS.index(intersection) + 1


def approach_edge(intersection: str, approach: str, spec: dict | None = None) -> str:
    """The last edge of an approach, whose downstream end is the stop line."""
    spec = spec or load_spec()
    k = k_of(intersection)
    if approach == "EB":
        return ["EB0b", "EB1b", "EB2b", "EB3c"][k - 1]
    if approach == "WB":
        return ["WB1b", "WB2b", "WB3b", "WB4b"][k - 1]
    bay = spec["cross_streets"][intersection]["bay_m"] > 0
    leg = "N" if approach == "SB" else "S"
    return f"{leg}{k}in_b" if bay else f"{leg}{k}in"


def departure_edge(intersection: str, direction: str) -> str:
    k = k_of(intersection)
    if direction == "EB":
        return ["EB1a", "EB2a", "EB3a", "EB4"][k - 1]
    if direction == "WB":
        return ["WB0", "WB1a", "WB2a", "WB3a"][k - 1]
    if direction == "NB":
        return f"N{k}out"
    if direction == "SB":
        return f"S{k}out"
    raise ValueError(direction)


def approach_edges_upstream(intersection: str, approach: str, spec: dict | None = None) -> list[str]:
    """All edges of an approach link, upstream to downstream (mainline from the
    previous intersection or entry; cross streets from the leg end)."""
    spec = spec or load_spec()
    if approach in ("EB", "WB"):
        for link in spec["mainline_links"][approach]:
            if link["to"] == intersection:
                return [e["id"] for e in link["edges"]]
        raise KeyError((intersection, approach))
    k = k_of(intersection)
    leg = "N" if approach == "SB" else "S"
    if spec["cross_streets"][intersection]["bay_m"] > 0:
        return [f"{leg}{k}in_a", f"{leg}{k}in_b"]
    return [f"{leg}{k}in"]


def movement_lanes(intersection: str, approach: str, turn: str, spec: dict | None = None) -> list[tuple[int, int]]:
    """(from_lane, to_lane) pairs for a movement at a signalized intersection.
    Empty list when the movement is prohibited."""
    spec = spec or load_spec()
    cs = spec["cross_streets"][intersection]
    if approach == "EB":
        if intersection == "I4":  # one through/right lane + left bay
            return {"T": [(0, 0)], "R": [(0, 0)], "L": [(1, 0)]}[turn]
        return {"T": [(0, 0), (1, 1)], "R": [(0, 0)], "L": [(2, 0)]}[turn]
    if approach == "WB":
        return {"T": [(0, 0), (1, 1)], "R": [(0, 0)], "L": [(2, 0)]}[turn]
    # cross-street approaches
    if cs.get("left_prohibited") and turn == "L":
        return []
    if cs["bay_m"] > 0:
        return {"T": [(0, 0)], "R": [(0, 0)], "L": [(1, 1)]}[turn]
    return {"T": [(0, 0)], "R": [(0, 0)], "L": [(0, 1)]}[turn]


def movements(spec: dict | None = None) -> list[Movement]:
    spec = spec or load_spec()
    out = []
    for i in INTERSECTIONS:
        for a in APPROACHES:
            for t in TURNS:
                if movement_lanes(i, a, t, spec):
                    out.append(Movement(i, a, t))
    return out


# ---------------------------------------------------------------------------
# Plain XML generation
# ---------------------------------------------------------------------------

def _nodes(spec: dict) -> list[tuple[str, float, float, str]]:
    xs = {i: spec["intersections"][i]["x"] for i in INTERSECTIONS}
    leg = spec["cross_streets"]["leg_length_m"]
    off = GEOM_JUNCTION_OFFSET_M
    nodes = [("W0", 0.0, 0.0, "priority"), ("E0", xs["I4"] + 600.0, 0.0, "priority")]
    for i in INTERSECTIONS:
        k = k_of(i)
        nodes.append((i, xs[i], 0.0, "traffic_light"))
        nodes.append((f"N{k}end", xs[i], leg + off, "priority"))
        nodes.append((f"S{k}end", xs[i], -(leg + off), "priority"))
        bay = spec["cross_streets"][i]["bay_m"]
        if bay > 0:
            nodes.append((f"BN{k}", xs[i], bay + off, "priority"))
            nodes.append((f"BS{k}", xs[i], -(bay + off), "priority"))
    # mainline bay-start nodes and the EB lane-drop node
    nodes += [
        ("B1E", xs["I1"] - 75.0 - off, 0.0, "priority"),
        ("B2E", xs["I2"] - 60.0 - off, 0.0, "priority"),
        ("B3E", xs["I3"] - 40.0 - off, 0.0, "priority"),
        ("D", xs["I4"] - 120.0 - off, 0.0, "priority"),
        ("B4E", xs["I4"] - 50.0 - off, 0.0, "priority"),
        ("B4W", xs["I4"] + 60.0 + off, 0.0, "priority"),
        ("B3W", xs["I3"] + 60.0 + off, 0.0, "priority"),
        ("B2W", xs["I2"] + 60.0 + off, 0.0, "priority"),
        ("B1W", xs["I1"] + 75.0 + off, 0.0, "priority"),
    ]
    return nodes


def _edges(spec: dict) -> list[dict]:
    v = spec["global"]["posted_speed_mps"]
    leg = spec["cross_streets"]["leg_length_m"]
    chain = {
        "EB": ["W0", "B1E", "I1", "B2E", "I2", "B3E", "I3", "D", "B4E", "I4", "E0"],
        "WB": ["E0", "B4W", "I4", "B3W", "I3", "B2W", "I2", "B1W", "I1", "W0"],
    }
    edges = []
    for direction in ("EB", "WB"):
        ids = [e for link in spec["mainline_links"][direction] for e in link["edges"]]
        nodes = chain[direction]
        assert len(nodes) == len(ids) + 1, (direction, nodes, ids)
        for n, e in enumerate(ids):
            edges.append({"id": e["id"], "from": nodes[n], "to": nodes[n + 1],
                          "numLanes": e["lanes"], "length": e["length_m"],
                          "speed": v, "priority": 3})
    for i in INTERSECTIONS:
        k = k_of(i)
        bay = spec["cross_streets"][i]["bay_m"]
        for legname, end in (("N", f"N{k}end"), ("S", f"S{k}end")):
            if bay > 0:
                mid = f"B{legname}{k}"
                edges.append({"id": f"{legname}{k}in_a", "from": end, "to": mid, "numLanes": 1,
                              "length": leg - bay, "speed": v, "priority": 2})
                edges.append({"id": f"{legname}{k}in_b", "from": mid, "to": i, "numLanes": 2,
                              "length": bay, "speed": v, "priority": 2})
            else:
                edges.append({"id": f"{legname}{k}in", "from": end, "to": i, "numLanes": 1,
                              "length": leg, "speed": v, "priority": 2})
            edges.append({"id": f"{legname}{k}out", "from": i, "to": end, "numLanes": 1,
                          "length": leg, "speed": v, "priority": 2})
    return edges


def _connections(spec: dict) -> list[tuple[str, str, int, int]]:
    cons: list[tuple[str, str, int, int]] = []
    # signalized intersections
    for i in INTERSECTIONS:
        for a in APPROACHES:
            for t in TURNS:
                for fl, tl in movement_lanes(i, a, t, spec):
                    cons.append((approach_edge(i, a, spec), departure_edge(i, DEPARTURE_DIRECTION[(a, t)]), fl, tl))
    # mainline bay starts (2 -> 3 lanes): inner lane feeds both through and bay
    for a, b in (("EB0a", "EB0b"), ("EB1a", "EB1b"), ("EB2a", "EB2b"),
                 ("WB4a", "WB4b"), ("WB3a", "WB3b"), ("WB2a", "WB2b"), ("WB1a", "WB1b")):
        cons += [(a, b, 0, 0), (a, b, 1, 1), (a, b, 1, 2)]
    # EB lane drop (both lanes merge into one at a priority node) and the I4
    # left bay after it. A zipper node was tried first and produced SUMO
    # emergency braking at the merge; the priority merge does not.
    cons += [("EB3a", "EB3b", 0, 0), ("EB3a", "EB3b", 1, 0)]
    cons += [("EB3b", "EB3c", 0, 0), ("EB3b", "EB3c", 0, 1)]
    # cross-street bays (1 -> 2 lanes)
    for i in INTERSECTIONS:
        if spec["cross_streets"][i]["bay_m"] > 0:
            k = k_of(i)
            for leg in ("N", "S"):
                cons += [(f"{leg}{k}in_a", f"{leg}{k}in_b", 0, 0), (f"{leg}{k}in_a", f"{leg}{k}in_b", 0, 1)]
    return sorted(set(cons))


def write_plain_xml(spec: dict, outdir: Path) -> dict[str, Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    paths = {"nod": outdir / "r1.nod.xml", "edg": outdir / "r1.edg.xml", "con": outdir / "r1.con.xml"}
    with open(paths["nod"], "w", encoding="utf-8") as fh:
        fh.write("<nodes>\n")
        for nid, x, y, typ in _nodes(spec):
            fh.write(f'    <node id={quoteattr(nid)} x="{x:.2f}" y="{y:.2f}" type="{typ}"/>\n')
        fh.write("</nodes>\n")
    with open(paths["edg"], "w", encoding="utf-8") as fh:
        fh.write("<edges>\n")
        for e in _edges(spec):
            fh.write(
                f'    <edge id="{e["id"]}" from="{e["from"]}" to="{e["to"]}" numLanes="{e["numLanes"]}" '
                f'length="{e["length"]:.2f}" speed="{e["speed"]:.2f}" priority="{e["priority"]}"/>\n')
        fh.write("</edges>\n")
    with open(paths["con"], "w", encoding="utf-8") as fh:
        fh.write("<connections>\n")
        for frm, to, fl, tl in _connections(spec):
            fh.write(f'    <connection from="{frm}" to="{to}" fromLane="{fl}" toLane="{tl}"/>\n')
        fh.write("</connections>\n")
    return paths


def build_network(spec: dict | None = None, outdir: Path = R1_DIR) -> Path:
    """Write plain XML and run netconvert. Returns the path of r1.net.xml."""
    spec = spec or load_spec()
    outdir = Path(outdir)
    paths = write_plain_xml(spec, outdir)
    net = outdir / "r1.net.xml"
    # Relative file names keep the embedded netconvert configuration
    # machine-independent, so rebuilds reproduce the committed file.
    cmd = [
        sumo_bin("netconvert"),
        "--node-files", paths["nod"].name,
        "--edge-files", paths["edg"].name,
        "--connection-files", paths["con"].name,
        "--output-file", net.name,
        "--no-turnarounds", "true",
        "--offset.disable-normalization", "true",
        "--tls.default-type", "static",
        "--junctions.join", "false",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, cwd=outdir, env=sumo_env())
    if res.returncode != 0:
        raise RuntimeError(f"netconvert failed:\n{res.stdout}\n{res.stderr}")
    warnings = [ln for ln in (res.stdout + res.stderr).splitlines() if ln.startswith("Warning")]
    if warnings:
        raise RuntimeError("netconvert warnings (treated as errors):\n" + "\n".join(warnings))
    return net


if __name__ == "__main__":
    print(build_network())
