"""NEMA phase assignment for Corridor R-1 and its mapping to SUMO signal links.

Phases follow the standard dual-ring convention with phase 2 eastbound:

    ring 1:  1 (WB L)  2 (EB T+R)  |  3 (NB L)  4 (SB T+R)
    ring 2:  5 (EB L)  6 (WB T+R)  |  7 (SB L)  8 (NB T+R)

The main group (1, 2, 5, 6) and the side group (3, 4, 7, 8) are separated
by barriers. At 6-phase intersections (I2, I4) phases 3 and 7 do not exist;
at I2 the single shared side-street lane gives permissive side lefts in
phases 4 and 8, and at I4 side-street lefts are prohibited.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from cadence.paths import R1_DIR, REPO_ROOT, load_spec
from cadence.simulation.network import (
    APPROACHES, DEPARTURE_DIRECTION, INTERSECTIONS, approach_edge, departure_edge, movement_lanes,
)

CTRL_PATH = REPO_ROOT / "configs" / "controller_r1.json"
MAIN, SIDE = "main", "side"
GROUPS = (MAIN, SIDE)
COORDINATED = (2, 6)

# (approach, turn) protected movements of each phase before lane availability
_PHASE_MOVEMENTS = {
    1: (("WB", "L"),),
    2: (("EB", "T"), ("EB", "R")),
    3: (("NB", "L"),),
    4: (("SB", "T"), ("SB", "R")),
    5: (("EB", "L"),),
    6: (("WB", "T"), ("WB", "R")),
    7: (("SB", "L"),),
    8: (("NB", "T"), ("NB", "R")),
}
_RING = {1: 1, 2: 1, 3: 1, 4: 1, 5: 2, 6: 2, 7: 2, 8: 2}
_GROUP = {1: MAIN, 2: MAIN, 5: MAIN, 6: MAIN, 3: SIDE, 4: SIDE, 7: SIDE, 8: SIDE}


def load_controller_config(path: Path = CTRL_PATH) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


@dataclass(frozen=True)
class PhaseDef:
    number: int
    ring: int
    group: str
    movements: tuple[tuple[str, str], ...]   # protected (approach, turn)
    permissive: tuple[tuple[str, str], ...]  # permissive (yielding) movements
    coordinated: bool
    min_green_s: float
    yellow_s: float
    red_clear_s: float
    ped: str | None = None
    walk_s: float = 0.0
    ped_clear_s: float = 0.0
    lpi_s: float = 0.0

    @property
    def is_left(self) -> bool:
        return self.number in (1, 3, 5, 7)

    @property
    def clearance_s(self) -> float:
        return self.yellow_s + self.red_clear_s


def phase_numbers(intersection: str, spec: dict | None = None) -> tuple[int, ...]:
    spec = spec or load_spec()
    if spec["intersections"][intersection]["phasing"] == "8-phase":
        return (1, 2, 3, 4, 5, 6, 7, 8)
    return (1, 2, 4, 5, 6, 8)


def phase_defs(intersection: str, spec: dict | None = None, ctrl: dict | None = None) -> dict[int, PhaseDef]:
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    tc = spec["timing_constants"]
    eight = spec["intersections"][intersection]["phasing"] == "8-phase"
    peds = {v["vehicle_phase"]: (name, v["crossing"]) for name, v in ctrl["pedestrian_phases"].items()}
    lpi_peds = set(ctrl.get("lpi_ped_phases", {}).get(intersection, []))
    out: dict[int, PhaseDef] = {}
    for p in phase_numbers(intersection, spec):
        protected = tuple(m for m in _PHASE_MOVEMENTS[p] if movement_lanes(intersection, *m, spec))
        permissive: tuple[tuple[str, str], ...] = ()
        if not eight and p in (4, 8):
            side_left = ("SB", "L") if p == 4 else ("NB", "L")
            if movement_lanes(intersection, *side_left, spec):
                permissive = (side_left,)
        mainline = p in (1, 2, 5, 6)
        if p in COORDINATED:
            min_green = tc["min_green_through_coordinated_s"]
        elif p in (1, 3, 5, 7):
            min_green = tc["min_green_left_s"]
        else:
            min_green = tc["min_green_side_s"]
        ped_name, walk, clear, lpi = None, 0.0, 0.0, 0.0
        if p in peds:
            ped_name, crossing = peds[p]
            walk = tc["walk_s"]
            clear = tc["ped_clear_mainline_crossing_s"] if crossing == "mainline" else tc["ped_clear_side_crossing_s"]
            lpi = tc["lpi_s"] if ped_name in lpi_peds else 0.0
        out[p] = PhaseDef(
            number=p, ring=_RING[p], group=_GROUP[p], movements=protected, permissive=permissive,
            coordinated=p in COORDINATED, min_green_s=min_green,
            yellow_s=tc["yellow_mainline_s"] if mainline else tc["yellow_side_s"],
            red_clear_s=tc["all_red_mainline_s"] if mainline else tc["all_red_side_s"],
            ped=ped_name, walk_s=walk, ped_clear_s=clear, lpi_s=lpi,
        )
    return out


def ring_sequences(intersection: str, spec: dict | None = None, ctrl: dict | None = None) -> dict[str, dict[int, list[int]]]:
    """{group: {ring: [phase numbers in service order]}} for phases that exist."""
    ctrl = ctrl or load_controller_config()
    present = set(phase_numbers(intersection, spec))
    out: dict[str, dict[int, list[int]]] = {MAIN: {}, SIDE: {}}
    for ring, groups in ctrl["rings"].items():
        for g in GROUPS:
            out[g][int(ring)] = [p for p in groups[g] if p in present]
    return out


def phase_lanes(intersection: str, phase: PhaseDef, spec: dict | None = None) -> list[tuple[str, int]]:
    """(edge, lane index) of every approach lane that feeds the phase."""
    spec = spec or load_spec()
    lanes: set[tuple[str, int]] = set()
    for a, t in phase.movements + phase.permissive:
        edge = approach_edge(intersection, a, spec)
        for fl, _ in movement_lanes(intersection, a, t, spec):
            lanes.add((edge, fl))
    return sorted(lanes)


# ---------------------------------------------------------------------------
# SUMO signal links
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SignalLink:
    index: int
    from_edge: str
    to_edge: str
    from_lane: int
    approach: str
    turn: str


def signal_links(intersection: str, net_path: Path = R1_DIR / "r1.net.xml", spec: dict | None = None) -> list[SignalLink]:
    """Signal links of one traffic light, ordered by SUMO link index."""
    spec = spec or load_spec()
    by_edge = {approach_edge(intersection, a, spec): a for a in APPROACHES}
    dep = {}
    for (a, t), d in DEPARTURE_DIRECTION.items():
        dep[(a, departure_edge(intersection, d))] = t
    links = []
    for c in ET.parse(net_path).getroot().findall("connection"):
        if c.get("tl") != intersection:
            continue
        a = by_edge[c.get("from")]
        links.append(SignalLink(int(c.get("linkIndex")), c.get("from"), c.get("to"),
                                int(c.get("fromLane")), a, dep[(a, c.get("to"))]))
    links.sort(key=lambda l: l.index)
    assert [l.index for l in links] == list(range(len(links)))
    return links


def link_phase_map(intersection: str, phases: dict[int, PhaseDef], links: list[SignalLink]) -> list[tuple[int, bool]]:
    """For each signal link: (controlling phase, permissive?)."""
    owner: dict[tuple[str, str], tuple[int, bool]] = {}
    for p, d in phases.items():
        for m in d.movements:
            owner[m] = (p, False)
        for m in d.permissive:
            owner[m] = (p, True)
    out = []
    for l in links:
        if (l.approach, l.turn) not in owner:
            raise ValueError(f"{intersection}: link {l.index} ({l.approach} {l.turn}) has no phase")
        out.append(owner[(l.approach, l.turn)])
    return out


def junction_foes(intersection: str, net_path: Path = R1_DIR / "r1.net.xml") -> list[set[int]]:
    """SUMO's geometric conflict matrix: foes[i] is the set of links that
    conflict with link i. In the net file the rightmost character of a foes
    string is link 0."""
    root = ET.parse(net_path).getroot()
    j = next(x for x in root.findall("junction") if x.get("id") == intersection)
    reqs = sorted(j.findall("request"), key=lambda r: int(r.get("index")))
    out = []
    for r in reqs:
        s = r.get("foes")
        out.append({k for k, ch in enumerate(reversed(s)) if ch == "1"})
    return out


def all_signal_maps(spec: dict | None = None, ctrl: dict | None = None,
                    net_path: Path = R1_DIR / "r1.net.xml") -> dict[str, list[tuple[int, bool]]]:
    spec = spec or load_spec()
    return {i: link_phase_map(i, phase_defs(i, spec, ctrl), signal_links(i, net_path, spec)) for i in INTERSECTIONS}
