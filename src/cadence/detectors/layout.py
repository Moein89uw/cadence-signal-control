"""Detector layout for Corridor R-1 (Document Zero section 3.6).

| Int. | Vehicle detection (DZ 3.6)                                            |
| ---- | --------------------------------------------------------------------- |
| I1   | stop-bar loops all lanes; advance loops EB/WB at 100 m; system loops midblock I1-I2 |
| I2   | stop-bar loops only                                                   |
| I3   | radar (advance 120 m + stop bar, trajectory-capable); video turning-movement counts |
| I4   | video presence detection, all approaches                              |

Every intersection also has pedestrian push buttons. Dimensions (zone and
loop lengths), the midblock position, radar sampling and count bins are
Milestone 1 choices recorded in spec.json under "detection".

Each detector belongs to the cabinet (intersection) that logs it and has a
channel number there. Stop-bar detectors are also the controller's
actuation inputs: each one feeds exactly one phase.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from cadence.controllers.phasing import load_controller_config, phase_defs, phase_lanes
from cadence.paths import load_spec
from cadence.simulation.network import APPROACHES, INTERSECTIONS, approach_edge, approach_edges_upstream

KIND_ORDER = ("stopbar_loop", "radar_stopbar", "video_presence", "advance_loop", "system_loop")
STOPBAR_KINDS = ("stopbar_loop", "radar_stopbar", "video_presence")


@dataclass(frozen=True)
class Detector:
    id: str
    intersection: str   # cabinet that logs the detector
    channel: int
    kind: str
    approach: str       # travel direction of the lane it sits on
    edge: str
    lane: int
    start_m: float      # position on the lane, from the lane's upstream end
    end_m: float
    distance_m: float   # distance from its downstream end to the stop line it serves
    phase: int | None   # phase it calls (stop-bar detectors only)

    @property
    def lane_id(self) -> str:
        return f"{self.edge}_{self.lane}"


@dataclass(frozen=True)
class RadarZone:
    intersection: str
    approach: str
    range_m: float
    segments: tuple[tuple[str, int, float, float, float], ...]
    # (edge, lane, from_m, to_m, distance of the lane's downstream end to the stop line)


def _lane_lengths(net_lanes: dict[str, list[float]], edge: str) -> list[float]:
    return net_lanes[edge]


def locate_upstream(intersection: str, approach: str, distance_m: float, net_lanes, spec) -> tuple[str, float]:
    """Edge and position (from the edge start) that lies `distance_m` upstream
    of the stop line of an approach, measured along the modeled lanes."""
    remaining = distance_m
    for edge in reversed(approach_edges_upstream(intersection, approach, spec)):
        length = net_lanes[edge][0]
        if remaining <= length:
            return edge, length - remaining
        remaining -= length
    raise ValueError(f"{intersection} {approach}: {distance_m} m is beyond the approach link")


def _edge_offsets(intersection: str, approach: str, net_lanes, spec) -> dict[str, float]:
    """Distance from each approach edge's downstream end to the stop line."""
    out, acc = {}, 0.0
    for edge in reversed(approach_edges_upstream(intersection, approach, spec)):
        out[edge] = acc
        acc += net_lanes[edge][0]
    return out


def build_layout(net_lanes: dict[str, list[float]], spec: dict | None = None, ctrl: dict | None = None
                 ) -> tuple[list[Detector], list[RadarZone]]:
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    det_cfg = spec["detection"]
    zone = det_cfg["stopbar_zone_m"]
    loop = det_cfg["point_loop_m"]
    raw: list[dict] = []
    radars: list[RadarZone] = []
    for i in INTERSECTIONS:
        cfg = det_cfg[i]
        defs = phase_defs(i, spec, ctrl)
        lane_phase = {}
        for p, d in defs.items():
            for e, l in phase_lanes(i, d, spec):
                lane_phase[(e, l)] = p
        stop_kind = cfg["stopbar"]
        for a in APPROACHES:
            e = approach_edge(i, a, spec)
            for l, length in enumerate(net_lanes[e]):
                raw.append(dict(intersection=i, kind=stop_kind, approach=a, edge=e, lane=l,
                                start_m=max(0.0, length - zone), end_m=length, distance_m=0.0,
                                phase=lane_phase[(e, l)]))
        for a in cfg.get("advance_loops", {}).get("approaches", []):
            d = cfg["advance_loops"]["distance_m"]
            e, pos = locate_upstream(i, a, d, net_lanes, spec)
            for l in range(len(net_lanes[e])):
                raw.append(dict(intersection=i, kind="advance_loop", approach=a, edge=e, lane=l,
                                start_m=pos - loop, end_m=pos, distance_m=d, phase=None))
        if "system_loops" in cfg:
            sl = cfg["system_loops"]
            for a, downstream in (("EB", sl["between"][1]), ("WB", sl["between"][0])):
                edges = approach_edges_upstream(downstream, a, spec)
                link_len = sum(net_lanes[x][0] for x in edges)
                d = link_len * sl["fraction_from_downstream"]
                e, pos = locate_upstream(downstream, a, d, net_lanes, spec)
                for l in range(len(net_lanes[e])):
                    raw.append(dict(intersection=i, kind="system_loop", approach=a, edge=e, lane=l,
                                    start_m=pos - loop, end_m=pos, distance_m=d, phase=None))
        if "radar" in cfg:
            rng = cfg["radar"]["range_m"]
            for a in cfg["radar"]["approaches"]:
                offs = _edge_offsets(i, a, net_lanes, spec)
                segs = []
                for e, off in offs.items():
                    length = net_lanes[e][0]
                    if off >= rng:
                        continue
                    lo = max(0.0, length - (rng - off))
                    for l in range(len(net_lanes[e])):
                        segs.append((e, l, lo, length, off))
                radars.append(RadarZone(i, a, rng, tuple(segs)))
    detectors = []
    for i in INTERSECTIONS:
        mine = [r for r in raw if r["intersection"] == i]
        mine.sort(key=lambda r: (KIND_ORDER.index(r["kind"]), APPROACHES.index(r["approach"]), r["distance_m"], r["lane"]))
        for ch, r in enumerate(mine, start=1):
            did = f"{r['kind']}_{i}_{r['approach']}_{r['edge']}_{r['lane']}"
            detectors.append(Detector(id=did, channel=ch, **r))
    return detectors, radars


def actuation_map(detectors: list[Detector]) -> dict[str, dict[int, list[str]]]:
    """{intersection: {phase: [stop-bar detector ids]}} for every phase."""
    out: dict[str, dict[int, list[str]]] = {i: {} for i in INTERSECTIONS}
    for d in detectors:
        if d.kind in STOPBAR_KINDS:
            out[d.intersection].setdefault(d.phase, []).append(d.id)
    return out


def write_additional(path: Path, detectors: list[Detector]) -> Path:
    lines = ["<additional>"]
    for d in detectors:
        lines.append(f'    <laneAreaDetector id="{d.id}" lane="{d.lane_id}" pos="{d.start_m:.2f}" '
                     f'endPos="{d.end_m:.2f}" period="86400" file="NUL"/>')
    lines.append("</additional>")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return Path(path)
