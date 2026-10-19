"""Stop-bar presence zones that place calls on the controller's phases.

Every Document Zero intersection has stop-bar presence detection (loops at
I1 and I2, the radar zone at I3, video zones at I4), so the controller is
actuated the same way everywhere: one presence zone at the stop line of each
lane that feeds a non-coordinated phase. These zones are the controller's
own inputs. The observation layer used for estimation (advance loops,
system loops, radar trajectories, video counts, event-log resolution) is a
separate increment.
"""

from __future__ import annotations

from pathlib import Path

from cadence.controllers.phasing import load_controller_config, phase_defs, phase_lanes
from cadence.paths import load_spec
from cadence.simulation.network import INTERSECTIONS


def zone_id(intersection: str, edge: str, lane: int) -> str:
    return f"act_{intersection}_{edge}_{lane}"


def actuation_zones(spec: dict | None = None, ctrl: dict | None = None) -> dict[str, dict[int, list[str]]]:
    """{intersection: {phase: [zone ids]}} for non-coordinated phases."""
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    out: dict[str, dict[int, list[str]]] = {}
    for i in INTERSECTIONS:
        out[i] = {}
        for p, d in phase_defs(i, spec, ctrl).items():
            if d.coordinated:
                continue
            out[i][p] = [zone_id(i, e, l) for e, l in phase_lanes(i, d, spec)]
    return out


def write_actuation_additional(path: Path, net_lanes: dict[str, list[float]],
                               spec: dict | None = None, ctrl: dict | None = None) -> Path:
    """Write SUMO lane-area detectors for every actuation zone. `net_lanes`
    maps edge id to lane lengths."""
    spec = spec or load_spec()
    ctrl = ctrl or load_controller_config()
    zone = ctrl["actuation"]["presence_zone_m"]
    seen = set()
    lines = ["<additional>"]
    for i in INTERSECTIONS:
        for p, d in phase_defs(i, spec, ctrl).items():
            if d.coordinated:
                continue
            for e, l in phase_lanes(i, d, spec):
                zid = zone_id(i, e, l)
                if zid in seen:
                    continue
                seen.add(zid)
                length = net_lanes[e][l]
                start = max(0.0, length - zone)
                lines.append(f'    <laneAreaDetector id="{zid}" lane="{e}_{l}" pos="{start:.2f}" '
                             f'endPos="{length:.2f}" period="86400" file="NUL"/>')
    lines.append("</additional>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
