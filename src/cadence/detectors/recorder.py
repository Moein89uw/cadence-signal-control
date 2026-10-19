"""Simulated observations from SUMO, without faults (Milestone 1).

The recorder reads every detector once per tick, before the controllers
step, so a detector's on/off event and the controller's response carry the
same tick. It produces:

* detector on/off events per cabinet (truth at 0.1 s);
* pedestrian push-button presses per cabinet;
* I3 radar tracks: per sample, track id, approach, lane, distance to the
  stop line and speed, for every vehicle inside the 120 m zone;
* I3 video turning-movement events: one record per vehicle leaving an I3
  approach, with its movement.

Vehicle identities from SUMO are kept only in `truth_track_vehicle`, for
tests; observation products never contain them. Faults are a later
increment: they will transform these products, not the simulation.
"""

from __future__ import annotations

from array import array
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

from cadence.detectors.layout import Detector, RadarZone
from cadence.simulation.network import APPROACHES, DEPARTURE_DIRECTION, approach_edge, departure_edge


RADAR_APPROACHES = ("EB", "WB", "SB", "NB")
RADAR_DTYPE = np.dtype([("tick", "i8"), ("track", "i4"), ("approach", "U2"), ("lane", "i1"),
                        ("distance_m", "f4"), ("speed_mps", "f4")])


class RadarStore:
    """Radar samples in compact typed columns (about 30 bytes per sample)."""

    def __init__(self):
        self.tick, self.track = array("q"), array("i")
        self.approach, self.lane = array("b"), array("b")
        self.distance, self.speed = array("f"), array("f")

    def append(self, tick, track, approach, lane, distance, speed):
        self.tick.append(tick); self.track.append(track)
        self.approach.append(RADAR_APPROACHES.index(approach)); self.lane.append(lane)
        self.distance.append(distance); self.speed.append(speed)

    def __len__(self):
        return len(self.tick)

    def truncate(self, n: int) -> None:
        for col in (self.tick, self.track, self.approach, self.lane, self.distance, self.speed):
            del col[n:]

    def to_array(self) -> np.ndarray:
        out = np.empty(len(self), dtype=RADAR_DTYPE)
        out["tick"] = np.frombuffer(self.tick, dtype="i8") if len(self) else []
        out["track"] = np.frombuffer(self.track, dtype="i4") if len(self) else []
        out["approach"] = np.array(RADAR_APPROACHES)[np.frombuffer(self.approach, dtype="i1")] if len(self) else []
        out["lane"] = np.frombuffer(self.lane, dtype="i1") if len(self) else []
        out["distance_m"] = np.frombuffer(self.distance, dtype="f4") if len(self) else []
        out["speed_mps"] = np.frombuffer(self.speed, dtype="f4") if len(self) else []
        return out


@dataclass
class Observations:
    detector_events: dict[str, list[tuple[int, int, int]]] = field(default_factory=lambda: defaultdict(list))
    ped_presses: dict[str, list[tuple[int, int]]] = field(default_factory=lambda: defaultdict(list))
    radar: RadarStore = field(default_factory=RadarStore)
    tmc: list[tuple[int, str, str]] = field(default_factory=list)


class ObservationRecorder:
    def __init__(self, detectors: list[Detector], radars: list[RadarZone], spec: dict, start_tick: int):
        self.detectors = detectors
        self.radars = radars
        self.state = {d.id: False for d in detectors}
        self.by_id = {d.id: d for d in detectors}
        self.obs = Observations()
        cfg = spec["detection"]["I3"]
        self.radar_period = max(1, int(round(cfg["radar"]["period_s"] * 10)))
        self.start_tick = start_tick
        self.truth_track_vehicle: dict[int, str] = {}
        self._track_of: dict[str, int] = {}
        self._next_track = 1
        # I3 video turning-movement counts: watch the departure edges
        self.tmc_intersection = "I3"
        self._dep_edges = {departure_edge("I3", d): d for d in APPROACHES}
        self._approach_of_edge = {approach_edge("I3", a, spec): a for a in APPROACHES}
        self._turn_of = {(a, d): t for (a, t), d in DEPARTURE_DIRECTION.items()}
        self._seen_on_dep: dict[str, set[str]] = {e: set() for e in self._dep_edges}

    # ------------------------------------------------------------------ per tick
    def read(self, tick: int) -> dict[str, bool]:
        """Read all detectors at the start of `tick` and log changes."""
        import libsumo

        get = libsumo.lanearea.getLastStepVehicleNumber
        ev = self.obs.detector_events
        for d in self.detectors:
            on = get(d.id) > 0
            if on != self.state[d.id]:
                self.state[d.id] = on
                ev[d.intersection].append((tick, d.channel, 1 if on else 0))
        if (tick - self.start_tick) % self.radar_period == 0:
            self._sample_radar(tick)
        self._sample_tmc(tick)
        return self.state

    def ped_press(self, tick: int, intersection: str, ped_phase: str) -> None:
        self.obs.ped_presses[intersection].append((tick, int(ped_phase[1:])))

    def _sample_radar(self, tick: int) -> None:
        import libsumo

        for z in self.radars:
            for edge, lane, lo, hi, off in z.segments:
                for v in libsumo.lane.getLastStepVehicleIDs(f"{edge}_{lane}"):
                    pos = libsumo.vehicle.getLanePosition(v)
                    if pos < lo:
                        continue
                    track = self._track_of.get(v)
                    if track is None:
                        track = self._next_track
                        self._next_track += 1
                        self._track_of[v] = track
                        self.truth_track_vehicle[track] = v
                    self.obs.radar.append(tick, track, z.approach, lane, hi - pos + off,
                                          libsumo.vehicle.getSpeed(v))

    def _sample_tmc(self, tick: int) -> None:
        import libsumo

        for edge, d in self._dep_edges.items():
            now = set(libsumo.edge.getLastStepVehicleIDs(edge))
            for v in now - self._seen_on_dep[edge]:
                route = libsumo.vehicle.getRoute(v)
                idx = libsumo.vehicle.getRouteIndex(v)
                prev = route[idx - 1] if idx > 0 else None
                a = self._approach_of_edge.get(prev)
                if a is not None:
                    self.obs.tmc.append((tick, a, self._turn_of[(a, d)]))
            self._seen_on_dep[edge] = now


def phase_presence(state: dict[str, bool], actuation: dict[int, list[str]]) -> dict[int, bool]:
    return {p: any(state[i] for i in ids) for p, ids in actuation.items()}
