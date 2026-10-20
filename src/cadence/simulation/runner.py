"""Run Corridor R-1 in SUMO with the Cadence actuated-coordinated controller.

The harness uses libsumo (in-process SUMO). Each tick of 0.1 s it:

1. reads every detector (the observation layer logs on/off changes);
2. derives each phase's call from its stop-bar detectors;
3. steps every controller and writes signal states that changed;
4. advances SUMO by one step.

Teleporting is disabled so that every inserted vehicle is accounted for;
collisions are detected and recorded.

Only one libsumo simulation can be open per process.
"""

from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from cadence.controllers.actuated import TICKS_PER_S, ActuatedCoordinatedController, build_controllers, signal_state
from cadence.controllers.phasing import all_signal_maps, load_controller_config
from cadence.controllers.timing import load_plans
from cadence.detectors.layout import actuation_map, build_layout, write_additional
from cadence.detectors.logs import hires_log, radar_array, tmc_counts, write_log_csv
from cadence.detectors.recorder import ObservationRecorder, phase_presence
from cadence.paths import R1_DIR, load_spec, sumo_bin, sumo_home
from cadence.simulation import demand as dm
from cadence.simulation.network import INTERSECTIONS


def sumo_seed(scenario: str, replication: int, spec: dict) -> int:
    """SUMO's own random seed (driver imperfection), derived from the master
    seed with a spawn key separate from every demand stream."""
    ss = np.random.SeedSequence(entropy=spec["scenarios"]["master_seed"],
                                spawn_key=(dm.SCENARIOS.index(scenario), replication, 9, 0))
    return int(ss.generate_state(1)[0] % (2**31 - 1))


def net_lane_lengths(net_path: Path) -> dict[str, list[float]]:
    out = {}
    for e in ET.parse(net_path).getroot().findall("edge"):
        if e.get("function") == "internal":
            continue
        lanes = sorted(e.findall("lane"), key=lambda l: int(l.get("index")))
        out[e.get("id")] = [float(l.get("length")) for l in lanes]
    return out


def lane_predecessors(net_path: Path) -> tuple[dict[str, str], dict[str, float]]:
    """({internal lane: the lane before it}, {lane: length}) for every lane,
    internal lanes included, so a vehicle's body can be traced back from its
    front through junction lanes to the lanes its rear still occupies."""
    root = ET.parse(net_path).getroot()
    prev = {}
    for c in root.findall("connection"):
        via = c.get("via")
        if via:
            prev[via] = f"{c.get('from')}_{c.get('fromLane')}"
    lengths = {l.get("id"): float(l.get("length")) for e in root.findall("edge") for l in e.findall("lane")}
    return prev, lengths


@dataclass
class RunRecord:
    """What a run produced, for tests and reports."""
    scenario: str
    replication: int
    begin_tick: int
    end_tick: int = 0
    state_changes: dict[str, list[tuple[int, str]]] = field(default_factory=lambda: defaultdict(list))
    collisions: list[tuple[int, str, str, str]] = field(default_factory=list)
    teleports: int = 0
    emergency_stops: int = 0
    arrived: int = 0
    departed: int = 0
    warnings: dict[str, list[str]] = field(default_factory=dict)


class CorridorSimulation:
    def __init__(self, scenario: str, replication: int, workdir: Path, begin_s: float | None = None,
                 end_s: float | None = None, spec: dict | None = None, ctrl_cfg: dict | None = None,
                 plans: dict | None = None, net_path: Path = R1_DIR / "r1.net.xml",
                 extra_sumo_args: tuple[str, ...] = (), presence_audit_every: int = 0):
        self.spec = spec or load_spec()
        self.ctrl_cfg = ctrl_cfg or load_controller_config()
        self.plans = plans or load_plans(spec=self.spec, ctrl=self.ctrl_cfg)
        self.scenario, self.replication = scenario, replication
        self.workdir = Path(workdir)
        self.workdir.mkdir(parents=True, exist_ok=True)
        w = dm.scenario_window(scenario, self.spec)
        self.begin_s = float(w.start if begin_s is None else begin_s)
        self.end_s = float(w.end if end_s is None else end_s)
        self.net_path = net_path
        self.extra_sumo_args = tuple(extra_sumo_args)
        self.step_length = self.ctrl_cfg["step_length_s"]
        assert abs(self.step_length * TICKS_PER_S - 1.0) < 1e-9, "controller ticks must equal the SUMO step"
        self.tick = int(round(self.begin_s * TICKS_PER_S))
        self.record = RunRecord(scenario, replication, self.tick)
        self.controllers: dict[str, ActuatedCoordinatedController] = {}
        self.presence_audit_every = presence_audit_every
        self.presence_audit = {"samples": 0, "mismatches": [], "ambiguous": 0, "vehicles_seen": defaultdict(set)}
        self._started = False

    # ------------------------------------------------------------------ setup
    def prepare(self) -> dict[str, Path]:
        files = dm.generate(self.scenario, self.replication, self.workdir, self.spec)
        self.net_lanes = net_lane_lengths(self.net_path)
        self.detectors, self.radars = build_layout(self.net_lanes, self.spec, self.ctrl_cfg)
        files["detectors"] = write_additional(self.workdir / "detectors.add.xml", self.detectors)
        return files

    def sumo_command(self, files: dict[str, Path]) -> list[str]:
        return [
            sumo_bin("sumo"), "-n", str(self.net_path), "-r", str(files["routes"]),
            "-a", str(files["detectors"]),
            "--begin", f"{self.begin_s:.1f}", "--end", f"{self.end_s:.1f}",
            "--step-length", f"{self.step_length}",
            "--seed", str(sumo_seed(self.scenario, self.replication, self.spec)),
            "--time-to-teleport", "-1",
            "--collision.action", "warn", "--collision.check-junctions", "true",
            "--no-step-log", "true", "--duration-log.disable", "true",
            "--error-log", str(self.workdir / "sumo_warnings.log"),
            *self.extra_sumo_args,
        ]

    def start(self) -> None:
        import os

        import libsumo

        os.environ.setdefault("SUMO_HOME", str(sumo_home()))
        files = self.prepare()
        self.files = files
        libsumo.start(self.sumo_command(files))
        self._started = True
        self.controllers = build_controllers(self.scenario, self.tick, self.spec, self.ctrl_cfg, self.plans)
        self.signal_maps = all_signal_maps(self.spec, self.ctrl_cfg, self.net_path)
        self.actuation = actuation_map(self.detectors)
        self.recorder = ObservationRecorder(self.detectors, self.radars, self.spec, self.tick)
        self._last_state = {i: None for i in INTERSECTIONS}
        self._prev, self._lane_len = lane_predecessors(self.net_path) if self.presence_audit_every else ({}, {})
        calls = dm.read_ped_calls(files["ped_calls"])
        self._ped_queue: dict[int, list[tuple[str, str]]] = defaultdict(list)
        for t, i, p in calls:
            self._ped_queue[max(self.tick, math.ceil(round(t * TICKS_PER_S, 6)))].append((i, p))

    # ------------------------------------------------------------------ stepping
    def step(self) -> None:
        import libsumo

        t = self.tick
        state = self.recorder.read(t)
        if self.presence_audit_every:
            self._audit(t, state)
        presses: dict[str, list[str]] = defaultdict(list)
        for i, p in self._ped_queue.pop(t, ()):
            presses[i].append(p)
            self.recorder.ped_press(t, i, p)
        for i, c in self.controllers.items():
            c.step(t, phase_presence(state, self.actuation[i]), presses.get(i, ()))
            sig = signal_state(c, self.signal_maps[i])
            if sig != self._last_state[i]:
                libsumo.trafficlight.setRedYellowGreenState(i, sig)
                self._last_state[i] = sig
                self.record.state_changes[i].append((t, sig))
        libsumo.simulationStep()
        self.tick += 1
        rec = self.record
        for col in libsumo.simulation.getCollisions():
            rec.collisions.append((self.tick, col.collider, col.victim, col.type))
        rec.teleports += libsumo.simulation.getStartingTeleportNumber()
        rec.emergency_stops += libsumo.simulation.getEmergencyStoppingVehiclesNumber()
        rec.arrived += libsumo.simulation.getArrivedNumber()
        rec.departed += libsumo.simulation.getDepartedNumber()
        rec.end_tick = self.tick

    def _audit(self, t: int, state: dict[str, bool]) -> None:
        """Independent check of detector states from vehicle positions."""
        import libsumo

        audit = self.presence_audit
        for d in self.detectors:
            if d.kind in ("advance_loop", "system_loop"):
                audit["vehicles_seen"][d.id].update(libsumo.lanearea.getLastStepVehicleIDs(d.id))
        if (t - self.record.begin_tick) % self.presence_audit_every:
            return
        audit["samples"] += 1
        # (back, front, traced): traced spans are rears overhanging the lanes
        # behind a vehicle's front lane. Which lane a rear occupies is ambiguous
        # when the vehicle changed lanes just before the junction, so a detector
        # may count a traced rear or not; anything else must match exactly.
        spans: dict[str, list[tuple[float, float, bool]]] = defaultdict(list)
        for v in libsumo.vehicle.getIDList():
            lane = libsumo.vehicle.getLaneID(v)
            pos = libsumo.vehicle.getLanePosition(v)
            length = libsumo.vehicle.getLength(v)
            spans[lane].append((pos - length, pos, False))
            rest, cur = length - pos, lane
            while rest > 0 and cur in self._prev:
                cur = self._prev[cur]
                L = self._lane_len[cur]
                spans[cur].append((L - rest, L, True))
                rest -= L
        for d in self.detectors:
            hits = [traced for back, front, traced in spans.get(d.lane_id, ()) if back < d.end_m and front > d.start_m]
            full = any(not tr for tr in hits)
            if state[d.id] == full or (state[d.id] and hits):
                if not state[d.id] and hits:
                    audit["ambiguous"] += 1
                continue
            audit["mismatches"].append((t, d.id, state[d.id], full))

    def run_until(self, end_s: float | None = None) -> RunRecord:
        end_tick = int(round((self.end_s if end_s is None else end_s) * TICKS_PER_S))
        while self.tick < end_tick:
            self.step()
        return self.record

    def events(self) -> dict[str, list[tuple[int, int, int]]]:
        return {i: list(c.events) for i, c in self.controllers.items()}

    # ------------------------------------------------------------------ outputs
    def write_observations(self) -> Path:
        out = self.workdir / "logs"
        out.mkdir(parents=True, exist_ok=True)
        obs = self.recorder.obs
        for i, c in self.controllers.items():
            fam = self.spec["intersections"][i]["family"]
            rows = hires_log(fam, c.events, obs.detector_events.get(i, []), obs.ped_presses.get(i, []))
            write_log_csv(out / f"{i}_hires.csv", rows)
        np.savez_compressed(out / "I3_radar.npz", radar=radar_array(obs.radar))
        with open(out / "I3_tmc.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["time_s", "approach", "turn"])
            w.writerows((round(t / 10, 1), a, turn) for t, a, turn in obs.tmc)
        bins = tmc_counts(obs.tmc, self.spec["detection"]["I3"]["video_tmc"]["bin_s"], self.record.begin_tick)
        with open(out / "I3_tmc_bins.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["bin_start_s", "approach", "turn", "count"])
            w.writerows((b, a, turn, n) for (b, a, turn), n in sorted(bins.items()))
        with open(out / "detectors.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["intersection", "channel", "kind", "approach", "lane_id", "start_m", "end_m",
                        "distance_to_stop_line_m", "phase"])
            for d in self.detectors:
                w.writerow([d.intersection, d.channel, d.kind, d.approach, d.lane_id, round(d.start_m, 2),
                            round(d.end_m, 2), round(d.distance_m, 2), d.phase if d.phase is not None else ""])
        return out

    def close(self) -> None:
        if self._started:
            import libsumo

            libsumo.close()
            self._started = False
            self.record.warnings = parse_sumo_warnings(self.workdir / "sumo_warnings.log")
            self.write_observations()

    def __enter__(self) -> "CorridorSimulation":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def parse_sumo_warnings(path: Path) -> dict[str, list[str]]:
    """Group SUMO warnings by kind (emergency braking, collision, teleport, other)."""
    out: dict[str, list[str]] = {"emergency_braking": [], "collision": [], "teleport": [], "other": []}
    if not Path(path).exists():
        return out
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        if "emergency braking" in line:
            out["emergency_braking"].append(line)
        elif "ollision" in line:
            out["collision"].append(line)
        elif "eleport" in line:
            out["teleport"].append(line)
        else:
            out["other"].append(line)
    return out
