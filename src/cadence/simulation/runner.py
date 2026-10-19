"""Run Corridor R-1 in SUMO with the Cadence actuated-coordinated controller.

The harness uses libsumo (in-process SUMO). Each tick of 0.1 s it reads the
actuation zones, steps every controller, writes signal states that changed,
and advances SUMO by one step. Teleporting is disabled so that every
inserted vehicle is accounted for; collisions are detected and recorded.

Only one libsumo simulation can be open per process.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from cadence.controllers.actuated import TICKS_PER_S, ActuatedCoordinatedController, build_controllers, signal_state
from cadence.controllers.actuation import actuation_zones, write_actuation_additional
from cadence.controllers.phasing import all_signal_maps, load_controller_config
from cadence.controllers.timing import load_plans
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
                 extra_sumo_args: tuple[str, ...] = ()):
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
        self._started = False

    # ------------------------------------------------------------------ setup
    def prepare(self) -> dict[str, Path]:
        files = dm.generate(self.scenario, self.replication, self.workdir, self.spec)
        files["actuation"] = write_actuation_additional(self.workdir / "actuation.add.xml",
                                                        net_lane_lengths(self.net_path), self.spec, self.ctrl_cfg)
        return files

    def sumo_command(self, files: dict[str, Path]) -> list[str]:
        return [
            sumo_bin("sumo"), "-n", str(self.net_path), "-r", str(files["routes"]),
            "-a", str(files["actuation"]),
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
        self.zones = actuation_zones(self.spec, self.ctrl_cfg)
        self._last_state = {i: None for i in INTERSECTIONS}
        calls = dm.read_ped_calls(files["ped_calls"])
        self._ped_queue: dict[int, list[tuple[str, str]]] = defaultdict(list)
        for t, i, p in calls:
            self._ped_queue[max(self.tick, math.ceil(round(t * TICKS_PER_S, 6)))].append((i, p))

    # ------------------------------------------------------------------ stepping
    def presence(self) -> dict[str, dict[int, bool]]:
        import libsumo

        get = libsumo.lanearea.getLastStepVehicleNumber
        return {i: {p: any(get(z) > 0 for z in zs) for p, zs in by_p.items()} for i, by_p in self.zones.items()}

    def step(self) -> None:
        import libsumo

        t = self.tick
        pres = self.presence()
        presses: dict[str, list[str]] = defaultdict(list)
        for i, p in self._ped_queue.pop(t, ()):
            presses[i].append(p)
        for i, c in self.controllers.items():
            c.step(t, pres[i], presses.get(i, ()))
            state = signal_state(c, self.signal_maps[i])
            if state != self._last_state[i]:
                libsumo.trafficlight.setRedYellowGreenState(i, state)
                self._last_state[i] = state
                self.record.state_changes[i].append((t, state))
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

    def run_until(self, end_s: float | None = None) -> RunRecord:
        end_tick = int(round((self.end_s if end_s is None else end_s) * TICKS_PER_S))
        while self.tick < end_tick:
            self.step()
        return self.record

    def events(self) -> dict[str, list[tuple[int, int, int]]]:
        return {i: list(c.events) for i, c in self.controllers.items()}

    def close(self) -> None:
        if self._started:
            import libsumo

            libsumo.close()
            self._started = False
            self.record.warnings = parse_sumo_warnings(self.workdir / "sumo_warnings.log")


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

    def __enter__(self) -> "CorridorSimulation":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()
