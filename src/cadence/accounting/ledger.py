"""Vehicle accounting with teleporting disabled.

Terms used throughout Cadence (SUMO's own names in brackets):

* scheduled: vehicles whose scheduled entry time has been reached.
* backlog: scheduled vehicles that could not yet enter because the entry lane
  is full; they wait outside the network [SUMO "pending"].
* inserted: vehicles that have entered the network [SUMO "departed"].
* in_network: inserted vehicles that have not yet left.
* queued: in-network vehicles that are halted (speed below 0.1 m/s, SUMO's
  halting threshold).
* exited: vehicles that reached the end of their route and left [SUMO
  "arrived"].

Identities that must hold at every tick:

    scheduled = backlog + inserted
    inserted  = in_network + exited
    queued   <= in_network

Vehicle-time in the system counts both the backlog and the vehicles in the
network, so a policy cannot look better by keeping vehicles out of the
network. Counts are measured after each SUMO step, at the step's end time.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

HALTING_SPEED_MPS = 0.1


@dataclass
class Snapshot:
    tick: int
    scheduled: int
    backlog: int
    inserted: int
    in_network: int
    queued: int      # -1 when not measured at this tick
    exited: int


@dataclass
class Ledger:
    """Per-tick counts plus cumulative vehicle-time."""
    route_departs: list[float]                      # scheduled entry times (s), sorted
    tick_s: float = 0.1
    snapshots: list[Snapshot] = field(default_factory=list)
    inserted: int = 0
    exited: int = 0
    vehicle_seconds: float = 0.0                    # backlog + in network
    network_vehicle_seconds: float = 0.0            # in network only
    queued_vehicle_seconds: float = 0.0
    violations: list[str] = field(default_factory=list)
    keep_every: int = 10       # keep one snapshot, and measure the queue, once per second
    _next_sched: int = 0

    def scheduled_by(self, t_s: float) -> int:
        # vehicles with depart <= t (small tolerance for float times written at 0.01 s)
        while self._next_sched < len(self.route_departs) and self.route_departs[self._next_sched] <= t_s + 1e-6:
            self._next_sched += 1
        return self._next_sched

    def observe(self, tick: int) -> Snapshot:
        """Record the state after the SUMO step that ended at `tick`."""
        import libsumo

        # SUMO inserts vehicles at the start of a step, so a vehicle is scheduled
        # for the step ending at `tick` if its depart time is at or before the
        # step's start time.
        step_start_s = (tick - 1) * self.tick_s
        self.inserted += libsumo.simulation.getDepartedNumber()
        self.exited += libsumo.simulation.getArrivedNumber()
        backlog = len(libsumo.simulation.getPendingVehicles())
        in_net = libsumo.vehicle.getIDCount()
        measure = tick % self.keep_every == 0
        queued = -1
        if measure:
            get = libsumo.vehicle.getSpeed
            queued = sum(1 for v in libsumo.vehicle.getIDList() if get(v) < HALTING_SPEED_MPS)
            self.queued_vehicle_seconds += queued * self.tick_s * self.keep_every
        snap = Snapshot(tick, self.scheduled_by(step_start_s), backlog, self.inserted, in_net, queued, self.exited)
        self._check(snap)
        self.vehicle_seconds += (backlog + in_net) * self.tick_s
        self.network_vehicle_seconds += in_net * self.tick_s
        if measure:
            self.snapshots.append(snap)
        self.last = snap
        return snap

    def _check(self, s: Snapshot) -> None:
        if s.scheduled != s.backlog + s.inserted:
            self.violations.append(f"t={s.tick}: scheduled {s.scheduled} != backlog {s.backlog} + inserted {s.inserted}")
        if s.inserted != s.in_network + s.exited:
            self.violations.append(f"t={s.tick}: inserted {s.inserted} != in network {s.in_network} + exited {s.exited}")
        if s.queued != -1 and not 0 <= s.queued <= s.in_network:
            self.violations.append(f"t={s.tick}: queued {s.queued} outside [0, {s.in_network}]")


def route_departs(route_file: Path) -> list[float]:
    times = [float(v.get("depart")) for v in ET.parse(route_file).getroot().iter("vehicle")]
    return sorted(times)


@dataclass(frozen=True)
class Trip:
    vid: str
    scheduled: float
    inserted: float
    exited: float | None


def read_tripinfo(path: Path, route_file: Path) -> dict[str, Trip]:
    """Per-vehicle times from SUMO's tripinfo output (with unfinished trips) and
    the route file: an independent record for cross-checking the ledger."""
    sched = {v.get("id"): float(v.get("depart")) for v in ET.parse(route_file).getroot().iter("vehicle")}
    trips: dict[str, Trip] = {}
    for t in ET.parse(path).getroot().iter("tripinfo"):
        vid = t.get("id")
        arrival = float(t.get("arrival"))
        trips[vid] = Trip(vid, sched[vid], float(t.get("depart")), arrival if arrival >= 0 else None)
    return trips


def vehicle_seconds_from_trips(trips: dict[str, Trip], scheduled: dict[str, float], t0: float, t1: float,
                               tick_s: float = 0.1) -> float:
    """System vehicle-time over (t0, t1] from per-vehicle records.

    SUMO conventions (measured): a vehicle is inserted at the start of the
    first step that begins at or after its scheduled time, and tripinfo
    stamps both insertion and exit with the start time of the step in which
    they happen. So a vehicle counts in the ledger snapshot taken after the
    step ending at tick k exactly when ceil(scheduled / dt) + 1 <= k and
    k <= exit / dt (k < exit / dt + 1)."""
    total = 0
    k0, k1 = int(round(t0 / tick_s)), int(round(t1 / tick_s))
    for vid, s in scheduled.items():
        trip = trips.get(vid)
        a = max(math.ceil(round(s / tick_s, 6)) + 1, k0 + 1)
        b = k1 + 1
        if trip is not None and trip.exited is not None:
            b = min(b, int(round(trip.exited / tick_s)) + 1)
        if b > a:
            total += b - a
    return total * tick_s
