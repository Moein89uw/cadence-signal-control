"""Helpers for driving the controller without SUMO."""

from __future__ import annotations

import numpy as np

from cadence.controllers.actuated import ActuatedCoordinatedController, signal_state
from cadence.controllers.phasing import junction_foes, link_phase_map, phase_defs, signal_links
from cadence.controllers.timing import load_plans
from cadence.paths import load_spec

_PLANS = None


def plans():
    global _PLANS
    if _PLANS is None:
        _PLANS = load_plans()
    return _PLANS


def make(intersection: str, scenario: str = "S1", start: int = 0) -> ActuatedCoordinatedController:
    family = load_spec()["intersections"][intersection]["family"]
    return ActuatedCoordinatedController(intersection, phase_defs(intersection), plans()[scenario][intersection], start,
                                         family=family)


def link_map(intersection: str):
    spec = load_spec()
    return link_phase_map(intersection, phase_defs(intersection, spec), signal_links(intersection, spec=spec))


def foes(intersection: str):
    return junction_foes(intersection)


def drive(c: ActuatedCoordinatedController, t0: int, t1: int, presence=None, peds=None):
    """Step the controller over [t0, t1). `presence(t, c)` returns {phase: bool};
    `peds(t, c)` returns pedestrian phases pressed at t. Returns state changes."""
    lm = link_map(c.intersection)
    changes, last = [], None
    for t in range(t0, t1):
        c.step(t, presence(t, c) if presence else {}, peds(t, c) if peds else ())
        s = signal_state(c, lm)
        if s != last:
            changes.append((t, s))
            last = s
    return changes


def events(c, code=None, param=None):
    return [(t, k, p) for t, k, p in c.events if (code is None or k == code) and (param is None or p == param)]


def first(c, code, param, after=-10**12):
    return next((t for t, k, p in c.events if k == code and p == param and t >= after), None)


class QueueTraffic:
    """Random vehicle queues per phase: Poisson arrivals, one departure every
    2 s while green, presence whenever a vehicle is waiting or discharging."""

    def __init__(self, c: ActuatedCoordinatedController, seed: int, rate_per_h: float = 300.0,
                 ped_rate_per_h: float = 60.0):
        self.rng = np.random.default_rng(seed)
        self.q = {p: 0 for p in c.ph}
        self.p_arr = rate_per_h / 36000.0
        self.p_ped = ped_rate_per_h / 36000.0
        self.next_dep = {p: 0 for p in c.ph}

    def presence(self, t, c):
        out = {}
        for p in c.ph:
            if self.rng.random() < self.p_arr:
                self.q[p] += 1
            if c.vehicle_display(p) == "G" and self.q[p] > 0 and t >= self.next_dep[p]:
                self.q[p] -= 1
                self.next_dep[p] = t + 20
            out[p] = self.q[p] > 0
        return out

    def peds(self, t, c):
        return [name for name in c.peds if self.rng.random() < self.p_ped]
