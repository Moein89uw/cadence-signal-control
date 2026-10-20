"""Test-only signal overrides (picklable, so checkpoints that use them can be
resumed in another process). They bypass the controller and its safety
checks, and exist only to create stress conditions such as a backlog."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HoldRed:
    """Show red on every link of one approach for a time window."""
    intersection: str
    approach: str
    start_s: float
    duration_s: float

    def __call__(self, intersection: str, tick: int, state: str, sim) -> str:
        if intersection != self.intersection or not (self.start_s * 10 <= tick < (self.start_s + self.duration_s) * 10):
            return state
        links = sim.signal_link_index[intersection][self.approach]
        return "".join("r" if k in links else ch for k, ch in enumerate(state))
