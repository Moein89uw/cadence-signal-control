"""Agency-format high-resolution logs and observation products.

Document Zero 3.5: Family A controllers (I1, I3) log enumerated events at
0.1 s; Family B controllers (I2, I4) log phase state and detector on/off at
1 s. Logs are lists of (time_s, code, parameter) in the Indiana
high-resolution enumeration. Within a tick, detector events come before
the controller events they cause.
"""

from __future__ import annotations

import csv
from collections import Counter
from pathlib import Path

import numpy as np

from cadence.controllers import actuated as A

EV_DETECTOR_OFF = 81
EV_DETECTOR_ON = 82
EV_PED_DETECTOR_OFF = 89
EV_PED_DETECTOR_ON = 90

FAMILY_B_CODES = frozenset({
    A.EV_PHASE_BEGIN_GREEN, A.EV_PHASE_BEGIN_YELLOW, A.EV_PHASE_BEGIN_RED_CLEAR, A.EV_PHASE_END_RED_CLEAR,
    A.EV_PED_BEGIN_WALK, A.EV_PED_BEGIN_CLEAR, A.EV_PED_BEGIN_DONT_WALK,
    EV_DETECTOR_OFF, EV_DETECTOR_ON, EV_PED_DETECTOR_OFF, EV_PED_DETECTOR_ON,
})


def hires_log(family: str, controller_events, detector_events, ped_presses) -> list[tuple[float, int, int]]:
    rows = []
    seq = 0
    for t, ch, on in detector_events:
        rows.append((t, 0, seq, EV_DETECTOR_ON if on else EV_DETECTOR_OFF, ch)); seq += 1
    for t, n in ped_presses:
        rows.append((t, 0, seq, EV_PED_DETECTOR_ON, n)); seq += 1
        rows.append((t + 1, 0, seq, EV_PED_DETECTOR_OFF, n)); seq += 1
    for t, code, p in controller_events:
        rows.append((t, 1, seq, code, p)); seq += 1
    rows.sort()
    if family == "A":
        return [(round(t / 10, 1), code, p) for t, _, _, code, p in rows]
    if family == "B":
        return [(t // 10, code, p) for t, _, _, code, p in rows if code in FAMILY_B_CODES]
    raise ValueError(family)


def write_log_csv(path: Path, rows) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "code", "param"])
        w.writerows(rows)
    return path


def tmc_counts(tmc, bin_s: int, begin_tick: int) -> dict[tuple[int, str, str], int]:
    """Turning-movement counts per bin: {(bin start time s, approach, turn): count}."""
    out: Counter = Counter()
    for t, a, turn in tmc:
        b = begin_tick // 10 + ((t - begin_tick) // 10 // bin_s) * bin_s
        out[(b, a, turn)] += 1
    return dict(out)


def radar_array(radar) -> np.ndarray:
    """Radar samples as a structured array (tick, track, approach, lane,
    distance_m, speed_mps)."""
    return radar.to_array()


def detector_counts(detector_events, bin_s: int, begin_tick: int) -> dict[tuple[int, int], int]:
    """Vehicle actuations (on events) per channel per bin."""
    out: Counter = Counter()
    for t, ch, on in detector_events:
        if on:
            out[(begin_tick // 10 + ((t - begin_tick) // 10 // bin_s) * bin_s, ch)] += 1
    return dict(out)
