"""Increment 2 acceptance tests: controller behavior without SUMO.

Times are ticks of 0.1 s. Plans come from configs/timing_plans_r1.json.
For S1 at I1 (cycle 110 s, offset 0): main group 65 s, yield point at 59 s,
phase 3 force-off at 70 s, phase 4 force-off at 105 s.
"""

import pytest

from cadence.controllers import actuated as A
from cadence.controllers.checks import (
    check_all, check_conflicts, check_intergreen, check_link_sequences, check_pedestrians,
)
from cadence.simulation.demand import SCENARIOS
from cadence.simulation.network import INTERSECTIONS

from ctrl_helpers import QueueTraffic, drive, events, first, foes, link_map, make


def waiting_vehicle(phase, arrive, linger=15):
    """Presence from `arrive` until `linger` ticks after the phase turns green."""
    seen = {}

    def f(t, c):
        if c.vehicle_display(phase) == "G" and "g" not in seen and t > arrive:
            seen["g"] = t
        on = t >= arrive and ("g" not in seen or t < seen["g"] + linger)
        return {phase: on}
    return f


def merge(*fns):
    def f(t, c):
        out = {}
        for fn in fns:
            for p, v in fn(t, c).items():
                out[p] = out.get(p, False) or v
        return out
    return f


def test_rests_in_coordinated_green_without_calls():
    c = make("I1")
    changes = drive(c, 0, 5000)
    greens = {p for _, k, p in events(c, A.EV_PHASE_BEGIN_GREEN)}
    assert greens == {2, 6}
    assert not events(c, A.EV_PHASE_BEGIN_YELLOW)
    assert len(changes) == 1


@pytest.mark.parametrize("i", INTERSECTIONS)
def test_saturated_calls_lock_to_programmed_splits(i):
    c = make(i)
    plan = c.plan
    C = c.cycle
    drive(c, 0, 6 * C, presence=lambda t, c: {p: True for p in c.ph},
          peds=lambda t, c: list(c.peds) if t % 50 == 0 else [])
    assert c.overruns == 0
    for p, d in c.ph.items():
        starts = [t for t, k, q in c.events if q == p and k in (A.EV_PHASE_BEGIN_LPI, A.EV_PHASE_BEGIN_GREEN)
                  and not (k == A.EV_PHASE_BEGIN_GREEN and d.lpi and any(
                      e == (t - d.lpi, A.EV_PHASE_BEGIN_LPI, p) for e in c.events))]
        yellows = [t for t, k, q in c.events if k == A.EV_PHASE_BEGIN_YELLOW and q == p]
        pairs = [(s, next(y for y in yellows if y > s)) for s in starts[2:-1]]
        for s, y in pairs:
            assert y - s == A.s2t(plan.splits_s[p]) - d.clearance, (i, p, s)
        taus = {(s - c.offset) % C for s, _ in pairs}
        assert len(taus) == 1, (i, p, taus)


def test_side_call_waits_for_yield_point():
    c = make("I1")
    drive(c, 0, 2000, presence=lambda t, c: {4: t >= 10})
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 2) == 590
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 6) == 590
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 4) == 650
    # continuous presence: phase 4 runs to its force-off at 105 s
    assert first(c, A.EV_PHASE_FORCE_OFF, 4) == 1050


def test_gap_out_gives_early_return_to_coordinated_green():
    c = make("I1")
    drive(c, 0, 1800, presence=waiting_vehicle(4, arrive=100))
    g4 = first(c, A.EV_PHASE_BEGIN_GREEN, 4)
    assert g4 == 650
    # minimum green 7 s, then gap-out (the vehicle left after 1.5 s, passage 2 s)
    assert first(c, A.EV_PHASE_GAP_OUT, 4) == g4 + 70
    # side clearance 5 s, then phase 2 returns 33 s before the next cycle starts
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 2, after=1) == g4 + 70 + 50 == 770


def test_simultaneous_gap_out_at_barrier():
    c = make("I2")
    drive(c, 0, 3000, presence=merge(waiting_vehicle(4, arrive=300), waiting_vehicle(8, arrive=300, linger=150)))
    g8 = first(c, A.EV_PHASE_BEGIN_GREEN, 8)
    y4, y8 = first(c, A.EV_PHASE_BEGIN_YELLOW, 4), first(c, A.EV_PHASE_BEGIN_YELLOW, 8)
    # phase 8 keeps extending until its vehicle leaves plus the passage time;
    # phase 4 gapped out long before but holds so both cross the barrier together
    assert y8 == g8 + 150 + 20
    assert y4 == y8
    assert first(c, A.EV_PHASE_GAP_OUT, 4) == y4


def test_ring_rests_in_red_without_dual_entry():
    c = make("I2")
    drive(c, 0, 3000, presence=waiting_vehicle(4, arrive=300))
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 4) is not None
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 8) is None


def test_resting_ring_reenters_when_called():
    c = make("I2")

    def f(t, c):
        g4 = first(c, A.EV_PHASE_BEGIN_GREEN, 4)
        return {4: t >= 100, 8: g4 is not None and t >= g4 + 30}
    drive(c, 0, 3000, presence=f)
    g4 = first(c, A.EV_PHASE_BEGIN_GREEN, 4)
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 8) == g4 + 30


def test_pedestrian_interval_completes_before_yellow():
    c = make("I1")
    drive(c, 0, 2000, peds=lambda t, c: ["P4"] if t == 100 else [])
    g4 = first(c, A.EV_PHASE_BEGIN_GREEN, 4)
    assert first(c, A.EV_PED_BEGIN_WALK, 4) == g4 == 650
    assert first(c, A.EV_PED_BEGIN_CLEAR, 4) == g4 + 70
    assert first(c, A.EV_PED_BEGIN_DONT_WALK, 4) == g4 + 70 + 160
    # no vehicles: the phase ends as soon as the pedestrian clearance ends
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 4) == g4 + 230


def test_leading_pedestrian_interval_at_i4_only():
    c = make("I4")
    drive(c, 0, 4000, peds=lambda t, c: ["P8"] if t == 100 else [])
    lpi = first(c, A.EV_PHASE_BEGIN_LPI, 8)
    assert first(c, A.EV_PED_BEGIN_WALK, 8) == lpi
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 8) == lpi + 50
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 8) == lpi + 230
    c1 = make("I1")
    drive(c1, 0, 2000, peds=lambda t, c: ["P8"] if t == 100 else [])
    assert first(c1, A.EV_PHASE_BEGIN_LPI, 8) is None
    assert first(c1, A.EV_PED_BEGIN_WALK, 8) == first(c1, A.EV_PHASE_BEGIN_GREEN, 8)


def test_coordinated_phase_recycles_walk_only_if_it_fits():
    c = make("I1")
    drive(c, 0, 3000, peds=lambda t, c: ["P2"] if t in (100, 500) else [])
    walks = [t for t, _, _ in events(c, A.EV_PED_BEGIN_WALK, 2)]
    # call at 10 s fits before the 59 s yield point; the call at 50 s does not
    # and is served when the resting coordinated phase enters the next cycle
    assert walks == [100, 1100]
    assert not events(c, A.EV_PHASE_BEGIN_YELLOW)


def test_lead_left_called_during_rest_goes_around_the_barrier():
    c = make("I1")
    drive(c, 0, 4000, presence=waiting_vehicle(1, arrive=700))
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 2) == 700
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 1) == 760
    g2 = first(c, A.EV_PHASE_BEGIN_GREEN, 2, after=1)
    assert g2 == first(c, A.EV_PHASE_GAP_OUT, 1) + 60
    # back in coordinated green, held to cycle 1's yield point and beyond (no calls)
    assert first(c, A.EV_PHASE_BEGIN_YELLOW, 2, after=g2) is None


def test_call_after_window_closes_waits_for_next_cycle():
    c = make("I1")
    drive(c, 0, 3000, presence=lambda t, c: {3: t >= 680})
    # phase 3 cannot finish 5 s of green before its 70 s force-off, so it is
    # served after the next yield point (110 + 59 = 169 s) plus clearance
    assert first(c, A.EV_PHASE_BEGIN_GREEN, 3) == 1750


@pytest.mark.parametrize("scenario", ["S0", "S1", "S2", "S3"])
@pytest.mark.parametrize("i", INTERSECTIONS)
def test_random_traffic_passes_all_checks(i, scenario):
    c = make(i, scenario, start=0)
    traffic = QueueTraffic(c, seed=10 * INTERSECTIONS.index(i) + SCENARIOS.index(scenario))
    end = 25 * c.cycle
    changes = drive(c, 0, end, presence=traffic.presence, peds=traffic.peds)
    violations = check_all(changes, end, link_map(i), foes(i), c)
    assert violations == []
    assert c.overruns == 0
    served = {p for _, k, p in events(c, A.EV_PHASE_BEGIN_GREEN)}
    assert served == set(c.ph)


def test_snapshot_and_restore_reproduce_the_future():
    c = make("I3", "S3")
    traffic = QueueTraffic(c, seed=7)
    script = []

    def record(t, c):
        p = traffic.presence(t, c)
        q = traffic.peds(t, c)
        script.append((p, q))
        return p
    drive(c, 0, 3000, presence=record, peds=lambda t, c: script[-1][1])
    snap = c.snapshot()
    n0 = len(c.events)
    tail = []

    def rec2(t, c):
        p = traffic.presence(t, c)
        q = traffic.peds(t, c)
        tail.append((p, q))
        return p
    first_run = drive(c, 3000, 9000, presence=rec2, peds=lambda t, c: tail[-1][1])
    after = list(c.events[n0:])
    c.restore(snap)
    replay = drive(c, 3000, 9000, presence=lambda t, c: tail[t - 3000][0], peds=lambda t, c: tail[t - 3000][1])
    assert replay == first_run
    assert c.events[n0:] == after


# ---------------------------------------------------------------------------
# The checker must catch violations (otherwise passing tests mean nothing)
# ---------------------------------------------------------------------------

def _good_run():
    c = make("I1")
    changes = drive(c, 0, 3 * c.cycle, presence=lambda t, c: {p: True for p in c.ph})
    return c, changes


def test_checker_detects_conflicting_greens():
    c, changes = _good_run()
    lm, fs = link_map("I1"), foes("I1")
    t, s = changes[3]
    i = s.index("G")
    j = next(iter(fs[i]))
    bad = list(s)
    bad[j] = "G"
    assert check_conflicts([(t, "".join(bad))], lm, fs, c)


def test_checker_detects_short_yellow_and_illegal_change():
    c, changes = _good_run()
    lm = link_map("I1")
    end = 3 * c.cycle
    k = next(n for n, (_, s) in enumerate(changes) if "y" in s and n > 2)
    t, s = changes[k]
    shifted = changes[:k + 1] + [(t2 - 5 if t2 > t else t2, s2) for t2, s2 in changes[k + 1:]]
    assert check_link_sequences(shifted, end, lm, c)
    skipped = changes[:k] + changes[k + 1:]  # green straight to red
    assert check_link_sequences(skipped, end, lm, c)


def test_checker_detects_short_red_clearance():
    c, changes = _good_run()
    lm, fs = link_map("I1"), foes("I1")
    end = 3 * c.cycle
    # remove the all-red interval by starting each red-clear state's successor earlier
    out = []
    for n, (t, s) in enumerate(changes):
        if n > 0 and "y" not in s and "G" in s and "y" in changes[n - 1][1]:
            continue
        out.append((t, s))
    assert check_intergreen(out, end, lm, fs, c)


def test_checker_detects_pedestrian_clearance_into_yellow():
    c = make("I1")
    drive(c, 0, 2000, peds=lambda t, c: ["P4"] if t == 100 else [])
    assert check_pedestrians(c.events, c) == []
    y = first(c, A.EV_PHASE_BEGIN_YELLOW, 4)
    c.events = [(t - 30 if (k == A.EV_PHASE_BEGIN_YELLOW and p == 4 and t == y) else t, k, p)
                for t, k, p in c.events]
    assert check_pedestrians(c.events, c)
