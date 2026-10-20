# Baseline signal control (π₀): actuated-coordinated, Milestone 1

This note describes the controller in `src/cadence/controllers/` that runs every Corridor R-1 intersection in Milestone 1. It is the baseline policy π₀ that later arms are compared against, and it is the policy the clearance runs fall back to. Settings live in `configs/controller_r1.json`; timing plans are generated into `configs/timing_plans_r1.json`. Everything here is tagged M1 and needs review before results are reported.

## Why not SUMO's built-in controllers

SUMO's NEMA controller has no pedestrian phases, and timing changes take effect only at the end of a cycle. The research needs pedestrian intervals, external calls, holds and force-offs that take effect immediately, and controller state that can be saved and restored exactly. The controller is therefore written in Python, with no SUMO dependency, and sets SUMO signal states through libsumo every 0.1 s.

## Phases

| Ring | Main group | | Side group | |
| --- | --- | --- | --- | --- |
| 1 | φ1 WB left | φ2 EB through + right | φ3 NB left | φ4 SB through + right |
| 2 | φ5 EB left | φ6 WB through + right | φ7 SB left | φ8 NB through + right |

- I1 and I3 run all eight phases. I2 and I4 run six (no φ3, φ7).
- Mainline and side-street lefts in bays are protected only. At I2 the single shared side-street lane runs its left turn permissively in φ4 and φ8, yielding to the opposing movement. I4 side streets have no left turns.
- Sequence: leading lefts in both groups (φ1 before φ2, φ5 before φ6, φ3 before φ4, φ7 before φ8).
- Pedestrian phases: P2 (north leg) with φ2 and P6 (south leg) with φ6 cross the side street, clearance 9 s; P4 (west leg) with φ4 and P8 (east leg) with φ8 cross the arterial, clearance 16 s. Walk is 7 s. At I4, P4 and P8 have a 5 s leading pedestrian interval.

Phase concurrency is verified against SUMO's own junction conflict matrix: no two protected movements that can show green together conflict.

## Coordination

- Cycle k starts at offset + k × cycle; cycle time 0 is the programmed start of the main group.
- φ2 and φ6 are coordinated and on recall. They hold until a fixed yield point (the programmed end of coordinated green) and then end together if any other phase can be served; otherwise they rest in green.
- Non-coordinated phases have fixed force-offs. A phase starts only if its minimum green, plus the full pedestrian interval when a walk will be served, fits before its force-off. A call that misses its window is served next cycle.
- Gap-out uses a 2.0 s passage time on 12 m stop-bar presence zones.
- The last phase of a ring before a barrier gaps out only when the other ring is also ready (simultaneous gap-out). A ring with nothing to serve rests in red (no dual entry), and re-enters if a later phase in its sequence is called while the other ring is still serving.
- Time left by phases that gap out returns early to the coordinated phases.
- A lead left called while the coordinated phases rest after the yield point is served by crossing the empty side group (both coordinated phases end, then φ1 or φ5 runs, then coordinated green returns and holds to the next yield point). This is standard dual-ring behavior, and it briefly interrupts the opposing coordinated through.

## Pedestrians

- Walk begins with the vehicle phase, or 5 s before vehicle green when there is an LPI.
- Walk and pedestrian clearance always end before the vehicle phase's yellow; the phase holds green if needed.
- A pedestrian call during green is served in the same green only if the whole pedestrian interval fits before the yield point or force-off (recycle); otherwise it waits for the next service of its phase.
- Milestone 1 models push-button calls only, so turning vehicles never yield to pedestrians.

## Timing plans

Splits are generated from expected movement flows (S2 uses its 17:00–17:45 core): effective green is shared in proportion to critical flow ratios, subject to minimum greens and pedestrian minimums, and rounded to whole seconds. Offsets give eastbound progression at the posted speed, aligned on the programmed start of φ2. These are reasonable placeholder plans, not tuned ones. Under S2 the I4 eastbound through runs at a volume-to-capacity ratio of about 0.95.

## Event log

The controller logs Indiana high-resolution events where a code exists: 1 begin green, 4 gap-out, 6 force-off, 7 green termination, 8 begin yellow, 9 end yellow, 10 begin red clearance, 11 end red clearance, 21 begin walk, 22 begin pedestrian clearance, 23 begin don't walk, 43 phase call registered, 44 phase call dropped, 45 pedestrian call registered, 151 coordinated yield point. Cadence diagnostics: 200 green held past force-off, 201 barrier crossing (1 into main, 2 into side), 202 LPI begins. Timestamps are exact ticks of 0.1 s; the 1 s resolution of Family B controllers is applied later, in the observation layer.

## How it is checked

`src/cadence/controllers/checks.py` checks what SUMO was actually told, against SUMO's conflict matrix rather than the controller's own phase table:

1. No two conflicting links are ever non-red together (except a permissive movement and its opposing movement).
2. Main-group and side-group movements are never active together.
3. Every link goes green → yellow → red; yellow has exactly its programmed length; green is never shorter than minimum green.
4. No conflicting link turns green before the red clearance after a yellow has elapsed.
5. Walk and pedestrian clearance have exact lengths and end before yellow; the LPI is exactly 5 s.
6. Coordinated phases never yield before the yield point; force-offs happen exactly at the programmed point; no phase is held past its force-off.
7. Every vehicle call and pedestrian call is served within two cycles.

Tests also confirm that the checker reports deliberately broken signal sequences, so a passing result is not vacuous.
