# Corridor R-1 in SUMO: conventions and Milestone 1 assumptions

This note records how Document Zero (CAD-000 v0.1, section 3–4) is represented in SUMO, and every assumption Milestone 1 adds where Document Zero is silent. Assumptions are tagged `M1` in `networks/corridor_r1/spec.json`.

## Conventions

- x increases eastbound along Alder Avenue; y increases northbound.
- Approaches are named by travel direction: EB, WB, SB (arrives from the north leg), NB (arrives from the south leg).
- SUMO lane 0 is the rightmost lane; on bay edges the left-turn bay is the highest lane index.
- Each link is split into edges at the start of a left-turn bay and, eastbound between I3 and I4, at the lane drop.

## Geometry taken from Document Zero

| Link | Edges (lanes, usable m) | Usable storage per through lane |
| --- | --- | --- |
| West entry → I1 (EB) | EB0a (2, 525) + EB0b (3, 75; LT bay) | 80 veh |
| I1 → I2 (EB) | EB1a (2, 315) + EB1b (3, 60) | 50 veh |
| I2 → I3 (EB) | EB2a (2, 80) + EB2b (3, 40) | 16 veh |
| I3 → I4 (EB) | EB3a (2, 157.5) + EB3b (1, 70; after lane drop) + EB3c (2, 50; LT bay) | 21 veh × 2, then 16 veh × 1 |
| East entry → I4 (WB) | WB4a (2, 540) + WB4b (3, 60) | 80 veh |
| I4 → I3 (WB) | WB3a (2, 217.5) + WB3b (3, 60) | 37 veh |
| I3 → I2 (WB) | WB2a (2, 60) + WB2b (3, 60) | 16 veh |
| I2 → I1 (WB) | WB1a (2, 300) + WB1b (3, 75) | 50 veh |

Usable storage is enforced exactly with SUMO's edge `length` override, using the Document Zero jam spacing of 7.5 m (vehicle length 5.0 m plus minimum gap 2.5 m).

## Milestone 1 assumptions

1. **Cross-street legs** are 300 m long in each direction (Document Zero gives lane configurations only).
2. **East exit link** is one lane, because only the single EB through/right lane at I4 and the NB right turn feed it.
3. **Lane drop**: both EB lanes connect into the single lane at a priority node, and netconvert gives the inner lane priority, so vehicles in the right lane yield at the merge. A zipper node was tried first; it produced SUMO emergency-braking warnings at the merge (about three per simulated hour), which the priority merge does not.
4. **Mainline right turns** share the rightmost through lane; there is no right-turn bay.
5. **Right turn on red** is not modeled.
5a. **Permissive lefts at I2 wait at the stop line.** The side-street left turns in I2's single shared lane have no internal waiting point (SUMO `contPos="0"`). A left-turner waiting for a gap therefore blocks the vehicles behind it (head-of-line blocking) and is never trapped inside the junction when the phase ends. With SUMO's default internal waiting point, a left-turner that entered on yellow was still clearing when the next protected left received green after the 1.5 s all-red, and SUMO recorded a junction collision. The cost of this choice is that end-of-phase left turns ("sneakers") are not modeled, so permissive left capacity at I2 is understated; the shared-lane validation will quantify this.
5b. **No lane changes inside junctions** for passenger cars (junction lanes allow lane changes only for emergency vehicles). This is realistic and makes saved states more faithful, because SUMO's state file does not keep all lane-change memory.
6. **Turning fractions**: at I1–I3 eastbound and I4–I2 westbound, left = right = (1 − t)/2, with through fraction t per scenario (S0 0.85, S1 0.745, S2 0.81, S3 0.745). These were calibrated so the derived EB through/right volume at I4 matches Document Zero 4.2 within 1% (430, 755, 1045, 718 veh/h against 430, 760, 1050, 720). Eastbound at I4 and westbound at I1 use L/T/R = 0.10/0.80/0.10. Cross streets use 0.15/0.70/0.15 at I1 and I3, 0.10/0.80/0.10 at I2, and 0/0.80/0.20 at I4 (no lefts).
7. **Cross-street volumes per approach**: S0 — I1 250, I2 150, I3 200, I4 150 veh/h; S1–S3 — 450, 300, 400, 300 veh/h (within the Document Zero ranges).
8. **Pedestrians** are push-button calls only, with no walking agents, so turning vehicles never yield to pedestrians in Milestone 1. Crosswalks: P2 north leg and P6 south leg (crossing the side street, served with φ2/φ6); P4 west leg and P8 east leg (crossing the arterial, served with φ4/φ8).
9. **Vehicle model**: one passenger type, Krauss car-following, accel 2.6 m/s², decel 4.5 m/s², sigma 0.5, tau 1.0 s.

## Not in Milestone 1

Bus Route R and transit priority, bicycles, emergency preemption at I2, the probe-trajectory feed, controller families' differing command sets and clocks, and the fault catalog. They are later increments.

## Randomness

Each entry stream and each crosswalk has its own random generator, built from `SeedSequence(master seed 20260811, spawn_key=(scenario, replication, kind, stream))`. Adding or removing a stream never changes another stream's draws; a test checks this for the S3 pulse.
