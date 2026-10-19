# Observations: detectors and logs, Milestone 1 (fault-free)

This note describes how Document Zero section 3.6 detection is simulated and what each cabinet reports. Milestone 1 observations are fault-free: they equal the simulation's ground truth, at each controller family's logging resolution. Faults (chatter, dropouts, radar bias, glare, occlusion, clock drift) come later and will transform these products, not the simulation.

## Layout

| Int. | Document Zero 3.6 | Simulated as |
| --- | --- | --- |
| I1 | Stop-bar loops all lanes; advance loops EB/WB at 100 m; system loops midblock I1–I2 | 12 m stop-bar presence zone on every approach lane; 1.8 m loops on both EB and WB through lanes 100 m upstream of the stop line; 1.8 m loops on both lanes in each direction at the midpoint of the modeled I1–I2 link (187.5 m from the downstream stop line), logged by I1 |
| I2 | Stop-bar loops only | 12 m stop-bar presence zone on every approach lane |
| I3 | Radar (advance 120 m + stop bar, trajectory-capable); video turning-movement counts | Radar tracks of every vehicle within 120 m of the stop line on all four approaches, sampled every 0.1 s; stop-bar presence zones derived from the radar; one turning-movement record per vehicle leaving an approach, binned in 5 min |
| I4 | Video presence detection, all approaches | 12 m video presence zone on every approach lane |

All intersections have pedestrian push buttons. The EB approach to I3 is only 120 m long, so the radar there covers the whole link from I2. Zone and loop lengths, the midblock position, the radar sampling interval and the count bin are Milestone 1 choices, recorded under `detection` in `networks/corridor_r1/spec.json`.

Not simulated in Milestone 1: bicycle video detection at I3, passive pedestrian video at I4 (there are no bicycles or walking pedestrians yet), and the corridor probe-trajectory feed.

## Detection drives the controller

Each stop-bar detector feeds exactly one phase, and the controller's calls and extensions come from these detectors, so the controller sees exactly what the cabinet logs.

## Logs

Each cabinet writes a high-resolution log of (time, code, parameter) in the Indiana enumeration.

- **Family A (I1, I3)**: all controller events and detector events at 0.1 s.
- **Family B (I2, I4)**: phase state (begin green, begin yellow, begin and end red clearance), pedestrian signal state, and vehicle and pedestrian detector on/off, at 1 s. Gap-out, force-off and call events are not logged.

Detector codes: 82 detector on, 81 detector off (parameter = detector channel); 90 pedestrian detector on, 89 off (parameter = pedestrian phase). Within a tick, detector events are written before the controller events they cause.

## Products written per run

`logs/<I>_hires.csv` for each cabinet, `logs/I3_radar.npz` (tick, track, approach, lane, distance to stop line, speed), `logs/I3_tmc.csv` (one row per vehicle movement) and `logs/I3_tmc_bins.csv` (counts per 5 min bin), and `logs/detectors.csv` (channel table). Radar track numbers are assigned by the simulated radar; SUMO vehicle identities never appear in observation products.

## How it is checked

- The layout matches Document Zero 3.6: stop-bar detection on every lane, loops at the stated distances, radar coverage of 120 m (or the whole approach when shorter) on every I3 approach.
- Every detector's reported state equals the state computed independently from vehicle positions and lengths.
- Point loops register exactly one actuation per vehicle.
- Video turning-movement counts equal the counts in SUMO's own route output.
- Radar tracks stay within the zone, move toward the stop line, and correspond one to one with vehicles.
- Family B logs carry whole-second timestamps and only the codes listed above.
