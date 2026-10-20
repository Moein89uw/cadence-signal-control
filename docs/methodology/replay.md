# Save, resume and counterfactual replay, Milestone 1

Step Zero's evaluation compares a policy's action with the baseline action from the same traffic state (paired counterfactual action evaluation), and its cost J includes a clearance run under the baseline controller with no new demand. Both need the simulation to be saved and resumed. Code: `CorridorSimulation.checkpoint` and `CorridorSimulation.resume` in `src/cadence/simulation/runner.py`, and `src/cadence/simulation/replay.py`.

## What a checkpoint holds

- SUMO's own state file: every vehicle (including the insertion backlog and vehicles SUMO has read ahead from the route file), signal programs, and SUMO's random number generators (`--save-state.rng`), written with 25 decimal places.
- The pickled Python state: all four controllers (rings, timers, pedestrian intervals, cycle counter, pending commands), the observation recorder (detector states, radar track numbering, turning-count bookkeeping), the vehicle ledger, the harness, and the detector reading for the checkpoint tick.

Saving takes a few milliseconds and does not disturb the run.

## How a branch runs

A branch resumes a checkpoint in a fresh process, in which SUMO is started exactly once, directly from the saved state, and the Python state is restored from the pickle. Actions are external controller commands issued at given times. A clearance branch starts without the route file and drops every vehicle scheduled at or after the checkpoint time, so the backlog and every vehicle already in the network are served and nothing new enters; it ends when the network and backlog are empty, or at the cap.

## Guarantees

| Property | Status | How it is tested |
| --- | --- | --- |
| Runs resumed from one checkpoint with the same actions are identical, bit for bit (positions, speeds, signal states, controller events, detector events, radar, turning counts, ledger) | Exact | Fast tests at an S1 checkpoint, with a backlog of more than 20 vehicles, with a one-action counterfactual, and for clearance; slow test at ten checkpoints across the S2 build-up and core |
| A resumed run tracks the uninterrupted run | Close; exact at most checkpoints | Vehicle-time after the checkpoint within 0.1 % (measured: identical to five significant digits at all ten S2 checkpoints, and fully exact at seven) |

The second property is not exact everywhere because SUMO's state file does not store the junction "approaching vehicle" registrations that yielding vehicles consult, nor all lane-change memory. The first steps after a load can then differ by a fraction of a metre for a vehicle at a merge or about to change lanes. The rule that follows: **compare a counterfactual branch only with a baseline branch resumed from the same checkpoint**, never with the uninterrupted run.

## What does not work, and is guarded against

- Loading a state into a running SUMO (`simulation.loadState`): stale internal records cause collisions within seconds.
- Saving or loading in a process that has already run a SUMO session: libsumo does not fully reset between sessions, which changes saved states. `checkpoint()` and `resume()` raise an error in that case; use `replay.in_fresh_process`.
- SUMO's default state precision (2 decimals), or 17: values below 1 m lose bits. 40 also failed; 25 is exact in every test.

Two network rules also support exact replay and are realistic in their own right: passenger cars do not change lanes inside junctions, and the I2 permissive left-turners wait at the stop line.

## External commands

The controller accepts call, hold, force-off and omit, limited by controller family as in Document Zero 3.5: Family A (I1, I3) accepts all four; Family B (I2, I4) has no omit. Commands change when phases start and end, never the minimum green, clearance or pedestrian intervals; a randomized test applies thousands of commands and checks every safety property. Force-off of a coordinated phase is not supported in Milestone 1.
