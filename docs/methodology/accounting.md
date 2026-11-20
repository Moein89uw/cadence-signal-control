# Vehicle accounting, Milestone 1

The research cost J is vehicle-time, so every vehicle must be counted from the moment it is due to enter until it leaves. This note defines the terms and how they are checked. Code: `src/cadence/accounting/ledger.py`.

## Terms

| Term | Meaning | SUMO name |
| --- | --- | --- |
| scheduled | vehicles whose scheduled entry time has been reached | — |
| backlog | scheduled vehicles waiting outside the network because the entry lane is full | pending |
| inserted | vehicles that have entered the network | departed |
| in network | inserted vehicles that have not yet left | running |
| queued | in-network vehicles that are halted (speed below 0.1 m/s) | halting |
| exited | vehicles that reached the end of their route | arrived |

At every 0.1 s step the ledger checks:

- scheduled = backlog + inserted
- inserted = in network + exited
- 0 ≤ queued ≤ in network

Teleporting is disabled (`--time-to-teleport -1`), so no vehicle can leave the network except by finishing its route; collisions are recorded with `--collision.action warn`, which does not remove vehicles.

## Vehicle-time

System vehicle-time adds the backlog to the vehicles in the network, so a policy cannot look better by keeping vehicles out of the network. The ledger integrates it over every step; it also keeps in-network vehicle-time and queued vehicle-time (the queue is measured once per second).

SUMO conventions, measured rather than assumed: a vehicle is inserted at the start of the first step that begins at or after its scheduled time, and SUMO's trip records stamp insertion and exit with the start time of the step in which they happen. The ledger counts a vehicle in the snapshot after the step ending at tick k exactly when ⌈scheduled/0.1 s⌉ + 1 ≤ k ≤ exit/0.1 s.

## How it is checked

- The identities hold at every step, in a normal 20 min S1 run and in a stress run where a test-only override holds I1 eastbound red for 10 minutes, so the entry link fills and a backlog of more than 50 vehicles forms outside the network and then drains.
- System vehicle-time from the ledger equals the vehicle-time computed independently from SUMO's per-vehicle trip records (including unfinished trips), to 10⁻⁶ vehicle-seconds.
- The number of exited vehicles equals SUMO's count of finished trips; every scheduled vehicle appears in SUMO's trip records.
- The queued count equals the sum of SUMO's per-lane halting counts over all lanes, including junction lanes.

## Outputs

Each run writes `ledger.csv`: one row per second with scheduled, backlog, inserted, in network, queued and exited, plus totals in the run summary.
