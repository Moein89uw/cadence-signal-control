# Corridor queue model (simplified prediction model), Milestone 1

The optimizer arms in Step Zero need a prediction model they can roll forward many times per decision. SUMO is the ground truth; this model is the fast approximation. Code: `src/cadence/prediction/queue_model.py`. Validation against SUMO: `docs/validation/prediction_model_m1.md`.

## Structure

A mesoscopic, discrete-vehicle model in 1 s steps. Each vehicle carries its remaining movements (one per intersection).

| Mechanism | Representation | Why it is needed on Corridor R-1 |
| --- | --- | --- |
| Stop-line queues | One first-in-first-out (FIFO) queue per approach lane; through vehicles pick the shorter allowed lane | Shared lanes: a vehicle that cannot go blocks the lane (head-of-line blocking) |
| Left-turn bays | Separate queue with Document Zero bay length; a left-turner that finds it full, or cannot reach it past the through queue, waits in the adjacent through lane and blocks it | Short bays at I3 (5 vehicles) and I4 (7) overflow in the peaks |
| Link storage | Each link holds at most its Document Zero usable storage; no discharge into a full link | Spillback on the 16-vehicle links I2–I3 and I3–I2 and at the I4 lane drop |
| Discharge | 2.0 s saturation headway per lane (1800 veh/h), 2 s start-up lost time, first 2 s of yellow used | Capacity of each green |
| Permissive lefts (I2 side streets) | A left-turner goes only if the opposing approach has no discharging through or right-turn queue and, optionally, no such departure in the last few seconds and no such arrival in the next few. Calibrated on S1, both extra margins came out at 0 s: SUMO's left-turners take any gap once the opposing queue has cleared | Gaps in opposing traffic set I2 side-street capacity |
| Travel between stop lines | Free-flow time over the modeled lanes, including junction lanes; vehicles join the next queue when they reach its stop line (point queue in time) | Arrival timing at downstream signals |
| Entries | Scheduled vehicles wait in a backlog until the entry link has room, inserted at one per 2 s per lane | Backlog counts toward vehicle-time, as in the ledger |

Inputs: the current state (vehicles and their positions or queue places, the backlog), future entries, and the signal states over the horizon. Output: vehicles in the system each second (whose sum is the horizon vehicle-time in the cost J), queues by approach, and discharges by movement.

`fifo=False` gives a comparison version in which each movement of a shared lane queues as if it had its own lane.

## What is deliberately left out

Lane changing upstream of the queue, speed variation between drivers, turning-speed differences in saturation flow, the spatial extent of queues, end-of-phase "sneaker" left turns (also absent from the SUMO network, see corridor_r1.md), pedestrians, buses and bicycles. The validation report states what these omissions cost.
