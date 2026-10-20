# Cadence signal control

Research testbed for the Cadence project: does using calibrated, fault-aware uncertainty about traffic state improve coordinated signal control on a corridor with finite storage, compared with an equally capable optimizer that uses point estimates?

The research design is set out in *Cadence Step Zero: Novelty, Formulation and Data Feasibility* (scientific design accepted October 17, 2026). The reference network is Corridor R-1 from Document Zero (CAD-000 v0.1).

## Status

Milestone 1 builds a trustworthy simulation foundation. It contains no optimizer and supports no research claim yet.

| Increment | Content | Status |
| --- | --- | --- |
| 1 | Corridor R-1 network and S0–S3 demand | Done |
| 2 | Dual-ring actuated-coordinated signal control with pedestrian intervals | Done |
| 3 | Loop, radar and camera observations (no faults) | Done |
| 4 | Vehicle accounting with teleporting disabled | Done |
| 5 | Save, resume and one-action counterfactual replay | Done |
| 6 | Prediction-model validation against SUMO | Planned |

## Layout

```
networks/corridor_r1/   spec.json (single source of truth) and generated SUMO network files
src/cadence/            simulation, controllers, detectors, accounting, evaluation
tests/                  acceptance tests per increment
experiments/            scripts that produce reported results
docs/methodology/       modeling assumptions and conventions
docs/validation/        validation reports
```

## Setup

Requires Python 3.10 or newer. SUMO is installed from PyPI with the package.

```bash
pip install -e ".[dev]"
python -m cadence.simulation.network   # rebuild the network from spec.json
python -m cadence.controllers.timing    # regenerate timing plans
pytest -m "not slow"                    # fast tests (about 40 s)
pytest -m slow                          # full scenario windows (about 5 min)
python experiments/run_scenario.py --scenario S1   # one run with a safety and timing summary
```

## Documentation

- `docs/methodology/corridor_r1.md`: how Document Zero is represented in SUMO, and every Milestone 1 assumption.
- `docs/methodology/signal_control.md`: the baseline actuated-coordinated controller and how it is checked.
- `docs/methodology/observations.md`: simulated detection (Document Zero 3.6) and the cabinet logs.
- `docs/methodology/accounting.md`: vehicle accounting and vehicle-time.
- `docs/methodology/replay.md`: checkpoints, resumed branches, clearance runs and what is exact.

## Ground rules

- `networks/corridor_r1/spec.json` is the single source of truth. Values tagged `DZ` come from Document Zero; values tagged `M1` are Milestone 1 placeholders that need review before any result is reported.
- Generated files are committed, and a test checks that a fresh build reproduces them.
- Every random draw comes from a seeded stream keyed by scenario, replication and stream, so results are reproducible.
- No GPL-3.0 code enters this repository without a license review (see `LICENSES/README.md`).
