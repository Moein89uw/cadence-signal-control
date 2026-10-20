# Licensing

## This repository

All rights reserved. No license has been chosen yet; the project is private and research-first (decision recorded October 17, 2026). Choose a license before any public release or distribution.

## Third-party dependencies

Cadence calls these as separate, unmodified installed packages. None of their source code is copied into this repository.

| Dependency | Use | License |
| --- | --- | --- |
| Eclipse SUMO 1.28.0 (`eclipse-sumo`, `libsumo`) | Microsimulation ground truth, network building | EPL-2.0, with GPL-2.0-or-later as a secondary license |
| NumPy | Random-number generation, arrays | BSD-3-Clause |
| pytest (development only) | Tests | MIT |
| Matplotlib (reports only, optional) | Figures in docs/validation | Matplotlib License (PSF-based, BSD-compatible) |

## Excluded on purpose

The Poelman real-time predictive control prototyping environment (4TU.ResearchData, 2024) is GPL-3.0. Step Zero allows it for independent benchmarking only. No GPL-3.0 code may be incorporated into or linked with this repository without a license review first.
