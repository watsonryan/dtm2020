# Model Notes

Author: Watson

## Operational Source Mapping

Canonical operational source for this port:

- `/Users/rmw/Documents/code/mcm/src/libswamif/dtm2020_F107_Kp-subr_MCM.f90`
- `/Users/rmw/Documents/code/mcm/src/libswamif/dtm2020_sigma_function.f90`
- `/Users/rmw/Documents/code/mcm/src/libswamif/m_dtm.f90`

Implemented in this repo:

- `src/dtm2020_operational.cpp`
- `src/sigma_uncertainty.cpp`

## Behavior Carried Over

- MCM-specific transition attenuation in the 120-150 km band (toggle via `Options::emulate_mcm_transition`).
- Density uncertainty model from `sigma_function`.
- Fortran coefficient-file ingestion semantics, including support for `D` exponents.

## Outstanding Items

- Golden regression vectors from Fortran baseline.
- Optional research model (`dtm5`) port.

## Research Model Caution

In `/Users/rmw/Documents/code/mcm/src/dtm2020/dtm2020_F30_Hp-subr.f90`, `ap60` is declared with dimension 7 but indexed up to 10. This must be resolved before a faithful `dtm5` port.

## Research Port Status

- `Dtm2020Research` now evaluates the `dtm5` path in C++.
- Coefficient loading supports both tokenized rows and fixed-width Fortran rows used in `DTM_2020_F30_ap60.dat`.
- Validation currently uses benchmark values from `Mod_DTM2020F30Hp60_Benchmark.f90` via `tests/research_benchmark_test.cpp`.
