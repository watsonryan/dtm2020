# Numerical Fidelity Notes

Author: Watson

## Current Strategy

- Core operational evaluation (`dtm3` / `gldtm`) uses `float` internally to match Fortran `real` behavior.
- Public API uses `double` inputs/outputs, with conversion at boundaries.
- Build defaults to strict floating-point settings:
  - non-MSVC: `-ffp-contract=off`
  - MSVC: `/fp:precise`

## Normalization Policy

- Longitude is normalized to `[0, 360)` degrees before conversion to radians.
- Local time is normalized to `[0, 24)` hours before conversion to radians.
- Latitude is validated in `[-90, 90]`.
- Altitude must be `> 120 km`.

## Known Gaps

- Golden-vector parity against CNES Fortran has not yet been added.
- Cross-compiler tolerance envelopes are not yet characterized.

## Next Fidelity Step

Generate and commit frozen operational vectors from the canonical Fortran MCM path (`dtm2020_F107_Kp-subr_MCM.f90`) and add tolerance-based regression tests.
