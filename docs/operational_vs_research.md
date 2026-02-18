# Operational vs Research Implementations

Author: Watson

This document compares the two model paths implemented in this repo:
- `Dtm2020Operational` (F10.7/Kp forcing)
- `Dtm2020Research` (F30/ap60 forcing)

## 1) Public API and inputs

Operational API:
- header: `include/dtm2020/dtm2020_operational.hpp`
- input type: `OperationalInputs`
- forcing fields: `f107`, `f107m`, `kp_delayed_3h`, `kp_mean_24h`
- extra API: `DensityUncertaintyPercent(...)`

Research API:
- header: `include/dtm2020/dtm2020_research.hpp`
- input type: `ResearchInputs`
- forcing fields: `f30`, `f30m`, `ap60[10]`
- no sigma/uncertainty helper in this class

Both use:
- `Result<T, Error>` return style
- shared `Outputs` structure
- structured `Error` payload (`code/message/detail/location`)

## 2) Core evaluator path

Operational implementation:
- file: `src/dtm2020_operational.cpp`
- core function path: `Gldtm(...)` / operational `dtm3` logic

Research implementation:
- file: `src/dtm2020_research.cpp`
- core function path: `GldtmHp(...)` / research `dtm5` logic

## 3) Geomagnetic handling

Operational path:
- computes geomagnetic terms using fixed constants (`cpmg`, `spmg`, `xlmg`)
- derives `p10mg/p20mg/p40mg` in that frame

Research path:
- computes geomagnetic coordinates via `Geogm(...)`
- derives additional geomagnetic Legendre terms (`p11mg`, `p22mg`, `p31mg`, etc.)

## 4) Activity-index preprocessing

Operational:
- directly uses 3h-delayed and 24h-mean Kp-style inputs

Research:
- derives Kp-like values from `ap60` using `BintOpenEnded(...)`
- builds a richer `akp` vector with latitude-dependent blending

## 5) Low-altitude compatibility behavior

Operational only:
- `Options::emulate_mcm_transition`
- applies selected `az2` attenuation in the 120-150 km band for MCM compatibility

Research:
- no equivalent transition toggle/path

## 6) Coefficient loading behavior

Operational loader:
- tokenized parsing with Fortran `D`/`d` exponent normalization

Research loader:
- supports both tokenized rows and fixed-width Fortran row format

## 7) Validation coverage

Operational tests:
- `dtm2020_operational_smoke`
- `dtm2020_operational_golden`
- `dtm2020_operational_numerical_regression`
- `dtm2020_operational_perf_benchmark`

Research tests:
- `dtm2020_research_smoke`
- `dtm2020_research_benchmark`
- `dtm2020_research_perf_benchmark`

## 8) Performance notes

Operational and research both have profile benchmark scripts:
- `tools/profile_operational.sh`
- `tools/profile_research.sh`

Current baseline and optimization deltas are tracked in:
- `docs/perf_baseline.md`
