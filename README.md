# dtm2020-cpp

Author: Watson

C++20 re-implementation workspace for DTM2020 operational model and uncertainty function.

## Build

```bash
cmake --preset macos-debug
cmake --build --preset macos-debug
ctest --preset macos-debug
```

## Full Validation

```bash
tools/validate_all.sh
```

This runs:
- debug test suite
- ASan test suite
- operational vector regeneration from Fortran (if `gfortran` is available)
- operational/research golden benchmark checks

## CLI Example

```bash
./build/macos-debug/dtm2020_cli <coeff_file> <alt_km> <lat_deg> <lon_deg> <lt_h> <doy> <f107> <f107m> <kp3h> <kp24h>
```

## Research Model

The research path (`Dtm2020Research`) is enabled by default (`DTM2020_ENABLE_RESEARCH=ON`) and validated with:
- synthetic smoke test
- benchmark regression against `Mod_DTM2020F30Hp60_Benchmark.f90` expected values
