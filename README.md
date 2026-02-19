# dtm2020-cpp

Author: Watson

C++20 implementation of the Drag Temperature Model (DTM) 2020 model (https://ccmc.gsfc.nasa.gov/models/DTM~2020/) with:
- operational model (`F10.7/Kp`) via `dtm3/gldtm`
- research model (`F30/ap60`) via `dtm5` path
- density uncertainty function (`sigma_function`)
- regression tests against generated/published reference vectors

## What is in this repo

- `include/dtm2020/`: public API headers
- `src/`: model implementations
- `examples/dtm2020_cli.cpp`: single-point operational CLI
- `tests/`: smoke + golden + benchmark tests
- `tools/validate_all.sh`: one-command local validation
- `docs/`: model and fidelity notes

## Prerequisites

- CMake >= 3.25
- Ninja
- C++20 compiler
  - macOS: AppleClang with C++20 support
  - Linux: GCC/Clang with C++20 support
  - Windows: MSVC with C++20 support
- Optional: `gfortran` (for regenerating Fortran operational vectors)

## Build and test

### Quick start (macOS)

```bash
cmake --preset macos-debug
cmake --build --preset macos-debug -j
ctest --preset macos-debug --output-on-failure
```

### Sanitizer run

```bash
cmake --preset macos-asan
cmake --build --preset macos-asan -j
ctest --preset macos-asan --output-on-failure
```

### Available presets

See `CMakePresets.json` for:
- debug/release
- asan/ubsan/tsan (platform-dependent)
- profile builds
- macOS/Linux/Windows preset families

### Install and package consumption

Install locally:

```bash
cmake --preset macos-release
cmake --build --preset macos-release -j
cmake --install build/macos-release --prefix /tmp/dtm2020-install
```

Consume from another CMake project:

```cmake
find_package(dtm2020 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE dtm2020::dtm2020)
```

## Performance workflow

Run the operational throughput benchmark with profile flags:

```bash
tools/profile_operational.sh
```

Run the research throughput benchmark:

```bash
tools/profile_research.sh
```

Or manually:

```bash
cmake --preset macos-profile
cmake --build --preset macos-profile -j
./build/macos-profile/tests/dtm2020_operational_perf_benchmark
```

Environment overrides for longer/shorter runs:
- `DTM2020_PERF_SAMPLES` (default `15`)
- `DTM2020_PERF_ITERATIONS` (default `80`)

## Full local validation

Run:

```bash
tools/validate_all.sh
```

This performs:
1. Debug build + tests
2. ASan build + tests
3. Operational Fortran vector regeneration (if `gfortran` is present)
4. Golden/benchmark regression tests

## Coefficient files

The model loaders expect DTM2020 coefficient files from your external data source.

External-data regression tests resolve coefficient paths in this order:
1. runtime argument passed directly to the test executable
2. repo-local defaults:
   - `testdata/DTM_2020_F107_Kp.dat`
   - `testdata/DTM_2020_F30_ap60.dat`
3. CMake fallback variables (primarily for CI):
   - `DTM2020_OPERATIONAL_COEFF_FILE`
   - `DTM2020_RESEARCH_COEFF_FILE`

Runtime override example:

```bash
./build/macos-debug/tests/dtm2020_operational_golden /path/to/DTM_2020_F107_Kp.dat
./build/macos-debug/tests/dtm2020_research_benchmark /path/to/DTM_2020_F30_ap60.dat
```

CMake fallback example (CI-oriented):

```bash
cmake --preset macos-debug \
  -DDTM2020_OPERATIONAL_COEFF_FILE=/path/to/DTM_2020_F107_Kp.dat \
  -DDTM2020_RESEARCH_COEFF_FILE=/path/to/DTM_2020_F30_ap60.dat
```

## CLI usage (operational)

Binary:

```bash
./build/macos-debug/dtm2020_cli \
  <coeff_file> <alt_km> <lat_deg> <lon_deg> <lt_h> <doy> <f107> <f107m> <kp3h> <kp24h>
```

Example:

```bash
./build/macos-debug/dtm2020_cli \
  /path/to/DTM_2020_F107_Kp.dat \
  400 45 -75 12 100 120 115 3 2
```

The CLI validates numeric arguments and prints key outputs:
- `temperature_k`
- `exospheric_temp_k`
- `density_g_cm3`
- `mean_mol_mass_g`

## Library API usage

### Operational model

```cpp
#include "dtm2020/dtm2020_operational.hpp"

auto model = dtm2020::Dtm2020Operational::LoadFromFile(coeff_path);
if (!model) {
  // model.error()
}

dtm2020::OperationalInputs in{};
in.altitude_km = 400.0;
in.latitude_deg = 45.0;
in.longitude_deg = -75.0;
in.local_time_h = 12.0;
in.day_of_year = 100.0;
in.f107 = 120.0;
in.f107m = 115.0;
in.kp_delayed_3h = 3.0;
in.kp_mean_24h = 2.0;

auto out = model.value().Evaluate(in);
if (!out) {
  // out.error()
}

const double rho = out.value().density_g_cm3;
```

### Research model

```cpp
#include "dtm2020/dtm2020_research.hpp"

auto model = dtm2020::Dtm2020Research::LoadFromFile(coeff_path);
if (!model) {
  // model.error()
}

dtm2020::ResearchInputs in{};
in.altitude_km = 300.0;
in.latitude_deg = 0.0;
in.longitude_deg = 0.0;
in.local_time_h = 12.0;
in.day_of_year = 180.0;
in.f30 = 120.0;
in.f30m = 120.0;
in.ap60 = {15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

auto out = model.value().Evaluate(in);
if (!out) {
  // out.error()
}
```

## Error handling and logging

- Library APIs return `Result<T, Error>` and do not write logs directly.
- `Error` includes:
  - `code`
  - `message`
  - `detail` (optional context)
  - `location` (optional source tag)
- Helper utilities:
  - `FormatError(const Error&)`
  - `ToString(ErrorCode)`
  - `MakeError(...)`
- App/test logging helpers are in `include/dtm2020/logging.hpp`:
  - `LogSink`, `LogLevel`
  - `MakeStderrLogSink()`
  - `Log(...)`, `LogError(...)`

## Testing strategy

- `dtm2020_operational_smoke`: parser/evaluation sanity checks
- `dtm2020_operational_golden`: parity against operational vectors
- `dtm2020_operational_numerical_regression`: frozen internal numerical guardrail vectors
- `dtm2020_research_smoke`: research path sanity checks
- `dtm2020_research_benchmark`: regression against benchmark values
- `dtm2020_sigma_smoke`: uncertainty function checks
- `dtm2020_thread_safety_smoke`: concurrent evaluation safety on shared model instances
- `dtm2020_operational_perf_benchmark`: throughput benchmark executable (not a pass/fail CTest)
- `dtm2020_research_perf_benchmark`: research throughput benchmark executable (not a pass/fail CTest)

## Additional docs

- `docs/model_notes.md`
- `docs/numerical_fidelity.md`
- `docs/operational_vs_research.md`
- `docs/thread_safety_and_performance.md`
- `docs/licensing.md`
