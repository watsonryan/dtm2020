# DTM2020 (from SWAMI MCM) — Full C++20 Re‑implementation Plan

> **Goal:** Re-implement the **DTM2020 thermosphere model** (operational + optional research version) from the SWAMI `mcm` repository as a **production-quality C++20 library** with **GTest**, **CMake**, **CMakePresets** for **macOS/Linux/Windows** (Debug/Release/Sanitize/Profile), and **CPM.cmake** for third‑party dependencies.  
> **Constraint:** Numerical behavior must match the reference implementation as closely as is practical; **any existing tests must pass**, and we will add a robust new test suite.

---

## 0) Licensing and “can we do this?” gate (must be step 0)

Before writing any code:

- The upstream `mcm` repo’s README includes restrictive terms (academic / non‑commercial; “shall not … make any modification or improvement … disseminate…”).  
- A **C++ rewrite intended for “modern flight software / production”** almost certainly **conflicts** with those terms unless you have an explicit commercial/derivative-work license from CNES (and potentially Met Office / Elecnor Deimos).

**Hard rule:** do **not** proceed beyond scaffolding until a human confirms the licensing/permission situation and documents it in the repo (e.g., `docs/licensing.md`).

Deliverable for this gate:
- `docs/licensing.md` summarizing:
  - what code/data is being ported,
  - allowed use/distribution scope,
  - whether coefficient data files may be redistributed,
  - whether clean-room constraints apply.

> If licensing forbids redistribution: keep the port private/internal, and require the coefficient `.dat` files be supplied by the user at runtime (do not commit them).

---

## 1) Scope and compatibility targets

### 1.1 What we are porting
From `swami-h2020-eu/mcm`, the DTM2020 parts exist in two places:

- **Operational DTM2020 used by MCM** (F10.7 + Kp) in `src/libswamif`:
  - `dtm2020_F107_Kp-subr_MCM.f90` (contains `dtm3`, `gldtm`, `lecdtm`)
  - `dtm2020_sigma_function.f90` (density uncertainty model: `sigma_function`)
  - `m_dtm.f90` (Fortran wrapper + unit conversions)
- **Research DTM2020** (F30 + Hp60 / ap60) in `src/dtm2020`:
  - `dtm2020_F30_Hp-subr.f90` (contains `dtm5`, `gldtm_Hp`, `lecdtm`, `geogm`, `bint_oe`, etc.)
- **Coefficient data files** in `data/`:
  - `DTM_2020_F107_Kp.dat`
  - `DTM_2020_F30_ap60.dat`

### 1.2 What we are **not** porting (unless explicitly extended)
- The Unified Model (UM) netCDF tables and the blended “MCM” whole-atmosphere model.
- The Python wrapper and Fortran executable interface.

(We *can* optionally add a CLI tool and/or pybind11 bindings later.)

### 1.3 Required behavior
- Implement **operational DTM2020** (`dtm3`) and the **sigma uncertainty model** (`sigma_function`) as first-class C++ API.
- Implement **research DTM2020** (`dtm5`) as an optional feature flag (build option), because it is not required for MCM’s operational path but is part of the DTM2020 deliverable set.
- Preserve key I/O assumptions:
  - Inputs and outputs units match the documented Fortran interface.
  - Coefficient files parse with the same format (robust whitespace parsing; do not assume line endings).

### 1.4 Determinism vs performance
DTM2020 reference code performs core computations in **single precision** (`real`) and wrappers convert to/from `real(8)`.

To match reference results:
- Default internal numeric type should be **`float`** for the model core (coefficients + computations), with outputs promoted to `double` only at the API boundary.
- Add an opt-in **double-precision mode** for research/analysis (not default; results will differ).

---

## 2) Target C++ API design

### 2.1 Data types (strong, explicit, minimal allocation)

Create a small header-only “types” layer:

- `dtm2020::Degrees`, `dtm2020::Radians`
- `dtm2020::Hours`
- `dtm2020::Kilometers`
- `dtm2020::DayOfYear`

Use `explicit` constructors and `constexpr` conversions:
- degrees → radians
- hours → radians (0–24 h maps to 0–2π)

### 2.2 Operational input/output

```cpp
namespace dtm2020 {

struct OperationalInputs {
  double altitude_km;     // > 120
  double latitude_deg;    // [-90, 90]
  double longitude_deg;   // [0, 360] (or allow [-180,180] but normalize)
  double local_time_h;    // [0, 24)
  double day_of_year;     // [1, 366]
  double f107;            // daily flux (t - 24h)
  double f107m;           // 81-day mean
  double kp_delayed_3h;   // Kp delayed by 3 hours
  double kp_mean_24h;     // mean Kp over last 24h
};

struct Outputs {
  double temperature_K;       // tz
  double exospheric_temp_K;   // tinf
  double density_g_cm3;       // ro
  double mean_mol_mass_g;     // wmm
  double d_H_g_cm3;
  double d_He_g_cm3;
  double d_O_g_cm3;
  double d_N2_g_cm3;
  double d_O2_g_cm3;
  double d_N_g_cm3;
};

} // namespace dtm2020
```

### 2.3 Model object and thread safety
Create a **reentrant** model class that owns coefficients; **no global state**:

```cpp
class Dtm2020Operational {
public:
  struct Options {
    bool emulate_mcm_transition = true;   // preserves az2 attenuation for 120–150 km as in *_MCM.f90
    bool strict_fp = true;               // disable fp-contract, fast-math, etc. where possible
  };

  static expected<Dtm2020Operational, Error> LoadFromFile(std::filesystem::path coeff_file, Options);

  Outputs Evaluate(const OperationalInputs&) const;
  double DensityUncertaintyPercent(const OperationalInputs&) const; // sigma_function equivalent

private:
  Coefficients coeff_;   // parsed arrays (nlatm=96)
  Options opt_;
};
```

- `LoadFromFile()` parses coefficient file once.
- `Evaluate()` uses **stack/local temporaries only**.
- One model instance can be safely used from multiple threads if `Evaluate()` is const and contains no mutation.

### 2.4 Research model (optional)
Provide a parallel class:

```cpp
struct ResearchInputs {
  double altitude_km;
  double latitude_deg;
  double longitude_deg;
  double local_time_h;
  double day_of_year;
  double f30;     // daily (t-24h)
  double f30m;    // 81-day mean
  std::array<double, 10> ap60; // as documented
};

class Dtm2020Research { ... };
```

> Keep operational and research separated to avoid accidental mixing of proxies/indices.

### 2.5 Errors and contracts
Use a small `enum class ErrorCode` + `struct Error` and return `expected<T, Error>` from constructors/IO.

- Dependency: either a tiny internal `expected` (C++20) or `tl::expected` via CPM.
- Runtime input validation:
  - altitude <= 120 km → error
  - latitude outside [-90,90] → error (or clamp if explicitly configured)
  - local_time outside [0,24) → normalize mod 24
  - longitude normalize to [0,360)

---

## 3) Codebase structure (CMake-first, no containers)

### 3.1 Proposed repository layout
```
dtm2020-cpp/
  CMakeLists.txt
  CMakePresets.json
  cmake/
    CPM.cmake
    toolchains/   (optional)
    sanitizers.cmake
    warnings.cmake
  include/
    dtm2020/
      dtm2020_operational.hpp
      dtm2020_research.hpp
      types.hpp
      expected.hpp (if internal)
      version.hpp
  src/
    dtm2020_operational.cpp
    dtm2020_research.cpp      (optional)
    coeff_parser.cpp
    sigma_uncertainty.cpp
    fp_control.cpp
  tests/
    CMakeLists.txt
    test_coeff_parser.cpp
    test_operational_golden.cpp
    test_uncertainty_sigma.cpp
    test_research_golden.cpp   (optional)
  testdata/
    operational_vectors.csv
    research_vectors.csv
  examples/
    dtm2020_cli.cpp
  docs/
    licensing.md
    numerical_fidelity.md
    model_notes.md
  .clang-format
  .clang-tidy
  README.md
```

### 3.2 “Author” requirement
Every new source/header file must begin with a short header comment containing:

- `Author: waston`
- a one-line purpose summary

Example:

```cpp
// Author: waston
// Purpose: DTM2020 operational model public API and evaluation entrypoint.
```

(Keep it uniform, minimal, and not legalistic unless licensing demands more.)

---

## 4) Build system requirements (CMake + Presets + CPM)

### 4.1 CMake baseline
- Require **CMake ≥ 3.25** (good presets support).
- Enforce `CMAKE_CXX_STANDARD 20` and `CMAKE_CXX_STANDARD_REQUIRED ON`.
- Default build uses warnings-as-errors in CI; locally configurable.

### 4.2 Presets (macOS/Linux/Windows; Debug/Release/Sanitize/Profile)

Create **configure presets** and **build presets** for:

- `macos-debug`, `macos-release`, `macos-asan`, `macos-ubsan`, `macos-profile`
- `linux-debug`, `linux-release`, `linux-asan`, `linux-ubsan`, `linux-tsan`, `linux-profile`
- `windows-debug`, `windows-release`, `windows-asan` (clang-cl or MSVC ASan), `windows-profile`

Sanitizer and profile flags:
- Linux/macOS:
  - ASan: `-fsanitize=address,undefined -fno-omit-frame-pointer -g`
  - TSan: `-fsanitize=thread -fno-omit-frame-pointer -g`
  - Profile: `-O3 -g -fno-omit-frame-pointer` (leave profiling tool choice open)
- MSVC:
  - `/fsanitize=address` (where supported) + `/Zi`
  - `/fp:precise` for strict mode

### 4.3 CPM dependencies
Use CPM.cmake for:
- **GTest** (required)
- Optional:
  - `tl::expected` (if you don’t want an internal expected)
  - `fmt` (for CLI formatting; avoid in core library unless needed)
  - `CLI11` (for CLI example)

Keep the core library dependency-light.

### 4.4 Tooling hooks
- Add targets:
  - `dtm2020` (library)
  - `dtm2020_cli` (example executable)
  - `dtm2020_tests` (gtest)
- Enable `ctest` integration.
- Add optional targets:
  - `clang-tidy` via `CMAKE_CXX_CLANG_TIDY`
  - `format` via `clang-format` (script or CMake custom target)

---

## 5) Numerical fidelity strategy (how we match the Fortran)

### 5.1 Preserve float core
- Coefficient arrays stored as `float`.
- Core evaluation uses `float` temporaries and math functions (`std::sin`, `std::cos`, `std::exp`, `std::log`) applied to `float` or casted values.

### 5.2 Floating point model controls
To reduce cross-compiler drift:
- Disable contraction/FMA where possible:
  - GCC/Clang: `-ffp-contract=off`
- Avoid `-ffast-math`.
- On MSVC: `/fp:precise`.

Document this in `docs/numerical_fidelity.md` and provide an opt-out `DTM2020_STRICT_FP=OFF` for users who prefer speed over reproducibility.

### 5.3 Preserve evaluation order
The Fortran implementation’s output depends on:
- expression evaluation order,
- float rounding,
- use of common-block variables.

Implementation approach:
1. **Phase A: Mechanical translation**  
   Implement `dtm3`, `gldtm`, `lecdtm` (and optionally `dtm5`, `gldtm_Hp`, …) in C++ with variable names and order kept close to Fortran.
2. **Phase B: Encapsulation & cleanup**  
   Wrap into classes, add types, remove globals, add tests.
3. **Phase C: Micro-optimizations**  
   Only after tests lock in.

---

## 6) Implementation notes (operational DTM2020)

### 6.1 Coefficient loading (`lecdtm`)
In Fortran (`lecdtm(iu10)`), it reads:
- a title line (a100)
- `npdtm`
- then `npdtm` records, each containing:
  - `ni` and 9 pairs of floats (value, uncertainty placeholder), storing arrays:
    - `tt(i), h(i), he(i), o(i), az2(i), o2(i), az(i), t0(i), tp(i)`

C++ parser requirements:
- Token-based parse (`operator>>`) rather than line-based parse.
- Accept `E` or `D` exponents (normalize `D`→`E` if encountered).
- Validate:
  - `npdtm <= 96`
  - arrays fully populated
  - fail with useful error if not.

### 6.2 Core evaluate (`dtm3`)
Implement as:
- `EvaluateOperational(...)` that:
  - converts input units:
    - lat_deg → rad
    - lon_deg → rad
    - local_time_h → rad
  - computes Legendre polynomials (`p10`, `p20`, …) and geomagnetic equivalents (`p10mg`, …)
  - calls `gldtm` repeatedly to compute:
    - `tinf`, `t120`, `tp120`
    - base densities for constituents
  - computes temperature at altitude and densities

### 6.3 MCM-specific attenuation (must match if enabled)
`dtm2020_F107_Kp-subr_MCM.f90` modifies `az2` terms for 120–150 km:
- If `alti > 150`: use `az2` directly
- Else:
  - `atten = (alti - 120)^3 / 27000`
  - multiply indices `{21,22,23,26,27,28,31,33,35,36,90,91}` by `atten`
  - use this modified `az2_mcm` when calling `gldtm`

This must be replicated under `Options::emulate_mcm_transition = true`.

### 6.4 Density uncertainty (`sigma_function`)
Port `dtm2020_sigma_function.f90` exactly.

Key behavior:
- For altitude < 200: evaluate using altitude fixed at 200
- 200–500: evaluate at altitude
- >500: evaluate using altitude fixed at 500

Sigma components are polynomials:
- `std_FP_M(x)`: degree 1 with coefficients `[26.04, -3.36e-2]`
- `std_KP_M(x)`: degree 3 with `[22.47, 7.46e-1, -1.99e-1, 6.56e-2]`
- `std_alat_M(x)`: degree 2 with `[22.33, -7.19e-3, 4.72e-4]`
- `std_hl_M(x)`: degree 3 with `[28.53, -3.95e-1, -5.64e-2, 3.15e-3]`
- `std_alti_M(x)`: degree 1 with `[-9.73, 7.92e-2]`
- `std_day_M(x)`: degree 1 with `[23.15, -3.91e-3]`
- final: `stdev = sum(sigmas) + Copt - Res` where `Copt=0.929`, `Res=112.79`

---

## 7) Testing strategy (GTest, golden vectors, strictness)

### 7.1 Golden-vector philosophy
To claim parity, we need a **frozen set of reference outputs** computed from the reference Fortran.

Recommended workflow:
1. Build the original Fortran DTM2020 operational model (from `mcm`) and run a “vector generator” program that prints:
   - inputs
   - `tz, tinf, ro, d(1..6), wmm`
2. Save these as `testdata/operational_vectors.csv`.
3. In C++ tests:
   - parse the CSV
   - evaluate the model
   - compare outputs.

### 7.2 Tolerances and exactness
- If `strict_fp` is enabled and you use float core, you *may* be able to use very tight tolerances.
- Expect cross-platform drift; use:
  - `EXPECT_NEAR` with relative tolerance for densities and partial densities (these can be small).
  - `EXPECT_NEAR` absolute tolerance for temperature.

Recommended starting tolerances (adjust once you see real deltas):
- temperature: `1e-4 K` (or tighter)
- density and partial densities: relative `1e-6`, absolute floor `1e-18 g/cm3`
- mean molecular mass: `1e-7 g`

### 7.3 Deterministic unit tests (no Fortran dependency)
The sigma uncertainty function is purely polynomial + branching. Add **exact** tests:
- choose representative `(lat, local_time, doy, altitude, f107m, kp)` values
- compute expected results using a small independent implementation in the test (or precomputed constants)
- compare with `EXPECT_FLOAT_EQ` if using float.

### 7.4 Edge-case tests
- Invalid altitude (<=120) should return error.
- Longitude normalization: -10 → 350; 370 → 10.
- Local time normalization: 24 → 0; -1 → 23.
- Day-of-year handling: enforce [1,366] or allow wrapping by config (document choice).

### 7.5 Optional research model tests
Same golden-vector method using `dtm5` and its coefficient file.

---

## 8) Performance and robustness enhancements (after parity)

### 8.1 Batch API
Add a batch evaluation interface for orbit propagators:

```cpp
void EvaluateBatch(std::span<const OperationalInputs> in,
                   std::span<Outputs> out) const;
```

- No heap allocation.
- Precompute any shared trigonometric values if you detect identical lat/lon/time patterns (optional).

### 8.2 Avoiding repeated trig for same point
Offer an advanced API:
- `PrecomputedLocation` containing:
  - `sin/cos(lat)`, `sin/cos(lon)`, Legendre polynomials, etc.
- Then allow:
  - `Evaluate(precomputed_location, drivers)`

Only do this if profiling shows it matters.

### 8.3 Defensive coding
- Validate coefficient arrays are initialized.
- Use `std::array` for fixed-size arrays.
- Use bounds-checked accessor in debug builds for Fortran-indexed arrays.

---

## 9) CI (optional but recommended for “flight software” quality)

Set up GitHub Actions (or your internal CI) with:
- Linux: Debug + ASan/UBSan, run tests
- macOS: Debug + Release, run tests
- Windows: Debug + Release, run tests

Add quality gates:
- clang-tidy (warning-level manageable; start small)
- formatting check
- compile with `-Werror` in CI only

---

## 10) Commit discipline (small, reviewable commits)

**Rule:** small commits, each logically isolated, each with a concise message, **never mention “codex”** in commit messages.

Suggested commit sequence:

1. **Scaffold CMake project**
   - Add `CMakeLists.txt`, `CMakePresets.json`, `cmake/CPM.cmake`
   - Add `.clang-format`, `.clang-tidy`
   - Commit message: `Scaffold C++20 CMake project`

2. **Add dependencies + test harness**
   - CPM + GTest, `tests/` skeleton, `ctest`
   - Commit message: `Add GTest and test scaffold`

3. **Add core types + error handling**
   - `types.hpp`, `expected.hpp` (or CPM `tl::expected`)
   - Commit message: `Add core types and error utilities`

4. **Implement coefficient parser**
   - Parse `DTM_2020_F107_Kp.dat`
   - Unit tests for parser (structure, counts)
   - Commit message: `Implement DTM2020 coefficient loader`

5. **Mechanical port of dtm3/gldtm**
   - Pure operational compute path (no sigma yet)
   - Golden tests added but initially marked `DISABLED_` until vectors exist
   - Commit message: `Port DTM2020 operational core`

6. **Implement MCM attenuation option**
   - Toggleable behavior in `Options`
   - Commit message: `Add MCM transition attenuation option`

7. **Implement sigma uncertainty**
   - Port sigma polynomials + tests
   - Commit message: `Add density uncertainty model`

8. **Add golden vectors and enable golden tests**
   - Add `testdata/operational_vectors.csv`
   - Enable tests
   - Commit message: `Add operational golden vectors`

9. **(Optional) Port research model**
   - `dtm5` + coefficients + tests
   - Commit message: `Add DTM2020 research model`

10. **CLI example**
   - `examples/dtm2020_cli.cpp`
   - Commit message: `Add dtm2020 CLI example`

11. **Documentation**
   - `docs/numerical_fidelity.md`, `docs/model_notes.md`
   - Commit message: `Add documentation for fidelity and usage`

### Practical git workflow per commit
For each step:
- edit only the minimal set of files
- run: `cmake --preset <platform-config>` and `ctest --preset <...>` as applicable
- stage narrowly: `git add <files>`
- commit with a short message
- keep diffs small enough to review

Also set local author identity once:
```bash
git config user.name "waston"
git config user.email "waston@example.com"
```

---

## 11) Acceptance criteria (definition of done)

- ✅ Library builds on macOS/Linux/Windows via CMake presets:
  - Debug, Release, Sanitizer (where supported), Profile
- ✅ All unit tests pass via `ctest` on all supported platforms.
- ✅ Operational model:
  - loads `DTM_2020_F107_Kp.dat`
  - produces outputs matching golden vectors within agreed tolerances
- ✅ Sigma uncertainty matches reference calculations for representative points
- ✅ No global mutable state; model object is re-entrant and thread-safe
- ✅ Clear API docs in `README.md` + examples
- ✅ No Docker/devcontainer files present

---

## Appendix A — Reference vector generation (recommended)

Create a one-off small Fortran (or C++/Fortran) tool in a separate private area:
- loads coefficients
- evaluates `dtm3` for a curated grid:
  - altitudes: 120, 130, 150, 200, 300, 500, 1000
  - latitudes: -80, -45, 0, 45, 80
  - local time: 0, 6, 12, 18
  - day-of-year: 1, 80, 172, 266, 355
  - solar activity: low/med/high f107/f107m
  - geomagnetic: kp low/high
- saves CSV

This creates a robust regression net for future refactors/optimizations.

---

## Appendix B — Notes on robustness for “flight software” use

If this will run in a safety-critical pipeline:
- add input validation policy (error vs clamp) and document it
- consider fixed-point/log-space handling if extreme values appear
- add explicit overflow/underflow handling for exponentials
- run sanitizers and fuzz the parser
- add a configuration knob to disable exceptions and use error codes only
