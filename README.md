# dtm2020-cpp

Author: Watson

C++20 re-implementation workspace for DTM2020 operational model and uncertainty function.

## Build

```bash
cmake --preset macos-debug
cmake --build --preset macos-debug
ctest --preset macos-debug
```

## CLI Example

```bash
./build/macos-debug/dtm2020_cli <coeff_file> <alt_km> <lat_deg> <lon_deg> <lt_h> <doy> <f107> <f107m> <kp3h> <kp24h>
```
