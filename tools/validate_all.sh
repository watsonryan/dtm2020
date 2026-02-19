#!/usr/bin/env bash
set -euo pipefail

# Author: Watson
# Purpose: Run full local validation for dtm2020 operational and research implementations.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "[1/5] Configure+build+test macos-debug"
cmake --preset macos-debug
cmake --build --preset macos-debug
ctest --preset macos-debug --output-on-failure

echo "[2/5] Configure+build+test macos-asan"
cmake --preset macos-asan
cmake --build --preset macos-asan
ctest --preset macos-asan --output-on-failure

echo "[3/5] Regenerate operational Fortran vectors"
if command -v gfortran >/dev/null 2>&1; then
  tools/generate_operational_vectors_fortran.sh
else
  echo "warning: gfortran not found; skipping Fortran vector generation"
fi

echo "[4/5] Re-run golden tests (debug)"
ctest --preset macos-debug --output-on-failure -R "operational_golden|research_benchmark"

echo "[5/6] Validate install and downstream package consumption"
tools/validate_package_install.sh

echo "[6/6] Validation complete"
