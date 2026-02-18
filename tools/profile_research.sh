#!/usr/bin/env bash
set -euo pipefail

# Author: Watson
# Purpose: Build profile preset and run research throughput benchmark.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

cmake --preset macos-profile
cmake --build --preset macos-profile -j
./build/macos-profile/tests/dtm2020_research_perf_benchmark
