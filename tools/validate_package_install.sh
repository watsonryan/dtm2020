#!/usr/bin/env bash
set -euo pipefail

# Author: Watson
# Purpose: Validate install + downstream find_package(dtm2020) consumption.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build/macos-release"
INSTALL_DIR="${DTM2020_INSTALL_PREFIX:-/tmp/dtm2020-install}"
CONSUMER_SRC="$ROOT/tests/package_consumer"
CONSUMER_BUILD="$ROOT/build/package-consumer"

cmake --preset macos-release
cmake --build --preset macos-release
cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"

cmake -S "$CONSUMER_SRC" -B "$CONSUMER_BUILD" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$INSTALL_DIR"

cmake --build "$CONSUMER_BUILD"
"$CONSUMER_BUILD/dtm2020_package_consumer"

echo "package install/consumer validation passed"
