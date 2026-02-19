#!/usr/bin/env bash
set -euo pipefail

# Author: Watson
# Purpose: Validate install + downstream find_package(dtm2020) consumption.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INSTALL_DIR="${DTM2020_INSTALL_PREFIX:-/tmp/dtm2020-install}"
CONSUMER_SRC="$ROOT/tests/package_consumer"
CONSUMER_BUILD="$ROOT/build/package-consumer"

if [[ -n "${DTM2020_RELEASE_PRESET:-}" ]]; then
  RELEASE_PRESET="$DTM2020_RELEASE_PRESET"
else
  case "$(uname -s)" in
    Darwin) RELEASE_PRESET="macos-release" ;;
    Linux) RELEASE_PRESET="linux-release" ;;
    MINGW*|MSYS*|CYGWIN*) RELEASE_PRESET="windows-release" ;;
    *)
      echo "error: unsupported platform for default release preset; set DTM2020_RELEASE_PRESET" >&2
      exit 1
      ;;
  esac
fi

BUILD_DIR="$ROOT/build/$RELEASE_PRESET"

cmake --preset "$RELEASE_PRESET"
cmake --build --preset "$RELEASE_PRESET"
cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"

cmake -S "$CONSUMER_SRC" -B "$CONSUMER_BUILD" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$INSTALL_DIR"

cmake --build "$CONSUMER_BUILD"
"$CONSUMER_BUILD/dtm2020_package_consumer"

echo "package install/consumer validation passed"
