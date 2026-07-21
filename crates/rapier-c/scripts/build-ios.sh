#!/usr/bin/env bash
# build-ios.sh — iOS device (arm64) + simulator (arm64 + x86_64, lipo'd) static libs →
# rapier-ios-static-Release.tar.gz. Two self-contained slices under device/ and simulator/,
# each with its own lib/ + include/, matching how CMakeLists selects
# libs/rapier/ios/{device,simulator} (lib/librapier_c.a + include/) by CMAKE_OSX_SYSROOT.
# Usage: bash scripts/build-ios.sh [OUT_DIR]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/_stage.sh"
OUT="${1:-out}"; mkdir -p "$OUT"

rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios

STAGE="$(mktemp -d)/ios"

# device slice: arm64 only.
build_lib aarch64-apple-ios
mkdir -p "$STAGE/device/lib"
cp "$LIB_A" "$STAGE/device/lib/librapier_c.a"
LIB_A="$STAGE/device/lib/librapier_c.a"
stage_tree "$STAGE/device" ios-device arm64

# simulator slice: fat arm64 + x86_64.
build_lib aarch64-apple-ios-sim; SIM_ARM="$LIB_A"; NL_SIM="$NATIVE_LIBS"
build_lib x86_64-apple-ios;      SIM_X64="$LIB_A"
mkdir -p "$STAGE/simulator/lib"
lipo -create "$SIM_ARM" "$SIM_X64" -output "$STAGE/simulator/lib/librapier_c.a"
LIB_A="$STAGE/simulator/lib/librapier_c.a"; NATIVE_LIBS="$NL_SIM"
stage_tree "$STAGE/simulator" ios-simulator arm64+x86_64

tar -C "$STAGE" -czf "$OUT/rapier-ios-static-Release.tar.gz" device simulator
echo "[rapier-c] wrote $OUT/rapier-ios-static-Release.tar.gz"
