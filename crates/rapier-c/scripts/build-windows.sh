#!/usr/bin/env bash
# build-windows.sh — x86_64 Windows (MSVC) rapier_c.lib → rapier-windows-x64-Release.tar.gz
# Runs under `shell: bash` (git-bash) on windows-latest; cargo/rustup/tar are all on PATH.
# The MSVC staticlib is named rapier_c.lib (no lib prefix), which is what CMakeLists expects
# via ${CMAKE_STATIC_LIBRARY_PREFIX}rapier_c${CMAKE_STATIC_LIBRARY_SUFFIX} on Windows.
# Usage: bash scripts/build-windows.sh [OUT_DIR]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/_stage.sh"
OUT="${1:-out}"; mkdir -p "$OUT"

rustup target add x86_64-pc-windows-msvc
build_lib x86_64-pc-windows-msvc

STAGE="$(mktemp -d)/windows"; mkdir -p "$STAGE/lib"
cp "$LIB_A" "$STAGE/lib/rapier_c.lib"
LIB_A="$STAGE/lib/rapier_c.lib"
stage_tree "$STAGE" windows x86_64

tar -C "$STAGE" -czf "$OUT/rapier-windows-x64-Release.tar.gz" lib include MANIFEST.txt
echo "[rapier-c] wrote $OUT/rapier-windows-x64-Release.tar.gz"
