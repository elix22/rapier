#!/usr/bin/env bash
# build-macos.sh — universal (arm64 + x86_64) macOS librapier_c.a → rapier-macos-universal-Release.tar.gz
# Usage: bash scripts/build-macos.sh [OUT_DIR]   (OUT_DIR default: out)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/_stage.sh"
OUT="${1:-out}"; mkdir -p "$OUT"

rustup target add aarch64-apple-darwin x86_64-apple-darwin
build_lib aarch64-apple-darwin; ARM64="$LIB_A"; NL="$NATIVE_LIBS"
build_lib x86_64-apple-darwin;  X64="$LIB_A"

STAGE="$(mktemp -d)/macos"; mkdir -p "$STAGE/lib"
lipo -create "$ARM64" "$X64" -output "$STAGE/lib/librapier_c.a"
LIB_A="$STAGE/lib/librapier_c.a"; NATIVE_LIBS="$NL"
stage_tree "$STAGE" macos universal

tar -C "$STAGE" -czf "$OUT/rapier-macos-universal-Release.tar.gz" lib include MANIFEST.txt
echo "[rapier-c] wrote $OUT/rapier-macos-universal-Release.tar.gz"
