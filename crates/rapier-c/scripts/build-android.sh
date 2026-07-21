#!/usr/bin/env bash
# build-android.sh — per-ABI static libs (arm64-v8a, armeabi-v7a, x86_64) →
# rapier-android-Release.tar.gz, laid out <abi>/{lib,include,MANIFEST.txt} the way
# CMakeLists reads libs/rapier/android/${ANDROID_ABI}. armeabi-v7a is the phase's headline
# target (the Redmi 6A) — native ARM code has none of the WAMR-AOT alignment fragility.
#
# rapier-c is a pure-Rust `staticlib` (the committed Cargo.lock has no -sys / cc deps), so
# rustc emits the archive with its own writer and never links — no NDK is required, only
# `rustup target add`. Usage: bash scripts/build-android.sh [OUT_DIR]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/_stage.sh"
OUT="${1:-out}"; mkdir -p "$OUT"

# abi -> rust triple
abis=(arm64-v8a armeabi-v7a x86_64)
triples=(aarch64-linux-android armv7-linux-androideabi x86_64-linux-android)

rustup target add "${triples[@]}"

STAGE="$(mktemp -d)/android"
for i in "${!abis[@]}"; do
  abi="${abis[$i]}"; triple="${triples[$i]}"
  build_lib "$triple"
  mkdir -p "$STAGE/$abi/lib"
  cp "$LIB_A" "$STAGE/$abi/lib/librapier_c.a"
  LIB_A="$STAGE/$abi/lib/librapier_c.a"
  stage_tree "$STAGE/$abi" "android-$abi" "$triple"
done

tar -C "$STAGE" -czf "$OUT/rapier-android-Release.tar.gz" "${abis[@]}"
echo "[rapier-c] wrote $OUT/rapier-android-Release.tar.gz"
