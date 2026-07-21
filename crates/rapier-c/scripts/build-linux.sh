#!/usr/bin/env bash
# build-linux.sh — x86_64 Linux (glibc) librapier_c.a → rapier-linux-x64-Release.tar.gz
# Usage: bash scripts/build-linux.sh [OUT_DIR]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/_stage.sh"
OUT="${1:-out}"; mkdir -p "$OUT"

rustup target add x86_64-unknown-linux-gnu
build_lib x86_64-unknown-linux-gnu

STAGE="$(mktemp -d)/linux"; mkdir -p "$STAGE/lib"
cp "$LIB_A" "$STAGE/lib/librapier_c.a"
LIB_A="$STAGE/lib/librapier_c.a"
stage_tree "$STAGE" linux x86_64

tar -C "$STAGE" -czf "$OUT/rapier-linux-x64-Release.tar.gz" lib include MANIFEST.txt
echo "[rapier-c] wrote $OUT/rapier-linux-x64-Release.tar.gz"
