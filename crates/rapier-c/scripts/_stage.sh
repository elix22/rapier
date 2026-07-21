# _stage.sh — shared helpers for the rapier-c prebuilt build scripts (Phase 19 P19.3).
#
# Each build-<platform>.sh sources this, then calls build_lib for one or more Rust targets
# and stage_tree to lay out lib/ + include/ + MANIFEST.txt in the EXACT shape the runtime's
# tools/fetch-libs.mjs unpacks and CMakeLists consumes — identical to what the local
# tools/build-rapier-lib.mjs stages. The tarball layout per platform:
#   macos / linux / windows :  <root>/{lib,include,MANIFEST.txt}
#   android                 :  <root>/<abi>/{lib,include,MANIFEST.txt}   (abi x3)
#   ios                     :  <root>/{device,simulator}/{lib,include}   + <root>/MANIFEST.txt
#
# KEY FACT that keeps CI cheap and robust: rapier-c is a `staticlib` crate, so rustc emits
# an ARCHIVE (its own built-in archive writer) and never invokes a platform linker. Cross
# targets therefore need only `rustup target add <triple>` — no NDK, no iOS SDK sysroot —
# except Android, where cargo-ndk is used purely as insurance for any transitive build.rs.
set -euo pipefail

crate_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CARGO="${CARGO:-cargo}"
fork_commit="$(git -C "$crate_dir" rev-parse HEAD 2>/dev/null || echo unknown)"

LIB_A=""        # set by build_lib: absolute path to the produced librapier_c.a / rapier_c.lib
NATIVE_LIBS=""  # set by build_lib: that target's `--print native-static-libs` line (for MANIFEST)

# build_lib TARGET [extra cargo args...]
# Cross-compiles the release staticlib for TARGET, capturing the platform link set. Always
# passes --target so the output path is deterministic (target/<triple>/release/).
build_lib() {
  local target="$1"; shift
  local log; log="$(mktemp)"
  echo "[rapier-c] cargo rustc --release --target $target ..."
  "$CARGO" rustc --release --manifest-path "$crate_dir/Cargo.toml" \
    --target "$target" "$@" -- --print native-static-libs > "$log" 2>&1 || {
      echo "[rapier-c] cargo build FAILED for $target:"; cat "$log"; rm -f "$log"; exit 1; }
  NATIVE_LIBS="$(sed -n 's/.*native-static-libs: *//p' "$log" | head -1)"
  rm -f "$log"
  local a="$crate_dir/target/$target/release/librapier_c.a"
  [ -f "$a" ] || a="$crate_dir/target/$target/release/rapier_c.lib"   # MSVC name
  [ -f "$a" ] || { echo "[rapier-c] no staticlib produced for $target"; exit 1; }
  LIB_A="$a"
}

# stage_tree DEST_DIR PLATFORM_LABEL ARCH_LABEL
# Copies the header + writes MANIFEST.txt into DEST_DIR. The caller has already copied the
# staticlib into DEST_DIR/lib (LIB_A is used only for the MANIFEST's `lib:` basename).
stage_tree() {
  local dest="$1" platform="$2" arch="$3"
  mkdir -p "$dest/include"
  cp "$crate_dir/include/rapier_c.h" "$dest/include/rapier_c.h"
  cat > "$dest/MANIFEST.txt" <<EOF
lib: $(basename "$LIB_A") (C ABI over native rapier3d 0.34.0, f32)
source-fork: elix22/rapier @ $fork_commit
crate: crates/rapier-c
built: CI (build-rapier-c.yml)
platform: $platform ($arch)
native-static-libs: ${NATIVE_LIBS:-(none)}
EOF
}
