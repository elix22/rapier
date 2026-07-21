# rapier-c

A stable **C ABI over the native Rust `rapier3d` (0.34.0, f32) solver**, for
[`threejs-native-runtime`](https://github.com/elix22/threejs-native-runtime) Phase 19.

It builds to `librapier_c.a` + `include/rapier_c.h`. The runtime's QuickJS host links the
archive and installs `globalThis.__rapier`; a JS shim then presents the exact
`@dimforge/rapier3d-compat` API, so Three.js app code runs **unchanged** on native (this
lib) and on the web (the real wasm compat). This replaces running Rapier's WebAssembly
under WAMR — native ARM code has none of the armv7 WAMR-AOT alignment fragility that made
the wasm path a dead end on low-tier devices.

## Layout

| path | what |
|---|---|
| `include/rapier_c.h` | the C ABI (the contract) |
| `src/lib.rs`         | `#[no_mangle] extern "C"` wrappers over rapier's `PhysicsWorld` |
| `tests/smoke.c`      | C harness: free-fall, resting height, raycast (the P19.1 gate) |
| `Cargo.toml`         | self-contained workspace; `staticlib`; `rapier3d = "=0.34.0"` |

## Build & test

```sh
cargo build --release                       # -> target/release/librapier_c.a
# or, from the runtime repo, the full gate (build + link C harness + run):
tools/test-rapier-native.sh
```

Needs a Rust toolchain (`rustup`); nothing else. CI in this fork cross-compiles the lib
for every runtime platform and publishes it as a Release asset (Phase 19 P19.3).

## Versioning

The native crate (`rapier3d` 0.34.0) and the npm `@dimforge/rapier3d-compat` (capped at
0.19.3) track **separately**, so the two backends necessarily run different solver
versions. The shim translates 0.34's API into the compat shape, and the shared physics
gates use tolerances (3% / 0.03 m) to absorb the small cross-version divergence.
`Cargo.lock` is committed so the built library is reproducible.
