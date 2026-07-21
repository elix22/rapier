//! rapier-c — a `#[no_mangle] extern "C"` shim exposing the native Rust `rapier3d`
//! (0.34.0, f32) solver as the flat C ABI declared in `include/rapier_c.h`. The
//! QuickJS host (`src/host/rapier_bind.c`) links the resulting `librapier_c.a` and
//! installs `globalThis.__rapier`; a JS shim then presents the exact
//! `@dimforge/rapier3d-compat` surface so Three.js app code is unchanged. See
//! `phase19-native-rapier-plan.md`.
//!
//! DESIGN NOTES
//! - The C-side "world" is `RprWorld`, a `Box`ed Rust struct that owns rapier's
//!   `PhysicsWorld` convenience bundle (all the sets + pipelines) plus vectors of the
//!   vehicle and character controllers (which are NOT rapier slotmap objects, so the
//!   world owns them and hands out 1-based integer handles).
//! - Rigid-body / collider handles cross the ABI as `u64` = `(index << 32) | generation`
//!   of rapier's own slotmap handle. 0 is reserved as "invalid" (rapier generations
//!   start at ≥1, so a live handle never packs to 0).
//! - rapier 0.34 uses glam math (`Vector` = `Vec3`, `Rotation` = `Quat`), so vectors and
//!   quaternions read/write via `.x/.y/.z[/.w]`. Everything crosses as `f32` arrays.
//! - `panic = "abort"` (Cargo.toml) means a Rust panic aborts rather than unwinding into
//!   C (which would be UB). Every accessor is written to no-op on a bad handle instead of
//!   panicking, so a stale handle from JS degrades gracefully rather than aborting.
//! - Each entry point binds an explicit `&`/`&mut` to the world pointer up front (`let rw
//!   = &mut *w`) rather than method-calling through `(*w).field`, which the
//!   `dangerous_implicit_autorefs` lint (rightly) rejects.

// Handles/descriptors cross as raw pointers to opaque or repr(C) types; that is exactly
// what an FFI boundary is, so silence the pedantic ctypes lint for the whole crate.
#![allow(improper_ctypes_definitions)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::c_char;
use rapier3d::control::{
    CharacterAutostep, CharacterLength, DynamicRayCastVehicleController,
    KinematicCharacterController, Wheel, WheelTuning,
};
use rapier3d::data::Index;
use rapier3d::dynamics::{
    FixedJointBuilder, GenericJoint, ImpulseJointHandle, JointAxis, PrismaticJointBuilder,
    RevoluteJointBuilder, RopeJointBuilder, SphericalJointBuilder, SpringJointBuilder,
};
use rapier3d::pipeline::{ActiveEvents, ChannelEventCollector};
use rapier3d::prelude::*;
use std::sync::mpsc::{channel, Receiver, Sender};

/// Invalid / "none" handle. u64::MAX matches rapier's own `Handle::invalid()`
/// (index=generation=0xFFFFFFFF). Crucially it is NOT 0: rapier's first handle is
/// (index 0, generation 0), which packs to 0, so 0 is a perfectly valid handle.
const RPR_INVALID: u64 = u64::MAX;

// ---------------------------------------------------------------------------------------
// World handle
// ---------------------------------------------------------------------------------------

/// The character controller plus the last movement it computed (the compat API splits
/// `computeColliderMovement()` from `computedMovement()`, so we stash the result).
struct CharState {
    controller: KinematicCharacterController,
    last: Vector,
    grounded: bool,
}

/// One drained collision event (compat drainCollisionEvents(h1, h2, started)).
struct CollEvt {
    c1: u64,
    c2: u64,
    started: i32,
}
/// One drained contact-force event (compat drainContactForceEvents(event)).
struct ForceEvt {
    c1: u64,
    c2: u64,
    total_mag: f32,
    max_mag: f32,
    dir: [f32; 3],
}

/// Everything one compat `World` owns. Kept behind a `Box`; the C side sees `RprWorld*`.
pub struct RprWorld {
    world: PhysicsWorld,
    vehicles: Vec<DynamicRayCastVehicleController>,
    characters: Vec<CharState>,
    // Collision/contact-force events. The channel pair is created once; each
    // rpr_world_step_events builds a fresh ChannelEventCollector from clones of the senders,
    // steps, then drains the receivers into the pending vecs for the C side to read back.
    coll_send: Sender<CollisionEvent>,
    coll_recv: Receiver<CollisionEvent>,
    force_send: Sender<ContactForceEvent>,
    force_recv: Receiver<ContactForceEvent>,
    pending_coll: Vec<CollEvt>,
    pending_force: Vec<ForceEvt>,
}

// ---------------------------------------------------------------------------------------
// repr(C) descriptors — MUST match the structs in include/rapier_c.h field-for-field.
// ---------------------------------------------------------------------------------------

#[repr(C)]
pub struct RprBodyDesc {
    body_type: i32, // 0 dynamic, 1 fixed, 2 kinematic-position
    translation: [f32; 3],
    rotation: [f32; 4], // x, y, z, w
    linear_damping: f32,
    angular_damping: f32,
    can_sleep: i32,
}

#[repr(C)]
pub struct RprColliderDesc {
    shape_type: i32, // 0 ball, 1 cuboid, 2 capsule, 3 trimesh
    ball_radius: f32,
    cuboid_half: [f32; 3],
    capsule_half_height: f32,
    capsule_radius: f32,
    border_radius: f32,
    trimesh_vertices: *const f32,
    trimesh_vertex_count: u32,
    trimesh_indices: *const u32,
    trimesh_index_count: u32,
    density: f32,
    has_density: i32,
    friction: f32,
    has_friction: i32,
    restitution: f32,
    has_restitution: i32,
    translation: [f32; 3],
    has_translation: i32,
    collision_groups: u32,
    has_collision_groups: i32,
    sensor: i32,
    has_sensor: i32,
    active_events: u32,
    has_active_events: i32,
    contact_force_threshold: f32,
    has_contact_force_threshold: i32,
    mass: f32,
    has_mass: i32,
}

#[repr(C)]
pub struct RprJointDesc {
    joint_type: i32, // 0 revolute, 1 fixed, 2 prismatic, 3 spring, 4 rope, 5 spherical
    anchor1: [f32; 3],
    anchor2: [f32; 3],
    axis: [f32; 3],
    rest_length: f32,
    stiffness: f32,
    damping: f32,
    contacts_enabled: i32,
    has_limits: i32,
    limit_min: f32,
    limit_max: f32,
    motor_kind: i32, // 0 none, 1 position, 2 velocity
    motor_target: f32,
    motor_stiffness: f32,
    motor_damping: f32,
}

// ---------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------

#[inline]
fn rb_to_u64(h: RigidBodyHandle) -> u64 {
    let (i, g) = h.into_raw_parts();
    ((i as u64) << 32) | g as u64
}
#[inline]
fn rb_from_u64(v: u64) -> RigidBodyHandle {
    RigidBodyHandle::from_raw_parts((v >> 32) as u32, v as u32)
}
#[inline]
fn co_to_u64(h: ColliderHandle) -> u64 {
    let (i, g) = h.into_raw_parts();
    ((i as u64) << 32) | g as u64
}
#[inline]
fn co_from_u64(v: u64) -> ColliderHandle {
    ColliderHandle::from_raw_parts((v >> 32) as u32, v as u32)
}
#[inline]
fn ij_to_u64(h: ImpulseJointHandle) -> u64 {
    let (i, g) = h.0.into_raw_parts();
    ((i as u64) << 32) | g as u64
}
#[inline]
fn ij_from_u64(v: u64) -> ImpulseJointHandle {
    ImpulseJointHandle(Index::from_raw_parts((v >> 32) as u32, v as u32))
}
#[inline]
fn axis_from_i32(a: i32) -> JointAxis {
    match a {
        1 => JointAxis::LinY,
        2 => JointAxis::LinZ,
        3 => JointAxis::AngX,
        4 => JointAxis::AngY,
        5 => JointAxis::AngZ,
        _ => JointAxis::LinX,
    }
}

#[inline]
unsafe fn rd3(p: *const f32) -> Vector {
    Vector::new(*p, *p.add(1), *p.add(2))
}
#[inline]
unsafe fn wr3(out: *mut f32, v: Vector) {
    *out = v.x;
    *out.add(1) = v.y;
    *out.add(2) = v.z;
}
#[inline]
unsafe fn wr4(out: *mut f32, q: &Rotation) {
    *out = q.x;
    *out.add(1) = q.y;
    *out.add(2) = q.z;
    *out.add(3) = q.w;
}

/// Unpack the compat single-u32 collision-groups value (`membership << 16 | filter`)
/// into rapier's `InteractionGroups`. The classic AND test mode matches the wasm compat.
#[inline]
fn unpack_groups(packed: u32) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_truncate(packed >> 16),
        Group::from_bits_truncate(packed & 0xFFFF),
        InteractionTestMode::And,
    )
}

/// Borrow one wheel of one vehicle (1-based `vc` handle, 0-based wheel index). The
/// returned lifetime is unconstrained (reborrow through a raw pointer); callers use it
/// only within the same FFI call, during which the world outlives it.
#[inline]
unsafe fn wheel_mut<'a>(w: *mut RprWorld, vc: u64, i: u32) -> Option<&'a mut Wheel> {
    let rw = &mut *w;
    let idx = (vc as usize).wrapping_sub(1);
    rw.vehicles
        .get_mut(idx)
        .and_then(|v| v.wheels_mut().get_mut(i as usize))
}

// ---------------------------------------------------------------------------------------
// World lifecycle
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create(gravity: *const f32) -> *mut RprWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = rd3(gravity);
    let (coll_send, coll_recv) = channel();
    let (force_send, force_recv) = channel();
    Box::into_raw(Box::new(RprWorld {
        world,
        vehicles: Vec::new(),
        characters: Vec::new(),
        coll_send,
        coll_recv,
        force_send,
        force_recv,
        pending_coll: Vec::new(),
        pending_force: Vec::new(),
    }))
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_free(w: *mut RprWorld) {
    if !w.is_null() {
        drop(Box::from_raw(w));
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_timestep(w: *const RprWorld) -> f32 {
    let rw = &*w;
    rw.world.integration_parameters.dt
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_set_timestep(w: *mut RprWorld, dt: f32) {
    let rw = &mut *w;
    rw.world.integration_parameters.dt = dt;
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_gravity(w: *const RprWorld, out: *mut f32) {
    let rw = &*w;
    wr3(out, rw.world.gravity);
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_set_gravity(w: *mut RprWorld, v: *const f32) {
    let rw = &mut *w;
    rw.world.gravity = rd3(v);
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_step(w: *mut RprWorld) {
    let rw = &mut *w;
    rw.world.step();
}

// ---------------------------------------------------------------------------------------
// Collision & contact-force events (compat EventQueue + world.step(eventQueue))
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_step_events(w: *mut RprWorld) {
    let rw = &mut *w;
    rw.pending_coll.clear();
    rw.pending_force.clear();
    // Fresh collector from clones of the persistent senders — the receivers stay on the world.
    let collector = ChannelEventCollector::new(rw.coll_send.clone(), rw.force_send.clone());
    rw.world.step_with_events(&(), &collector);
    while let Ok(ev) = rw.coll_recv.try_recv() {
        let (c1, c2, started) = match ev {
            CollisionEvent::Started(a, b, _) => (co_to_u64(a), co_to_u64(b), 1),
            CollisionEvent::Stopped(a, b, _) => (co_to_u64(a), co_to_u64(b), 0),
        };
        rw.pending_coll.push(CollEvt { c1, c2, started });
    }
    while let Ok(ev) = rw.force_recv.try_recv() {
        rw.pending_force.push(ForceEvt {
            c1: co_to_u64(ev.collider1),
            c2: co_to_u64(ev.collider2),
            total_mag: ev.total_force_magnitude,
            max_mag: ev.max_force_magnitude,
            dir: [ev.max_force_direction.x, ev.max_force_direction.y, ev.max_force_direction.z],
        });
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_num_collision_events(w: *const RprWorld) -> u32 {
    (*w).pending_coll.len() as u32
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_collision_event(
    w: *const RprWorld,
    i: u32,
    out_handles: *mut u64,
    out_started: *mut i32,
) {
    let rw = &*w;
    if let Some(e) = rw.pending_coll.get(i as usize) {
        *out_handles = e.c1;
        *out_handles.add(1) = e.c2;
        *out_started = e.started;
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_num_contact_force_events(w: *const RprWorld) -> u32 {
    (*w).pending_force.len() as u32
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_contact_force_event(
    w: *const RprWorld,
    i: u32,
    out_handles: *mut u64,
    out5: *mut f32,
) {
    let rw = &*w;
    if let Some(e) = rw.pending_force.get(i as usize) {
        *out_handles = e.c1;
        *out_handles.add(1) = e.c2;
        *out5 = e.total_mag;
        *out5.add(1) = e.max_mag;
        *out5.add(2) = e.dir[0];
        *out5.add(3) = e.dir[1];
        *out5.add(4) = e.dir[2];
    }
}

// rapier3d crate version pinned in Cargo.toml. Static, NUL-terminated.
static RAPIER_VERSION: &[u8] = b"0.34.0\0";

#[no_mangle]
pub extern "C" fn rpr_version() -> *const c_char {
    RAPIER_VERSION.as_ptr() as *const c_char
}

// ---------------------------------------------------------------------------------------
// Rigid bodies
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create_rigid_body(w: *mut RprWorld, d: *const RprBodyDesc) -> u64 {
    let rw = &mut *w;
    let d = &*d;
    let base = match d.body_type {
        1 => RigidBodyBuilder::fixed(),
        2 => RigidBodyBuilder::kinematic_position_based(),
        3 => RigidBodyBuilder::kinematic_velocity_based(),
        _ => RigidBodyBuilder::dynamic(),
    };
    let t = Vector::new(d.translation[0], d.translation[1], d.translation[2]);
    let rot = Rotation::from_xyzw(d.rotation[0], d.rotation[1], d.rotation[2], d.rotation[3]);
    let b = base
        .pose(Pose::from_parts(t, rot))
        .linear_damping(d.linear_damping)
        .angular_damping(d.angular_damping)
        .can_sleep(d.can_sleep != 0);
    rb_to_u64(rw.world.insert_body(b))
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_remove_rigid_body(w: *mut RprWorld, body: u64) {
    let rw = &mut *w;
    rw.world.remove_body(rb_from_u64(body));
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_translation(w: *const RprWorld, body: u64, out: *mut f32) {
    let rw = &*w;
    if let Some(b) = rw.world.bodies.get(rb_from_u64(body)) {
        wr3(out, b.translation());
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_rotation(w: *const RprWorld, body: u64, out: *mut f32) {
    let rw = &*w;
    if let Some(b) = rw.world.bodies.get(rb_from_u64(body)) {
        wr4(out, b.rotation());
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_linvel(w: *const RprWorld, body: u64, out: *mut f32) {
    let rw = &*w;
    if let Some(b) = rw.world.bodies.get(rb_from_u64(body)) {
        wr3(out, b.linvel());
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_translation(w: *mut RprWorld, body: u64, v: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_translation(rd3(v), wake != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_rotation(w: *mut RprWorld, body: u64, q: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_rotation(Rotation::from_xyzw(*q, *q.add(1), *q.add(2), *q.add(3)), wake != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_linvel(w: *mut RprWorld, body: u64, v: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_linvel(rd3(v), wake != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_angvel(w: *mut RprWorld, body: u64, v: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        // In 3D, AngVector == Vector (Vec3).
        b.set_angvel(rd3(v), wake != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_angvel(w: *const RprWorld, body: u64, out: *mut f32) {
    let rw = &*w;
    if let Some(b) = rw.world.bodies.get(rb_from_u64(body)) {
        wr3(out, b.angvel());
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_next_kinematic_rotation(w: *mut RprWorld, body: u64, q: *const f32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_next_kinematic_rotation(Rotation::from_xyzw(*q, *q.add(1), *q.add(2), *q.add(3)));
    }
}

// ---- forces & impulses ----------------------------------------------------------------

/// A tiny macro for the "borrow the body mut, call one method with (Vector, wake)" shape that
/// every force/impulse entry point repeats.
macro_rules! body_vec_wake {
    ($name:ident, $method:ident) => {
        #[no_mangle]
        pub unsafe extern "C" fn $name(w: *mut RprWorld, body: u64, v: *const f32, wake: i32) {
            let rw = &mut *w;
            if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
                b.$method(rd3(v), wake != 0);
            }
        }
    };
}
body_vec_wake!(rpr_body_apply_impulse, apply_impulse);
body_vec_wake!(rpr_body_add_force, add_force);
body_vec_wake!(rpr_body_apply_torque_impulse, apply_torque_impulse);
body_vec_wake!(rpr_body_add_torque, add_torque);

#[no_mangle]
pub unsafe extern "C" fn rpr_body_apply_impulse_at_point(w: *mut RprWorld, body: u64, imp: *const f32, point: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.apply_impulse_at_point(rd3(imp), rd3(point), wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_add_force_at_point(w: *mut RprWorld, body: u64, f: *const f32, point: *const f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.add_force_at_point(rd3(f), rd3(point), wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_reset_forces(w: *mut RprWorld, body: u64, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.reset_forces(wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_reset_torques(w: *mut RprWorld, body: u64, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.reset_torques(wake != 0);
    }
}

// ---- axis locks -----------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_enabled_translations(w: *mut RprWorld, body: u64, x: i32, y: i32, z: i32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_enabled_translations(x != 0, y != 0, z != 0, wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_enabled_rotations(w: *mut RprWorld, body: u64, x: i32, y: i32, z: i32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_enabled_rotations(x != 0, y != 0, z != 0, wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_lock_translations(w: *mut RprWorld, body: u64, locked: i32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.lock_translations(locked != 0, wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_lock_rotations(w: *mut RprWorld, body: u64, locked: i32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.lock_rotations(locked != 0, wake != 0);
    }
}

// ---- body state -----------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_body_type(w: *mut RprWorld, body: u64, body_type: i32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        let t = match body_type {
            1 => RigidBodyType::Fixed,
            2 => RigidBodyType::KinematicPositionBased,
            3 => RigidBodyType::KinematicVelocityBased,
            _ => RigidBodyType::Dynamic,
        };
        b.set_body_type(t, wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_sleep(w: *mut RprWorld, body: u64) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.sleep();
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_wake_up(w: *mut RprWorld, body: u64, strong: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.wake_up(strong != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_is_sleeping(w: *const RprWorld, body: u64) -> i32 {
    let rw = &*w;
    rw.world.bodies.get(rb_from_u64(body)).map(|b| b.is_sleeping() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_enabled(w: *mut RprWorld, body: u64, enabled: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_enabled(enabled != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_is_enabled(w: *const RprWorld, body: u64) -> i32 {
    let rw = &*w;
    rw.world.bodies.get(rb_from_u64(body)).map(|b| b.is_enabled() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_gravity_scale(w: *mut RprWorld, body: u64, scale: f32, wake: i32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_gravity_scale(scale, wake != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_mass(w: *const RprWorld, body: u64) -> f32 {
    let rw = &*w;
    rw.world.bodies.get(rb_from_u64(body)).map(|b| b.mass()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_linear_damping(w: *mut RprWorld, body: u64, damping: f32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_linear_damping(damping);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_angular_damping(w: *mut RprWorld, body: u64, damping: f32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_angular_damping(damping);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_num_colliders(w: *const RprWorld, body: u64) -> u32 {
    let rw = &*w;
    rw.world.bodies.get(rb_from_u64(body)).map(|b| b.colliders().len() as u32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_body_collider(w: *const RprWorld, body: u64, i: u32) -> u64 {
    let rw = &*w;
    rw.world
        .bodies
        .get(rb_from_u64(body))
        .and_then(|b| b.colliders().get(i as usize).copied())
        .map(co_to_u64)
        .unwrap_or(RPR_INVALID)
}

// ---------------------------------------------------------------------------------------
// Colliders
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create_collider(
    w: *mut RprWorld,
    d: *const RprColliderDesc,
    parent: u64,
) -> u64 {
    let rw = &mut *w;
    let d = &*d;
    let base: Option<ColliderBuilder> = match d.shape_type {
        0 => Some(ColliderBuilder::ball(d.ball_radius)),
        1 => Some(ColliderBuilder::cuboid(d.cuboid_half[0], d.cuboid_half[1], d.cuboid_half[2])),
        2 => Some(ColliderBuilder::capsule_y(d.capsule_half_height, d.capsule_radius)),
        3 => {
            if d.trimesh_vertices.is_null() || d.trimesh_indices.is_null() {
                None
            } else {
                let vcount = d.trimesh_vertex_count as usize;
                let vslice = std::slice::from_raw_parts(d.trimesh_vertices, vcount * 3);
                let verts: Vec<Vector> = (0..vcount)
                    .map(|k| Vector::new(vslice[k * 3], vslice[k * 3 + 1], vslice[k * 3 + 2]))
                    .collect();
                let tcount = (d.trimesh_index_count as usize) / 3;
                let islice = std::slice::from_raw_parts(d.trimesh_indices, tcount * 3);
                let tris: Vec<[u32; 3]> = (0..tcount)
                    .map(|k| [islice[k * 3], islice[k * 3 + 1], islice[k * 3 + 2]])
                    .collect();
                // Returns Err on a degenerate mesh; treat that as "no collider".
                ColliderBuilder::trimesh(verts, tris).ok()
            }
        }
        4 => Some(ColliderBuilder::cylinder(d.capsule_half_height, d.capsule_radius)),
        5 => Some(ColliderBuilder::cone(d.capsule_half_height, d.capsule_radius)),
        6 => Some(ColliderBuilder::round_cuboid(
            d.cuboid_half[0],
            d.cuboid_half[1],
            d.cuboid_half[2],
            d.border_radius,
        )),
        7 => {
            // Convex hull of a point cloud (trimesh_vertices reused; indices ignored).
            if d.trimesh_vertices.is_null() {
                None
            } else {
                let vcount = d.trimesh_vertex_count as usize;
                let vslice = std::slice::from_raw_parts(d.trimesh_vertices, vcount * 3);
                let pts: Vec<Vector> = (0..vcount)
                    .map(|k| Vector::new(vslice[k * 3], vslice[k * 3 + 1], vslice[k * 3 + 2]))
                    .collect();
                ColliderBuilder::convex_hull(&pts)
            }
        }
        _ => None,
    };
    let mut b = match base {
        Some(b) => b,
        None => return RPR_INVALID,
    };
    if d.has_density != 0 {
        b = b.density(d.density);
    }
    if d.has_friction != 0 {
        b = b.friction(d.friction);
    }
    if d.has_restitution != 0 {
        b = b.restitution(d.restitution);
    }
    if d.has_translation != 0 {
        b = b.translation(Vector::new(d.translation[0], d.translation[1], d.translation[2]));
    }
    if d.has_collision_groups != 0 {
        b = b.collision_groups(unpack_groups(d.collision_groups));
    }
    if d.has_sensor != 0 {
        b = b.sensor(d.sensor != 0);
    }
    if d.has_active_events != 0 {
        b = b.active_events(ActiveEvents::from_bits_truncate(d.active_events));
    }
    if d.has_contact_force_threshold != 0 {
        b = b.contact_force_event_threshold(d.contact_force_threshold);
    }
    if d.has_mass != 0 {
        b = b.mass(d.mass);
    }
    let parent_opt = if parent == RPR_INVALID { None } else { Some(rb_from_u64(parent)) };
    co_to_u64(rw.world.insert_collider(b, parent_opt))
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_remove_collider(w: *mut RprWorld, collider: u64, wake: i32) {
    let rw = &mut *w;
    // Use the low-level remove so we can honor the `wake` flag (PhysicsWorld::remove_collider
    // hard-codes wake=true).
    rw.world.colliders.remove(
        co_from_u64(collider),
        &mut rw.world.islands,
        &mut rw.world.bodies,
        wake != 0,
    );
}

#[no_mangle]
pub unsafe extern "C" fn rpr_collider_set_collision_groups(w: *mut RprWorld, collider: u64, groups: u32) {
    let rw = &mut *w;
    if let Some(c) = rw.world.colliders.get_mut(co_from_u64(collider)) {
        c.set_collision_groups(unpack_groups(groups));
    }
}

/// The "borrow the collider mut, call a one-f32 setter" shape shared by the material setters.
macro_rules! collider_set_f32 {
    ($name:ident, $method:ident) => {
        #[no_mangle]
        pub unsafe extern "C" fn $name(w: *mut RprWorld, collider: u64, v: f32) {
            let rw = &mut *w;
            if let Some(c) = rw.world.colliders.get_mut(co_from_u64(collider)) {
                c.$method(v);
            }
        }
    };
}
collider_set_f32!(rpr_collider_set_restitution, set_restitution);
collider_set_f32!(rpr_collider_set_friction, set_friction);
collider_set_f32!(rpr_collider_set_density, set_density);
collider_set_f32!(rpr_collider_set_mass, set_mass);
collider_set_f32!(rpr_collider_set_contact_force_event_threshold, set_contact_force_event_threshold);

#[no_mangle]
pub unsafe extern "C" fn rpr_collider_set_sensor(w: *mut RprWorld, collider: u64, is_sensor: i32) {
    let rw = &mut *w;
    if let Some(c) = rw.world.colliders.get_mut(co_from_u64(collider)) {
        c.set_sensor(is_sensor != 0);
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_collider_is_sensor(w: *const RprWorld, collider: u64) -> i32 {
    let rw = &*w;
    rw.world.colliders.get(co_from_u64(collider)).map(|c| c.is_sensor() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_collider_set_active_events(w: *mut RprWorld, collider: u64, bits: u32) {
    let rw = &mut *w;
    if let Some(c) = rw.world.colliders.get_mut(co_from_u64(collider)) {
        c.set_active_events(ActiveEvents::from_bits_truncate(bits));
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_collider_set_translation(w: *mut RprWorld, collider: u64, v: *const f32) {
    let rw = &mut *w;
    if let Some(c) = rw.world.colliders.get_mut(co_from_u64(collider)) {
        c.set_translation(rd3(v));
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_collider_parent(w: *const RprWorld, collider: u64) -> u64 {
    let rw = &*w;
    rw.world
        .colliders
        .get(co_from_u64(collider))
        .and_then(|c| c.parent())
        .map(rb_to_u64)
        .unwrap_or(RPR_INVALID)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_collider_translation(w: *const RprWorld, collider: u64, out: *mut f32) {
    let rw = &*w;
    if let Some(c) = rw.world.colliders.get(co_from_u64(collider)) {
        wr3(out, c.translation());
    }
}

// ---------------------------------------------------------------------------------------
// Ray cast
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_cast_ray(
    w: *mut RprWorld,
    origin: *const f32,
    dir: *const f32,
    max_toi: f32,
    solid: i32,
    filter_groups: u32,
    has_filter_groups: i32,
    exclude_collider: u64,
    has_exclude: i32,
    out_collider: *mut u64,
    out_toi: *mut f32,
) -> i32 {
    let rw = &*w;
    let mut filter = QueryFilter::default();
    if has_filter_groups != 0 {
        filter = filter.groups(unpack_groups(filter_groups));
    }
    if has_exclude != 0 {
        filter = filter.exclude_collider(co_from_u64(exclude_collider));
    }
    let ray = Ray::new(rd3(origin), rd3(dir));
    match rw.world.cast_ray(&ray, max_toi, solid != 0, filter) {
        Some((h, toi)) => {
            if !out_collider.is_null() {
                *out_collider = co_to_u64(h);
            }
            if !out_toi.is_null() {
                *out_toi = toi;
            }
            1
        }
        None => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_cast_ray_and_get_normal(
    w: *mut RprWorld,
    origin: *const f32,
    dir: *const f32,
    max_toi: f32,
    solid: i32,
    filter_groups: u32,
    has_filter_groups: i32,
    exclude_collider: u64,
    has_exclude: i32,
    out_collider: *mut u64,
    out_toi: *mut f32,
    out_normal: *mut f32,
) -> i32 {
    let rw = &*w;
    let mut filter = QueryFilter::default();
    if has_filter_groups != 0 {
        filter = filter.groups(unpack_groups(filter_groups));
    }
    if has_exclude != 0 {
        filter = filter.exclude_collider(co_from_u64(exclude_collider));
    }
    let ray = Ray::new(rd3(origin), rd3(dir));
    match rw.world.cast_ray_and_get_normal(&ray, max_toi, solid != 0, filter) {
        Some((h, hit)) => {
            if !out_collider.is_null() {
                *out_collider = co_to_u64(h);
            }
            if !out_toi.is_null() {
                *out_toi = hit.time_of_impact;
            }
            if !out_normal.is_null() {
                wr3(out_normal, hit.normal);
            }
            1
        }
        None => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_project_point(
    w: *mut RprWorld,
    point: *const f32,
    max_dist: f32,
    solid: i32,
    filter_groups: u32,
    has_filter_groups: i32,
    exclude_collider: u64,
    has_exclude: i32,
    out_collider: *mut u64,
    out_point: *mut f32,
    out_inside: *mut i32,
) -> i32 {
    let rw = &*w;
    let mut filter = QueryFilter::default();
    if has_filter_groups != 0 {
        filter = filter.groups(unpack_groups(filter_groups));
    }
    if has_exclude != 0 {
        filter = filter.exclude_collider(co_from_u64(exclude_collider));
    }
    match rw.world.project_point(rd3(point), max_dist, solid != 0, filter) {
        Some((h, proj)) => {
            if !out_collider.is_null() {
                *out_collider = co_to_u64(h);
            }
            if !out_point.is_null() {
                wr3(out_point, proj.point);
            }
            if !out_inside.is_null() {
                *out_inside = proj.is_inside as i32;
            }
            1
        }
        None => 0,
    }
}

// ---------------------------------------------------------------------------------------
// Impulse joints
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create_impulse_joint(
    w: *mut RprWorld,
    body1: u64,
    body2: u64,
    d: *const RprJointDesc,
    _wake: i32,
) -> u64 {
    let rw = &mut *w;
    let d = &*d;
    let axis = Vector::new(d.axis[0], d.axis[1], d.axis[2]);
    // Each typed builder Into<GenericJoint>; anchors/limits/motor are then set on the generic form.
    let mut j: GenericJoint = match d.joint_type {
        0 => RevoluteJointBuilder::new(axis).into(),
        2 => PrismaticJointBuilder::new(axis).into(),
        3 => SpringJointBuilder::new(d.rest_length, d.stiffness, d.damping).into(),
        4 => RopeJointBuilder::new(d.rest_length).into(),
        5 => SphericalJointBuilder::new().into(),
        _ => FixedJointBuilder::new().into(),
    };
    j.set_local_anchor1(Vector::new(d.anchor1[0], d.anchor1[1], d.anchor1[2]));
    j.set_local_anchor2(Vector::new(d.anchor2[0], d.anchor2[1], d.anchor2[2]));
    if d.contacts_enabled != 0 {
        j.set_contacts_enabled(true);
    }
    // The joint's canonical free axis: rotation about local X for a revolute, translation along
    // local X for prismatic/spring/rope. Limits + a desc-time motor apply to that axis.
    let free = if d.joint_type == 0 { JointAxis::AngX } else { JointAxis::LinX };
    if d.has_limits != 0 {
        j.set_limits(free, [d.limit_min, d.limit_max]);
    }
    match d.motor_kind {
        1 => {
            j.set_motor_position(free, d.motor_target, d.motor_stiffness, d.motor_damping);
        }
        2 => {
            j.set_motor_velocity(free, d.motor_target, d.motor_damping);
        }
        _ => {}
    }
    ij_to_u64(rw.world.insert_impulse_joint(rb_from_u64(body1), rb_from_u64(body2), j))
}

#[no_mangle]
pub unsafe extern "C" fn rpr_world_remove_impulse_joint(w: *mut RprWorld, joint: u64, _wake: i32) {
    let rw = &mut *w;
    rw.world.remove_impulse_joint(ij_from_u64(joint));
}

#[no_mangle]
pub unsafe extern "C" fn rpr_joint_configure_motor_position(
    w: *mut RprWorld,
    joint: u64,
    axis: i32,
    target: f32,
    stiffness: f32,
    damping: f32,
) {
    let rw = &mut *w;
    if let Some(j) = rw.world.impulse_joints.get_mut(ij_from_u64(joint), true) {
        j.data.set_motor_position(axis_from_i32(axis), target, stiffness, damping);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_joint_configure_motor_velocity(
    w: *mut RprWorld,
    joint: u64,
    axis: i32,
    target_vel: f32,
    factor: f32,
) {
    let rw = &mut *w;
    if let Some(j) = rw.world.impulse_joints.get_mut(ij_from_u64(joint), true) {
        j.data.set_motor_velocity(axis_from_i32(axis), target_vel, factor);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_joint_set_limits(w: *mut RprWorld, joint: u64, axis: i32, min: f32, max: f32) {
    let rw = &mut *w;
    if let Some(j) = rw.world.impulse_joints.get_mut(ij_from_u64(joint), true) {
        j.data.set_limits(axis_from_i32(axis), [min, max]);
    }
}

// ---------------------------------------------------------------------------------------
// Raycast vehicle controller
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create_vehicle_controller(w: *mut RprWorld, chassis: u64) -> u64 {
    let rw = &mut *w;
    rw.vehicles
        .push(DynamicRayCastVehicleController::new(rb_from_u64(chassis)));
    rw.vehicles.len() as u64 // 1-based
}

#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_add_wheel(
    w: *mut RprWorld,
    vc: u64,
    connection: *const f32,
    direction: *const f32,
    axle: *const f32,
    suspension_rest: f32,
    radius: f32,
) -> u32 {
    let rw = &mut *w;
    let idx = (vc as usize).wrapping_sub(1);
    if let Some(v) = rw.vehicles.get_mut(idx) {
        v.add_wheel(
            rd3(connection),
            rd3(direction),
            rd3(axle),
            suspension_rest,
            radius,
            &WheelTuning::default(),
        );
        (v.wheels().len() as u32).wrapping_sub(1) // index of the wheel just added
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_suspension_stiffness(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.suspension_stiffness = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_max_suspension_travel(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.max_suspension_travel = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_friction_slip(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.friction_slip = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_side_friction_stiffness(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.side_friction_stiffness = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_engine_force(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.engine_force = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_brake(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.brake = val;
    }
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_set_steering(w: *mut RprWorld, vc: u64, i: u32, val: f32) {
    if let Some(wh) = wheel_mut(w, vc, i) {
        wh.steering = val;
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_update(
    w: *mut RprWorld,
    vc: u64,
    dt: f32,
    filter_groups: u32,
    has_filter_groups: i32,
) {
    let idx = (vc as usize).wrapping_sub(1);
    // Split-borrow: the controller vec and the physics world are distinct fields.
    let RprWorld { world, vehicles, .. } = &mut *w;
    if let Some(v) = vehicles.get_mut(idx) {
        // The wheel rays MUST exclude the chassis body: they start at the suspension hard points,
        // which sit INSIDE the chassis collider, so without this exclusion every ray hits the
        // chassis itself at toi=0 — a permanent fake "contact" that travels with the car. The
        // fingerprint is unmistakable once seen: wheels always in contact (even mid-air),
        // suspension pinned at its minimum length, near-zero real tyre forces. rapier 0.34 moved
        // the filter from update_vehicle's own internals to the caller-built QueryPipeline, so
        // the exclusion is OUR job here (same lesson as the character controller's
        // exclude_collider in rpr_character_move).
        let mut filter = QueryFilter::default().exclude_rigid_body(v.chassis);
        if has_filter_groups != 0 {
            filter = filter.groups(unpack_groups(filter_groups));
        }
        let queries = world.broad_phase.as_query_pipeline_mut(
            world.narrow_phase.query_dispatcher(),
            &mut world.bodies,
            &mut world.colliders,
            filter,
        );
        v.update_vehicle(dt, queries);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_wheel_suspension_length(w: *mut RprWorld, vc: u64, i: u32) -> f32 {
    wheel_mut(w, vc, i)
        .map(|wh| wh.raycast_info().suspension_length)
        .unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_wheel_steering(w: *mut RprWorld, vc: u64, i: u32) -> f32 {
    wheel_mut(w, vc, i).map(|wh| wh.steering).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_wheel_rotation(w: *mut RprWorld, vc: u64, i: u32) -> f32 {
    wheel_mut(w, vc, i).map(|wh| wh.rotation).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_wheel_in_contact(w: *mut RprWorld, vc: u64, i: u32) -> i32 {
    wheel_mut(w, vc, i)
        .map(|wh| wh.raycast_info().is_in_contact as i32)
        .unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_current_speed(w: *mut RprWorld, vc: u64) -> f32 {
    let rw = &*w;
    let idx = (vc as usize).wrapping_sub(1);
    rw.vehicles.get(idx).map(|v| v.current_vehicle_speed).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn rpr_vehicle_num_wheels(w: *mut RprWorld, vc: u64) -> u32 {
    let rw = &*w;
    let idx = (vc as usize).wrapping_sub(1);
    rw.vehicles.get(idx).map(|v| v.wheels().len() as u32).unwrap_or(0)
}

// ---------------------------------------------------------------------------------------
// Kinematic character controller
// ---------------------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn rpr_world_create_character_controller(w: *mut RprWorld, offset: f32) -> u64 {
    let rw = &mut *w;
    let mut controller = KinematicCharacterController::default();
    // Compat passes a small absolute gap (e.g. 0.01). rapier's default offset is Relative.
    controller.offset = CharacterLength::Absolute(offset);
    rw.characters.push(CharState {
        controller,
        last: Vector::new(0.0, 0.0, 0.0),
        grounded: false,
    });
    rw.characters.len() as u64 // 1-based
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_set_up(w: *mut RprWorld, cc: u64, up: *const f32) {
    let rw = &mut *w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get_mut(idx) {
        c.controller.up = rd3(up);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_compute_movement(
    w: *mut RprWorld,
    cc: u64,
    collider: u64,
    desired: *const f32,
) {
    let idx = (cc as usize).wrapping_sub(1);
    let coll_h = co_from_u64(collider);
    let desired_v = rd3(desired);
    // Split-borrow: the character vec and the physics world are distinct fields. All the
    // world borrows below are shared (query pipeline + the collider's shape/pose), so they
    // coexist fine with the &mut into `characters`.
    let RprWorld { world, characters, .. } = &mut *w;
    let cs = match characters.get_mut(idx) {
        Some(c) => c,
        None => return,
    };
    let coll = match world.colliders.get(coll_h) {
        Some(c) => c,
        None => return,
    };
    // Exclude the character's OWN collider from the query — otherwise move_shape sees the
    // shape overlapping itself and blocks the movement. This is exactly what the collider
    // handle is for in compat's computeColliderMovement(collider, ...).
    let qp = world.query_pipeline_with_filter(QueryFilter::default().exclude_collider(coll_h));
    let result = cs.controller.move_shape(
        world.integration_parameters.dt,
        &qp,
        coll.shape(),
        coll.position(),
        desired_v,
        |_| {},
    );
    cs.last = result.translation;
    cs.grounded = result.grounded;
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_computed_movement(w: *mut RprWorld, cc: u64, out: *mut f32) {
    let rw = &*w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get(idx) {
        wr3(out, c.last);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_computed_grounded(w: *const RprWorld, cc: u64) -> i32 {
    let rw = &*w;
    let idx = (cc as usize).wrapping_sub(1);
    rw.characters.get(idx).map(|c| c.grounded as i32).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_set_max_slope_climb_angle(w: *mut RprWorld, cc: u64, angle: f32) {
    let rw = &mut *w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get_mut(idx) {
        c.controller.max_slope_climb_angle = angle;
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_set_min_slope_slide_angle(w: *mut RprWorld, cc: u64, angle: f32) {
    let rw = &mut *w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get_mut(idx) {
        c.controller.min_slope_slide_angle = angle;
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_enable_autostep(
    w: *mut RprWorld,
    cc: u64,
    max_height: f32,
    min_width: f32,
    include_dynamic: i32,
) {
    let rw = &mut *w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get_mut(idx) {
        c.controller.autostep = Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(max_height),
            min_width: CharacterLength::Absolute(min_width),
            include_dynamic_bodies: include_dynamic != 0,
        });
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_char_enable_snap_to_ground(w: *mut RprWorld, cc: u64, dist: f32) {
    let rw = &mut *w;
    let idx = (cc as usize).wrapping_sub(1);
    if let Some(c) = rw.characters.get_mut(idx) {
        c.controller.snap_to_ground = Some(CharacterLength::Absolute(dist));
    }
}

#[no_mangle]
pub unsafe extern "C" fn rpr_body_set_next_kinematic_translation(w: *mut RprWorld, body: u64, v: *const f32) {
    let rw = &mut *w;
    if let Some(b) = rw.world.bodies.get_mut(rb_from_u64(body)) {
        b.set_next_kinematic_translation(rd3(v));
    }
}
