/*
 * rapier_c.h — stable C ABI over the native Rust `rapier3d` (0.34.0, f32) crate.
 *
 * WHAT THIS IS. The Three.js apps in this runtime drive physics through the
 * `@dimforge/rapier3d-compat` JavaScript API (a single `World` object with high-level
 * methods). On the WEB that API is the genuine wasm package. On the NATIVE runtime the
 * same app code runs against a JS shim (`src/js/rapier/rapier-shim.js`) that talks to
 * `globalThis.__rapier`, a QuickJS binding (`src/host/rapier_bind.c`) that in turn calls
 * the functions declared here — implemented by `librapier_c.a`, built from the sibling
 * Rust crate over the real native rapier3d solver.
 *
 * WHY A HAND-WRITTEN ABI. Native rapier3d is NOT a single `World`: a simulation is a
 * bundle of ~10 structs (PhysicsPipeline, IntegrationParameters, IslandManager,
 * BroadPhase, NarrowPhase, RigidBodySet, ColliderSet, ImpulseJointSet,
 * MultibodyJointSet, CCDSolver, QueryPipeline) plus a gravity vector. `RprWorld` owns all
 * of them behind one opaque handle so JS sees the compat-shaped `World`.
 *
 * MARSHALLING CONTRACT (chosen for ABI robustness — see phase19 plan §5):
 *   - Handles (rigid bodies, colliders, and the controllers this world owns) cross as
 *     opaque `uint64_t`. Zero (RPR_INVALID) means "none / not found".
 *   - Vectors are `float[3]` (x,y,z); quaternions are `float[4]` (x,y,z,w). They cross as
 *     pointers to caller-owned stack arrays — never returned by value — so there are no
 *     struct-return ABI subtleties across arm64/armv7/x86_64.
 *   - Booleans cross as `int32_t` (0/1). Optional args use an explicit `has_*` companion
 *     flag so the solver keeps its OWN default when the app never called the setter (this
 *     is what preserves numeric parity with the wasm compat build, which the shared
 *     tolerant gates check).
 *   - All scalars are f32 (`float`) — the compat API and this crate are both f32.
 *
 * OWNERSHIP. `RprWorld*` is created by rpr_world_create and destroyed by rpr_world_free;
 * every other handle is valid only for the lifetime of its world. Vehicle and character
 * controllers are not rapier slotmap objects — the world owns them in internal vectors and
 * their `uint64_t` handles index those vectors.
 */
#ifndef RAPIER_C_H
#define RAPIER_C_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque owner of the whole native simulation. One per compat `World`. */
typedef struct RprWorld RprWorld;

/* Sentinel handle: "none" / "not found". This is u64::MAX, which is exactly how rapier
 * encodes its OWN invalid handle (index=generation=0xFFFFFFFF). Note that 0 is NOT
 * invalid: the very first body/collider rapier hands out is (index 0, generation 0),
 * which packs to 0 — so a real handle can legitimately be 0, and callers must compare
 * against RPR_INVALID, never against 0. */
#define RPR_INVALID (~(uint64_t)0)

/* Rigid-body kinds — mirrors RAPIER.RigidBodyDesc.{dynamic,fixed,kinematicPositionBased}. */
typedef enum {
    RPR_BODY_DYNAMIC             = 0,
    RPR_BODY_FIXED               = 1,
    RPR_BODY_KINEMATIC_POSITION  = 2
} RprBodyType;

/* Collider shapes. cylinder/cone reuse the capsule half-height+radius fields; round_cuboid
 * reuses cuboid_half plus border_radius; convex_hull reuses trimesh_vertices (points, no
 * indices). Mirrors RAPIER.ColliderDesc.{ball,cuboid,capsule,trimesh,cylinder,cone,roundCuboid,
 * convexHull}. */
typedef enum {
    RPR_SHAPE_BALL         = 0,
    RPR_SHAPE_CUBOID       = 1,
    RPR_SHAPE_CAPSULE      = 2,
    RPR_SHAPE_TRIMESH      = 3,
    RPR_SHAPE_CYLINDER     = 4,
    RPR_SHAPE_CONE         = 5,
    RPR_SHAPE_ROUND_CUBOID = 6,
    RPR_SHAPE_CONVEX_HULL  = 7
} RprShapeType;

/*
 * Flat descriptor for a rigid body. The JS shim accumulates the chainable
 * RigidBodyDesc.setX() calls into one of these and passes it once to create. Rotation is
 * (x,y,z,w). Damping defaults to 0 in the shim (matching the builder default), can_sleep
 * defaults to 1.
 */
typedef struct {
    int32_t body_type;        /* RprBodyType */
    float   translation[3];
    float   rotation[4];      /* x, y, z, w */
    float   linear_damping;
    float   angular_damping;
    int32_t can_sleep;        /* 0/1 */
} RprBodyDesc;

/*
 * Flat descriptor for a collider. `shape_type` selects which shape fields are read.
 * Modifiers each carry a `has_*` flag: when 0, the native builder default stands (density
 * from a default 1.0 material, friction 0.5, restitution 0.0, no local offset, default
 * collision groups) — exactly as the compat builder would leave them.
 *
 * TRIMESH: `trimesh_vertices` is `trimesh_vertex_count * 3` floats; `trimesh_indices` is
 * `trimesh_index_count` u32s (== triangle_count * 3). Both buffers are borrowed for the
 * duration of the create call only.
 */
typedef struct {
    int32_t shape_type;             /* RprShapeType */

    float   ball_radius;            /* BALL */
    float   cuboid_half[3];         /* CUBOID: hx, hy, hz */
    float   capsule_half_height;    /* CAPSULE / CYLINDER / CONE (segment half-height, +Y axis) */
    float   capsule_radius;         /* CAPSULE / CYLINDER / CONE */
    float   border_radius;          /* ROUND_CUBOID border radius */

    const float*    trimesh_vertices;   /* TRIMESH */
    uint32_t        trimesh_vertex_count;
    const uint32_t* trimesh_indices;    /* TRIMESH (flat, triangle_count*3) */
    uint32_t        trimesh_index_count;

    float   density;         int32_t has_density;
    float   friction;        int32_t has_friction;
    float   restitution;     int32_t has_restitution;
    float   translation[3];  int32_t has_translation;   /* collider-local offset */
    uint32_t collision_groups; int32_t has_collision_groups; /* packed membership<<16|filter */
    int32_t sensor;          int32_t has_sensor;        /* ColliderDesc.setSensor */
    uint32_t active_events;  int32_t has_active_events; /* ActiveEvents bits: 1 COLLISION, 2 CONTACT_FORCE */
    float   contact_force_threshold; int32_t has_contact_force_threshold;
    float   mass;            int32_t has_mass;          /* ColliderDesc.setMass (overrides density) */
} RprColliderDesc;

/* ---- world lifecycle ------------------------------------------------------------- */

RprWorld* rpr_world_create(const float gravity[3]);
void      rpr_world_free(RprWorld* w);
/* The integration timestep the solver will use (IntegrationParameters::dt, default 1/60). */
float     rpr_world_timestep(const RprWorld* w);
void      rpr_world_set_timestep(RprWorld* w, float dt);
void      rpr_world_gravity(const RprWorld* w, float out[3]);
void      rpr_world_set_gravity(RprWorld* w, const float v[3]);
void      rpr_world_step(RprWorld* w);
/* rapier3d crate version, e.g. "0.34.0" — static storage, do not free. */
const char* rpr_version(void);

/* ---- collision & contact-force events (compat EventQueue + world.step(events)) ----
 * The world owns one std::mpsc channel pair. rpr_world_step_events steps WITH a collector,
 * then drains this step's events into internal buffers (autoDrain=true semantics: each step
 * replaces the previous step's events). The C side reads them back with the count + indexed
 * getters below and hands them to the JS drain callbacks. A collider only produces events if
 * it was given ActiveEvents (COLLISION_EVENTS / CONTACT_FORCE_EVENTS) via its ColliderDesc or
 * rpr_collider_set_active_events. */
void     rpr_world_step_events(RprWorld* w);
uint32_t rpr_world_num_collision_events(const RprWorld* w);
/* out_handles gets [collider1, collider2]; *out_started is 1 for Started, 0 for Stopped. */
void     rpr_world_collision_event(const RprWorld* w, uint32_t i, uint64_t out_handles[2], int32_t* out_started);
uint32_t rpr_world_num_contact_force_events(const RprWorld* w);
/* out_handles gets [collider1, collider2]; out5 gets [total_force_mag, max_force_mag, dir.x, dir.y, dir.z]. */
void     rpr_world_contact_force_event(const RprWorld* w, uint32_t i, uint64_t out_handles[2], float out5[5]);

/* ---- rigid bodies ---------------------------------------------------------------- */

uint64_t rpr_world_create_rigid_body(RprWorld* w, const RprBodyDesc* d);
void     rpr_world_remove_rigid_body(RprWorld* w, uint64_t body);

void rpr_body_translation(const RprWorld* w, uint64_t body, float out[3]);
void rpr_body_rotation(const RprWorld* w, uint64_t body, float out[4]);      /* x,y,z,w */
void rpr_body_linvel(const RprWorld* w, uint64_t body, float out[3]);
void rpr_body_angvel(const RprWorld* w, uint64_t body, float out[3]);
void rpr_body_set_translation(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_set_rotation(RprWorld* w, uint64_t body, const float q[4], int32_t wake);
void rpr_body_set_linvel(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_set_angvel(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
/* Kinematic-position bodies: the pose to interpolate to on the next step (compat
 * setNextKinematicTranslation / setNextKinematicRotation). */
void rpr_body_set_next_kinematic_translation(RprWorld* w, uint64_t body, const float v[3]);
void rpr_body_set_next_kinematic_rotation(RprWorld* w, uint64_t body, const float q[4]);

/* Forces & impulses (dynamic bodies). `wake` wakes the body if it was sleeping. */
void rpr_body_apply_impulse(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_apply_impulse_at_point(RprWorld* w, uint64_t body, const float imp[3], const float point[3], int32_t wake);
void rpr_body_add_force(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_add_force_at_point(RprWorld* w, uint64_t body, const float f[3], const float point[3], int32_t wake);
void rpr_body_apply_torque_impulse(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_add_torque(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_reset_forces(RprWorld* w, uint64_t body, int32_t wake);
void rpr_body_reset_torques(RprWorld* w, uint64_t body, int32_t wake);

/* Axis locks — pin a body's motion to a plane/axis (setEnabledTranslations/Rotations,
 * lockTranslations/lockRotations). Essential for 2D (allow X,Y translation, Z rotation). */
void rpr_body_set_enabled_translations(RprWorld* w, uint64_t body, int32_t x, int32_t y, int32_t z, int32_t wake);
void rpr_body_set_enabled_rotations(RprWorld* w, uint64_t body, int32_t x, int32_t y, int32_t z, int32_t wake);
void rpr_body_lock_translations(RprWorld* w, uint64_t body, int32_t locked, int32_t wake);
void rpr_body_lock_rotations(RprWorld* w, uint64_t body, int32_t locked, int32_t wake);

/* Body state. body_type: 0 dynamic, 1 fixed, 2 kinematic-position, 3 kinematic-velocity. */
void    rpr_body_set_body_type(RprWorld* w, uint64_t body, int32_t body_type, int32_t wake);
void    rpr_body_sleep(RprWorld* w, uint64_t body);
void    rpr_body_wake_up(RprWorld* w, uint64_t body, int32_t strong);
int32_t rpr_body_is_sleeping(const RprWorld* w, uint64_t body);
void    rpr_body_set_enabled(RprWorld* w, uint64_t body, int32_t enabled);
int32_t rpr_body_is_enabled(const RprWorld* w, uint64_t body);
void    rpr_body_set_gravity_scale(RprWorld* w, uint64_t body, float scale, int32_t wake);
float   rpr_body_mass(const RprWorld* w, uint64_t body);
void    rpr_body_set_linear_damping(RprWorld* w, uint64_t body, float damping);
void    rpr_body_set_angular_damping(RprWorld* w, uint64_t body, float damping);
uint32_t rpr_body_num_colliders(const RprWorld* w, uint64_t body);
uint64_t rpr_body_collider(const RprWorld* w, uint64_t body, uint32_t i);

/* ---- colliders ------------------------------------------------------------------- */

/* `parent_body` may be RPR_INVALID for a free collider, but the app always parents them. */
uint64_t rpr_world_create_collider(RprWorld* w, const RprColliderDesc* d, uint64_t parent_body);
void     rpr_world_remove_collider(RprWorld* w, uint64_t collider, int32_t wake);
void     rpr_collider_set_collision_groups(RprWorld* w, uint64_t collider, uint32_t groups);
/* Runtime material + event setters (compat Collider.setX). */
void     rpr_collider_set_restitution(RprWorld* w, uint64_t collider, float v);
void     rpr_collider_set_friction(RprWorld* w, uint64_t collider, float v);
void     rpr_collider_set_density(RprWorld* w, uint64_t collider, float v);
void     rpr_collider_set_mass(RprWorld* w, uint64_t collider, float v);
void     rpr_collider_set_sensor(RprWorld* w, uint64_t collider, int32_t is_sensor);
int32_t  rpr_collider_is_sensor(const RprWorld* w, uint64_t collider);
void     rpr_collider_set_active_events(RprWorld* w, uint64_t collider, uint32_t bits);
void     rpr_collider_set_contact_force_event_threshold(RprWorld* w, uint64_t collider, float threshold);
void     rpr_collider_set_translation(RprWorld* w, uint64_t collider, const float v[3]);
/* Read-back: which body owns this collider (RPR_INVALID if none), and its world translation. */
uint64_t rpr_collider_parent(const RprWorld* w, uint64_t collider);
void     rpr_collider_translation(const RprWorld* w, uint64_t collider, float out[3]);

/* ---- ray cast -------------------------------------------------------------------- */

/*
 * Cast a ray and return the first hit. Returns 1 on hit (fills *out_collider and
 * *out_toi), 0 on miss. `has_filter_groups`/`has_exclude` gate the two optional filters
 * (the app passes collision-group bitmasks and an excluded collider). `solid` matches the
 * compat `solid` argument.
 */
int32_t rpr_world_cast_ray(RprWorld* w,
                           const float origin[3], const float dir[3],
                           float max_toi, int32_t solid,
                           uint32_t filter_groups, int32_t has_filter_groups,
                           uint64_t exclude_collider, int32_t has_exclude,
                           uint64_t* out_collider, float* out_toi);

/* Like cast_ray but also returns the surface normal at the hit (compat castRayAndGetNormal). */
int32_t rpr_world_cast_ray_and_get_normal(RprWorld* w,
                           const float origin[3], const float dir[3],
                           float max_toi, int32_t solid,
                           uint32_t filter_groups, int32_t has_filter_groups,
                           uint64_t exclude_collider, int32_t has_exclude,
                           uint64_t* out_collider, float* out_toi, float out_normal[3]);

/* Project a point onto the nearest collider (compat projectPoint). Returns 1 on a hit within
 * max_dist (fills *out_collider, out_point[3], *out_inside), 0 otherwise. */
int32_t rpr_world_project_point(RprWorld* w,
                           const float point[3], float max_dist, int32_t solid,
                           uint32_t filter_groups, int32_t has_filter_groups,
                           uint64_t exclude_collider, int32_t has_exclude,
                           uint64_t* out_collider, float out_point[3], int32_t* out_inside);

/* ---- impulse joints (rapier3d::dynamics) -----------------------------------------
 * A flat joint descriptor (compat JointData.<type> + createImpulseJoint). Motor/limit apply
 * to the joint's canonical free axis (revolute: AngX, prismatic/spring/rope: LinX); the runtime
 * config calls below take an explicit JointAxis (0 LinX,1 LinY,2 LinZ,3 AngX,4 AngY,5 AngZ). */
typedef struct {
    int32_t joint_type;      /* 0 revolute, 1 fixed, 2 prismatic, 3 spring, 4 rope, 5 spherical */
    float   anchor1[3];      /* body-local anchor on body1 */
    float   anchor2[3];      /* body-local anchor on body2 */
    float   axis[3];         /* free axis for revolute/prismatic */
    float   rest_length;     /* spring rest length / rope max length */
    float   stiffness;       /* spring */
    float   damping;         /* spring */
    int32_t contacts_enabled;
    int32_t has_limits;   float limit_min; float limit_max;
    int32_t motor_kind;      /* 0 none, 1 position, 2 velocity */
    float   motor_target;    /* target position (kind 1) or target velocity (kind 2) */
    float   motor_stiffness; /* position motor stiffness */
    float   motor_damping;   /* position motor damping, or velocity motor factor */
} RprJointDesc;

uint64_t rpr_world_create_impulse_joint(RprWorld* w, uint64_t body1, uint64_t body2, const RprJointDesc* d, int32_t wake);
void     rpr_world_remove_impulse_joint(RprWorld* w, uint64_t joint, int32_t wake);
/* Runtime reconfiguration (compat configureMotorPosition/Velocity + setLimits). */
void     rpr_joint_configure_motor_position(RprWorld* w, uint64_t joint, int32_t axis, float target, float stiffness, float damping);
void     rpr_joint_configure_motor_velocity(RprWorld* w, uint64_t joint, int32_t axis, float target_vel, float factor);
void     rpr_joint_set_limits(RprWorld* w, uint64_t joint, int32_t axis, float min, float max);

/* ---- raycast vehicle controller (rapier3d::control) ------------------------------ */

/* World-owned; the handle indexes the world's vehicle vector. `chassis_body` is dynamic. */
uint64_t rpr_world_create_vehicle_controller(RprWorld* w, uint64_t chassis_body);
/* Returns the new wheel's index. connection/direction/axle are chassis-local. */
uint32_t rpr_vehicle_add_wheel(RprWorld* w, uint64_t vc,
                               const float connection[3], const float direction[3],
                               const float axle[3], float suspension_rest, float radius);
void  rpr_vehicle_set_suspension_stiffness(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_max_suspension_travel(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_friction_slip(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_side_friction_stiffness(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_engine_force(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_brake(RprWorld* w, uint64_t vc, uint32_t i, float v);
void  rpr_vehicle_set_steering(RprWorld* w, uint64_t vc, uint32_t i, float v);
/* Raycast the wheels and apply this tick's forces. MUST run before rpr_world_step. */
void  rpr_vehicle_update(RprWorld* w, uint64_t vc, float dt,
                         uint32_t filter_groups, int32_t has_filter_groups);
float rpr_vehicle_wheel_suspension_length(RprWorld* w, uint64_t vc, uint32_t i);
float rpr_vehicle_wheel_steering(RprWorld* w, uint64_t vc, uint32_t i);
float rpr_vehicle_wheel_rotation(RprWorld* w, uint64_t vc, uint32_t i);
int32_t rpr_vehicle_wheel_in_contact(RprWorld* w, uint64_t vc, uint32_t i);
float rpr_vehicle_current_speed(RprWorld* w, uint64_t vc);
uint32_t rpr_vehicle_num_wheels(RprWorld* w, uint64_t vc);

/* ---- kinematic character controller (rapier3d::control) -------------------------- */

/* World-owned; the handle indexes the world's character-controller vector. */
uint64_t rpr_world_create_character_controller(RprWorld* w, float offset);
void rpr_char_set_up(RprWorld* w, uint64_t cc, const float up[3]);
/* Compute the collision-corrected movement for `collider`'s shape given a desired move;
 * the result is stashed and read back with rpr_char_computed_movement. */
void rpr_char_compute_movement(RprWorld* w, uint64_t cc, uint64_t collider, const float desired[3]);
void rpr_char_computed_movement(RprWorld* w, uint64_t cc, float out[3]);
/* Whether the last computed movement ended grounded (compat computedGrounded()). */
int32_t rpr_char_computed_grounded(const RprWorld* w, uint64_t cc);
void rpr_char_set_max_slope_climb_angle(RprWorld* w, uint64_t cc, float angle);   /* radians */
void rpr_char_set_min_slope_slide_angle(RprWorld* w, uint64_t cc, float angle);   /* radians */
/* Enable auto-stepping over small ledges (heights/widths are absolute world units). */
void rpr_char_enable_autostep(RprWorld* w, uint64_t cc, float max_height, float min_width, int32_t include_dynamic);
/* Enable snap-to-ground within `dist` (absolute world units). */
void rpr_char_enable_snap_to_ground(RprWorld* w, uint64_t cc, float dist);

#ifdef __cplusplus
}
#endif

#endif /* RAPIER_C_H */
