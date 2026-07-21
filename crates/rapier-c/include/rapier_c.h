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

/* Collider shapes — mirrors RAPIER.ColliderDesc.{ball,cuboid,capsule,trimesh}. */
typedef enum {
    RPR_SHAPE_BALL    = 0,
    RPR_SHAPE_CUBOID  = 1,
    RPR_SHAPE_CAPSULE = 2,
    RPR_SHAPE_TRIMESH = 3
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
    float   capsule_half_height;    /* CAPSULE (segment half-height, +Y axis) */
    float   capsule_radius;         /* CAPSULE */

    const float*    trimesh_vertices;   /* TRIMESH */
    uint32_t        trimesh_vertex_count;
    const uint32_t* trimesh_indices;    /* TRIMESH (flat, triangle_count*3) */
    uint32_t        trimesh_index_count;

    float   density;         int32_t has_density;
    float   friction;        int32_t has_friction;
    float   restitution;     int32_t has_restitution;
    float   translation[3];  int32_t has_translation;   /* collider-local offset */
    uint32_t collision_groups; int32_t has_collision_groups; /* packed membership<<16|filter */
} RprColliderDesc;

/* ---- world lifecycle ------------------------------------------------------------- */

RprWorld* rpr_world_create(const float gravity[3]);
void      rpr_world_free(RprWorld* w);
/* The integration timestep the solver will use (IntegrationParameters::dt, default 1/60). */
float     rpr_world_timestep(const RprWorld* w);
void      rpr_world_step(RprWorld* w);
/* rapier3d crate version, e.g. "0.34.0" — static storage, do not free. */
const char* rpr_version(void);

/* ---- rigid bodies ---------------------------------------------------------------- */

uint64_t rpr_world_create_rigid_body(RprWorld* w, const RprBodyDesc* d);
void     rpr_world_remove_rigid_body(RprWorld* w, uint64_t body);

void rpr_body_translation(const RprWorld* w, uint64_t body, float out[3]);
void rpr_body_rotation(const RprWorld* w, uint64_t body, float out[4]);      /* x,y,z,w */
void rpr_body_linvel(const RprWorld* w, uint64_t body, float out[3]);
void rpr_body_set_translation(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_set_rotation(RprWorld* w, uint64_t body, const float q[4], int32_t wake);
void rpr_body_set_linvel(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
void rpr_body_set_angvel(RprWorld* w, uint64_t body, const float v[3], int32_t wake);
/* Kinematic-position bodies: the pose to interpolate to on the next step (compat
 * setNextKinematicTranslation). */
void rpr_body_set_next_kinematic_translation(RprWorld* w, uint64_t body, const float v[3]);

/* ---- colliders ------------------------------------------------------------------- */

/* `parent_body` may be RPR_INVALID for a free collider, but the app always parents them. */
uint64_t rpr_world_create_collider(RprWorld* w, const RprColliderDesc* d, uint64_t parent_body);
void     rpr_world_remove_collider(RprWorld* w, uint64_t collider, int32_t wake);
void     rpr_collider_set_collision_groups(RprWorld* w, uint64_t collider, uint32_t groups);

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
