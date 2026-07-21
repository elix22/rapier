/*
 * smoke.c — the P19.1 gate: a tiny C harness that links librapier_c.a and proves the
 * native rapier3d solver integrates correctly through the C ABI. This is the native-side
 * mirror of tools/physics-smoke.mjs's numeric checks (same tolerances), so "the toolchain
 * builds a lib" is not mistaken for "the physics is right".
 *
 * Every check pins a NUMBER a correct solver must produce — free-fall distance against the
 * closed-form integrator, resting height against the geometry, and a raycast distance —
 * because "it linked and didn't crash" is a near-worthless assertion (a solver that
 * integrates nothing, or with the wrong sign, would pass it).
 *
 * Built + run by tools/test-rapier-native.sh. Exit 0 = all checks pass, 1 = a failure.
 */
#include "rapier_c.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks = 0, fails = 0;
static void chk(const char* name, int ok, const char* fmt, double a, double b) {
    checks++;
    if (ok) { printf("ok   %s\n", name); }
    else    { fails++; printf("FAIL %s -> ", name); printf(fmt, a, b); printf("\n"); }
}
static int near_(double a, double b, double tol) { return fabs(a - b) <= tol; }

int main(void) {
    const float G = -9.81f;
    const double Gd = -9.81;

    printf("rapier3d native version: %s\n", rpr_version());
    chk("rpr_version returns a non-empty string",
        rpr_version() != NULL && strlen(rpr_version()) > 0, "%f %f", 0, 0);

    /* ---- 1. free fall obeys Newton (mirrors physics-smoke.mjs check 1) ------------ */
    {
        float gravity[3] = { 0.0f, G, 0.0f };
        RprWorld* w = rpr_world_create(gravity);

        RprBodyDesc bd;
        memset(&bd, 0, sizeof bd);
        bd.body_type = RPR_BODY_DYNAMIC;
        bd.translation[0] = 0; bd.translation[1] = 100; bd.translation[2] = 0;
        bd.rotation[3] = 1;              /* identity quat */
        bd.can_sleep = 1;
        uint64_t body = rpr_world_create_rigid_body(w, &bd);

        RprColliderDesc cd;
        memset(&cd, 0, sizeof cd);
        cd.shape_type = RPR_SHAPE_BALL;
        cd.ball_radius = 0.5f;
        rpr_world_create_collider(w, &cd, body);

        float dt = rpr_world_timestep(w);
        chk("world timestep is the documented default (1/60)", near_(dt, 1.0/60.0, 1e-6),
            "dt=%f want %f", dt, 1.0/60.0);

        int n = 60;
        for (int i = 0; i < n; i++) rpr_world_step(w);
        double t = (double)n * dt;

        float pos[3], vel[3];
        rpr_body_translation(w, body, pos);
        rpr_body_linvel(w, body, vel);
        double drop = 100.0 - pos[1];
        double want_drop = -Gd * t * t / 2.0;
        double want_vel  = Gd * t;

        chk("free fall distance obeys s = g*t^2/2",
            near_(drop, want_drop, fabs(want_drop) * 0.03), "drop=%f want=%f", drop, want_drop);
        chk("free fall velocity obeys v = g*t",
            near_(vel[1], want_vel, fabs(want_vel) * 0.03), "v=%f want=%f", vel[1], want_vel);
        chk("a 1 s fall under standard gravity drops 4.905 m",
            near_(t, 1.0, 1e-6) && near_(drop, 4.905, 0.15), "t=%f drop=%f", t, drop);
        rpr_world_free(w);
    }

    /* ---- 2. a body comes to rest ON the ground (mirrors check 2) ------------------ */
    {
        float gravity[3] = { 0.0f, G, 0.0f };
        RprWorld* w = rpr_world_create(gravity);

        RprBodyDesc gd; memset(&gd, 0, sizeof gd);
        gd.body_type = RPR_BODY_FIXED; gd.rotation[3] = 1; gd.can_sleep = 1;
        uint64_t ground = rpr_world_create_rigid_body(w, &gd);
        RprColliderDesc gc; memset(&gc, 0, sizeof gc);
        gc.shape_type = RPR_SHAPE_CUBOID;
        gc.cuboid_half[0] = 10; gc.cuboid_half[1] = 0.1f; gc.cuboid_half[2] = 10;
        rpr_world_create_collider(w, &gc, ground);

        RprBodyDesc bd; memset(&bd, 0, sizeof bd);
        bd.body_type = RPR_BODY_DYNAMIC; bd.translation[1] = 5; bd.rotation[3] = 1; bd.can_sleep = 1;
        uint64_t ball = rpr_world_create_rigid_body(w, &bd);
        RprColliderDesc bc; memset(&bc, 0, sizeof bc);
        bc.shape_type = RPR_SHAPE_BALL; bc.ball_radius = 0.5f;
        rpr_world_create_collider(w, &bc, ball);

        for (int i = 0; i < 240; i++) rpr_world_step(w);
        float pos[3]; rpr_body_translation(w, ball, pos);
        chk("dynamic body rests on the ground (y ~ 0.60)", near_(pos[1], 0.6, 0.03),
            "resting y=%f want=%f", pos[1], 0.6);
        chk("body did not tunnel through the ground", pos[1] > 0.0, "y=%f (>0?)", pos[1], 0);
        rpr_world_free(w);
    }

    /* ---- 3. raycast returns the right distance (mirrors check 4) ------------------ */
    {
        float gravity[3] = { 0.0f, G, 0.0f };
        RprWorld* w = rpr_world_create(gravity);
        RprBodyDesc gd; memset(&gd, 0, sizeof gd);
        gd.body_type = RPR_BODY_FIXED; gd.rotation[3] = 1; gd.can_sleep = 1;
        uint64_t ground = rpr_world_create_rigid_body(w, &gd);
        RprColliderDesc gc; memset(&gc, 0, sizeof gc);
        gc.shape_type = RPR_SHAPE_CUBOID;
        gc.cuboid_half[0] = 10; gc.cuboid_half[1] = 0.1f; gc.cuboid_half[2] = 10;
        rpr_world_create_collider(w, &gc, ground);
        rpr_world_step(w);   /* build the broad-phase before querying */

        float o1[3] = { 0, 5, 0 }, d[3] = { 0, -1, 0 };
        uint64_t hitc = 0; float toi = 0;
        int hit = rpr_world_cast_ray(w, o1, d, 20.0f, 1, 0, 0, 0, 0, &hitc, &toi);
        /* from y=5 down to the cuboid top at y=0.1 => toi 4.9 */
        chk("raycast hits and reports the right distance",
            hit && near_(toi, 4.9, 0.02), "toi=%f want=%f", toi, 4.9);

        float o2[3] = { 50, 5, 0 };
        int miss = rpr_world_cast_ray(w, o2, d, 20.0f, 1, 0, 0, 0, 0, &hitc, &toi);
        chk("raycast into empty space misses", !miss, "%f %f", 0, 0);
        rpr_world_free(w);
    }

    printf(fails == 0 ? "\nNATIVE PHYSICS OK (%d checks)\n" : "\nNATIVE PHYSICS FAILED (%d/%d)\n",
           fails == 0 ? checks : fails, checks);
    return fails == 0 ? 0 : 1;
}
