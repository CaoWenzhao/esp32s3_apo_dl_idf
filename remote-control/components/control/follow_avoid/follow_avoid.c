#include "follow_avoid.h"

#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846

/* ------------------------------------------------------------ small helpers */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

float fa_wrap_pi(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

/* Rate-limit `current` toward `target` by at most rate*dt. */
static float ramp(float current, float target, float rate, float dt)
{
    if (rate <= 0.0f || dt <= 0.0f) {
        return target;
    }
    const float step = rate * dt;
    float d = target - current;
    if (d > step) {
        d = step;
    } else if (d < -step) {
        d = -step;
    }
    return current + d;
}

/* --------------------------------------------------------------- config/init */

fa_config_t fa_default_config(void)
{
    fa_config_t c;
    memset(&c, 0, sizeof(c));

    c.follow_distance_m = 1.00f;   /* trail the user by ~1 m */
    c.stop_band_m = 0.25f;         /* hold still within 0.75..1.0 m */
    c.reacquire_timeout_s = 0.7f;
    c.search_timeout_s = 6.0f;

    c.max_linear_mps = 0.7f;
    c.max_angular_rps = 1.6f;
    c.max_lin_accel_mps2 = 0.8f;
    c.max_lin_decel_mps2 = 2.0f;   /* brake faster than we accelerate */
    c.max_ang_accel_rps2 = 6.0f;

    c.kp_dist = 0.9f;
    c.kp_bear = 1.6f;
    c.kd_bear = 0.25f;

    c.emergency_distance_m = 0.35f;
    c.slow_distance_m = 1.20f;
    c.safe_distance_m = 0.60f;
    c.robot_half_width_m = 0.22f;
    c.front_cone_rad = (float)(40.0 * M_PI / 180.0); /* +-40 deg */

    c.search_angular_rps = 0.8f;

    c.w_goal = 1.0f;
    c.w_smooth = 0.35f;
    return c;
}

void fa_init(fa_ctx_t *ctx, const fa_config_t *cfg)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = (cfg != NULL) ? *cfg : fa_default_config();
    ctx->state = FA_STATE_IDLE;
    ctx->initialized = true;
}

/* ----------------------------------------------------------- obstacle field */

void fa_obstacle_reset(fa_obstacle_field_t *f, int num_sectors, float fov_rad)
{
    if (f == NULL) {
        return;
    }
    if (num_sectors < 1) {
        num_sectors = 1;
    }
    if (num_sectors > FA_MAX_SECTORS) {
        num_sectors = FA_MAX_SECTORS;
    }
    if (fov_rad <= 0.0f) {
        fov_rad = (float)M_PI;
    }
    f->num_sectors = num_sectors;
    f->fov_rad = fov_rad;
    f->sector_width_rad = fov_rad / (float)num_sectors;
    for (int i = 0; i < num_sectors; ++i) {
        f->min_dist_m[i] = FA_NO_OBSTACLE;
    }
}

/* Map a body-frame angle to a sector index, or -1 if outside the FOV. */
static int sector_of(const fa_obstacle_field_t *f, float angle_rad)
{
    const float half = 0.5f * f->fov_rad;
    if (angle_rad < -half || angle_rad > half) {
        return -1;
    }
    /* angle -half -> sector 0, angle +half -> last sector. */
    int idx = (int)((angle_rad + half) / f->sector_width_rad);
    if (idx < 0) {
        idx = 0;
    }
    if (idx >= f->num_sectors) {
        idx = f->num_sectors - 1;
    }
    return idx;
}

/* Center angle of a sector in the body frame. */
static float sector_angle(const fa_obstacle_field_t *f, int idx)
{
    const float half = 0.5f * f->fov_rad;
    return -half + ((float)idx + 0.5f) * f->sector_width_rad;
}

void fa_obstacle_add(fa_obstacle_field_t *f, float angle_rad, float dist_m)
{
    if (f == NULL || dist_m <= 0.0f) {
        return;
    }
    const int idx = sector_of(f, angle_rad);
    if (idx < 0) {
        return;
    }
    if (dist_m < f->min_dist_m[idx]) {
        f->min_dist_m[idx] = dist_m;
    }
}

/* ----------------------------------------------------- obstacle queries */

/* Closest obstacle within +-cone_half_rad of straight ahead. */
static float front_clearance(const fa_obstacle_field_t *f, float cone_half_rad)
{
    float best = FA_NO_OBSTACLE;
    if (f == NULL) {
        return best;
    }
    for (int i = 0; i < f->num_sectors; ++i) {
        const float a = sector_angle(f, i);
        if (a >= -cone_half_rad && a <= cone_half_rad) {
            if (f->min_dist_m[i] < best) {
                best = f->min_dist_m[i];
            }
        }
    }
    return best;
}

/*
 * Build a "blocked" mask: a sector is blocked when its closest obstacle is
 * within safe_distance, then the blocked region is widened on each side by the
 * angular footprint of the robot at that range (so we don't try to thread a gap
 * narrower than the suitcase). Returns the number of free sectors.
 */
static int build_blocked(const fa_obstacle_field_t *f, const fa_config_t *cfg,
                         bool blocked[FA_MAX_SECTORS])
{
    const int n = f->num_sectors;
    for (int i = 0; i < n; ++i) {
        blocked[i] = false;
    }
    for (int i = 0; i < n; ++i) {
        const float d = f->min_dist_m[i];
        if (d >= cfg->safe_distance_m) {
            continue;
        }
        /* Angular half-width the robot occupies at distance d. */
        float ratio = cfg->robot_half_width_m / (d > 0.05f ? d : 0.05f);
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        const float enlarge = asinf(ratio);
        int span = (int)(enlarge / f->sector_width_rad + 0.5f);
        for (int k = i - span; k <= i + span; ++k) {
            if (k >= 0 && k < n) {
                blocked[k] = true;
            }
        }
    }
    int free_count = 0;
    for (int i = 0; i < n; ++i) {
        if (!blocked[i]) {
            ++free_count;
        }
    }
    return free_count;
}

/*
 * VFH-lite heading selection. Among the free sectors choose the one with the
 * lowest cost = w_goal*|sector - goal| + w_smooth*|sector - prev|. Returns the
 * chosen heading (rad) via *out_heading; returns false if everything is blocked.
 */
static bool choose_heading(const fa_obstacle_field_t *f, const fa_config_t *cfg,
                           const bool blocked[FA_MAX_SECTORS], float goal_rad,
                           float prev_rad, float *out_heading)
{
    float best_cost = 1.0e30f;
    int best = -1;
    for (int i = 0; i < f->num_sectors; ++i) {
        if (blocked[i]) {
            continue;
        }
        const float a = sector_angle(f, i);
        const float cost = cfg->w_goal * fabsf(fa_wrap_pi(a - goal_rad)) +
                           cfg->w_smooth * fabsf(fa_wrap_pi(a - prev_rad));
        if (cost < best_cost) {
            best_cost = cost;
            best = i;
        }
    }
    if (best < 0) {
        return false;
    }
    *out_heading = sector_angle(f, best);
    return true;
}

/* --------------------------------------------------------------- main update */

fa_output_t fa_update(fa_ctx_t *ctx, const fa_target_t *target,
                      const fa_obstacle_field_t *field,
                      const fa_range_t *ultra_left,
                      const fa_range_t *ultra_right, float dt_s)
{
    fa_output_t out;
    memset(&out, 0, sizeof(out));

    if (ctx == NULL || !ctx->initialized) {
        return out;
    }
    const fa_config_t *cfg = &ctx->cfg;
    if (dt_s <= 0.0f) {
        dt_s = 0.02f;
    }

    /* --- 1. Fuse the front clearance from lidar + the two ultrasonics ----- */
    float clearance = front_clearance(field, cfg->front_cone_rad);
    const float ul = (ultra_left && ultra_left->valid) ? ultra_left->dist_m
                                                       : FA_NO_OBSTACLE;
    const float ur = (ultra_right && ultra_right->valid) ? ultra_right->dist_m
                                                          : FA_NO_OBSTACLE;
    if (ul < clearance) {
        clearance = ul;
    }
    if (ur < clearance) {
        clearance = ur;
    }
    out.front_clearance_m = clearance;

    /* --- 2. Track target loss ------------------------------------------- */
    const bool have_target = (target != NULL && target->valid);
    if (have_target) {
        ctx->lost_timer_s = 0.0f;
        ctx->last_known_bearing = target->bearing_rad;
        ctx->has_last_known = true;
    } else {
        ctx->lost_timer_s += dt_s;
    }

    /* --- 3. Emergency stop has top priority ----------------------------- */
    if (clearance <= cfg->emergency_distance_m) {
        ctx->state = FA_STATE_ESTOP;
        /* Rotate in place toward the freer side to wiggle out, but never
         * drive forward into the obstacle. */
        float turn = 0.0f;
        if (field != NULL) {
            /* Compare clearance of the left half vs right half of the FOV. */
            float left_min = FA_NO_OBSTACLE;
            float right_min = FA_NO_OBSTACLE;
            for (int i = 0; i < field->num_sectors; ++i) {
                const float a = sector_angle(field, i);
                if (a > 0 && field->min_dist_m[i] < left_min) {
                    left_min = field->min_dist_m[i];
                }
                if (a < 0 && field->min_dist_m[i] < right_min) {
                    right_min = field->min_dist_m[i];
                }
            }
            if (ul < left_min) {
                left_min = ul;
            }
            if (ur < right_min) {
                right_min = ur;
            }
            turn = (left_min >= right_min) ? cfg->search_angular_rps
                                           : -cfg->search_angular_rps;
        }
        ctx->cmd_v = ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
        ctx->cmd_w = ramp(ctx->cmd_w, turn, cfg->max_ang_accel_rps2, dt_s);
        out.v_mps = ctx->cmd_v;
        out.omega_rps = ctx->cmd_w;
        out.state = ctx->state;
        out.blocked = true;
        ctx->prev_heading = (turn >= 0.0f) ? 0.5f : -0.5f;
        return out;
    }

    /* --- 4. No target: search, then idle -------------------------------- */
    if (!have_target && ctx->lost_timer_s > cfg->reacquire_timeout_s) {
        if (ctx->state != FA_STATE_SEARCH && ctx->state != FA_STATE_IDLE) {
            ctx->search_timer_s = 0.0f;
        }
        ctx->search_timer_s += dt_s;
        if (!ctx->has_last_known ||
            ctx->search_timer_s > cfg->search_timeout_s) {
            ctx->state = FA_STATE_IDLE;
            ctx->cmd_v = ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
            ctx->cmd_w = ramp(ctx->cmd_w, 0.0f, cfg->max_ang_accel_rps2, dt_s);
        } else {
            ctx->state = FA_STATE_SEARCH;
            const float dir =
                (ctx->last_known_bearing >= 0.0f) ? 1.0f : -1.0f;
            ctx->cmd_v = ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
            ctx->cmd_w = ramp(ctx->cmd_w, dir * cfg->search_angular_rps,
                              cfg->max_ang_accel_rps2, dt_s);
        }
        out.v_mps = ctx->cmd_v;
        out.omega_rps = ctx->cmd_w;
        out.state = ctx->state;
        out.goal_bearing_rad = ctx->last_known_bearing;
        return out;
    }

    if (!have_target) {
        /* Briefly lost but within reacquire window: coast/hold. */
        ctx->cmd_v = ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
        ctx->cmd_w = ramp(ctx->cmd_w, 0.0f, cfg->max_ang_accel_rps2, dt_s);
        out.v_mps = ctx->cmd_v;
        out.omega_rps = ctx->cmd_w;
        out.state = ctx->state;
        return out;
    }

    /* --- 5. Following: pick a heading (VFH around obstacles) ------------- */
    ctx->search_timer_s = 0.0f;
    const float goal = target->bearing_rad;
    out.goal_bearing_rad = goal;

    float heading = goal;
    bool goal_blocked = false;

    if (field != NULL && field->num_sectors > 0) {
        bool blocked[FA_MAX_SECTORS];
        const int free_count = build_blocked(field, cfg, blocked);
        const int goal_idx = sector_of(field, goal);

        /* The goal direction is obstructed if it falls in a blocked sector or
         * lies outside the lidar FOV while something is close ahead. */
        if (goal_idx >= 0) {
            goal_blocked = blocked[goal_idx];
        }
        if (goal_blocked || goal_idx < 0) {
            float h;
            if (free_count > 0 &&
                choose_heading(field, cfg, blocked, goal, ctx->prev_heading,
                               &h)) {
                heading = h;
            } else {
                /* Fully boxed in: treat as emergency-ish, stop & turn. */
                ctx->state = FA_STATE_ESTOP;
                ctx->cmd_v =
                    ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
                const float dir = (goal >= 0.0f) ? 1.0f : -1.0f;
                ctx->cmd_w = ramp(ctx->cmd_w, dir * cfg->search_angular_rps,
                                  cfg->max_ang_accel_rps2, dt_s);
                out.v_mps = ctx->cmd_v;
                out.omega_rps = ctx->cmd_w;
                out.state = ctx->state;
                out.blocked = true;
                out.chosen_heading_rad = heading;
                return out;
            }
        }
    }

    ctx->state = goal_blocked ? FA_STATE_AVOID : FA_STATE_FOLLOW;
    out.blocked = goal_blocked;
    out.chosen_heading_rad = heading;

    /* --- 6. Angular control toward the chosen heading ------------------- */
    const float heading_rate = (heading - ctx->prev_heading) / dt_s;
    float omega_des = cfg->kp_bear * heading + cfg->kd_bear * heading_rate;
    omega_des = clampf(omega_des, -cfg->max_angular_rps, cfg->max_angular_rps);

    /* --- 7. Linear control from range error, then governors ------------- */
    const float err = target->distance_m - cfg->follow_distance_m;
    float v_des;
    if (err > 0.0f) {
        v_des = cfg->kp_dist * err;          /* too far -> catch up */
    } else if (-err <= cfg->stop_band_m) {
        v_des = 0.0f;                         /* comfortable band -> hold */
    } else {
        /* Closer than the stop band. We deliberately do NOT reverse: there are
         * no rear-facing sensors, so backing up blind is unsafe. Stop instead. */
        v_des = 0.0f;
    }
    v_des = clampf(v_des, 0.0f, cfg->max_linear_mps);

    /* Slow down for a sharp turn (don't barrel forward while pivoting). */
    const float turn_scale =
        clampf(1.0f - fabsf(heading) / (0.5f * (float)M_PI), 0.0f, 1.0f);
    v_des *= turn_scale;

    /* Slow down as the front clearance shrinks (linear between emergency and
     * slow distance). This is the smooth part of obstacle avoidance. */
    float gov = 1.0f;
    if (clearance < cfg->slow_distance_m) {
        const float span = cfg->slow_distance_m - cfg->emergency_distance_m;
        gov = (span > 1e-3f)
                  ? (clearance - cfg->emergency_distance_m) / span
                  : 0.0f;
        gov = clampf(gov, 0.0f, 1.0f);
    }
    v_des *= gov;

    /* --- 8. Acceleration limiting (separate up/down rates) -------------- */
    const float lin_rate = (v_des >= ctx->cmd_v) ? cfg->max_lin_accel_mps2
                                                 : cfg->max_lin_decel_mps2;
    ctx->cmd_v = ramp(ctx->cmd_v, v_des, lin_rate, dt_s);
    ctx->cmd_w = ramp(ctx->cmd_w, omega_des, cfg->max_ang_accel_rps2, dt_s);

    ctx->prev_heading = heading;

    out.v_mps = ctx->cmd_v;
    out.omega_rps = ctx->cmd_w;
    out.state = ctx->state;
    return out;
}
