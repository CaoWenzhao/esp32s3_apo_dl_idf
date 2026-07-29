#include "follow_avoid.h"

#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846
#define TARGET_HOLD_DISTANCE_M 1.50f
#define TARGET_REFLECTION_RANGE_TOLERANCE_M 0.30f
#define TARGET_REFLECTION_HALF_WIDTH_M 0.35f
#define STRAIGHT_DEADBAND_RAD ((float)(8.0 * M_PI / 180.0))
#define LIDAR_TRACK_LIMIT_RAD ((float)(62.0 * M_PI / 180.0))
#define TURN_SETTLE_RAD ((float)(8.0 * M_PI / 180.0))
#define TURN_BRAKE_DECEL_RPS2 4.0f
#define TURN_RESPONSE_LATENCY_S 0.20f
#define TURN_SENSOR_PREDICTION_S 0.10f

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

static float predictive_turn_speed(float heading_error, float current_w,
                                   float angular_limit)
{
    const float direction = heading_error >= 0.0f ? 1.0f : -1.0f;
    const float current_toward_target =
        current_w * direction > 0.0f ? fabsf(current_w) : 0.0f;
    const float remaining =
        fabsf(heading_error) - TURN_SETTLE_RAD -
        current_toward_target * TURN_RESPONSE_LATENCY_S;
    if (remaining <= 0.0f) {
        return 0.0f;
    }
    const float braking_speed =
        sqrtf(2.0f * TURN_BRAKE_DECEL_RPS2 * remaining);
    return direction * fminf(braking_speed, angular_limit);
}

static float target_turn_feedback(const fa_target_t *target, float fallback_w)
{
    if (target != NULL && isfinite(target->vehicle_yaw_rate_rps) &&
        fabsf(target->vehicle_yaw_rate_rps) > 0.04f) {
        const float measured_w = target->vehicle_yaw_rate_rps;
        if (measured_w * fallback_w > 0.0f) {
            return fabsf(measured_w) > fabsf(fallback_w)
                       ? measured_w
                       : fallback_w;
        }
    }
    return fallback_w;
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

    c.max_linear_mps = 0.9f;
    c.max_angular_rps = 2.4f;
    c.max_lin_accel_mps2 = 2.4f;
    c.max_lin_decel_mps2 = 3.0f;
    c.max_ang_accel_rps2 = 30.0f;

    c.kp_dist = 1.65f;
    c.kp_bear = 1.76f;
    c.kd_bear = 0.85f;

    c.emergency_distance_m = 0.35f;
    c.slow_distance_m = 0.80f;
    c.safe_distance_m = 0.80f;
    c.side_release_distance_m = 0.60f;
    c.avoid_forward_mps = 0.35f;
    c.robot_half_width_m = 0.20f;
    c.front_cone_rad = (float)(40.0 * M_PI / 180.0); /* +-40 deg */
    c.predictive_distance_m = 3.50f;
    c.predictive_margin_m = 0.05f;
    c.predictive_max_heading_rad =
        (float)(32.0 * M_PI / 180.0);
    c.predictive_max_angular_rps = 1.70f;
    c.predictive_min_speed_scale = 0.62f;

    c.search_angular_rps = 0.8f;

    c.w_goal = 1.0f;
    c.w_smooth = 0.35f;
    c.w_clearance = 0.80f;
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

static const fa_obstacle_field_t *without_target_reflection(
    const fa_obstacle_field_t *field, const fa_target_t *target,
    fa_obstacle_field_t *filtered)
{
    if (field == NULL || target == NULL || !target->valid ||
        target->distance_m <= 0.0f || filtered == NULL) {
        return field;
    }

    *filtered = *field;
    const float target_half_angle = clampf(
        atanf(TARGET_REFLECTION_HALF_WIDTH_M / target->distance_m),
        (float)(6.0 * M_PI / 180.0), (float)(20.0 * M_PI / 180.0));
    for (int i = 0; i < filtered->num_sectors; ++i) {
        const float lidar_distance = filtered->min_dist_m[i];
        if (lidar_distance >= FA_NO_OBSTACLE * 0.5f ||
            fabsf(lidar_distance - target->distance_m) >
                TARGET_REFLECTION_RANGE_TOLERANCE_M) {
            continue;
        }
        const float angle_error =
            fabsf(fa_wrap_pi(sector_angle(filtered, i) -
                             target->bearing_rad));
        if (angle_error <= target_half_angle) {
            filtered->min_dist_m[i] = FA_NO_OBSTACLE;
        }
    }
    return filtered;
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
float fa_obstacle_clearance(const fa_obstacle_field_t *f,
                            float min_angle_rad, float max_angle_rad)
{
    float best = FA_NO_OBSTACLE;
    if (f == NULL) {
        return best;
    }
    for (int i = 0; i < f->num_sectors; ++i) {
        const float a = sector_angle(f, i);
        if (a >= min_angle_rad && a <= max_angle_rad) {
            if (f->min_dist_m[i] < best) {
                best = f->min_dist_m[i];
            }
        }
    }
    return best;
}

static float candidate_clearance(const fa_obstacle_field_t *f, int center)
{
    float best = FA_NO_OBSTACLE;
    for (int i = center - 2; i <= center + 2; ++i) {
        if (i >= 0 && i < f->num_sectors && f->min_dist_m[i] < best) {
            best = f->min_dist_m[i];
        }
    }
    return best;
}

static float best_side_clearance(const fa_obstacle_field_t *f, bool left)
{
    float best = 0.0f;
    if (f == NULL) {
        return best;
    }
    for (int i = 0; i < f->num_sectors; ++i) {
        const float angle = sector_angle(f, i);
        if ((left && angle <= 0.0f) || (!left && angle >= 0.0f)) {
            continue;
        }
        const float clearance = candidate_clearance(f, i);
        if (clearance > best) {
            best = clearance;
        }
    }
    return best;
}

static float avoid_direction(const fa_obstacle_field_t *field,
                             const fa_range_t *ultra_left,
                             const fa_range_t *ultra_right,
                             float previous_heading)
{
    const bool left_valid = ultra_left != NULL && ultra_left->valid;
    const bool right_valid = ultra_right != NULL && ultra_right->valid;
    if (left_valid && right_valid) {
        const float difference = ultra_left->dist_m - ultra_right->dist_m;
        if (fabsf(difference) >= 0.05f) {
            return difference > 0.0f ? 1.0f : -1.0f;
        }
    }

    if (previous_heading > 0.05f) return 1.0f;
    if (previous_heading < -0.05f) return -1.0f;

    if (field != NULL) {
        return best_side_clearance(field, true) >=
                       best_side_clearance(field, false)
                   ? 1.0f
                   : -1.0f;
    }
    return 1.0f;
}

/*
 * Build a predictive blocked mask. Returns within the current path look-ahead
 * are widened by the suitcase footprint so the planner cannot select a gap
 * narrower than the suitcase.
 */
static int build_predictive_blocked(const fa_obstacle_field_t *f,
                                    const fa_config_t *cfg,
                                    float lookahead_m,
                                    bool blocked[FA_MAX_SECTORS])
{
    const int n = f->num_sectors;
    const float corridor_half_width =
        cfg->robot_half_width_m + cfg->predictive_margin_m;
    for (int i = 0; i < n; ++i) {
        blocked[i] = false;
    }
    for (int i = 0; i < n; ++i) {
        const float d = f->min_dist_m[i];
        if (d >= lookahead_m) {
            continue;
        }
        /* Inflate each return by the suitcase half-width and a tracking
         * margin. The angular span naturally grows as the obstacle gets
         * closer, producing a small early correction and a stronger late
         * correction without a discrete steering threshold. */
        float ratio = corridor_half_width / (d > 0.05f ? d : 0.05f);
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

/* Nearest obstacle whose inflated footprint intersects the ray from the
 * suitcase toward goal_rad. The result is measured along that ray. */
static float predictive_path_clearance(const fa_obstacle_field_t *f,
                                       const fa_config_t *cfg,
                                       float goal_rad, float lookahead_m)
{
    if (f == NULL) {
        return FA_NO_OBSTACLE;
    }
    const float corridor_half_width =
        cfg->robot_half_width_m + cfg->predictive_margin_m;
    float nearest = FA_NO_OBSTACLE;
    for (int i = 0; i < f->num_sectors; ++i) {
        const float distance = f->min_dist_m[i];
        if (distance >= lookahead_m) {
            continue;
        }
        const float relative = fa_wrap_pi(sector_angle(f, i) - goal_rad);
        const float along = distance * cosf(relative);
        const float lateral = fabsf(distance * sinf(relative));
        if (along > 0.10f && lateral <= corridor_half_width &&
            along < nearest) {
            nearest = along;
        }
    }
    return nearest;
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
        const float local_clearance = candidate_clearance(f, i);
        const float clearance_ref = 2.5f;
        const float clearance_norm = local_clearance >= FA_NO_OBSTACLE * 0.5f
                                         ? 1.0f
                                         : clampf(local_clearance / clearance_ref,
                                                  0.0f, 1.0f);
        const float cost = cfg->w_goal * fabsf(fa_wrap_pi(a - goal_rad)) +
                           cfg->w_smooth * fabsf(fa_wrap_pi(a - prev_rad)) +
                           cfg->w_clearance * (1.0f - clearance_norm);
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

    /* A person is also a valid lidar reflector. When a return is both close
     * to the UWB bearing and within 30 cm of the UWB range, remove it from the
     * obstacle field. The angular gate keeps unrelated objects at the same
     * distance available to the planner. */
    fa_obstacle_field_t filtered_field;
    const fa_obstacle_field_t *obstacle_field =
        without_target_reflection(field, target, &filtered_field);

    /* Front clearance comes only from the forward lidar cone. The side-facing
     * ultrasonics select an avoidance direction, never front clearance. */
    float clearance =
        fa_obstacle_clearance(obstacle_field, -cfg->front_cone_rad,
                              cfg->front_cone_rad);
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

    const float target_abs_bearing =
        have_target ? fabsf(fa_wrap_pi(target->bearing_rad)) : 0.0f;
    const float target_lateral_m =
        have_target ? target->distance_m * sinf(target->bearing_rad) : 0.0f;
    /* Inside the hold radius the suitcase must remain completely still:
     * no following, obstacle avoidance, searching, or orientation correction. */
    const bool target_near = have_target &&
                             target->distance_m <= TARGET_HOLD_DISTANCE_M;
    if (target_near) {
        ctx->avoidance_active = false;
        ctx->avoidance_direction = 0.0f;
        ctx->prev_heading = target->bearing_rad;
        ctx->search_timer_s = 0.0f;
        ctx->state = FA_STATE_FOLLOW;
        ctx->cmd_v = 0.0f;
        ctx->cmd_w = 0.0f;

        out.v_mps = 0.0f;
        out.omega_rps = 0.0f;
        out.state = ctx->state;
        out.goal_bearing_rad = target->bearing_rad;
        out.chosen_heading_rad = target->bearing_rad;
        out.blocked = false;
        return out;
    }

    /* The lidar cannot identify a target outside its physical +-65 degree
     * view. UWB still has a signed X/Y solution, so pivot toward any target
     * outside that view before allowing the obstacle planner to choose a
     * different direction. This includes every negative-Y (>90 degree) fix. */
    if (have_target && target_abs_bearing > LIDAR_TRACK_LIMIT_RAD) {
        ctx->avoidance_active = false;
        ctx->avoidance_direction = 0.0f;
        ctx->state = FA_STATE_SEARCH;
        ctx->cmd_v = ramp(ctx->cmd_v, 0.0f, cfg->max_lin_decel_mps2, dt_s);
        const float turn_feedback =
            target_turn_feedback(target, ctx->cmd_w);
        const float desired_w =
            predictive_turn_speed(target->bearing_rad, turn_feedback,
                                  cfg->max_angular_rps);
        ctx->cmd_w = ramp(ctx->cmd_w, desired_w,
                          cfg->max_ang_accel_rps2, dt_s);
        ctx->prev_heading = target->bearing_rad;

        out.v_mps = ctx->cmd_v;
        out.omega_rps = ctx->cmd_w;
        out.state = ctx->state;
        out.goal_bearing_rad = target->bearing_rad;
        out.chosen_heading_rad = target->bearing_rad;
        return out;
    }

    const float governed_clearance = clearance;
    const bool lidar_blocked = obstacle_field != NULL &&
                               clearance < cfg->safe_distance_m;

    if (!ctx->avoidance_active && lidar_blocked && have_target) {
        ctx->avoidance_active = true;
        ctx->avoidance_direction = avoid_direction(
            obstacle_field, ultra_left, ultra_right, ctx->prev_heading);
    }

    /* A missing lidar return means the nearest obstacle is outside the
     * useful range. Leave avoidance immediately and resume UWB following. */
    if (ctx->avoidance_active && obstacle_field == NULL) {
        ctx->avoidance_active = false;
        ctx->avoidance_direction = 0.0f;
        ctx->prev_heading = have_target ? target->bearing_rad : 0.0f;
    }

    if (ctx->avoidance_active) {
        ctx->state = FA_STATE_AVOID;
        out.state = ctx->state;
        out.blocked = true;
        out.goal_bearing_rad = have_target ? target->bearing_rad
                                           : ctx->last_known_bearing;

        if (lidar_blocked) {
            /* Turn hard when boxed in, then taper before the forward cone
             * becomes clear so wheel/gearbox inertia does not carry the
             * suitcase far beyond the opening. */
            const float turn_taper = clampf(
                (cfg->safe_distance_m - clearance) / 0.32f, 0.22f, 1.0f);
            ctx->cmd_v = ramp(ctx->cmd_v, 0.0f,
                              cfg->max_lin_decel_mps2, dt_s);
            ctx->cmd_w = ramp(ctx->cmd_w,
                              ctx->avoidance_direction *
                                  cfg->max_angular_rps * turn_taper,
                              cfg->max_ang_accel_rps2, dt_s);
            ctx->prev_heading = ctx->avoidance_direction *
                                (float)(55.0 * M_PI / 180.0);
            out.v_mps = ctx->cmd_v;
            out.omega_rps = ctx->cmd_w;
            out.chosen_heading_rad = ctx->prev_heading;
            return out;
        }

        /* After a right turn watch only the left sensor; after a left turn
         * watch only the right sensor. Keep driving straight until that
         * opposite side has cleared the obstacle by the release threshold. */
        const fa_range_t *opposite_side = ctx->avoidance_direction < 0.0f
                                              ? ultra_left
                                              : ultra_right;
        /* No ultrasonic echo also means that side is beyond the sensor's
         * useful range, so it is safe to return to UWB following. */
        const bool opposite_clear = opposite_side == NULL ||
                                    !opposite_side->valid ||
                                    opposite_side->dist_m >
                                        cfg->side_release_distance_m;
        if (opposite_clear) {
            ctx->avoidance_active = false;
            ctx->avoidance_direction = 0.0f;
            ctx->prev_heading = have_target ? target->bearing_rad : 0.0f;
        } else {
            ctx->cmd_v = ramp(ctx->cmd_v, cfg->avoid_forward_mps,
                              cfg->max_lin_accel_mps2, dt_s);
            ctx->cmd_w = ramp(ctx->cmd_w, 0.0f,
                              cfg->max_ang_accel_rps2, dt_s);
            ctx->prev_heading = 0.0f;
            out.v_mps = ctx->cmd_v;
            out.omega_rps = ctx->cmd_w;
            out.chosen_heading_rad = 0.0f;
            return out;
        }
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

    /* Far-path planning runs during ordinary following. It checks the full
     * useful lidar FOV against the ray toward the UWB target, then selects the
     * nearest collision-free sector. Near obstacles are still handled by the
     * locked pivot state above. */
    ctx->search_timer_s = 0.0f;
    const float goal = target->bearing_rad;
    out.goal_bearing_rad = goal;

    float heading = goal;
    bool predictive_active = false;
    float predictive_clearance = FA_NO_OBSTACLE;
    if (obstacle_field != NULL) {
        const float path_lookahead =
            fminf(cfg->predictive_distance_m, target->distance_m);
        const float half_fov = 0.5f * obstacle_field->fov_rad;
        const float edge_margin = 0.5f * obstacle_field->sector_width_rad;
        if (goal >= -half_fov + edge_margin &&
            goal <= half_fov - edge_margin) {
            predictive_clearance = predictive_path_clearance(
                obstacle_field, cfg, goal, path_lookahead);
            if (predictive_clearance < path_lookahead) {
                bool blocked[FA_MAX_SECTORS];
                if (build_predictive_blocked(obstacle_field, cfg,
                                             path_lookahead, blocked) > 0) {
                    float candidate = goal;
                    if (choose_heading(obstacle_field, cfg, blocked, goal,
                                       ctx->prev_heading, &candidate)) {
                        float offset = fa_wrap_pi(candidate - goal);
                        offset = clampf(offset,
                                        -cfg->predictive_max_heading_rad,
                                        cfg->predictive_max_heading_rad);
                        heading = fa_wrap_pi(goal + offset);
                        predictive_active =
                            fabsf(offset) >
                            0.5f * obstacle_field->sector_width_rad;
                    }
                }
            }
        }
    }

    ctx->state = predictive_active ? FA_STATE_AVOID : FA_STATE_FOLLOW;
    out.blocked = predictive_active;
    out.chosen_heading_rad = heading;

    /* --- 6. Angular control toward the chosen heading ------------------- */
    const float heading_rate =
        predictive_active ? 0.0f : target->bearing_rate_rps;
    float omega_des = 0.0f;
    const float lateral_m = target_lateral_m;
    const float angular_limit = predictive_active
                                    ? cfg->predictive_max_angular_rps
                                    : cfg->max_angular_rps;
    const bool follow_turn_needed =
        fabsf(heading) >= STRAIGHT_DEADBAND_RAD &&
        fabsf(lateral_m) >= 0.18f;
    if (predictive_active || follow_turn_needed) {
        const float predicted_heading =
            fa_wrap_pi(heading + heading_rate * TURN_SENSOR_PREDICTION_S);
        omega_des = predictive_turn_speed(
            predicted_heading, target_turn_feedback(target, ctx->cmd_w),
            angular_limit);
    }
    omega_des = clampf(omega_des, -angular_limit, angular_limit);

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

    /* Small errors stay straight. Medium errors form a smooth arc. Beyond
     * 50 degrees translation reaches zero so the fast turn is a true pivot. */
    const float abs_heading = fabsf(heading);
    if (abs_heading >= (float)(50.0 * M_PI / 180.0)) {
        v_des = 0.0f;
    } else if (abs_heading > (float)(20.0 * M_PI / 180.0)) {
        const float turn_scale =
            ((float)(50.0 * M_PI / 180.0) - abs_heading) /
            (float)(30.0 * M_PI / 180.0);
        v_des *= clampf(turn_scale, 0.20f, 1.0f);
    }

    /* Far obstacles should bend the trajectory rather than stop it. The speed
     * reduction grows continuously from zero at the look-ahead boundary to
     * the configured floor at the near-avoid boundary. */
    if (predictive_active) {
        const float span =
            cfg->predictive_distance_m - cfg->safe_distance_m;
        const float risk = span > 1e-3f
                               ? clampf((cfg->predictive_distance_m -
                                         predictive_clearance) / span,
                                        0.0f, 1.0f)
                               : 1.0f;
        const float predictive_scale =
            1.0f - risk * (1.0f - cfg->predictive_min_speed_scale);
        v_des *= predictive_scale;
    }

    /* Slow down as the front clearance shrinks (linear between emergency and
     * slow distance). This is the smooth part of obstacle avoidance. */
    float gov = 1.0f;
    if (governed_clearance < cfg->slow_distance_m) {
        const float span = cfg->slow_distance_m - cfg->emergency_distance_m;
        gov = (span > 1e-3f)
                  ? (governed_clearance - cfg->emergency_distance_m) / span
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
