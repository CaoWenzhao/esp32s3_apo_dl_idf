#pragma once

/*
 * follow_avoid.h - Follow-me + obstacle-avoidance algorithm for the suitcase.
 *
 * This module is PURE C with no ESP / FreeRTOS dependencies, so the exact same
 * logic can be unit-tested on a PC (see tests/algorithm/). The application
 * layer (examples/follow_robot) is responsible for reading the sensors,
 * converting them into the frame-agnostic structs below, calling fa_update()
 * on a fixed period, and pushing the resulting (v, omega) to the chassis.
 *
 * ---------------------------------------------------------------------------
 * Coordinate convention (robot body frame), used everywhere in this module:
 *
 *        +x  (forward / driving direction)
 *         ^
 *         |
 *   +y <--+      angle 0    = straight ahead (+x)
 *  (left) |      angle > 0  = to the LEFT  (counter-clockwise)
 *         |      angle < 0  = to the RIGHT (clockwise)
 *
 *   omega > 0 turns the robot left (CCW), matching chassis_set_velocity().
 *
 * Sensor mounting (this build):
 *   - RPLIDAR at the front, 360 deg scan -> obstacle field (front sector only).
 *   - 2x ultrasonic at the front-left / front-right corners -> near-field
 *     safety, catches thin/low obstacles in the lidar blind spots.
 *   - UWB tag carried by the user -> follow target (range + bearing).
 *   - IMU yaw is available to the app for heading hold; not required here.
 * ---------------------------------------------------------------------------
 */

#include <stdbool.h>

/* Max histogram resolution. 72 sectors over a 180 deg FOV = 2.5 deg each. */
#define FA_MAX_SECTORS 72

/* ------------------------------------------------------------------ inputs */

/* The user to follow, as resolved from the UWB tag. */
typedef struct {
    bool valid;          /* true when a fresh, trustworthy fix is available */
    float distance_m;    /* range to the user */
    float bearing_rad;   /* direction to the user in the body frame (see above) */
} fa_target_t;

/* A single ultrasonic reading (front-corner). */
typedef struct {
    bool valid;
    float dist_m;
} fa_range_t;

/*
 * Polar obstacle histogram covering a forward field of view centered on +x.
 * Each sector holds the closest obstacle distance seen in that angular slice
 * (FA_NO_OBSTACLE when empty). Fill it every cycle from the lidar via
 * fa_obstacle_reset() + fa_obstacle_add().
 */
#define FA_NO_OBSTACLE 1.0e9f

typedef struct {
    int num_sectors;
    float fov_rad;            /* total FOV, centered on forward (e.g. PI) */
    float sector_width_rad;
    float min_dist_m[FA_MAX_SECTORS];
} fa_obstacle_field_t;

/* ----------------------------------------------------------------- outputs */

typedef enum {
    FA_STATE_IDLE = 0,   /* no target known -> motors stopped */
    FA_STATE_SEARCH,     /* target lost -> rotate to reacquire */
    FA_STATE_FOLLOW,     /* target valid, path to it is clear */
    FA_STATE_AVOID,      /* target valid, steering around an obstacle */
    FA_STATE_ESTOP,      /* obstacle/too-close emergency -> hard stop */
} fa_state_t;

typedef struct {
    float v_mps;             /* commanded forward velocity */
    float omega_rps;         /* commanded yaw rate (+ = left) */
    fa_state_t state;
    /* debug / telemetry */
    float goal_bearing_rad;  /* where the target is */
    float chosen_heading_rad;/* heading the planner selected */
    float front_clearance_m; /* closest obstacle in the front cone */
    bool blocked;            /* true when the direct path is obstructed */
} fa_output_t;

/* ------------------------------------------------------------------ config */

typedef struct {
    /* Follow geometry */
    float follow_distance_m;   /* desired standoff behind the user */
    float stop_band_m;         /* deadband below standoff where v = 0 */
    float reacquire_timeout_s; /* no valid target this long -> SEARCH */
    float search_timeout_s;    /* searching this long with no target -> IDLE */

    /* Speed / accel limits */
    float max_linear_mps;
    float max_angular_rps;
    float max_lin_accel_mps2;  /* ramp-up limit */
    float max_lin_decel_mps2;  /* ramp-down limit (>= accel for safety) */
    float max_ang_accel_rps2;

    /* Controller gains */
    float kp_dist;             /* range error -> linear velocity */
    float kp_bear;             /* heading error -> yaw rate */
    float kd_bear;             /* heading rate damping */

    /* Obstacle thresholds */
    float emergency_distance_m;/* <= this in front cone / ultrasonic -> ESTOP */
    float slow_distance_m;     /* start linearly slowing down */
    float safe_distance_m;     /* a sector closer than this is "blocked" */
    float robot_half_width_m;  /* widens blocked sectors to robot's footprint */
    float front_cone_rad;      /* half-angle of the cone used for the governor */

    /* Search / escape */
    float search_angular_rps;

    /* VFH cost weights (lower cost = preferred heading) */
    float w_goal;              /* deviation from the target bearing */
    float w_smooth;            /* deviation from the previous heading */
} fa_config_t;

typedef struct {
    fa_config_t cfg;
    fa_state_t state;
    float cmd_v;               /* last issued command (for accel limiting) */
    float cmd_w;
    float prev_heading;        /* last chosen heading (for kd + smoothing) */
    float lost_timer_s;        /* time since last valid target */
    float search_timer_s;      /* time spent searching */
    float last_known_bearing;  /* bearing of the target when last seen */
    bool has_last_known;
    bool initialized;
} fa_ctx_t;

/* ------------------------------------------------------------------ API */

fa_config_t fa_default_config(void);
void fa_init(fa_ctx_t *ctx, const fa_config_t *cfg);

/* Reset the histogram for a new scan. num_sectors clamped to FA_MAX_SECTORS. */
void fa_obstacle_reset(fa_obstacle_field_t *f, int num_sectors, float fov_rad);

/* Add one obstacle point in the BODY frame (angle per the convention above).
 * Points outside the FOV or with non-positive distance are ignored. */
void fa_obstacle_add(fa_obstacle_field_t *f, float angle_rad, float dist_m);

/*
 * Run one control step.
 *   target      : user position from UWB (may be invalid).
 *   field       : lidar-derived obstacle histogram (may be NULL = no lidar).
 *   ultra_left  : front-left ultrasonic (may be NULL/invalid).
 *   ultra_right : front-right ultrasonic (may be NULL/invalid).
 *   dt_s        : time since the previous call, seconds.
 * Returns the velocity command and state. Always safe to call every cycle.
 */
fa_output_t fa_update(fa_ctx_t *ctx, const fa_target_t *target,
                      const fa_obstacle_field_t *field,
                      const fa_range_t *ultra_left,
                      const fa_range_t *ultra_right, float dt_s);

/* Small helper exposed for tests: wrap an angle to [-PI, PI]. */
float fa_wrap_pi(float a);
