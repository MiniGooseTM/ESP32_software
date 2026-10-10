/* Fixed-rate control loop on core 1 and its thread-safe interface.

   Every function here except control_start() can be called from any task on
   either core; none of them block on the control loop. */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "policy.h"

typedef enum {
    CTRL_STATE_INIT,         /* waiting for the IMU */
    CTRL_STATE_IDLE,         /* motors off, sensing only */
    CTRL_STATE_RUNNING,      /* policy drives the motors */
    CTRL_STATE_MANUAL,       /* motors driven directly from the web UI (testing) */
    CTRL_STATE_CALIBRATING,  /* IMU offset calibration, motors off */
} ctrl_state_t;

#define CTRL_MAX_OBS 64
#define CTRL_FRAME_DIM 11           /* one observation frame (OBS_FRAME_FIELDS) */

typedef struct {
    ctrl_state_t state;
    const char *stop_reason;    /* why the robot last stopped, "" if it hasn't */
    int fault;                  /* last stop was a fault (fall, IMU error, link lost) */

    float pitch_deg;            /* forward lean negative, as in training */
    float pitch_rate_dps;
    float gyro[3];              /* rad/s, body frame (x right, y forward, z up) */
    float acc[3];               /* m/s^2, body frame */
    float wheel_vel[2];         /* rad/s, forward positive */

    float cmd_lin;              /* active command after timeout handling */
    float cmd_ang;
    float act[2];               /* motor commands sent, [-1, 1] */
    int obs_dim;
    float obs[CTRL_MAX_OBS];    /* newest observation frame fed to the policy */

    /* Timing, maxima over the last second */
    float infer_us;             /* policy forward pass, last */
    float infer_us_max;
    float busy_us_max;          /* whole loop body */
    float period_us_avg;
    float jitter_us_max;        /* max |period - nominal| */
    uint32_t overruns;          /* total ticks missed since boot */
    uint32_t ticks;
    uint32_t imu_errors;        /* total failed IMU reads since boot */
    char policy_name[sizeof(((policy_file_header_t *)0)->name)];   /* "" if none loaded */
    const char *policy_source;  /* "built-in", "uploaded" or "none" */
    uint32_t policy_crc;
    int policy_ok;              /* loaded, matches the observation, self-test passed */
    int policy_busy;            /* a policy upload is in progress */
    uint32_t policy_generation; /* counts policy (re)loads */
} control_telemetry_t;

/* One control step while the policy drives */
typedef struct {
    uint32_t step;                  /* since start: 0 is the first step of a run */
    uint32_t time_us;               /* since start */
    float frame[CTRL_FRAME_DIM];    /* newest observation frame fed to the policy */
    float act[2];                   /* the policy's action, sent to the motors */
} control_step_t;

/* Create the control task pinned to CONTROL_CORE. It initialises the IMU and
   the loop timer itself, so their interrupts land on that core. */
void control_start(void);

/* Desired linear (m/s) and angular (rad/s) velocity. Also acts as the
   heartbeat: if it isn't called for CMD_TIMEOUT_MS the command drops to 0. */
void control_set_command(float lin, float ang);

/* Keep the link alive without changing the command */
void control_heartbeat(void);

/* Start balancing (from IDLE or MANUAL, only if close to upright) */
void control_request_start(void);

/* Stop the motors and return to IDLE, from any state */
void control_request_stop(void);

/* Run IMU calibration (only from IDLE) */
void control_request_calibrate(void);

/* Drive the motors directly with a, b in [-1, 1]. Enters MANUAL from IDLE.
   Subject to the same command timeout. */
void control_set_manual(float a, float b);

/* ---- policy updates, from the web server ----
   Upload sequence: control_begin_policy_update(), write the partition
   (policy_store_*), control_reload_policy(), control_end_policy_update(). */

/* Check a policy against this firmware: observation layout, control rate,
   limits, and its own test vectors. Returns NULL if it can be used,
   otherwise what is wrong. */
const char *control_check_policy(const policy_net_t *net);

/* Keep the robot from starting, calibrating or driving manually until
   control_end_policy_update(), so flash can be written with the motors off.
   Waits a few control ticks. Returns NULL if the update may go ahead,
   otherwise why not (robot not idle, another update running). */
const char *control_begin_policy_update(void);
void control_end_policy_update(void);

/* Make the control task load the policy again (the uploaded one if it is
   usable, otherwise the built-in one) and wait until it has. */
void control_reload_policy(void);

/* Copy of the latest telemetry */
void control_get_telemetry(control_telemetry_t *out);

const char *control_state_name(ctrl_state_t state);

/* Names of the observation frame fields, comma-separated, as in training */
const char *control_frame_fields(void);

/* Take the oldest logged step; false if there is none. Every step is logged
   while RUNNING; steps nobody takes in time are dropped, leaving a gap in
   step. Call from one task only. */
bool control_take_step(control_step_t *out);
