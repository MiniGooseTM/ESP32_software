/* Fixed-rate control loop on core 1 and its thread-safe interface.

   Every function here except control_start() can be called from any task on
   either core; none of them block on the control loop. */
#pragma once

#include <stdint.h>

typedef enum {
    CTRL_STATE_INIT,         /* waiting for the IMU */
    CTRL_STATE_IDLE,         /* motors off, sensing only */
    CTRL_STATE_RUNNING,      /* policy drives the motors */
    CTRL_STATE_MANUAL,       /* motors driven directly from the web UI (testing) */
    CTRL_STATE_CALIBRATING,  /* IMU offset calibration, motors off */
} ctrl_state_t;

#define CTRL_MAX_OBS 64

typedef struct {
    ctrl_state_t state;
    const char *stop_reason;    /* why the robot last stopped, "" if it hasn't */
    int fault;                  /* last stop was a fault (fall, IMU error, link lost) */

    float pitch_deg;
    float pitch_rate_dps;
    float gyro[3];              /* rad/s */
    float acc[3];               /* m/s^2 */
    float wheel_vel[2];         /* rad/s, forward positive */

    float cmd_lin;              /* active command after timeout handling */
    float cmd_ang;
    float act[2];               /* motor commands sent, [-1, 1] */
    int obs_dim;
    float obs[CTRL_MAX_OBS];    /* last observation fed to the policy */

    /* Timing, maxima over the last second */
    float infer_us;             /* policy forward pass, last */
    float infer_us_max;
    float busy_us_max;          /* whole loop body */
    float period_us_avg;
    float jitter_us_max;        /* max |period - nominal| */
    uint32_t overruns;          /* total ticks missed since boot */
    uint32_t ticks;
    uint32_t imu_errors;        /* total failed IMU reads since boot */
    int policy_placeholder;
} control_telemetry_t;

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

/* Copy of the latest telemetry */
void control_get_telemetry(control_telemetry_t *out);

const char *control_state_name(ctrl_state_t state);
