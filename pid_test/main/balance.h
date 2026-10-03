/* PID balance controller on core 1 and its thread-safe interface.
   Every function except balance_start() may be called from any task. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BAL_STATE_INIT,         /* waiting for the IMU */
    BAL_STATE_IDLE,         /* motors off, sensing only */
    BAL_STATE_RUNNING,      /* PID drives the motors */
    BAL_STATE_CALIBRATING,  /* IMU offset calibration, motors off */
} bal_state_t;

typedef struct {
    float kp;           /* per degree */
    float ki;           /* per degree-second */
    float kd;           /* per deg/s */
    float kv;           /* deg of setpoint per rad/s of wheel speed */
    float trim_deg;     /* balance tilt */
    float out_limit;    /* max |motor command| */
    float i_limit;      /* max |integral term| */
    float friction;     /* static friction compensation, [0, 1) */
    float sp_limit;     /* max |speed loop setpoint shift|, deg */
} pid_params_t;

/* One control step, streamed to the web page for plotting */
typedef struct {
    uint32_t t_ms;
    float tilt;         /* deg, forward positive */
    float sp;           /* setpoint, deg */
    float rate;         /* deg/s */
    float p, i, d;      /* PID terms, motor command units */
    float u;            /* motor command sent */
} bal_sample_t;

typedef struct {
    bal_state_t state;
    const char *stop_reason;
    int fault;
    float tilt_deg;
    float rate_dps;
    float wheel_vel[2];     /* rad/s, forward positive */
    float u;
    float busy_us_max;      /* over the last second */
    float jitter_us_max;
    uint32_t overruns;
    uint32_t imu_errors;
    uint32_t samples_dropped;
    pid_params_t params;
} bal_status_t;

void balance_start(void);

void balance_heartbeat(void);
void balance_request_start(void);
void balance_request_stop(void);
void balance_request_calibrate(void);
/* Set the trim to the current tilt (hold the robot at its balance point) */
void balance_request_set_trim(void);
/* Save the current gains to flash (only while idle) */
void balance_request_save(void);

/* Change one gain by name (kp, ki, kd, kv, trim, out_limit, i_limit,
   friction, sp_limit). Takes effect on the next control step. Returns false
   for an unknown name. */
bool balance_set_param(const char *name, float value);

void balance_get_status(bal_status_t *out);

/* Move up to max buffered samples into out; returns how many */
size_t balance_take_samples(bal_sample_t *out, size_t max);

const char *balance_state_name(bal_state_t state);
