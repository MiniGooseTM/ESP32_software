/* PID test settings. Pins, IMU axes, safety limits and WiFi come from
   robot/main/config.h; the loop rate is set in main/CMakeLists.txt. */
#pragma once

/* Default gains, used until gains are saved from the web page. Units:
   error in degrees of tilt, rate in deg/s, output is the motor command in
   [-1, 1]. Positive gains drive the wheels toward the side the robot leans. */
#define PID_DEFAULT_KP          0.05f   /* per degree: 20 deg of tilt -> full power */
#define PID_DEFAULT_KI          0.0f    /* per degree-second */
#define PID_DEFAULT_KD          0.001f  /* per deg/s */
/* Outer speed loop: leans the setpoint back by KV degrees per rad/s of
   average wheel speed, so the robot stops rolling away. 0 = off. */
#define PID_DEFAULT_KV          0.0f
/* Tilt at which the robot balances, deg (centre of mass not over the axle) */
#define PID_DEFAULT_TRIM_DEG    0.0f
/* Largest motor command the PID may output */
#define PID_DEFAULT_OUT_LIMIT   0.8f
/* Largest contribution of the integral term */
#define PID_DEFAULT_I_LIMIT     0.3f
/* Static friction compensation: any non-zero command is mapped onto
   [FRICTION, 1] so small corrections still turn the wheels */
#define PID_DEFAULT_FRICTION    0.0f
/* Most the speed loop may move the setpoint away from the trim, deg */
#define PID_DEFAULT_SP_LIMIT    8.0f

/* Samples buffered between the control loop and the web page */
#define SAMPLE_RING_SIZE        256
