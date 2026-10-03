/* Robot configuration: every pin, rate and limit lives here.

   Pins match the test programs in ../main/motor_test_main.c and
   ../imu_test/main/imu_test_main.c.
*/
#pragma once

/* ------------------------------------------------------------------------ */
/* Control loop                                                              */
/* ------------------------------------------------------------------------ */

/* Rate at which the IMU is read and the policy is evaluated. Must match the
   control frequency the policy was trained with. Any value up to ~800 Hz
   works; the loop is driven by a hardware timer, not the FreeRTOS tick.
   Other projects that share these drivers (pid_test) may set their own rate
   as a compile definition. */
#ifndef CONTROL_HZ
#define CONTROL_HZ                  100
#endif

/* Both pinned tasks and the interrupts they allocate */
#define CONTROL_CORE                1
#define CONTROL_TASK_PRIORITY       (configMAX_PRIORITIES - 1)
#define CONTROL_TASK_STACK          6144

/* Everything that isn't the control loop runs on this core */
#define NET_CORE                    0

/* ------------------------------------------------------------------------ */
/* Safety                                                                    */
/* ------------------------------------------------------------------------ */

/* Motors are cut if the estimated pitch exceeds this (robot has fallen) */
#define FALL_ANGLE_DEG              45.0f
/* "start" is refused unless the robot is within this angle of upright */
#define START_MAX_ANGLE_DEG         20.0f
/* No velocity command / heartbeat for this long -> command forced to zero */
#define CMD_TIMEOUT_MS              500
/* No command for this long while running -> stop (0 disables, robot then
   keeps balancing in place with a zero command) */
#define LINK_LOST_STOP_MS           5000
/* Consecutive failed IMU reads tolerated before the motors are cut */
#define IMU_MAX_CONSECUTIVE_ERRORS  3
/* Apply the fall cutoff in manual (motor test) mode too */
#define MANUAL_FALL_CUTOFF          1

/* Clamp for incoming velocity commands */
#define CMD_MAX_LIN_VEL             1.0f    /* m/s */
#define CMD_MAX_ANG_VEL             4.0f    /* rad/s */

/* ------------------------------------------------------------------------ */
/* Motors (TB6612)                                                           */
/* ------------------------------------------------------------------------ */

#define MOTOR_AIN1_GPIO             5
#define MOTOR_AIN2_GPIO             6
#define MOTOR_PWMA_GPIO             4

#define MOTOR_BIN1_GPIO             7
#define MOTOR_BIN2_GPIO             15
#define MOTOR_PWMB_GPIO             16

/* Policy action 0 drives motor A, action 1 drives motor B. Flip a sign so
   that +1 drives that wheel forward (the two motors are mirrored). */
#define MOTOR_A_SIGN                (-1)
#define MOTOR_B_SIGN                (-1)

/* 20 kHz is above the audible range and within the TB6612's 100 kHz limit */
#define MOTOR_PWM_FREQ_HZ           20000
#define MOTOR_PWM_RES_BITS          10

/* |action| below MOTOR_DEADBAND gives 0 duty; above it the duty is mapped
   onto [MOTOR_MIN_DUTY, 1] to overcome static friction. Leave both at 0
   unless the simulation modelled the same thing. */
#define MOTOR_DEADBAND              0.0f
#define MOTOR_MIN_DUTY              0.0f

/* ------------------------------------------------------------------------ */
/* Wheel encoders (N20, quadrature, all 4 edges counted)                    */
/* ------------------------------------------------------------------------ */

#define ENCODER_A_C1_GPIO           39
#define ENCODER_A_C2_GPIO           38
#define ENCODER_B_C1_GPIO           18
#define ENCODER_B_C2_GPIO           8

/* TODO: set from the motor datasheet: encoder PPR * 4 * gear ratio.
   Assumes 7 PPR and a 1:150 gearbox. */
#define ENCODER_COUNTS_PER_REV      (7.0f * 4.0f * 150.0f)
/* Encoder sign so that forward wheel rotation counts up */
#define ENCODER_A_SIGN              (-1)
#define ENCODER_B_SIGN              (-1)

/* ------------------------------------------------------------------------ */
/* IMU (BMI160 over I2C)                                                     */
/* ------------------------------------------------------------------------ */

#define IMU_SDA_GPIO                35
#define IMU_SCL_GPIO                36
#define IMU_I2C_FREQ_HZ             400000

/* Output data rate code for ACC_CONF / GYR_CONF (normal filter mode):
   0x28 = 100 Hz, 0x29 = 200 Hz, 0x2A = 400 Hz, 0x2B = 800 Hz.
   Run the IMU faster than the control loop so every read sees a fresh sample. */
#if CONTROL_HZ <= 100
#define IMU_ODR_CONF                0x29
#elif CONTROL_HZ <= 200
#define IMU_ODR_CONF                0x2A
#else
#define IMU_ODR_CONF                0x2B
#endif

/* Tilt estimate (forward/backward lean, positive forward). Axis indices
   0=x 1=y 2=z. The IMU is mounted x right, y forward, z up, so the robot
   tilts about the IMU's x axis (its roll). A forward lean is a negative
   rotation about x by the right-hand rule, hence the -1 gyro sign.
   Check in the web UI: leaning the robot forward must read positive. */
#define IMU_TILT_GYRO_AXIS          0       /* gyro axis the robot tilts about */
#define IMU_TILT_GYRO_SIGN          (-1.0f)
#define IMU_FWD_ACC_AXIS            1       /* accel axis pointing forward */
#define IMU_UP_ACC_AXIS             2       /* accel axis pointing up when upright */

/* Complementary filter: weight of the integrated gyro vs. the accel angle */
#define TILT_FILTER_ALPHA           0.98f

/* Fast offset compensation target, see BMI160 datasheet FOC_CONF (0x69):
   bit 6 gyro enable, bits 5:4 acc x, 3:2 acc y, 1:0 acc z,
   each 01 = +1 g, 10 = -1 g, 11 = 0 g, 00 = off.
   0x7D: gyro + accel with Z pointing up (IMU flat, robot held upright and
   still). Use 0x40 for gyro-only if the IMU isn't mounted flat. */
#define IMU_FOC_CONF                0x7D
/* Time to let go of the robot after pressing "calibrate" */
#define IMU_CALIB_SETTLE_MS         1500

/* ------------------------------------------------------------------------ */
/* Status LED (on-board WS2812)                                              */
/* ------------------------------------------------------------------------ */

#define RGB_LED_GPIO                48
#define RGB_LED_BRIGHTNESS          32

/* ------------------------------------------------------------------------ */
/* WiFi access point + web interface                                         */
/* ------------------------------------------------------------------------ */

#define WIFI_AP_SSID                "goose"
#define WIFI_AP_PASSWORD            "goosegoose"   /* >= 8 chars, or "" for open */
#define WIFI_AP_CHANNEL             6
#define WIFI_AP_MAX_CLIENTS         2

/* Rate at which telemetry and buffered log lines are pushed to the browser */
#define TELEMETRY_HZ                20
/* Bytes of log text buffered for the browser; lines are dropped if full */
#define LOG_STREAM_BUFFER_BYTES     8192
