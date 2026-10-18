#pragma once

/* Rate at which the IMU is read and the policy is evaluated. Must match the
   control frequency the policy was trained with.*/
#ifndef CONTROL_HZ
#define CONTROL_HZ                  100
#endif

/* Both pinned tasks and the interrupts they allocate */
#define CONTROL_CORE                1
#define CONTROL_TASK_PRIORITY       (configMAX_PRIORITIES - 1)
#define CONTROL_TASK_STACK          6144

/* Everything that isn't the control loop runs on this core */
#define NET_CORE                    0

/* Motors are cut if the estimated pitch or roll exceeds this (robot has fallen).*/
#define FALL_ANGLE_DEG              45.0f
/* "start" is refused unless the robot is within this angle of upright */
#define START_MAX_ANGLE_DEG         20.0f
/* No velocity command / heartbeat for this long -> command forced to zero */
#define CMD_TIMEOUT_MS              500
/* No command for this long while running -> stop (0 disables, robot then
   keeps balancing in place with a zero command) */
#define LINK_LOST_STOP_MS           5000
/* Consecutive failed IMU reads tolerated before the motors are cut */
#define IMU_MAX_CONSECUTIVE_ERRORS  15
/* Apply the fall cutoff in manual (motor test) mode too */
#define MANUAL_FALL_CUTOFF          1

/* Full command range (CurriculumConfig.command_limits). Incoming commands
   are clamped to it and then scaled down together so the outer wheel needs
   at most CMD_MAX_LIN_VEL. */
#define CMD_MAX_LIN_VEL             0.4f    /* m/s */
#define CMD_MAX_ANG_VEL             2.5f    /* rad/s */

/* Half the distance between the wheel centres in the MuJoCo model */
#define WHEEL_HALF_TRACK_M          0.0535f
/* Motor no-load speed*/
#define WHEEL_MAX_SPEED_RAD_S       19.4f


#define MOTOR_AIN1_GPIO             7
#define MOTOR_AIN2_GPIO             15
#define MOTOR_PWMA_GPIO             16

#define MOTOR_BIN1_GPIO             5
#define MOTOR_BIN2_GPIO             6
#define MOTOR_PWMB_GPIO             4

/* Policy action 0 drives motor A, action 1 drives motor B. As in training,
   motor A must be the LEFT wheel and motor B the right one (seen from
   behind, looking forward); encoder A/B follow the same order. Flip a sign
   so that +1 drives that wheel forward (the two motors are mirrored). */
#define MOTOR_A_SIGN                (-1)
#define MOTOR_B_SIGN                (-1)

#define MOTOR_PWM_FREQ_HZ           20000
#define MOTOR_PWM_RES_BITS          10

/* |action| below MOTOR_DEADBAND gives 0 duty; above it the duty is mapped
   onto [MOTOR_MIN_DUTY, 1] to overcome static friction. Leave both at 0
   unless the simulation modelled the same thing. */
#define MOTOR_DEADBAND              0.0f
#define MOTOR_MIN_DUTY              0.0f


#define ENCODER_A_C1_GPIO           18
#define ENCODER_A_C2_GPIO           8
#define ENCODER_B_C1_GPIO           39
#define ENCODER_B_C2_GPIO           38

/* Assumes 7 PPR and a 1:150 gearbox. */
#define ENCODER_COUNTS_PER_REV      (7.0f * 4.0f * 150.0f)
/* Encoder sign so that forward wheel rotation counts up */
#define ENCODER_A_SIGN              (-1)
#define ENCODER_B_SIGN              (1)

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

/* IMU mounting: the IMU axis (0=x 1=y 2=z) and sign giving each axis of the
   robot body frame, which is the frame the policy was trained in:
   x right (along the axle), y forward, z up. The IMU is mounted x right,
   y forward, z up, so this is the identity. */
#define IMU_BODY_X_AXIS             0
#define IMU_BODY_X_SIGN             (1.0f)
#define IMU_BODY_Y_AXIS             1
#define IMU_BODY_Y_SIGN             (1.0f)
#define IMU_BODY_Z_AXIS             2
#define IMU_BODY_Z_SIGN             (1.0f)

/* pid_test's tilt, forward lean positive. It
   ignores the accel signs, so these only hold while those are +1. */
#define IMU_TILT_GYRO_AXIS          IMU_BODY_X_AXIS
#define IMU_TILT_GYRO_SIGN          (-IMU_BODY_X_SIGN)
#define IMU_FWD_ACC_AXIS            IMU_BODY_Y_AXIS
#define IMU_UP_ACC_AXIS             IMU_BODY_Z_AXIS

/* Complementary filter: weight of the integrated gyro vs. the accel angle */
#define TILT_FILTER_ALPHA           0.98f

#define IMU_FOC_CONF                0x7D
/* Time to let go of the robot after pressing "calibrate" */
#define IMU_CALIB_SETTLE_MS         1500

#define BATTERY_ADC_GPIO            2
/* Voltage at the pin for a full and an empty battery; the charge is taken
   as linear in between */
#define BATTERY_MV_FULL             2260
#define BATTERY_MV_EMPTY            1900
/* The web UI warns below this charge */
#define BATTERY_LOW_PERCENT         15
#define BATTERY_SAMPLE_MS           250
/* Low-pass time constant, so the motors' load doesn't make the charge jump */
#define BATTERY_FILTER_TAU_MS       5000

#define RGB_LED_GPIO                48
#define RGB_LED_BRIGHTNESS          32

#define WIFI_AP_SSID                "goose"
#define WIFI_AP_PASSWORD            "goosegoose"  
#define WIFI_AP_CHANNEL             6
#define WIFI_AP_MAX_CLIENTS         2

/* Rate at which telemetry and buffered log lines are pushed to the browser */
#define TELEMETRY_HZ                20
/* Bytes of log text buffered for the browser; lines are dropped if full */
#define LOG_STREAM_BUFFER_BYTES     8192
/* Bytes of per-step state log buffered for the browser (80 bytes a step);
   steps are dropped if full */
#define STATE_LOG_BUFFER_BYTES      8192
