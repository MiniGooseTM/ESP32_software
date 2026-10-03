/* BMI160 IMU over I2C */
#pragma once

#include "esp_err.h"

typedef struct {
    float gyro[3];  /* rad/s, IMU frame */
    float acc[3];   /* m/s^2, IMU frame */
} imu_sample_t;

/* Create the I2C bus, find the BMI160 at 0x68/0x69, configure ranges and
   rates, power up the sensors and restore saved offsets from NVS. Safe to
   call again after a failure. The I2C interrupt is allocated on the calling
   core, so call it from the control task. */
esp_err_t imu_init(void);

/* One burst read of all six axes. Never logs, so it's safe in the control loop. */
esp_err_t imu_read(imu_sample_t *out);

/* Run the chip's fast offset compensation (robot must be still, in the
   orientation IMU_FOC_CONF expects), enable the offsets and save them to NVS.
   Blocks for up to ~300 ms. */
esp_err_t imu_calibrate(void);
