/* BMI160 IMU Test

   Brings up a BMI160 6-axis IMU over I2C, optionally runs the chip's built-in
   fast offset compensation (FOC), then prints accelerometer (g), gyroscope
   (deg/s), the gyro rates integrated into angles (deg) and temperature (degC)
   forever.

   The angles are each gyro axis integrated on its own, starting at 0 on boot.
   They are exact for rotation about a single axis, but mix up once the board
   is tilted about several axes at once, and they slowly drift because any
   remaining gyro offset is integrated too.

   Wiring (BMI160 module -> ESP32-S3-DevKitC-1):
     VIN     -> 5V (or 3V3; the module has its own LDO)
     GND     -> GND
     SCL     -> GPIO 9
     SDA     -> GPIO 8
     CS      -> 3V3  (selects I2C; a rising edge on CS would switch the chip to SPI)
     SAO/SDO -> GND  (I2C address 0x68; tie to 3V3 for 0x69, both are probed)

   Register addresses and values are from the Bosch BMI160 datasheet
   (BST-BMI160-DS000-09, rev 1.0).

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "imu_test";

/* I2C wiring */
#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_SDA_GPIO        8
#define IMU_SCL_GPIO        9
#define IMU_I2C_FREQ_HZ     400000
#define IMU_I2C_TIMEOUT_MS  100

/* I2C address depends on the SAO/SDO pin: GND -> 0x68, VDDIO -> 0x69 */
#define BMI160_ADDR_SDO_LOW   0x68
#define BMI160_ADDR_SDO_HIGH  0x69

/* BMI160 registers */
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_REG_ERR          0x02
#define BMI160_REG_PMU_STATUS   0x03
#define BMI160_REG_DATA_GYR_X   0x0C  /* gyr x/y/z then acc x/y/z, 12 bytes, little endian */
#define BMI160_REG_STATUS       0x1B
#define BMI160_REG_TEMPERATURE  0x20
#define BMI160_REG_ACC_CONF     0x40
#define BMI160_REG_ACC_RANGE    0x41
#define BMI160_REG_GYR_CONF     0x42
#define BMI160_REG_GYR_RANGE    0x43
#define BMI160_REG_FOC_CONF     0x69
#define BMI160_REG_OFFSET_0     0x71  /* 0x71-0x73 acc x/y/z, 0x74-0x76 gyr x/y/z LSBs, 0x77 MSBs + enables */
#define BMI160_REG_OFFSET_6     0x77
#define BMI160_REG_CMD          0x7E

#define BMI160_CHIP_ID          0xD1

/* CMD register opcodes */
#define BMI160_CMD_START_FOC    0x03
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYR_NORMAL   0x15
#define BMI160_CMD_SOFTRESET    0xB6

#define BMI160_STATUS_FOC_RDY       (1 << 3)
#define BMI160_OFFSET6_ACC_OFF_EN   (1 << 6)
#define BMI160_OFFSET6_GYR_OFF_EN   (1 << 7)

/* Output data rate 100 Hz, normal filter mode (acc_bwp / gyr_bwp = 0b010) */
#define BMI160_ACC_CONF_100HZ   0x28
#define BMI160_GYR_CONF_100HZ   0x28

/* Measurement ranges and the matching sensitivities */
#define BMI160_ACC_RANGE_4G     0x05
#define ACC_LSB_PER_G           8192.0f
#define BMI160_GYR_RANGE_500DPS 0x02
#define GYR_LSB_PER_DPS         65.6f

/* Offset register units, independent of the selected range */
#define ACC_OFFSET_MG_PER_LSB   3.9f
#define GYR_OFFSET_DPS_PER_LSB  0.061f

/* Calibration: FOC measures the offsets with the board lying still and flat,
   component side up (Z axis pointing up, so the Z target is +1 g). The result
   lives in volatile registers and is lost on power-down. */
#define IMU_RUN_FOC             1
#define BMI160_FOC_CONF_FLAT    0x7D  /* foc_gyr_en | acc_x 0 g | acc_y 0 g | acc_z +1 g */
#define FOC_SETTLE_MS           2000
#define FOC_TIMEOUT_MS          500   /* datasheet max is 250 ms */

/* Sample at the 100 Hz output data rate so the integration sees every gyro sample */
#define SAMPLE_PERIOD_MS        10
#define PRINT_EVERY_N_SAMPLES   10

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_dev;

static esp_err_t bmi160_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1, data, len, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t bmi160_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(imu_dev, buf, sizeof(buf), IMU_I2C_TIMEOUT_MS);
}

static uint8_t bmi160_read_byte(uint8_t reg)
{
    uint8_t value = 0;
    ESP_ERROR_CHECK(bmi160_read(reg, &value, 1));
    return value;
}

static void configure_i2c(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* The module most likely has its own pull-ups; the internal ones don't hurt */
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    /* Find the IMU at either address so the SAO/SDO wiring doesn't matter */
    uint16_t addr;
    if (i2c_master_probe(i2c_bus, BMI160_ADDR_SDO_LOW, IMU_I2C_TIMEOUT_MS) == ESP_OK) {
        addr = BMI160_ADDR_SDO_LOW;
    } else if (i2c_master_probe(i2c_bus, BMI160_ADDR_SDO_HIGH, IMU_I2C_TIMEOUT_MS) == ESP_OK) {
        addr = BMI160_ADDR_SDO_HIGH;
    } else {
        ESP_LOGE(TAG, "No device at 0x%02X or 0x%02X - check wiring (SDA=%d, SCL=%d, CS tied high)",
                 BMI160_ADDR_SDO_LOW, BMI160_ADDR_SDO_HIGH, IMU_SDA_GPIO, IMU_SCL_GPIO);
        abort();
    }
    ESP_LOGI(TAG, "Found device at I2C address 0x%02X", addr);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &imu_dev));
}

static void configure_imu(void)
{
    /* Start from a known state; afterwards both sensors are in suspend mode */
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_CMD, BMI160_CMD_SOFTRESET));
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t chip_id = bmi160_read_byte(BMI160_REG_CHIP_ID);
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID 0x%02X (expected 0x%02X)", chip_id, BMI160_CHIP_ID);
        abort();
    }
    ESP_LOGI(TAG, "BMI160 chip ID 0x%02X OK", chip_id);

    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_ACC_CONF, BMI160_ACC_CONF_100HZ));
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_4G));
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_GYR_CONF, BMI160_GYR_CONF_100HZ));
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_500DPS));

    /* Power up the sensors; the CMD register ignores new writes while busy,
       so wait out the max start-up time of each (3.8 ms acc, 80 ms gyro) */
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* acc_pmu_status is bits 5:4, gyr_pmu_status bits 3:2; 0b01 = normal */
    uint8_t pmu = bmi160_read_byte(BMI160_REG_PMU_STATUS);
    uint8_t err = bmi160_read_byte(BMI160_REG_ERR);
    ESP_LOGI(TAG, "PMU_STATUS 0x%02X (acc %s, gyro %s), ERR_REG 0x%02X",
             pmu,
             ((pmu >> 4) & 0x3) == 1 ? "normal" : "NOT normal",
             ((pmu >> 2) & 0x3) == 1 ? "normal" : "NOT normal",
             err);
    if (err != 0) {
        ESP_LOGW(TAG, "ERR_REG is non-zero - a config value may have been rejected");
    }
}

static void print_offsets(void)
{
    uint8_t off[7];
    ESP_ERROR_CHECK(bmi160_read(BMI160_REG_OFFSET_0, off, sizeof(off)));

    /* Gyro offsets are 10-bit two's complement: LSBs in 0x74-0x76, MSBs in 0x77 */
    int16_t gyr_off[3];
    for (int i = 0; i < 3; i++) {
        int16_t raw = off[3 + i] | (((off[6] >> (2 * i)) & 0x3) << 8);
        gyr_off[i] = (raw & 0x200) ? raw - 0x400 : raw;
    }
    ESP_LOGI(TAG, "Accel offsets: x %+.1f  y %+.1f  z %+.1f mg",
             (int8_t)off[0] * ACC_OFFSET_MG_PER_LSB,
             (int8_t)off[1] * ACC_OFFSET_MG_PER_LSB,
             (int8_t)off[2] * ACC_OFFSET_MG_PER_LSB);
    ESP_LOGI(TAG, "Gyro offsets:  x %+.3f  y %+.3f  z %+.3f deg/s",
             gyr_off[0] * GYR_OFFSET_DPS_PER_LSB,
             gyr_off[1] * GYR_OFFSET_DPS_PER_LSB,
             gyr_off[2] * GYR_OFFSET_DPS_PER_LSB);
}

/* Fast offset compensation: the chip measures its own offsets and writes the
   negated values into the OFFSET registers */
static void calibrate_imu(void)
{
    ESP_LOGI(TAG, "Calibrating in %d ms - keep the board flat (component side up) and still",
             FOC_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(FOC_SETTLE_MS));

    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_FOC_CONF, BMI160_FOC_CONF_FLAT));
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_CMD, BMI160_CMD_START_FOC));

    int waited_ms = 0;
    while (!(bmi160_read_byte(BMI160_REG_STATUS) & BMI160_STATUS_FOC_RDY)) {
        if (waited_ms >= FOC_TIMEOUT_MS) {
            ESP_LOGE(TAG, "FOC did not finish within %d ms, continuing uncalibrated", FOC_TIMEOUT_MS);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }

    /* Apply the offsets, keeping the gyro offset MSBs FOC wrote into OFFSET_6 */
    uint8_t off6 = bmi160_read_byte(BMI160_REG_OFFSET_6);
    ESP_ERROR_CHECK(bmi160_write(BMI160_REG_OFFSET_6,
                                 off6 | BMI160_OFFSET6_ACC_OFF_EN | BMI160_OFFSET6_GYR_OFF_EN));
    ESP_LOGI(TAG, "FOC done after ~%d ms", waited_ms);
    print_offsets();

    /* The first sample after changing the offsets is invalid */
    vTaskDelay(pdMS_TO_TICKS(20));
}

void app_main(void)
{
    configure_i2c();
    configure_imu();
#if IMU_RUN_FOC
    calibrate_imu();
#endif

    float angle_deg[3] = {0};
    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();
    int sample = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        /* One burst read keeps all six axes from the same sample */
        uint8_t raw[12];
        ESP_ERROR_CHECK(bmi160_read(BMI160_REG_DATA_GYR_X, raw, sizeof(raw)));
        int16_t v[6];
        for (int i = 0; i < 6; i++) {
            v[i] = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
        }

        /* Integrate with the measured time step, so a late wake-up doesn't skew the angles */
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) / 1e6f;
        last_us = now_us;
        for (int i = 0; i < 3; i++) {
            angle_deg[i] += v[i] / GYR_LSB_PER_DPS * dt;
        }

        if (++sample < PRINT_EVERY_N_SAMPLES) {
            continue;
        }
        sample = 0;

        uint8_t temp_raw[2];
        ESP_ERROR_CHECK(bmi160_read(BMI160_REG_TEMPERATURE, temp_raw, sizeof(temp_raw)));
        int16_t temp = (int16_t)(temp_raw[0] | (temp_raw[1] << 8));
        float temp_c = 23.0f + temp / 512.0f;

        ESP_LOGI(TAG, "acc [g] x %+6.3f y %+6.3f z %+6.3f | gyr [deg/s] x %+8.2f y %+8.2f z %+8.2f"
                 " | angle [deg] x %+8.2f y %+8.2f z %+8.2f | %.1f C",
                 v[3] / ACC_LSB_PER_G, v[4] / ACC_LSB_PER_G, v[5] / ACC_LSB_PER_G,
                 v[0] / GYR_LSB_PER_DPS, v[1] / GYR_LSB_PER_DPS, v[2] / GYR_LSB_PER_DPS,
                 angle_deg[0], angle_deg[1], angle_deg[2],
                 temp_c);
    }
}
