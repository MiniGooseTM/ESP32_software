/* BMI160 IMU driver

   Register-level driver adapted from imu_test/main/imu_test_main.c.
   Register addresses and values are from the Bosch BMI160 datasheet
   (BST-BMI160-DS000-09, rev 1.0).

   Wiring: SDA -> IMU_SDA_GPIO, SCL -> IMU_SCL_GPIO, CS -> 3V3 (selects I2C),
   SAO/SDO -> GND (0x68) or 3V3 (0x69); both addresses are probed.

   The FOC offsets live in volatile chip registers, so after a calibration
   they are saved to NVS and written back on every boot.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nvs.h"
#include "config.h"
#include "imu.h"

static const char *TAG = "imu";

#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_I2C_TIMEOUT_MS  20

#define BMI160_ADDR_SDO_LOW   0x68
#define BMI160_ADDR_SDO_HIGH  0x69

#define BMI160_REG_CHIP_ID      0x00
#define BMI160_REG_ERR          0x02
#define BMI160_REG_PMU_STATUS   0x03
#define BMI160_REG_DATA_GYR_X   0x0C  /* gyr x/y/z then acc x/y/z, 12 bytes, little endian */
#define BMI160_REG_STATUS       0x1B
#define BMI160_REG_ACC_CONF     0x40
#define BMI160_REG_ACC_RANGE    0x41
#define BMI160_REG_GYR_CONF     0x42
#define BMI160_REG_GYR_RANGE    0x43
#define BMI160_REG_FOC_CONF     0x69
#define BMI160_REG_OFFSET_0     0x71  /* 0x71-0x73 acc x/y/z, 0x74-0x76 gyr x/y/z LSBs, 0x77 MSBs + enables */
#define BMI160_REG_OFFSET_6     0x77
#define BMI160_REG_CMD          0x7E

#define BMI160_CHIP_ID          0xD1

/* Bus idle time needed after every write: 2 us in normal mode, 450 us in
   suspend mode (datasheet 3.2.4). Writes sent sooner are silently dropped. */
#define BMI160_WRITE_IDLE_US    500

#define BMI160_CMD_START_FOC    0x03
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYR_NORMAL   0x15
#define BMI160_CMD_SOFTRESET    0xB6

#define BMI160_STATUS_FOC_RDY       (1 << 3)
#define BMI160_OFFSET6_ACC_OFF_EN   (1 << 6)
#define BMI160_OFFSET6_GYR_OFF_EN   (1 << 7)

#define BMI160_ACC_RANGE_4G     0x05
#define ACC_LSB_PER_G           8192.0f
/* Widest range so a fall or a knock never clips the gyro */
#define BMI160_GYR_RANGE_2000DPS 0x00
#define GYR_LSB_PER_DPS         16.4f

#define ACC_OFFSET_MG_PER_LSB   3.9f
#define GYR_OFFSET_DPS_PER_LSB  0.061f

#define FOC_TIMEOUT_MS          500   /* datasheet max is 250 ms */

#define GRAVITY                 9.80665f
#define DEG_TO_RAD              0.017453292519943295f

#define NVS_NAMESPACE           "imu"
#define NVS_KEY_OFFSETS         "offsets"

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_dev;

static esp_err_t bmi160_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1, data, len, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t bmi160_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(imu_dev, buf, sizeof(buf), IMU_I2C_TIMEOUT_MS);
    esp_rom_delay_us(BMI160_WRITE_IDLE_US);
    return err;
}

/* Write a register and read it back, since a wrong range would silently
   scale every reading wrongly */
static esp_err_t bmi160_write_verified(uint8_t reg, uint8_t value)
{
    ESP_RETURN_ON_ERROR(bmi160_write(reg, value), TAG, "write 0x%02X failed", reg);
    uint8_t readback = 0;
    ESP_RETURN_ON_ERROR(bmi160_read(reg, &readback, 1), TAG, "read 0x%02X failed", reg);
    if (readback != value) {
        ESP_LOGE(TAG, "Register 0x%02X reads 0x%02X after writing 0x%02X", reg, readback, value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t configure_i2c(void)
{
    if (!i2c_bus) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = IMU_I2C_PORT,
            .sda_io_num = IMU_SDA_GPIO,
            .scl_io_num = IMU_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus), TAG, "I2C bus init failed");
    }
    if (imu_dev) {
        return ESP_OK;
    }

    uint16_t addr;
    if (i2c_master_probe(i2c_bus, BMI160_ADDR_SDO_LOW, IMU_I2C_TIMEOUT_MS) == ESP_OK) {
        addr = BMI160_ADDR_SDO_LOW;
    } else if (i2c_master_probe(i2c_bus, BMI160_ADDR_SDO_HIGH, IMU_I2C_TIMEOUT_MS) == ESP_OK) {
        addr = BMI160_ADDR_SDO_HIGH;
    } else {
        ESP_LOGE(TAG, "No device at 0x%02X or 0x%02X - check wiring (SDA=%d, SCL=%d, CS tied high)",
                 BMI160_ADDR_SDO_LOW, BMI160_ADDR_SDO_HIGH, IMU_SDA_GPIO, IMU_SCL_GPIO);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Found BMI160 at I2C address 0x%02X", addr);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_config, &imu_dev);
}

static void log_offsets(const uint8_t off[7])
{
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

static void save_offsets(const uint8_t off[7])
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, NVS_KEY_OFFSETS, off, 7);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Saving offsets to NVS failed: %s", esp_err_to_name(err));
    }
}

static void restore_offsets(void)
{
    uint8_t off[7];
    size_t len = sizeof(off);
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "No saved calibration - run calibrate from the web UI");
        return;
    }
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_OFFSETS, off, &len);
    nvs_close(nvs);
    if (err != ESP_OK || len != sizeof(off)) {
        ESP_LOGW(TAG, "No saved calibration - run calibrate from the web UI");
        return;
    }
    for (int i = 0; i < 6; i++) {
        if (bmi160_write_verified(BMI160_REG_OFFSET_0 + i, off[i]) != ESP_OK) {
            ESP_LOGW(TAG, "Restoring offsets failed");
            return;
        }
    }
    if (bmi160_write_verified(BMI160_REG_OFFSET_6,
                              off[6] | BMI160_OFFSET6_ACC_OFF_EN | BMI160_OFFSET6_GYR_OFF_EN) != ESP_OK) {
        ESP_LOGW(TAG, "Restoring offsets failed");
        return;
    }
    ESP_LOGI(TAG, "Restored calibration from NVS");
    log_offsets(off);
}

esp_err_t imu_init(void)
{
    ESP_RETURN_ON_ERROR(configure_i2c(), TAG, "I2C setup failed");

    /* Start from a known state; afterwards both sensors are in suspend mode */
    ESP_RETURN_ON_ERROR(bmi160_write(BMI160_REG_CMD, BMI160_CMD_SOFTRESET), TAG, "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(bmi160_read(BMI160_REG_CHIP_ID, &chip_id, 1), TAG, "chip ID read failed");
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID 0x%02X (expected 0x%02X)", chip_id, BMI160_CHIP_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(bmi160_write_verified(BMI160_REG_ACC_CONF, IMU_ODR_CONF), TAG, "ACC_CONF");
    ESP_RETURN_ON_ERROR(bmi160_write_verified(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_4G), TAG, "ACC_RANGE");
    ESP_RETURN_ON_ERROR(bmi160_write_verified(BMI160_REG_GYR_CONF, IMU_ODR_CONF), TAG, "GYR_CONF");
    ESP_RETURN_ON_ERROR(bmi160_write_verified(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_2000DPS), TAG, "GYR_RANGE");

    /* The CMD register ignores new writes while busy, so wait out the max
       start-up time of each sensor (3.8 ms acc, 80 ms gyro) */
    ESP_RETURN_ON_ERROR(bmi160_write(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL), TAG, "acc power up");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(bmi160_write(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL), TAG, "gyro power up");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* acc_pmu_status is bits 5:4, gyr_pmu_status bits 3:2; 0b01 = normal */
    uint8_t pmu = 0;
    ESP_RETURN_ON_ERROR(bmi160_read(BMI160_REG_PMU_STATUS, &pmu, 1), TAG, "PMU read failed");
    if (((pmu >> 4) & 0x3) != 1 || ((pmu >> 2) & 0x3) != 1) {
        ESP_LOGE(TAG, "Sensors not in normal mode (PMU_STATUS 0x%02X)", pmu);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "BMI160 ready: acc +-4 g, gyro +-2000 deg/s, ODR conf 0x%02X", IMU_ODR_CONF);

    restore_offsets();
    /* The first sample after changing the offsets is invalid */
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t *out)
{
    /* One burst read keeps all six axes from the same sample */
    uint8_t raw[12];
    esp_err_t err = bmi160_read(BMI160_REG_DATA_GYR_X, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 3; i++) {
        int16_t g = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
        int16_t a = (int16_t)(raw[6 + 2 * i] | (raw[6 + 2 * i + 1] << 8));
        out->gyro[i] = g * (DEG_TO_RAD / GYR_LSB_PER_DPS);
        out->acc[i] = a * (GRAVITY / ACC_LSB_PER_G);
    }
    return ESP_OK;
}

esp_err_t imu_calibrate(void)
{
    ESP_RETURN_ON_ERROR(bmi160_write_verified(BMI160_REG_FOC_CONF, IMU_FOC_CONF), TAG, "FOC_CONF");
    ESP_RETURN_ON_ERROR(bmi160_write(BMI160_REG_CMD, BMI160_CMD_START_FOC), TAG, "start FOC");

    int waited_ms = 0;
    uint8_t status = 0;
    while (true) {
        ESP_RETURN_ON_ERROR(bmi160_read(BMI160_REG_STATUS, &status, 1), TAG, "status read");
        if (status & BMI160_STATUS_FOC_RDY) {
            break;
        }
        if (waited_ms >= FOC_TIMEOUT_MS) {
            ESP_LOGE(TAG, "FOC did not finish within %d ms", FOC_TIMEOUT_MS);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }

    /* Enable the offsets, keeping the gyro offset MSBs FOC wrote into OFFSET_6 */
    uint8_t off[7];
    ESP_RETURN_ON_ERROR(bmi160_read(BMI160_REG_OFFSET_0, off, sizeof(off)), TAG, "offset read");
    off[6] |= BMI160_OFFSET6_ACC_OFF_EN | BMI160_OFFSET6_GYR_OFF_EN;
    ESP_RETURN_ON_ERROR(bmi160_write(BMI160_REG_OFFSET_6, off[6]), TAG, "offset enable");
    ESP_LOGI(TAG, "FOC done after ~%d ms", waited_ms);
    log_offsets(off);
    save_offsets(off);

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}
