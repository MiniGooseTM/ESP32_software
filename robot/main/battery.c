/* Battery voltage through a divider on BATTERY_ADC_GPIO

   The divider maps the battery from empty to full onto BATTERY_MV_EMPTY ..
   BATTERY_MV_FULL at the pin. Each reading averages a burst of ADC samples,
   and the readings are low-pass filtered so the voltage sag while the motors
   pull current doesn't make the charge jump around.
*/
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "config.h"
#include "battery.h"

static const char *TAG = "battery";

#define BATTERY_TASK_STACK      3072
#define BATTERY_TASK_PRIORITY   2
#define SAMPLES_PER_READING     16
/* Charge must rise this far above BATTERY_LOW_PERCENT before another low
   battery warning is logged */
#define LOW_HYSTERESIS_PERCENT  3

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static adc_channel_t channel;

/* Filtered voltage at the pin, -1 until the first reading */
static atomic_int pin_mv = -1;

static int mv_to_percent(int mv)
{
    float pct = (float)(mv - BATTERY_MV_EMPTY) * 100.0f / (BATTERY_MV_FULL - BATTERY_MV_EMPTY);
    return (int)lroundf(fminf(fmaxf(pct, 0.0f), 100.0f));
}

static bool adc_setup(void)
{
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &unit, &channel) != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC pin, no battery reading. "
                 "Wire the divider to one of GPIO1-10 and set BATTERY_ADC_GPIO in config.h.",
                 BATTERY_ADC_GPIO);
        return false;
    }
    if (unit != ADC_UNIT_1) {
        ESP_LOGW(TAG, "GPIO%d is on ADC2, which WiFi also uses: some readings will fail",
                 BATTERY_ADC_GPIO);
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,   /* about 0-3.1 V: covers BATTERY_MV_FULL */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .chan = channel,
        .atten = chan_cfg.atten,
        .bitwidth = chan_cfg.bitwidth,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &adc);
    if (err == ESP_OK) {
        err = adc_oneshot_config_channel(adc, channel, &chan_cfg);
    }
    if (err == ESP_OK) {
        err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC setup failed (%s), no battery reading", esp_err_to_name(err));
        return false;
    }
    return true;
}

/* Averaged voltage at the pin in mV, -1 if no sample could be read */
static int read_pin_mv(void)
{
    int sum = 0, n = 0;
    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        int raw;
        if (adc_oneshot_read(adc, channel, &raw) == ESP_OK) {
            sum += raw;
            n++;
        }
    }
    int mv;
    if (n == 0 || adc_cali_raw_to_voltage(cali, (sum + n / 2) / n, &mv) != ESP_OK) {
        return -1;
    }
    return mv;
}

static void battery_task(void *arg)
{
    const float alpha = (float)BATTERY_SAMPLE_MS / (BATTERY_FILTER_TAU_MS + BATTERY_SAMPLE_MS);
    float filtered = -1.0f;
    bool low = false;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        int mv = read_pin_mv();
        if (mv >= 0) {
            if (filtered < 0.0f) {
                filtered = mv;
                ESP_LOGI(TAG, "Battery at %d%% (%d mV at GPIO%d)", mv_to_percent(mv), mv,
                         BATTERY_ADC_GPIO);
            } else {
                filtered += alpha * (mv - filtered);
            }
            int filtered_mv = (int)lroundf(filtered);
            atomic_store(&pin_mv, filtered_mv);

            int pct = mv_to_percent(filtered_mv);
            if (!low && pct < BATTERY_LOW_PERCENT) {
                low = true;
                ESP_LOGW(TAG, "Battery low: %d%% (%d mV at the pin), recharge soon", pct, filtered_mv);
            } else if (low && pct >= BATTERY_LOW_PERCENT + LOW_HYSTERESIS_PERCENT) {
                low = false;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BATTERY_SAMPLE_MS));
    }
}

void battery_start(void)
{
    if (!adc_setup()) {
        return;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(battery_task, "battery", BATTERY_TASK_STACK, NULL,
                                            BATTERY_TASK_PRIORITY, NULL, NET_CORE);
    configASSERT(ok == pdPASS);
}

void battery_get(int *mv_out, int *percent_out)
{
    int mv = atomic_load(&pin_mv);
    if (mv_out) {
        *mv_out = mv;
    }
    if (percent_out) {
        *percent_out = mv < 0 ? -1 : mv_to_percent(mv);
    }
}
