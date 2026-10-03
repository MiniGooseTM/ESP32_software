/* Two-wheeled balancing robot firmware

   Core 1: control loop only (IMU -> policy -> motors at CONTROL_HZ).
   Core 0: WiFi access point, web interface, log streaming, status LED.

   See config.h for pins, rates and limits, control.c for the observation
   the policy sees, and tools/export_policy.py for loading a trained policy.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "config.h"
#include "control.h"
#include "log_stream.h"
#include "motors.h"
#include "net.h"

static const char *TAG = "main";

static led_strip_handle_t led_strip;

static void status_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

/* white: init, blue: idle, red: idle after a fault, green: running,
   purple: manual, yellow: calibrating. Blinks while nobody is driving. */
static void status_led_task(void *arg)
{
    const uint32_t b = RGB_LED_BRIGHTNESS;
    bool blink = false;
    while (true) {
        control_telemetry_t t;
        control_get_telemetry(&t);
        uint32_t r = 0, g = 0, bl = 0;
        switch (t.state) {
        case CTRL_STATE_INIT:        r = g = bl = b / 2;   break;
        case CTRL_STATE_IDLE:        if (t.fault) { r = b; } else { bl = b; } break;
        case CTRL_STATE_RUNNING:     g = b;                break;
        case CTRL_STATE_MANUAL:      r = b; bl = b;        break;
        case CTRL_STATE_CALIBRATING: r = b; g = b;         break;
        }
        blink = !blink;
        bool steady = t.state == CTRL_STATE_RUNNING || t.state == CTRL_STATE_MANUAL;
        if (!steady && blink) {
            r /= 4;
            g /= 4;
            bl /= 4;
        }
        led_strip_set_pixel(led_strip, 0, r, g, bl);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void app_main(void)
{
    /* Motor pins into a safe (coasting) state before anything else */
    motors_init();

    log_stream_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    status_led_init();

    control_start();
    net_start();

    xTaskCreatePinnedToCore(status_led_task, "status_led", 3072, NULL, 2, NULL, NET_CORE);
    ESP_LOGI(TAG, "Started: control on core %d at %d Hz, network on core %d",
             CONTROL_CORE, CONTROL_HZ, NET_CORE);
}
