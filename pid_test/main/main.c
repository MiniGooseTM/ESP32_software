/* PID balance test

   Balances the robot with a tilt PID on core 1 and serves a tuning page on
   core 0 (http://192.168.4.1 after joining the robot's WiFi). Gains can be
   changed live and saved to flash.

   Hardware drivers and settings are shared with the robot firmware in
   robot/main/; PID settings are in pid_config.h.
*/
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "config.h"
#include "balance.h"
#include "log_stream.h"
#include "motors.h"
#include "net.h"

static const char *TAG = "main";

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

    balance_start();
    net_start();
    ESP_LOGI(TAG, "PID test started: control on core %d at %d Hz, network on core %d",
             CONTROL_CORE, CONTROL_HZ, NET_CORE);
}
