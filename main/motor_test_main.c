/* TB6612 Motor Test

   Ramps motor A of an Adafruit TB6612 breakout from 0% to 100% duty cycle
   and back down again in 10% steps, forever. The on-board RGB LED shows the
   current duty cycle: green at 0%, fading through yellow to red at 100%.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "motor_test";

/* Wiring to the TB6612 breakout */
#define MOTOR_AIN2_GPIO     36
#define MOTOR_AIN1_GPIO     37
#define MOTOR_PWMA_GPIO     39

/* On-board WS2812 RGB LED (ESP32-S3-DevKitC-1 v1.0) */
#define RGB_LED_GPIO        48
#define RGB_LED_BRIGHTNESS  32  /* max channel value, 0-255; the LED is very bright */

/* PWM settings: 20 kHz is above the audible range and within the TB6612's 100 kHz limit */
#define MOTOR_PWM_TIMER     LEDC_TIMER_0
#define MOTOR_PWM_MODE      LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_CHANNEL   LEDC_CHANNEL_0
#define MOTOR_PWM_RES       LEDC_TIMER_10_BIT
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_MAX_DUTY  ((1 << MOTOR_PWM_RES) - 1)

#define STEP_PERCENT        10
#define STEP_DELAY_MS       1000

/* N20 motor quadrature encoder outputs */
#define ENCODER_C1_GPIO     4
#define ENCODER_C2_GPIO     5
/* Hardware counter limits; the driver accumulates past them into a 32-bit count */
#define ENCODER_PCNT_HIGH_LIMIT  30000
#define ENCODER_PCNT_LOW_LIMIT   -30000
#define ENCODER_PRINT_PERIOD_MS  250

static led_strip_handle_t led_strip;
static pcnt_unit_handle_t encoder_unit;

static void configure_rgb_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

/* Map 0-100% onto a green -> yellow -> red gradient */
static void show_speed_on_led(int percent)
{
    uint32_t red, green;
    if (percent <= 50) {
        red = RGB_LED_BRIGHTNESS * percent / 50;
        green = RGB_LED_BRIGHTNESS;
    } else {
        red = RGB_LED_BRIGHTNESS;
        green = RGB_LED_BRIGHTNESS * (100 - percent) / 50;
    }
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, red, green, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

/* Decode C1/C2 in hardware with the pulse counter, counting all 4 edges per encoder cycle */
static void configure_encoder(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &encoder_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_C1_GPIO,
        .level_gpio_num = ENCODER_C2_GPIO,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &chan_a_config, &chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_C2_GPIO,
        .level_gpio_num = ENCODER_C1_GPIO,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Many N20 encoders have open-collector outputs, so enable the internal pull-ups */
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_C1_GPIO));
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_C2_GPIO));

    /* Watch points at the limits let the driver accumulate counts on overflow */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));
}

static void encoder_print_task(void *arg)
{
    int last_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_PRINT_PERIOD_MS));
        int count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &count));
        int counts_per_sec = (count - last_count) * 1000 / ENCODER_PRINT_PERIOD_MS;
        last_count = count;
        ESP_LOGI(TAG, "Encoder C1=%d C2=%d  count %d  speed %d counts/s",
                 gpio_get_level(ENCODER_C1_GPIO), gpio_get_level(ENCODER_C2_GPIO),
                 count, counts_per_sec);
    }
}

static void configure_motor(void)
{
    /* Direction pins: AIN1 high, AIN2 low = forward (clockwise) */
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << MOTOR_AIN1_GPIO) | (1ULL << MOTOR_AIN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_conf));
    gpio_set_level(MOTOR_AIN1_GPIO, 0);
    gpio_set_level(MOTOR_AIN2_GPIO, 1);

    ledc_timer_config_t timer_conf = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .gpio_num = MOTOR_PWMA_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

static void set_motor_speed(int percent)
{
    uint32_t duty = (MOTOR_PWM_MAX_DUTY * percent) / 100;
    ESP_LOGI(TAG, "Motor PWM %3d%% (duty %lu/%d)", percent, (unsigned long)duty, MOTOR_PWM_MAX_DUTY);
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL));
    show_speed_on_led(percent);
}

void app_main(void)
{
    configure_rgb_led();
    configure_motor();
    configure_encoder();
    xTaskCreate(encoder_print_task, "encoder_print", 4096, NULL, 5, NULL);
    int mode = 0;

    while (1) {
        /* Ramp up: 0% -> 100% */
        for (int percent = 0; percent <= 100; percent += STEP_PERCENT) {
            set_motor_speed(percent);
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
        }
        /* Ramp down: 90% -> 10% (0% and 100% are covered by the up ramp) */
        for (int percent = 100 - STEP_PERCENT; percent > 0; percent -= STEP_PERCENT) {
            set_motor_speed(percent);
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
        }
        if (mode == 1) {
            mode = 0;
            gpio_set_level(MOTOR_AIN1_GPIO, 0);
            gpio_set_level(MOTOR_AIN2_GPIO, 1);
        } else {
            mode = 1;
            gpio_set_level(MOTOR_AIN1_GPIO, 1);
            gpio_set_level(MOTOR_AIN2_GPIO, 0);
        }
    }
}
