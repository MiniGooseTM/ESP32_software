/* TB6612 motor driver + N20 quadrature encoders

   Adapted from main/motor_test_main.c. Each motor has two direction inputs
   (IN1/IN2) and a PWM input driven by an LEDC channel:
     IN1=1 IN2=0  forward     IN1=0 IN2=1  reverse
     IN1=0 IN2=0  coast       IN1=1 IN2=1  short brake
*/
#include <math.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "config.h"
#include "motors.h"

static const char *TAG = "motors";

#define MOTOR_PWM_TIMER     LEDC_TIMER_0
#define MOTOR_PWM_MODE      LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_MAX_DUTY  ((1 << MOTOR_PWM_RES_BITS) - 1)

/* Hardware counter limits; the driver accumulates past them into a 32-bit count */
#define ENCODER_PCNT_HIGH_LIMIT  30000
#define ENCODER_PCNT_LOW_LIMIT   -30000

typedef struct {
    int in1_gpio;
    int in2_gpio;
    int pwm_gpio;
    ledc_channel_t pwm_channel;
    int sign;
    int encoder_c1_gpio;
    int encoder_c2_gpio;
    int encoder_sign;
    pcnt_unit_handle_t encoder_unit;
} motor_t;

static motor_t motors[NUM_MOTORS] = {
    {
        .in1_gpio = MOTOR_AIN1_GPIO,
        .in2_gpio = MOTOR_AIN2_GPIO,
        .pwm_gpio = MOTOR_PWMA_GPIO,
        .pwm_channel = LEDC_CHANNEL_0,
        .sign = MOTOR_A_SIGN,
        .encoder_c1_gpio = ENCODER_A_C1_GPIO,
        .encoder_c2_gpio = ENCODER_A_C2_GPIO,
        .encoder_sign = ENCODER_A_SIGN,
    },
    {
        .in1_gpio = MOTOR_BIN1_GPIO,
        .in2_gpio = MOTOR_BIN2_GPIO,
        .pwm_gpio = MOTOR_PWMB_GPIO,
        .pwm_channel = LEDC_CHANNEL_1,
        .sign = MOTOR_B_SIGN,
        .encoder_c1_gpio = ENCODER_B_C1_GPIO,
        .encoder_c2_gpio = ENCODER_B_C2_GPIO,
        .encoder_sign = ENCODER_B_SIGN,
    },
};

/* Decode C1/C2 in hardware with the pulse counter, counting all 4 edges per encoder cycle */
static void configure_encoder(motor_t *motor)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &motor->encoder_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor->encoder_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = motor->encoder_c1_gpio,
        .level_gpio_num = motor->encoder_c2_gpio,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor->encoder_unit, &chan_a_config, &chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = motor->encoder_c2_gpio,
        .level_gpio_num = motor->encoder_c1_gpio,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor->encoder_unit, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Many N20 encoders have open-collector outputs, so enable the internal pull-ups */
    ESP_ERROR_CHECK(gpio_pullup_en(motor->encoder_c1_gpio));
    ESP_ERROR_CHECK(gpio_pullup_en(motor->encoder_c2_gpio));

    /* Watch points at the limits let the driver accumulate counts on overflow */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor->encoder_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor->encoder_unit, ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(motor->encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(motor->encoder_unit));
}

static void configure_motor(motor_t *motor)
{
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << motor->in1_gpio) | (1ULL << motor->in2_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_conf));
    gpio_set_level(motor->in1_gpio, 0);
    gpio_set_level(motor->in2_gpio, 0);

    ledc_channel_config_t channel_conf = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = motor->pwm_channel,
        .timer_sel = MOTOR_PWM_TIMER,
        .gpio_num = motor->pwm_gpio,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

void motors_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES_BITS,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (int i = 0; i < NUM_MOTORS; i++) {
        configure_motor(&motors[i]);
        configure_encoder(&motors[i]);
    }
    ESP_LOGI(TAG, "Motors ready (coasting), PWM %d Hz, %d bit", MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
}

static void set_one(motor_t *motor, float cmd)
{
    cmd *= motor->sign;
    if (isnan(cmd)) {
        cmd = 0.0f;
    } else if (cmd > 1.0f) {
        cmd = 1.0f;
    } else if (cmd < -1.0f) {
        cmd = -1.0f;
    }

    float mag = fabsf(cmd);
    float duty_frac = 0.0f;
    if (mag > MOTOR_DEADBAND) {
        duty_frac = MOTOR_MIN_DUTY + (1.0f - MOTOR_MIN_DUTY) * (mag - MOTOR_DEADBAND) / (1.0f - MOTOR_DEADBAND);
    }
    uint32_t duty = (uint32_t)(duty_frac * MOTOR_PWM_MAX_DUTY + 0.5f);

    bool forward = cmd >= 0.0f;
    gpio_set_level(motor->in1_gpio, forward);
    gpio_set_level(motor->in2_gpio, !forward);
    ledc_set_duty(MOTOR_PWM_MODE, motor->pwm_channel, duty);
    ledc_update_duty(MOTOR_PWM_MODE, motor->pwm_channel);
}

void motors_set(float a, float b)
{
    set_one(&motors[0], a);
    set_one(&motors[1], b);
}

void motors_coast(void)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_set_duty(MOTOR_PWM_MODE, motors[i].pwm_channel, 0);
        ledc_update_duty(MOTOR_PWM_MODE, motors[i].pwm_channel);
        gpio_set_level(motors[i].in1_gpio, 0);
        gpio_set_level(motors[i].in2_gpio, 0);
    }
}

void motors_brake(void)
{
    /* The TB6612 short-brakes when IN1=IN2=1 and PWM is high */
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_level(motors[i].in1_gpio, 1);
        gpio_set_level(motors[i].in2_gpio, 1);
        ledc_set_duty(MOTOR_PWM_MODE, motors[i].pwm_channel, MOTOR_PWM_MAX_DUTY);
        ledc_update_duty(MOTOR_PWM_MODE, motors[i].pwm_channel);
    }
}

void encoders_read(int32_t counts[NUM_MOTORS])
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        int c = 0;
        pcnt_unit_get_count(motors[i].encoder_unit, &c);
        counts[i] = motors[i].encoder_sign * c;
    }
}
