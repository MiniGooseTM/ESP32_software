/* Control loop

   Runs alone on core 1 at CONTROL_HZ:
     hardware timer ISR -> task notification -> read IMU + encoders ->
     pitch estimate -> observation -> policy -> motor PWM

   The timer and the I2C bus are created from inside this task, so their
   interrupts are allocated on core 1 as well and WiFi traffic on core 0
   can't delay them.

   Communication with core 0 goes through two tiny spinlock-protected structs
   (commands in, telemetry out) that are only held for a memcpy, and through
   log_deferred() for messages. The loop never waits on the network side.
*/
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "config.h"
#include "control.h"
#include "imu.h"
#include "log_stream.h"
#include "motors.h"
#include "policy.h"

static const char *TAG = "control";

#define PERIOD_US       (1000000 / CONTROL_HZ)
#define RAD_TO_DEG      57.29577951308232f
#define DEG_TO_RAD      0.017453292519943295f
#define TWO_PI          6.283185307179586f

/* ======================================================================== */
/* Observation: EDIT THIS TO MATCH THE TRAINING ENVIRONMENT EXACTLY          */
/* (order, units, signs and scaling). Normalisation (VecNormalize) is        */
/* applied afterwards inside the policy, don't apply it here.               */
/* ======================================================================== */

typedef struct {
    imu_sample_t imu;       /* gyro rad/s, acc m/s^2, IMU frame */
    float pitch;            /* rad, forward positive */
    float pitch_rate;       /* rad/s */
    float cmd_lin;          /* m/s */
    float cmd_ang;          /* rad/s */
    float prev_act[2];      /* motor commands of the previous step, [-1, 1] */
    float wheel_vel[2];     /* rad/s, forward positive */
} obs_inputs_t;

#define OBS_DIM 13

static void build_observation(const obs_inputs_t *in, float obs[OBS_DIM])
{
    obs[0] = in->imu.gyro[0];
    obs[1] = in->imu.gyro[1];
    obs[2] = in->imu.gyro[2];
    obs[3] = in->imu.acc[0];
    obs[4] = in->imu.acc[1];
    obs[5] = in->imu.acc[2];
    obs[6] = in->pitch;
    obs[7] = in->cmd_lin;
    obs[8] = in->cmd_ang;
    obs[9] = in->prev_act[0];
    obs[10] = in->prev_act[1];
    obs[11] = in->wheel_vel[0];
    obs[12] = in->wheel_vel[1];
}

_Static_assert(OBS_DIM <= CTRL_MAX_OBS, "raise CTRL_MAX_OBS");

/* ======================================================================== */

#define REQ_START       (1u << 0)
#define REQ_STOP        (1u << 1)
#define REQ_CALIBRATE   (1u << 2)
#define REQ_MANUAL      (1u << 3)

typedef struct {
    float lin, ang;
    float manual[2];
    int64_t last_cmd_us;
    uint32_t requests;
} command_t;

static TaskHandle_t control_task_handle;

static portMUX_TYPE cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static command_t shared_cmd;

static portMUX_TYPE telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static control_telemetry_t shared_telemetry = {
    .state = CTRL_STATE_INIT,
    .stop_reason = "",
};

/* ---- interface for other tasks ----------------------------------------- */

static float clampf(float v, float lo, float hi)
{
    if (!(v == v)) {
        return 0.0f;
    }
    return v < lo ? lo : (v > hi ? hi : v);
}

void control_set_command(float lin, float ang)
{
    lin = clampf(lin, -CMD_MAX_LIN_VEL, CMD_MAX_LIN_VEL);
    ang = clampf(ang, -CMD_MAX_ANG_VEL, CMD_MAX_ANG_VEL);
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&cmd_lock);
    shared_cmd.lin = lin;
    shared_cmd.ang = ang;
    shared_cmd.last_cmd_us = now;
    portEXIT_CRITICAL(&cmd_lock);
}

void control_set_manual(float a, float b)
{
    a = clampf(a, -1.0f, 1.0f);
    b = clampf(b, -1.0f, 1.0f);
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&cmd_lock);
    shared_cmd.manual[0] = a;
    shared_cmd.manual[1] = b;
    shared_cmd.last_cmd_us = now;
    shared_cmd.requests |= REQ_MANUAL;
    portEXIT_CRITICAL(&cmd_lock);
}

static void post_request(uint32_t req)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&cmd_lock);
    shared_cmd.requests |= req;
    shared_cmd.last_cmd_us = now;
    portEXIT_CRITICAL(&cmd_lock);
}

void control_heartbeat(void) { post_request(0); }
void control_request_start(void) { post_request(REQ_START); }
void control_request_stop(void) { post_request(REQ_STOP); }
void control_request_calibrate(void) { post_request(REQ_CALIBRATE); }

void control_get_telemetry(control_telemetry_t *out)
{
    portENTER_CRITICAL(&telemetry_lock);
    memcpy(out, &shared_telemetry, sizeof(*out));
    portEXIT_CRITICAL(&telemetry_lock);
}

const char *control_state_name(ctrl_state_t state)
{
    switch (state) {
    case CTRL_STATE_INIT:        return "init";
    case CTRL_STATE_IDLE:        return "idle";
    case CTRL_STATE_RUNNING:     return "running";
    case CTRL_STATE_MANUAL:      return "manual";
    case CTRL_STATE_CALIBRATING: return "calibrating";
    }
    return "?";
}

/* ---- control task ------------------------------------------------------- */

static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(control_task_handle, &woken);
    return woken == pdTRUE;
}

static void start_loop_timer(void)
{
    gptimer_handle_t timer;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm,
    };
    /* The ISR is installed here, on the calling core */
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

/* Pitch from gravity alone; valid when the robot isn't accelerating */
static float accel_pitch(const imu_sample_t *s)
{
    return atan2f(-s->acc[IMU_FWD_ACC_AXIS], s->acc[IMU_UP_ACC_AXIS]);
}

typedef struct {
    ctrl_state_t state;
    const char *stop_reason;
    int fault;
    bool policy_ok;

    imu_sample_t imu;
    int imu_consecutive_errors;
    uint32_t imu_errors;
    float pitch;
    float pitch_rate;

    int32_t last_counts[NUM_MOTORS];
    float wheel_vel[NUM_MOTORS];

    float act[2];
    float obs[OBS_DIM];

    /* timing */
    int64_t last_tick_us;
    uint32_t ticks;
    uint32_t overruns;
    float infer_us;
    /* running maxima / sums over the current one-second window */
    float win_infer_max, win_busy_max, win_jitter_max, win_period_sum;
    int win_count;
    /* last completed window */
    float pub_infer_max, pub_busy_max, pub_jitter_max, pub_period_avg;
} ctrl_t;

static void stop_motors(ctrl_t *c, ctrl_state_t next, const char *reason, int fault)
{
    motors_coast();
    c->act[0] = c->act[1] = 0.0f;
    if (reason) {
        c->stop_reason = reason;
        c->fault = fault;
        log_deferred(fault ? ESP_LOG_WARN : ESP_LOG_INFO, "%s -> %s: %s",
                     control_state_name(c->state), control_state_name(next), reason);
    }
    c->state = next;
}

static void publish_telemetry(const ctrl_t *c, float cmd_lin, float cmd_ang)
{
    control_telemetry_t t = {
        .state = c->state,
        .stop_reason = c->stop_reason,
        .fault = c->fault,
        .pitch_deg = c->pitch * RAD_TO_DEG,
        .pitch_rate_dps = c->pitch_rate * RAD_TO_DEG,
        .wheel_vel = {c->wheel_vel[0], c->wheel_vel[1]},
        .cmd_lin = cmd_lin,
        .cmd_ang = cmd_ang,
        .act = {c->act[0], c->act[1]},
        .obs_dim = OBS_DIM,
        .infer_us = c->infer_us,
        .infer_us_max = c->pub_infer_max,
        .busy_us_max = c->pub_busy_max,
        .period_us_avg = c->pub_period_avg,
        .jitter_us_max = c->pub_jitter_max,
        .overruns = c->overruns,
        .ticks = c->ticks,
        .imu_errors = c->imu_errors,
        .policy_placeholder = policy_is_placeholder(),
    };
    memcpy(t.gyro, c->imu.gyro, sizeof(t.gyro));
    memcpy(t.acc, c->imu.acc, sizeof(t.acc));
    memcpy(t.obs, c->obs, sizeof(c->obs));

    portENTER_CRITICAL(&telemetry_lock);
    memcpy(&shared_telemetry, &t, sizeof(t));
    portEXIT_CRITICAL(&telemetry_lock);
}

static void run_calibration(ctrl_t *c)
{
    stop_motors(c, CTRL_STATE_CALIBRATING, NULL, 0);
    publish_telemetry(c, 0.0f, 0.0f);
    log_deferred(ESP_LOG_INFO, "Calibrating in %d ms - hold the robot still%s", IMU_CALIB_SETTLE_MS,
                 IMU_FOC_CONF == 0x7D ? " with the IMU flat (Z up)" : "");
    vTaskDelay(pdMS_TO_TICKS(IMU_CALIB_SETTLE_MS));

    /* Motors are off, so blocking on the UART log here is harmless */
    esp_err_t err = imu_calibrate();
    if (err != ESP_OK) {
        log_deferred(ESP_LOG_ERROR, "Calibration failed: %s", esp_err_to_name(err));
    } else {
        log_deferred(ESP_LOG_INFO, "Calibration done");
    }
    if (imu_read(&c->imu) == ESP_OK) {
        c->pitch = accel_pitch(&c->imu);
    }
    c->state = CTRL_STATE_IDLE;

    /* Drop the ticks that piled up and restart the timing statistics */
    ulTaskNotifyTake(pdTRUE, 0);
    c->last_tick_us = 0;
}

static void control_task(void *arg)
{
    static ctrl_t c = {
        .state = CTRL_STATE_INIT,
        .stop_reason = "",
    };

    /* ---- init: everything here runs on core 1 ---- */
    control_task_handle = xTaskGetCurrentTaskHandle();
    while (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed, retrying in 2 s (motors stay off)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    policy_selftest();
    c.policy_ok = policy_obs_dim() == OBS_DIM && policy_act_dim() == 2;
    if (!c.policy_ok) {
        ESP_LOGE(TAG, "Policy expects %d obs / %d actions but build_observation() makes %d / 2 - "
                 "start is disabled until they match", policy_obs_dim(), policy_act_dim(), OBS_DIM);
    }
    if (policy_is_placeholder()) {
        ESP_LOGW(TAG, "Running the PLACEHOLDER policy (all outputs 0)");
    }

    if (imu_read(&c.imu) == ESP_OK) {
        c.pitch = accel_pitch(&c.imu);
    }
    encoders_read(c.last_counts);

    /* Discard any start/stop requests that arrived during init */
    portENTER_CRITICAL(&cmd_lock);
    shared_cmd.requests = 0;
    portEXIT_CRITICAL(&cmd_lock);

    start_loop_timer();
    c.state = CTRL_STATE_IDLE;
    ESP_LOGI(TAG, "Control loop running at %d Hz on core %d", CONTROL_HZ, xPortGetCoreID());

    while (true) {
        uint32_t pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (pending == 0) {
            /* The timer stopped firing; never leave the motors running blind */
            if (c.state == CTRL_STATE_RUNNING || c.state == CTRL_STATE_MANUAL) {
                stop_motors(&c, CTRL_STATE_IDLE, "loop timer stalled", 1);
            }
            continue;
        }
        int64_t now = esp_timer_get_time();
        c.overruns += pending - 1;
        c.ticks++;

        float dt = 1.0f / CONTROL_HZ;
        if (c.last_tick_us) {
            float period = (float)(now - c.last_tick_us);
            c.win_period_sum += period;
            c.win_count++;
            float jitter = fabsf(period - PERIOD_US);
            if (jitter > c.win_jitter_max) {
                c.win_jitter_max = jitter;
            }
            /* Use the real interval, but don't let one hiccup wreck the filter */
            dt = fminf(fmaxf(period, 0.5f * PERIOD_US), 2.0f * PERIOD_US) * 1e-6f;
        }
        c.last_tick_us = now;

        /* ---- 1. take commands ---- */
        command_t cmd;
        portENTER_CRITICAL(&cmd_lock);
        cmd = shared_cmd;
        shared_cmd.requests = 0;
        portEXIT_CRITICAL(&cmd_lock);

        int64_t cmd_age_ms = (now - cmd.last_cmd_us) / 1000;
        bool cmd_fresh = cmd.last_cmd_us != 0 && cmd_age_ms < CMD_TIMEOUT_MS;
        float cmd_lin = cmd_fresh ? cmd.lin : 0.0f;
        float cmd_ang = cmd_fresh ? cmd.ang : 0.0f;

        /* ---- 2. sense ---- */
        if (imu_read(&c.imu) == ESP_OK) {
            c.imu_consecutive_errors = 0;
            c.pitch_rate = IMU_TILT_GYRO_SIGN * c.imu.gyro[IMU_TILT_GYRO_AXIS];
            c.pitch = TILT_FILTER_ALPHA * (c.pitch + c.pitch_rate * dt)
                      + (1.0f - TILT_FILTER_ALPHA) * accel_pitch(&c.imu);
        } else {
            /* Keep the previous sample for this step */
            c.imu_errors++;
            c.imu_consecutive_errors++;
        }

        int32_t counts[NUM_MOTORS];
        encoders_read(counts);
        for (int i = 0; i < NUM_MOTORS; i++) {
            c.wheel_vel[i] = (counts[i] - c.last_counts[i]) * (TWO_PI / ENCODER_COUNTS_PER_REV) / dt;
            c.last_counts[i] = counts[i];
        }

        /* ---- 3. state machine and safety ---- */
        bool driving = c.state == CTRL_STATE_RUNNING || c.state == CTRL_STATE_MANUAL;
        bool upright = fabsf(c.pitch) < START_MAX_ANGLE_DEG * DEG_TO_RAD;

        if (cmd.requests & REQ_STOP) {
            if (driving) {
                stop_motors(&c, CTRL_STATE_IDLE, "stop requested", 0);
            }
        } else if (cmd.requests & REQ_CALIBRATE) {
            if (c.state == CTRL_STATE_IDLE) {
                run_calibration(&c);
                publish_telemetry(&c, 0.0f, 0.0f);
                continue;
            }
            log_deferred(ESP_LOG_WARN, "Calibrate ignored: stop the robot first");
        } else if (cmd.requests & REQ_START) {
            if (c.state == CTRL_STATE_RUNNING) {
                /* already running */
            } else if (!c.policy_ok) {
                log_deferred(ESP_LOG_ERROR, "Start refused: policy / observation size mismatch");
            } else if (!upright) {
                log_deferred(ESP_LOG_WARN, "Start refused: pitch %.1f deg, hold the robot within %.0f deg of upright",
                             c.pitch * RAD_TO_DEG, START_MAX_ANGLE_DEG);
            } else if (c.imu_consecutive_errors) {
                log_deferred(ESP_LOG_WARN, "Start refused: IMU not responding");
            } else {
                motors_coast();
                c.act[0] = c.act[1] = 0.0f;
                c.state = CTRL_STATE_RUNNING;
                c.stop_reason = "";
                c.fault = 0;
                log_deferred(ESP_LOG_INFO, "Running (pitch %.1f deg)", c.pitch * RAD_TO_DEG);
            }
        } else if (cmd.requests & REQ_MANUAL) {
            if (c.state == CTRL_STATE_IDLE) {
                c.state = CTRL_STATE_MANUAL;
                c.stop_reason = "";
                c.fault = 0;
                log_deferred(ESP_LOG_INFO, "Manual motor control");
            }
        }

        driving = c.state == CTRL_STATE_RUNNING || c.state == CTRL_STATE_MANUAL;
        if (driving) {
            bool check_fall = c.state == CTRL_STATE_RUNNING || MANUAL_FALL_CUTOFF;
            if (check_fall && fabsf(c.pitch) > FALL_ANGLE_DEG * DEG_TO_RAD) {
                stop_motors(&c, CTRL_STATE_IDLE, "fell over", 1);
            } else if (c.imu_consecutive_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
                stop_motors(&c, CTRL_STATE_IDLE, "IMU read errors", 1);
            } else if (c.state == CTRL_STATE_MANUAL && !cmd_fresh) {
                stop_motors(&c, CTRL_STATE_IDLE, "manual command timeout", 0);
            } else if (c.state == CTRL_STATE_RUNNING && LINK_LOST_STOP_MS > 0
                       && cmd_age_ms > LINK_LOST_STOP_MS) {
                stop_motors(&c, CTRL_STATE_IDLE, "link lost", 1);
            }
        }

        /* ---- 4. act ---- */
        obs_inputs_t in = {
            .imu = c.imu,
            .pitch = c.pitch,
            .pitch_rate = c.pitch_rate,
            .cmd_lin = cmd_lin,
            .cmd_ang = cmd_ang,
            .prev_act = {c.act[0], c.act[1]},
            .wheel_vel = {c.wheel_vel[0], c.wheel_vel[1]},
        };
        build_observation(&in, c.obs);

        if (c.state == CTRL_STATE_RUNNING) {
            uint32_t t0 = esp_cpu_get_cycle_count();
            policy_forward(c.obs, c.act);
            c.infer_us = (float)(esp_cpu_get_cycle_count() - t0) / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
            if (c.infer_us > c.win_infer_max) {
                c.win_infer_max = c.infer_us;
            }
            motors_set(c.act[0], c.act[1]);
        } else if (c.state == CTRL_STATE_MANUAL) {
            c.act[0] = cmd.manual[0];
            c.act[1] = cmd.manual[1];
            motors_set(c.act[0], c.act[1]);
        }

        /* ---- 5. bookkeeping ---- */
        float busy = (float)(esp_timer_get_time() - now);
        if (busy > c.win_busy_max) {
            c.win_busy_max = busy;
        }
        if (c.ticks % CONTROL_HZ == 0) {
            c.pub_infer_max = c.win_infer_max;
            c.pub_busy_max = c.win_busy_max;
            c.pub_jitter_max = c.win_jitter_max;
            c.pub_period_avg = c.win_count ? c.win_period_sum / c.win_count : 0.0f;
            c.win_infer_max = c.win_busy_max = c.win_jitter_max = c.win_period_sum = 0.0f;
            c.win_count = 0;
        }
        publish_telemetry(&c, cmd_lin, cmd_ang);
    }
}

void control_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                                            CONTROL_TASK_PRIORITY, NULL, CONTROL_CORE);
    configASSERT(ok == pdPASS);
}
