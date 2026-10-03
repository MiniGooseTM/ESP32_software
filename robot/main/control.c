/* Control loop

   Runs alone on core 1 at CONTROL_HZ:
     hardware timer ISR -> task notification -> read IMU + encoders ->
     tilt estimate -> observation history -> policy -> motor PWM

   The timer and the I2C bus are created from inside this task, so their
   interrupts are allocated on core 1 as well and WiFi traffic on core 0
   can't delay them.

   Communication with core 0 goes through two tiny spinlock-protected structs
   (commands in, telemetry out) that are only held for a memcpy, and through
   log_deferred() for messages. The loop never waits on the network side.
*/
#include <math.h>
#include <stdlib.h>
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

/* Largest |output - SB3 action| accepted on a policy's test vectors */
#define POLICY_TEST_MAX_ERROR       1e-4f
#define POLICY_RELOAD_TIMEOUT_MS    3000

/* ======================================================================== */
/* Observation: must match the training environment exactly                 */
/* (ACTOR_FRAME_FIELDS and _actor_frame() in envs/balance_env.py).          */
/* The policy file carries a CRC of the training field names; it must      */
/* match OBS_FRAME_FIELDS (policy_mismatch() below).                        */
/* ======================================================================== */

typedef struct {
    float gyro[3];          /* rad/s, body frame (x right, y forward, z up) */
    float roll;             /* rad, right lean positive */
    float pitch;            /* rad, forward lean NEGATIVE */
    float cmd_lin;          /* m/s */
    float cmd_ang;          /* rad/s */
    float prev_act[2];      /* left, right motor commands of the previous step, [-1, 1] */
    float wheel_vel[2];     /* left, right, rad/s, forward positive */
} obs_inputs_t;

#define OBS_FRAME_FIELDS "v_cmd,omega_cmd,roll,pitch,roll_rate,pitch_rate,yaw_rate," \
                         "left_wheel_velocity,right_wheel_velocity," \
                         "previous_action_left,previous_action_right"
#define OBS_FRAME_DIM   11
#define OBS_HISTORY     8       /* EnvironmentConfig.history_length */
#define OBS_DIM         (OBS_FRAME_DIM * OBS_HISTORY)

static void build_frame(const obs_inputs_t *in, float frame[OBS_FRAME_DIM])
{
    frame[0] = in->cmd_lin / CMD_MAX_LIN_VEL;
    frame[1] = in->cmd_ang / CMD_MAX_ANG_VEL;
    frame[2] = in->roll;
    frame[3] = in->pitch;
    frame[4] = in->gyro[1];     /* roll rate */
    frame[5] = in->gyro[0];     /* pitch rate */
    frame[6] = in->gyro[2];     /* yaw rate */
    frame[7] = in->wheel_vel[0] / WHEEL_MAX_SPEED_RAD_S;
    frame[8] = in->wheel_vel[1] / WHEEL_MAX_SPEED_RAD_S;
    frame[9] = in->prev_act[0];
    frame[10] = in->prev_act[1];
}

/* The policy sees the last OBS_HISTORY frames, newest first. With restart
   set, every slot gets the new frame, as at the start of a training episode. */
static void push_frame(const float frame[OBS_FRAME_DIM], float obs[OBS_DIM], bool restart)
{
    if (restart) {
        for (int k = 0; k < OBS_HISTORY; k++) {
            memcpy(obs + k * OBS_FRAME_DIM, frame, OBS_FRAME_DIM * sizeof(float));
        }
        return;
    }
    memmove(obs + OBS_FRAME_DIM, obs, (OBS_DIM - OBS_FRAME_DIM) * sizeof(float));
    memcpy(obs, frame, OBS_FRAME_DIM * sizeof(float));
}

/* Telemetry carries the newest frame only */
_Static_assert(OBS_FRAME_DIM <= CTRL_MAX_OBS, "raise CTRL_MAX_OBS");

static bool nearly_equal(float a, float b)
{
    return fabsf(a - b) <= 1e-4f * fmaxf(1.0f, fabsf(b));
}

/* The policy must have been trained with exactly this observation and
   these limits, and its forward pass must reproduce the SB3 actions stored
   with it. Called for the policy being loaded and for an upload. */
const char *control_check_policy(const policy_net_t *net)
{
    const policy_file_header_t *h = net->header;
    if (h->input_size != OBS_DIM || h->output_size != 2) {
        return "input / output size";
    }
    if (h->frame_size != OBS_FRAME_DIM || h->history_length != OBS_HISTORY
        || h->frame_fields_crc != policy_crc32(OBS_FRAME_FIELDS, strlen(OBS_FRAME_FIELDS))) {
        return "observation frame (OBS_FRAME_FIELDS, OBS_HISTORY)";
    }
    if (!nearly_equal(h->control_period_s, 1.0f / CONTROL_HZ)) {
        return "control rate (CONTROL_HZ)";
    }
    if (!nearly_equal(h->max_linear_command, CMD_MAX_LIN_VEL)
        || !nearly_equal(h->max_angular_command, CMD_MAX_ANG_VEL)) {
        return "command limits (CMD_MAX_LIN_VEL, CMD_MAX_ANG_VEL)";
    }
    if (!nearly_equal(h->half_track_m, WHEEL_HALF_TRACK_M)) {
        return "WHEEL_HALF_TRACK_M";
    }
    if (!nearly_equal(h->wheel_no_load_speed, WHEEL_MAX_SPEED_RAD_S)) {
        return "WHEEL_MAX_SPEED_RAD_S";
    }
    /* On the heap: this also runs on the main task, which has a small stack */
    float *bufs = malloc(2 * POLICY_MAX_WIDTH * sizeof(float));
    if (!bufs) {
        return "out of memory for the self-test";
    }
    float err = policy_net_test_error(net, bufs, bufs + POLICY_MAX_WIDTH);
    free(bufs);
    if (!(err < POLICY_TEST_MAX_ERROR)) {
        return "forward pass doesn't reproduce the test vectors in the file";
    }
    return NULL;
}

/* ======================================================================== */

#define REQ_START       (1u << 0)
#define REQ_STOP        (1u << 1)
#define REQ_CALIBRATE   (1u << 2)
#define REQ_MANUAL      (1u << 3)
#define REQ_POLICY      (1u << 4)   /* load the policy again */

typedef struct {
    float lin, ang;
    float manual[2];
    int64_t last_cmd_us;
    uint32_t requests;
    bool policy_busy;               /* a policy upload is in progress */
} command_t;

/* The active policy as the control loop sees it. Written by select_policy()
   only: in control_start() before the control task runs, then only by the
   control task. */
static struct {
    bool ok;                        /* loaded and checked: start allowed */
    float tilt_limit;               /* rad, motors are cut past this roll or pitch */
    char name[sizeof(((policy_file_header_t *)0)->name)];
    uint32_t crc;
    policy_source_t source;
    uint32_t generation;            /* counts (re)loads */
} active_policy;

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
    /* Keep the outer wheel within the full speed, as in training */
    float outer_wheel = fabsf(lin) + fabsf(ang) * WHEEL_HALF_TRACK_M;
    if (outer_wheel > CMD_MAX_LIN_VEL) {
        float scale = CMD_MAX_LIN_VEL / outer_wheel;
        lin *= scale;
        ang *= scale;
    }
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

static void set_policy_busy(bool busy)
{
    portENTER_CRITICAL(&cmd_lock);
    shared_cmd.policy_busy = busy;
    portEXIT_CRITICAL(&cmd_lock);
}

const char *control_begin_policy_update(void)
{
    portENTER_CRITICAL(&cmd_lock);
    bool already_busy = shared_cmd.policy_busy;
    shared_cmd.policy_busy = true;
    portEXIT_CRITICAL(&cmd_lock);
    if (already_busy) {
        return "another policy update is in progress";
    }
    /* Once the loop has seen the flag for a tick it starts nothing new, so
       the state it reports after that stays put */
    vTaskDelay(pdMS_TO_TICKS(3 * 1000 / CONTROL_HZ) + 1);
    control_telemetry_t t;
    control_get_telemetry(&t);
    if (t.state != CTRL_STATE_IDLE) {
        set_policy_busy(false);
        return "stop the robot first";
    }
    return NULL;
}

void control_end_policy_update(void) { set_policy_busy(false); }

void control_reload_policy(void)
{
    control_telemetry_t t;
    control_get_telemetry(&t);
    uint32_t generation = t.policy_generation;
    post_request(REQ_POLICY);
    for (int i = 0; i < POLICY_RELOAD_TIMEOUT_MS / 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        control_get_telemetry(&t);
        if (t.policy_generation != generation) {
            return;
        }
    }
    ESP_LOGW(TAG, "Policy reload didn't finish in %d ms", POLICY_RELOAD_TIMEOUT_MS);
}

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

/* Read the IMU and rotate the sample into the body frame */
static esp_err_t imu_read_body(imu_sample_t *out)
{
    imu_sample_t s;
    esp_err_t err = imu_read(&s);
    if (err != ESP_OK) {
        return err;
    }
    static const int axis[3] = {IMU_BODY_X_AXIS, IMU_BODY_Y_AXIS, IMU_BODY_Z_AXIS};
    static const float sign[3] = {IMU_BODY_X_SIGN, IMU_BODY_Y_SIGN, IMU_BODY_Z_SIGN};
    for (int i = 0; i < 3; i++) {
        out->gyro[i] = sign[i] * s.gyro[axis[i]];
        out->acc[i] = sign[i] * s.acc[axis[i]];
    }
    return ESP_OK;
}

/* Roll and pitch from gravity alone, defined as in the training environment
   (_tilt()); valid when the robot isn't accelerating. Body frame sample. */
static void accel_tilt(const imu_sample_t *s, float *roll, float *pitch)
{
    *pitch = atan2f(s->acc[1], s->acc[2]);
    *roll = atan2f(-s->acc[0], hypotf(s->acc[1], s->acc[2]));
}

typedef struct {
    ctrl_state_t state;
    const char *stop_reason;
    int fault;

    imu_sample_t imu;           /* body frame */
    int imu_consecutive_errors;
    uint32_t imu_errors;
    float roll;                 /* rad, right lean positive */
    float pitch;                /* rad, forward lean negative */

    int32_t last_counts[NUM_MOTORS];
    float wheel_vel[NUM_MOTORS];

    float act[2];
    float obs[OBS_DIM];
    bool obs_restart;           /* refill the history on the next tick */

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

static void publish_telemetry(const ctrl_t *c, float cmd_lin, float cmd_ang, bool policy_busy)
{
    control_telemetry_t t = {
        .state = c->state,
        .stop_reason = c->stop_reason,
        .fault = c->fault,
        .pitch_deg = c->pitch * RAD_TO_DEG,
        .pitch_rate_dps = c->imu.gyro[0] * RAD_TO_DEG,
        .wheel_vel = {c->wheel_vel[0], c->wheel_vel[1]},
        .cmd_lin = cmd_lin,
        .cmd_ang = cmd_ang,
        .act = {c->act[0], c->act[1]},
        .obs_dim = OBS_FRAME_DIM,
        .infer_us = c->infer_us,
        .infer_us_max = c->pub_infer_max,
        .busy_us_max = c->pub_busy_max,
        .period_us_avg = c->pub_period_avg,
        .jitter_us_max = c->pub_jitter_max,
        .overruns = c->overruns,
        .ticks = c->ticks,
        .imu_errors = c->imu_errors,
        .policy_source = policy_source_name(active_policy.source),
        .policy_crc = active_policy.crc,
        .policy_ok = active_policy.ok,
        .policy_busy = policy_busy,
        .policy_generation = active_policy.generation,
    };
    memcpy(t.policy_name, active_policy.name, sizeof(t.policy_name));
    memcpy(t.gyro, c->imu.gyro, sizeof(t.gyro));
    memcpy(t.acc, c->imu.acc, sizeof(t.acc));
    memcpy(t.obs, c->obs, OBS_FRAME_DIM * sizeof(float));

    portENTER_CRITICAL(&telemetry_lock);
    memcpy(&shared_telemetry, &t, sizeof(t));
    portEXIT_CRITICAL(&telemetry_lock);
}

static void run_calibration(ctrl_t *c)
{
    stop_motors(c, CTRL_STATE_CALIBRATING, NULL, 0);
    publish_telemetry(c, 0.0f, 0.0f, false);
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
    if (imu_read_body(&c->imu) == ESP_OK) {
        accel_tilt(&c->imu, &c->roll, &c->pitch);
    }
    c->state = CTRL_STATE_IDLE;

    /* Drop the ticks that piled up and restart the timing statistics */
    ulTaskNotifyTake(pdTRUE, 0);
    c->last_tick_us = 0;
}

/* Use the uploaded policy if there is a usable one, else the built-in one.
   Only call while the motors are off. */
static void select_policy(void)
{
    const char *err = "nothing uploaded";
    if (!policy_store_empty()) {
        err = policy_load(POLICY_SOURCE_UPLOADED);
        if (!err) {
            err = control_check_policy(policy_active());
        }
        if (err) {
            log_deferred(ESP_LOG_WARN, "Uploaded policy not used: %s", err);
        }
    }
    if (err) {
        err = policy_load(POLICY_SOURCE_BUILTIN);
        if (!err) {
            err = control_check_policy(policy_active());
        }
    }

    const policy_net_t *net = policy_active();
    active_policy.ok = err == NULL;
    active_policy.source = policy_active_source();
    active_policy.crc = net ? net->crc : 0;
    strlcpy(active_policy.name, net ? net->header->name : "", sizeof(active_policy.name));
    /* The policy was never trained past its tilt limit */
    active_policy.tilt_limit = FALL_ANGLE_DEG * DEG_TO_RAD;
    if (net && net->header->tilt_limit_rad < active_policy.tilt_limit) {
        active_policy.tilt_limit = net->header->tilt_limit_rad;
    }
    active_policy.generation++;

    if (err) {
        log_deferred(ESP_LOG_ERROR, "No usable policy (%s %s) - start is disabled",
                     policy_source_name(active_policy.source), err);
        return;
    }
    const policy_file_header_t *h = net->header;
    int params = 0;
    for (int l = 0; l < h->num_layers; l++) {
        params += (net->layers[l].in + 1) * net->layers[l].out;
    }
    log_deferred(ESP_LOG_INFO, "Policy '%s' (%s, CRC %08lx): %d -> %d, %d layers, %d parameters, "
                 "self-test OK, motors cut past %.0f deg", h->name,
                 policy_source_name(active_policy.source), (unsigned long)net->crc,
                 h->input_size, h->output_size, h->num_layers, params,
                 active_policy.tilt_limit * RAD_TO_DEG);
}

static void control_task(void *arg)
{
    static ctrl_t c = {
        .state = CTRL_STATE_INIT,
        .stop_reason = "",
        .obs_restart = true,
    };

    /* ---- init: everything here runs on core 1 ---- */
    control_task_handle = xTaskGetCurrentTaskHandle();
    while (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed, retrying in 2 s (motors stay off)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* The policy itself was chosen in control_start(); time it on this core */
    if (active_policy.ok) {
        log_deferred(ESP_LOG_INFO, "Policy inference %.1f us", policy_benchmark_us());
    }

    if (imu_read_body(&c.imu) == ESP_OK) {
        accel_tilt(&c.imu, &c.roll, &c.pitch);
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
        if (imu_read_body(&c.imu) == ESP_OK) {
            c.imu_consecutive_errors = 0;
            float acc_roll, acc_pitch;
            accel_tilt(&c.imu, &acc_roll, &acc_pitch);
            /* Pitch is the rotation about body x, roll about body y */
            c.pitch = TILT_FILTER_ALPHA * (c.pitch + c.imu.gyro[0] * dt)
                      + (1.0f - TILT_FILTER_ALPHA) * acc_pitch;
            c.roll = TILT_FILTER_ALPHA * (c.roll + c.imu.gyro[1] * dt)
                     + (1.0f - TILT_FILTER_ALPHA) * acc_roll;
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

        if (cmd.requests & REQ_POLICY) {
            /* Only the web server's update sequence asks, with the robot idle */
            if (driving) {
                stop_motors(&c, CTRL_STATE_IDLE, "policy reload", 0);
            }
            select_policy();
            if (active_policy.ok) {
                log_deferred(ESP_LOG_INFO, "Policy inference %.1f us", policy_benchmark_us());
            }
            publish_telemetry(&c, 0.0f, 0.0f, cmd.policy_busy);
            /* Drop the ticks that piled up and restart the timing statistics */
            ulTaskNotifyTake(pdTRUE, 0);
            c.last_tick_us = 0;
            continue;
        }

        if (cmd.requests & REQ_STOP) {
            if (driving) {
                stop_motors(&c, CTRL_STATE_IDLE, "stop requested", 0);
            }
        } else if (cmd.requests & REQ_CALIBRATE) {
            if (cmd.policy_busy) {
                log_deferred(ESP_LOG_WARN, "Calibrate ignored: policy upload in progress");
            } else if (c.state == CTRL_STATE_IDLE) {
                run_calibration(&c);
                publish_telemetry(&c, 0.0f, 0.0f, cmd.policy_busy);
                continue;
            } else {
                log_deferred(ESP_LOG_WARN, "Calibrate ignored: stop the robot first");
            }
        } else if (cmd.requests & REQ_START) {
            if (c.state == CTRL_STATE_RUNNING) {
                /* already running */
            } else if (cmd.policy_busy) {
                log_deferred(ESP_LOG_WARN, "Start refused: policy upload in progress");
            } else if (!active_policy.ok) {
                log_deferred(ESP_LOG_ERROR, "Start refused: no usable policy, see the log");
            } else if (!upright) {
                log_deferred(ESP_LOG_WARN, "Start refused: pitch %.1f deg, hold the robot within %.0f deg of upright",
                             c.pitch * RAD_TO_DEG, START_MAX_ANGLE_DEG);
            } else if (c.imu_consecutive_errors) {
                log_deferred(ESP_LOG_WARN, "Start refused: IMU not responding");
            } else {
                motors_coast();
                c.act[0] = c.act[1] = 0.0f;
                c.obs_restart = true;
                c.state = CTRL_STATE_RUNNING;
                c.stop_reason = "";
                c.fault = 0;
                log_deferred(ESP_LOG_INFO, "Running (pitch %.1f deg)", c.pitch * RAD_TO_DEG);
            }
        } else if (cmd.requests & REQ_MANUAL) {
            if (c.state == CTRL_STATE_IDLE && !cmd.policy_busy) {
                c.state = CTRL_STATE_MANUAL;
                c.stop_reason = "";
                c.fault = 0;
                log_deferred(ESP_LOG_INFO, "Manual motor control");
            }
        }

        driving = c.state == CTRL_STATE_RUNNING || c.state == CTRL_STATE_MANUAL;
        if (driving) {
            bool check_fall = c.state == CTRL_STATE_RUNNING || MANUAL_FALL_CUTOFF;
            float tilt_limit = c.state == CTRL_STATE_RUNNING ? active_policy.tilt_limit
                                                             : FALL_ANGLE_DEG * DEG_TO_RAD;
            if (check_fall && fmaxf(fabsf(c.pitch), fabsf(c.roll)) > tilt_limit) {
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
            .gyro = {c.imu.gyro[0], c.imu.gyro[1], c.imu.gyro[2]},
            .roll = c.roll,
            .pitch = c.pitch,
            .cmd_lin = cmd_lin,
            .cmd_ang = cmd_ang,
            .prev_act = {c.act[0], c.act[1]},
            .wheel_vel = {c.wheel_vel[0], c.wheel_vel[1]},
        };
        float frame[OBS_FRAME_DIM];
        build_frame(&in, frame);
        push_frame(frame, c.obs, c.obs_restart);
        c.obs_restart = false;

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
        publish_telemetry(&c, cmd_lin, cmd_ang, cmd.policy_busy);
    }
}

void control_start(void)
{
    /* Before the WiFi takes its share of the internal RAM */
    select_policy();
    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                                            CONTROL_TASK_PRIORITY, NULL, CONTROL_CORE);
    configASSERT(ok == pdPASS);
}
