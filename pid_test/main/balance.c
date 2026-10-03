/* PID balance controller

   Runs alone on core 1 at CONTROL_HZ, woken by a hardware timer (same
   scheme as robot/main/control.c):

     tilt   = complementary filter of gyro rate and accelerometer angle
     sp     = trim - kv * wheel_speed          (optional outer speed loop)
     e      = tilt - sp
     u      = kp*e + ki*integral(e) + kd*tilt_rate
     motors = u on both wheels

   The D term uses the gyro rate directly instead of differentiating the
   error, so it is clean and doesn't kick when the setpoint changes.
   Leaning forward gives a positive error and drives the wheels forward.
*/
#include <math.h>
#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "config.h"
#include "imu.h"
#include "log_stream.h"
#include "motors.h"
#include "pid_config.h"
#include "balance.h"

static const char *TAG = "balance";

#define PERIOD_US       (1000000 / CONTROL_HZ)
#define RAD_TO_DEG      57.29577951308232f
#define TWO_PI          6.283185307179586f

#define NVS_NAMESPACE   "pid"
#define NVS_KEY_PARAMS  "params"

#define REQ_START       (1u << 0)
#define REQ_STOP        (1u << 1)
#define REQ_CALIBRATE   (1u << 2)
#define REQ_SET_TRIM    (1u << 3)
#define REQ_SAVE        (1u << 4)

static TaskHandle_t task_handle;

/* Requests and gains, written by the network side */
static portMUX_TYPE cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t pending_requests;
static int64_t last_heartbeat_us;
static pid_params_t shared_params = {
    .kp = PID_DEFAULT_KP,
    .ki = PID_DEFAULT_KI,
    .kd = PID_DEFAULT_KD,
    .kv = PID_DEFAULT_KV,
    .trim_deg = PID_DEFAULT_TRIM_DEG,
    .out_limit = PID_DEFAULT_OUT_LIMIT,
    .i_limit = PID_DEFAULT_I_LIMIT,
    .friction = PID_DEFAULT_FRICTION,
    .sp_limit = PID_DEFAULT_SP_LIMIT,
};

/* Status snapshot, written by the control task */
static portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
static bal_status_t shared_status = {
    .state = BAL_STATE_INIT,
    .stop_reason = "",
};

/* Single-producer single-consumer sample ring: the control task writes,
   the network task reads. Indices only ever increase. */
static bal_sample_t ring[SAMPLE_RING_SIZE];
static atomic_uint ring_head;
static atomic_uint ring_tail;
static atomic_uint ring_dropped;

/* ---- interface for other tasks ------------------------------------------ */

static void post_request(uint32_t req)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&cmd_lock);
    pending_requests |= req;
    last_heartbeat_us = now;
    portEXIT_CRITICAL(&cmd_lock);
}

void balance_heartbeat(void) { post_request(0); }
void balance_request_start(void) { post_request(REQ_START); }
void balance_request_stop(void) { post_request(REQ_STOP); }
void balance_request_calibrate(void) { post_request(REQ_CALIBRATE); }
void balance_request_set_trim(void) { post_request(REQ_SET_TRIM); }
void balance_request_save(void) { post_request(REQ_SAVE); }

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

bool balance_set_param(const char *name, float value)
{
    if (!isfinite(value)) {
        return false;
    }
    bool ok = true;
    portENTER_CRITICAL(&cmd_lock);
    pid_params_t *p = &shared_params;
    if (strcmp(name, "kp") == 0) {
        p->kp = value;
    } else if (strcmp(name, "ki") == 0) {
        p->ki = value;
    } else if (strcmp(name, "kd") == 0) {
        p->kd = value;
    } else if (strcmp(name, "kv") == 0) {
        p->kv = value;
    } else if (strcmp(name, "trim") == 0) {
        p->trim_deg = clampf(value, -30.0f, 30.0f);
    } else if (strcmp(name, "out_limit") == 0) {
        p->out_limit = clampf(value, 0.0f, 1.0f);
    } else if (strcmp(name, "i_limit") == 0) {
        p->i_limit = clampf(value, 0.0f, 1.0f);
    } else if (strcmp(name, "friction") == 0) {
        p->friction = clampf(value, 0.0f, 0.9f);
    } else if (strcmp(name, "sp_limit") == 0) {
        p->sp_limit = clampf(value, 0.0f, 30.0f);
    } else {
        ok = false;
    }
    portEXIT_CRITICAL(&cmd_lock);
    return ok;
}

void balance_get_status(bal_status_t *out)
{
    portENTER_CRITICAL(&status_lock);
    memcpy(out, &shared_status, sizeof(*out));
    portEXIT_CRITICAL(&status_lock);
}

size_t balance_take_samples(bal_sample_t *out, size_t max)
{
    unsigned tail = atomic_load_explicit(&ring_tail, memory_order_relaxed);
    unsigned head = atomic_load_explicit(&ring_head, memory_order_acquire);
    size_t n = head - tail;
    if (n > max) {
        n = max;
    }
    for (size_t i = 0; i < n; i++) {
        out[i] = ring[(tail + i) % SAMPLE_RING_SIZE];
    }
    atomic_store_explicit(&ring_tail, tail + n, memory_order_release);
    return n;
}

const char *balance_state_name(bal_state_t state)
{
    switch (state) {
    case BAL_STATE_INIT:        return "init";
    case BAL_STATE_IDLE:        return "idle";
    case BAL_STATE_RUNNING:     return "running";
    case BAL_STATE_CALIBRATING: return "calibrating";
    }
    return "?";
}

/* ---- control task --------------------------------------------------------- */

static void push_sample(const bal_sample_t *s)
{
    unsigned head = atomic_load_explicit(&ring_head, memory_order_relaxed);
    unsigned tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
    if (head - tail >= SAMPLE_RING_SIZE) {
        atomic_fetch_add(&ring_dropped, 1);  /* nobody is reading; drop */
        return;
    }
    ring[head % SAMPLE_RING_SIZE] = *s;
    atomic_store_explicit(&ring_head, head + 1, memory_order_release);
}

static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle, &woken);
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
    /* The ISR is installed on the calling core, i.e. core 1 */
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

/* Tilt from gravity alone, deg; valid when the robot isn't accelerating */
static float accel_tilt_deg(const imu_sample_t *s)
{
    return atan2f(-s->acc[IMU_FWD_ACC_AXIS], s->acc[IMU_UP_ACC_AXIS]) * RAD_TO_DEG;
}

static void load_params(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "No saved gains, using defaults from pid_config.h");
        return;
    }
    pid_params_t p;
    size_t len = sizeof(p);
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_PARAMS, &p, &len);
    nvs_close(nvs);
    if (err != ESP_OK || len != sizeof(p)) {
        ESP_LOGI(TAG, "No saved gains, using defaults from pid_config.h");
        return;
    }
    portENTER_CRITICAL(&cmd_lock);
    shared_params = p;
    portEXIT_CRITICAL(&cmd_lock);
    ESP_LOGI(TAG, "Loaded gains: kp %.4f ki %.4f kd %.5f kv %.3f trim %.2f",
             p.kp, p.ki, p.kd, p.kv, p.trim_deg);
}

static void save_params(const pid_params_t *p)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, NVS_KEY_PARAMS, p, sizeof(*p));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err == ESP_OK) {
        log_deferred(ESP_LOG_INFO, "Gains saved: kp %.4f ki %.4f kd %.5f kv %.3f trim %.2f",
                     p->kp, p->ki, p->kd, p->kv, p->trim_deg);
    } else {
        log_deferred(ESP_LOG_ERROR, "Saving gains failed: %s", esp_err_to_name(err));
    }
}

typedef struct {
    bal_state_t state;
    const char *stop_reason;
    int fault;

    imu_sample_t imu;
    int imu_consecutive_errors;
    uint32_t imu_errors;
    float tilt;             /* deg */
    float rate;             /* deg/s */

    int32_t last_counts[NUM_MOTORS];
    float wheel_vel[NUM_MOTORS];

    float integral;         /* integral term, motor command units */
    float u;

    int64_t last_tick_us;
    uint32_t ticks;
    uint32_t overruns;
    float win_busy_max, win_jitter_max;
    float pub_busy_max, pub_jitter_max;
} bal_t;

static void stop_motors(bal_t *b, const char *reason, int fault)
{
    motors_coast();
    b->u = 0.0f;
    b->integral = 0.0f;
    if (b->state == BAL_STATE_RUNNING) {
        b->stop_reason = reason;
        b->fault = fault;
        log_deferred(fault ? ESP_LOG_WARN : ESP_LOG_INFO, "Stopped: %s", reason);
    }
    b->state = BAL_STATE_IDLE;
}

static void publish_status(const bal_t *b, const pid_params_t *p)
{
    bal_status_t s = {
        .state = b->state,
        .stop_reason = b->stop_reason,
        .fault = b->fault,
        .tilt_deg = b->tilt,
        .rate_dps = b->rate,
        .wheel_vel = {b->wheel_vel[0], b->wheel_vel[1]},
        .u = b->u,
        .busy_us_max = b->pub_busy_max,
        .jitter_us_max = b->pub_jitter_max,
        .overruns = b->overruns,
        .imu_errors = b->imu_errors,
        .samples_dropped = atomic_load(&ring_dropped),
        .params = *p,
    };
    portENTER_CRITICAL(&status_lock);
    memcpy(&shared_status, &s, sizeof(s));
    portEXIT_CRITICAL(&status_lock);
}

static void run_calibration(bal_t *b, const pid_params_t *p)
{
    b->state = BAL_STATE_CALIBRATING;
    publish_status(b, p);
    log_deferred(ESP_LOG_INFO, "Calibrating in %d ms - hold the robot still and upright", IMU_CALIB_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(IMU_CALIB_SETTLE_MS));
    esp_err_t err = imu_calibrate();
    log_deferred(err == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR, "Calibration %s",
                 err == ESP_OK ? "done" : esp_err_to_name(err));
    if (imu_read(&b->imu) == ESP_OK) {
        b->tilt = accel_tilt_deg(&b->imu);
    }
    b->state = BAL_STATE_IDLE;
    ulTaskNotifyTake(pdTRUE, 0);
    b->last_tick_us = 0;
}

static void balance_task(void *arg)
{
    static bal_t b = {
        .state = BAL_STATE_INIT,
        .stop_reason = "",
    };
    task_handle = xTaskGetCurrentTaskHandle();

    while (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed, retrying in 2 s (motors stay off)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    load_params();
    if (imu_read(&b.imu) == ESP_OK) {
        b.tilt = accel_tilt_deg(&b.imu);
    }
    encoders_read(b.last_counts);

    portENTER_CRITICAL(&cmd_lock);
    pending_requests = 0;
    portEXIT_CRITICAL(&cmd_lock);

    start_loop_timer();
    b.state = BAL_STATE_IDLE;
    ESP_LOGI(TAG, "PID loop running at %d Hz on core %d", CONTROL_HZ, xPortGetCoreID());

    while (true) {
        uint32_t pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (pending == 0) {
            if (b.state == BAL_STATE_RUNNING) {
                stop_motors(&b, "loop timer stalled", 1);
            }
            continue;
        }
        int64_t now = esp_timer_get_time();
        b.overruns += pending - 1;
        b.ticks++;

        float dt = 1.0f / CONTROL_HZ;
        if (b.last_tick_us) {
            float period = (float)(now - b.last_tick_us);
            float jitter = fabsf(period - PERIOD_US);
            if (jitter > b.win_jitter_max) {
                b.win_jitter_max = jitter;
            }
            dt = fminf(fmaxf(period, 0.5f * PERIOD_US), 2.0f * PERIOD_US) * 1e-6f;
        }
        b.last_tick_us = now;

        /* ---- take requests and the current gains ---- */
        uint32_t req;
        int64_t heartbeat_us;
        pid_params_t p;
        portENTER_CRITICAL(&cmd_lock);
        req = pending_requests;
        pending_requests = 0;
        heartbeat_us = last_heartbeat_us;
        p = shared_params;
        portEXIT_CRITICAL(&cmd_lock);

        /* ---- sense ---- */
        if (imu_read(&b.imu) == ESP_OK) {
            b.imu_consecutive_errors = 0;
            b.rate = IMU_TILT_GYRO_SIGN * b.imu.gyro[IMU_TILT_GYRO_AXIS] * RAD_TO_DEG;
            b.tilt = TILT_FILTER_ALPHA * (b.tilt + b.rate * dt)
                     + (1.0f - TILT_FILTER_ALPHA) * accel_tilt_deg(&b.imu);
        } else {
            b.imu_errors++;
            b.imu_consecutive_errors++;
        }

        int32_t counts[NUM_MOTORS];
        encoders_read(counts);
        for (int i = 0; i < NUM_MOTORS; i++) {
            b.wheel_vel[i] = (counts[i] - b.last_counts[i]) * (TWO_PI / ENCODER_COUNTS_PER_REV) / dt;
            b.last_counts[i] = counts[i];
        }
        float wheel_avg = 0.5f * (b.wheel_vel[0] + b.wheel_vel[1]);

        /* ---- requests ---- */
        if (req & REQ_SET_TRIM) {
            float trim = clampf(b.tilt, -30.0f, 30.0f);
            portENTER_CRITICAL(&cmd_lock);
            shared_params.trim_deg = trim;
            portEXIT_CRITICAL(&cmd_lock);
            p.trim_deg = trim;
            log_deferred(ESP_LOG_INFO, "Trim set to %.2f deg", trim);
        }
        if (req & REQ_STOP) {
            if (b.state == BAL_STATE_RUNNING) {
                stop_motors(&b, "stop requested", 0);
            }
        } else if (req & REQ_CALIBRATE) {
            if (b.state == BAL_STATE_IDLE) {
                run_calibration(&b, &p);
                publish_status(&b, &p);
                continue;
            }
            log_deferred(ESP_LOG_WARN, "Calibrate ignored: stop the robot first");
        } else if (req & REQ_SAVE) {
            if (b.state == BAL_STATE_IDLE) {
                /* Flash writes stall both cores for a few ms; fine while idle */
                save_params(&p);
                ulTaskNotifyTake(pdTRUE, 0);
                b.last_tick_us = 0;
            } else {
                log_deferred(ESP_LOG_WARN, "Save ignored: stop the robot first");
            }
        } else if (req & REQ_START) {
            if (b.state == BAL_STATE_RUNNING) {
                /* already running */
            } else if (fabsf(b.tilt - p.trim_deg) > START_MAX_ANGLE_DEG) {
                log_deferred(ESP_LOG_WARN, "Start refused: tilt %.1f deg, hold the robot near upright", b.tilt);
            } else if (b.imu_consecutive_errors) {
                log_deferred(ESP_LOG_WARN, "Start refused: IMU not responding");
            } else {
                b.state = BAL_STATE_RUNNING;
                b.integral = 0.0f;
                b.stop_reason = "";
                b.fault = 0;
                log_deferred(ESP_LOG_INFO, "Balancing (tilt %.1f deg)", b.tilt);
            }
        }

        /* ---- safety ---- */
        if (b.state == BAL_STATE_RUNNING) {
            int64_t link_age_ms = (now - heartbeat_us) / 1000;
            if (fabsf(b.tilt) > FALL_ANGLE_DEG) {
                stop_motors(&b, "fell over", 1);
            } else if (b.imu_consecutive_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
                stop_motors(&b, "IMU read errors", 1);
            } else if (LINK_LOST_STOP_MS > 0 && link_age_ms > LINK_LOST_STOP_MS) {
                stop_motors(&b, "link lost", 1);
            }
        }

        /* ---- PID ---- */
        bal_sample_t s = {
            .t_ms = (uint32_t)(now / 1000),
            .tilt = b.tilt,
            .rate = b.rate,
            .sp = p.trim_deg,
        };
        if (b.state == BAL_STATE_RUNNING) {
            float shift = clampf(-p.kv * wheel_avg, -p.sp_limit, p.sp_limit);
            s.sp = p.trim_deg + shift;
            float e = b.tilt - s.sp;

            s.p = p.kp * e;
            s.d = p.kd * b.rate;
            float u_unsat = s.p + b.integral + s.d;
            /* Anti-windup: stop integrating while the output is saturated
               in the direction the integral would push it further */
            bool saturated = fabsf(u_unsat) >= p.out_limit && (u_unsat > 0.0f) == (e > 0.0f);
            if (!saturated) {
                b.integral = clampf(b.integral + p.ki * e * dt, -p.i_limit, p.i_limit);
            }
            s.i = b.integral;

            float u = clampf(s.p + s.i + s.d, -p.out_limit, p.out_limit);
            if (p.friction > 0.0f && fabsf(u) > 1e-3f) {
                u = copysignf(p.friction + (1.0f - p.friction) * fabsf(u), u);
            }
            b.u = u;
            /* Same command on both wheels; motors.c applies the mirroring */
            motors_set(u, u);
        }
        s.u = b.u;
        push_sample(&s);

        /* ---- bookkeeping ---- */
        float busy = (float)(esp_timer_get_time() - now);
        if (busy > b.win_busy_max) {
            b.win_busy_max = busy;
        }
        if (b.ticks % CONTROL_HZ == 0) {
            b.pub_busy_max = b.win_busy_max;
            b.pub_jitter_max = b.win_jitter_max;
            b.win_busy_max = b.win_jitter_max = 0.0f;
        }
        publish_status(&b, &p);
    }
}

void balance_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(balance_task, "balance", CONTROL_TASK_STACK, NULL,
                                            CONTROL_TASK_PRIORITY, NULL, CONTROL_CORE);
    configASSERT(ok == pdPASS);
}
