/* WiFi access point + web interface

   The robot opens its own network (WIFI_AP_SSID); browse to
   http://192.168.4.1 once connected. The page talks to the robot over a
   WebSocket at /ws.

   HTTP:
     GET    /          the web page
     POST   /policy    store the body (a policy file from `make export`) in
                       the policy partition and switch to it, if it checks out
     DELETE /policy    erase the uploaded policy, back to the built-in one

   Browser -> robot over the WebSocket, one text command per frame:
     cmd <lin m/s> <ang rad/s>   velocity command, also the heartbeat
     act <a> <b>                 drive motors directly (manual test mode)
     start | stop | calib | ping
   Robot -> browser, text frames:
     L<log text>                 serial log output
     T{...}                      telemetry JSON
     C<csv rows>                 state log: one row per control step while
                                 running, after a header line at the start
                                 of each run and for a new browser

   Everything here runs on core 0: the WiFi and lwIP tasks are pinned there
   by sdkconfig, the HTTP server by its config, and net_tx_task explicitly.
*/
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "battery.h"
#include "config.h"
#include "control.h"
#include "log_stream.h"
#include "net.h"

static const char *TAG = "net";

#define MAX_HTTP_SOCKETS    4
#define MAX_CMD_LEN         128
#define LOG_CHUNK_BYTES     2048
#define TELEMETRY_BYTES     1536
#define STATE_CHUNK_BYTES   4096
/* Room for the header and the longest possible row (13 floats with %.4f) */
#define STATE_ROW_ROOM      1024
#define MAX_IN_FLIGHT       6     /* queued broadcasts before new ones are dropped */
#define NET_TX_STACK        4096
#define NET_TX_PRIORITY     5
#define UPLOAD_CHUNK_BYTES  4096
#define UPLOAD_RECV_RETRIES 5     /* receive timeouts tolerated during an upload */

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t server;
/* Number of WebSocket clients seen by the last broadcast. Counted where the
   frames are sent, because ESP-IDF 6.x no longer calls the URI handler on the
   WebSocket handshake. */
static atomic_int ws_clients;
static atomic_int in_flight;

/* ---- WiFi --------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "Station " MACSTR " joined", MAC2STR(e->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "Station " MACSTR " left", MAC2STR(e->mac));
    }
}

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    /* Keep WiFi settings in RAM: flash writes stall both cores, including the control loop */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* No modem sleep: lower and steadier latency for commands */
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Access point '%s' up, open http://192.168.4.1", WIFI_AP_SSID);
}

/* ---- HTTP + WebSocket --------------------------------------------------- */

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    /* EMBED_TXTFILES appends a NUL terminator */
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static void handle_command(char *line)
{
    float a, b;
    if (sscanf(line, "cmd %f %f", &a, &b) == 2) {
        control_set_command(a, b);
    } else if (sscanf(line, "act %f %f", &a, &b) == 2) {
        control_set_manual(a, b);
    } else if (strcmp(line, "ping") == 0) {
        control_heartbeat();
    } else if (strcmp(line, "start") == 0) {
        control_request_start();
    } else if (strcmp(line, "stop") == 0) {
        control_request_stop();
    } else if (strcmp(line, "calib") == 0) {
        control_request_calibrate();
    } else {
        ESP_LOGW(TAG, "Unknown command '%s'", line);
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    /* Older ESP-IDF versions call the handler for the handshake as well */
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }
    if (frame.len >= MAX_CMD_LEN) {
        ESP_LOGW(TAG, "Dropping oversized frame (%u bytes)", (unsigned)frame.len);
        return ESP_FAIL;  /* closes the connection; can't skip the payload otherwise */
    }
    char buf[MAX_CMD_LEN];
    frame.payload = (uint8_t *)buf;
    err = httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
    if (err != ESP_OK) {
        return err;
    }
    if (frame.type == HTTPD_WS_TYPE_TEXT) {
        buf[frame.len] = '\0';
        handle_command(buf);
    }
    return ESP_OK;
}

static esp_err_t send_text(httpd_req_t *req, const char *status, const char *text)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, text);
}

/* Stream the request body into the policy partition, then check it there */
static const char *store_upload(httpd_req_t *req)
{
    char *chunk = malloc(UPLOAD_CHUNK_BYTES);
    if (!chunk) {
        return "out of memory";
    }
    const char *err = policy_store_begin() == ESP_OK ? NULL : "can't erase the policy partition";
    size_t left = req->content_len;
    int retries = 0;
    while (!err && left > 0) {
        int n = httpd_req_recv(req, chunk, left < UPLOAD_CHUNK_BYTES ? left : UPLOAD_CHUNK_BYTES);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++retries <= UPLOAD_RECV_RETRIES) {
            continue;
        }
        if (n <= 0) {
            err = "upload interrupted";
        } else if (policy_store_write(chunk, n) != ESP_OK) {
            err = "flash write failed";
        } else {
            left -= n;
        }
    }
    free(chunk);
    return err ? err : policy_store_check(control_check_policy);
}

/* Reply with the policy now in use */
static esp_err_t send_policy_result(httpd_req_t *req, const char *err)
{
    control_telemetry_t t;
    control_get_telemetry(&t);
    char msg[192];
    if (err) {
        snprintf(msg, sizeof(msg), "Rejected: %s. Now using the %s policy '%s'.", err,
                 t.policy_source, t.policy_name);
        return send_text(req, "400 Bad Request", msg);
    }
    snprintf(msg, sizeof(msg), "Now using the %s policy '%s' (CRC %08lx)%s.", t.policy_source,
             t.policy_name, (unsigned long)t.policy_crc, t.policy_ok ? "" : ", but it is NOT usable");
    return send_text(req, "200 OK", msg);
}

static esp_err_t policy_upload_handler(httpd_req_t *req)
{
    if (req->content_len == 0 || req->content_len > policy_store_capacity()) {
        return send_text(req, "400 Bad Request", "No policy file, or larger than the policy partition.");
    }
    const char *err = control_begin_policy_update();
    if (err) {
        return send_text(req, "409 Conflict", err);
    }
    ESP_LOGI(TAG, "Receiving a policy (%u bytes)", (unsigned)req->content_len);
    err = store_upload(req);
    if (err) {
        ESP_LOGW(TAG, "Policy upload rejected: %s", err);
        /* A half-written or bad file must not be picked up at the next boot */
        policy_store_erase();
    }
    control_reload_policy();
    control_end_policy_update();
    return send_policy_result(req, err);
}

static esp_err_t policy_delete_handler(httpd_req_t *req)
{
    const char *err = control_begin_policy_update();
    if (err) {
        return send_text(req, "409 Conflict", err);
    }
    err = policy_store_erase() == ESP_OK ? NULL : "can't erase the policy partition";
    control_reload_policy();
    control_end_policy_update();
    return send_policy_result(req, err);
}

static void on_session_close(httpd_handle_t hd, int fd)
{
    if (httpd_ws_get_fd_info(hd, fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
        ESP_LOGI(TAG, "Web client disconnected");
    }
    close(fd);
}

static void http_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = NET_CORE;
    config.max_open_sockets = MAX_HTTP_SOCKETS;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 1;
    config.close_fn = on_session_close;
    config.stack_size = 8192;   /* the policy upload check runs here */
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    static const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    static const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
    };
    static const httpd_uri_t policy_upload_uri = {
        .uri = "/policy",
        .method = HTTP_POST,
        .handler = policy_upload_handler,
    };
    static const httpd_uri_t policy_delete_uri = {
        .uri = "/policy",
        .method = HTTP_DELETE,
        .handler = policy_delete_handler,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &policy_upload_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &policy_delete_uri));
}

/* ---- broadcasting ------------------------------------------------------- */

typedef struct {
    size_t len;
    char data[];
} ws_msg_t;

/* Runs in the HTTP server task, which owns the sockets */
static void broadcast_work(void *arg)
{
    ws_msg_t *msg = arg;
    int fds[MAX_HTTP_SOCKETS];
    size_t n = MAX_HTTP_SOCKETS;
    int clients = 0;
    if (httpd_get_client_list(server, &n, fds) == ESP_OK) {
        for (size_t i = 0; i < n; i++) {
            if (httpd_ws_get_fd_info(server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
                continue;
            }
            clients++;
            httpd_ws_frame_t frame = {
                .final = true,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t *)msg->data,
                .len = msg->len,
            };
            if (httpd_ws_send_frame_async(server, fds[i], &frame) != ESP_OK) {
                httpd_sess_trigger_close(server, fds[i]);
            }
        }
    }
    if (clients > atomic_exchange(&ws_clients, clients)) {
        ESP_LOGI(TAG, "Web client connected");
    }
    free(msg);
    atomic_fetch_sub(&in_flight, 1);
}

/* Takes ownership of msg */
static void broadcast(ws_msg_t *msg)
{
    if (atomic_load(&in_flight) >= MAX_IN_FLIGHT) {
        free(msg);  /* a slow client is backing up; drop rather than pile up memory */
        return;
    }
    atomic_fetch_add(&in_flight, 1);
    if (httpd_queue_work(server, broadcast_work, msg) != ESP_OK) {
        free(msg);
        atomic_fetch_sub(&in_flight, 1);
    }
}

static ws_msg_t *build_telemetry(void)
{
    control_telemetry_t t;
    control_get_telemetry(&t);
    int batt_mv, batt_pct;
    battery_get(&batt_mv, &batt_pct);

    ws_msg_t *msg = malloc(sizeof(ws_msg_t) + TELEMETRY_BYTES);
    if (!msg) {
        return NULL;
    }
    char *p = msg->data;
    char *end = msg->data + TELEMETRY_BYTES;
    p += snprintf(p, end - p,
                  "T{\"state\":\"%s\",\"reason\":\"%s\",\"fault\":%d,"
                  "\"pitch\":%.2f,\"pitch_rate\":%.1f,"
                  "\"gyro\":[%.3f,%.3f,%.3f],\"acc\":[%.2f,%.2f,%.2f],"
                  "\"wheel\":[%.2f,%.2f],\"cmd\":[%.2f,%.2f],\"act\":[%.3f,%.3f],"
                  "\"infer_us\":%.1f,\"infer_us_max\":%.1f,\"busy_us_max\":%.0f,"
                  "\"period_us\":%.1f,\"jitter_us_max\":%.0f,\"overruns\":%lu,"
                  "\"ticks\":%lu,\"imu_errors\":%lu,\"policy\":\"%s\",\"policy_src\":\"%s\",\"policy_crc\":\"%08lx\","
                  "\"policy_ok\":%d,\"policy_busy\":%d,\"hz\":%d,"
                  "\"heap\":%lu,\"batt_mv\":%d,\"batt_pct\":%d,\"batt_low\":%d,\"obs\":[",
                  control_state_name(t.state), t.stop_reason, t.fault,
                  t.pitch_deg, t.pitch_rate_dps,
                  t.gyro[0], t.gyro[1], t.gyro[2], t.acc[0], t.acc[1], t.acc[2],
                  t.wheel_vel[0], t.wheel_vel[1], t.cmd_lin, t.cmd_ang, t.act[0], t.act[1],
                  t.infer_us, t.infer_us_max, t.busy_us_max,
                  t.period_us_avg, t.jitter_us_max, (unsigned long)t.overruns,
                  (unsigned long)t.ticks, (unsigned long)t.imu_errors, t.policy_name, t.policy_source,
                  (unsigned long)t.policy_crc, t.policy_ok, t.policy_busy, CONTROL_HZ,
                  (unsigned long)esp_get_free_heap_size(), batt_mv, batt_pct, BATTERY_LOW_PERCENT);
    for (int i = 0; i < t.obs_dim && p < end - 16; i++) {
        p += snprintf(p, end - p, i ? ",%.3f" : "%.3f", t.obs[i]);
    }
    if (p < end - 3) {
        p += snprintf(p, end - p, "]}");
    }
    msg->len = p < end ? (size_t)(p - msg->data) : TELEMETRY_BYTES - 1;
    return msg;
}

/* CSV rows of the logged control steps. NULL if there are none. */
static ws_msg_t *build_state_rows(bool *header_pending)
{
    ws_msg_t *msg = malloc(sizeof(ws_msg_t) + STATE_CHUNK_BYTES);
    if (!msg) {
        return NULL;
    }
    char *p = msg->data;
    char *end = msg->data + STATE_CHUNK_BYTES;
    *p++ = 'C';
    control_step_t s;
    while (end - p >= STATE_ROW_ROOM && control_take_step(&s)) {
        if (s.step == 0 || *header_pending) {
            p += snprintf(p, end - p, "step,time_s,%s,action_left,action_right\n",
                          control_frame_fields());
            *header_pending = false;
        }
        p += snprintf(p, end - p, "%lu,%lu.%06lu", (unsigned long)s.step,
                      (unsigned long)(s.time_us / 1000000), (unsigned long)(s.time_us % 1000000));
        for (int i = 0; i < CTRL_FRAME_DIM; i++) {
            p += snprintf(p, end - p, ",%.4f", s.frame[i]);
        }
        p += snprintf(p, end - p, ",%.4f,%.4f\n", s.act[0], s.act[1]);
    }
    msg->len = p - msg->data;
    if (msg->len == 1) {
        free(msg);
        return NULL;
    }
    return msg;
}

static void net_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int clients_seen = 0;
    bool state_header_pending = false;
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / TELEMETRY_HZ));

        /* Messages from the control task go to the UART (and back into the
           web log buffer) from here, so core 1 never waits on the UART */
        log_stream_flush_deferred();

        /* Telemetry always goes out: sending it is also what counts the
           connected browsers */
        ws_msg_t *tel_msg = build_telemetry();
        if (tel_msg) {
            broadcast(tel_msg);
        }

        /* The state log is drained even with nobody connected, so a browser
           never gets the stale rows of an earlier run. A new browser gets
           the header with its first rows. */
        int clients = atomic_load(&ws_clients);
        if (clients > clients_seen) {
            state_header_pending = true;
        }
        clients_seen = clients;
        ws_msg_t *state_msg = build_state_rows(&state_header_pending);
        if (state_msg) {
            if (clients > 0) {
                broadcast(state_msg);
            } else {
                free(state_msg);
            }
        }

        /* With nobody connected, keep the log text buffered so a browser
           that connects later still sees the boot messages */
        if (atomic_load(&ws_clients) == 0) {
            continue;
        }

        ws_msg_t *log_msg = malloc(sizeof(ws_msg_t) + 1 + LOG_CHUNK_BYTES);
        if (log_msg) {
            log_msg->data[0] = 'L';
            log_msg->len = log_stream_take(log_msg->data + 1, LOG_CHUNK_BYTES);
            if (log_msg->len > 0) {
                log_msg->len += 1;
                broadcast(log_msg);
            } else {
                free(log_msg);
            }
        }
    }
}

void net_start(void)
{
    wifi_start_ap();
    http_start();
    BaseType_t ok = xTaskCreatePinnedToCore(net_tx_task, "net_tx", NET_TX_STACK, NULL,
                                            NET_TX_PRIORITY, NULL, NET_CORE);
    configASSERT(ok == pdPASS);
}
