/* WiFi access point + PID tuning page

   Same setup as robot/main/net.c: the robot opens WIFI_AP_SSID and serves
   the page at http://192.168.4.1, which talks over a WebSocket at /ws.

   Browser -> robot, one text command per frame:
     set <name> <value>          change a gain (see balance_set_param)
     start | stop | calib | trim | save | ping
   Robot -> browser, text frames:
     S[[t_ms,tilt,sp,rate,p,i,d,u],...]   every control step since the last frame
     T{...}                                status and current gains
     L<log text>                           serial log output
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
#include "config.h"
#include "balance.h"
#include "log_stream.h"
#include "net.h"

static const char *TAG = "net";

#define MAX_HTTP_SOCKETS    4
#define MAX_CMD_LEN         96
#define LOG_CHUNK_BYTES     2048
#define STATUS_BYTES        1024
#define MAX_SAMPLES_PER_MSG 64
#define SAMPLE_JSON_BYTES   96      /* worst case per sample */
#define MAX_IN_FLIGHT       8
#define SEND_HZ             20
#define STATUS_EVERY_N      4       /* status at SEND_HZ / 4 = 5 Hz */
#define NET_TX_STACK        4096
#define NET_TX_PRIORITY     5

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t server;
/* WebSocket clients seen by the last broadcast (ESP-IDF 6.x doesn't call
   the URI handler on the handshake, so they're counted while sending) */
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
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Access point '%s' up, open http://192.168.4.1", WIFI_AP_SSID);
}

/* ---- HTTP + WebSocket --------------------------------------------------- */

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static void handle_command(char *line)
{
    char name[24];
    float value;
    if (sscanf(line, "set %23s %f", name, &value) == 2) {
        if (balance_set_param(name, value)) {
            balance_heartbeat();
        } else {
            ESP_LOGW(TAG, "Unknown or invalid parameter '%s'", name);
        }
    } else if (strcmp(line, "ping") == 0) {
        balance_heartbeat();
    } else if (strcmp(line, "start") == 0) {
        balance_request_start();
    } else if (strcmp(line, "stop") == 0) {
        balance_request_stop();
    } else if (strcmp(line, "calib") == 0) {
        balance_request_calibrate();
    } else if (strcmp(line, "trim") == 0) {
        balance_request_set_trim();
    } else if (strcmp(line, "save") == 0) {
        balance_request_save();
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
        return ESP_FAIL;
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
    config.stack_size = 6144;
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
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));
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
        free(msg);
        return;
    }
    atomic_fetch_add(&in_flight, 1);
    if (httpd_queue_work(server, broadcast_work, msg) != ESP_OK) {
        free(msg);
        atomic_fetch_sub(&in_flight, 1);
    }
}

static ws_msg_t *build_status(void)
{
    bal_status_t s;
    balance_get_status(&s);
    ws_msg_t *msg = malloc(sizeof(ws_msg_t) + STATUS_BYTES);
    if (!msg) {
        return NULL;
    }
    const pid_params_t *p = &s.params;
    int n = snprintf(msg->data, STATUS_BYTES,
                     "T{\"state\":\"%s\",\"reason\":\"%s\",\"fault\":%d,"
                     "\"tilt\":%.2f,\"rate\":%.1f,\"wheel\":[%.2f,%.2f],\"u\":%.3f,"
                     "\"busy_us_max\":%.0f,\"jitter_us_max\":%.0f,\"overruns\":%lu,"
                     "\"imu_errors\":%lu,\"dropped\":%lu,\"hz\":%d,"
                     "\"params\":{\"kp\":%.5g,\"ki\":%.5g,\"kd\":%.5g,\"kv\":%.5g,"
                     "\"trim\":%.3f,\"out_limit\":%.3f,\"i_limit\":%.3f,"
                     "\"friction\":%.3f,\"sp_limit\":%.2f}}",
                     balance_state_name(s.state), s.stop_reason, s.fault,
                     s.tilt_deg, s.rate_dps, s.wheel_vel[0], s.wheel_vel[1], s.u,
                     s.busy_us_max, s.jitter_us_max, (unsigned long)s.overruns,
                     (unsigned long)s.imu_errors, (unsigned long)s.samples_dropped, CONTROL_HZ,
                     p->kp, p->ki, p->kd, p->kv, p->trim_deg, p->out_limit, p->i_limit,
                     p->friction, p->sp_limit);
    if (n <= 0 || n >= STATUS_BYTES) {
        free(msg);
        return NULL;
    }
    msg->len = n;
    return msg;
}

static ws_msg_t *build_samples(const bal_sample_t *s, size_t count)
{
    size_t cap = 4 + count * SAMPLE_JSON_BYTES;
    ws_msg_t *msg = malloc(sizeof(ws_msg_t) + cap);
    if (!msg) {
        return NULL;
    }
    char *p = msg->data;
    char *end = msg->data + cap;
    p += snprintf(p, end - p, "S[");
    for (size_t i = 0; i < count; i++) {
        int n = snprintf(p, end - p, "%s[%lu,%.3f,%.3f,%.2f,%.4f,%.4f,%.4f,%.4f]",
                         i ? "," : "", (unsigned long)s[i].t_ms, s[i].tilt, s[i].sp, s[i].rate,
                         s[i].p, s[i].i, s[i].d, s[i].u);
        if (n <= 0 || n >= end - p - 1) {
            break;
        }
        p += n;
    }
    *p++ = ']';
    msg->len = p - msg->data;
    return msg;
}

static void net_tx_task(void *arg)
{
    static bal_sample_t samples[MAX_SAMPLES_PER_MSG];
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / SEND_HZ));
        cycle++;
        log_stream_flush_deferred();

        /* Status always goes out: sending it is also what counts the browsers */
        if (cycle % STATUS_EVERY_N == 0 || atomic_load(&ws_clients) == 0) {
            ws_msg_t *msg = build_status();
            if (msg) {
                broadcast(msg);
            }
        }

        /* Drain the samples even with nobody watching, so the ring doesn't
           fill up and count drops */
        size_t n = balance_take_samples(samples, MAX_SAMPLES_PER_MSG);
        if (atomic_load(&ws_clients) == 0) {
            continue;
        }
        if (n > 0) {
            ws_msg_t *msg = build_samples(samples, n);
            if (msg) {
                broadcast(msg);
            }
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
