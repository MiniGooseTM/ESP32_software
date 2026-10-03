/* Log tee to the web interface

   esp_log_set_vprintf() routes every ESP_LOGx line through log_vprintf(),
   which copies it into a ring buffer and then hands it to the original
   vprintf so the serial console keeps working. The network task drains the
   ring buffer and sends the text to the browser. Nothing here ever waits:
   if the buffer is full the line is dropped from the web view.
*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "config.h"
#include "log_stream.h"

#define LINE_MAX_LEN        256
#define DEFERRED_BUF_BYTES  2048

static RingbufHandle_t web_rb;       /* log text for the browser */
static RingbufHandle_t deferred_rb;  /* messages from the control task */
static vprintf_like_t original_vprintf;

/* One shared format buffer instead of a stack buffer, so tasks with small
   stacks (WiFi, timers) can log safely. If it's busy the line still goes to
   the UART, it's only missing from the web view. */
static char line_buf[LINE_MAX_LEN];
static SemaphoreHandle_t line_lock;

static int log_vprintf(const char *fmt, va_list args)
{
    if (!xPortInIsrContext() && xSemaphoreTake(line_lock, 0) == pdTRUE) {
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(line_buf, sizeof(line_buf), fmt, copy);
        va_end(copy);
        if (n > 0) {
            size_t len = (size_t)n < sizeof(line_buf) ? (size_t)n : sizeof(line_buf) - 1;
            xRingbufferSend(web_rb, line_buf, len, 0);
        }
        xSemaphoreGive(line_lock);
    }
    return original_vprintf(fmt, args);
}

void log_stream_init(void)
{
    web_rb = xRingbufferCreate(LOG_STREAM_BUFFER_BYTES, RINGBUF_TYPE_NOSPLIT);
    deferred_rb = xRingbufferCreate(DEFERRED_BUF_BYTES, RINGBUF_TYPE_NOSPLIT);
    line_lock = xSemaphoreCreateMutex();
    configASSERT(web_rb && deferred_rb && line_lock);
    original_vprintf = esp_log_set_vprintf(log_vprintf);
}

void log_deferred(esp_log_level_t level, const char *fmt, ...)
{
    /* First byte carries the level */
    char msg[160];
    msg[0] = (char)level;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(msg + 1, sizeof(msg) - 1, fmt, args);
    va_end(args);
    if (n < 0) {
        return;
    }
    size_t len = 1 + ((size_t)n < sizeof(msg) - 1 ? (size_t)n : sizeof(msg) - 2);
    xRingbufferSend(deferred_rb, msg, len, 0);
}

void log_stream_flush_deferred(void)
{
    static const char *TAG = "control";
    size_t size;
    char *item;
    while ((item = xRingbufferReceive(deferred_rb, &size, 0)) != NULL) {
        esp_log_level_t level = (esp_log_level_t)item[0];
        int len = (int)size - 1;
        if (level == ESP_LOG_ERROR) {
            ESP_LOGE(TAG, "%.*s", len, item + 1);
        } else if (level == ESP_LOG_WARN) {
            ESP_LOGW(TAG, "%.*s", len, item + 1);
        } else {
            ESP_LOGI(TAG, "%.*s", len, item + 1);
        }
        vRingbufferReturnItem(deferred_rb, item);
    }
}

size_t log_stream_take(char *buf, size_t max)
{
    size_t used = 0;
    /* Lines are at most LINE_MAX_LEN, so stop once one might not fit */
    while (max - used >= LINE_MAX_LEN) {
        size_t size;
        char *item = xRingbufferReceive(web_rb, &size, 0);
        if (!item) {
            break;
        }
        memcpy(buf + used, item, size);
        used += size;
        vRingbufferReturnItem(web_rb, item);
    }
    return used;
}
