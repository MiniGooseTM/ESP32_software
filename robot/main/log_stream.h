#pragma once

#include <stddef.h>
#include "esp_log.h"

void log_stream_init(void);

/* Queue a log message without touching the UART. Formats into a ring buffer
   (dropped if full) and returns; log_stream_flush_deferred() prints it later
   from core 0. Use this from the control task instead of ESP_LOGx: a UART
   print at 115200 baud blocks for ~1 ms per 10 characters. */
void log_deferred(esp_log_level_t level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* Print queued deferred messages through ESP_LOG. Call from a core-0 task. */
void log_stream_flush_deferred(void);

/* Move buffered log text into buf (not NUL-terminated). Returns the number
   of bytes written; 0 if nothing is pending. */
size_t log_stream_take(char *buf, size_t max);
