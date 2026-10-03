/* The robot's MLP policy and where it comes from.

   A policy is a policy file (format in policy_net.h) written by
   `make export` in software_stack. There are two sources:
     built-in   main/policy.bin, embedded in the firmware
     uploaded   the "policy" flash partition, written through the web page

   Loading, unloading and policy_forward() are not thread-safe: the control
   module does them from one task at a time, and never while the web server
   is storing an upload (control_begin_policy_update()). */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "policy_net.h"

typedef enum {
    POLICY_SOURCE_NONE,
    POLICY_SOURCE_BUILTIN,
    POLICY_SOURCE_UPLOADED,
} policy_source_t;

const char *policy_source_name(policy_source_t source);

/* Replace the active policy with the one from source: copied to internal
   RAM (aligned, and faster than reading the weights through the flash
   cache every step), or run from flash if there is no RAM for it. Returns
   NULL on success, otherwise why it failed, and then no policy is active. */
const char *policy_load(policy_source_t source);

/* The active policy, NULL if none */
const policy_net_t *policy_active(void);
policy_source_t policy_active_source(void);

/* act = clip(MLP(obs), -1, 1), all zeros if no policy is active */
void policy_forward(const float *obs, float *act);

/* Average forward pass time of the active policy in us, 0 if none */
float policy_benchmark_us(void);

/* ---- the uploaded policy's flash partition ---- */

/* Bytes the partition holds, 0 if the partition table has none */
size_t policy_store_capacity(void);

/* Erase the partition and start writing at its beginning. Flash erases and
   writes stall both cores, so only do this with the motors off. */
esp_err_t policy_store_begin(void);
esp_err_t policy_store_write(const void *data, size_t len);

/* Check the file in the partition: parse it and pass it to check, which
   returns NULL if the policy is usable. Returns NULL or what is wrong. */
const char *policy_store_check(const char *(*check)(const policy_net_t *net));

/* True if no policy file is stored (no partition, or no file header in it) */
bool policy_store_empty(void);

/* Erase the partition, so the built-in policy is used */
esp_err_t policy_store_erase(void);
