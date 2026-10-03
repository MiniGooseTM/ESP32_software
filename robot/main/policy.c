/* The robot's MLP policy and where it comes from, see policy.h

   policy_net.c does the parsing and the math; this file finds the policy
   file (embedded in the firmware or in the "policy" flash partition) and
   keeps the active one in internal RAM.
*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "sdkconfig.h"
#include "policy.h"

#define BENCH_ITERATIONS            200
/* Custom data subtype of the "policy" partition in partitions.csv */
#define POLICY_PARTITION_SUBTYPE    0x40
#define POLICY_PARTITION_LABEL      "policy"

extern const uint8_t policy_bin_start[] asm("_binary_policy_bin_start");
extern const uint8_t policy_bin_end[] asm("_binary_policy_bin_end");

static policy_net_t net;
static policy_source_t active_source = POLICY_SOURCE_NONE;
static void *ram_copy;                          /* the active file, if in RAM */
static esp_partition_mmap_handle_t flash_map;   /* the active file, if run from flash */
static bool flash_mapped;

static size_t store_offset;

/* Ping-pong activation buffers, in internal RAM */
static float buf_a[POLICY_MAX_WIDTH];
static float buf_b[POLICY_MAX_WIDTH];

const char *policy_source_name(policy_source_t source)
{
    switch (source) {
    case POLICY_SOURCE_BUILTIN:  return "built-in";
    case POLICY_SOURCE_UPLOADED: return "uploaded";
    default:                     return "none";
    }
}

static const esp_partition_t *store_partition(void)
{
    static const esp_partition_t *partition;
    if (!partition) {
        partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, POLICY_PARTITION_SUBTYPE,
                                             POLICY_PARTITION_LABEL);
    }
    return partition;
}

static void unload(void)
{
    active_source = POLICY_SOURCE_NONE;
    free(ram_copy);
    ram_copy = NULL;
    if (flash_mapped) {
        esp_partition_munmap(flash_map);
        flash_mapped = false;
    }
}

/* Map the partition read-only; the pointer is page aligned */
static const char *map_store(const void **data, esp_partition_mmap_handle_t *handle)
{
    const esp_partition_t *partition = store_partition();
    if (!partition) {
        return "no policy partition in the partition table";
    }
    if (esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA,
                           data, handle) != ESP_OK) {
        return "can't map the policy partition";
    }
    return NULL;
}

static const char *load_uploaded(void)
{
    const void *data;
    esp_partition_mmap_handle_t handle;
    const char *err = map_store(&data, &handle);
    if (err) {
        return err;
    }
    policy_net_t mapped;
    err = policy_net_parse(&mapped, data, store_partition()->size);
    if (err) {
        esp_partition_munmap(handle);
        return err;
    }
    size_t len = mapped.header->total_size;
    ram_copy = heap_caps_malloc(len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ram_copy) {
        memcpy(ram_copy, data, len);
        esp_partition_munmap(handle);
        return policy_net_parse(&net, ram_copy, len);
    }
    /* No RAM for a copy: run from flash, slower through the cache */
    flash_map = handle;
    flash_mapped = true;
    net = mapped;
    return NULL;
}

static const char *load_builtin(void)
{
    size_t len = policy_bin_end - policy_bin_start;
    ram_copy = heap_caps_malloc(len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ram_copy) {
        memcpy(ram_copy, policy_bin_start, len);
        return policy_net_parse(&net, ram_copy, len);
    }
    /* Works in place only if the linker happened to align it */
    return policy_net_parse(&net, policy_bin_start, len);
}

const char *policy_load(policy_source_t source)
{
    unload();
    const char *err = source == POLICY_SOURCE_UPLOADED ? load_uploaded()
                    : source == POLICY_SOURCE_BUILTIN  ? load_builtin()
                    : "no source";
    if (err) {
        unload();
        return err;
    }
    active_source = source;
    return NULL;
}

const policy_net_t *policy_active(void)
{
    return active_source != POLICY_SOURCE_NONE ? &net : NULL;
}

policy_source_t policy_active_source(void) { return active_source; }

void policy_forward(const float *obs, float *act)
{
    if (active_source == POLICY_SOURCE_NONE) {
        act[0] = act[1] = 0.0f;
        return;
    }
    policy_net_forward(&net, obs, act, buf_a, buf_b);
}

float policy_benchmark_us(void)
{
    if (active_source == POLICY_SOURCE_NONE) {
        return 0.0f;
    }
    const policy_file_header_t *h = net.header;
    float act[POLICY_MAX_WIDTH];
    uint32_t start = esp_cpu_get_cycle_count();
    for (int n = 0; n < BENCH_ITERATIONS; n++) {
        policy_net_forward(&net, net.test_inputs + (n % h->num_tests) * h->input_size, act,
                           buf_a, buf_b);
    }
    uint32_t cycles = esp_cpu_get_cycle_count() - start;
    return (float)cycles / BENCH_ITERATIONS / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
}

/* ---- the uploaded policy's flash partition ------------------------------ */

size_t policy_store_capacity(void)
{
    const esp_partition_t *partition = store_partition();
    return partition ? partition->size : 0;
}

esp_err_t policy_store_begin(void)
{
    const esp_partition_t *partition = store_partition();
    if (!partition) {
        return ESP_ERR_NOT_FOUND;
    }
    store_offset = 0;
    return esp_partition_erase_range(partition, 0, partition->size);
}

esp_err_t policy_store_write(const void *data, size_t len)
{
    const esp_partition_t *partition = store_partition();
    if (!partition || store_offset + len > partition->size) {
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = esp_partition_write(partition, store_offset, data, len);
    store_offset += len;
    return err;
}

const char *policy_store_check(const char *(*check)(const policy_net_t *net))
{
    const void *data;
    esp_partition_mmap_handle_t handle;
    const char *err = map_store(&data, &handle);
    if (err) {
        return err;
    }
    policy_net_t stored;
    err = policy_net_parse(&stored, data, store_partition()->size);
    if (!err) {
        err = check(&stored);
    }
    esp_partition_munmap(handle);
    return err;
}

bool policy_store_empty(void)
{
    /* Erased, or whatever was in that flash before the partition existed */
    const esp_partition_t *partition = store_partition();
    char magic[4];
    if (!partition || esp_partition_read(partition, 0, magic, sizeof(magic)) != ESP_OK) {
        return true;
    }
    return memcmp(magic, POLICY_FILE_MAGIC, sizeof(magic)) != 0;
}

esp_err_t policy_store_erase(void)
{
    const esp_partition_t *partition = store_partition();
    if (!partition) {
        return ESP_ERR_NOT_FOUND;
    }
    return esp_partition_erase_range(partition, 0, partition->size);
}
