/* Policy file format and MLP forward pass, in portable C (no ESP-IDF), so
   test/policy_host_test.c can check a policy file on the PC.

   A policy file is written by `make export` in software_stack
   (src/self_balancing_robot/export.py). Little-endian, every field 4-byte
   aligned:

     policy_file_header_t                         96 bytes
     policy_file_layer_t[num_layers]              8 bytes each
     per layer: W[out][in] row-major, then b[out] float32, y = act(W x + b)
     test_inputs[num_tests][input_size]           float32
     test_outputs[num_tests][output_size]         float32, SB3's actions, clipped to [-1, 1]
     CRC-32 (as zlib.crc32) of everything before  uint32
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

#define POLICY_FILE_MAGIC       "GPOL"
#define POLICY_FILE_VERSION     1
#define POLICY_MAX_LAYERS       8
#define POLICY_MAX_WIDTH        256

#define POLICY_ACTIVATION_NONE  0
#define POLICY_ACTIVATION_TANH  1
#define POLICY_ACTIVATION_RELU  2
#define POLICY_ACTIVATION_ELU   3

typedef struct {
    char magic[4];
    uint16_t version;
    uint16_t header_size;
    uint32_t total_size;            /* whole file, CRC included */
    uint16_t num_layers;
    uint16_t input_size;
    uint16_t output_size;
    uint16_t frame_size;            /* values per observation frame */
    uint16_t history_length;        /* frames in the input, newest first */
    uint16_t num_tests;
    uint32_t frame_fields_crc;      /* CRC-32 of the frame field names joined by "," */
    float control_period_s;
    float max_linear_command;       /* m/s */
    float max_angular_command;      /* rad/s */
    float half_track_m;
    float wheel_no_load_speed;      /* rad/s */
    float tilt_limit_rad;           /* training ended episodes past this roll or pitch */
    char name[44];                  /* model it was exported from, NUL terminated */
} policy_file_header_t;

typedef struct {
    uint16_t in;
    uint16_t out;
    uint16_t activation;            /* POLICY_ACTIVATION_* */
    uint16_t reserved;
} policy_file_layer_t;

_Static_assert(sizeof(policy_file_header_t) == 96, "policy file header layout");
_Static_assert(sizeof(policy_file_layer_t) == 8, "policy file layer layout");

/* A checked policy file; the pointers point into the file's memory */
typedef struct {
    const policy_file_header_t *header;
    const policy_file_layer_t *layers;
    const float *weights[POLICY_MAX_LAYERS];
    const float *biases[POLICY_MAX_LAYERS];
    const float *test_inputs;
    const float *test_outputs;
    uint32_t crc;                   /* the file's CRC, identifies the policy */
} policy_net_t;

/* CRC-32 as zlib.crc32() computes it */
uint32_t policy_crc32(const void *data, size_t len);

/* Check a policy file of len bytes (len may exceed the file, e.g. a flash
   partition) and point net into it. data must be 4-byte aligned and stay
   valid while net is in use. Returns NULL if the file is good, otherwise
   what is wrong with it. */
const char *policy_net_parse(policy_net_t *net, const void *data, size_t len);

/* act = clip(MLP(obs), -1, 1); NaN outputs become 0. buf_a and buf_b hold
   POLICY_MAX_WIDTH floats each. */
void policy_net_forward(const policy_net_t *net, const float *obs, float *act,
                        float *buf_a, float *buf_b);

/* Largest |output - expected| over the test vectors in the file */
float policy_net_test_error(const policy_net_t *net, float *buf_a, float *buf_b);
