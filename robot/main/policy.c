/* MLP policy forward pass

   Plain C on the ESP32-S3's single-precision FPU. For the network sizes a
   PPO balancing policy uses (a few thousand multiply-adds) this beats
   TFLite Micro, whose per-op dispatch and unoptimised float kernels cost
   more than the math itself. policy_selftest() measures the real time.
*/
#include <math.h>
#include <string.h>
#include "esp_cpu.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "policy.h"
#include "policy_weights.h"

static const char *TAG = "policy";

#define BENCH_ITERATIONS 1000

/* Ping-pong activation buffers, in internal RAM */
static float buf_a[POLICY_MAX_WIDTH];
static float buf_b[POLICY_MAX_WIDTH];

int policy_obs_dim(void) { return POLICY_OBS_DIM; }
int policy_act_dim(void) { return POLICY_ACT_DIM; }
int policy_is_placeholder(void) { return POLICY_IS_PLACEHOLDER; }

/* y = act(W x + b), W row-major [n_out][n_in] */
static void dense(const float *restrict W, const float *restrict b,
                  const float *restrict x, float *restrict y,
                  int n_in, int n_out, int activation)
{
    for (int o = 0; o < n_out; o++) {
        const float *row = W + o * n_in;
        /* Four accumulators break the add dependency chain so the FPU pipelines */
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        int i = 0;
        for (; i + 4 <= n_in; i += 4) {
            s0 += row[i] * x[i];
            s1 += row[i + 1] * x[i + 1];
            s2 += row[i + 2] * x[i + 2];
            s3 += row[i + 3] * x[i + 3];
        }
        for (; i < n_in; i++) {
            s0 += row[i] * x[i];
        }
        float v = b[o] + (s0 + s1) + (s2 + s3);
        switch (activation) {
        case POLICY_ACTIVATION_TANH:
            v = tanhf(v);
            break;
        case POLICY_ACTIVATION_RELU:
            v = v > 0.0f ? v : 0.0f;
            break;
        case POLICY_ACTIVATION_ELU:
            v = v > 0.0f ? v : expm1f(v);
            break;
        default:
            break;
        }
        y[o] = v;
    }
}

void policy_forward(const float *obs, float *act)
{
    float *in = buf_a;
    float *out = buf_b;

    for (int i = 0; i < POLICY_OBS_DIM; i++) {
        float v = obs[i];
#if POLICY_HAS_OBS_NORM
        v = (v - policy_obs_mean[i]) * policy_obs_inv_std[i];
#endif
        if (POLICY_OBS_CLIP > 0.0f) {
            v = fminf(fmaxf(v, -POLICY_OBS_CLIP), POLICY_OBS_CLIP);
        }
        in[i] = v;
    }

    for (int l = 0; l < POLICY_NUM_LAYERS; l++) {
        dense(policy_w[l], policy_b[l], in, out,
              policy_layer_in[l], policy_layer_out[l], policy_layer_act[l]);
        float *t = in;
        in = out;
        out = t;
    }

    for (int i = 0; i < POLICY_ACT_DIM; i++) {
        float v = in[i];
        /* NaN fails both comparisons and ends up 0 */
        act[i] = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : (v == v ? v : 0.0f));
    }
}

float policy_selftest(void)
{
    float act[POLICY_ACT_DIM];
    float max_err = 0.0f;
    for (int t = 0; t < POLICY_NUM_TESTS; t++) {
        policy_forward(policy_test_obs[t], act);
        for (int i = 0; i < POLICY_ACT_DIM; i++) {
            float err = fabsf(act[i] - policy_test_act[t][i]);
            if (!(err <= max_err)) {
                max_err = err;
            }
        }
    }

    uint32_t start = esp_cpu_get_cycle_count();
    for (int n = 0; n < BENCH_ITERATIONS; n++) {
        policy_forward(policy_test_obs[n % POLICY_NUM_TESTS], act);
    }
    uint32_t cycles = esp_cpu_get_cycle_count() - start;
    float us = (float)cycles / BENCH_ITERATIONS / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

    ESP_LOGI(TAG, "%s policy: %d obs -> %d actions, %d layers, %d parameters",
             POLICY_IS_PLACEHOLDER ? "PLACEHOLDER" : "Trained",
             POLICY_OBS_DIM, POLICY_ACT_DIM, POLICY_NUM_LAYERS, POLICY_NUM_PARAMS);
    ESP_LOGI(TAG, "Self-test max error %.2e (%s), inference %.1f us (avg of %d)",
             max_err, max_err < 1e-4f ? "OK" : "MISMATCH - check export", us, BENCH_ITERATIONS);
    return max_err;
}
