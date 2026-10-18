/* Policy file parsing and MLP forward pass, see policy_net.h*/
#include <math.h>
#include <string.h>
#include "policy_net.h"

uint32_t policy_crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static int all_finite(const float *v, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (!isfinite(v[i])) {
            return 0;
        }
    }
    return 1;
}

const char *policy_net_parse(policy_net_t *net, const void *data, size_t len)
{
    const uint8_t *base = data;
    const policy_file_header_t *h = data;

    if ((uintptr_t)data % 4 != 0) {
        return "file not 4-byte aligned in memory";
    }
    if (len < sizeof(*h) + sizeof(uint32_t) || memcmp(h->magic, POLICY_FILE_MAGIC, 4) != 0) {
        return "not a policy file";
    }
    if (h->version != POLICY_FILE_VERSION || h->header_size != sizeof(*h)) {
        return "unsupported policy file version";
    }
    if (h->total_size > len || h->total_size % 4 != 0 || h->total_size < sizeof(*h) + 4) {
        return "file truncated";
    }
    uint32_t stored_crc;
    memcpy(&stored_crc, base + h->total_size - 4, sizeof(stored_crc));
    if (policy_crc32(base, h->total_size - 4) != stored_crc) {
        return "CRC mismatch (file corrupted)";
    }
    if (h->num_layers < 1 || h->num_layers > POLICY_MAX_LAYERS) {
        return "too many layers";
    }
    if (h->num_tests < 1) {
        return "no test vectors";
    }
    if (h->name[sizeof(h->name) - 1] != '\0') {
        return "name not terminated";
    }

    size_t offset = sizeof(*h);
    net->layers = (const policy_file_layer_t *)(base + offset);
    offset += h->num_layers * sizeof(policy_file_layer_t);
    if (offset > h->total_size) {
        return "file truncated";
    }
    for (int l = 0; l < h->num_layers; l++) {
        const policy_file_layer_t *layer = &net->layers[l];
        int expected_in = l == 0 ? h->input_size : net->layers[l - 1].out;
        if (layer->in != expected_in || layer->in < 1 || layer->out < 1
            || layer->in > POLICY_MAX_WIDTH || layer->out > POLICY_MAX_WIDTH) {
            return "layer sizes don't chain or exceed POLICY_MAX_WIDTH";
        }
        if (layer->activation > POLICY_ACTIVATION_ELU) {
            return "unknown activation";
        }
        net->weights[l] = (const float *)(base + offset);
        offset += (size_t)layer->in * layer->out * sizeof(float);
        net->biases[l] = (const float *)(base + offset);
        offset += layer->out * sizeof(float);
        if (offset > h->total_size) {
            return "file truncated";
        }
        if (!all_finite(net->weights[l], (size_t)layer->in * layer->out)
            || !all_finite(net->biases[l], layer->out)) {
            return "weights contain NaN/Inf";
        }
    }
    if (net->layers[h->num_layers - 1].out != h->output_size) {
        return "last layer size != output size";
    }
    net->test_inputs = (const float *)(base + offset);
    offset += (size_t)h->num_tests * h->input_size * sizeof(float);
    net->test_outputs = (const float *)(base + offset);
    offset += (size_t)h->num_tests * h->output_size * sizeof(float);
    if (offset + 4 != h->total_size) {
        return "file size doesn't match its contents";
    }

    net->header = h;
    net->crc = stored_crc;
    return NULL;
}

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

void policy_net_forward(const policy_net_t *net, const float *obs, float *act,
                        float *buf_a, float *buf_b)
{
    const policy_file_header_t *h = net->header;
    float *bufs[2] = {buf_a, buf_b};
    const float *in = obs;
    for (int l = 0; l < h->num_layers; l++) {
        const policy_file_layer_t *layer = &net->layers[l];
        float *out = bufs[l & 1];
        dense(net->weights[l], net->biases[l], in, out, layer->in, layer->out, layer->activation);
        in = out;
    }
    for (int i = 0; i < h->output_size; i++) {
        float v = in[i];
        /* NaN fails both comparisons and ends up 0 */
        act[i] = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : (v == v ? v : 0.0f));
    }
}

float policy_net_test_error(const policy_net_t *net, float *buf_a, float *buf_b)
{
    const policy_file_header_t *h = net->header;
    float act[POLICY_MAX_WIDTH];
    float max_err = 0.0f;
    for (int t = 0; t < h->num_tests; t++) {
        policy_net_forward(net, net->test_inputs + t * h->input_size, act, buf_a, buf_b);
        for (int i = 0; i < h->output_size; i++) {
            float err = fabsf(act[i] - net->test_outputs[t * h->output_size + i]);
            if (!(err <= max_err)) {
                max_err = err;
            }
        }
    }
    return max_err;
}
