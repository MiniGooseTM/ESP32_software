/* Check a policy file on the PC with the firmware's own parser and forward
   pass (main/policy_net.c), before flashing it:

     cc -O2 -std=c11 -I../main policy_host_test.c ../main/policy_net.c -lm -o policy_host_test
     ./policy_host_test ../main/policy.bin

   Exits non-zero if the file is rejected or the forward pass doesn't
   reproduce the SB3 actions stored in it.
*/
#include <stdio.h>
#include <stdlib.h>
#include "policy_net.h"

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s policy.bin\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        perror(argv[1]);
        return 2;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    void *data = malloc(len);  /* malloc memory is aligned for any type */
    if (!data || fread(data, 1, len, f) != (size_t)len) {
        fprintf(stderr, "%s: read failed\n", argv[1]);
        return 2;
    }
    fclose(f);

    policy_net_t net;
    const char *err = policy_net_parse(&net, data, len);
    if (err) {
        fprintf(stderr, "%s rejected: %s\n", argv[1], err);
        return 1;
    }
    const policy_file_header_t *h = net.header;
    printf("'%s' (CRC %08x): %d -> %d, %d layers, %d tests, frame %d x %d, period %.4f s\n",
           h->name, (unsigned)net.crc, h->input_size, h->output_size, h->num_layers,
           h->num_tests, h->frame_size, h->history_length, h->control_period_s);
    printf("limits: %.3f m/s, %.3f rad/s, half track %.4f m, wheel %.2f rad/s, tilt %.2f rad\n",
           h->max_linear_command, h->max_angular_command, h->half_track_m,
           h->wheel_no_load_speed, h->tilt_limit_rad);

    float buf_a[POLICY_MAX_WIDTH], buf_b[POLICY_MAX_WIDTH];
    float max_err = policy_net_test_error(&net, buf_a, buf_b);
    int ok = max_err < 1e-4f;
    printf("test vectors: max error %.2e (%s)\n", max_err, ok ? "OK" : "MISMATCH");
    free(data);
    return ok ? 0 : 1;
}
