#include <check.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Pull in the actual production code under test */
#include "Mcu/l431/Src/eeprom.c"

/* Invariant: eeprom read/write must not access memory outside valid bounds
 * regardless of index, length, or offset parameters. */

START_TEST(test_eeprom_bounds_invariant)
{
    /* Payloads: (index, out_buff_len) pairs
     * 1. Exact exploit: large index causing index*4 overflow
     * 2. Boundary: index at max valid - 1
     * 3. Valid: normal small index and length
     * 4. out_buff_len larger than buffer
     */
    struct { int index; int out_buff_len; } payloads[] = {
        { 0x40000001, 4   },   /* index*4 integer overflow / OOB */
        { 255,        4   },   /* boundary index */
        { 1,          4   },   /* valid input */
        { 0,          4096},   /* out_buff_len exceeds typical buffer */
    };
    int num_payloads = sizeof(payloads) / sizeof(payloads[0]);

    /* Allocate a guarded output buffer with known sentinel bytes */
    const int SAFE_BUF_SIZE = 256;
    uint8_t out_buf[256 + 16];
    memset(out_buf, 0xAB, sizeof(out_buf));

    for (int i = 0; i < num_payloads; i++) {
        int idx = payloads[i].index;
        int len = payloads[i].out_buff_len;

        /* Clamp len to safe buffer size to test the invariant:
         * the function must not write beyond what was requested
         * and must not crash on adversarial index values. */
        int safe_len = (len > SAFE_BUF_SIZE) ? SAFE_BUF_SIZE : len;

        memset(out_buf, 0xAB, sizeof(out_buf));

        /* The invariant: calling eeprom_read with adversarial params
         * must not corrupt memory beyond the output buffer.
         * Sentinel bytes after safe_len must remain 0xAB. */
        int ret = eeprom_read(idx, out_buf, safe_len);

        /* Sentinel check: bytes beyond safe_len must be untouched */
        for (int j = safe_len; j < (int)sizeof(out_buf); j++) {
            ck_assert_msg(out_buf[j] == 0xAB,
                "Buffer overwrite detected at byte %d for payload index=%d len=%d",
                j, idx, len);
        }

        /* Return value must be a defined/valid status, not garbage */
        ck_assert_msg(ret == 0 || ret == -1 || ret == 1,
            "Unexpected return value %d for payload index=%d len=%d",
            ret, idx, len);
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_eeprom_bounds_invariant);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}