/*
 * hwci_perf.c - Hardware-CI performance instrumentation for AM32
 *
 * See Inc/hwci_perf.h for the rationale and the struct-layout contract.
 *
 * The whole translation unit is empty unless HWCI_PERF is defined, so it is
 * safe to leave in the common source list (the Makefile globs the Src
 * directory); default builds emit nothing from it.
 */
#include "hwci_perf.h"

#ifdef HWCI_PERF

/*
 * The single instrumentation struct. Kept volatile so the compiler never
 * caches fields (the debugger reads them asynchronously over SWD) and never
 * elides the storage. The host finds it by the ELF symbol "hwci_perf".
 *
 * Min/extreme accumulators are seeded so the first real sample replaces them.
 */
volatile hwci_perf_t hwci_perf = {
    .magic = HWCI_PERF_MAGIC,
    .version = HWCI_PERF_VERSION,
    .size = (uint16_t)sizeof(hwci_perf_t),
    .ctrl_period_us_min = 0xFFFFu,
    .host_cmd = HWCI_CMD_NONE,
};

/*
 * Clear the sticky min/max accumulators so worst-case timing can be measured
 * for a single test run without power-cycling the ESC. Called for the host
 * HWCI_CMD_RESET_STATS command, and automatically on the armed 0->1 edge
 * (the arming tune blocks the control loop for ~300 ms with IRQs off, so the
 * 16-bit timestamps recorded around it alias to garbage that must not leak
 * into a run's maxima regardless of when the host issues its reset).
 */
void hwci_perf_reset_stats(void)
{
    hwci_perf.ctrl_exec_us_max = 0;
    hwci_perf.ctrl_period_us_max = 0;
    hwci_perf.ctrl_period_us_min = 0xFFFFu;
    hwci_perf.main_loop_us_max = 0;
    hwci_perf.commutation_interval_max = 0;
    hwci_perf.zc_jitter_max = 0;
}

/*
 * Service a host command. Called from the main loop (low priority) when
 * host_cmd is non-zero. Always clears host_cmd so the command fires once.
 */
void hwci_perf_apply_cmd(void)
{
    switch (hwci_perf.host_cmd) {
    case HWCI_CMD_RESET_STATS:
        hwci_perf_reset_stats();
        break;
    default:
        break;
    }
    hwci_perf.host_cmd = HWCI_CMD_NONE;
}

#endif /* HWCI_PERF */
