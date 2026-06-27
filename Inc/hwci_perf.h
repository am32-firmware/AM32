/*
 * hwci_perf.h - Hardware-CI performance instrumentation for AM32
 *
 * Optional, compile-gated (HWCI_PERF) instrumentation that maintains a small
 * struct in RAM with live timing / CPU-load / health counters. A debugger
 * (ST-Link via OpenOCD, or J-Link) reads this struct non-intrusively over SWD
 * background memory access while the motor runs.
 *
 * WHY a RAM struct instead of SWO/ITM trace:
 *   The ARK 4IN1 ESC uses the STM32F051 (Cortex-M0 / ARMv6-M). The M0 has no
 *   DWT cycle counter, no ITM, and no SWO output, so trace-based profiling is
 *   impossible on ANY probe. Background memory reads of an instrumented struct
 *   are the portable way to get CPU load and loop times off this core.
 *
 * When HWCI_PERF is NOT defined, every macro below expands to a no-op and no
 * struct or code is emitted - production builds are byte-for-byte unaffected.
 *
 * Timestamp source: UTILITY_TIMER (TIM17 on F0) runs free at 1 MHz, i.e. 1 us
 * per tick, 16-bit (wraps every 65.536 ms). All loop/ISR durations measured
 * here are far below the wrap period, so 16-bit unsigned subtraction is exact.
 */
#ifndef HWCI_PERF_H_
#define HWCI_PERF_H_

#ifdef HWCI_PERF

#include <stdint.h>

/* ASCII "HWC1" in little-endian memory order - lets the host locate/validate
 * the struct either by ELF symbol or by scanning RAM for the magic. */
#define HWCI_PERF_MAGIC   0x31435748u
#define HWCI_PERF_VERSION 1u

/* Commands the host may write to hwci_perf.host_cmd (cleared by firmware). */
#define HWCI_CMD_NONE          0u
#define HWCI_CMD_RESET_STATS   0xA5u   /* clear the min/max accumulators */

/*
 * Naturally-aligned layout (NOT packed): Cortex-M0 faults on unaligned word
 * access, so all 32-bit fields sit on 4-byte boundaries and 16-bit fields on
 * 2-byte boundaries. The explicit offsets below are the contract the host
 * decoder (hwci/hwci/perf.py) mirrors. Keep them in sync and bump
 * HWCI_PERF_VERSION on any layout change.
 */
typedef struct hwci_perf_s {
    uint32_t magic;                    /* off 0  : HWCI_PERF_MAGIC          */
    uint16_t version;                  /* off 4  : HWCI_PERF_VERSION        */
    uint16_t size;                     /* off 6  : sizeof(hwci_perf_t)      */

    /* --- loop timing, microseconds (UTILITY_TIMER 1us tick) --- */
    uint16_t ctrl_exec_us_last;        /* off 8  : last tenKhzRoutine run   */
    uint16_t ctrl_exec_us_max;         /* off 10 : worst-case run           */
    uint16_t ctrl_period_us_last;      /* off 12 : last entry-to-entry gap  */
    uint16_t ctrl_period_us_max;       /* off 14 : worst-case gap (jitter)  */
    uint16_t ctrl_period_us_min;       /* off 16 : best-case gap            */
    uint16_t main_loop_us_last;        /* off 18 : last while(1) iteration  */
    uint16_t main_loop_us_max;         /* off 20 : worst-case iteration     */

    /* --- live state snapshot (mirrors telemetry for self-contained logs) --- */
    uint16_t input;                    /* off 22 : throttle input 0..2047   */
    uint16_t duty_cycle;               /* off 24 : applied duty 0..2000     */
    uint16_t e_rpm;                    /* off 26 : electrical rpm / 100     */
    uint16_t voltage_cv;               /* off 28 : battery, centivolts      */
    int16_t  current_ca;               /* off 30 : current, centiamps       */
    int16_t  temperature_c;            /* off 32 : MCU/FET temp, Celsius    */
    uint8_t  bemf_timeout_state;       /* off 34 : bemf_timeout_happened    */
    uint8_t  armed;                    /* off 35 : armed flag               */
    uint8_t  running;                  /* off 36 : running flag             */
    uint8_t  _pad0;                    /* off 37 : alignment                */
    uint16_t _pad1;                    /* off 38 : align next u32 to off 40 */

    /* --- monotonic counters (host differences these vs wall-clock) --- */
    uint32_t loop_iters;               /* off 40 : while(1) iterations      */
    uint32_t zero_cross_count;         /* off 44 : zero_crosses mirror      */
    uint32_t commutation_interval;     /* off 48 : raw, 0.5us units         */
    uint32_t commutation_interval_max; /* off 52 : worst-case (slowest)     */
    uint32_t update_count;             /* off 56 : ++ each main-loop tick   */
    volatile uint32_t host_cmd;        /* off 60 : host writes, fw clears   */
} hwci_perf_t;                         /* total size: 64 bytes              */

extern volatile hwci_perf_t hwci_perf;

/* Apply a pending host_cmd (e.g. reset accumulators). Defined in hwci_perf.c. */
void hwci_perf_apply_cmd(void);

/* 16-bit microsecond timestamp from the free-running utility timer. Macro (not
 * inline) so it is only evaluated where the device headers / UTILITY_TIMER are
 * already in scope. */
#define HWCI_NOW_US() ((uint16_t)(UTILITY_TIMER->CNT))

/*
 * Control-loop (tenKhzRoutine, 20 kHz) instrumentation. ENTER at the very top,
 * EXIT at the very bottom. ENTER records the period (entry-to-entry); EXIT
 * records execution time (entry-to-exit). Runs in ISR context - keep it tiny.
 */
#define HWCI_PERF_CTRL_ENTER()                                                 \
    uint16_t _hwci_ctrl_t0 = HWCI_NOW_US();                                    \
    do {                                                                       \
        static uint16_t _hwci_last_entry;                                      \
        static uint8_t  _hwci_ctrl_init;                                       \
        if (_hwci_ctrl_init) {                                                 \
            uint16_t _p = (uint16_t)(_hwci_ctrl_t0 - _hwci_last_entry);        \
            hwci_perf.ctrl_period_us_last = _p;                               \
            if (_p > hwci_perf.ctrl_period_us_max) hwci_perf.ctrl_period_us_max = _p; \
            if (_p < hwci_perf.ctrl_period_us_min) hwci_perf.ctrl_period_us_min = _p; \
        }                                                                      \
        _hwci_last_entry = _hwci_ctrl_t0;                                      \
        _hwci_ctrl_init = 1;                                                   \
    } while (0)

#define HWCI_PERF_CTRL_EXIT()                                                  \
    do {                                                                       \
        uint16_t _e = (uint16_t)(HWCI_NOW_US() - _hwci_ctrl_t0);               \
        hwci_perf.ctrl_exec_us_last = _e;                                      \
        if (_e > hwci_perf.ctrl_exec_us_max) hwci_perf.ctrl_exec_us_max = _e;  \
    } while (0)

/*
 * Background-loop instrumentation. Call once at the top of the main while(1).
 * Measures iteration time, counts iterations (the host derives CPU load from
 * the iteration rate vs an idle baseline), snapshots live state, and services
 * host commands. The snapshot references main.c globals, so this macro must be
 * expanded where those globals are in scope (i.e. in main()).
 */
#define HWCI_PERF_MAIN_LOOP()                                                  \
    do {                                                                       \
        uint16_t _n = HWCI_NOW_US();                                           \
        static uint16_t _hwci_main_last;                                       \
        static uint8_t  _hwci_main_init;                                       \
        if (_hwci_main_init) {                                                 \
            uint16_t _d = (uint16_t)(_n - _hwci_main_last);                    \
            hwci_perf.main_loop_us_last = _d;                                  \
            if (_d > hwci_perf.main_loop_us_max) hwci_perf.main_loop_us_max = _d; \
        }                                                                      \
        _hwci_main_last = _n;                                                  \
        _hwci_main_init = 1;                                                   \
        hwci_perf.loop_iters++;                                                \
        hwci_perf.input = input;                                               \
        hwci_perf.duty_cycle = duty_cycle;                                     \
        hwci_perf.e_rpm = e_rpm;                                               \
        hwci_perf.voltage_cv = battery_voltage;                                \
        hwci_perf.current_ca = actual_current;                                 \
        hwci_perf.temperature_c = degrees_celsius;                            \
        hwci_perf.bemf_timeout_state = (uint8_t)bemf_timeout_happened;         \
        hwci_perf.armed = (uint8_t)armed;                                      \
        hwci_perf.running = (uint8_t)running;                                  \
        hwci_perf.zero_cross_count = zero_crosses;                            \
        hwci_perf.commutation_interval = commutation_interval;                \
        if (commutation_interval > hwci_perf.commutation_interval_max)         \
            hwci_perf.commutation_interval_max = commutation_interval;         \
        hwci_perf.update_count++;                                              \
        if (hwci_perf.host_cmd != HWCI_CMD_NONE) hwci_perf_apply_cmd();        \
    } while (0)

#else /* !HWCI_PERF - all hooks vanish, no struct, no code */

#define HWCI_PERF_CTRL_ENTER() do {} while (0)
#define HWCI_PERF_CTRL_EXIT()  do {} while (0)
#define HWCI_PERF_MAIN_LOOP()  do {} while (0)

#endif /* HWCI_PERF */

#endif /* HWCI_PERF_H_ */
