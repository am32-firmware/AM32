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
 * Timestamp source: the free-running 1 MHz utility timer, read through
 * get_timer_us16() (Inc/functions.h), which handles each vendor family's
 * register layout. 16-bit (wraps every 65.536 ms); all loop/ISR durations
 * measured here are far below the wrap period, so 16-bit unsigned subtraction
 * is exact. Values recorded across a >65 ms blocking event (startup/arming
 * tunes play with IRQs off inside the control loop) alias - the armed-edge
 * reset below discards those before a run's measurements start.
 */
#ifndef HWCI_PERF_H_
#define HWCI_PERF_H_

#ifdef HWCI_PERF

#if defined(NXP) || defined(WCH)
/* NXP has no free-running 1 MHz utility timer (get_timer_us16() returns 0).
 * On WCH the utility timer doubles as INTERVAL_TIMER, which main.c zeroes at
 * every zero-cross, so mid-measurement resets would corrupt every reading. */
#error "HWCI_PERF is not supported on this MCU family (needs a free-running 1 MHz utility timer)"
#endif

#include <stdint.h>

/* ASCII "HWC1" in little-endian memory order - lets the host locate/validate
 * the struct either by ELF symbol or by scanning RAM for the magic. */
#define HWCI_PERF_MAGIC   0x31435748u
/* v2: appended the zero-cross jitter block (zc_*) after host_cmd.
 * v3: appended zc_confirm_reject after the v2 jitter block (host_cmd stays
 *     frozen at offset 60).
 * v4: appended the 32-bin PWM-phase histogram of accepted zero-crossings. */
#define HWCI_PERF_VERSION 4u

/* PWM-phase histogram bins (power of two: the binning multiply-shift and the
 * host's modular math both assume it). */
#define HWCI_ZC_PHASE_BINS 32u

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

    /* --- counters ---
     * loop_iters is monotonic (the host differences it vs wall-clock for the
     * idle-residual CPU-load estimate). zero_cross_count mirrors main.c's
     * zero_crosses, which SATURATES at 10000 and resets on desync/stop - it
     * is a diagnostic value, NOT a monotonic counter. update_count increments
     * once per snapshot (every HWCI_PERF_SNAPSHOT_DIV main-loop iterations).
     */
    uint32_t loop_iters;               /* off 40 : while(1) iterations      */
    uint32_t zero_cross_count;         /* off 44 : zero_crosses mirror (sat)*/
    uint32_t commutation_interval;     /* off 48 : raw, 0.5us units         */
    uint32_t commutation_interval_max; /* off 52 : worst-case (slowest)     */
    uint32_t update_count;             /* off 56 : ++ each snapshot         */
    volatile uint32_t host_cmd;        /* off 60 : host writes, fw clears   */

    /* --- v2: zero-cross timing jitter (fed by HWCI_PERF_ZC) ---
     * Appended AFTER host_cmd so its offset (60) is identical in v1 and v2
     * firmware: during an A/B session the host must be able to issue
     * RESET_STATS to whichever vintage is flashed without a layout lookup. */
    uint32_t zc_count;                 /* off 64 : commutations accumulated */
    uint32_t zc_jitter_sum;            /* off 68 : sum |deviation|, ticks   */
    uint32_t zc_interval_sum;          /* off 72 : sum raw interval, ticks  */
    uint16_t zc_jitter_max;            /* off 76 : worst single deviation   */
    uint16_t _pad2;                    /* off 78 : keep sizeof 4-aligned    */

    /* --- v3: zero-cross confirm rejections (fed by HWCI_PERF_CONFIRM_REJECT)
     * Monotonic like the v2 sums: the host differences consecutive
     * snapshots, so delta(reject)/delta(zc_count) over a window is the
     * rejected-edges-per-accepted-commutation ratio - the live monitor for
     * the F051 glitch-tolerant confirm loop's reject mechanism. */
    uint32_t zc_confirm_reject;        /* off 80 : confirm-loop early-outs  */

    /* --- v4: PWM-phase histogram of accepted zero-crossings ---
     * bin = TIM1 phase (CNT/ARR) of the comparator edge at ISR entry, 32
     * bins across the PWM period. Each bin is a monotonic u16 event counter
     * that wraps naturally; the host differences consecutive snapshots
     * per-bin (mod 2^16), so the per-window histogram is exact as long as
     * one bin gains < 65536 events between 200 Hz reads (it gains < 100).
     * Uniform phase distribution = commutation free-running vs PWM; strong
     * peaks = ZC<->PWM phase correlation (injection locking) - the
     * discriminator for the beat-band jitter hump investigation (PR #23
     * found 3.3x edge-window enrichment at t30). */
    uint16_t zc_phase_hist[HWCI_ZC_PHASE_BINS]; /* off 84..147              */
} hwci_perf_t;                         /* total size: 148 bytes             */

extern volatile hwci_perf_t hwci_perf;

/* Q16 phase-binning factor, (HWCI_ZC_PHASE_BINS << 16) / (tim1_arr + 1).
 * Recomputed from the main loop (HWCI_PERF_MAIN_LOOP snapshot branch, one
 * soft division per ~1 ms, never in an ISR); 0 until first computed, which
 * the commit macro treats as "histogram off". Defined in hwci_perf.c. */
extern volatile uint32_t hwci_zc_phase_scale_q16;

/* Apply a pending host_cmd (e.g. reset accumulators). Defined in hwci_perf.c. */
void hwci_perf_apply_cmd(void);

/* Clear the sticky min/max accumulators (host command, and automatically on
 * the armed 0->1 edge to discard aliased values recorded while the arming
 * tune blocked the control loop). Defined in hwci_perf.c. */
void hwci_perf_reset_stats(void);

/* 16-bit microsecond timestamp from the free-running utility timer. Macro (not
 * inline) so it is only evaluated where Inc/functions.h is already included
 * (the struct definition above stays compilable on a host compiler). */
#define HWCI_NOW_US() get_timer_us16()

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
 * Zero-cross jitter instrumentation. Call once per commutation, at the END of
 * PeriodElapsedCallback() - never from interruptRoutine(), so the comparator
 * ISR whose detection timing this metric characterizes is not perturbed by
 * the act of measuring it. When PeriodElapsedCallback runs, thiszctime holds
 * the newest raw zero-cross interval (INTERVAL_TIMER ticks; interruptRoutine
 * zeroes the timer at every accepted crossing).
 *
 * The deviation is taken against the interval SIX commutations earlier - the
 * same motor phase and comparator edge one electrical revolution back.
 * Adjacent steps differ systematically (phase/comparator-edge asymmetry), so
 * a tick-to-tick delta would bury detection noise under that fixed
 * alternation; the 6-back reference cancels it.
 *
 * Accumulation is gated on zero_crosses >= 100, the same "stable running"
 * threshold the zero-cross filter tiers use: startup seeds (interval forced
 * to 10000) and post-desync recovery (firmware zeroes zero_crosses on
 * desync) stay out of the sums, and the history ring refills during the
 * gated-out span. zc_count/zc_jitter_sum/zc_interval_sum are monotonic - the
 * host differences consecutive SWD snapshots (wrap-safe u32, like
 * loop_iters), so delta(jitter_sum)/delta(interval_sum) over a window is the
 * mean fractional jitter with every commutation counted, immune to the 200 Hz
 * host sampling rate. zc_jitter_max is sticky and cleared alongside the other
 * maxima by hwci_perf_reset_stats().
 */
#define HWCI_PERF_ZC()                                                         \
    do {                                                                       \
        static uint16_t _zc_hist[6];                                           \
        static uint8_t _zc_idx;                                                \
        uint16_t _t = thiszctime;                                              \
        uint16_t _ref = _zc_hist[_zc_idx];                                     \
        _zc_hist[_zc_idx] = _t;                                                \
        if (++_zc_idx == 6u) _zc_idx = 0u;                                     \
        if (zero_crosses >= 100) {                                             \
            uint16_t _d = (_t >= _ref) ? (uint16_t)(_t - _ref)                 \
                                       : (uint16_t)(_ref - _t);                \
            hwci_perf.zc_count++;                                              \
            hwci_perf.zc_jitter_sum += _d;                                     \
            hwci_perf.zc_interval_sum += _t;                                   \
            if (_d > hwci_perf.zc_jitter_max) hwci_perf.zc_jitter_max = _d;    \
        }                                                                      \
    } while (0)

/*
 * Zero-cross confirm rejection counter. Fires on the confirm loop's
 * early-out reject (an edge whose window failed confirmation and was handed
 * back to the comparator). Runs in the comparator ISR, but only on the
 * reject path - a single volatile u32 increment, off the hot accept path.
 */
#define HWCI_PERF_CONFIRM_REJECT() do { hwci_perf.zc_confirm_reject++; } while (0)

/*
 * PWM-phase histogram of accepted zero-crossings (F051 only: reads TIM1
 * registers directly). CAPTURE at the TOP of interruptRoutine() - the CNT
 * read must happen at ISR entry, before the confirm loop's wall-clock window
 * smears it (the capture is a local, so a rejected edge simply discards it).
 * COMMIT after the zero-cross is accepted, outside the IRQ-masked timing
 * section: one 32x32 multiply, shift, bounds check and u16 increment.
 * Gated on zero_crosses >= 100 like HWCI_PERF_ZC (same "stable running"
 * threshold), and on the Q16 scale being initialized.
 */
#if defined(MCU_F051)
#define HWCI_PERF_ZC_PHASE_CAPTURE() uint16_t _hwci_zc_phase_cnt = (uint16_t)TIM1->CNT
#define HWCI_PERF_ZC_PHASE_COMMIT()                                            \
    do {                                                                       \
        uint32_t _s = hwci_zc_phase_scale_q16;                                 \
        if (_s != 0u && zero_crosses >= 100) {                                 \
            uint32_t _bin = ((uint32_t)_hwci_zc_phase_cnt * _s) >> 16;         \
            if (_bin < HWCI_ZC_PHASE_BINS) {                                   \
                hwci_perf.zc_phase_hist[_bin]++;                               \
            }                                                                  \
        }                                                                      \
    } while (0)
#else
#define HWCI_PERF_ZC_PHASE_CAPTURE() do {} while (0)
#define HWCI_PERF_ZC_PHASE_COMMIT()  do {} while (0)
#endif

/*
 * Background-loop instrumentation. Call once at the top of the main while(1).
 *
 * Every iteration: measures iteration time and counts iterations (the host
 * derives CPU load from the iteration rate vs an idle baseline), so the two
 * hot signals stay exact.
 *
 * Every HWCI_PERF_SNAPSHOT_DIV-th iteration only: snapshots live state and
 * services host commands. The host samples the struct at <= 200 Hz while the
 * main loop runs at ~100 kHz, so refreshing the snapshot every iteration would
 * burn ~10-15% of the core's spare cycles on stores that are overwritten
 * unread - and, worse, depress the idle loop_iters rate that IS the CPU-load
 * reference. At DIV=64 the snapshot is never staler than ~1 ms.
 *
 * The snapshot references main.c globals, so this macro must be expanded
 * where those globals are in scope (i.e. in main()). ISR-written globals are
 * cached into a local before use so the compare and the store can't see two
 * different values (a torn read would let the sticky max go backwards).
 */
#define HWCI_PERF_SNAPSHOT_DIV 64u

#define HWCI_PERF_MAIN_LOOP()                                                  \
    do {                                                                       \
        uint16_t _n = HWCI_NOW_US();                                           \
        static uint16_t _hwci_main_last;                                       \
        static uint8_t  _hwci_main_init;                                       \
        static uint8_t  _hwci_prev_armed;                                      \
        if (_hwci_main_init) {                                                 \
            uint16_t _d = (uint16_t)(_n - _hwci_main_last);                    \
            hwci_perf.main_loop_us_last = _d;                                  \
            if (_d > hwci_perf.main_loop_us_max) hwci_perf.main_loop_us_max = _d; \
        }                                                                      \
        _hwci_main_last = _n;                                                  \
        _hwci_main_init = 1;                                                   \
        hwci_perf.loop_iters++;                                                \
        if ((hwci_perf.loop_iters & (HWCI_PERF_SNAPSHOT_DIV - 1u)) == 0u) {    \
            uint32_t _ci = commutation_interval;                               \
            uint8_t _armed = (uint8_t)armed;                                   \
            hwci_perf.input = input;                                           \
            hwci_perf.duty_cycle = duty_cycle;                                 \
            hwci_perf.e_rpm = e_rpm;                                           \
            hwci_perf.voltage_cv = battery_voltage;                            \
            hwci_perf.current_ca = actual_current;                             \
            hwci_perf.temperature_c = degrees_celsius;                         \
            hwci_perf.bemf_timeout_state = (uint8_t)bemf_timeout_happened;     \
            hwci_perf.armed = _armed;                                          \
            hwci_perf.running = (uint8_t)running;                              \
            hwci_perf.zero_cross_count = zero_crosses;                         \
            hwci_perf.commutation_interval = _ci;                              \
            if (_ci > hwci_perf.commutation_interval_max)                      \
                hwci_perf.commutation_interval_max = _ci;                      \
            if (_armed && !_hwci_prev_armed)                                   \
                hwci_perf_reset_stats(); /* drop aliased arming-tune maxima */ \
            _hwci_prev_armed = _armed;                                         \
            /* phase-binning factor: one soft division per snapshot (~1 ms), \
             * tracks variable-PWM tim1_arr changes automatically */          \
            hwci_zc_phase_scale_q16 =                                          \
                ((uint32_t)HWCI_ZC_PHASE_BINS << 16) / ((uint32_t)tim1_arr + 1u); \
            hwci_perf.update_count++;                                          \
            if (hwci_perf.host_cmd != HWCI_CMD_NONE) hwci_perf_apply_cmd();    \
        }                                                                      \
    } while (0)

#else /* !HWCI_PERF - all hooks vanish, no struct, no code */

#define HWCI_PERF_CTRL_ENTER()       do {} while (0)
#define HWCI_PERF_CTRL_EXIT()        do {} while (0)
#define HWCI_PERF_ZC()               do {} while (0)
#define HWCI_PERF_MAIN_LOOP()        do {} while (0)
#define HWCI_PERF_CONFIRM_REJECT()   do {} while (0)
#define HWCI_PERF_ZC_PHASE_CAPTURE() do {} while (0)
#define HWCI_PERF_ZC_PHASE_COMMIT()  do {} while (0)

#endif /* HWCI_PERF */

#endif /* HWCI_PERF_H_ */
