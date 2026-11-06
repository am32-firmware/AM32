#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Demagnetisation guard: see Src/demag_guard.c. DEMAG_GUARD_ENABLED and
 * DEMAG_GUARD_CURRENT come from targets.h; DEMAG_GUARD_FORCE_GATE holds the
 * load gate open for bench and simulator tests of an unloaded motor. */
#ifndef DEMAG_GUARD_FORCE_GATE
#define DEMAG_GUARD_FORCE_GATE 0
#endif // DEMAG_GUARD_FORCE_GATE

#ifndef DEMAG_MAX_ADVANCE_LEVEL
#if defined(MCU_G071) || defined(MCU_F051)
#define DEMAG_MAX_ADVANCE_LEVEL 16 /* 15 degrees: keeps 8 us from crossing to commutation at 32 us sectors on the M0+ */
#else
#define DEMAG_MAX_ADVANCE_LEVEL 24 /* 0.9375 degree levels: 22.5 degrees */
#endif // M0+
#endif // DEMAG_MAX_ADVANCE_LEVEL
#ifndef DEMAG_MIN_SPAN_TICKS
#define DEMAG_MIN_SPAN_TICKS 16 /* skip checks when commutation to ZC is under 8 us */
#endif // DEMAG_MIN_SPAN_TICKS
#ifndef DEMAG_MIN_WAIT_TICKS
#if defined(MCU_G071) || defined(MCU_F051)
#define DEMAG_MIN_WAIT_TICKS 20 /* no guard under 10 us from crossing to commutation on the M0/M0+, whatever the configured advance */
#else
#define DEMAG_MIN_WAIT_TICKS 8 /* 4 us */
#endif // M0+
#endif // DEMAG_MIN_WAIT_TICKS
#ifndef DEMAG_DEADLINE_MARGIN_TICKS
#if defined(MCU_G071) || defined(MCU_F051)
#define DEMAG_DEADLINE_MARGIN_TICKS 12 /* the deadline handler must return before the commutation it arms: 6 us on the M0/M0+ */
#else
#define DEMAG_DEADLINE_MARGIN_TICKS 4 /* 2 us */
#endif // M0+
#endif // DEMAG_DEADLINE_MARGIN_TICKS
#if defined(MCU_SITL)
/* the SITL takes the policy limits from its model so a profile can select
 * another MCU's ceiling, minimum delay and deadline margin at run time */
uint8_t sitl_demag_max_advance_level(void);
uint8_t sitl_demag_min_wait_ticks(void);
uint8_t sitl_demag_deadline_margin_ticks(void);
#undef DEMAG_MAX_ADVANCE_LEVEL
#define DEMAG_MAX_ADVANCE_LEVEL sitl_demag_max_advance_level()
#undef DEMAG_MIN_WAIT_TICKS
#define DEMAG_MIN_WAIT_TICKS sitl_demag_min_wait_ticks()
#undef DEMAG_DEADLINE_MARGIN_TICKS
#define DEMAG_DEADLINE_MARGIN_TICKS sitl_demag_deadline_margin_ticks()
#endif // MCU_SITL
#ifndef COMP_DELAY_TICKS
#define COMP_DELAY_TICKS 0 /* comparator plus RC delay compensated at commutation */
#endif // COMP_DELAY_TICKS

#if DEMAG_GUARD_ENABLED
extern volatile uint8_t demag_active, demag_latched, demag_level, demag_adv_offset;
extern volatile uint16_t demag_cap;
extern volatile uint8_t demag_advance_level;
extern volatile uint32_t demag_valid, demag_predicted, demag_warnings, demag_faults, demag_late;
extern volatile uint32_t demag_commutation_late, demag_commutation_late_max; /* commutation interrupts entered more than 4 us after the armed time, and the worst entry delay, in ticks */
extern volatile uint32_t demag_switch_delay_max; /* worst armed time to after the phase switch, in ticks: the path cost the bench compares against */
extern volatile uint32_t demag_late_max; /* worst check or deadline service delay seen, in ticks */
extern volatile uint8_t demag_desync_request;

bool demag_guard_loaded(void);
bool demag_guard_compensate(void);
bool demag_guard_timer(void); /* first thing in PeriodElapsedCallback */
void demag_guard_switched(void); /* right after commutate(): lateness at the phase switch, the deferred response */
void demag_guard_commutated(void); /* last thing in PeriodElapsedCallback */
void demag_guard_pwm_period(uint16_t arr); /* before the loop writes the PWM period */
void demag_guard_off(void); /* guard state cleared and its timer events cancelled (before a flash write) */
void demag_guard_edge(void); /* every serviced comparator interrupt, after the handler's own blanking and crossing filter */
void demag_guard_zc(void); /* in interruptRoutine after the COM timer is armed */
uint8_t demag_guard_advance(uint8_t level);
uint16_t demag_guard_duty(uint16_t duty);
void demag_guard_pwm_commit(uint16_t compare, bool brake); /* the loop's final PWM compare write, bounded by the cap unless it is a brake */
void demag_guard_bound_sine(uint16_t compare[3], uint32_t waveform_peak); /* sine start's three compares, scaled together under the cap */
void demag_guard_housekeeping(void); /* 20 kHz */
void demag_guard_current_sample(void); /* after a fresh ADC conversion, about 1 kHz */
static inline bool demag_desync_pending(void) { return demag_desync_request != 0; }
static inline void demag_desync_clear(void) { demag_desync_request = 0; }
bool demag_comparator_pending(void); /* an edge of the armed polarity is pending; read only */
#else
#define demag_active 0
#define demag_latched 0
static inline bool demag_guard_loaded(void) { return false; }
static inline bool demag_guard_compensate(void) { return false; }
static inline bool demag_guard_timer(void) { return false; }
static inline void demag_guard_switched(void) { }
static inline void demag_guard_commutated(void) { }
static inline void demag_guard_pwm_period(uint16_t arr) { (void)arr; }
static inline void demag_guard_off(void) { }
static inline void demag_guard_edge(void) { }
static inline void demag_guard_zc(void) { }
static inline uint8_t demag_guard_advance(uint8_t level) { return level; }
static inline uint16_t demag_guard_duty(uint16_t duty) { return duty; }
static inline void demag_guard_housekeeping(void) { }
static inline void demag_guard_current_sample(void) { }
static inline void demag_guard_bound_sine(uint16_t compare[3], uint32_t waveform_peak) { (void)compare; (void)waveform_peak; }
static inline bool demag_desync_pending(void) { return false; }
static inline void demag_desync_clear(void) { }
#endif // DEMAG_GUARD_ENABLED
