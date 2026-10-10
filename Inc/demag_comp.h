#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Bluejay-style demag compensation: see Src/demag_comp.c. DEMAG_COMP_ENABLED
 * comes from targets.h. */

/* demag metric at which the PWM phase is switched off from the crossing to
 * the commutation: 160 is Bluejay's "low" setting, 130 "high", 256 never */
#ifndef DEMAG_COMP_POWER_OFF
#define DEMAG_COMP_POWER_OFF 160
#endif // DEMAG_COMP_POWER_OFF
#ifndef DEMAG_COMP_MIN_INTERVAL
#if defined(MCU_G071) || defined(MCU_F051)
#define DEMAG_COMP_MIN_INTERVAL 100 /* 50 us sectors: the scan event costs too much of a shorter sector on the M0+ */
#else
#define DEMAG_COMP_MIN_INTERVAL 40
#endif // M0+
#endif // DEMAG_COMP_MIN_INTERVAL

#if DEMAG_COMP_ENABLED
extern volatile uint8_t demag_comp_state, demag_comp_invert, demag_comp_desync;
extern volatile uint8_t demag_comp_metric, demag_comp_metric_max;
extern volatile uint32_t demag_comp_sectors, demag_comp_flagged, demag_comp_blind, demag_comp_cuts, demag_comp_timeouts, demag_comp_blind_limit;

bool demag_comp_timer(void); /* first in PeriodElapsedCallback: true unless this is the armed commutation, due */
void demag_comp_commutate(void); /* first in commutate(), on every path */
bool demag_comp_scan(void); /* at commutation: true when the scan event will start listening */
uint8_t demag_comp_advance(uint8_t level);
/* the level the crossing filter accepts against: inverted while waiting for a clamp to end */
static inline char demag_comp_expect(char r) { return r ^ demag_comp_invert; }
bool demag_comp_clamp_end(void); /* after the crossing filter: true when the edge ended a clamp */
void demag_comp_disarm_event(void);
/* in interruptRoutine before the COM timer is armed: drop a pending event */
static inline void demag_comp_disarm(void)
{
    if (demag_comp_state) {
        demag_comp_disarm_event();
    }
}
void demag_comp_zc(void); /* in interruptRoutine after the COM timer is armed */
/* the scan has opened listening: the handler's blanking gate no longer applies */
static inline bool demag_comp_scanned(void) { return demag_comp_state >= 2; }
static inline bool demag_comp_desync_pending(void) { return demag_comp_desync != 0; }
static inline void demag_comp_desync_clear(void) { demag_comp_desync = 0; }
/* port functions */
void floatPwmPhase(char s); /* the PWM phase of step s off until the next comStep() */
void setCompEdge(char r); /* the comparator edge changeCompInput() selects for rising == r */
#else
static inline bool demag_comp_timer(void) { return false; }
static inline void demag_comp_commutate(void) { }
static inline bool demag_comp_scan(void) { return false; }
static inline uint8_t demag_comp_advance(uint8_t level) { return level; }
static inline char demag_comp_expect(char r) { return r; }
static inline bool demag_comp_clamp_end(void) { return false; }
static inline bool demag_comp_scanned(void) { return false; }
static inline void demag_comp_disarm(void) { }
static inline void demag_comp_zc(void) { }
static inline bool demag_comp_desync_pending(void) { return false; }
static inline void demag_comp_desync_clear(void) { }
#endif // DEMAG_COMP_ENABLED
