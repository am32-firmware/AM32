/*
 * demag_comp.c - demag compensation after Bluejay (Timing.asm)
 *
 * Bluejay treats a sector as demagnetised when the comparator already shows
 * the post-crossing level as its zero-cross scan starts: the freewheeling
 * diode holds the floating phase at a rail. It refuses the post-crossing
 * level until it has seen the pre-crossing level (the clamp has ended),
 * then extends the scan timeout and exits run mode if no crossing follows;
 * a clamp that outlasts the first timeout is commutated blind. Blind
 * sectors feed a metric, a sliding average of 256 per blind sector floored
 * at 120: 130 adds 7.5 degrees of advance and 160 adds 15 (not at high
 * speed), and at the power-off threshold the PWM phase is switched off from
 * the crossing to the next commutation.
 *
 * Here the scan is a COM timer event as the comparator handler's blanking
 * gate opens, half an interval after the last crossing (Bluejay scans at
 * 37.5 degrees; the gate keeps the stock acceptance of early crossings). A
 * clamp selects the opposite comparator edge, so the stock handler and its
 * filter detect the end of the clamp. The first timeout is 7/8 of an
 * interval after the last crossing (52.5 degrees), the extended one 16
 * intervals or 1 ms at high speed; at its expiry, as at a long run of
 * blind sectors (not in Bluejay), the bridge is switched off and a desync
 * requested.
 */

#include "targets.h"
#include "demag_comp.h"

#if DEMAG_COMP_ENABLED

#include "common.h"
#include "comparator.h"
#include "peripherals.h"
#include "phaseouts.h"

#ifndef COM_TIMER_CLEAR_PENDING
#define COM_TIMER_CLEAR_PENDING() do { } while (0) /* port: drop a pending COM timer interrupt */
#endif // COM_TIMER_CLEAR_PENDING

extern volatile char rising;
extern char step, old_routine, prop_brake_active, stepper_sine;
extern uint8_t running, filter_level;
extern volatile uint16_t duty_cycle;
extern volatile uint32_t commutation_interval, average_interval, zero_crosses;
extern volatile uint16_t thiszctime, lastzctime, waitTime;
extern void interruptRoutine(void);

#define METRIC_FLOOR 120
#define METRIC_ADVANCE_1 130
#define METRIC_ADVANCE_2 160
#define ADVANCE_STEP 8 /* 7.5 degrees in 0.9375 degree levels */
#define ADVANCE_MAX 32 /* 30 degrees */
#define HIGH_RPM_INTERVAL 125 /* Bluejay's high rpm flag, sectors under 62.5 us */
#define MIN_ZERO_CROSSES 50
#define MAX_BLIND_RUN 24 /* four electrical turns without a crossing */
#define MAX_EVENT_TICKS 40000 /* below the stock 45000 tick back-EMF timeout */

enum { ST_IDLE, ST_SCAN, ST_WAIT_PRE, ST_LISTEN };

volatile uint8_t demag_comp_state, demag_comp_invert, demag_comp_desync;
volatile uint8_t demag_comp_metric = METRIC_FLOOR, demag_comp_metric_max;
volatile uint32_t demag_comp_sectors, demag_comp_flagged, demag_comp_blind, demag_comp_cuts, demag_comp_timeouts, demag_comp_blind_limit;
static uint8_t generation, event_generation, blind_run;
static uint8_t armed; /* a crossing or blind timeout armed the commutation */
static uint16_t armed_wait; /* its delay, against which a COM invocation is due */
static uint8_t used_timer; /* this sector armed a COM event, so a stale invocation may be pending */
static uint16_t event_start, event_ticks; /* interval timer count when the event was armed, and its delay */
static uint16_t run_interval; /* commutation_interval as the blind run started */

static bool eligible(void)
{
    /* blind crossings are early and shorten the interval: judge a blind run by its start */
    const uint32_t ci = blind_run ? run_interval : commutation_interval;
    return running && !old_routine && !prop_brake_active && !stepper_sine && duty_cycle > 0 && zero_crosses > MIN_ZERO_CROSSES
        && ci >= DEMAG_COMP_MIN_INTERVAL && ci <= 10000;
}

/* still the sector, and the interrupt-mode run, the event was armed in */
static bool live(void)
{
    return running && !old_routine && event_generation == generation;
}

/* every read at the post-crossing (want != rising) or pre-crossing level */
static bool level(bool post)
{
    for (uint8_t i = 0; i < filter_level; i++) {
        if ((getCompOutputLevel() == rising) == post) {
            return false;
        }
    }
    return true;
}

static void arm_in(uint32_t ticks)
{
    if (ticks > MAX_EVENT_TICKS) {
        ticks = MAX_EVENT_TICKS;
    }
    if (ticks < 2) {
        ticks = 2;
    }
    event_start = INTERVAL_TIMER_COUNT;
    event_ticks = ticks;
    used_timer = 1;
    COM_TIMER_CLEAR_PENDING();
    SET_AND_ENABLE_COM_INT(ticks);
}

/* target as an interval timer count since the last crossing */
static void arm_at(uint32_t target)
{
    const uint32_t now = INTERVAL_TIMER_COUNT;
    arm_in(target > now ? target - now : 0);
}

static void edge(bool clamp_end)
{
    if (demag_comp_invert != clamp_end) {
        demag_comp_invert = clamp_end;
        setCompEdge(rising ^ clamp_end);
    }
    maskPhaseInterrupts();
    enableCompInterrupts();
}

/* listen for the crossing, under Bluejay's extended timeout; true when the
 * crossing is already present, after the last edge we could see */
static bool listen(void)
{
    demag_comp_state = ST_LISTEN;
    edge(false);
    arm_in(commutation_interval < HIGH_RPM_INTERVAL ? 2000 : 16 * commutation_interval);
    return level(true);
}

/* back to the stock path, listening only if the run continues */
static void quit(void)
{
    DISABLE_COM_TIMER_INT();
    demag_comp_state = ST_IDLE;
    blind_run = 0;
    if (demag_comp_invert) {
        demag_comp_invert = 0;
        setCompEdge(rising);
    }
    if (running && !old_routine && event_generation == generation) {
        maskPhaseInterrupts();
        enableCompInterrupts();
    }
}

static void sector_end(bool blind)
{
    uint16_t m = ((uint16_t)demag_comp_metric * 7 + (blind ? 256 : 0)) >> 3;
    if (m < METRIC_FLOOR) {
        m = METRIC_FLOOR;
    }
    demag_comp_metric = m;
    if (m > demag_comp_metric_max) {
        demag_comp_metric_max = m;
    }
#ifndef PWM_ENABLE_BRIDGE /* phaseXPWM() restores no enable without complementary PWM */
    if (m >= DEMAG_COMP_POWER_OFF && eligible()) {
        floatPwmPhase(step);
        demag_comp_cuts++;
    }
#endif // PWM_ENABLE_BRIDGE
}

/* Bluejay switches the bridge off and leaves run mode */
static void power_off_desync(void)
{
    DISABLE_COM_TIMER_INT();
    COM_TIMER_CLEAR_PENDING();
    maskPhaseInterrupts();
    allOff();
    demag_comp_state = ST_IDLE;
    armed = 0;
    demag_comp_desync = 1;
}

static void scan_event(void)
{
    DISABLE_COM_TIMER_INT();
    if (!live() || !eligible()) {
        quit();
        return;
    }
    demag_comp_sectors++;
    if (level(true)) {
        demag_comp_flagged++;
        demag_comp_state = ST_WAIT_PRE;
        edge(true);
        if (!level(false)) {
            const uint32_t ci = commutation_interval;
            const uint32_t now = INTERVAL_TIMER_COUNT;
            if (ci - (ci >> 3) > now + (ci >> 2)) {
                arm_at(ci - (ci >> 3));
            } else {
                arm_in(ci >> 2);
            }
            return;
        }
        /* the clamp ended before its edge was selected */
    }
    if (listen()) {
        interruptRoutine();
    }
}

/* Bluejay commutates blind when the clamp outlasts the first timeout */
static void blind_event(void)
{
    if (!live() || !eligible()) {
        quit();
        return;
    }
    if (level(false)) { /* the clamp ended unseen, inside a gate that grew */
        if (listen()) {
            interruptRoutine();
        }
        return;
    }
    /* blind crossings shorten the interval: stop before the scan costs too much */
    if (blind_run + 1 >= MAX_BLIND_RUN || commutation_interval < DEMAG_COMP_MIN_INTERVAL / 2) {
        demag_comp_blind++;
        demag_comp_blind_limit++;
        power_off_desync();
        return;
    }
    if (blind_run == 0) {
        run_interval = commutation_interval;
    }
    __disable_irq();
    DISABLE_COM_TIMER_INT();
    COM_TIMER_CLEAR_PENDING();
    maskPhaseInterrupts();
    demag_comp_invert = 0;
    setCompEdge(rising);
    demag_comp_state = ST_IDLE;
    lastzctime = thiszctime;
    thiszctime = INTERVAL_TIMER_COUNT;
    SET_INTERVAL_TIMER_COUNT(0);
    SET_AND_ENABLE_COM_INT(waitTime + 1);
    armed = 1;
    armed_wait = waitTime;
    __enable_irq();
    demag_comp_blind++;
    blind_run++;
    sector_end(true);
}

/* no crossing within the extended timeout: Bluejay exits run mode */
static void timeout_event(void)
{
    if (!live() || !eligible()) {
        quit();
        return;
    }
    demag_comp_timeouts++;
    power_off_desync();
}

bool demag_comp_timer(void)
{
    if (demag_comp_state == ST_IDLE) {
        if (!armed) {
            DISABLE_COM_TIMER_INT(); /* nothing armed: a stale invocation */
            return true;
        }
        if (used_timer && INTERVAL_TIMER_COUNT < armed_wait) {
            return true; /* early: an invocation left by one of our events */
        }
        armed = 0;
        return false;
    }
    if (live() && (uint16_t)(INTERVAL_TIMER_COUNT - event_start) + 1 < event_ticks) {
        return true; /* early: an invocation left by an earlier arm */
    }
    switch (demag_comp_state) {
    case ST_SCAN:
        scan_event();
        break;
    case ST_WAIT_PRE:
        blind_event();
        break;
    default:
        timeout_event();
        break;
    }
    return true;
}

void demag_comp_commutate(void)
{
    generation++;
    if (demag_comp_state != ST_IDLE || armed) {
        DISABLE_COM_TIMER_INT(); /* a polling or start commutation: nothing armed survives it */
        COM_TIMER_CLEAR_PENDING();
    }
    demag_comp_state = ST_IDLE;
    demag_comp_invert = 0; /* commutate() selects the edge again */
    armed = 0;
    used_timer = 0;
    demag_comp_desync = 0; /* a commutation already followed the request */
    if (!running || old_routine || zero_crosses <= MIN_ZERO_CROSSES) {
        demag_comp_metric = METRIC_FLOOR;
        blind_run = 0;
    }
}

bool demag_comp_scan(void)
{
    if (!eligible()) {
        blind_run = 0;
        return false;
    }
    /* just after the handler's gate, which some ports take from average_interval */
    const uint32_t gate = commutation_interval > average_interval ? commutation_interval : average_interval;
    event_generation = generation;
    demag_comp_state = ST_SCAN;
    arm_at((gate >> 1) + 2);
    return true;
}

uint8_t demag_comp_advance(uint8_t level)
{
    if (demag_comp_metric < METRIC_ADVANCE_1 || commutation_interval < HIGH_RPM_INTERVAL || !eligible()) {
        return level;
    }
    const uint8_t boosted = level + (demag_comp_metric >= METRIC_ADVANCE_2 ? 2 * ADVANCE_STEP : ADVANCE_STEP);
    return boosted > ADVANCE_MAX ? (level > ADVANCE_MAX ? level : ADVANCE_MAX) : boosted;
}

bool demag_comp_clamp_end(void)
{
    if (demag_comp_state != ST_WAIT_PRE) {
        return false;
    }
    __disable_irq();
    DISABLE_COM_TIMER_INT();
    COM_TIMER_CLEAR_PENDING();
    bool crossed = false;
    if (live()) {
        crossed = listen();
    } else {
        quit();
    }
    __enable_irq();
    if (crossed) {
        interruptRoutine();
    }
    return true;
}

void demag_comp_disarm_event(void)
{
    DISABLE_COM_TIMER_INT();
    COM_TIMER_CLEAR_PENDING();
    if (demag_comp_invert) {
        demag_comp_invert = 0;
        setCompEdge(rising);
    }
}

void demag_comp_zc(void)
{
    armed = 1;
    armed_wait = waitTime;
    blind_run = 0;
    demag_comp_desync = 0; /* a measured crossing: the run has recovered */
    if (demag_comp_state == ST_IDLE) {
        /* a sector we did not scan decays the metric as a healthy one */
        const uint16_t m = ((uint16_t)demag_comp_metric * 7) >> 3;
        demag_comp_metric = m < METRIC_FLOOR ? METRIC_FLOOR : m;
        return;
    }
    demag_comp_state = ST_IDLE;
    sector_end(false);
}

#endif // DEMAG_COMP_ENABLED
