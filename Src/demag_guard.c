/*
 * Demagnetisation guard for the six-step commutation loop.
 *
 * The problem. The comparator reads the floating phase against the virtual
 * neutral; interruptRoutine() accepts a zero crossing once the level has
 * held through filter_level reads, and PeriodElapsedCallback() commutates
 * waitTime later. When the outgoing phase current is large (heavy load, a
 * big low-inductance motor, a throttle punch) the freewheeling diode pins
 * that phase to a rail for part of the sector: the comparator shows the
 * post-crossing level long before the real crossing, and the stock path
 * either accepts a crossing far too early or sees none and desyncs.
 *
 * What the guard does. It hooks into the existing path without changing
 * it. Every serviced comparator interrupt calls demag_guard_edge() after
 * the handler's own blanking and crossing filter, interruptRoutine() calls
 * demag_guard_zc() after arming the COM timer, PeriodElapsedCallback()
 * calls demag_guard_timer() first (the guard's own events run on the same
 * timer), demag_guard_switched() after commutate() and
 * demag_guard_commutated() last, and tenKhzRoutine() calls
 * demag_guard_housekeeping(), demag_guard_duty() and
 * demag_guard_pwm_commit() around the compare write. Per sector, in
 * INTERVAL_TIMER ticks (0.5 us) from the last accepted crossing:
 *
 *   EV_BEGIN     16 ticks after the commutation: read the comparator; the
 *                sector is observed only if it shows the post-crossing level
 *   EV_CHECK1/2  half and three quarters of the way to the expected
 *                crossing: the level must still be post-crossing; a rail
 *                held longer than a PWM period by CHECK2 is level 2
 *   EV_DEADLINE  1.25 commutation intervals, still silent: arm EV_PREDICT
 *   EV_PREDICT   commutate when a crossing at the expected time would have;
 *                an edge pending at this point hands the sector back instead
 *
 * Any comparator edge from EV_BEGIN on cancels the sector's events and
 * leaves it to the stock path: a released phase near its crossing chatters
 * at the PWM edges, a diode-clamped one is silent. Measured crossings are
 * accepted exactly as before, the guard never rejects one, and a prediction
 * re-bases the interval timer only at the moment it commutates, without
 * touching commutation_interval's history.
 *
 * The response, applied after the phase switch. Level 2 raises the advance
 * one level per two sectors up to DEMAG_MAX_ADVANCE_LEVEL. A prediction with
 * the motor driving (level 3) cuts the duty cap by a tenth; the cap bounds
 * the compare write in demag_guard_pwm_commit() and the sine-start
 * compares in demag_guard_bound_sine(). At the advance ceiling a level-2
 * sector cuts 20 duty points, but only while the interval of the same
 * polarity keeps growing, so the rising/falling alternation of the stock
 * acceptance at light load cannot drive it. Advance and cap recover through
 * clean sectors. A run of predictions past BLIND_SECTORS or BLIND_TICKS ends
 * in the stock desync exit. Comparator delay compensation
 * (COMP_DELAY_TICKS) starts with the first real response and lasts through
 * that loaded episode.
 *
 * The gate. None of the above runs unless loaded: applied duty above
 * LOAD_DUTY_ENTER (40 %, 35 % to leave) with the commutation interval under
 * LOAD_MAX_INTERVAL, armed, running, in interrupt mode, not in sine start
 * or braking. With DEMAG_GUARD_CURRENT a current sensor can open the gate
 * earlier, after an idle zero learned from stable samples that fits the
 * target's declared scale and two measured responses to duty changes; a
 * sensor that fails that is ignored. A falling throttle or duty suspends
 * observation until the drive has settled, so a deceleration is never read
 * as a clamp. With the gate closed every hook returns at once and the
 * commutation timing is the stock timing.
 *
 * Hardware it relies on: the comparator EXTI pending flag readable at any
 * time (demag_comparator_pending()), the COM timer re-armable within a
 * sector with COM_TIMER_CLEAR_PENDING(), INTERVAL_TIMER in 0.5 us ticks,
 * and the edge hook placed after the handler's filter: on some boards
 * (Sequre G431) a few hundred nanoseconds ahead of the filter changes which
 * short glitches are accepted as crossings and shifts the timing.
 *
 * The counters (demag_valid, demag_predicted, demag_warnings, demag_faults
 * and the lateness counters) go out over DroneCAN FlexDebug when the guard
 * is built. DEMAG_GUARD_FORCE_GATE=1 holds the gate open for bench and
 * simulator tests of an unloaded motor.
 */
#include "main.h"
#include "targets.h"
#include "demag_guard.h"

#if DEMAG_GUARD_ENABLED
#include "common.h"
#include "comparator.h"
#include "peripherals.h"
#include "phaseouts.h"

#ifndef COM_TIMER_CLEAR_PENDING
#define COM_TIMER_CLEAR_PENDING() do { } while (0) /* port: drop a pending COM timer interrupt */
#endif // COM_TIMER_CLEAR_PENDING

extern volatile char armed, rising;
extern uint8_t running;
extern char old_routine, stepper_sine, prop_brake_active;
extern uint16_t input, minimum_duty_cycle, last_duty_cycle, ADC_raw_current;
extern volatile uint16_t duty_cycle, waitTime, thiszctime, lastzctime;
extern volatile uint32_t commutation_interval, zero_crosses;

volatile uint8_t demag_active, demag_latched, demag_level, demag_adv_offset;
volatile uint16_t demag_cap = 2000;
volatile uint8_t demag_advance_level;
volatile uint32_t demag_valid, demag_predicted, demag_warnings, demag_faults, demag_late;
volatile uint32_t demag_commutation_late, demag_commutation_late_max, demag_switch_delay_max, demag_late_max;
volatile uint8_t demag_desync_request;

enum { EV_NONE, EV_BEGIN, EV_CHECK1, EV_CHECK2, EV_DEADLINE, EV_PREDICT };

/* policy constants, in 0.5 us ticks, 50 us housekeeping ticks or sectors */
#define LOAD_ENTER 100 /* measured bus current, in 10 mA units */
#define LOAD_LEAVE 30
#define LOAD_PEAK_AMPS 8
#define LOAD_HOLD 80 /* 4 ms after a peak ADC sample */
#define LOAD_DUTY_ENTER 800 /* 40% of the applied 0..2000 duty */
#define LOAD_DUTY_LEAVE 700 /* 35% */
#define LOAD_MAX_INTERVAL 2000 /* the slowest interval at which the stock commutation is stable */
#if DEMAG_GUARD_CURRENT && !defined(NO_CURRENT_SENSE)
#define LOAD_CURRENT_SENSE 1
#else
#define LOAD_CURRENT_SENSE 0
#endif // DEMAG_GUARD_CURRENT
#define CUT_HOLD 2 /* at most one cut per 100 us */
#define RECOVER_HOLD 20 /* 1 ms after a cut before the cap recovers */
#define RECOVER_STEP 4 /* then one duty point per 200 us */
#define ADVANCE_DECAY 400 /* one level per 20 ms without a level-2 sector */
#define CLEAN_SECTORS 6 /* clean sectors before the cap recovers and the prediction budget refills */
#define BLIND_SECTORS 6 /* consecutive predicted sectors before the desync exit */
#define LATE_TICKS 8 /* a check or deadline serviced more than 4 us late is counted */
#define BLIND_TICKS 40000 /* 20 ms of predicted sectors (summed sector time, 0.5 us ticks) ends prediction */
#define BLIND_TICKS_LOW_DUTY 200000 /* 100 ms below a quarter duty (regeneration) */
static uint8_t event, level, blind, clean, severe, adv_hold;
enum { PEND_NONE, PEND_RESPONSE, PEND_PREDICT };
static uint8_t pending;
static uint8_t loaded, compensating, post_seen, activity, interval_growing;
static uint8_t proxy_loaded;
#if LOAD_CURRENT_SENSE
static uint8_t current_hold, current_loaded, current_tracking, tracking_samples;
static uint16_t current_age, tracking_duty;
static uint32_t tracking_current, duty_average;
#define CURRENT_MAX_AGE 200 /* ignore stale ADC data after 10 ms */
#define TRACK_SAMPLES 64 /* compare averaged current and duty about every 67 ms */
#define TRACK_DUTY_STEP 80 /* at least four percentage points */
#define TRACK_CONFIRMATIONS 2 /* two observed current responses before widening */
/* Only fresh, stable, stopped samples may establish a sensor zero. */
#define ZERO_SAMPLES 64
#define ZERO_SPREAD 4
static uint8_t zero_samples, current_zero_valid;
static uint16_t zero_min, zero_max, current_zero;
static uint32_t zero_sum, current_average;
#endif // LOAD_CURRENT_SENSE
static uint32_t t_post, t_event;
static uint16_t pwm_ticks = 252;
static uint16_t predicted_wait, measured[2];
static uint32_t dead_now;
static uint32_t t_check1, t_check2, t_dead, span;
static uint32_t ticks, cut_at, recover_at, decay_at, blind_ticks;
static uint32_t cap_scale;
static uint16_t scaled_arr, written_arr;
static uint8_t written_age, full_on, full_on_age;
static uint16_t drive_duty, drive_interval, drive_previous, drive_input, drive_hold;
static uint8_t coasting;
#define DRIVE_SETTLE 2000 /* 100 ms after a falling drive command */

/* the duty actually applied by the last loop pass, not the loop's working value */
static uint16_t applied(void)
{
    return last_duty_cycle;
}

/* A throttle cut precedes the PWM ramp reaching its new duty. */
static bool regenerating(void)
{
    return applied() < 500 || input < applied();
}

bool demag_guard_loaded(void)
{
    return loaded != 0;
}

bool demag_guard_compensate(void)
{
    return compensating != 0;
}

static bool eligible(void)
{
    return armed && running && !old_routine && !stepper_sine && !prop_brake_active
        && loaded && input > 47 && zero_crosses > 100
        && !coasting && input >= drive_input && applied() * 4U >= drive_previous * 3U
        && commutation_interval >= 64 && commutation_interval <= 4000;
}

/* arm the commutation timer for an absolute interval-frame target */
static void arm_at(uint8_t ev, uint32_t target)
{
    const uint32_t now = INTERVAL_TIMER_COUNT;
    event = ev;
    t_event = target;
    COM_TIMER_CLEAR_PENDING(); /* an invocation left pending by an earlier expiry is not this event */
    SET_AND_ENABLE_COM_INT((int32_t)(target - now) > 1 ? target - now : 1);
}

static void cancel(void)
{
    if (event != EV_NONE) {
        DISABLE_COM_TIMER_INT();
        COM_TIMER_CLEAR_PENDING();
        event = EV_NONE;
    }
}

/* Repeated reads seed an observation; only an edge-free duration proves it. */
static bool clamped(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        if (getCompOutputLevel() == rising) {
            return false;
        }
    }
    return true;
}

/* Convert the cap with a scale maintained outside the commutation ISR. */
static uint32_t cap_compare(uint16_t cap)
{
    return (((uint32_t)cap * cap_scale) >> 16) + 1;
}

static void write_cap(void)
{
    if (duty_cycle > demag_cap || last_duty_cycle > demag_cap) {
        duty_cycle = demag_cap;
        last_duty_cycle = duty_cycle;
        const uint32_t compare = cap_compare(demag_cap);
        full_on = full_on_age = 0;
        SET_DUTY_CYCLE_ALL(compare);
    }
}

static void cut(uint16_t amount)
{
    const uint16_t base = applied() < demag_cap ? applied() : demag_cap;
    const uint16_t floor = base < minimum_duty_cycle ? base : minimum_duty_cycle;
    demag_cap = base > floor + amount ? base - amount : floor;
    write_cap();
    cut_at = ticks + CUT_HOLD;
    recover_at = ticks + RECOVER_HOLD;
}

/* leave the sector machinery; the cap is kept so a derated restart stays derated */
static void reset(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (event != EV_NONE) {
        cancel();
    }
    demag_active = 0;
    demag_adv_offset = 0;
    blind = clean = severe = adv_hold = level = pending = 0;
    measured[0] = measured[1] = 0;
    blind_ticks = 0;
    post_seen = activity = interval_growing = 0;
    if (!mask) {
        __enable_irq();
    }
}

static void raise_advance(void)
{
    if (adv_hold) {
        adv_hold--;
        return;
    }
    if (demag_advance_level < DEMAG_MAX_ADVANCE_LEVEL) {
        demag_adv_offset++;
        adv_hold = 2; /* one level per two sectors */
    }
}

/* Apply the result only after the phase switch. */
static void respond(void)
{
    demag_level = level;
    if (!loaded) {
        return;
    }
    if (level < 2 || (level == 2 && !interval_growing)) {
        if (clean < 2 * CLEAN_SECTORS) {
            clean++;
        }
    } else {
        clean = 0;
    }
    if (level < 2) {
        severe = 0;
        return;
    }
    demag_warnings++;
    if (level == 3 && regenerating()) {
        /* A regenerative crossing needs prediction without further derating. */
        return;
    }
    compensating = 1; /* A real response starts correction; retain it through the final coast. */
    decay_at = ticks + ADVANCE_DECAY;
    if (severe < 255) {
        severe++;
    }
    const bool at_ceiling = demag_advance_level >= DEMAG_MAX_ADVANCE_LEVEL;
    raise_advance();
    if (level == 3) {
        const uint16_t tenth = ((uint32_t)demag_cap * 6554) >> 16; /* a tenth, without a division */
        if ((int32_t)(ticks - cut_at) >= 0) {
            cut(tenth > 100 ? tenth : 100);
        }
        return;
    }
    if (at_ceiling && interval_growing && severe >= 2 && (int32_t)(ticks - cut_at) >= 0) {
        cut(20);
        severe = 0;
    }
}

static void desync_exit(void)
{
    demag_faults++;
    maskPhaseInterrupts();
    cancel();
    pending = 0;
    demag_active = 0;
    allOff(); /* nothing commutates until the stock restart */
    if (demag_cap > 1000) {
        demag_cap = 1000; /* derate the restart; recovers through clean sectors */
    }
    write_cap();
    blind = 0;
    demag_desync_request = 1; /* the main loop runs the stock desync branch */
}

/* A tentative prediction never changes the measured interval frame. */
static void deadline(void)
{
    dead_now = INTERVAL_TIMER_COUNT;
    arm_at(EV_PREDICT, commutation_interval + predicted_wait + 1);
}

static void handoff(void)
{
    cancel();
    level = 0;
}

/* Observe only the edge the comparator interrupt already selected, including
 * inside blanking. Switching transients before EV_BEGIN go through the stock
 * path unchanged. */
void demag_guard_edge(void)
{
    if (!demag_active || event == EV_BEGIN || activity || pending != PEND_NONE) {
        return;
    }
    activity = 1;
    if (event == EV_PREDICT) {
        level = 0; /* A real edge refutes the tentative prediction. */
    }
    cancel();
}

static bool prediction_expired(uint32_t now)
{
    return blind >= BLIND_SECTORS
        || blind_ticks + commutation_interval > (regenerating() ? BLIND_TICKS_LOW_DUTY : BLIND_TICKS)
        || now > commutation_interval + predicted_wait + span / 4;
}

bool demag_guard_timer(void)
{
    if (!demag_active && event == EV_NONE && pending == PEND_NONE) {
        return false;
    }
    if (event == EV_NONE) {
        if (pending == PEND_NONE || INTERVAL_TIMER_COUNT < waitTime) {
            return true; /* A cancelled check cannot commutate. */
        }
        DISABLE_COM_TIMER_INT();
        COM_TIMER_CLEAR_PENDING();
        if (demag_active) {
            const int32_t late = (int32_t)(INTERVAL_TIMER_COUNT - waitTime - 1);
            if (late > (int32_t)demag_commutation_late_max) {
                demag_commutation_late_max = late;
            }
            if (late > LATE_TICKS) {
                demag_commutation_late++;
            }
        }
        return false;
    }
    const uint8_t ev = event;
    const uint32_t target = t_event;
    event = EV_NONE;
    DISABLE_COM_TIMER_INT();
    COM_TIMER_CLEAR_PENDING();
    if (!eligible()) {
        handoff();
        demag_active = 0;
        return true;
    }
    if (activity || demag_comparator_pending()) {
        activity = 1;
        handoff();
        return true;
    }
    const uint32_t now = INTERVAL_TIMER_COUNT;
    const int32_t late = (int32_t)(now - target);
    if (late > (int32_t)demag_late_max) {
        demag_late_max = late;
    }
    if (late > LATE_TICKS) {
        demag_late++;
    }
    if (ev == EV_PREDICT) {
        if (late > (int32_t)demag_commutation_late_max) {
            demag_commutation_late_max = late;
        }
        if (late > LATE_TICKS) {
            demag_commutation_late++;
        }
    }
    switch (ev) {
    case EV_BEGIN:
        if (late > LATE_TICKS) {
            handoff(); /* A late sample cannot establish the start of silence. */
            break;
        }
        post_seen = clamped();
        if (!post_seen) {
            handoff();
            break;
        }
        t_post = INTERVAL_TIMER_COUNT;
        arm_at(EV_CHECK1, t_check1);
        break;
    case EV_CHECK1:
    case EV_CHECK2:
        /* The opposite edge is deliberately not armed. A return to the
         * pre-crossing level is also activity, without changing EXTI. */
        if (!clamped()) {
            handoff();
            break;
        }
        /* Continuous drive has no PWM edges; its preload has already settled. */
        if (ev == EV_CHECK2 && post_seen
            && (now - t_post > pwm_ticks || (full_on && full_on_age >= 5))) {
            level = 2;
        }
        arm_at(ev == EV_CHECK1 ? EV_CHECK2 : EV_DEADLINE,
               ev == EV_CHECK1 ? t_check2 : t_dead);
        break;
    case EV_DEADLINE:
        if (!clamped()) {
            handoff();
            break;
        }
        deadline();
        break;
    case EV_PREDICT: {
        if (!clamped()) {
            handoff();
            return true;
        }
        const bool expired = prediction_expired(now);
        const uint32_t switch_now = INTERVAL_TIMER_COUNT;
        if (demag_comparator_pending()) {
            activity = 1;
            handoff();
            return true;
        }
        /* Rebase only when committing the phase switch, after the final veto. */
        if (expired) {
            desync_exit();
            return true;
        }
        maskPhaseInterrupts();
        SET_INTERVAL_TIMER_COUNT(switch_now - commutation_interval);
        waitTime = predicted_wait;
        pending = PEND_PREDICT;
        return false;
    }
    }
    return true;
}

/* right after the phase switch: how late it was, then the response the
 * crossing or the deadline deferred, still ahead of the advance calculation */
void demag_guard_switched(void)
{
    if (!demag_active) {
        return;
    }
    /* armed time to after the switch: the whole path, recorded, not judged */
    const int32_t delay = (int32_t)(INTERVAL_TIMER_COUNT - waitTime - 1);
    if (delay > (int32_t)demag_switch_delay_max) {
        demag_switch_delay_max = delay;
    }
    if (pending == PEND_PREDICT) {
        blind_ticks += commutation_interval;
        if ((int32_t)(dead_now - t_dead) > LATE_TICKS) {
            blind++;
        }
        if (!regenerating()) {
            blind++;
        }
        demag_predicted++;
        lastzctime = thiszctime;
        thiszctime = commutation_interval;
        level = 3;
    }
    if (pending != PEND_NONE) {
        pending = PEND_NONE;
        respond();
    }
}

/* sine start writes its three compares directly: one factor, constant over
 * the electrical cycle (from the waveform's own maximum, not the instant's),
 * brings its peak under the cap against the period the loop last wrote, so a
 * derated restart keeps the waveform's shape and the voltage angle */
void demag_guard_bound_sine(uint16_t compare[3], uint32_t waveform_peak)
{
    if (demag_cap >= 2000 || waveform_peak == 0) {
        return;
    }
    const uint32_t period = written_arr ? written_arr : TIMER1_MAX_ARR;
    const uint32_t bound = ((uint32_t)demag_cap * period) / 2000;
    if (waveform_peak <= bound) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        compare[i] = (uint16_t)(((uint32_t)compare[i] * bound) / waveform_peak);
    }
}

/* Shrinking the period lowers the cap scale before the preload is written. */
void demag_guard_pwm_period(uint16_t arr)
{
    if (arr != written_arr) {
        const uint16_t period = (((uint32_t)arr + 1) * 2 + CPU_FREQUENCY_MHZ - 1) / CPU_FREQUENCY_MHZ;
        if (period > pwm_ticks) {
            pwm_ticks = period;
        }
        written_arr = arr;
        written_age = full_on_age = 0;
        if (arr < scaled_arr) {
            scaled_arr = arr;
            cap_scale = ((uint32_t)arr << 16) / 2000;
        }
    }
}

void demag_guard_off(void)
{
    reset();
    compensating = 0;
}

void demag_guard_commutated(void)
{
    event = EV_NONE;
    pending = PEND_NONE;
    if (!eligible()) {
        demag_active = 0;
        measured[0] = measured[1] = 0;
        return;
    }
    const uint32_t now = INTERVAL_TIMER_COUNT;
    const uint32_t zc = commutation_interval;
    if (zc < now + DEMAG_MIN_SPAN_TICKS || waitTime < DEMAG_MIN_WAIT_TICKS) {
        demag_active = 0;
        return;
    }
    span = zc - now;
    t_check1 = now + span / 2;
    t_check2 = now + span * 3 / 4;
    /* A predicted commutation may be later than a measured one at high advance. */
    t_dead = zc + (zc + 3) / 4;
    predicted_wait = waitTime;
    if (predicted_wait < (zc + 3) / 4 + DEMAG_DEADLINE_MARGIN_TICKS) {
        predicted_wait = (zc + 3) / 4 + DEMAG_DEADLINE_MARGIN_TICKS;
    }
    level = 0;
    post_seen = activity = interval_growing = 0;
    const uint32_t t_begin = now + 16;
    if (t_begin + 2 >= t_check1) {
        demag_active = 0;
        return;
    }
    demag_active = 1;
    arm_at(EV_BEGIN, t_begin);
}

void demag_guard_zc(void)
{
    /* the crossing re-armed the timer: a check that pended meanwhile is stale */
    COM_TIMER_CLEAR_PENDING();
    blind_ticks = 0; /* a measured crossing, guarded or not */
    if (blind) {
        blind--;
    }
    if (!demag_active) {
        return;
    }
    event = EV_NONE;
    const uint16_t previous = measured[!!rising];
    /* Compare like polarities with room for crossing jitter on either sample. */
    interval_growing = previous && thiszctime > previous + commutation_interval / 4;
    measured[!!rising] = thiszctime;
    demag_valid++;
    pending = PEND_RESPONSE; /* respond after the switch, off this handler */
}

uint8_t demag_guard_advance(uint8_t level_in)
{
    uint32_t l = (uint32_t)level_in + (loaded ? demag_adv_offset : 0);
    /* the offset is bounded by the ceiling; a configured level above it is kept */
    if (l > DEMAG_MAX_ADVANCE_LEVEL && l > level_in) {
        l = level_in > DEMAG_MAX_ADVANCE_LEVEL ? level_in : DEMAG_MAX_ADVANCE_LEVEL;
    }
    demag_advance_level = l;
    return l;
}

uint16_t demag_guard_duty(uint16_t duty)
{
    return duty > demag_cap ? demag_cap : duty;
}

void demag_guard_pwm_commit(uint16_t compare, bool brake)
{
    /* the loop's final compare write: bound it by the cap, publish the duty
     * that is actually applied, and write, all in one short transaction so a
     * cut from an interrupt is never overwritten by a stale value */
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* a brake compare (proportional: inverse duty on the low sides; active:
     * comStep(2) with its own power) is not motoring duty: the cap, which a
     * derated stop keeps for the restart, must not change the brake. The tag
     * was set before this transaction; if a drive command has started the
     * motor meanwhile, the compare is stale and is bounded like motoring. */
    if (brake && armed && running && input > 47) {
        brake = false;
    }
    if (demag_cap < 2000 && !brake) {
        const uint32_t bound = cap_compare(demag_cap);
        if (compare > bound) {
            compare = bound;
            duty_cycle = demag_cap;
            last_duty_cycle = demag_cap;
        }
    }
    full_on = !brake && compare > written_arr;
    if (!full_on) {
        full_on_age = 0;
    }
    SET_DUTY_CYCLE_ALL(compare);
    if (!mask) {
        __enable_irq();
    }
}

/* raise the cap by one duty point when still eligible; a cut from an
 * interrupt wins and keeps its hold */
static void recover_step(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (demag_cap < 2000 && (!demag_active || clean >= CLEAN_SECTORS) && (int32_t)(ticks - recover_at) >= 0) {
        demag_cap = demag_cap + 20 > 2000 ? 2000 : demag_cap + 20;
        recover_at = ticks + RECOVER_STEP;
    }
    if (!mask) {
        __enable_irq();
    }
}

void demag_guard_current_sample(void)
{
#if LOAD_CURRENT_SENSE
    const uint16_t raw = ADC_raw_current;
    current_age = CURRENT_MAX_AGE;
    if (!armed || raw > 4095) {
        current_zero_valid = zero_samples = current_hold = current_loaded = current_tracking = tracking_samples = 0;
        return;
    }
    if (!running && input < 48 && !applied() && !prop_brake_active && !stepper_sine) {
        current_loaded = current_hold = current_tracking = tracking_samples = 0;
        duty_average = tracking_duty = tracking_current = 0;
        if (!zero_samples || raw + ZERO_SPREAD < zero_max || raw > zero_min + ZERO_SPREAD) {
            /* A changed idle zero invalidates the old one immediately. */
            current_zero_valid = 0;
            zero_samples = 0;
            zero_sum = 0;
            zero_min = zero_max = raw;
        }
        if (raw > zero_max) {
            zero_max = raw;
        }
        if (raw < zero_min) {
            zero_min = raw;
        }
        if (zero_samples < ZERO_SAMPLES) {
            zero_sum += raw;
            if (++zero_samples == ZERO_SAMPLES) {
                current_zero = (zero_sum + ZERO_SAMPLES - 1) / ZERO_SAMPLES;
                current_average = (uint32_t)current_zero << 5;
                /* A learned zero must also fit the target's sensor transfer:
                 * allow one amp of offset error plus four ADC counts of noise.
                 * A mid-rail reading with a ground-referenced target is ignored. */
                const int32_t offset = (int32_t)current_zero * 3300 - CURRENT_OFFSET * 4095;
                const int32_t tolerance = MILLIVOLT_PER_AMP * 4095 + ZERO_SPREAD * 3300;
                current_zero_valid = offset >= -tolerance && offset <= tolerance
                    && current_zero < 4095;
            }
        }
        return;
    }
    zero_samples = 0;
    if (!current_zero_valid) {
        current_loaded = current_hold = current_tracking = tracking_samples = 0;
        return;
    }
    /* Use our own baseline-relative average: telemetry may have the wrong
     * offset or ADC scale (including a permanently zero actual_current).
     * Average signed ADC levels before clipping regenerative current. */
    current_average = current_average - (current_average >> 5) + raw;
    const uint32_t zero = (uint32_t)current_zero << 5;
    const uint32_t delta = current_average > zero ? current_average - zero : 0;
    duty_average = duty_average - (duty_average >> 5) + applied();
    if (!running || old_routine || stepper_sine || prop_brake_active || coasting || input < 48 || zero_crosses <= 100) {
        tracking_samples = 0;
        tracking_duty = duty_average >> 5;
        tracking_current = delta;
    } else if (++tracking_samples >= TRACK_SAMPLES) {
        tracking_samples = 0;
        const uint16_t duty = duty_average >> 5;
        const int32_t duty_change = (int32_t)duty - tracking_duty;
        if (duty_change >= TRACK_DUTY_STEP || duty_change <= -TRACK_DUTY_STEP) {
            /* Demand must produce a measurable, same-direction response.
             * Static, inverted or shared-but-unrelated readings cannot qualify.
             * Compare ADC sums directly, without a conversion/division. */
            const int32_t change = (int32_t)delta - tracking_current;
            const uint32_t response = ((MILLIVOLT_PER_AMP * 4095U + 16499) / 16500 + ZERO_SPREAD) * 32;
            if ((duty_change > 0 && change >= (int32_t)response)
                || (duty_change < 0 && change <= -(int32_t)response)) {
                if (current_tracking < TRACK_CONFIRMATIONS) {
                    current_tracking++;
                }
            } else {
                current_tracking = 0;
            }
            tracking_duty = applied();
            tracking_current = delta;
        } else if (duty + ZERO_SPREAD >= tracking_duty && duty <= tracking_duty + ZERO_SPREAD) {
            /* Finish the previous step's filter tail at the same duty. It
             * cannot count as a second response to a later duty change. */
            tracking_current = delta;
        }
    }
    if (current_tracking < TRACK_CONFIRMATIONS) {
        current_loaded = current_hold = 0;
        return;
    }
    if (raw > current_zero
        && (uint32_t)(raw - current_zero) * 3300 >= LOAD_PEAK_AMPS * MILLIVOLT_PER_AMP * 4095U) {
        current_hold = LOAD_HOLD;
    }
    current_loaded = current_hold || delta * 3300 >=
        (uint32_t)(current_loaded ? LOAD_LEAVE : LOAD_ENTER) * MILLIVOLT_PER_AMP * 4095U * 32 / 100;
#endif // LOAD_CURRENT_SENSE
}

void demag_guard_housekeeping(void)
{
    /* A quiet comparator during a throttle reduction is not clamp evidence.
     * Freeze the preceding duty and speed through the coast. In addition to
     * settling the ramp, require 75% of the previous drive per electrical RPM
     * before observing again: a long coast cannot outlast a fixed timeout.
     * Guard cuts themselves must not disable overload protection. */
    const uint16_t duty = applied();
    if (!running || old_routine || input < 48) {
        drive_hold = 0;
        coasting = 0;
    } else if (input < drive_input || (duty + 4U < drive_previous && duty < demag_cap)) {
        drive_hold = DRIVE_SETTLE;
        coasting = 1;
    } else if (drive_hold) {
        drive_hold--;
    } else if ((uint32_t)duty * commutation_interval * 4 >= (uint32_t)drive_duty * drive_interval * 3) {
        coasting = 0;
    }
    if (!coasting) {
        drive_duty = duty;
        drive_interval = commutation_interval <= 4000 ? commutation_interval : 4000;
    }
    drive_previous = duty;
    drive_input = input;
    /* Current is only an optional widening of this universal proxy. Keep
     * its hysteresis independent: sensor failure cannot close the proxy. */
    const bool stable = armed && running && !old_routine && !stepper_sine && !prop_brake_active
        && input > 47 && zero_crosses > 100 && commutation_interval <= LOAD_MAX_INTERVAL;
    proxy_loaded = stable && duty >= (proxy_loaded ? LOAD_DUTY_LEAVE : LOAD_DUTY_ENTER);
    loaded = proxy_loaded;
#if LOAD_CURRENT_SENSE
    if (current_hold) {
        current_hold--;
    }
    if (current_age) {
        current_age--;
    }
    if (!armed || !current_age) {
        current_zero_valid = zero_samples = current_hold = current_loaded = current_tracking = tracking_samples = 0;
    }
    loaded |= stable && current_loaded;
#endif // LOAD_CURRENT_SENSE
#if DEMAG_GUARD_FORCE_GATE
    loaded = 1; /* bench and simulator tests of an unloaded motor */
#endif // DEMAG_GUARD_FORCE_GATE
    if ((!loaded && input > 47) || !running || old_routine) {
        compensating = 0;
    }
    const uint16_t period = (((uint32_t)written_arr + 1) * 2 + CPU_FREQUENCY_MHZ - 1) / CPU_FREQUENCY_MHZ;
    if (period > pwm_ticks || written_age >= 5) {
        pwm_ticks = period;
    }
    if (full_on && running && !old_routine && !stepper_sine && written_age >= 5) {
        if (full_on_age < 5) {
            full_on_age++;
        }
    } else {
        full_on_age = 0;
    }
    ticks++;
    if (written_age < 255) {
        written_age++;
    }
    if (written_arr != scaled_arr && written_age >= 5) {
        /* a longer period: at least four full ticks after the write, 200 us
         * at a 20 kHz loop, past TIM1's next update even at 8 kHz PWM, so
         * the compare never exceeds the period in effect */
        scaled_arr = written_arr;
        cap_scale = ((uint32_t)written_arr << 16) / 2000;
    }
    if (!armed || !running || old_routine || input < 48) {
        if (demag_active || demag_adv_offset) {
            reset();
        }
        /* not guarding (stopped, polling restart, idle): forget a derating slowly */
        recover_step();
        return;
    }
    recover_step();
    /* release the cap before releasing the advance */
    if (demag_adv_offset && demag_cap == 2000 && (int32_t)(ticks - decay_at) >= 0) {
        demag_adv_offset--;
        decay_at = ticks + ADVANCE_DECAY;
    }
}
#endif // DEMAG_GUARD_ENABLED
