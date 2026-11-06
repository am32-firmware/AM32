#include <assert.h>
#include <stdio.h>
#include "../Src/demag_guard.c"

uint32_t timer_count, timer_delay, irq_mask;
unsigned timer_reads, bridge_off, comparator_masks, edge_on_read;
bool timer_enabled, edge_pending, comparator_pre;
uint16_t TIMER1_MAX_ARR = 2665;
volatile char armed = 1, rising = 1;
uint8_t running = 1;
char old_routine, stepper_sine, prop_brake_active, step = 1, forward = 1;
uint16_t input = 1000, minimum_duty_cycle = 40, tim1_arr = 2665, last_duty_cycle = 1000;
volatile uint16_t duty_cycle = 1000, waitTime = 160, thiszctime = 400, lastzctime = 400;
volatile uint32_t commutation_interval = 400, zero_crosses = 1000;
int16_t actual_current;
uint16_t ADC_raw_current;

uint8_t getCompOutputLevel(void) { return comparator_pre ? rising : !rising; }
void maskPhaseInterrupts(void) { comparator_masks++; edge_pending = false; }
void enableCompInterrupts(void) { }
void allOff(void) { bridge_off++; }
bool demag_comparator_pending(void) { return edge_pending; }

static void fresh(void)
{
    reset();
    armed = running = rising = 1;
    old_routine = stepper_sine = prop_brake_active = 0;
    input = last_duty_cycle = duty_cycle = 1000;
    waitTime = 160;
    commutation_interval = thiszctime = lastzctime = 400;
    zero_crosses = 1000;
    timer_count = waitTime;
    timer_enabled = edge_pending = comparator_pre = false;
    ADC_raw_current = 0;
#if LOAD_CURRENT_SENSE
    current_hold = current_loaded = current_tracking = tracking_samples = 0;
    current_age = CURRENT_MAX_AGE;
    duty_average = tracking_duty = tracking_current = 0;
    current_zero_valid = 1;
    current_zero = zero_samples = current_average = 0;
#endif // LOAD_CURRENT_SENSE
    actual_current = 1000;
    loaded = 1;
    proxy_loaded = 0;
    compensating = 0;
    drive_hold = drive_input = drive_duty = drive_interval = drive_previous = coasting = 0;
    ticks = written_age = 10;
    written_arr = scaled_arr = 2665;
    cap_scale = ((uint32_t)written_arr << 16) / 2000;
    pwm_ticks = 84;
    full_on = full_on_age = 0;
    demag_cap = 2000;
    demag_advance_level = 6;
    demag_valid = demag_predicted = demag_warnings = demag_faults = 0;
    demag_late = demag_late_max = demag_desync_request = 0;
    bridge_off = edge_on_read = 0;
}

static void fire(void)
{
    timer_count = t_event;
    assert(demag_guard_timer());
}

static void silent_sector(void)
{
    const unsigned masks = comparator_masks;
    demag_guard_commutated();
    assert(event == EV_BEGIN);
    fire();
    assert(!level); /* The first level sample cannot prove a clamp. */
    fire();
    assert(!level);
    fire();
    assert(level == 2); /* The rail outlasted a PWM period without a selected edge. */
    assert(event == EV_DEADLINE);
    assert(t_dead >= commutation_interval + commutation_interval / 4);
    assert(comparator_masks == masks); /* Observation never clears the EXTI flags the handler relies on. */
}

/* The same interval update and timer rearm as interruptRoutine(). The port
 * calls the edge hook only after the handler's blanking and filter path has run:
 * accepted crossings set pending first; rejected entries record activity. */
static void crossing(void)
{
    edge_pending = false;
    lastzctime = thiszctime;
    thiszctime = timer_count;
    timer_count = 0;
    timer_enabled = true;
    demag_guard_zc();
    demag_guard_edge();
    timer_count = waitTime + 1;
    assert(!demag_guard_timer());
    demag_guard_switched();
}

#if LOAD_CURRENT_SENSE
static uint16_t idle_adc(void)
{
    return (CURRENT_OFFSET * 4095U + 3299) / 3300;
}

static void sample_current(uint16_t raw, unsigned samples)
{
    ADC_raw_current = raw;
    for (unsigned i = 0; i < samples; i++) {
        demag_guard_current_sample();
        demag_guard_housekeeping();
    }
}

static void learn_zero(void)
{
    fresh();
    running = input = last_duty_cycle = 0;
    sample_current(idle_adc(), ZERO_SAMPLES);
    assert(current_zero_valid && !current_loaded && !demag_guard_loaded());
    running = 1;
    input = last_duty_cycle = 150;
}

static void tracked_load(void)
{
    learn_zero();
    /* Two increases in our duty produce two measured current responses,
     * with the duty always below the proxy's 40% enter threshold. */
    for (unsigned i = 0; i < 3; i++) {
        input = last_duty_cycle = 150 + i * 200;
        sample_current(idle_adc() + ((1 + i * 3) * MILLIVOLT_PER_AMP * 4095U + 3299) / 3300, 256);
    }
    assert(current_tracking == TRACK_CONFIRMATIONS && current_loaded);
    assert(demag_guard_loaded() && !proxy_loaded);
}
#endif // LOAD_CURRENT_SENSE

int main(void)
{
    /* The same 40/35% duty gate on every target, even with a missing,
     * uncalibrated, invalid or shared current input and nonsense telemetry. */
    fresh();
    last_duty_cycle = 799;
    ADC_raw_current = 4096;
    actual_current = 30000;
    demag_guard_current_sample();
    demag_guard_housekeeping();
    assert(!demag_guard_loaded());
    timer_reads = comparator_masks = 0;
    demag_guard_commutated();
    assert(!demag_active && !timer_enabled && !comparator_masks);
    assert(!demag_guard_timer() && !timer_reads);
    assert(demag_guard_advance(28) == 28 && demag_guard_duty(1500) == 1500);
    last_duty_cycle = 800;
    demag_guard_housekeeping();
    assert(proxy_loaded && demag_guard_loaded() && !demag_guard_compensate());
    /* A sensor failure cannot close the proxy, even asynchronously. */
    demag_guard_current_sample();
    assert(demag_guard_loaded());
    ADC_raw_current = 0;
    actual_current = 0;
    demag_guard_current_sample();
    demag_guard_housekeeping();
    assert(proxy_loaded && demag_guard_loaded());
    last_duty_cycle = 700;
    demag_guard_housekeeping();
    assert(demag_guard_loaded());
    last_duty_cycle = 699;
    demag_guard_housekeeping();
    assert(!demag_guard_loaded());
    last_duty_cycle = 1000;
    commutation_interval = LOAD_MAX_INTERVAL + 1;
    demag_guard_housekeeping();
    assert(!demag_guard_loaded());
    commutation_interval = LOAD_MAX_INTERVAL;
    demag_guard_housekeeping();
    assert(demag_guard_loaded());
    old_routine = 1;
    demag_guard_housekeeping();
    assert(!demag_guard_loaded());
    old_routine = 0;
    zero_crosses = 100;
    demag_guard_housekeeping();
    assert(!demag_guard_loaded());

#if LOAD_CURRENT_SENSE
    fresh();
    loaded = current_zero_valid = 0;
    input = last_duty_cycle = 300;
    actual_current = 1200;
    sample_current(2044, 200);
    assert(!current_loaded && !demag_guard_loaded());
    /* A stable measured zero must fit the target's transfer function. The
     * original Sequre offset mismatch is rejected, even if ADC is mid-rail. */
    running = input = last_duty_cycle = 0;
    sample_current(2044, ZERO_SAMPLES);
#ifdef TEST_MIDRAIL
    assert(current_zero_valid && current_zero == 2044);
#else
    assert(!current_zero_valid && !demag_guard_loaded());
#endif // TEST_MIDRAIL
    /* Zero telemetry can coexist with a working, validated raw sensor. */
    tracked_load();
    actual_current = 0;
    demag_guard_housekeeping();
    assert(demag_guard_loaded() && !demag_guard_compensate());
    commutation_interval = LOAD_MAX_INTERVAL + 1;
    demag_guard_housekeeping();
    assert(!demag_guard_loaded()); /* Current cannot bypass the speed boundary. */
    commutation_interval = 400;
    demag_guard_housekeeping();
    assert(demag_guard_loaded());
    /* Validate against further demand changes too: flat and inverted
     * responses revoke current widening, without affecting the proxy. */
    input = last_duty_cycle = 750;
    sample_current(ADC_raw_current, 256);
    assert(!current_tracking && !current_loaded && !demag_guard_loaded());
    input = last_duty_cycle = 1000;
    sample_current(idle_adc(), 256);
    assert(!current_tracking && !current_loaded && proxy_loaded && demag_guard_loaded());
    learn_zero();
    for (unsigned i = 0; i < 3; i++) {
        input = last_duty_cycle = 150 + i * 200;
        sample_current(idle_adc() + ((10 - i * 3) * MILLIVOLT_PER_AMP * 4095U + 3299) / 3300, 256);
    }
    assert(!current_tracking && !demag_guard_loaded());
    /* A stuck high input never qualifies across our duty changes. */
    learn_zero();
    for (unsigned i = 0; i < 3; i++) {
        input = last_duty_cycle = 150 + i * 200;
        sample_current(idle_adc() + (8 * MILLIVOLT_PER_AMP * 4095U + 3299) / 3300, 256);
    }
    assert(!current_tracking && !demag_guard_loaded());
    tracked_load();
    ADC_raw_current = 4096;
    demag_guard_current_sample();
    demag_guard_housekeeping();
    assert(!current_zero_valid && !current_loaded && !demag_guard_loaded());
    tracked_load();
    for (unsigned i = 0; i < CURRENT_MAX_AGE; i++) demag_guard_housekeeping();
    assert(!current_zero_valid && !current_loaded && !demag_guard_loaded());
    learn_zero();
    running = input = last_duty_cycle = 0;
    sample_current(idle_adc() + 10, 1);
    assert(!current_zero_valid && !demag_guard_loaded());
    for (unsigned i = 0; i < 200; i++) {
        sample_current(idle_adc() + ((i & 1) ? 0 : 10), 1);
        assert(!current_zero_valid && !demag_guard_loaded());
    }
    sample_current(4096, 1);
    assert(!current_zero_valid && !demag_guard_loaded());
#endif // LOAD_CURRENT_SENSE

    /* With the gate open on a healthy motor, comparator activity keeps every
     * crossing acceptance and interval as the stock path produces them. */
    fresh();
    loaded = 1;
    comparator_masks = 0;
    demag_guard_commutated();
    assert(!comparator_masks && event == EV_BEGIN);
    fire();
    fire(); /* Service CHECK1 before healthy activity, ahead of the deadline. */
    const uint32_t accepted_at = 300;
    timer_count = accepted_at;
    crossing();
    assert(thiszctime == accepted_at && lastzctime == 400 && waitTime == 160);
    assert(!demag_predicted && !demag_warnings && !demag_faults);
    assert(demag_cap == 2000 && !demag_adv_offset && !demag_guard_compensate());
    assert(!comparator_masks);

    fresh();
    demag_guard_commutated();
    timer_count = t_event + LATE_TICKS + 1;
    edge_pending = true;
    assert(demag_guard_timer());
    assert(edge_pending && !timer_enabled && event == EV_NONE);
    assert(!demag_predicted && !demag_warnings);

    fresh();
    assert(!demag_guard_compensate());
    level = 2;
    respond();
    assert(demag_guard_compensate());
    input = 0;
    demag_guard_housekeeping();
    assert(!demag_adv_offset && !demag_guard_loaded() && demag_guard_compensate());
    loaded = 0;
    assert(demag_guard_compensate()); /* Closing the gate cannot shift coast timing. */
    actual_current = 0;
    last_duty_cycle = 0;
    running = 0;
    demag_guard_housekeeping();
    assert(!compensating);

    /* An unarmed opposite edge can return to the pre-crossing level at any
     * observation. Silence of the selected edge alone is not a clamp. */
    for (unsigned checks = 0; checks < 5; checks++) {
        fresh();
        demag_guard_commutated();
        for (unsigned i = 0; i < checks; i++) {
            fire();
        }
        comparator_pre = true;
        const uint32_t frame = t_event;
        const unsigned masks = comparator_masks;
        fire();
        assert(event == EV_NONE && !timer_enabled && !level);
        assert(!demag_predicted && !demag_warnings && timer_count == frame);
        assert(comparator_masks == masks);
    }

    /* Any activity, including chatter that never passes the crossing filter,
     * cancels every possible remaining timer event. */
    for (unsigned checks = 0; checks < 4; checks++) {
        fresh();
        demag_guard_commutated();
        fire();
        for (unsigned i = 0; i < checks; i++) {
            fire();
        }
        const uint32_t frame = timer_count;
        demag_guard_edge();
        assert(activity && !timer_enabled && event == EV_NONE);
        assert(timer_count == frame && !demag_predicted);
        assert(demag_guard_timer()); /* A stale cancelled check cannot switch. */
        timer_count = 620; /* A healthy late rising crossing at 1.55 ci. */
        crossing();
        assert(thiszctime == 620 && !demag_predicted);
        if (checks == 3) {
            assert(!demag_warnings && !demag_adv_offset && demag_cap == 2000);
        }
    }

    fresh();
    demag_guard_housekeeping();
    silent_sector();
    fire();
    assert(event == EV_PREDICT && timer_count == t_dead);
    assert(thiszctime == 400 && lastzctime == 400 && waitTime == 160);
    assert(!demag_predicted && !demag_warnings);
    timer_count = t_event;
    assert(!demag_guard_timer());
    assert(timer_count == (uint32_t)predicted_wait + 1 && pending == PEND_PREDICT);
    assert(!demag_predicted && !demag_warnings); /* Bookkeeping stays after the switch. */
    demag_guard_switched();
    assert(demag_predicted == 1 && demag_warnings == 1 && demag_adv_offset == 1 && demag_cap < 2000);
    demag_guard_housekeeping();
    assert(!coasting); /* Our own protective cut is not a throttle-down. */

    /* A pending edge wins even when the timer ISR runs first. */
    for (unsigned tentative = 0; tentative < 2; tentative++) {
        fresh();
        silent_sector();
        if (tentative) {
            fire();
        }
        const uint32_t frame = t_event;
        edge_pending = true;
        fire();
        assert(timer_count == frame && edge_pending && !timer_enabled);
        assert(!demag_predicted && !demag_warnings && level == 0);
        crossing();
        assert(thiszctime == frame && demag_cap == 2000);
    }

    fresh();
    silent_sector();
    fire();
    timer_count = t_event;
    timer_reads = 0;
    edge_on_read = 2; /* Edge arrives during the final interval-counter read. */
    assert(demag_guard_timer());
    assert(edge_pending && !demag_predicted && timer_count == t_event);
    assert(thiszctime == 400 && lastzctime == 400 && waitTime == 160);

    /* A closing gate must preserve the old frame and the pending crossing. */
    for (unsigned tentative = 0; tentative < 2; tentative++) {
        fresh();
        silent_sector();
        if (tentative) {
            fire();
        }
        const uint32_t frame = t_event;
        loaded = 0;
        edge_pending = true;
        fire();
        assert(timer_count == frame && waitTime == 160 && edge_pending);
        assert(!demag_active && !timer_enabled && !demag_predicted);
    }

    /* Stable rising/falling alternation, with +/-5% ci jitter, cannot feed
     * level-2 cuts at the advance ceiling. Every real crossing repays debt. */
    fresh();
    demag_advance_level = DEMAG_MAX_ADVANCE_LEVEL;
    blind = 5;
    for (unsigned i = 0; i < 30; i++) {
        rising = i & 1;
        demag_active = 1;
        level = 2;
        timer_count = (rising ? 576 : 224) + ((i & 2) ? 20 : -20);
        crossing();
        ticks += CUT_HOLD;
        assert(!interval_growing && demag_cap == 2000);
    }
    assert(!blind && clean >= CLEAN_SECTORS && demag_warnings == 30);
    level = 2;
    timer_count = measured[!!rising] + commutation_interval / 4 + 1;
    crossing();
    assert(interval_growing && demag_cap == 980);

    /* High-speed continuous drive has no PWM edges to wait out. */
    fresh();
    pwm_ticks = 1000;
    demag_guard_commutated();
    fire();
    fire();
    assert(!level);
    full_on = 1;
    full_on_age = 5;
    fire();
    assert(level == 2);
    demag_guard_pwm_commit(100, false);
    assert(!full_on_age);

    fresh();
    pwm_ticks = 1000;
    full_on_age = 5; /* A preempted housekeeping store cannot prove continuous drive. */
    demag_guard_commutated();
    fire();
    fire();
    fire();
    assert(!level);

    fresh();
    full_on_age = 5;
    demag_guard_pwm_period(3000);
    assert(!full_on_age); /* A period preload invalidates continuous-drive proof. */

    fresh();
    silent_sector();
    fire();
    timer_count = t_event + span / 4 + 1;
    assert(demag_guard_timer());
    assert(bridge_off && demag_desync_request && !demag_predicted);

    fresh();
    input = 150;
    last_duty_cycle = duty_cycle = 800;
    level = 3;
    respond();
    assert(demag_warnings == 1 && demag_cap == 2000 && demag_adv_offset == 0);

    /* Six blind sectors are allowed; the seventh switches the bridge off. */
    fresh();
    for (unsigned i = 0; i <= BLIND_SECTORS; i++) {
        last_duty_cycle = duty_cycle = 1000;
        timer_count = waitTime;
        silent_sector();
        fire();
        timer_count = t_event;
        if (i == BLIND_SECTORS) {
            assert(demag_guard_timer());
            assert(demag_faults == 1 && bridge_off == 1 && demag_desync_request);
        } else {
            assert(!demag_guard_timer());
            demag_guard_switched();
            ticks += CUT_HOLD;
        }
    }
    assert(demag_predicted == BLIND_SECTORS);
    /* A down step cancels a pending prediction without clearing an edge or
     * changing the measured frame; the quiet coast stays unobservable. */
    fresh();
    demag_guard_housekeeping();
    silent_sector();
    fire();
    input = 300;
    timer_count = t_event;
    const uint32_t frame = timer_count;
    const unsigned masks = comparator_masks;
    assert(demag_guard_timer());
    assert(!demag_active && !demag_predicted && timer_count == frame);
    assert(comparator_masks == masks);
    demag_guard_housekeeping();
    last_duty_cycle = duty_cycle = 300;
    demag_guard_housekeeping();
    for (unsigned i = 1; i < DRIVE_SETTLE; i++) {
        demag_guard_housekeeping();
        loaded = 1;
        timer_count = waitTime;
        demag_guard_commutated();
        assert(!demag_active && !demag_predicted);
    }
    /* Even after 100 ms, duty far below that required at the old speed
     * cannot prove a clamp. Resume only after electrical speed has fallen. */
    for (unsigned i = 0; i < DRIVE_SETTLE; i++) {
        demag_guard_housekeeping();
        loaded = 1;
        timer_count = waitTime;
        demag_guard_commutated();
        assert(!demag_active && !demag_predicted);
    }
    commutation_interval = 1200;
    demag_guard_housekeeping();
    loaded = 1;
    timer_count = waitTime;
    demag_guard_commutated();
    assert(demag_active);

    /* Expected edges before the first observation belong to the stock handler. */
    fresh();
    comparator_masks = 0;
    demag_guard_commutated();
    assert(!comparator_masks);
    edge_pending = true;
    fire();
    assert(edge_pending && !comparator_masks && event == EV_NONE);
    fresh();
    demag_guard_commutated();
    demag_guard_edge();
    assert(!activity && event == EV_BEGIN && timer_enabled);
    assert(!comparator_masks);
    puts("demag guard transition tests passed");
    return 0;
}
