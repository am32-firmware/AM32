// This example reads the as5048 using
// the as5048-spi.h driver
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "blanking.h"
#include "bridge.h"
#include "comparator.h"
#include "comp-timer.h"
#include "debug.h"
#include "drv8323-spi.h"
#include "functions.h"
#include "math.h"
#include "mcu.h"
#include "utility-timer.h"
#include "watchdog.h"

as5048_t as5048;

uint16_t current_angle;

#define MAGNET_ANGLES_MAX 100
uint16_t magnet_angles[MAGNET_ANGLES_MAX];
uint16_t zc_angles[MAGNET_ANGLES_MAX];
uint16_t pole_index;

#define WAIT_MS 65 // 65 ms is max


gpio_t gpioCompPhaseATest = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseBTest = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseCTest = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);


// aka the period
uint32_t compA_rising_time;
uint32_t compA_falling_time;
uint32_t compA_duty;

// uint32_t comp_period, comp_duty;
void phaseATestcb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compA_rising_time = cnt;
        // this gives ~17ms of period available (keep period < 17ms)
        compA_duty = compA_falling_time * 1000 / compA_rising_time;
        // if (compA_duty > 650) {
        if (compA_duty > 600 && cnt > 3300) {
        // if (compA_duty > 650 && cnt > 7500) {
                debug_toggle_2();
        }
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compA_falling_time = cnt;
    }
    // if(gpio_read(&gpioCompPhaseATest)) {
    //     debug_set_1();
    // } else {
    //     debug_reset_1();
    // }
}

void phaseAFallingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compA_rising_time = cnt;
        // this gives ~17ms of period available (keep period < 17ms)
        compA_duty = compA_falling_time * 1000 / compA_rising_time;
        // if (compA_duty > 650) {
        if (compA_duty < 400 && cnt > 3300) {
        // if (compA_duty > 650 && cnt > 7500) {
                debug_toggle_2();
        }
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compA_falling_time = cnt;
    }
    // if(gpio_read(&gpioCompPhaseATest)) {
    //     debug_set_1();
    // } else {
    //     debug_reset_1();
    // }
}

void phaseBTestcb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        debug_set_2();
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_2();
        EXTI->FPR1 |= mask;
    }
    // if(gpio_read(&gpioCompPhaseBTest)) {
    //     debug_set_2();
    // } else {
    //     debug_reset_2();
    // }
}

void phaseCTestcb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        debug_set_2();
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_2();
        EXTI->FPR1 |= mask;
    }
}


comparator_t comp = {
    .phaseA = &gpioCompPhaseATest,
    .phaseB = &gpioCompPhaseBTest,
    .phaseC = &gpioCompPhaseCTest,
    .phaseAcb = phaseATestcb,
    // .phaseBcb = phaseBTestcb,
    .phaseBcb = 0,
    // .phaseCcb = phaseCTestcb,
    .phaseCcb = 0,
};

void bridge_timer_irq_handler()
{
    if (BRIDGE_TIMER->SR & TIM_SR_CC4IF) {
        BRIDGE_TIMER->SR &= ~TIM_SR_CC4IF;

        // debug_toggle_3();
        // switch (bridgeComStep) {
        //     case 1:
        //     case 2:
        //     case 4:
        //     case 5:
        //         break;
        //     case 3:
        //         for (int i = 0; i < 10; i ++) {
        //             if (gpio_read(&gpioCompPhaseATest)) {
        //                 goto leave3;
        //             }
        //         }
        //         debug_reset_2();
        //         // if (!gpio_read(&gpioCompPhaseATest)) {
        //         //     debug_reset_2();
        //         // }
        //         leave3:
        //         break;
        //     case 0:
        //         for (int i = 0; i < 10; i ++) {
        //             if (!gpio_read(&gpioCompPhaseATest)) {
        //                 goto leave0;
        //             }
        //         }
        //         debug_set_2();
        //         // if (gpio_read(&gpioCompPhaseATest)) {
        //         //     debug_set_2();
        //         // }
        //         leave0:
        //         break;

        //     default:
        //         while(1);
        // }
    }
}

void blanking_interrupt_handler()
{
    if (BLANKING_TIMER->SR & TIM_SR_CC1IF) {
        BLANKING_TIMER->SR &= ~TIM_SR_CC1IF;
        blanking_disable();
        // debug_toggle_3();

        switch (bridgeComStep) {
            case 0:
                comp.phaseAcb = phaseATestcb;
                break;
            case 3:
                comp.phaseAcb = phaseAFallingCb;
                break;
            default:
                comp.phaseAcb = 0;
                break;

        }
        comparator_configure_callbacks(&comp);
        comp_timer_enable();
        comparator_enable_interrupts(&comp);
        // bridge_sample_interrupt_enable();
    }
}

int main()
{
    mcu_setup(250);

    debug_initialize();

    comp_timer_initialize();

    blanking_initialize();

    utility_timer_initialize();
    utility_timer_enable();

    as5048_initialize(&as5048);
    drv8323_initialize(&DRV8323);

    watchdog_initialize_period(500);
    watchdog_enable();

    bridge_initialize();
    bridge_set_deadtime_ns(0);
    bridge_set_mode_run();
    bridge_set_run_frequency(24000);
    bridge_set_run_duty(0x0300);
    // bridge_sample_interrupt_enable();
    bridge_enable();
    bridge_commutate();
    // wait extra for first step because it may be up to 3 steps out of alignment
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    delayMillis(WAIT_MS);
    as5048_set_zero_position(&as5048);


    do {
        current_angle = as5048_read_angle(&as5048);
    } while (current_angle != 0);


    pole_index = 0;
    magnet_angles[pole_index++] = current_angle;


    uint16_t i = 0;
    do {
        watchdog_reload();
        bridge_commutate();
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);


        uint16_t last_angle = current_angle;
        current_angle = as5048_read_angle(&as5048);
        magnet_angles[pole_index++] = current_angle;
        debug_write_string("\n\rstep: ");
        debug_write_int(pole_index);
        debug_write_string(" angle: ");
        debug_write_int(current_angle);
        debug_write_string(" diff: ");
        debug_write_int(current_angle - last_angle);


    } while (current_angle < 16353  && current_angle > 30);
    watchdog_reload();

    // bridge_disable();

    debug_write_string("\n\rcurrent_angle: ");
    debug_write_int(current_angle);

    uint16_t num_poles = pole_index - 1;
    magnet_angles[num_poles] = 1<<14;

    comparator_initialize(&comp);

    // set a low priority on comparator interrupt
    // this is necessary for this example
    // to use the sk6812 led spi interrupt
    comparator_nvic_set_priority(&comp, 1);

    comparator_enable_interrupts(&comp);

    for (int i = 0; i < num_poles; i++) {
        zc_angles[i] = magnet_angles[i];

        if (i < 3 || i > num_poles - 3) {
            debug_write_string("\n\rindex: ");
            debug_write_int(i);
            debug_write_string("\tmagnet_angle: ");
            debug_write_int(magnet_angles[i]);
            debug_write_string("\tzc_angle: ");
            debug_write_int(zc_angles[i]);
            delayMillis(5);
        }

    }

    watchdog_reload();

    for (int i = 0; i < num_poles; i++) {
        if (i == num_poles - 1) {
            zc_angles[i] = (1<<14);
        } else {
            // zc_angles[i] = magnet_angles[i] - 20;
            zc_angles[i] = zc_angles[i+1];
        }


        if (i < 3 || i > num_poles - 3) {
            debug_write_string("\n\rindex: ");
            debug_write_int(i);
            debug_write_string("\tmagnet_angle: ");
            debug_write_int(magnet_angles[i]);
            debug_write_string("\tzc_angle: ");
            debug_write_int(zc_angles[i]);
            delayMillis(5);
        }
    }

    watchdog_reload();


    // apply advance
    for (int i = 0; i < num_poles; i++) {
        uint16_t diff;
        if (i == 0) {
            diff = zc_angles[0];
        } else {
            diff = zc_angles[i] - zc_angles[i - 1];
        }
        zc_angles[i] -= round(diff / 4.0f);
        // zc_angles[i] -= diff;

        if (i < 3 || i > num_poles - 3) {
            debug_write_string("\n\rindex: ");
            debug_write_int(i);
            debug_write_string("\tmagnet_angle: ");
            debug_write_int(magnet_angles[i]);
            debug_write_string("\tzc_angle: ");
            debug_write_int(zc_angles[i]);
            debug_write_string("\tdiff: ");
            debug_write_int(diff);
            delayMillis(5);
        }
    }

    watchdog_reload();

    bridge_set_run_duty(0x0500);

    // here we are at angle = 0

    // debug_set_3();
    bridge_commutate();
    // for (int n = 0; n < 50; n++) {
    while (1) {

        do {
            current_angle = as5048_read_angle(&as5048);
        } while (current_angle > magnet_angles[num_poles - 2]);
        // delayMicros(10);
        for (int i = 0; i < num_poles; i++) {
            // bridge_sample_interrupt_disable();
            comparator_disable_interrupts(&comp);
            blanking_enable();
            bridge_commutate();
            debug_toggle_3();
            watchdog_reload();
            do {
                current_angle = as5048_read_angle(&as5048);
            } while (current_angle < zc_angles[i]);

            // debug_toggle_2();
        }
    }

    bridge_disable();
    while(1) {
        watchdog_reload();
        current_angle = as5048_read_angle(&as5048);
    }
}