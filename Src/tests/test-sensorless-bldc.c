// This example reads the as5048 using
// the as5048-spi.h driver
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "blanking.h"
#include "bridge.h"
#include "commutation-timer.h"
#include "comparator.h"
#include "comp-timer.h"
#include "debug.h"
#include "drv8323-spi.h"
#include "functions.h"
#include "math.h"
#include "mcu.h"
#include "stm32h563xx.h"
#include "utility-timer.h"
#include "watchdog.h"


gpio_t gpioCompPhaseATest = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseBTest = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseCTest = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);


comparator_t comp = {
    .phaseA = &gpioCompPhaseATest,
    .phaseB = &gpioCompPhaseBTest,
    .phaseC = &gpioCompPhaseCTest,
    .phaseAcb = 0,
    // .phaseBcb = phaseBTestcb,
    .phaseBcb = 0,
    // .phaseCcb = phaseCTestcb,
    .phaseCcb = 0,
};


void commutation_timer_interrupt_handler()
{
    if (COM_TIMER->SR & TIM_SR_CC1IF) {
        comparator_disable_interrupts(&comp);
        blanking_enable();
        comp_timer_enable();
        bridge_commutate();
        debug_toggle_2();
        COM_TIMER->SR &= ~TIM_SR_CC1IF;
        watchdog_reload();
    }
}

// aka the period
uint32_t compA_prev_rising_time;
uint32_t compA_rising_time;
uint32_t compA_falling_time;
uint32_t compA_duty;

uint32_t compB_prev_rising_time;
uint32_t compB_rising_time;
uint32_t compB_falling_time;
uint32_t compB_duty;

uint32_t compC_prev_rising_time;
uint32_t compC_rising_time;
uint32_t compC_falling_time;
uint32_t compC_duty;

#define COMP_TIM_CNT_VALID 1500
#define COMP_DUTY_THRESHOLD 100
#define COMP_DUTY_THRESHOLD_RISING (500 + COMP_DUTY_THRESHOLD)
#define COMP_DUTY_THRESHOLD_FALLING (500 - COMP_DUTY_THRESHOLD)
// uint32_t comp_period, comp_duty;
void phaseARisingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compA_prev_rising_time = compA_rising_time;
        compA_rising_time = cnt;
        if (compA_prev_rising_time > 0 && compA_rising_time > compA_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compA_duty = compA_falling_time * 1000 / compA_rising_time;
            if (compA_duty > COMP_DUTY_THRESHOLD_RISING && cnt > COMP_TIM_CNT_VALID) {
                debug_toggle_2();
                COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                commutation_timer_enable();
                comparator_disable_interrupts(&comp);
            }
        }

    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compA_falling_time = cnt;
    }
}

void phaseAFallingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compA_prev_rising_time = compA_rising_time;
        compA_rising_time = cnt;
        if (compA_prev_rising_time > 0 && compA_rising_time > compA_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compA_duty = compA_falling_time * 1000 / compA_rising_time;
            if (compA_duty < COMP_DUTY_THRESHOLD_FALLING && cnt > COMP_TIM_CNT_VALID) {
                debug_toggle_2();
                COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                commutation_timer_enable();
                comparator_disable_interrupts(&comp);
            }
        }
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compA_falling_time = cnt;
    }
}



// uint32_t comp_period, comp_duty;
void phaseBRisingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compB_prev_rising_time = compB_rising_time;
        compB_rising_time = cnt;
        if (compB_prev_rising_time > 0 && compB_rising_time > compB_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compB_duty = compB_falling_time * 1000 / compB_rising_time;
            if (compB_duty > COMP_DUTY_THRESHOLD_RISING && cnt > COMP_TIM_CNT_VALID) {
                debug_toggle_2();
                COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                commutation_timer_enable();
                comparator_disable_interrupts(&comp);
            }
        }

    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compB_falling_time = cnt;
    }
}

void phaseBFallingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compB_prev_rising_time = compB_rising_time;
        compB_rising_time = cnt;
        if (compB_prev_rising_time > 0 && compB_rising_time > compB_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compB_duty = compB_falling_time * 1000 / compB_rising_time;
            if (compB_duty < COMP_DUTY_THRESHOLD_FALLING && cnt > COMP_TIM_CNT_VALID) {
                    debug_toggle_2();
                    COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                    commutation_timer_enable();
                    comparator_disable_interrupts(&comp);
            }
        }
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compB_falling_time = cnt;
    }
}




// uint32_t comp_period, comp_duty;
void phaseCRisingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compC_prev_rising_time = compC_rising_time;
        compC_rising_time = cnt;
        if (compC_prev_rising_time > 0 && compC_rising_time > compC_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compC_duty = compC_falling_time * 1000 / compC_rising_time;
            if (compC_duty > COMP_DUTY_THRESHOLD_RISING && cnt > COMP_TIM_CNT_VALID) {
                debug_toggle_2();
                COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                commutation_timer_enable();
                comparator_disable_interrupts(&comp);
            }
        }

    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compC_falling_time = cnt;
    }
}

void phaseCFallingCb(extiChannel_t* exti)
{
    uint32_t cnt = COMP_TIMER->CNT;
    uint32_t comCnt = COM_TIMER->CNT;
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        comp_timer_enable();
        debug_set_1();
        EXTI->RPR1 |= mask;
        compC_prev_rising_time = compC_rising_time;
        compC_rising_time = cnt;
        if (compC_prev_rising_time > 0 && compC_rising_time > compC_falling_time) { // somehow this is not always the case TODO figure out why and take this out
            // this gives ~17ms of period available (keep period < 17ms)
            compC_duty = compC_falling_time * 1000 / compC_rising_time;
            if (compC_duty < COMP_DUTY_THRESHOLD_FALLING && cnt > COMP_TIM_CNT_VALID) {
                    debug_toggle_2();
                    COM_TIMER->CCR1 = comCnt/2 - comCnt/8;
                    commutation_timer_enable();
                    comparator_disable_interrupts(&comp);
            }
        }
    }
    if (EXTI->FPR1 & mask) {
        debug_reset_1();
        EXTI->FPR1 |= mask;
        compC_falling_time = cnt;
    }
}

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

        comp.phaseAcb = 0;
        comp.phaseBcb = 0;
        comp.phaseCcb = 0;

        switch (bridgeComStep) {
            case 0:
                comp.phaseAcb = phaseARisingCb;
                compA_rising_time = 0;
                compA_falling_time = 1<<31;
                break;
            case 1:
                comp.phaseCcb = phaseCFallingCb;
                compC_rising_time = 0;
                compC_falling_time = 1<<31;
                break;
            case 2:
                comp.phaseBcb = phaseBRisingCb;
                compB_rising_time = 0;
                compB_falling_time = 1<<31;
                break;
            case 3:
                comp.phaseAcb = phaseAFallingCb;
                compA_rising_time = 0;
                compA_falling_time = 1<<31;
                break;
            case 4:
                comp.phaseCcb = phaseCRisingCb;
                compC_rising_time = 0;
                compC_falling_time = 1<<31;
                break;
            case 5:
                comp.phaseBcb = phaseBFallingCb;
                compB_rising_time = 0;
                compB_falling_time = 1<<31;
                break;
            default:
                comp.phaseAcb = 0;
                comp.phaseBcb = 0;
                comp.phaseCcb = 0;
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

    commutation_timer_initialize();
    // commutation_timer_interrupt_enable();
    comp_timer_initialize();

    blanking_initialize();

    utility_timer_initialize();
    utility_timer_enable();

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

    comparator_initialize(&comp);

    // set a low priority on comparator interrupt
    // this is necessary for this example
    // to use the sk6812 led spi interrupt
    comparator_nvic_set_priority(&comp, 1);

    bridge_set_run_duty(0x0100);

    comp_timer_enable();
    blanking_enable();
    bridge_commutate();

    // COM_TIMER->SR &= ~TIM_SR_CC1IF;
    COM_TIMER->CCR1 = 100000;
    commutation_timer_interrupt_enable();

    while(1) {
    }
}