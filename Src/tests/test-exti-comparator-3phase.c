// this is a test of the comparator.h interface

// #include "stm32h563xx.h"

#include "targets.h"
#include "comparator.h"
#include "exti.h"
// #include "led.h"

// IMR1 reset value is 0xfffe0000

gpio_t gpioCompPhaseA = DEF_GPIO(COMPA_GPIO_PORT, COMPA_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseB = DEF_GPIO(COMPB_GPIO_PORT, COMPB_GPIO_PIN, 0, GPIO_INPUT);
gpio_t gpioCompPhaseC = DEF_GPIO(COMPC_GPIO_PORT, COMPC_GPIO_PIN, 0, GPIO_INPUT);

gpio_t gpioPhaseALed = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);
gpio_t gpioPhaseBLed = DEF_GPIO(LED_G_GPIO_PORT, LED_G_GPIO_PIN, 0, GPIO_OUTPUT);
gpio_t gpioPhaseCLed = DEF_GPIO(LED_B_GPIO_PORT, LED_B_GPIO_PIN, 0, GPIO_OUTPUT);


void phaseA_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        // gpio_set(&gpioPhaseALed);
        EXTI->RPR1 |= mask;
    } 
    if (EXTI->FPR1 & mask) {
        // gpio_reset(&gpioPhaseALed);
        EXTI->FPR1 |= mask;
    }
    // gpio_toggle(&gpioPhaseALed);

    if(gpio_read(&gpioCompPhaseA)) {
        gpio_reset(&gpioPhaseALed);
    } else {
        gpio_set(&gpioPhaseALed);
    }
}

void phaseB_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        // gpio_set(&gpioPhaseBLed);
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        // gpio_reset(&gpioPhaseBLed);
        EXTI->FPR1 |= mask;
    }
    if(gpio_read(&gpioCompPhaseB)) {
        gpio_reset(&gpioPhaseBLed);
    } else {
        gpio_set(&gpioPhaseBLed);
    }
}

void phaseC_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    // EXTI->RPR1 |= mask
    if (EXTI->RPR1 & mask) {
        // gpio_set(&gpioPhaseCLed);
        EXTI->RPR1 |= mask;
    }
    if (EXTI->FPR1 & mask) {
        // gpio_reset(&gpioPhaseCLed);
        EXTI->FPR1 |= mask;
    }
    if(gpio_read(&gpioCompPhaseC)) {
        gpio_reset(&gpioPhaseCLed);
    } else {
        gpio_set(&gpioPhaseCLed);
    }

}


comparator_t comp = {
    .phaseA = &gpioCompPhaseA,
    .phaseB = &gpioCompPhaseB,
    .phaseC = &gpioCompPhaseC,
    .phaseAcb = phaseA_cb,
    .phaseBcb = phaseB_cb,
    .phaseCcb = phaseC_cb
};

int main()
{
    comparator_initialize(&comp);

    gpio_initialize(&gpioPhaseALed);
    gpio_reset(&gpioPhaseALed);


    gpio_initialize(&gpioPhaseBLed);
    gpio_reset(&gpioPhaseBLed);

    gpio_initialize(&gpioPhaseCLed);
    gpio_reset(&gpioPhaseCLed);

    comparator_enable_interrupts(&comp);

    while(1) {
    }
}
