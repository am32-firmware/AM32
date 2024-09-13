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


static comparator_t comp = {
    .phaseA = &gpioCompPhaseA,
    .phaseB = &gpioCompPhaseB,
    .phaseC = &gpioCompPhaseC,
};


void phaseA_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        gpio_set(&gpioPhaseALed);
    } 
    if (EXTI->FPR1 & mask) {
        gpio_reset(&gpioPhaseALed);
    }
    EXTI->RPR1 |= mask;
    EXTI->FPR1 |= mask;
    // gpio_toggle(&gpioPhaseALed);
}

void phaseB_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        gpio_set(&gpioPhaseBLed);
    }
    if (EXTI->FPR1 & mask) {
        gpio_reset(&gpioPhaseBLed);
    }
    EXTI->RPR1 |= mask;
    EXTI->FPR1 |= mask;
}

void phaseC_cb(extiChannel_t* exti)
{
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        gpio_set(&gpioPhaseCLed);
    }
    if (EXTI->FPR1 & mask) {
        gpio_reset(&gpioPhaseCLed);
    }
    EXTI->RPR1 |= mask;
    EXTI->FPR1 |= mask;
}

int main()
{
    exti_configure_port(&extiChannels[gpioCompPhaseA.pin], EXTI_CHANNEL_FROM_PORT(gpioCompPhaseA.port));
    exti_configure_trigger(&extiChannels[gpioCompPhaseA.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioCompPhaseA.pin], phaseA_cb);
    EXTI_NVIC_ENABLE(gpioCompPhaseA.pin);
    // EXTI_INTERRUPT_ENABLE_MASK(1 << gpioCompPhaseA.pin);

    exti_configure_port(&extiChannels[gpioCompPhaseB.pin], EXTI_CHANNEL_FROM_PORT(gpioCompPhaseB.port));
    exti_configure_trigger(&extiChannels[gpioCompPhaseB.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioCompPhaseB.pin], phaseB_cb);
    EXTI_NVIC_ENABLE(gpioCompPhaseB.pin);
    // EXTI_INTERRUPT_ENABLE_MASK(1 << gpioCompPhaseB.pin);

    exti_configure_port(&extiChannels[gpioCompPhaseC.pin], EXTI_CHANNEL_FROM_PORT(gpioCompPhaseC.port));
    exti_configure_trigger(&extiChannels[gpioCompPhaseC.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioCompPhaseC.pin], phaseC_cb);
    EXTI_NVIC_ENABLE(gpioCompPhaseC.pin);
    // EXTI_INTERRUPT_ENABLE_MASK(1 << gpioCompPhaseC.pin);




    gpio_initialize(&gpioCompPhaseA);
    gpio_initialize(&gpioCompPhaseB);
    gpio_initialize(&gpioCompPhaseC);

    gpio_initialize(&gpioPhaseALed);
    gpio_reset(&gpioPhaseALed);


    gpio_initialize(&gpioPhaseBLed);
    gpio_reset(&gpioPhaseBLed);


    gpio_initialize(&gpioPhaseCLed);
    gpio_reset(&gpioPhaseCLed);

    while(1) {
        if(gpio_read(&gpioCompPhaseA)) {
            gpio_reset(&gpioPhaseALed);
        } else {
            gpio_set(&gpioPhaseALed);
        }
        if(gpio_read(&gpioCompPhaseB)) {
            gpio_reset(&gpioPhaseBLed);
        } else {
            gpio_set(&gpioPhaseBLed);
        }
        if(gpio_read(&gpioCompPhaseC)) {
            gpio_reset(&gpioPhaseCLed);
        } else {
            gpio_set(&gpioPhaseCLed);
        }
    }
}

// void compAcb() {

// }


// void EXTI13_IRQHandler() {
//     // EXTI->
//     // FPR1 |= EXTI_FPR1_FPIF13;
//     // EXTI->RPR1 |= EXTI_RPR1_RPIF13;
// }