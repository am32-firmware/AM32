// #include "stm32h563xx.h"

#include "targets.h"
#include "comparator.h"
#include "exti.h"
// #include "led.h"


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
    EXTI->RPR1 |= 1 << exti->channel;
    EXTI->FPR1 |= 1 << exti->channel;
    gpio_toggle(&gpioPhaseALed);
}

void phaseB_cb(extiChannel_t* exti)
{
    EXTI->RPR1 |= 1 << exti->channel;
    EXTI->FPR1 |= 1 << exti->channel;
    gpio_toggle(&gpioPhaseBLed);
}

void phaseC_cb(extiChannel_t* exti)
{
    EXTI->RPR1 |= 1 << exti->channel;
    EXTI->FPR1 |= 1 << exti->channel;
    gpio_toggle(&gpioPhaseCLed);
}

int main()
{
    exti_configure_port(&extiChannels[gpioCompPhaseA.pin], EXTI_CHANNEL_FROM_PORT(gpioCompPhaseA.port));
    exti_configure_trigger(&extiChannels[gpioCompPhaseA.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioCompPhaseA.pin], phaseA_cb);
    EXTI_NVIC_ENABLE(gpioCompPhaseA.pin);
    EXTI_INTERRUPT_ENABLE_MASK(1 << gpioCompPhaseA.pin);

    gpio_initialize(&gpioCompPhaseA);

    gpio_initialize(&gpioPhaseALed);
    gpio_reset(&gpioPhaseALed);

    while(1) {

    }
}

// void compAcb() {

// }


// void EXTI13_IRQHandler() {
//     // EXTI->
//     // FPR1 |= EXTI_FPR1_FPIF13;
//     // EXTI->RPR1 |= EXTI_RPR1_RPIF13;
// }