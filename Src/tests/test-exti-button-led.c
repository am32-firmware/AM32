#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "exti.h"

gpio_t gpioButton = DEF_GPIO(GPIOC, 13, 0, GPIO_INPUT);

gpio_t gpioLed = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);


void button_exti_cb(extiChannel_t* exti)
{
    EXTI->RPR1 |= 1 << exti->channel;
    EXTI->FPR1 |= 1 << exti->channel;
    gpio_toggle(&gpioLed);
}

int main()
{
    exti_configure_port(&extiChannels[gpioButton.pin], EXTI_CHANNEL_FROM_PORT(gpioButton.port));
    exti_configure_trigger(&extiChannels[gpioButton.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioButton.pin], button_exti_cb);
    EXTI_NVIC_ENABLE(gpioButton.pin);
    EXTI_INTERRUPT_ENABLE_MASK(1 << gpioButton.pin);

    gpio_initialize(&gpioButton);

    gpio_initialize(&gpioLed);
    gpio_reset(&gpioLed);

    while(1) {

    }
}
