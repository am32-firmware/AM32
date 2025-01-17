#include "mcu.h"
#include "stm32h563xx.h"
#include "targets.h"
#include "gpio.h"
#include "exti.h"

gpio_t gpioButton = DEF_GPIO(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN, 0, GPIO_INPUT);

gpio_t gpioLed = DEF_GPIO(LED_R_GPIO_PORT, LED_R_GPIO_PIN, 0, GPIO_OUTPUT);


void button_exti_cb(extiChannel_t* exti)
{
    EXTI->RPR1 |= 1 << exti->channel;
    EXTI->FPR1 |= 1 << exti->channel;
    gpio_toggle(&gpioLed);
}

int main()
{
    mcu_setup(250);

    exti_configure_port(&extiChannels[gpioButton.pin], EXTI_CHANNEL_FROM_PORT(gpioButton.port));
    exti_configure_trigger(&extiChannels[gpioButton.pin], EXTI_TRIGGER_RISING_FALLING);
    exti_configure_cb(&extiChannels[gpioButton.pin], button_exti_cb);
    EXTI_NVIC_ENABLE(gpioButton.pin);
    EXTI_INTERRUPT_ENABLE_MASK(1 << gpioButton.pin);

    gpio_initialize(&gpioButton);
    gpio_configure_pupdr(&gpioButton, GPIO_PULL_UP);

    gpio_initialize(&gpioLed);
    gpio_reset(&gpioLed);

    while(1) {
        gpio_toggle(&gpioLed);
        for (int i = 0 ; i < 0xffffff; i++) {
            asm("nop");
        }
    }
}
