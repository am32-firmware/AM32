// #include "phaseouts.h"
#include "targets.h"
#include "gpio.h"
#include "bridge.h"
char comp_pwm = 1;
uint8_t i = 0;

gpio_t gpioPhaseAHigh = DEF_GPIO(
    PHASE_A_GPIO_PORT_HIGH,
    PHASE_A_GPIO_HIGH_PIN,
    PHASE_A_HIGH_AF,
    GPIO_AF
);

gpio_t gpioPhaseALow = DEF_GPIO(
    PHASE_A_GPIO_PORT_LOW,
    PHASE_A_GPIO_LOW_PIN,
    PHASE_A_LOW_AF,
    GPIO_AF
);

gpio_t gpioPhaseBHigh = DEF_GPIO(
    PHASE_B_GPIO_PORT_HIGH,
    PHASE_B_GPIO_HIGH_PIN,
    PHASE_B_HIGH_AF,
    GPIO_AF
);

gpio_t gpioPhaseBLow = DEF_GPIO(
    PHASE_B_GPIO_PORT_LOW,
    PHASE_B_GPIO_LOW_PIN,
    PHASE_B_LOW_AF,
    GPIO_AF
);

gpio_t gpioPhaseCHigh = DEF_GPIO(
    PHASE_C_GPIO_PORT_HIGH,
    PHASE_C_GPIO_HIGH_PIN,
    PHASE_C_HIGH_AF,
    GPIO_AF
);

gpio_t gpioPhaseCLow = DEF_GPIO(
    PHASE_C_GPIO_PORT_LOW,
    PHASE_C_GPIO_LOW_PIN,
    PHASE_C_LOW_AF,
    GPIO_AF
);

int main()
{

    gpio_t gpioDrv8323Enable = DEF_GPIO(
        GPIOF,
        0,
        0,
        GPIO_OUTPUT);
    gpio_initialize(&gpioDrv8323Enable);
    gpio_set_speed(&gpioDrv8323Enable, 0b11);
    gpio_reset(&gpioDrv8323Enable);
    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    gpio_set(&gpioDrv8323Enable);

    for (int i = 0; i < 0xfffff; i++) {
        asm("nop");
    }
    // gpio_initialize(&gpioPhaseAHigh);
    // gpio_initialize(&gpioPhaseALow);
    // gpio_initialize(&gpioPhaseBHigh);
    // gpio_initialize(&gpioPhaseBLow);
    // gpio_initialize(&gpioPhaseCHigh);
    // gpio_initialize(&gpioPhaseCLow);

    bridge_initialize();
    bridge_set_mode_run();
    // bridge_set_run_frequency(16000);
    bridge_set_run_duty(0x8000);
    bridge_enable();



    while(1) {
        for (uint32_t i = 0; i < 32000; i++)
        {
            asm("nop");
        }
        bridge_commutate();
    }
}