// #include "phaseouts.h"
#include "clock.h"
#include "targets.h"
#include "gpio.h"
#include "bridge.h"
#include "mcu.h"

// gpio_t gpioPhaseAHigh = DEF_GPIO(
//     PHASE_A_GPIO_PORT_HIGH,
//     PHASE_A_GPIO_HIGH_PIN,
//     PHASE_A_HIGH_AF,
//     GPIO_AF
// );

// gpio_t gpioPhaseALow = DEF_GPIO(
//     PHASE_A_GPIO_PORT_LOW,
//     PHASE_A_GPIO_LOW_PIN,
//     PHASE_A_LOW_AF,
//     GPIO_AF
// );

// gpio_t gpioPhaseBHigh = DEF_GPIO(
//     PHASE_B_GPIO_PORT_HIGH,
//     PHASE_B_GPIO_HIGH_PIN,
//     PHASE_B_HIGH_AF,
//     GPIO_AF
// );

// gpio_t gpioPhaseBLow = DEF_GPIO(
//     PHASE_B_GPIO_PORT_LOW,
//     PHASE_B_GPIO_LOW_PIN,
//     PHASE_B_LOW_AF,
//     GPIO_AF
// );

// gpio_t gpioPhaseCHigh = DEF_GPIO(
//     PHASE_C_GPIO_PORT_HIGH,
//     PHASE_C_GPIO_HIGH_PIN,
//     PHASE_C_HIGH_AF,
//     GPIO_AF
// );

// gpio_t gpioPhaseCLow = DEF_GPIO(
//     PHASE_C_GPIO_PORT_LOW,
//     PHASE_C_GPIO_LOW_PIN,
//     PHASE_C_LOW_AF,
//     GPIO_AF
// );

int main()
{
    mcu_setup();

    bridge_initialize();

    gpio_t gpioDrv8323nFault = DEF_GPIO(
        DRV_FAULT_PORT,
        DRV_FAULT_PIN,
        0,
        GPIO_INPUT);
    gpio_initialize(&gpioDrv8323nFault);
    gpio_configure_pupdr(&gpioDrv8323nFault, GPIO_PULL_UP);


    gpio_t gpioDrv8323Cal = DEF_GPIO(
        DRV_CAL_PORT,
        DRV_CAL_PIN,
        0,
        GPIO_OUTPUT);
    gpio_initialize(&gpioDrv8323Cal);
    gpio_reset(&gpioDrv8323Cal);


    gpio_t gpioDrv8323Enable = DEF_GPIO(
        DRV_ENABLE_PORT,
        DRV_ENABLE_PIN,
        0,
        GPIO_OUTPUT);
    // gpio_t gpioDrv8323Enable = DEF_GPIO( // nucleo
    // GPIOF,
    // 0,
    // 0,
    // GPIO_OUTPUT);
    gpio_set_speed(&gpioDrv8323Enable, 0b11);
    gpio_reset(&gpioDrv8323Enable);
    for (uint32_t i = 0; i < 25000000; i++) {
        asm("nop");
    }
    gpio_set(&gpioDrv8323Enable);

    for (uint32_t i = 0; i < 25000000; i++) {
        asm("nop");
    }
    // gpio_initialize(&gpioPhaseAHigh);
    // gpio_initialize(&gpioPhaseALow);
    // gpio_initialize(&gpioPhaseBHigh);
    // gpio_initialize(&gpioPhaseBLow);
    // gpio_initialize(&gpioPhaseCHigh);
    // gpio_initialize(&gpioPhaseCLow);

    bridge_set_mode_audio();
    bridge_set_audio_frequency(2000);
    // bridge_set_audio_duty(0x80);
    // bridge_set_audio_duty(0x40);
    bridge_set_audio_duty(0x0f);
    bridge_enable();

    for (uint32_t i = 0; i < 25000000; i++)
    {
        // if (!gpio_read(&gpioDrv8323nFault)) {
        //     for (;;);
        // }
    }

    bridge_disable();
    while(1) {
    }
}