// #include "stm32h563xx.h"

// This test configures gpio to turn on
// the 5V regulator, and to control the MCP2542
// CAN transceiver standby mode pin (STBY)
// The example also toggles the CAN TX pin
// to verify the can tranceiver functionality with
// an oscilloscope
// the cpu core frequency can be changed to change
// the 'bitrate'
#include "targets.h"
#include "vreg.h"

#include "gpio.h"
#include "mcu.h"

int main()
{
    mcu_setup(24);
    vreg5V_initialize();
    vreg5V_enable();

    gpio_t gpioMcp2542Enable = DEF_GPIO(
        CAN_STBY_GPIO_PORT,
        CAN_STBY_GPIO_PIN,
        0,
        GPIO_OUTPUT
    );
    gpio_initialize(&gpioMcp2542Enable);
    // drive standby (STBY) pin low to bring the device
    // out of standby
    gpio_reset(&gpioMcp2542Enable);

    gpio_t gpioCanTx = DEF_GPIO(
        CAN_TX_GPIO_PORT,
        CAN_TX_GPIO_PIN,
        CAN_TX_GPIO_AF,
        GPIO_OUTPUT
    );

    gpio_initialize(&gpioCanTx);

    gpio_set_speed(&gpioCanTx, GPIO_SPEED_VERYFAST);

    while(1) {
        CAN_TX_GPIO_PORT->BSRR |= 1<<CAN_TX_GPIO_PIN;
        CAN_TX_GPIO_PORT->BRR |= 1<<CAN_TX_GPIO_PIN;
        // gpio_set(&gpioCanTx);
        // gpio_reset(&gpioCanTx);
    }
}