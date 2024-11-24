// #include "stm32h563xx.h"

// This test configures gpio to turn on
// the 5V regulator, and to control the MCP2542
// CAN transceiver standby mode pin (STBY)
#include "targets.h"
#include "vreg.h"

#include "gpio.h"


int main()
{

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


    gpio_t gpioCanRx = DEF_GPIO(
        CAN_RX_GPIO_PORT,
        CAN_RX_GPIO_PIN,
        CAN_RX_GPIO_AF,
        GPIO_AF
    );

    gpio_t gpioCanTx = DEF_GPIO(
        CAN_TX_GPIO_PORT,
        CAN_TX_GPIO_PIN,
        CAN_TX_GPIO_AF,
        GPIO_AF
    );

    gpio_initialize(&gpioCanRx);
    gpio_initialize(&gpioCanTx);


    while(1) {

    }
}