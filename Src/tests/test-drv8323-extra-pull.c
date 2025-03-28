// This example configures a couple
// of registers on the drv8323
// This example also configures extra pullup
// pins attached to the MISO pin in order to
// increase the pullup strength and maximum bitrate

#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "drv8323-spi.h"
#include "mcu.h"
#include "utility-timer.h"

gpio_t gpioExtra1 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA1_PORT, GATE_DRIVER_SPI_MISO_EXTRA1_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra2 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA2_PORT, GATE_DRIVER_SPI_MISO_EXTRA2_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra3 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA3_PORT, GATE_DRIVER_SPI_MISO_EXTRA3_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra4 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA4_PORT, GATE_DRIVER_SPI_MISO_EXTRA4_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra5 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA5_PORT, GATE_DRIVER_SPI_MISO_EXTRA5_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra6 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA6_PORT, GATE_DRIVER_SPI_MISO_EXTRA6_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra7 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA7_PORT, GATE_DRIVER_SPI_MISO_EXTRA7_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra8 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA8_PORT, GATE_DRIVER_SPI_MISO_EXTRA8_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra9 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA9_PORT, GATE_DRIVER_SPI_MISO_EXTRA9_PIN, 0, GPIO_INPUT);
gpio_t gpioExtra10 = DEF_GPIO(GATE_DRIVER_SPI_MISO_EXTRA10_PORT, GATE_DRIVER_SPI_MISO_EXTRA10_PIN, 0, GPIO_INPUT);

void configureExtra(gpio_t* gpio)
{
    gpio_initialize(gpio);
    gpio_configure_pupdr(gpio, GPIO_PULL_UP);
}
int main()
{
    mcu_setup(250);
    utility_timer_initialize();
    utility_timer_enable();
    drv8323_initialize(&DRV8323);

    configureExtra(&gpioExtra1);
    configureExtra(&gpioExtra2);
    configureExtra(&gpioExtra3);
    configureExtra(&gpioExtra4);
    configureExtra(&gpioExtra5);
    configureExtra(&gpioExtra6);
    configureExtra(&gpioExtra7);
    configureExtra(&gpioExtra8);
    configureExtra(&gpioExtra9);
    configureExtra(&gpioExtra10);
    while (!drv8323_write_reg(&DRV8323,
        DRV8323_REG_CSA_CONTROL |
        DRV8323_CSA_VREF_DIV |
        DRV8323_CSA_CSA_GAIN_40VV |
        DRV8323_CSA_SEN_LVL_250mV
    ));

    while (!drv8323_write_reg(&DRV8323,
        DRV8323_REG_OCP_CONTROL |
        DRV8323_OCP_DEADTIME_400ns |
        DRV8323_OCP_DEGLITCH_6us |
        DRV8323_OCP_VDSLVL_600mV
    ));

    while(1) {
    }
}