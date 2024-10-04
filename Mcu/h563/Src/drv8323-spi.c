#include "drv8323-spi.h"
#include "spi.h"
#include "functions.h"


gpio_t gpioDrv8323Enable = DEF_GPIO(
    DRV_ENABLE_PORT,
    DRV_ENABLE_PIN,
    0,
    GPIO_OUTPUT);

drv8323_t DRV8323 = {
    .gpioEnable = &gpioDrv8323Enable,
    // .gpioFault = gpioFault
    .gpioFault = 0,
    .spi = 0,
};

void drv8323_reset(drv8323_t* drv)
{
    delayMillis(2);
    drv8323_disable(drv);
    // delay at least 1ms
    delayMillis(2);
    drv8323_enable(drv);
    // delayMicros(3000);
    delayMillis(2);
}

void drv_initialize_gpio(drv8323_t* drv)
{
    if (drv->gpioEnable) {
        gpio_initialize(drv->gpioEnable);
    }
    if (drv->gpioFault) {
        gpio_initialize(drv->gpioFault);
    }
}

void drv8323_disable(drv8323_t* drv)
{
    gpio_reset(drv->gpioEnable);
}

void drv8323_enable(drv8323_t* drv)
{
    gpio_set(drv->gpioEnable);
}

void drv8323_initialize(drv8323_t* drv)
{
    if (drv->spi) {
        spi_initialize(drv->spi);
    }
    drv_initialize_gpio(drv);
    drv8323_reset(drv);
}

bool drv8323_write_reg(drv8323_t* drv, uint16_t word)
{
    drv8323_spi_write_word(drv, word);
    return (drv8323_read_reg(drv, word) & DRV8323_FRAME_DATA_MASK) == (word & DRV8323_FRAME_DATA_MASK);
}

uint16_t drv8323_read_reg(drv8323_t* drv, uint16_t word)
{
    word |= DRV8323_READ;
    word &= ~DRV8323_FRAME_DATA_MASK;
    return drv8323_spi_write_word(drv, word);
}

uint16_t drv8323_spi_write_word(drv8323_t* drv, uint16_t word)
{
    return spi_write_word(drv->spi, word);
}
