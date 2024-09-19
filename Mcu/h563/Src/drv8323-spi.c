#include "drv8323-spi.h"
#include "spi.h"


void drv8323_reset(drv8323_t* drv)
{
    drv8323_disable(drv);
    // delay 1ms
    for (int i = 0; i < 0x1ffff; i++) {
        asm("nop");
    }
    drv8323_enable(drv);
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
    spi_initialize(drv->spi);
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
