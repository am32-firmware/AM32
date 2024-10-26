#include "drv8323-spi.h"
#include "exti.h"
#include "spi.h"
#include "functions.h"


gpio_t gpioDrv8323Enable = DEF_GPIO(
    DRV_ENABLE_PORT,
    DRV_ENABLE_PIN,
    0,
    GPIO_OUTPUT);

gpio_t gpioDrv8323nFault = DEF_GPIO(
    DRV_FAULT_PORT,
    DRV_FAULT_PIN,
    0,
    GPIO_INPUT);

gpio_t gpioDrv8323Cal = DEF_GPIO(
    DRV_CAL_PORT,
    DRV_CAL_PIN,
    0,
    GPIO_OUTPUT);


drv8323_t DRV8323 = {
    .gpioEnable = &gpioDrv8323Enable,
    .gpioNFault = 0,
    .gpioCal = 0,
    .spi = 0,
};


void drv8323_fault_cb(extiChannel_t* exti)
{
    drv8323_disable(&DRV8323);
    uint32_t mask = 1 << exti->channel;
    if (EXTI->RPR1 & mask) {
        EXTI->RPR1 |= mask;
    } 
    if (EXTI->FPR1 & mask) {
        EXTI->FPR1 |= mask;
    }

    drv8323_read_all(&DRV8323);
}

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
        // gpio_set_speed(&gpioDrv8323Enable, 0b11);
    }
    if (drv->gpioNFault) {
        gpio_initialize(drv->gpioNFault);
        gpio_configure_pupdr(drv->gpioNFault, GPIO_PULL_UP);
        exti_configure_cb(&extiChannels[drv->gpioNFault->pin], drv8323_fault_cb)
    }
    if (drv->gpioCal) {
        gpio_initialize(drv->gpioCal);
        gpio_reset(drv->gpioCal);
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

    // drv8323_setup_fault_callback(drv, int (*)(void))
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

void drv8323_read_all(drv8323_t* drv)
{
    volatile uint32_t reg = 0;
    reg = drv8323_read_reg(drv, DRV8323_REG_FAULT_STATUS_1);
    reg = drv8323_read_reg(drv, DRV8323_REG_VGS_STATUS_2);
    reg = drv8323_read_reg(drv, DRV8323_REG_DRIVER_CONTROL);
    reg = drv8323_read_reg(drv, DRV8323_REG_GATE_DRIVE_HS);
    reg = drv8323_read_reg(drv, DRV8323_REG_GATE_DRIVE_LS);
    reg = drv8323_read_reg(drv, DRV8323_REG_OCP_CONTROL);
    reg = drv8323_read_reg(drv, DRV8323_REG_CSA_CONTROL);
}
