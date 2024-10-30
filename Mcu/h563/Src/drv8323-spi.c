#include "stm32h563xx.h"
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
    .gpioNFault = &gpioDrv8323nFault,
    .gpioCal = &gpioDrv8323Cal,
    .spi = SPI5,
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
    EXTI_INTERRUPT_DISABLE_MASK(1 << drv->gpioNFault->pin);
    delayMillis(2);
    drv8323_disable(drv);
    // delay at least 1ms
    delayMillis(2);
    drv8323_enable(drv);
    // delayMicros(3000);
    delayMillis(2);
    EXTI_INTERRUPT_ENABLE_MASK(1 << drv->gpioNFault->pin);
}

// this is hard coded for now
void drv8323_initialize_gpio_spi(drv8323_t* drv)
{
    gpio_t gpioSpiNSS = DEF_GPIO(
        GATE_DRIVER_SPI_NSS_PORT,
        GATE_DRIVER_SPI_NSS_PIN,
        GATE_DRIVER_SPI_NSS_AF,
        GPIO_AF);
    gpio_t gpioSpiSCK = DEF_GPIO(
        GATE_DRIVER_SPI_SCK_PORT,
        GATE_DRIVER_SPI_SCK_PIN,
        GATE_DRIVER_SPI_SCK_AF,
        GPIO_AF);
    gpio_t gpioSpiMISO = DEF_GPIO(
        GATE_DRIVER_SPI_MISO_PORT,
        GATE_DRIVER_SPI_MISO_PIN,
        GATE_DRIVER_SPI_MISO_AF,
        GPIO_AF);
    gpio_t gpioSpiMOSI = DEF_GPIO(
        GATE_DRIVER_SPI_MOSI_PORT,
        GATE_DRIVER_SPI_MOSI_PIN,
        GATE_DRIVER_SPI_MOSI_AF,
        GPIO_AF);
    

    gpio_initialize(&gpioSpiNSS);
    gpio_initialize(&gpioSpiSCK);
    gpio_initialize(&gpioSpiMISO);
    gpio_configure_pupdr(&gpioSpiMISO, GPIO_PULL_UP);
    gpio_initialize(&gpioSpiMOSI);
    gpio_set_speed(&gpioSpiMOSI, GPIO_SPEED_VERYFAST);
}

void drv8323_initialize_gpio(drv8323_t* drv)
{
    if (drv->gpioEnable) {
        gpio_initialize(drv->gpioEnable);
        gpio_reset(drv->gpioEnable);
        // gpio_set_speed(&gpioDrv8323Enable, 0b11);
    }
    if (drv->gpioNFault) {
        gpio_initialize(drv->gpioNFault);
        gpio_configure_pupdr(drv->gpioNFault, GPIO_PULL_UP);
        exti_configure_cb(&extiChannels[drv->gpioNFault->pin], drv8323_fault_cb);
        exti_configure_port(&extiChannels[drv->gpioNFault->pin], EXTI_CHANNEL_FROM_PORT(drv->gpioNFault->port));
        exti_configure_trigger(&extiChannels[drv->gpioNFault->pin], EXTI_TRIGGER_FALLING);
        EXTI_NVIC_ENABLE(drv->gpioNFault->pin);
        // EXTI_INTERRUPT_ENABLE_MASK(1 << drv->gpioNFault->pin);
    }
    if (drv->gpioCal) {
        gpio_initialize(drv->gpioCal);
        gpio_reset(drv->gpioCal);
    }
    drv8323_initialize_gpio_spi(drv);
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


    drv8323_configure_spi(drv);

    // drv8323_configure_spi(drv);
    // if (drv->spi) {
        spi_initialize(drv->spi);
    // }

    drv8323_initialize_gpio(drv);

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
uint16_t spi_rx_buffer[256];
uint16_t spi_tx_buffer[256];
spi_t spi;

void drv8323_configure_spi(drv8323_t* drv)
{
    // enable spi clock
    GATE_DRIVER_SPI_ENABLE_CLOCK();

    spi.ref = SPI5;

    // 000: rcc_pclk3 selected as kernel clock (default after reset)
    // 001: pll2_q_ck selected as kernel clock
    // 010: pll3_q_ck selected as kernel clock
    // 011: hsi_ker_ck selected as kernel clock
    // 100: csi_ker_ck selected as kernel clock
    // 101: hse_ck selected as kernel clock
    // others: reserved, the kernel clock is disabled
    spi_configure_rcc_clock_selection(&spi, 0b101);

    spi._rx_buffer = spi_rx_buffer;
    spi._tx_buffer = spi_tx_buffer;
    spi._rx_buffer_size = 256;
    spi._tx_buffer_size = 256;
    spi.rxDma = &dmaChannels[7];
    spi.txDma = &dmaChannels[0];
    spi.txDmaRequest = LL_GPDMA1_REQUEST_SPI5_TX;
    spi.rxDmaRequest = LL_GPDMA1_REQUEST_SPI5_RX;
    // spi.CFG1_MBR = 0b011; // prescaler = 16 // this DOES NOT work on blueesc
    // spi.CFG1_MBR = 0b100; // prescaler = 32 // this works on blueesc
    spi.CFG1_MBR = 0b101; // prescaler = 64 // this works on blueesc
    // spi.CFG1_MBR = 0b100; // prescaler = 128 // this works on blueesc
    // spi.CFG1_MBR = 0b111; // prescaler = 256 // this works on blueesc

    drv->spi = &spi;
}

bool drv8323_get_fault_status(drv8323_t* drv)
{
    return !gpio_read(drv->gpioNFault);
}
