/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"
#include "kiss_telemetry.h"

#include "main.h"
#include "common.h"
#include "targets.h"

void telem_UART_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_GPIOB);
    RCU_CLOCK_ENABLE(RCU_DMA);
    RCU_CLOCK_ENABLE(RCU_USART0);

    dmaDeinit(DMA_CH1);

    gpioSetAf(GPIOB, GPIO_AF_0, GPIO_PIN_6);

    gpioSetMode(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpioSetOutputOptions(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);

    /* DMA channel1: memory to USART0 data, 8 bit, memory increment, low priority, not circular */
    DMA_CHPADDR(DMA_CH1) = (uint32_t)&USART_TDATA(USART0);
    DMA_CHMADDR(DMA_CH1) = (uint32_t)aTxBuffer;
    DMA_CHCNT(DMA_CH1) = sizeof(aTxBuffer);
    DMA_CHCTL(DMA_CH1) = DMA_PERIPHERAL_WIDTH_8BIT | DMA_MEMORY_WIDTH_8BIT | DMA_PRIORITY_LOW
        | DMA_CHXCTL_MNAGA | DMA_CHXCTL_DIR;

    USART_CTL2(USART0) |= USART_CTL2_DENT;
    /* enable DMA channel1 */
    DMA_CHCTL(DMA_CH1) |= DMA_CHXCTL_CHEN;
    /* half duplex, must be set with the usart disabled */
    USART_CTL0(USART0) &= ~USART_CTL0_UEN;
    USART_CTL2(USART0) |= USART_CTL2_HDEN;
    /* 115200 baud from the 72mhz APB2 clock, divider format depends on 8x or 16x oversampling */
    if (USART_CTL0(USART0) & USART_CTL0_OVSMOD) {
        const uint32_t udiv = (2U * CPU_FREQUENCY_MHZ * 1000000U + 115200U / 2U) / 115200U;
        USART_BAUD(USART0) = (udiv & 0xFFF0U) | ((udiv >> 1U) & 0x7U);
    } else {
        USART_BAUD(USART0) = (CPU_FREQUENCY_MHZ * 1000000U + 115200U / 2U) / 115200U;
    }
    USART_CTL0(USART0) |= USART_CTL0_REN | USART_CTL0_TEN;

    USART_CTL0(USART0) |= USART_CTL0_UEN;
}

void send_telem_DMA(uint8_t bytes)
{
    // set data length and enable channel to start transfer
    USART_CTL0(USART0) = (USART_CTL0(USART0) & ~USART_CTL0_REN) | USART_CTL0_TEN;
    DMA_CHCTL(DMA_CH1) &= ~DMA_CHXCTL_CHEN;
    DMA_CHCNT(DMA_CH1) = bytes;
    USART_CTL2(USART0) |= USART_CTL2_DENT;
    DMA_CHCTL(DMA_CH1) |= DMA_CHXCTL_CHEN;
    USART_CTL0(USART0) |= USART_CTL0_REN;
    //  usart_transmit_config(USART0, USART_TRANSMIT_DISABLE);
}
