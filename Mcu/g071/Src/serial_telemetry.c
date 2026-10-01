/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"
#include "kiss_telemetry.h"
#include "targets.h"
#include "common.h"

void telem_UART_Init()
{
    LL_USART_InitTypeDef USART_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
#ifdef ULTRA_DEDICATED
    // 2 Mbaud telemetry: the pull-up of an open drain output is far too
    // slow for 500 ns bits (100.20 drives this pin push-pull as well)
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
#else
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
#endif
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#ifndef ULTRA_DEDICATED // no USART interrupt is used, and there is no handler for one (off in 100.20 too)
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);
#endif

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART1_TX);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART1);
    LL_USART_ConfigHalfDuplexMode(USART1);

    LL_USART_Enable(USART1);
    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1)))) {
    }

    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_3, (uint32_t)aTxBuffer,
        LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(aTxBuffer));

#ifndef ULTRA_DEDICATED
    // ultra builds run the TX channel without interrupts (as 100.20):
    // send_telem_DMA() checks and restarts it itself, and a transfer
    // complete handler running below the dshot EXTI could stop a frame
    // that was just started
    /* (5) Enable DMA transfer complete/error interrupts  */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
#endif
}

#ifdef ULTRA_DEDICATED
// a telemetry frame is still going out
uint8_t telem_tx_busy(void)
{
    return LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_3) && (LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3) != 0);
}
#endif

void send_telem_DMA(uint8_t bytes)
{ // set data length and enable channel to start transfer
#ifdef ULTRA_DEDICATED
    // never abort an in-flight transfer: disabling the channel mid-frame
    // puts a partial frame on the wire and desyncs the FC parser.
    // Skip instead - the FC re-requests with the next packet.
    if (telem_tx_busy()) {
        return;
    }
    // the previous frame is out: nothing else stops the channel in ultra
    // builds, and the new length is only taken while it is stopped
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_GI3(DMA1);
#endif
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX);
    //  GPIOB->OTYPER &= 0 << 6;
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, bytes);
    LL_USART_EnableDMAReq_TX(USART1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_RX);
}

#ifdef ULTRA_DEDICATED
void setBaudRate(uint32_t baud)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    // usart kernel clock is PCLK = CPU frequency. BRR is written with the
    // USART enabled, as 100.20 does on this MCU: the reference manual asks
    // for UE = 0, the direct write is what flies in the field and was kept
    // on purpose (agreed with Alka)
    USART1->BRR = (CPU_FREQUENCY_MHZ * 1000000U + (baud / 2)) / baud;
}
#endif
