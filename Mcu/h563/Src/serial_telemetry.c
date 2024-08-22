/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_usart.h"

#include "targets.h"

uint8_t aTxBuffer[10] = "deadbeef\r\n";
uint8_t nbDataToTransmit = sizeof(aTxBuffer);
void telem_UART_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    // __HAL_RCC_USART3_CLK_ENABLE();
    // __HAL_RCC_GPIOB_CLK_ENABLE();j
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**USART3 GPIO Configuration

    PB6   ------> USART3_TX
    */

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART3 DMA Init */

    /* USART3_TX Init */
    LL_DMA_SetDataTransferDirection(GPDMA1, LL_DMA_CHANNEL_2,
    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetPeriphRequest(GPDMA1, LL_DMA_CHANNEL_2, LL_GPDMA1_REQUEST_USART3_TX);
    LL_DMA_SetChannelPriorityLevel(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_LOW_PRIORITY_LOW_WEIGHT);
    // LL_DMA_SetMode(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

    // LL_DMA_SetPeriphIncMode(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetDestIncMode(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_DEST_FIXED);
    LL_DMA_SetSrcIncMode(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_SRC_INCREMENT);

    LL_DMA_SetDestDataWidth(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_DEST_DATAWIDTH_BYTE);
    LL_DMA_SetSrcDataWidth(GPDMA1, LL_DMA_CHANNEL_2, LL_DMA_SRC_DATAWIDTH_BYTE);

    /* USART3 interrupt Init */
    NVIC_SetPriority(GPDMA1_Channel2_IRQn, 3);
    NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_DisableIT_CTS(USART3);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_TX(USART3);
    LL_USART_Enable(USART3);

    // set dma address
    LL_DMA_ConfigAddresses(
        GPDMA1, LL_DMA_CHANNEL_2, (uint32_t)aTxBuffer,
        LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_2, nbDataToTransmit);

    /* (5) Enable DMA transfer complete/error interrupts  */
    LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_DTE(GPDMA1, LL_DMA_CHANNEL_2);
}

void send_telem_DMA()
{
    if (LL_DMA_IsEnabledChannel(GPDMA1, LL_DMA_CHANNEL_2)) {
        return;
    }
        // set dma address
    LL_DMA_ConfigAddresses(
        GPDMA1, LL_DMA_CHANNEL_2, (uint32_t)aTxBuffer,
        LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));
    // set data length and enable channel to start transfer
    LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_2, nbDataToTransmit);
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_2);
    // LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_RX);
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return (crc_u);
}

uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++)
        crc = update_crc8(Buf[i], crc);
    return (crc);
}

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current,
    uint16_t consumption, uint16_t e_rpm)
{
    aTxBuffer[0] = temp; // temperature

    aTxBuffer[1] = (voltage >> 8) & 0xFF; // voltage hB
    aTxBuffer[2] = voltage & 0xFF; // voltage   lowB

    aTxBuffer[3] = (current >> 8) & 0xFF; // current
    aTxBuffer[4] = current & 0xFF; // divide by 10 for Amps

    aTxBuffer[5] = (consumption >> 8) & 0xFF; // consumption
    aTxBuffer[6] = consumption & 0xFF; //  in mah

    aTxBuffer[7] = (e_rpm >> 8) & 0xFF; //
    aTxBuffer[8] = e_rpm & 0xFF; // eRpM *100

    aTxBuffer[9] = get_crc8(aTxBuffer, 9);
}
