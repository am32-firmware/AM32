/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

void telem_UART_Init(void)
{
    DDL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    DDL_USART_InitTypeDef USART_InitStruct = {0};
    DDL_DMA_InitTypeDef DMA_InitStruct = {0};
    
    DDL_RCC_Unlock();
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_USART);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_DMA);
    DDL_RCC_Lock();

    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);
    
    GPIO_InitStruct.Pin         =   DDL_GPIO_PIN_9;
    GPIO_InitStruct.Mode        =   DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull        =   DDL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate   =   DDL_GPIO_AF_0;
    DDL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);

    DDL_DMA_StructInit(&DMA_InitStruct);
    DDL_DMA_DeInit(DMA, DDL_DMA_CHANNEL_0);
    DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&USART->DR;
    DMA_InitStruct.MemoryOrM2MDstAddress  = 0UL;
    DMA_InitStruct.Direction              = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitStruct.Mode                   = DDL_DMA_MODE_NORMAL;
    DMA_InitStruct.PeriphOrM2MSrcIncMode  = DDL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode  = DDL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_WORD;
    DMA_InitStruct.MemoryOrM2MDstDataSize = DDL_DMA_PDATAALIGN_WORD;
    DMA_InitStruct.NbData                 = 0UL;
    DMA_InitStruct.Peripheral              = DDL_DMA_PERIPHERAL_11;
    DMA_InitStruct.Priority               = DDL_DMA_PRIORITY_LOW;
    DMA_InitStruct.FIFOMode               = DDL_DMA_FIFOMODE_DISABLE;
    DMA_InitStruct.FIFOThreshold          = DDL_DMA_FIFOTHRESHOLD_1_4;
    DMA_InitStruct.MemBurst               = DDL_DMA_MBURST_SINGLE;
    DMA_InitStruct.PeriphBurst            = DDL_DMA_PBURST_SINGLE;
    DDL_DMA_Init(DMA, DDL_DMA_CHANNEL_0, &DMA_InitStruct);
    
    USART_InitStruct.BaudRate             = 115200U;
    USART_InitStruct.DataWidth            = DDL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits             = DDL_USART_STOPBITS_1;
    USART_InitStruct.Parity               = DDL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection    = DDL_USART_DIRECTION_TX;
    DDL_USART_Init(USART, &USART_InitStruct);
    
    DDL_USART_Enable(USART);
    DDL_USART_EnableDMAReq_TX(USART);
}
static uint32_t tx[48];
void send_telem_DMA(uint8_t bytes)
{
    for(int i=0; i<bytes; i++) tx[i] = aTxBuffer[i];
    
    DDL_DMA_DisableChannel(DMA, DDL_DMA_CHANNEL_0);
    DDL_DMA_SetMemoryAddress(DMA, DDL_DMA_CHANNEL_0, (uint32_t) tx);
    DDL_DMA_SetDataLength(DMA, DDL_DMA_CHANNEL_0, bytes);
    DDL_DMA_EnableChannel(DMA, DDL_DMA_CHANNEL_0);
}
