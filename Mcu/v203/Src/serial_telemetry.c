/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 *      modified by TempersLee June 21, 2024
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

//USART2_TX
void send_telem_DMA(uint8_t bytes)
{
    // set data length and enable channel to start transfer
    // set data length and enable channel to start transfer
    MODIFY_REG(USART2->CTLR1, USART_CTLR1_RE | USART_CTLR1_TE, USART_CTLR1_TE);  //ʹ�ܷ���
    DMA1_Channel7->CNTR  = bytes;
    DMA1_Channel7->MADDR = (uint32_t)&aTxBuffer[0];  //DMA��ַ�����ģ���Ҫ��������
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
    DMA_Cmd(DMA1_Channel7, ENABLE);
}

void telem_UART_Init(void)
{
    USART_InitTypeDef USART_InitStruct = {0};
    GPIO_InitTypeDef  GPIO_InitStruct = {0};
    DMA_InitTypeDef   DMA_InitStructure = {0};

    /* Peripheral clock enable */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

    /* configure the usart2 tx pin */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate            = 115200;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);
    USART_Cmd(USART2, ENABLE);


    /* USART2 DMA Init */
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&aTxBuffer[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = sizeof(aTxBuffer);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);


    /* Enable DMA transfer complete/error interrupts  */
    DMA_ClearFlag(DMA1_FLAG_TC7|DMA1_FLAG_TE7|DMA1_FLAG_HT7);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC|DMA_IT_TE, ENABLE);

    NVIC_SetPriority(DMA1_Channel7_IRQn, 0xE0);  //��������ж�
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}
