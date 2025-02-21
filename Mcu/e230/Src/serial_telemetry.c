/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "serial_telemetry.h"

#include "main.h"
#include "common.h"

uint8_t aTxBuffer[49];
uint8_t nbDataToTransmit = sizeof(aTxBuffer);

void telem_UART_Init(void)
{
    dma_parameter_struct dma_init_struct;
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_USART0);

    dma_deinit(DMA_CH1);

    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);

    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)aTxBuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = nbDataToTransmit;
    dma_init_struct.periph_addr = (uint32_t)&USART_TDATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init(DMA_CH1, &dma_init_struct);

    dma_circulation_disable(DMA_CH1);

    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    /* enable DMA channel1 transfer complete interrupt */
    //  dma_interrupt_enable(DMA_CH1, DMA_INT_FTF);
    /* enable DMA channel1 */
    dma_channel_enable(DMA_CH1);
    usart_halfduplex_enable(USART0);
    /* USART configure */
    //  usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_enable(USART0);
}

void send_telem_DMA(uint8_t bytes)
{ // set data length and enable channel to start transfer

    usart_receive_config(USART0, USART_TRANSMIT_DISABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    dma_channel_disable(DMA_CH1);
    DMA_CHCNT(DMA_CH1) = bytes;
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    dma_channel_enable(DMA_CH1);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    //  usart_transmit_config(USART0, USART_TRANSMIT_DISABLE);
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

void makeInfoPacket(){
   for(int i = 0;i < 48; i++){
     aTxBuffer[i] = eepromBuffer.buffer[i];
    }
    aTxBuffer[48] = get_crc8(aTxBuffer, 48);
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
