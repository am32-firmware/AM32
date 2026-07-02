/*
 * serial_telemetry_at32.h
 *
 * Shared AT32 implementation of the UART DMA telemetry driver
 * (send_telem_DMA / telem_UART_Init).
 *
 * This file is #included at the end of Mcu/<family>/Src/serial_telemetry.c
 * after the family defines its TELEM_* configuration macros. It is not a
 * standalone translation unit and must not be compiled directly.
 *
 * Required macros:
 *   TELEM_DMA_CH             DMA1_CHANNELx used for TX
 *   TELEM_GPIO_PULL          GPIO_PULL_UP / GPIO_PULL_NONE for the TX pin
 *   TELEM_DMA_IRQN           NVIC interrupt line enabled at the end of init
 *
 * Optional hooks (default empty), placed to preserve each family's
 * exact register write order:
 *   TELEM_PIN_MUX()          IOMUX selection after gpio_init (F421)
 *   TELEM_DMA_ROUTE()        flexible DMA request routing (F415)
 *   TELEM_PIN_REMAP()        pin remap before usart_init (F415)
 *
 * Optional feature switches (#define to enable):
 *   TELEM_ENABLE_DMA_INTS    enable DMA full-transfer/error interrupts
 *   TELEM_SEND_DISABLE       disable the DMA channel before reloading the
 *                            transfer count in send (F421)
 */

#ifndef TELEM_PIN_MUX
#define TELEM_PIN_MUX()
#endif
#ifndef TELEM_DMA_ROUTE
#define TELEM_DMA_ROUTE()
#endif
#ifndef TELEM_PIN_REMAP
#define TELEM_PIN_REMAP()
#endif

void send_telem_DMA(uint8_t bytes)
{ // set data length and enable channel to start transfer
#ifdef TELEM_SEND_DISABLE
    TELEM_DMA_CH->ctrl_bit.chen = FALSE;
#endif
    TELEM_DMA_CH->dtcnt = bytes;
    TELEM_DMA_CH->ctrl_bit.chen = TRUE;
}

void telem_UART_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* configure the usart1 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = TELEM_GPIO_PULL;
    gpio_init(GPIOB, &gpio_init_struct);
    TELEM_PIN_MUX();

    dma_reset(TELEM_DMA_CH);
    TELEM_DMA_ROUTE();

    dma_init_type dma_init_struct;
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = sizeof(aTxBuffer);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)aTxBuffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(TELEM_DMA_CH, &dma_init_struct);

#ifdef TELEM_ENABLE_DMA_INTS
    TELEM_DMA_CH->ctrl |= DMA_FDT_INT;
    TELEM_DMA_CH->ctrl |= DMA_DTERR_INT;
#endif

    /* configure usart1 param */
    TELEM_PIN_REMAP();
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_single_line_halfduplex_select(USART1, TRUE);
    usart_dma_transmitter_enable(USART1, TRUE);
    usart_enable(USART1, TRUE);

    nvic_irq_enable(TELEM_DMA_IRQN, 3, 0);
}
