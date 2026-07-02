/*
 * serial_telemetry_stm32_ll.h
 *
 * Shared STM32 LL implementation of the UART DMA telemetry driver
 * (telem_UART_Init / send_telem_DMA).
 *
 * This file is #included at the end of Mcu/<family>/Src/serial_telemetry.c
 * after the family defines its TELEM_* configuration macros. It is not a
 * standalone translation unit and must not be compiled directly.
 *
 * Required macros:
 *   TELEM_USART              USART instance
 *   TELEM_DMA_CH             LL_DMA_CHANNEL_x used for TX
 *   TELEM_USART_DIR          initial LL_USART_DIRECTION_* programmed at init
 *   TELEM_GPIO_PORT / TELEM_GPIO_PIN / TELEM_GPIO_AF
 *   TELEM_GPIO_SPEED / TELEM_GPIO_OTYPE
 *   TELEM_USART_CLK_EN()     USART peripheral clock enable
 *   TELEM_GPIO_CLK_EN()      GPIO port clock enable
 *
 * Optional hooks (default empty), placed to preserve each family's
 * exact register write order:
 *   TELEM_RCC_CLOCKSOURCE()  USART kernel clock selection (G431/L431)
 *   TELEM_NVIC_EARLY()       USART IRQ setup before DMA config (G0/G431)
 *   TELEM_NVIC_LATE()        USART IRQ setup after DMA config (F031/F051-PA14)
 *   TELEM_DMA_SETUP()        DMAMUX/request routing before channel config
 *   TELEM_DMA_REMAP()        SYSCFG DMA remap after channel config (F031)
 *
 * Optional feature switches (#define to enable):
 *   TELEM_HAS_PRESCALER      USART init struct has PrescalerValue (G0/G431)
 *   TELEM_HAS_FIFO           USART FIFO thresholds to set + disable (G0/G431)
 *   TELEM_ASYNC_MODE         async wiring (F0) instead of half-duplex mode
 *                            with TEACK/REACK wait
 *   TELEM_ENABLE_TCTE_IRQ    enable DMA TC/TE interrupts at init
 *   TELEM_DMAREQ_AT_INIT     assert the USART TX DMA request once at init
 *                            instead of on every send (F051 default)
 *   TELEM_SEND_DISABLE_BEFORE_DIR  disable DMA channel in send before the
 *                                  direction switch (F031)
 *   TELEM_SEND_DISABLE_AFTER_DIR   disable DMA channel in send after the
 *                                  direction switch (G431/F051 default)
 */

#ifndef TELEM_RCC_CLOCKSOURCE
#define TELEM_RCC_CLOCKSOURCE()
#endif
#ifndef TELEM_NVIC_EARLY
#define TELEM_NVIC_EARLY()
#endif
#ifndef TELEM_NVIC_LATE
#define TELEM_NVIC_LATE()
#endif
#ifndef TELEM_DMA_SETUP
#define TELEM_DMA_SETUP()
#endif
#ifndef TELEM_DMA_REMAP
#define TELEM_DMA_REMAP()
#endif

void telem_UART_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    TELEM_RCC_CLOCKSOURCE();

    /* Peripheral clock enable */
    TELEM_USART_CLK_EN();
    TELEM_GPIO_CLK_EN();

    GPIO_InitStruct.Pin = TELEM_GPIO_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = TELEM_GPIO_SPEED;
    GPIO_InitStruct.OutputType = TELEM_GPIO_OTYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = TELEM_GPIO_AF;
    LL_GPIO_Init(TELEM_GPIO_PORT, &GPIO_InitStruct);

    TELEM_NVIC_EARLY();

    TELEM_DMA_SETUP();

    LL_DMA_SetDataTransferDirection(DMA1, TELEM_DMA_CH,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetChannelPriorityLevel(DMA1, TELEM_DMA_CH, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, TELEM_DMA_CH, LL_DMA_MODE_NORMAL);

    LL_DMA_SetPeriphIncMode(DMA1, TELEM_DMA_CH, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, TELEM_DMA_CH, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, TELEM_DMA_CH, LL_DMA_PDATAALIGN_BYTE);

    LL_DMA_SetMemorySize(DMA1, TELEM_DMA_CH, LL_DMA_MDATAALIGN_BYTE);

    TELEM_DMA_REMAP();

    TELEM_NVIC_LATE();

#ifdef TELEM_HAS_PRESCALER
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
#endif
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = TELEM_USART_DIR;
#ifdef TELEM_ASYNC_MODE
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
#endif
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(TELEM_USART, &USART_InitStruct);
#ifdef TELEM_HAS_FIFO
    LL_USART_SetTXFIFOThreshold(TELEM_USART, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(TELEM_USART, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(TELEM_USART);
#endif
#ifdef TELEM_ASYNC_MODE
    LL_USART_DisableIT_CTS(TELEM_USART);
    LL_USART_ConfigAsyncMode(TELEM_USART);
    LL_USART_Enable(TELEM_USART);
#else
    LL_USART_ConfigHalfDuplexMode(TELEM_USART);

    LL_USART_Enable(TELEM_USART);
    while ((!(LL_USART_IsActiveFlag_TEACK(TELEM_USART))) || (!(LL_USART_IsActiveFlag_REACK(TELEM_USART)))) {
    }
#endif

    LL_DMA_ConfigAddresses(
        DMA1, TELEM_DMA_CH, (uint32_t)aTxBuffer,
        LL_USART_DMA_GetRegAddr(TELEM_USART, LL_USART_DMA_REG_DATA_TRANSMIT),
        LL_DMA_GetDataTransferDirection(DMA1, TELEM_DMA_CH));
    LL_DMA_SetDataLength(DMA1, TELEM_DMA_CH, sizeof(aTxBuffer));

#ifdef TELEM_ENABLE_TCTE_IRQ
    /* (5) Enable DMA transfer complete/error interrupts  */
    LL_DMA_EnableIT_TC(DMA1, TELEM_DMA_CH);
    LL_DMA_EnableIT_TE(DMA1, TELEM_DMA_CH);
#endif
#ifdef TELEM_DMAREQ_AT_INIT
    LL_USART_EnableDMAReq_TX(TELEM_USART);
#endif
}

void send_telem_DMA(uint8_t bytes)
{ // set data length and enable channel to start transfer
#ifdef TELEM_SEND_DISABLE_BEFORE_DIR
    LL_DMA_DisableChannel(DMA1, TELEM_DMA_CH);
#endif
    LL_USART_SetTransferDirection(TELEM_USART, LL_USART_DIRECTION_TX);
#ifdef TELEM_SEND_DISABLE_AFTER_DIR
    LL_DMA_DisableChannel(DMA1, TELEM_DMA_CH);
#endif
    LL_DMA_SetDataLength(DMA1, TELEM_DMA_CH, bytes);
#ifndef TELEM_DMAREQ_AT_INIT
    LL_USART_EnableDMAReq_TX(TELEM_USART);
#endif
    LL_DMA_EnableChannel(DMA1, TELEM_DMA_CH);
    LL_USART_SetTransferDirection(TELEM_USART, LL_USART_DIRECTION_RX);
}
