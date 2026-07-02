/*
 * serial_telemetry.c
 *
 * STM32G071 configuration for the shared STM32 LL telemetry driver.
 */

#include "serial_telemetry.h"
#include "targets.h"
#include "common.h"
#include "kiss_telemetry.h"

#define TELEM_USART USART1
#define TELEM_DMA_CH LL_DMA_CHANNEL_3
#define TELEM_USART_DIR LL_USART_DIRECTION_TX_RX
#define TELEM_GPIO_PORT GPIOB
#define TELEM_GPIO_PIN LL_GPIO_PIN_6
#define TELEM_GPIO_AF LL_GPIO_AF_0
#define TELEM_GPIO_SPEED LL_GPIO_SPEED_FREQ_LOW
#define TELEM_GPIO_OTYPE LL_GPIO_OUTPUT_OPENDRAIN
#define TELEM_USART_CLK_EN() LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define TELEM_GPIO_CLK_EN() LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB)
#define TELEM_NVIC_EARLY()                \
    do {                                  \
        NVIC_SetPriority(USART1_IRQn, 3); \
        NVIC_EnableIRQ(USART1_IRQn);      \
    } while (0)
#define TELEM_DMA_SETUP() LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART1_TX)
#define TELEM_HAS_PRESCALER
#define TELEM_HAS_FIFO
#define TELEM_ENABLE_TCTE_IRQ

#include "../../shared/serial_telemetry_stm32_ll.h"
