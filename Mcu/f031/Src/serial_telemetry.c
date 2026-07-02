/*
 * serial_telemetry.c
 *
 * STM32F031 configuration for the shared STM32 LL telemetry driver.
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

#define TELEM_USART USART1
#define TELEM_DMA_CH LL_DMA_CHANNEL_4
#define TELEM_USART_DIR LL_USART_DIRECTION_TX
#define TELEM_GPIO_PORT GPIOB
#define TELEM_GPIO_PIN LL_GPIO_PIN_6
#define TELEM_GPIO_AF LL_GPIO_AF_0
#define TELEM_GPIO_SPEED LL_GPIO_SPEED_FREQ_HIGH
#define TELEM_GPIO_OTYPE LL_GPIO_OUTPUT_PUSHPULL
#define TELEM_USART_CLK_EN() LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1)
#define TELEM_GPIO_CLK_EN() LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)
#define TELEM_DMA_REMAP() LL_SYSCFG_SetRemapDMA_USART(LL_SYSCFG_USART1TX_RMP_DMA1CH4)
#define TELEM_NVIC_LATE()                 \
    do {                                  \
        NVIC_SetPriority(USART1_IRQn, 3); \
        NVIC_EnableIRQ(USART1_IRQn);      \
    } while (0)
#define TELEM_ASYNC_MODE
#define TELEM_SEND_DISABLE_BEFORE_DIR

#include "../../shared/serial_telemetry_stm32_ll.h"
