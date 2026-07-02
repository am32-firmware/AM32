/*
 * serial_telemetry.c
 *
 * STM32F051 configuration for the shared STM32 LL telemetry driver.
 * Two pinouts: USART2 on PA14 (USE_PA14_TELEMETRY) or USART1 on PB6.
 */

#include "serial_telemetry.h"
#include "common.h"
#include "targets.h"
#include "kiss_telemetry.h"

#ifdef USE_PA14_TELEMETRY

#define TELEM_USART USART2
#define TELEM_DMA_CH LL_DMA_CHANNEL_4
#define TELEM_USART_DIR LL_USART_DIRECTION_RX
#define TELEM_GPIO_PORT GPIOA
#define TELEM_GPIO_PIN LL_GPIO_PIN_14
#define TELEM_GPIO_AF LL_GPIO_AF_1
#define TELEM_GPIO_SPEED LL_GPIO_SPEED_FREQ_HIGH
#define TELEM_GPIO_OTYPE LL_GPIO_OUTPUT_PUSHPULL
#define TELEM_USART_CLK_EN() LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
#define TELEM_GPIO_CLK_EN() LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define TELEM_NVIC_LATE()                 \
    do {                                  \
        NVIC_SetPriority(USART2_IRQn, 3); \
        NVIC_EnableIRQ(USART2_IRQn);      \
    } while (0)
#define TELEM_ASYNC_MODE
#define TELEM_ENABLE_TCTE_IRQ

#else

#define TELEM_USART USART1
#define TELEM_DMA_CH LL_DMA_CHANNEL_2
#define TELEM_USART_DIR LL_USART_DIRECTION_RX
#define TELEM_GPIO_PORT GPIOB
#define TELEM_GPIO_PIN LL_GPIO_PIN_6
#define TELEM_GPIO_AF LL_GPIO_AF_0
#define TELEM_GPIO_SPEED LL_GPIO_SPEED_FREQ_HIGH
#define TELEM_GPIO_OTYPE LL_GPIO_OUTPUT_PUSHPULL
#define TELEM_USART_CLK_EN() LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1)
#define TELEM_GPIO_CLK_EN() LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)
#define TELEM_ASYNC_MODE
#define TELEM_DMAREQ_AT_INIT
#define TELEM_SEND_DISABLE_AFTER_DIR

#endif

#include "../../shared/serial_telemetry_stm32_ll.h"
