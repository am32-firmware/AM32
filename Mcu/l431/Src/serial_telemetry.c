/*
 * serial_telemetry.c
 *
 * STM32L431 configuration for the shared STM32 LL telemetry driver.
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

#define TELEM_USART USART1
#define TELEM_DMA_CH LL_DMA_CHANNEL_4
#define TELEM_USART_DIR LL_USART_DIRECTION_TX_RX
#define TELEM_GPIO_PORT GPIOB
#define TELEM_GPIO_PIN LL_GPIO_PIN_6
#define TELEM_GPIO_AF LL_GPIO_AF_7
#define TELEM_GPIO_SPEED LL_GPIO_SPEED_FREQ_LOW
#define TELEM_GPIO_OTYPE LL_GPIO_OUTPUT_PUSHPULL
#define TELEM_USART_CLK_EN() LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define TELEM_GPIO_CLK_EN() LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define TELEM_RCC_CLOCKSOURCE() LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2)
#define TELEM_DMA_SETUP() LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_2)
#define TELEM_ENABLE_TCTE_IRQ

#include "../../shared/serial_telemetry_stm32_ll.h"
