/*
 * serial_telemetry.c
 *
 * AT32F415 configuration for the shared AT32 telemetry driver.
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

#define TELEM_DMA_CH DMA1_CHANNEL4
#define TELEM_GPIO_PULL GPIO_PULL_NONE
#define TELEM_DMA_IRQN DMA1_Channel4_IRQn
#define TELEM_DMA_ROUTE() dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_UART1_TX)
#define TELEM_PIN_REMAP() gpio_pin_remap_config(USART1_MUX, TRUE)
#define TELEM_ENABLE_DMA_INTS

#include "../../shared/serial_telemetry_at32.h"
