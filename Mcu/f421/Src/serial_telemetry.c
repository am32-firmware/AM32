/*
 * serial_telemetry.c
 *
 * AT32F421 configuration for the shared AT32 telemetry driver.
 */

#include "serial_telemetry.h"
#include "common.h"
#include "kiss_telemetry.h"

#define TELEM_DMA_CH DMA1_CHANNEL2
#define TELEM_GPIO_PULL GPIO_PULL_UP
#define TELEM_DMA_IRQN DMA1_Channel3_2_IRQn
#define TELEM_PIN_MUX() gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_0)
#define TELEM_SEND_DISABLE

#include "../../shared/serial_telemetry_at32.h"
