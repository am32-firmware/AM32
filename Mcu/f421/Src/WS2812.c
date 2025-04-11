/*
 * WS2812.c
 *
 *  Created on: Sep 9, 2020
 *      Author: Alka
 */

#include "WS2812.h"

#include "functions.h"
#include "targets.h"

#ifdef USE_LED_STRIP

void waitClockCycles(uint16_t cycles)
{
  UTILITY_TIMER->cval = 0;
  while (UTILITY_TIMER->cval < cycles) {
  }
}

// void sendBit(uint8_t inbit){
//  if(inbit){
//	GPIOB->scr = GPIO_PINS_7;
//	waitClockCycles(CPU_FREQUENCY_MHZ>>1);
//	GPIOB->clr = GPIO_PINS_7;
//	waitClockCycles(CPU_FREQUENCY_MHZ>>2);
//   return;
//  }else{
//	GPIOB->scr = GPIO_PINS_7;
//	waitClockCycles(CPU_FREQUENCY_MHZ>>2);
//	GPIOB->clr = GPIO_PINS_7;
//	waitClockCycles(CPU_FREQUENCY_MHZ>>1);
//  }
// }

void sendBit(uint8_t inbit)
{
  GPIOB->scr = WS2812_PIN;
  waitClockCycles(CPU_FREQUENCY_MHZ >> (2 - inbit));
  GPIOB->clr = WS2812_PIN;
  waitClockCycles(CPU_FREQUENCY_MHZ >> (1 + inbit));
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
  __disable_irq();
  UTILITY_TIMER->div = 0;
  UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
  uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue;
  for (int i = 0; i < 24; i++) {
    sendBit((twenty_four_bit_color_number >> (23 - i)) & 1);
  }
  GPIOB->clr = WS2812_PIN;
  UTILITY_TIMER->div = CPU_FREQUENCY_MHZ;
  UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
  __enable_irq();
}

void WS2812_Init(void)
{
  gpio_mode_QUICK(GPIOB, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, WS2812_PIN);
}

#endif
