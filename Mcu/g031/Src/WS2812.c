/*
 * WS2812.c
 *
 *  Created on: Sep 9, 2020
 *      Author: Alka
 */

#ifdef USE_LED_STRIP

#include "WS2812.h"
#include "targets.h"

char dma_busy;
uint16_t led_Buffer[28] = {
    0,
    0,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    60,
    60,
    60,
    60,
    60,
    60,
    60,
    60,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    20,
    0,
    0,
};

void waitClockCycles(uint16_t cycles){
  UTILITY_TIMER->CNT = 0;
    while (UTILITY_TIMER->CNT < cycles){
    }
}

void sendBit(uint8_t inbit){
  WS2812_LED_PORT->BSRR = WS2812_LED_PIN;
  waitClockCycles(CPU_FREQUENCY_MHZ>>(3-inbit));
  WS2812_LED_PORT->BRR = WS2812_LED_PIN;
  waitClockCycles(CPU_FREQUENCY_MHZ>>(1+inbit));
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue){
  disable_irq();
  UTILITY_TIMER->PSC = 0;
  LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
  uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue ;
  for(int i = 0; i < 24 ; i ++){
    sendBit((twenty_four_bit_color_number >> (23 - i))&1);
  }
  WS2812_LED_PORT->BRR = WS2812_LED_PIN;
  UTILITY_TIMER->PSC = CPU_FREQUENCY_MHZ;
  LL_TIM_GenerateEvent_UPDATE(UTILITY_TIMER);
__enable_irq();
 }

void WS2812_Init(void){
  LL_GPIO_SetPinMode(WS2812_LED_PORT, WS2812_LED_PIN, LL_GPIO_MODE_OUTPUT);
}

#endif