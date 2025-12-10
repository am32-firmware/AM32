/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 7;


void changeToOutput()
{
  INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
  tmr_reset(IC_TIMER_REGISTER);
#ifdef USE_TIMER_3_CHANNEL_1
  IC_TIMER_REGISTER->cm1 = 0x60; // oc mode pwm
  IC_TIMER_REGISTER->cctrl = 0x3; //
#endif
#ifdef USE_TIMER_2_CHANNEL_3
  IC_TIMER_REGISTER->cm2 = 0x60; // oc mode pwm
  IC_TIMER_REGISTER->cctrl = 0x300; //
#endif
  IC_TIMER_REGISTER->div = output_timer_prescaler;
  IC_TIMER_REGISTER->pr = 95;
  out_put = 1;
  IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
}

void changeToInput()
{
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_NONE, INPUT_PIN);
  INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;
  GPIOB->scr = INPUT_PIN;
  IC_TIMER_REGISTER->cval = 0;
  tmr_reset(IC_TIMER_REGISTER);
#ifdef USE_TIMER_3_CHANNEL_1
  IC_TIMER_REGISTER->cm1 = 0x41;
  IC_TIMER_REGISTER->cctrl = 0xB;
#endif
#ifdef USE_TIMER_2_CHANNEL_3
  IC_TIMER_REGISTER->cm2 = 0x41;
  IC_TIMER_REGISTER->cctrl = 0xB00;
#endif

  IC_TIMER_REGISTER->div = ic_timer_prescaler;
  IC_TIMER_REGISTER->pr = 0xFFFF;

  IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
  out_put = 0;
}
void receiveDshotDma()
{
  changeToInput();
#ifdef USE_TIMER_3_CHANNEL_1
  INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
  IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
#endif
#ifdef USE_TIMER_2_CHANNEL_3
  INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c3dt;
  IC_TIMER_REGISTER->iden |= TMR_C3_DMA_REQUEST;
#endif
  INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
  INPUT_DMA_CHANNEL->dtcnt = buffersize;
  IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
  INPUT_DMA_CHANNEL->ctrl = 0x0000098b;
}

void sendDshotDma()
{
  changeToOutput();
#ifdef USE_TIMER_3_CHANNEL_1
  INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
  IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
#endif
#ifdef USE_TIMER_2_CHANNEL_3
  INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c3dt;
  IC_TIMER_REGISTER->iden |= TMR_C3_DMA_REQUEST;
#endif
  INPUT_DMA_CHANNEL->maddr = (uint32_t)&gcr;
  INPUT_DMA_CHANNEL->dtcnt = 23 + buffer_padding;
  INPUT_DMA_CHANNEL->ctrl |= DMA_FDT_INT;
  INPUT_DMA_CHANNEL->ctrl |= DMA_DTERR_INT;
  INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
  IC_TIMER_REGISTER->brk_bit.oen = TRUE;
  IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
}


uint8_t getInputPinState()
{
  return (INPUT_PIN_PORT->idt & INPUT_PIN);
}

void setInputPolarityRising()
{
  IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
}

void setInputPullDown()
{
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_DOWN, INPUT_PIN);
}

void setInputPullUp()
{
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_UP, INPUT_PIN);
}

void enableHalfTransferInt()
{
  INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT;
}
void setInputPullNone()
{
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
}
