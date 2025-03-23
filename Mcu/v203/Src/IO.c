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

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 8;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 7;

void changeToOutput()
{
//    tmr_reset;
    INPUT_PIN_PORT->BSHR  = INPUT_PIN;
    uint32_t mul = INPUT_PIN*INPUT_PIN*INPUT_PIN*INPUT_PIN;
    uint32_t mask = 0xf * mul;
    INPUT_PIN_PORT->CFGLR &= ~mask;   //output HIGH first
    INPUT_PIN_PORT->CFGLR |= 0x3*mul;

//    INPUT_PIN_PORT->CFGLR &= ~mask;
    INPUT_PIN_PORT->CFGLR |= 0x8*mul;          //then change to AF_PP

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,DISABLE);

    IC_TIMER_REGISTER->CHCTLR1 = 0x60; // oc mode pwm
    IC_TIMER_REGISTER->CCER    = 0x3;     // outenable

    IC_TIMER_REGISTER->PSC     = output_timer_prescaler;
    IC_TIMER_REGISTER->ATRLR   = 63;                //48MHz / output_timer_prescaler / (63+1)
    out_put = 1;
    TIM_GenerateEvent(IC_TIMER_REGISTER,TIM_EventSource_Update);
}

void changeToInput()
{
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,DISABLE);
    IC_TIMER_REGISTER->CCER =  0;

    INPUT_PIN_PORT->BSHR = INPUT_PIN;
    uint32_t mul = INPUT_PIN*INPUT_PIN*INPUT_PIN*INPUT_PIN;
    uint32_t mask = 0xf * mul;
    INPUT_PIN_PORT->CFGLR &= ~mask;
    INPUT_PIN_PORT->CFGLR |= 0x4*mul;       //float in

    if(servoPwm)
    {
        IC_TIMER_REGISTER->CHCTLR1 = 0x1;
    }
    else
    {
        IC_TIMER_REGISTER->CHCTLR1 = 0x3;
        IC_TIMER_REGISTER->SMCFGR  = (0x1<<6);  //enable egle detect
    }

    IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    IC_TIMER_REGISTER->ATRLR = 0xffff;
    IC_TIMER_REGISTER->INTFR = 0;
    TIM_GenerateEvent(IC_TIMER_REGISTER,TIM_EventSource_Update);
    out_put = 0;
}
void receiveDshotDma()
{

    if(servoPwm == 1){
        MODIFY_REG(IC_TIMER_REGISTER->CCER,(0x3<<(4*IC_TIMER_CHANNEL)),(0x1<<(4*IC_TIMER_CHANNEL)));// setup rising pin trigger.
    }
    changeToInput();
    INPUT_DMA_CHANNEL->MADDR = (uint32_t)&dma_buffer[0];
    INPUT_DMA_CHANNEL->PADDR = (uint32_t)&IC_TIMER_REGISTER->CH1CVR;
    INPUT_DMA_CHANNEL->CNTR  = buffersize;
    INPUT_DMA_CHANNEL->CFGR  = 0x98b;

    IC_TIMER_REGISTER->CNT = 0;
    IC_TIMER_REGISTER->DMAINTENR |= TIM_DMA_CC1;
    IC_TIMER_REGISTER->CCER      |= (1<<0);           //CC3E
    IC_TIMER_REGISTER->CTLR1     |= (1<<0);         //TIM EN

    INPUT_DMA_CHANNEL->CFGR  |= (0x1<<2);

}

void sendDshotDma()
{
    changeToOutput();

    INPUT_DMA_CHANNEL->CFGR  = 0x99a;
    INPUT_DMA_CHANNEL->MADDR = (uint32_t)&gcr[0];
    INPUT_DMA_CHANNEL->PADDR = (uint32_t)&IC_TIMER_REGISTER->CH1CVR;
    INPUT_DMA_CHANNEL->CNTR  = 23 + buffer_padding;

    IC_TIMER_REGISTER->DMAINTENR |= TIM_DMA_CC1;
    IC_TIMER_REGISTER->CCER |= (1<<0);
    IC_TIMER_REGISTER->CTLR1  |= (1<<0);
    INPUT_DMA_CHANNEL->CFGR  |= 0x1;
}

uint8_t getInputPinState()
{
    return (INPUT_PIN_PORT->INDR & INPUT_PIN);
}

void setInputPullDown()
{
    volatile uint32_t *cfgr;
    uint32_t pin = INPUT_PIN;
    if (pin >= (1U<<8)) {
        pin >>= 8;
        cfgr = &INPUT_PIN_PORT->CFGHR;
    } else {
        cfgr = &INPUT_PIN_PORT->CFGLR;
    }
    const uint32_t mul = pin*pin*pin*pin;
    const uint32_t CFG = (*cfgr) & ~(0xf * mul);
    INPUT_PIN_PORT->OUTDR &= ~pin;
    *cfgr = CFG | (0x8*mul);
}

void setInputPullUp()
{
    volatile uint32_t *cfgr;
    uint32_t pin = INPUT_PIN;
    if (pin >= (1U<<8)) {
        pin >>= 8;
        cfgr = &INPUT_PIN_PORT->CFGHR;
    } else {
        cfgr = &INPUT_PIN_PORT->CFGLR;
    }
    const uint32_t mul = pin*pin*pin*pin;
    const uint32_t CFG = (*cfgr) & ~(0xf * mul);
    INPUT_PIN_PORT->OUTDR |= pin;
    *cfgr = CFG | (0x8*mul);
}

#ifndef WCH

void setInputPolarityRising()
{
    #error you have to make the setInputPolarityRising() function available.
}
void enableHalfTransferInt()
{
    #error you have to make the enableHalfTransferInt() function available.
}
void setInputPullNone()
{
    #error you have to make the setInputPullNone() function available.
}

#endif
