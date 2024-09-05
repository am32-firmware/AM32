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
// char output_timer_prescaler;
// int buffersize = 32;
// int smallestnumber = 0;
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 7;
// int dshot_runout_timer = 62500;
// uint32_t average_signal_pulse;

void changeToOutput()
{
//    tmr_reset;
    GPIOA->BSHR  = GPIO_Pin_0;
    GPIOA->CFGLR &= 0xFFFFFFF0;   //output HIGH first
    GPIOA->CFGLR |= 0x3;

//    GPIOA->CFGLR &= 0xFFFFFFF0;
    GPIOA->CFGLR |= 0x8;          //then change to AF_PP

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

    GPIOA->BSHR = GPIO_Pin_0;
    GPIOA->CFGLR &= 0xFFFFFFF0;
    GPIOA->CFGLR |= 0x4;       //float in

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

    if(servoPwm == 1)
        MODIFY_REG(IC_TIMER_REGISTER->CCER,(0x3<<(4*IC_TIMER_CHANNEL)),(0x1<<(4*IC_TIMER_CHANNEL)));// setup rising pin trigger.

    changeToInput();
    INPUT_DMA_CHANNEL->MADDR = (uint32_t)&dma_buffer[0];
    INPUT_DMA_CHANNEL->PADDR = (uint32_t)&IC_TIMER_REGISTER->CH1CVR;
    INPUT_DMA_CHANNEL->CNTR  = buffersize;
    INPUT_DMA_CHANNEL->CFGR  = 0x98b;

    IC_TIMER_REGISTER->CNT = 0;
    IC_TIMER_REGISTER->DMAINTENR |= TIM_DMA_CC1;
    IC_TIMER_REGISTER->CCER      |= (1<<0);           //CC3E
    IC_TIMER_REGISTER->CTLR1     |= (1<<0);         //TIM EN


//    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
//    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
//    INPUT_DMA_CHANNEL->dtcnt = buffersize;
//    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
//    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
//    INPUT_DMA_CHANNEL->ctrl = 0x0000098b;

    if(servoPwm == 1)
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

// void detectInput(){
// 	smallestnumber = 20000;
// 	average_signal_pulse = 0;
// 	dshot = 0;
// 	servoPwm = 0;
// 	int lastnumber = dma_buffer[0];

// 	for ( int j = 1 ; j < 31; j++){
// 		if(dma_buffer[j] - lastnumber > 0 ){
// 		if((dma_buffer[j] - lastnumber) < smallestnumber){

// 			smallestnumber = dma_buffer[j] - lastnumber;

// 	}

// 		average_signal_pulse += (dma_buffer[j] - lastnumber);
// 	}
// 		lastnumber = dma_buffer[j];
// 	}
// 	average_signal_pulse = average_signal_pulse/32 ;

// 	if ((smallestnumber > 1)&&(smallestnumber <= 5)&& (average_signal_pulse
// < 60)) { 		ic_timer_prescaler= 0; output_timer_prescaler=1; dshot
// = 1; 		buffer_padding = 12; 		dshot_runout_timer =
// 65000; 		armed_count_threshold = 10000; 		buffersize = 32;
// 	}
// 	if ((smallestnumber > 5 )&&(smallestnumber <= 10)&&
// (average_signal_pulse < 100)){ 		dshot = 1;
// ic_timer_prescaler=1; 		output_timer_prescaler=3;
// IC_TIMER_REGISTER->cval = 0xffff;
// 	//	TIMER_CNT(IC_TIMER_REGISTER) = 0xffff;
// 		buffer_padding = 7;
// 		dshot_runout_timer = 65000;
// 		armed_count_threshold = 10000;
// 		buffersize = 32;
// 	}
// //	if ((smallestnumber > 100 )&&(smallestnumber < 400)){
// //		multishot = 1;
// //		armed_count_threshold = 1000;
// //		buffersize = 4;
// //	}
// //	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
// //		oneshot42 = 1;
// //	}
// 		if (smallestnumber > 30 && smallestnumber < 20000){
// 			servoPwm = 1;
// 			ic_timer_prescaler=143;
// 			armed_count_threshold = 35;
// 			buffersize = 2;
// 		}

// 	if (smallestnumber == 0 || smallestnumber == 20000){
// 		inputSet = 0;
// 	}else{

// 		inputSet = 1;
// 	}

// }

uint8_t getInputPinState()
{
    return (INPUT_PIN_PORT->INDR & INPUT_PIN);
}

void setInputPullDown()
{
    INPUT_PIN_PORT->BCR = INPUT_PIN;
    #warning ONLY FROM GPIO_Pin_0 TO GPIO_Pin_7
    INPUT_PIN_PORT->CFGLR &= ~(0xF << (INPUT_PIN)*4);  //note only for Px0--Px7
    INPUT_PIN_PORT->CFGLR |= (0x4<<(INPUT_PIN)*4);
}

void setInputPullUp()
{
    INPUT_PIN_PORT->BSHR = INPUT_PIN;
    #warning ONLY FROM GPIO_Pin_0 TO GPIO_Pin_7
    INPUT_PIN_PORT->CFGLR &= ~(0xF << (INPUT_PIN)*4);  //note only for Px0--Px7
    INPUT_PIN_PORT->CFGLR |= (0x4<<(INPUT_PIN)*4);
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
