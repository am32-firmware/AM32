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
    //	LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL,
    // LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    //  dma_transfer_direction_config(INPUT_DMA_CHANNEL,
    //  DMA_MEMORY_TO_PERIPHERAL);
    // INPUT_DMA_CHANNEL->CHCTRL |= DMA_DIR_PERIPHERALDST;
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;

    // 	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init
    // timer 2
    //  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);

    //  timer_deinit(IC_TIMER_REGISTER);
    tmr_reset(IC_TIMER_REGISTER);
    // IC_TIMER_REGISTER->CCMR1 = 0x60;
    //  TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x60;
    IC_TIMER_REGISTER->cm1 = 0x60; // oc mode pwm

    // IC_TIMER_REGISTER->CCER = 0x3;
    //  TIMER_CHCTL2(IC_TIMER_REGISTER) = 0x3;
    IC_TIMER_REGISTER->cctrl = 0x3; //

    //  IC_TIMER_REGISTER->PSC = output_timer_prescaler;
    //  IC_TIMER_REGISTER->prR = 61;
    IC_TIMER_REGISTER->div = output_timer_prescaler;
    IC_TIMER_REGISTER->pr = 95;

    out_put = 1;
    //  LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
    // timer_event_software_generate(IC_TIMER_REGISTER, TIMER_EVENT_SRC_UPG);
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
}

void changeToInput()
{
    gpio_mode_QUICK(GPIOB, GPIO_MODE_INPUT, GPIO_PULL_NONE, INPUT_PIN);

    //  LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL,
    //  LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // dma_transfer_direction_config(INPUT_DMA_CHANNEL,
    // DMA_PERIPHERAL_TO_MEMORY);
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;

    GPIOB->scr = INPUT_PIN;

    //  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init
    //  timer 2
    //	  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
    //	timer_deinit(IC_TIMER_REGISTER);
    IC_TIMER_REGISTER->cval = 0;
    tmr_reset(IC_TIMER_REGISTER);

    // IC_TIMER_REGISTER->CCMR1 = 0x1;
    //	TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x1;
    IC_TIMER_REGISTER->cm1 = 0x71;
    //	  IC_TIMER_REGISTER->CCER = 0xa;
    // TIMER_CHCTL2(IC_TIMER_REGISTER) = 0xa;
    IC_TIMER_REGISTER->cctrl = 0xB;

    // IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
    //	  IC_TIMER_REGISTER->prR = 0xFFFF;
    IC_TIMER_REGISTER->div = ic_timer_prescaler;
    IC_TIMER_REGISTER->pr = 0xFFFF;

    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
    out_put = 0;
}
void receiveDshotDma()
{
    changeToInput();
    //	IC_TIMER_REGISTER->CNT = 0;

    //  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
    //  (uint32_t)&IC_TIMER_REGISTER->CCR1, (uint32_t)&dma_buffer,
    //  LL_DMA_GetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL));
    // dma_periph_address_config(INPUT_DMA_CHANNEL,
    // (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));

    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    // dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&dma_buffer);
    //  LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, buffersize);
    // dma_transfer_number_config(INPUT_DMA_CHANNEL, buffersize);
    INPUT_DMA_CHANNEL->dtcnt = buffersize;

    //  LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
    //	dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_FTF);
    //   LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
    // dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_ERR);

    // LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
    // TMR_DMACmd(IC_TIMER_REGISTER, TMR_DMA_CC1, ENABLE);
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;

    // LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);
    //  timer_dma_enable(IC_TIMER_REGISTER, TIMER_DMA_CH0D) ;
    //  TMR_DMAConfig(IC_TIMER_REGISTER, TMR_DMABase_CC1,
    //  TMR_DMABurstLength_1Transfer);

    //   LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
    //	TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;

    //   LL_TIM_EnableCounter(IC_TIMER_REGISTER);
    //		 TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;

    // IC_TIMER_REGISTER->CTRL1 |= TMR_CTRL1_CNTEN;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;

    // DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000098b;   // just set the whole reg in
    // one g to enable

    // gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_Mode_AF, GPIO_Pull_NOPULL,
    // INPUT_PIN);

    INPUT_DMA_CHANNEL->ctrl = 0x0000098b;
}

void sendDshotDma()
{
    changeToOutput();

    //		  LL_DMA_ConfigAddresses(DMA1, INPUT_DMA_CHANNEL,
    //(uint32_t)&gcr, (uint32_t)&IC_TIMER_REGISTER->CCR1,
    // LL_DMA_GetDataTransferDirection(DMA1,
    // INPUT_DMA_CHANNEL));
    // dma_periph_address_config(INPUT_DMA_CHANNEL,
    // (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));
    // dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&gcr);

    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&gcr;

    //		  LL_DMA_SetDataLength(DMA1, INPUT_DMA_CHANNEL, 30);
    // dma_transfer_number_config(INPUT_DMA_CHANNEL, 26);
    INPUT_DMA_CHANNEL->dtcnt = 23 + buffer_padding;

    // LL_DMA_EnableIT_TC(DMA1, INPUT_DMA_CHANNEL);
    // dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_FTF);
    INPUT_DMA_CHANNEL->ctrl |= DMA_FDT_INT;
    //	  LL_DMA_EnableIT_TE(DMA1, INPUT_DMA_CHANNEL);
    // dma_interrupt_enable(INPUT_DMA_CHANNEL, DMA_INT_ERR);
    INPUT_DMA_CHANNEL->ctrl |= DMA_DTERR_INT;

    //	LL_DMA_EnableChannel(DMA1, INPUT_DMA_CHANNEL);
    // dma_channel_enable(INPUT_DMA_CHANNEL);

    INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;

    //	  LL_TIM_EnableDMAReq_CC1(IC_TIMER_REGISTER);
    // timer_dma_enable(IC_TIMER_REGISTER, TIMER_DMA_CH0D) ;
    // TMR_DMAConfig(IC_TIMER_REGISTER, TMR_DMABase_CC1,
    // TMR_DMABurstLength_1Transfer);

    // TMR_DMACmd(IC_TIMER_REGISTER, TMR_DMA_CC1, ENABLE);
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;

    //		  LL_TIM_CC_EnableChannel(IC_TIMER_REGISTER, IC_TIMER_CHANNEL);
    //    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;

    //  LL_TIM_EnableAllOutputs(IC_TIMER_REGISTER);
    //			timer_primary_output_config(IC_TIMER_REGISTER, ENABLE);

    // IC_TIMER_REGISTER->BRKDT |= TMR_BRKDT_MOEN;
    IC_TIMER_REGISTER->brk_bit.oen = TRUE;

    //	  LL_TIM_EnableCounter(IC_TIMER_REGISTER);
    //		TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
    //		IC_TIMER_REGISTER->CTRL1 |= TMR_CTRL1_CNTEN;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
    //		 gpio_mode_QUICK(GPIOB, GPIO_Mode_AF, GPIO_Pull_NOPULL,
    // INPUT_PIN);
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

uint8_t getInputPinState() { return (INPUT_PIN_PORT->idt & INPUT_PIN); }

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

void enableHalfTransferInt() { INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT; }
void setInputPullNone()
{
    gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
}
