#include "main.h"
#include "stm32l4xx_it.h"
#ifndef BOOTLOADER
#include "ADC.h"
#include "IO.h"
#include "targets.h"
#endif
#include "WS2812.h"


extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern char send_telemetry;
int interrupt_time = 0;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;

int interrupt = 0;

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void SVC_Handler(void) { }

void DebugMon_Handler(void) { }

void PendSV_Handler(void) { }

void SysTick_Handler(void) { }

#ifndef BOOTLOADER
void DMA1_Channel1_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
		LL_DMA_ClearFlag_GI1(DMA1);
		ADC_DMA_Callback();
	}

	if(LL_DMA_IsActiveFlag_TE2(DMA1) == 1) {
		LL_DMA_ClearFlag_TE2(DMA1);
	}
}

void DMA1_Channel4_IRQHandler(void){
#ifdef USE_SERIAL_TELEMETRY
	if(LL_DMA_IsActiveFlag_TC4(DMA1))
	{
	LL_DMA_ClearFlag_GI4(DMA1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* Call function Transmission complete Callback */

	}
	else if(LL_DMA_IsActiveFlag_TE4(DMA1))
	{
		LL_DMA_ClearFlag_GI4(DMA1);
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* Call Error function */
	// USART_TransferError_Callback();
	}
#endif
}

void DMA1_Channel5_IRQHandler(void)
{
    if (armed && dshot_telemetry) {
        DMA1->IFCR |= DMA_IFCR_CGIF5;
        DMA1_Channel5->CCR = 0x00;
        if (out_put) {
            receiveDshotDma();
            compute_dshot_flag = 2;
        } else {
            sendDshotDma();
            compute_dshot_flag = 1;
        }
        EXTI->SWIER1 |= LL_EXTI_LINE_15;

        return;
    }

	if(LL_DMA_IsActiveFlag_HT5(DMA1)) {
		if(servoPwm){
			LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_FALLING);
			LL_DMA_ClearFlag_HT5(DMA1);
		}
	}

	if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1) {
        DMA1->IFCR |= DMA_IFCR_CGIF5;
        DMA1_Channel5->CCR = 0x00;
		transfercomplete();
		EXTI->SWIER1 |= LL_EXTI_LINE_15;
	}
	else if(LL_DMA_IsActiveFlag_TE5(DMA1) == 1) {
		LL_DMA_ClearFlag_GI5(DMA1);
	}
}

void TIM6_DAC_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
		LL_TIM_ClearFlag_UPDATE(TIM6);
		tenKhzRoutine();
	}
}

void TIM1_UP_TIM16_IRQHandler(void)
{
	PeriodElapsedCallback();
	LL_TIM_ClearFlag_UPDATE(TIM16);
}

void COMP_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
		interruptRoutine();
	}
}

void EXTI15_10_IRQHandler(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    processDshot();
}

#endif // BOOTLOADER
