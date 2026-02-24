
#include "stm32g4xx_it.h"
#include "ADC.h"
#include "IO.h"
#include "WS2812.h"
#include "main.h"
#include "targets.h"

extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern char send_telemetry;
uint16_t interrupt_time = 0;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;
extern uint16_t commutation_interval;

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

void DMA1_Channel1_IRQHandler(void)
{
    if (armed && dshot_telemetry) {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        DMA1_Channel1->CCR = 0x00;
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
    /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
    if (LL_DMA_IsActiveFlag_HT1(DMA1)) {
        if (servoPwm) {
            LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
                LL_TIM_IC_POLARITY_FALLING);
            LL_DMA_ClearFlag_HT1(DMA1);
        }
    }
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        DMA1_Channel1->CCR = 0x00;
        transfercomplete();
        EXTI->SWIER1 |= LL_EXTI_LINE_15;
    } else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
        LL_DMA_ClearFlag_GI1(DMA1);
    }
}

void COMP1_2_3_IRQHandler(void)
{
	if(INTERVAL_TIMER->CNT > (commutation_interval>>1)){
    interrupt++;
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
        interruptRoutine();
        return;
    }

    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
        interruptRoutine();
        return;
    }
	}else{
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
	}
}

void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        tenKhzRoutine();
    }
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    PeriodElapsedCallback();
    LL_TIM_ClearFlag_UPDATE(TIM16);
}

void EXTI15_10_IRQHandler(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    processDshot();
}

void DMA1_Channel3_IRQHandler(void)
{
}
