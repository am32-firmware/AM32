extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void signalEdgeRoutine();

extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;

#include "gd32e23x_it.h"
#include "main.h"
#include "systick.h"
#include "targets.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

#if 0
void DMA_Channel1_2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_GI2(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        /* Call function Transmission complete Callback */
    }
    else if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_GI2(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        /* Call Error function */
        // USART_TransferError_Callback();
    }
}
#endif

void DMA_Channel3_4_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA_CH3, DMA_INT_FLAG_HTF)) {
        if (servoPwm) {
            TIMER_CHCTL2(TIMER2) |= (uint32_t)(TIMER_IC_POLARITY_FALLING);
            dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_HTF);
        }
    }
    if (dma_interrupt_flag_get(DMA_CH3, DMA_INT_FLAG_FTF) == 1) {
        dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_G);

        dma_channel_disable(DMA_CH3);

        transfercomplete();
        //      TIMER_CAR(TEN_KHZ_TIMER) = TIMER_CNT(TEN_KHZ_TIMER)+ 2;
        //   TIMER_CNT(TEN_KHZ_TIMER) = TIMER_CAR(TEN_KHZ_TIMER) - 2;
    }
    else if (dma_interrupt_flag_get(DMA_CH3, DMA_INT_FLAG_ERR) == 1) {
        dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_G);

    }
}

/**
  * @brief This function handles ADC and COMP interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
void ADC_CMP_IRQHandler(void)
{
    //  TIM17->CNT = 0;
    /* USER CODE BEGIN ADC1_COMP_IRQn 0 */
    if (exti_interrupt_flag_get(EXTI_21)) {
        /* Clear flag of EXTI */
        exti_flag_clear(EXTI_21);

        /* Call interruption treatment function */
        interruptRoutine();
    }
    /* USER CODE END ADC1_COMP_IRQn 0 */
    //  process_time = TIM17->CNT;
    /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

    /* USER CODE END ADC1_COMP_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC underrun error interrupts.
  */
void TIMER13_IRQHandler(void)
{
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    //TIM6->DIER &= ~(0x1UL << (0U));

    timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
    tenKhzRoutine();

    /* USER CODE END TIM6_DAC_IRQn 0 */

    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

    /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIMER15_IRQHandler(void)
{
    /* USER CODE BEGIN TIM14_IRQn 0 */
    //    if(LL_TIM_IsActiveFlag_UPDATE(TIM14) == 1)
    //    {
    timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);

    PeriodElapsedCallback();

    //    }

    /* USER CODE END TIM14_IRQn 0 */
    /* USER CODE BEGIN TIM14_IRQn 1 */

    /* USER CODE END TIM14_IRQn 1 */
}

void TIMER14_IRQHandler(void)
{
    /* USER CODE BEGIN TIM14_IRQn 0 */
    //    if(LL_TIM_IsActiveFlag_UPDATE(TIM14) == 1)
    //    {
    timer_flag_clear(TIMER14, TIMER_FLAG_UP);
    timer_flag_clear(TIMER14, TIMER_FLAG_CH0);
    timer_flag_clear(TIMER14, TIMER_FLAG_CH1);

    //    }

    /* USER CODE END TIM14_IRQn 0 */
    /* USER CODE BEGIN TIM14_IRQn 1 */

    /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

void TIMER2_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH0) == SET) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
    }

    if (timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP) == SET) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    }
}

#if 0
void DMA_Channel0_IRQHandler(void)         // ADC
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
        /* Clear flag DMA global interrupt */
        /* (global interrupt flag: half transfer and transfer complete flags) */
        LL_DMA_ClearFlag_GI1(DMA1);
        ADC_DMA_Callback();
        /* Call interruption treatment function */
        //   AdcDmaTransferComplete_Callback();
    }

    /* Check whether DMA transfer error caused the DMA interruption */
    if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
        /* Clear flag DMA transfer error */
        LL_DMA_ClearFlag_TE1(DMA1);
        /* Call interruption treatment function */
    }
}
#endif

void EXTI4_15_IRQHandler(void)
{
    //    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET) {
    //      LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);

    //      signalEdgeRoutine();
    //    }
}
