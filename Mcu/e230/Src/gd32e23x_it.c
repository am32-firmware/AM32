#include <stdint.h>

extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void signalEdgeRoutine();
extern void processDshot();
extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot_telemetry;
extern char out_put;
extern char armed;

uint16_t interrupt_time = 0;

#include "gd32e23x_it.h"

#include "common.h"
#include "main.h"
#include "systick.h"
#include "targets.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void) { }

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
void SVC_Handler(void) { }

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void) { }

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void) { delay_decrement(); }

void DMA_Channel3_4_IRQHandler(void)
{
    if (dshot_telemetry && armed) {
        DMA_INTC |= DMA_FLAG_ADD(DMA_INT_FLAG_G, INPUT_DMA_CHANNEL);
        DMA_CHCTL(INPUT_DMA_CHANNEL) &= ~DMA_CHXCTL_CHEN;
        if (out_put) {
            receiveDshotDma();
            compute_dshot_flag = 2;
        } else {
            sendDshotDma();
            compute_dshot_flag = 1;
        }
        EXTI_SWIEV |= (uint32_t)EXTI_15;
        return;
    }

    if (dma_interrupt_flag_get(INPUT_DMA_CHANNEL, DMA_INT_FLAG_HTF)) {
        if (servoPwm) {
            TIMER_CHCTL2(TIMER2) |= (uint32_t)(TIMER_IC_POLARITY_FALLING);
            dma_interrupt_flag_clear(INPUT_DMA_CHANNEL, DMA_INT_FLAG_HTF);
        }
    }
    if (dma_interrupt_flag_get(INPUT_DMA_CHANNEL, DMA_INT_FLAG_FTF) == 1) {
        dma_interrupt_flag_clear(INPUT_DMA_CHANNEL, DMA_INT_FLAG_G);
        dma_channel_disable(INPUT_DMA_CHANNEL);
        transfercomplete();
        EXTI_SWIEV |= (uint32_t)EXTI_15;
    } else if (dma_interrupt_flag_get(INPUT_DMA_CHANNEL, DMA_INT_FLAG_ERR) == 1) {
        dma_interrupt_flag_clear(INPUT_DMA_CHANNEL, DMA_INT_FLAG_G);
    }
}

/**
 * @brief This function handles ADC and COMP interrupts (COMP interrupts
 * through EXTI lines 21 and 22).
 */
void ADC_CMP_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_21)) {
        exti_flag_clear(EXTI_21);
        interruptRoutine();
    }
}

/**
 * @brief This function handles TIM6 global and DAC underrun error interrupts.
 */
void TIMER13_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
    tenKhzRoutine();
}

/**
 * @brief This function handles TIM14 global interrupt.
 */
void TIMER15_IRQHandler(void)
{
    interrupt_time = TIMER_CNT(UTILITY_TIMER);
    timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
    PeriodElapsedCallback();
    interrupt_time = ((uint16_t)TIMER_CNT(UTILITY_TIMER)) - interrupt_time;
}

void TIMER14_IRQHandler(void) { timer_flag_clear(TIMER14, TIMER_FLAG_UP); }

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up
 * interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void) { }

void TIMER2_IRQHandler(void)
{
    // sendDshotDma();
}

void EXTI4_15_IRQHandler(void)
{
    exti_flag_clear(EXTI_15);

    processDshot();
}
