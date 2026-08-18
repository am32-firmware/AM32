/*
 * g32f031_int.c
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#include "g32f031_int.h"
#include "targets.h"
#include "peripherals.h"
#include "common.h"
#include "main.h"
#include "dshot.h"
#include "IO.h"
#include "signal.h"
#include "ADC.H"

extern void tenKhzRoutine();
extern void interruptRoutine();
extern void transfercomplete();
/**
 * @brief     This function handles NMI exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void NMI_Handler(void)
{
}

/**
 * @brief     This function handles Hard Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Memory Manage exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Bus Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles Usage Fault exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief     This function handles SVCall exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SVC_Handler(void)
{
}

/**
 * @brief     This function handles Debug Monitor exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief     This function handles PendSV_Handler exception
 *
 * @param     None
 *
 * @retval    None
 *
 */
void PendSV_Handler(void)
{
    
}
/*******************************************************************************
* Function Name  : TMR1_BRK_UP_TRG_COM_IRQHandler
* Description    : BRK and UPDate Interrupt handler
*                :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ATMR_BRK_UP_TRG_COM_IRQHandler(void)
{
    if(DDL_ATMR_IsActiveFlag_UPDATE(ATMR)==SET)
    {
        DDL_ATMR_ClearFlag_UPDATE(ATMR);
    }
    if(DDL_ATMR_IsActiveFlag_BRK(ATMR)==SET)
    {
        DDL_ATMR_ClearFlag_BRK(ATMR);
    }
}

extern COMP1_TypeDef* currentComp;
void COMP1_2_3_IRQHandler(void)
{
    if(currentComp->ISR & COMP_ISR_IFLG)
    {
        COMP1->ISR |= COMP_ISR_IFLG;
        COMP2->ISR |= COMP_ISR_IFLG;
        COMP3->ISR |= COMP_ISR_IFLG;
        interruptRoutine();
    }
}

void BTMR1_IRQHandler(void)
{
    if(BTMR1->SR & BTMR_SR_UIFLG)
    {
        BTMR1->SR &= ~BTMR_SR_UIFLG;
        PeriodElapsedCallback();
    }
}

void DMA_CH1_IRQHandler(void)
{
    if (DMA->ISR & DMA_ISR_TXCIFLG1)
    {
        DMA->IFCLR |= DMA_IFCLR_CTXCIFLG1;
        fill_dest_array();
        transfercomplete();
        EINT->SWIEN |=DDL_EINT_LINE_1;
    }
}

extern void processDshot();
void EINT0_1_IRQHandler(void)
{
    EINT->IPEND |= DDL_EINT_LINE_1;
    processDshot();
}

/**
 * @brief     This function handles SysTick request
 *
 * @param     None
 *
 * @retval    None
 *
 */
void SysTick_Handler(void)
{
    tenKhzRoutine();
}
