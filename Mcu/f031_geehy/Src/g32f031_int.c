/*!
 * @file        g32f031int.c
 *
 * @brief       
 *
 * @version     V1.0
 *
 * @date        2026-01-04
 *
 * @attention
 *
 *  Copyright (C) 2026 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes ***************************************************************/
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
    if(DDL_COMP1_IsActiveFlag_IT(currentComp))
    {
        DDL_COMP1_ClearFlag_IT(COMP1);
        DDL_COMP1_ClearFlag_IT(COMP2);
        DDL_COMP1_ClearFlag_IT(COMP3);
        interruptRoutine();
    }
}

void BTMR1_IRQHandler(void)
{
    if(DDL_BTMR_IsActiveFlag_UPDATE(BTMR1))
    {
        DDL_BTMR_ClearFlag_UPDATE(BTMR1);
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
    DDL_EINT_ClearFlag_0_31(DDL_EINT_LINE_1);
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
