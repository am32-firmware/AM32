/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

#include "ADC.h"
#include "main.h"
#include "targets.h"
#include "common.h"
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
extern char dshot;
int recieved_ints = 0;

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//for adc DMA
void DMA1_Channel1_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
//for IC timer DMA
void DMA1_Channel5_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
//for 20KHz int
void SysTick_Handler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));

//for cmp int
#ifdef USE_PA2_AS_COMP
void EXTI2_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
#else
void EXTI3_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
//for tele DMA
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//for com
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//dor dshot
void SW_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  PRINT("mcause:%08x\r\n",__get_MCAUSE());
  PRINT("mtval:%08x\r\n",__get_MTVAL());
  PRINT("mepc:%08x\r\n",__get_MEPC());
  while (1)
  {

  }
}

// for adc
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1))
    {
        DMA_ClearFlag(DMA1_IT_TC1|DMA1_IT_HT1);
        ADC_DMA_Callback( );
    }
    /* Check whether DMA transfer error caused the DMA interruption */
    if(DMA_GetITStatus(DMA1_IT_TE1))
    {
        DMA_ClearFlag(DMA1_IT_TE1);
    }

}

//for ic timer
void DMA1_Channel5_IRQHandler(void)
{
    if(DMA1->INTFR & DMA1_IT_HT5) //´«Êä¹ý°ë,ÇÐ»»±ßÑØ
    {
        if(servoPwm)
        {
            IC_TIMER_REGISTER->CCER = 0x03;  //ÇÐ»»ÎªÏÂ½µÑØ
        }
        DMA1->INTFCR = DMA1_IT_HT5;
    }
    if( DMA1->INTFR & DMA1_IT_TC5)
    {
        CLEAR_BIT(INPUT_DMA_CHANNEL->CFGR,0x1);  //disable DMA1_CH5
        transfercomplete();
        DMA1->INTFCR = DMA1_IT_TC5;
        input_ready = 1;
    }
    /* Check whether DMA transfer error caused the DMA interruption */
    if( DMA1->INTFR & DMA1_IT_TE5)
    {
        CLEAR_BIT(INPUT_DMA_CHANNEL->CFGR,0x1);  //disable DMA1_CH5
        DMA_ClearFlag(DMA1_IT_TE5);
        transfercomplete( );
        input_ready = 1;
    }
}

//for tele
void DMA1_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC7))
    {
        USART_DMACmd(USART2,USART_DMAReq_Tx,DISABLE);
        DMA_Cmd(DMA1_Channel7, DISABLE);
        MODIFY_REG(USART2->CTLR1, 0x3<<2, 0x0<<3);  //disable send
        DMA_ClearFlag(DMA1_IT_TC7);
    }
    /* Check whether DMA transfer error caused the DMA interruption */
    if(DMA_GetITStatus(DMA1_IT_TE7))
    {
        USART_DMACmd(USART2,USART_DMAReq_Tx,DISABLE);
        MODIFY_REG(USART2->CTLR1, 0x3<<2, 0x0<<3);  //disable send
        DMA_Cmd(DMA1_Channel7, DISABLE);
        DMA_ClearFlag(DMA1_IT_TE7);
    }
}

//for tenkhz
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    tenKhzRoutine( );
}

//for cmp
#ifdef USE_PA2_AS_COMP
void EXTI2_IRQHandler(void)
{
    EXTI->INTFR = EXTI_Line2;
    interruptRoutine( );
}
#else
void EXTI3_IRQHandler(void)
{
    EXTI->INTFR = EXTI_Line3;
    interruptRoutine( );
}

void EXTI4_IRQHandler(void)
{
    EXTI->INTFR = EXTI_Line4;
    interruptRoutine( );
}
#endif

//for com
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3,TIM_IT_Update))
    {
        PeriodElapsedCallback( );
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}


//for processDshot
void SW_Handler(void)
{
//   processDshot();
}
