/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */

#include "peripherals.h"
#include "serial_telemetry.h"
#include "ADC.h"
#include "targets.h"
#include "string.h"

static void SystemClock_Config(void);
static void drv_comp0_init(void);
static void drv_comp1_init(void);
static void drv_pwm_init(void);
static void drv_gtmr_init(void);
static void drv_btmr0_init(void);
static void drv_btmr1_init(void);
static void drv_tmr_enable(void);
static void drv_gpio_init(void);
static void drv_systick_init(void);

//#pragma GCC push_options
//#pragma GCC optimize("O0")

//void gpio_test_set(void)
//{
//    GPIO_TEST->BSRR |= GPIO_PIN_TEST;
//}
//void gpio_test_reset(void)
//{
//    GPIO_TEST->BRR |= GPIO_PIN_TEST;
//}
//void gpio_test_inv(void)
//{
//    GPIO_TEST->OUTDR ^= GPIO_PIN_TEST;
//}
//void reloadWatchDogCounter(void)
//{
//    DDL_IWDT_ReloadCounter(IWDT);
//}

//#pragma GCC pop_options

void interrupts_init(void)
{
    NVIC_SetPriorityGrouping(0x00000003U);
    
//    NVIC_EnableIRQ(ATMR_BRK_UP_TRG_COM_IRQn);
//    NVIC_SetPriority(ATMR_BRK_UP_TRG_COM_IRQn, 0);

    NVIC_EnableIRQ(COMP1_2_3_IRQn);
    NVIC_SetPriority(COMP1_2_3_IRQn, 1);

    NVIC_EnableIRQ(BTMR1_IRQn);
    NVIC_SetPriority(BTMR1_IRQn, 0);

    NVIC_EnableIRQ(DMA_CH1_IRQn);
    NVIC_SetPriority(DMA_CH1_IRQn, 2);

    NVIC_EnableIRQ(EINT0_1_IRQn);
    NVIC_SetPriority(EINT0_1_IRQn, 3);

    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 6);
}

void initCorePeripherals(void)
{
    __disable_irq();
    SystemClock_Config();
    interrupts_init();
    
    drv_gpio_init();
    drv_pwm_init();
    drv_btmr0_init();
    drv_btmr1_init();
    drv_comp1_init();
    drv_systick_init();
    drv_gtmr_init();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void enableCorePeripherals()
{
    /* Enable counter */
    drv_tmr_enable();

#ifdef USE_ADC
    ADC_Init();
#endif
    
#ifdef USE_LED_STRIP

#endif

#ifdef USE_RGB_LED

#endif

#ifndef BRUSHED_MODE

#endif
    __enable_irq();
}




void initAfterJump()
{
    SCB->VTOR = APPLICATION_ADDRESS;
    __enable_irq();
}

void SystemClock_Config(void)
{
    DDL_RCC_Unlock();

    /* Set HSIEN */
    DDL_RCC_HSI_Enable();
    /* Wait for HSI READY */
    while(DDL_RCC_HSI_IsReady() != 1U)
    {}

    /* Configure FLASH latency */
    DDL_FLASH_SetLatency(DDL_FLASH_LATENCY2);

    /* Set HSI clock as system source clock, SYS_CLK = 64MHz */
    DDL_RCC_SetSysClkSource(DDL_RCC_SYS_CLKSOURCE_HSI);
    while(DDL_RCC_GetSysClkSource() != DDL_RCC_SYS_CLKSOURCE_HSI)
    {}

     /* Configure HSI Prescaler, SYS_CLK = HSI_CLK/1 */
    DDL_RCC_SetHSIPrescaler(DDL_RCC_HSI_DIV_1);

     /* Configure AHB Prescaler, AHB_CLK = SYS_CLK/1 */
    DDL_RCC_SetAHBPrescaler(DDL_RCC_AHB_DIV_1);

    /* Configure APB prescaler, APB_CLK = AHB_CLK/1 */
    DDL_RCC_SetAPBPrescaler(DDL_RCC_APB_DIV_1);

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();

    /* Set Systick to 1ms in using frequency set to SystemCoreClock */
    DDL_Init1msTick(SystemCoreClock);

    DDL_RCC_Lock();
}

void drv_gpio_init(void)
{
    DDL_GPIO_InitTypeDef GPIO_InitStruct;

    DDL_RCC_Unlock();
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_RCC_Lock();
    
    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_DISABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);

//    GPIO_InitStruct.Pin        =  GPIO_PIN_TEST;
//    GPIO_InitStruct.Mode       =  DDL_GPIO_MODE_OUTPUT;
//    GPIO_InitStruct.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
//    DDL_GPIO_Init(GPIO_TEST, &GPIO_InitStruct);
   
//    GPIO_InitStruct.Pin        =  GPIO_PIN_LED;
//    GPIO_InitStruct.Mode       =  DDL_GPIO_MODE_OUTPUT;
//    GPIO_InitStruct.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
//    DDL_GPIO_Init(GPIO_LED, &GPIO_InitStruct);
    
    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_ENABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);
}


void drv_iwdt_init(void)
{
    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_IWDT);
    DDL_RCC_Lock();

    DDL_IWDT_Enable(IWDT);
    DDL_IWDT_EnableWriteAccess(IWDT);
    DDL_IWDT_SetPrescaler(IWDT, DDL_IWDT_PSC_32);
    DDL_IWDT_SetReloadCounter(IWDT, 625U);
    DDL_IWDT_ReloadCounter(IWDT);
    
    if (DDL_RCC_IsActiveFlag_IWDTRST())
    {
        DDL_RCC_Unlock();
        DDL_RCC_ClearFlag_IWDTRST();
        DDL_RCC_Lock();
    }
}

void  drv_pwm_init(void)
{
    DDL_GPIO_InitTypeDef        GPIO_InitStructure;
    DDL_ATMR_InitTypeDef        TIM_TimeBaseInitStructure;
    DDL_ATMR_BDT_InitTypeDef    TIM_BDTRInitStructure;
    DDL_ATMR_OC_InitTypeDef     TIM_OCInitStructure;

    DDL_RCC_Unlock();
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_DIV);
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_ATMR);
    DDL_RCC_Lock();

    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_DISABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);
    
    GPIO_InitStructure.Pin        =  PHASE_A_GPIO_LOW;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull       =  DDL_GPIO_PULL_NO;
    DDL_GPIO_Init(PHASE_A_GPIO_PORT_LOW, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        =  PHASE_B_GPIO_LOW;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull       =  DDL_GPIO_PULL_NO;
    DDL_GPIO_Init(PHASE_B_GPIO_PORT_LOW, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        =  PHASE_C_GPIO_LOW;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    DDL_GPIO_Init(PHASE_C_GPIO_PORT_LOW, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        =  PHASE_A_GPIO_HIGH;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull       =  DDL_GPIO_PULL_NO;
    DDL_GPIO_Init(PHASE_A_GPIO_PORT_HIGH, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        =  PHASE_B_GPIO_HIGH;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull       =  DDL_GPIO_PULL_NO;
    DDL_GPIO_Init(PHASE_B_GPIO_PORT_HIGH, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        =  PHASE_C_GPIO_HIGH;
    GPIO_InitStructure.Mode       =  DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Drive      =  DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStructure.OutputType =  DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull       =  DDL_GPIO_PULL_NO;
    DDL_GPIO_Init(PHASE_C_GPIO_PORT_HIGH, &GPIO_InitStructure);

    DDL_GPIO_SetAF3Pin_10_15(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH, DDL_GPIO_AF3_ATMR_CH0);
    DDL_GPIO_SetAF3Pin_10_15(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH, DDL_GPIO_AF3_ATMR_CH1);
    DDL_GPIO_SetAF3Pin_10_15(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH, DDL_GPIO_AF3_ATMR_CH2);
    DDL_GPIO_SetAF3Pin_10_15(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, DDL_GPIO_AF3_ATMR_CH0N);
    DDL_GPIO_SetAF3Pin_10_15(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW, DDL_GPIO_AF3_ATMR_CH1N);
    DDL_GPIO_SetAF3Pin_10_15(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW, DDL_GPIO_AF3_ATMR_CH2N);
    
    DDL_GPIO_SetAFPin_8_15(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, DDL_GPIO_AF_3);
    DDL_GPIO_SetAFPin_8_15(PHASE_B_GPIO_PORT_LOW, PHASE_B_GPIO_LOW, DDL_GPIO_AF_3);
    DDL_GPIO_SetAFPin_8_15(PHASE_C_GPIO_PORT_LOW, PHASE_C_GPIO_LOW, DDL_GPIO_AF_3);
    DDL_GPIO_SetAFPin_8_15(PHASE_A_GPIO_PORT_HIGH, PHASE_A_GPIO_HIGH, DDL_GPIO_AF_3);
    DDL_GPIO_SetAFPin_8_15(PHASE_B_GPIO_PORT_HIGH, PHASE_B_GPIO_HIGH, DDL_GPIO_AF_3);
    DDL_GPIO_SetAFPin_8_15(PHASE_C_GPIO_PORT_HIGH, PHASE_C_GPIO_HIGH, DDL_GPIO_AF_3);
    
    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_ENABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);
    
    /* Timing configuration */
    DDL_ATMR_StructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.ClockDivision     = 0;
    TIM_TimeBaseInitStructure.Prescaler         = 0;
    TIM_TimeBaseInitStructure.Autoreload        = TIM1_AUTORELOAD;
    TIM_TimeBaseInitStructure.RepetitionCounter = 0;
    TIM_TimeBaseInitStructure.CounterMode       = DDL_ATMR_COUNTERMODE_UP;
    DDL_ATMR_Init(ATMR, &TIM_TimeBaseInitStructure);

    /* PWM output configuration */
    DDL_ATMR_OC_StructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.CompareValue            = 0;
    TIM_OCInitStructure.OCMode                  = DDL_ATMR_OCMODE_PWM1;
    TIM_OCInitStructure.OCState                 = DDL_ATMR_OCSTATE_DISABLE;
    TIM_OCInitStructure.OCNState                = DDL_ATMR_OCSTATE_DISABLE;
    TIM_OCInitStructure.OCPolarity              = DDL_ATMR_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNPolarity             = DDL_ATMR_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState             = DDL_ATMR_OCIDLESTATE_LOW;
    TIM_OCInitStructure.OCNIdleState            = DDL_ATMR_OCIDLESTATE_LOW;
    DDL_ATMR_OC_Init(ATMR,DDL_ATMR_CHANNEL_CH0,&TIM_OCInitStructure);
    DDL_ATMR_OC_Init(ATMR,DDL_ATMR_CHANNEL_CH1,&TIM_OCInitStructure);
    DDL_ATMR_OC_Init(ATMR,DDL_ATMR_CHANNEL_CH2,&TIM_OCInitStructure);
    
    /* Dead zone and braking configuration */
    DDL_ATMR_BDT_StructInit(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.DeadTime0              = DEAD_TIME;
    TIM_BDTRInitStructure.BreakState            = DDL_ATMR_BREAK_DISABLE;
    TIM_BDTRInitStructure.BreakPolarity         = DDL_ATMR_BREAK_POLARITY_LOW;
    TIM_BDTRInitStructure.AutomaticOutput       = DDL_ATMR_AUTOMATICOUTPUT_DISABLE;
    TIM_BDTRInitStructure.LockLevel             = DDL_ATMR_LOCKLEVEL_OFF;
    TIM_BDTRInitStructure.OSSIState             = DDL_ATMR_OSSI_ENABLE;
    TIM_BDTRInitStructure.OSSRState             = DDL_ATMR_OSSR_ENABLE;
    DDL_ATMR_BDT_Init(ATMR,&TIM_BDTRInitStructure);
    
    DDL_ATMR_SetBrkFilter(ATMR,0X3F);
    DDL_ATMR_SetBreakSource(ATMR, DDL_ATMR_BREAKSOURCE_COMP0);
    DDL_ATMR_ClearFlag_BRK(ATMR);
    DDL_ATMR_EnableIT_BRK(ATMR);
    
    DDL_ATMR_OC_EnablePreload(ATMR, DDL_ATMR_CHANNEL_CH0);
    DDL_ATMR_OC_EnablePreload(ATMR, DDL_ATMR_CHANNEL_CH1);
    DDL_ATMR_OC_EnablePreload(ATMR, DDL_ATMR_CHANNEL_CH2);
    DDL_ATMR_EnableARRPreload(ATMR);
}

void  drv_btmr0_init(void)
{
    DDL_BTMR_InitTypeDef BTMR_InitStruct={0};

    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_BTMR0);
    DDL_RCC_Lock();
    
    BTMR_InitStruct.Prescaler   =   63;
    BTMR_InitStruct.Autoreload  =   0xFFFF;
    BTMR_InitStruct.CounterMode =   DDL_BTMR_COUNTERMODE_UP;
    DDL_BTMR_Init(BTMR0, &BTMR_InitStruct);
}

void drv_btmr1_init(void)
{
    DDL_BTMR_InitTypeDef BTMR_InitStruct={0};

    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_BTMR1);
    DDL_RCC_Lock();
    
    BTMR_InitStruct.Prescaler   =   63;
    BTMR_InitStruct.Autoreload  =   0xFFFF;
    BTMR_InitStruct.CounterMode =   DDL_BTMR_COUNTERMODE_UP;
    DDL_BTMR_Init(BTMR1, &BTMR_InitStruct);
}

void drv_comp0_init(void)
{
    DDL_GPIO_InitTypeDef    GPIO_InitStruct;
    DDL_COMP0_InitTypeDef   COMP0_InitStruct;
    
    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_COMP0);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_RCC_Lock();
    
    GPIO_InitStruct.Pin     =   COMP0_INN_IBUS_PIN;
    GPIO_InitStruct.Mode    =   DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(COMP0_INN_IBUS_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin     =   COMP0_INP_VREF_PIN;
    GPIO_InitStruct.Mode    =   DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(COMP0_INP_VREF_PORT, &GPIO_InitStruct);
    
    COMP0_InitStruct.HsyN           =   DDL_COMP0_HYSN_DISABLE;
    COMP0_InitStruct.HsyP           =   DDL_COMP0_HYSP_DISABLE;
    COMP0_InitStruct.FilterCFG      =   DDL_COMP0_FILTERCFG_1;
    COMP0_InitStruct.FilterPSC      =   DDL_COMP0_FILTERPSC_1;
    COMP0_InitStruct.OutputPol      =   DDL_COMP0_OUTPUTPOL_NONINVERTED;
    COMP0_InitStruct.InputMinus     =   DDL_COMP0_INPUT_MINUS_PA8;
    COMP0_InitStruct.InputPlus      =   DDL_COMP0_INPUT_PLUS_PA7;
    DDL_COMP0_Init(COMP0, &COMP0_InitStruct);
    
    DDL_COMP0_Enable(COMP0);
}
void drv_comp1_init(void)
{
    DDL_GPIO_InitTypeDef    GPIO_InitStruct;
    DDL_COMP1_InitTypeDef   COMP1_InitStruct;

    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_COMP1);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_RCC_Lock();
    
    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_DISABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);
    
    GPIO_InitStruct.Pin     =   COMP1_BEMFA_PIN;
    GPIO_InitStruct.Mode    =   DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(COMP1_BEMFA_PROT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin     =   COMP2_BEMFC_PIN;
    GPIO_InitStruct.Mode    =   DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(COMP2_BEMFC_PROT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin     =   COMP3_BEMFB_PIN;
    GPIO_InitStruct.Mode    =   DDL_GPIO_MODE_ANALOG;
    DDL_GPIO_Init(COMP3_BEMFB_PROT, &GPIO_InitStruct);

    DDL_GPIO_LockKey(GPIOA, DDL_GPIO_LOCK_ENABLE);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);

    COMP1_InitStruct.HsyN           =   DDL_COMP1_HYSN_DISABLE;
    COMP1_InitStruct.HsyP           =   DDL_COMP1_HYSP_DISABLE;
    COMP1_InitStruct.FilterCFG      =   DDL_COMP1_FILTERCFG_1;
    COMP1_InitStruct.FilterPSC      =   DDL_COMP1_FILTERPSC_1;
    COMP1_InitStruct.OutputPol      =   DDL_COMP1_OUTPUTPOL_NONINVERTED;
    COMP1_InitStruct.InputPlus      =   DDL_COMP1_INPUT_PLUS_VIRTUAL;
    COMP1_InitStruct.InputMinus     =   DDL_COMP1_INPUT_MINUS_PA1;
    DDL_COMP1_Init(COMP1, &COMP1_InitStruct);
    
    COMP1_InitStruct.InputPlus      =   DDL_COMP1_INPUT_PLUS_VIRTUAL;
    COMP1_InitStruct.InputMinus     =   DDL_COMP1_INPUT_MINUS_PA2;
    DDL_COMP1_Init(COMP2, &COMP1_InitStruct);
    
    COMP1_InitStruct.InputPlus      =   DDL_COMP1_INPUT_PLUS_VIRTUAL;
    COMP1_InitStruct.InputMinus     =   DDL_COMP1_INPUT_MINUS_PA0;
    DDL_COMP1_Init(COMP3, &COMP1_InitStruct);
    
    DDL_COMP1_Enable(COMP1);
    DDL_COMP1_Enable(COMP2);
    DDL_COMP1_Enable(COMP3); 
}

void drv_tmr_enable(void)
{
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH0);
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH0N);
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH1);
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH1N);
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH2);
    DDL_ATMR_CC_EnableChannel(ATMR, DDL_ATMR_CHANNEL_CH2N);
    DDL_ATMR_EnableCounter(ATMR);
    DDL_ATMR_EnableAllOutputs(ATMR);
    DDL_ATMR_GenerateEvent_UPDATE(ATMR);
    
    DDL_BTMR_EnableCounter(BTMR0);
    DDL_BTMR_EnableCounter(BTMR1);
}

void MX_IWDG_Init(void)
{
    drv_iwdt_init();
}
extern uint8_t PROCESS_ADC_FLAG;
void RELOAD_WATCHDOG_COUNTER(void)
{
    if(PROCESS_ADC_FLAG==1)
        IWDT->KEY  = DDL_IWDT_KEY_RELOAD ;
}

extern uint32_t last_dma_buffer[32];
void drv_gtmr_init(void)
{

    DDL_GTMR_IC_InitTypeDef GTMR_IC_InitStruct = {0};
    DDL_GTMR_OC_InitTypeDef GTMR_OC_InitStruct = {0};
    DDL_GTMR_InitTypeDef GTMR_InitStruct = {0};
    DDL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    DDL_DMA_InitTypeDef DMA_InitStruct = {0};
    DDL_EINT_InitTypeDef EINT_InitStruct = {0};

    DDL_RCC_Unlock();
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_GTMR);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOA);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_GPIOB);
    DDL_AHB_GRP1_EnableClock(DDL_AHB_GRP1_PERIPH_DMA);
    DDL_APB_GRP1_EnableClock(DDL_APB_GRP1_PERIPH_EINT);
    DDL_RCC_Lock();

    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_DISABLE);
    GPIO_InitStruct.Mode = DDL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Drive = DDL_GPIO_DRIVE_HIGH;
    GPIO_InitStruct.OutputType = DDL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = DDL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = DDL_GPIO_AF_4;
    GPIO_InitStruct.Pin = DDL_GPIO_PIN_1;
    DDL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    DDL_GPIO_LockKey(GPIOB, DDL_GPIO_LOCK_ENABLE);

    DMA_InitStruct.Peripheral = DDL_DMA_PERIPHERAL_4;
    DMA_InitStruct.Mode                    = DDL_DMA_MODE_NORMAL;
    DMA_InitStruct.Direction               = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.Priority                = DDL_DMA_PRIORITY_HIGH;
    DMA_InitStruct.PeriphOrM2MSrcIncMode   = DDL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode   = DDL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize  = DDL_DMA_PDATAALIGN_WORD;
    DMA_InitStruct.MemoryOrM2MDstDataSize  = DDL_DMA_MDATAALIGN_WORD;
    DMA_InitStruct.NbData = 32;                    
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&GTMR->CC0;
    DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&last_dma_buffer;
    DDL_DMA_Init(DMA, DDL_DMA_CHANNEL_1, &DMA_InitStruct);

    EINT_InitStruct.Line_0_31   = DDL_EINT_LINE_1;
    EINT_InitStruct.LineCommand = ENABLE;
    EINT_InitStruct.Mode        = DDL_EINT_MODE_IT;
    EINT_InitStruct.Trigger     = DDL_EINT_TRIGGER_FALLING;
    DDL_EINT_SetEINTSource(DDL_EINT_SOURCE_PORTB, EINT_InitStruct.Line_0_31);
    DDL_EINT_Init(&EINT_InitStruct);

}
void drv_systick_init(void)
{
    DDL_InitTick(CPU_FREQUENCY_MHZ*1000000, 20000U);
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
}

void setPWMCompare1(uint16_t compareone)   { ATMR->CC0 = compareone; }
void setPWMCompare2(uint16_t comparetwo)   { ATMR->CC1 = comparetwo; }
void setPWMCompare3(uint16_t comparethree) { ATMR->CC2 = comparethree; }

void generatePwmTimerEvent() 
{
}

void resetInputCaptureTimer()
{
    IC_TIMER_REGISTER->PSC = 0;
    IC_TIMER_REGISTER->CNT = 0;
}

void Error_Handler(void)
{

}
