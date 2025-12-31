/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#define KR_KEY_Reload ((uint16_t)0xAAAA)
#define KR_KEY_Enable ((uint16_t)0xCCCC)

#include "peripherals.h"

#include "ADC.h"
#include "common.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"
#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif

void initCorePeripherals(void)
{
    system_clock_config();

    MX_GPIO_Init();
    MX_DMA_Init();
    TIM1_Init();
    TIM6_Init();
    TIM14_Init();
    AT_COMP_Init();
    TIM17_Init();
    TIM16_Init();

    UN_TIM_Init();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
#ifdef USE_LED_STRIP
    WS2812_Init();
#endif
#ifdef USE_RGB_LED
    LED_GPIO_init();
#endif 
}

void initAfterJump(void) { __enable_irq(); }

void system_clock_config(void)
{
    flash_psr_set(FLASH_WAIT_CYCLE_3);
    crm_reset();
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET) {
    }
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_30);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    }
    crm_auto_step_mode_enable(FALSE);
    system_core_clock_update();
}

void AT_COMP_Init(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_CMP_PERIPH_CLOCK, TRUE);
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_1);
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_5);
    NVIC_SetPriority(ADC1_CMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_CMP_IRQn);
    cmp_enable(CMP1_SELECTION, TRUE);
}

void MX_IWDG_Init(void)
{
    WDT->cmd = WDT_CMD_UNLOCK;
    WDT->cmd = WDT_CMD_ENABLE;
    WDT->div = WDT_CLK_DIV_32;
    WDT->rld = 4000;
    WDT->cmd = WDT_CMD_RELOAD;
}

void TIM1_Init(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    TMR1->pr = TIM1_AUTORELOAD;
    TMR1->div = 0;

    TMR1->cm1 = 0x6868; // Channel 1 and 2 in PWM output mode
    TMR1->cm2 = 0x68; // channel 3 in PWM output mode
#ifdef USE_INVERTED_HIGH
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_1,
        TMR_POLARITY_ACTIVE_LOW);
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_2,
        TMR_POLARITY_ACTIVE_LOW);
    tmr_output_channel_polarity_set(TMR1, TMR_SELECT_CHANNEL_3,
        TMR_POLARITY_ACTIVE_LOW);
#endif

    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);

    tmr_period_buffer_enable(TMR1, TRUE);
    TMR1->brk_bit.dtc = DEAD_TIME;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_A_GPIO_LOW);

    gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_B_GPIO_LOW);

    gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_C_GPIO_LOW);

    gpio_pin_mux_config(PHASE_A_GPIO_PORT_LOW, PHASE_A_PIN_SOURCE_LOW,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_B_GPIO_PORT_LOW, PHASE_B_PIN_SOURCE_LOW,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_C_GPIO_PORT_LOW, PHASE_C_PIN_SOURCE_LOW,
        GPIO_MUX_2);

    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_A_GPIO_HIGH);
    //  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //  GPIO_OSPEED_50MHZ,PHASE_A_GPIO_HIGH);

    gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_B_GPIO_HIGH);
    //   gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //   GPIO_OSPEED_50MHZ,PHASE_B_GPIO_HIGH);

    gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE,
        PHASE_C_GPIO_HIGH);
    //    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP,
    //    GPIO_OSPEED_50MHZ,PHASE_C_GPIO_HIGH);

    gpio_pin_mux_config(PHASE_A_GPIO_PORT_HIGH, PHASE_A_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_B_GPIO_PORT_HIGH, PHASE_B_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
    gpio_pin_mux_config(PHASE_C_GPIO_PORT_HIGH, PHASE_C_PIN_SOURCE_HIGH,
        GPIO_MUX_2);
}

void TIM6_Init(void)
{
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    TMR6->pr = 0xFFFF;
    TMR6->div = 59;
}

void TIM14_Init(void)
{
    crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
    TMR14->pr = 1000000 / LOOP_FREQUENCY_HZ;
    TMR14->div = 119;

    NVIC_SetPriority(TMR14_GLOBAL_IRQn, 3);
    NVIC_EnableIRQ(TMR14_GLOBAL_IRQn);

    // TMR_Cmd(TMR14, ENABLE);
}

void TIM16_Init(void)
{
    crm_periph_clock_enable(CRM_TMR16_PERIPH_CLOCK, TRUE);
    TMR16->pr = 500;
    TMR16->div = 59;
    NVIC_SetPriority(TMR16_GLOBAL_IRQn, 0);
    NVIC_EnableIRQ(TMR16_GLOBAL_IRQn);
}

void TIM17_Init(void)
{
    crm_periph_clock_enable(CRM_TMR17_PERIPH_CLOCK, TRUE);
    TMR17->pr = 0xFFFF;
    TMR17->div = 119;
    TMR17->ctrl1_bit.prben = TRUE;

    // TMR_Cmd(TMR15, ENABLE);
}

void MX_DMA_Init(void)
{
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    NVIC_SetPriority(DMA1_Channel5_4_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel5_4_IRQn);
}

void MX_GPIO_Init(void) { }

void UN_TIM_Init(void)
{
#ifdef USE_TIMER_3_CHANNEL_1
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
    gpio_pin_mux_config(INPUT_PIN_PORT, INPUT_PIN_SOURCE, GPIO_MUX_1);
#endif
#ifdef USE_TIMER_15_CHANNEL_1
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
    gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
#endif

    //	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1,ENABLE);

    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    INPUT_DMA_CHANNEL->ctrl = 0X98a; //  PERIPHERAL HALF WORD, MEMROY WORD ,
                                     //  MEMORY INC ENABLE , TC AND ERROR INTS
    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
    IC_TIMER_REGISTER->pr = 0xFFFF;
    IC_TIMER_REGISTER->div = 16;
    IC_TIMER_REGISTER->ctrl1_bit.prben = TRUE;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
}

void reloadWatchDogCounter()
{
    WDT->cmd = WDT_CMD_RELOAD;
}

void setPWMCompare1(uint16_t compareone) { TMR1->c1dt = compareone; }
void setPWMCompare2(uint16_t comparetwo) { TMR1->c2dt = comparetwo; }
void setPWMCompare3(uint16_t comparethree) { TMR1->c3dt = comparethree; }

void generatePwmTimerEvent()
{
    TMR1->swevt |= TMR_OVERFLOW_SWTRIG;
    ;
}

void resetInputCaptureTimer()
{
    IC_TIMER_REGISTER->pr = 0;
    IC_TIMER_REGISTER->cval = 0;
}

#ifdef USE_RGB_LED // has 3 color led
void LED_GPIO_init()
{
    /* GPIO Ports Clock Enable */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_mode_QUICK(RED_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, RED_PIN);

    gpio_mode_QUICK(GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GREEN_PIN);

    gpio_mode_QUICK(BLUE_PORT, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, BLUE_PIN);    
}

void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue){

  if(red > 0){   
    RED_PORT->clr = RED_PIN;
  }else{
    RED_PORT->scr = RED_PIN;
  }
  if(green > 0){
    GREEN_PORT->clr = GREEN_PIN;
  }else{
    GREEN_PORT->scr = GREEN_PIN;
  }
  if(blue > 0){
    BLUE_PORT->clr = BLUE_PIN;
  }else{
    BLUE_PORT->scr = BLUE_PIN;;
  }
}

#endif


void enableCorePeripherals()
{
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);

    TMR1->ctrl1_bit.tmren = TRUE;
    TMR1->brk_bit.oen = TRUE;
    TMR1->swevt |= TMR_OVERFLOW_SWTRIG;

#ifndef BRUSHED_MODE
    COM_TIMER->ctrl1_bit.tmren = TRUE;
    COM_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    COM_TIMER->iden &= ~TMR_OVF_INT;
#endif
    UTILITY_TIMER->ctrl1_bit.tmren = TRUE;
    INTERVAL_TIMER->ctrl1_bit.tmren = TRUE;
    INTERVAL_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    TEN_KHZ_TIMER->ctrl1_bit.tmren = TRUE;
    TEN_KHZ_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    TEN_KHZ_TIMER->iden |= TMR_OVF_INT;

#ifdef USE_ADC
    ADC_Init();
#endif

#ifdef USE_ADC_INPUT

#else
    tmr_channel_enable(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, TRUE);
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
#endif

    NVIC_SetPriority(EXINT15_4_IRQn, 2);
    NVIC_EnableIRQ(EXINT15_4_IRQn);
    EXINT->inten |= EXINT_LINE_15;
		
#ifdef USE_PULSE_OUT
 gpio_mode_QUICK(GPIOB, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_PINS_8);
#endif
}
