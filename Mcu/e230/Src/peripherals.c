/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

// PERIPHERAL SETUP

#include "peripherals.h"

#include "ADC.h"
#include "common.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"

void initCorePeripherals(void)
{
#ifdef PA11_12_REMAP
    SYSCFG_CFG0 |= SYSCFG_PA11_REMAP_PA12;
#endif

    //	fmc_wscnt_set(2);
    // fmc_prefetch_enable();
    MX_GPIO_Init();
    MX_DMA_Init();
    TIM0_Init();
    TIMER5_Init();
    TIMER16_Init();
    COMP_Init();
    TIMER15_Init();
    TIMER13_Init();

    //  UN_TIM_Init();
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void COMP_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);

    /* configure PA1 as comparator input */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);
    rcu_periph_clock_enable(RCU_CFGCMP);
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_5);
    /* configure comparator channel0 */
    cmp_mode_init(CMP_HIGHSPEED, CMP_PA5, CMP_HYSTERESIS_NO);

    //  cmp_enable();
    //  delay_1ms(1);

    cmp_output_init(CMP_OUTPUT_NONE, CMP_OUTPUT_POLARITY_NOINVERTED);

    /* initialize exti line21 */
    exti_init(EXTI_21, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(EXTI_21);
    /* configure ADC_CMP NVIC */
    nvic_irq_enable(ADC_CMP_IRQn, 0);
}

void MX_IWDG_Init(void)
{
    fwdgt_config(4000, FWDGT_PSC_DIV16);
    fwdgt_enable();
}

void reloadWatchDogCounter() { fwdgt_counter_reload(); }

void TIM0_Init(void)
{
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    // TIMER_CAR(TIMER0) = 3000;
    // TIMER_PSC(TIMER0) = 0;

    timer_deinit(TIMER0);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER0 configuration */
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = TIM1_AUTORELOAD;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0/CH0N, CH1/CH1N and CH2/CH2N configuration in timing mode */
    timer_ocinitpara.outputstate = TIMER_CCX_DISABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);

    /* configure TIMER channel 0 */
#ifdef HARDWARE_GROUP_GD_B // reversed pa7 pa8
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
#else
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
#endif

    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0,
        TIMER_OC_SHADOW_ENABLE);

    /* configure TIMER channel 1 */

    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1,
        TIMER_OC_SHADOW_ENABLE);

    /* configure TIMER channel 2 */

    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2,
        TIMER_OC_SHADOW_ENABLE);

    timer_break_struct_para_init(&timer_breakpara);
    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate = TIMER_IOS_STATE_DISABLE;
    timer_breakpara.deadtime = DEAD_TIME;
    timer_breakpara.breakpolarity = TIMER_BREAK_POLARITY_HIGH;
    timer_breakpara.outputautostate = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER0, &timer_breakpara);

    //  timer_primary_output_config(TIMER0, ENABLE);

    /* TIMER0 channel control update interrupt enable */
    timer_interrupt_disable(TIMER0, TIMER_INT_CMT);
    /* TIMER0 break interrupt disable */
    timer_interrupt_disable(TIMER0, TIMER_INT_BRK);

    /* TIMER0 counter enable */
    //  timer_enable(TIMER0);
    timer_auto_reload_shadow_enable(TIMER0);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    gpio_mode_set(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_A_GPIO_LOW);
    gpio_output_options_set(PHASE_A_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_A_GPIO_LOW);

    gpio_mode_set(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_B_GPIO_LOW);
    gpio_output_options_set(PHASE_B_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_B_GPIO_LOW);

    gpio_mode_set(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_C_GPIO_LOW);
    gpio_output_options_set(PHASE_C_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_C_GPIO_LOW);

    gpio_af_set(PHASE_A_GPIO_PORT_LOW, GPIO_AF_2, PHASE_A_GPIO_LOW);
    gpio_af_set(PHASE_B_GPIO_PORT_LOW, GPIO_AF_2, PHASE_B_GPIO_LOW);
    gpio_af_set(PHASE_C_GPIO_PORT_LOW, GPIO_AF_2, PHASE_C_GPIO_LOW);

    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    gpio_mode_set(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_A_GPIO_HIGH);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_A_GPIO_HIGH);

    gpio_mode_set(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_B_GPIO_HIGH);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_B_GPIO_HIGH);

    gpio_mode_set(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_C_GPIO_HIGH);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_C_GPIO_HIGH);

    gpio_af_set(PHASE_A_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_A_GPIO_HIGH);
    gpio_af_set(PHASE_B_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_B_GPIO_HIGH);
    gpio_af_set(PHASE_C_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_C_GPIO_HIGH);
}

void TIMER5_Init(void)
{
    rcu_periph_clock_enable(RCU_TIMER5);
    TIMER_CAR(TIMER5) = 0xFFFF;
    TIMER_PSC(TIMER5) = 35;
}

void TIMER13_Init(void)
{
    rcu_periph_clock_enable(RCU_TIMER13);
    TIMER_CAR(TIMER13) = 1000000 / LOOP_FREQUENCY_HZ;
    TIMER_PSC(TIMER13) = 71;
    NVIC_SetPriority(TIMER13_IRQn, 3);
    NVIC_EnableIRQ(TIMER13_IRQn);
    timer_enable(TIMER13);
}

void TIMER15_Init(void)
{
    rcu_periph_clock_enable(RCU_TIMER15);
    TIMER_CAR(TIMER15) = 0xFFFF;
    TIMER_PSC(TIMER15) = 35;
    timer_auto_reload_shadow_enable(TIMER15);
    timer_enable(TIMER15);

    NVIC_SetPriority(TIMER15_IRQn, 1);
    NVIC_EnableIRQ(TIMER15_IRQn);
}

void TIMER16_Init(void)
{
    rcu_periph_clock_enable(RCU_TIMER16);
    TIMER_CAR(TIMER16) = 0xFFFF;
    TIMER_PSC(TIMER16) = 71;
    timer_auto_reload_shadow_enable(TIMER16);
    timer_enable(TIMER16);
}

void MX_DMA_Init(void)
{
    rcu_periph_clock_enable(RCU_DMA);

    NVIC_SetPriority(DMA_Channel1_2_IRQn, 3);
    NVIC_EnableIRQ(DMA_Channel1_2_IRQn);

    //  NVIC_SetPriority(DMA_Channel3_4_IRQn, 1);
    //  NVIC_EnableIRQ(DMA_Channel3_4_IRQn);
}

void MX_GPIO_Init(void) { }

void UN_TIM_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_DMA);

    gpio_output_options_set(INPUT_PIN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, INPUT_PIN);
    gpio_af_set(INPUT_PIN_PORT, INPUT_PIN_AF, INPUT_PIN);

    dma_periph_address_config(INPUT_DMA_CHANNEL,
        (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));
    dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&dma_buffer);

    // dma_transfer_direction_config(INPUT_DMA_CHANNEL,
    // DMA_PERIPHERAL_TO_MEMORY);
    //   DMA_CHCTL(INPUT_DMA_CHANNEL) |=  DMA_PRIORITY_LOW;
    //   dma_circulation_disable(INPUT_DMA_CHANNEL);
    //   dma_periph_increase_disable(INPUT_DMA_CHANNEL);
    // 	dma_memory_increase_enable(INPUT_DMA_CHANNEL);
    //   dma_periph_width_config(INPUT_DMA_CHANNEL, DMA_MEMORY_WIDTH_16BIT);
    //   dma_memory_width_config(INPUT_DMA_CHANNEL, DMA_MEMORY_WIDTH_32BIT);

    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000098a;

    NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
    NVIC_EnableIRQ(IC_DMA_IRQ_NAME);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER14);
    TIMER_CAR(TIMER2) = 0xFFFF;
    TIMER_PSC(TIMER2) = 10;
    /* enable a TIMER */

    //	LL_TIM_DisableARRPreload(IC_TIMER_REGISTER);
    timer_auto_reload_shadow_disable(IC_TIMER_REGISTER);

    timer_ic_parameter_struct timer_icinitpara;
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity = TIMER_IC_POLARITY_BOTH_EDGE;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter = 0x0;
    timer_input_pwm_capture_config(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, &timer_icinitpara);

    // NVIC_SetPriority(TIMER2_IRQn, 0);
    //  NVIC_EnableIRQ(TIMER2_IRQn);
    timer_enable(IC_TIMER_REGISTER);

    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, INPUT_PIN);
}

#ifdef USE_RGB_LED // has 3 color led
void LED_GPIO_init()
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

#endif

void setPWMCompare1(uint16_t compareone) { TIMER_CH0CV(TIMER0) = compareone; }
void setPWMCompare2(uint16_t comparetwo) { TIMER_CH1CV(TIMER0) = comparetwo; }
void setPWMCompare3(uint16_t comparethree)
{
    TIMER_CH2CV(TIMER0) = comparethree;
}

void generatePwmTimerEvent()
{
    timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);
}

void resetInputCaptureTimer()
{
    TIMER_PSC(IC_TIMER_REGISTER) = 0;
    TIMER_CNT(IC_TIMER_REGISTER) = 0;
}

void initAfterJump()
{
    __enable_irq();
    fmc_wscnt_set(2);
    fmc_prefetch_enable();
}

void enableCorePeripherals()
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0,
        TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1,
        TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2,
        TIMER_CCXN_ENABLE);

    /* Enable counter */
    TIMER_CTL0(TIMER0) |= (uint32_t)TIMER_CTL0_CEN;
    timer_primary_output_config(TIMER0, ENABLE);

    /* Force update generation */
    // timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);

#ifndef BRUSHED_MODE
    TIMER_CTL0(COM_TIMER) |= (uint32_t)TIMER_CTL0_CEN; // commutation_timer priority 0
    timer_event_software_generate(COM_TIMER, TIMER_EVENT_SRC_UPG);
    TIMER_DMAINTEN(COM_TIMER) &= (~(uint32_t)TIMER_INT_UP); // disable interrupt until after polling mode
#endif
    TIMER_CTL0(UTILITY_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    TIMER_CTL0(INTERVAL_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    timer_event_software_generate(INTERVAL_TIMER, TIMER_EVENT_SRC_UPG);

    //                  // 10khz timer
    TIMER_CTL0(TEN_KHZ_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    timer_event_software_generate(TEN_KHZ_TIMER, TIMER_EVENT_SRC_UPG);
    TIMER_DMAINTEN(TEN_KHZ_TIMER) |= (uint32_t)TIMER_INT_UP;
#ifdef USE_ADC
    ADC_Init();
#endif

    cmp_enable();
    delayMicros(1000);

#ifdef USE_ADC_INPUT

#else
    timer_channel_output_state_config(IC_TIMER_REGISTER, IC_TIMER_CHANNEL,
        TIMER_CCX_ENABLE);
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN; // enable counter
#endif
    rcu_periph_clock_disable(RCU_GPIOB);
    rcu_periph_clock_disable(RCU_DMA);

    NVIC_SetPriority(EXTI4_15_IRQn, 2);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    EXTI_INTEN |= (uint32_t)EXTI_15;

    UN_TIM_Init();
}
