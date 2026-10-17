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
#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif


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
#ifdef USE_RGB_LED
    LED_GPIO_init();
#endif 
#ifdef USE_LED_STRIP
    WS2812_Init();
#endif
#ifdef USE_SERIAL_TELEMETRY
    telem_UART_Init();
#endif
}

void COMP_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_GPIOA);

    /* configure PA1 as comparator input */
    gpioSetMode(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);
    RCU_CLOCK_ENABLE(RCU_CFGCMP);
    gpioSetMode(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_5);
    /* comparator high speed, PA5 inverting input, no hysteresis, no output selection, not inverted */
    CMP_CS = (CMP_CS & ~(CMP_CS_CMPM | CMP_CS_CMPMSEL | CMP_CS_CMPHST | CMP_CS_CMPOSEL | CMP_CS_CMPPL))
        | CS_CMPM(CMP_HIGHSPEED) | CS_CMPMSEL(CMP_PA5) | CS_CMPHST(CMP_HYSTERESIS_NO) | CS_CMPOSEL(CMP_OUTPUT_NONE);

    /* exti line21 interrupt on rising edge */
    EXTI_EVEN &= ~(uint32_t)EXTI_21;
    EXTI_FTEN &= ~(uint32_t)EXTI_21;
    EXTI_RTEN |= (uint32_t)EXTI_21;
    EXTI_INTEN |= (uint32_t)EXTI_21;
    EXTI_PD = (uint32_t)EXTI_21;
    NVIC_SetPriority(ADC_CMP_IRQn, 0);
    NVIC_EnableIRQ(ADC_CMP_IRQn);
}

void MX_IWDG_Init(void)
{
    // 4000 counts at 40khz / 16, same as fwdgt_config(4000, FWDGT_PSC_DIV16)
    uint32_t timeout = FWDGT_PSC_TIMEOUT;
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
    while ((FWDGT_STAT & FWDGT_STAT_PUD) && --timeout) {
    }
    if (timeout) {
        FWDGT_PSC = FWDGT_PSC_DIV16;
        timeout = FWDGT_RLD_TIMEOUT;
        while ((FWDGT_STAT & FWDGT_STAT_RUD) && --timeout) {
        }
        if (timeout) {
            FWDGT_RLD = 4000 & FWDGT_RLD_RLD;
            FWDGT_CTL = FWDGT_KEY_RELOAD;
        }
    }
    FWDGT_CTL = FWDGT_KEY_ENABLE;
}

void reloadWatchDogCounter() { FWDGT_RELOAD(); }

void TIM0_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_TIMER0);
    RCU_RESET_PULSE(RCU_TIMER0RST); // all TIMER0 registers back to reset values

    // edge aligned, up counting, no clock division, no repetition
    TIMER_PSC(TIMER0) = 0;
    TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
    TIMER_CREP(TIMER0) = 0;
    TIMER_SWEVG(TIMER0) |= (uint32_t)TIMER_SWEVG_UPG;

    // CH0-CH2 pwm with compare shadow, outputs stay disabled with high polarity and low idle state
#ifdef HARDWARE_GROUP_GD_B // reversed pa7 pa8
    TIMER_CHCTL0(TIMER0) = (TIMER_OC_MODE_PWM1 | TIMER_OC_SHADOW_ENABLE)
        | ((uint32_t)(TIMER_OC_MODE_PWM0 | TIMER_OC_SHADOW_ENABLE) << 8U);
#else
    TIMER_CHCTL0(TIMER0) = (TIMER_OC_MODE_PWM0 | TIMER_OC_SHADOW_ENABLE)
        | ((uint32_t)(TIMER_OC_MODE_PWM0 | TIMER_OC_SHADOW_ENABLE) << 8U);
#endif
    TIMER_CHCTL1(TIMER0) = TIMER_OC_MODE_PWM0 | TIMER_OC_SHADOW_ENABLE;

    // dead time, break disabled with high polarity, no automatic output, no protection
    TIMER_CCHP(TIMER0) = (uint32_t)TIMER_ROS_STATE_DISABLE | TIMER_IOS_STATE_DISABLE | (uint16_t)DEAD_TIME
        | TIMER_BREAK_POLARITY_HIGH | TIMER_OUTAUTO_DISABLE | TIMER_CCHP_PROT_OFF | TIMER_BREAK_DISABLE;

    TIMER_DMAINTEN(TIMER0) &= ~(uint32_t)(TIMER_INT_CMT | TIMER_INT_BRK);
    TIMER_CTL0(TIMER0) |= (uint32_t)TIMER_CTL0_ARSE;

    RCU_CLOCK_ENABLE(RCU_GPIOA);
    RCU_CLOCK_ENABLE(RCU_GPIOB);

    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    gpioSetMode(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_A_GPIO_LOW);
    gpioSetOutputOptions(PHASE_A_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_A_GPIO_LOW);

    gpioSetMode(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_B_GPIO_LOW);
    gpioSetOutputOptions(PHASE_B_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_B_GPIO_LOW);

    gpioSetMode(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_C_GPIO_LOW);
    gpioSetOutputOptions(PHASE_C_GPIO_PORT_LOW, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, PHASE_C_GPIO_LOW);

    gpioSetAf(PHASE_A_GPIO_PORT_LOW, GPIO_AF_2, PHASE_A_GPIO_LOW);
    gpioSetAf(PHASE_B_GPIO_PORT_LOW, GPIO_AF_2, PHASE_B_GPIO_LOW);
    gpioSetAf(PHASE_C_GPIO_PORT_LOW, GPIO_AF_2, PHASE_C_GPIO_LOW);

    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    gpioSetMode(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_A_GPIO_HIGH);
    gpioSetOutputOptions(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_A_GPIO_HIGH);

    gpioSetMode(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_B_GPIO_HIGH);
    gpioSetOutputOptions(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_B_GPIO_HIGH);

    gpioSetMode(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        PHASE_C_GPIO_HIGH);
    gpioSetOutputOptions(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
        PHASE_C_GPIO_HIGH);

    gpioSetAf(PHASE_A_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_A_GPIO_HIGH);
    gpioSetAf(PHASE_B_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_B_GPIO_HIGH);
    gpioSetAf(PHASE_C_GPIO_PORT_HIGH, GPIO_AF_2, PHASE_C_GPIO_HIGH);
}

void TIMER5_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_TIMER5);
    TIMER_CAR(TIMER5) = 0xFFFF;
    TIMER_PSC(TIMER5) = 35;
}

void TIMER13_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_TIMER13);
    TIMER_CAR(TIMER13) = 1000000 / LOOP_FREQUENCY_HZ;
    TIMER_PSC(TIMER13) = 71;
    NVIC_SetPriority(TIMER13_IRQn, 3);
    NVIC_EnableIRQ(TIMER13_IRQn);
    TIMER_CTL0(TIMER13) |= (uint32_t)TIMER_CTL0_CEN;
}

void TIMER15_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_TIMER15);
    TIMER_CAR(TIMER15) = 0xFFFF;
    TIMER_PSC(TIMER15) = 35;
    TIMER_CTL0(TIMER15) |= (uint32_t)(TIMER_CTL0_ARSE | TIMER_CTL0_CEN);

    NVIC_SetPriority(TIMER15_IRQn, 1);
    NVIC_EnableIRQ(TIMER15_IRQn);
}

void TIMER16_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_TIMER16);
    TIMER_CAR(TIMER16) = 0xFFFF;
    TIMER_PSC(TIMER16) = 71;
    TIMER_CTL0(TIMER16) |= (uint32_t)(TIMER_CTL0_ARSE | TIMER_CTL0_CEN);
}

void MX_DMA_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_DMA);

    NVIC_SetPriority(DMA_Channel1_2_IRQn, 3);
    NVIC_EnableIRQ(DMA_Channel1_2_IRQn);

}

void MX_GPIO_Init(void) { }

void UN_TIM_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_GPIOA);
    RCU_CLOCK_ENABLE(RCU_GPIOB);
    RCU_CLOCK_ENABLE(RCU_DMA);

    gpioSetOutputOptions(INPUT_PIN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, INPUT_PIN);
    gpioSetAf(INPUT_PIN_PORT, INPUT_PIN_AF, INPUT_PIN);

    DMA_CHPADDR(INPUT_DMA_CHANNEL) = (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER);
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&dma_buffer;

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
    #ifdef LED_USES_PA2
    RCU_CLOCK_ENABLE(RCU_TIMER2);
    TIMER_CAR(TIMER2) = 0xFFFF;
    TIMER_PSC(TIMER2) = 10;
    #endif
    #ifdef LED_USES_PB4
    RCU_CLOCK_ENABLE(RCU_TIMER14);
    TIMER_CAR(TIMER14) = 0xFFFF;
    TIMER_PSC(TIMER14) = 10;
    #endif
    /* enable a TIMER */

    //	LL_TIM_DisableARRPreload(IC_TIMER_REGISTER);
    TIMER_CTL0(IC_TIMER_REGISTER) &= ~(uint32_t)TIMER_CTL0_ARSE;

    // pwm input capture on CH0 (IC_TIMER_CHANNEL is CH0 on all targets): CH0 both edges
    // from its own input, CH1 rising edge from the opposite input, no filter or prescaler
    TIMER_CHCTL2(IC_TIMER_REGISTER) &= ~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP
        | TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP);
    TIMER_CHCTL0(IC_TIMER_REGISTER) = (TIMER_CHCTL0(IC_TIMER_REGISTER)
        & ~(uint32_t)(TIMER_CHCTL0_CH0MS | TIMER_CHCTL0_CH0CAPFLT | TIMER_CHCTL0_CH0CAPPSC
            | TIMER_CHCTL0_CH1MS | TIMER_CHCTL0_CH1CAPFLT | TIMER_CHCTL0_CH1CAPPSC))
        | TIMER_IC_SELECTION_DIRECTTI | ((uint32_t)TIMER_IC_SELECTION_INDIRECTTI << 8U);
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_IC_POLARITY_BOTH_EDGE | TIMER_CHCTL2_CH0EN
        | ((uint32_t)TIMER_IC_POLARITY_RISING << 4U) | TIMER_CHCTL2_CH1EN;

    // NVIC_SetPriority(TIMER2_IRQn, 0);
    //  NVIC_EnableIRQ(TIMER2_IRQn);
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;

    gpioSetMode(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, INPUT_PIN);
}

#ifdef USE_RGB_LED // has 3 color led
void LED_GPIO_init()
{
    /* GPIO Ports Clock Enable */
    RCU_CLOCK_ENABLE(RCU_GPIOA);

    gpioSetMode(RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
        RED_PIN);
    gpioSetOutputOptions(RED_PORT, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, RED_PIN);

    gpioSetMode(GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
        GREEN_PIN);
    gpioSetOutputOptions(GREEN_PORT, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, GREEN_PIN);

    gpioSetMode(BLUE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
        BLUE_PIN);
    gpioSetOutputOptions(BLUE_PORT, GPIO_OTYPE_PP,
        GPIO_OSPEED_50MHZ, BLUE_PIN);
}

void setIndividualRGBLed(uint8_t red, uint8_t green, uint8_t blue){

  if(red > 0){   
    GPIO_BOP(RED_PORT) = RED_PIN;
  }else{
    GPIO_BC(RED_PORT) = RED_PIN;
  }
  if(green > 0){
    GPIO_BOP(GREEN_PORT) = GREEN_PIN;
  }else{
    GPIO_BC(GREEN_PORT) = GREEN_PIN;
  }
  if(blue > 0){
    GPIO_BOP(BLUE_PORT) = BLUE_PIN;
  }else{
    GPIO_BC(BLUE_PORT) = BLUE_PIN;
  }
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
    TIMER_SWEVG(TIMER0) |= (uint32_t)TIMER_EVENT_SRC_UPG;
}

void resetInputCaptureTimer()
{
    TIMER_PSC(IC_TIMER_REGISTER) = 0;
    TIMER_CNT(IC_TIMER_REGISTER) = 0;
}

void initAfterJump()
{
    __enable_irq();
    FMC_WS = (FMC_WS & ~FMC_WS_WSCNT) | 2U; // 2 wait states
    FMC_WS |= FMC_WS_PFEN; // prefetch
}

void enableCorePeripherals()
{
    TIMER_CHCTL2(TIMER0) |= (uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH2EN
        | TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH1NEN | TIMER_CHCTL2_CH2NEN);

    /* Enable counter */
    TIMER_CTL0(TIMER0) |= (uint32_t)TIMER_CTL0_CEN;
    TIMER_CCHP(TIMER0) |= (uint32_t)TIMER_CCHP_POEN;

    /* Force update generation */
    // timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);

#ifndef BRUSHED_MODE
    TIMER_CTL0(COM_TIMER) |= (uint32_t)TIMER_CTL0_CEN; // commutation_timer priority 0
    TIMER_SWEVG(COM_TIMER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    TIMER_DMAINTEN(COM_TIMER) &= (~(uint32_t)TIMER_INT_UP); // disable interrupt until after polling mode
#endif
    TIMER_CTL0(UTILITY_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    TIMER_CTL0(INTERVAL_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    TIMER_SWEVG(INTERVAL_TIMER) |= (uint32_t)TIMER_EVENT_SRC_UPG;

    //                  // 10khz timer
    TIMER_CTL0(TEN_KHZ_TIMER) |= (uint32_t)TIMER_CTL0_CEN;
    TIMER_SWEVG(TEN_KHZ_TIMER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    TIMER_DMAINTEN(TEN_KHZ_TIMER) |= (uint32_t)TIMER_INT_UP;
#ifdef USE_ADC
    ADC_Init();
#endif

    CMP_CS |= CMP_CS_CMPEN;
    delayMicros(1000);

#ifdef USE_ADC_INPUT

#else
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CHCTL2_CH0EN; // IC_TIMER_CHANNEL
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN; // enable counter
#endif
    RCU_CLOCK_DISABLE(RCU_GPIOB);
    RCU_CLOCK_DISABLE(RCU_DMA);

    NVIC_SetPriority(EXTI4_15_IRQn, 2);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    EXTI_INTEN |= (uint32_t)EXTI_15;

    UN_TIM_Init();
}
