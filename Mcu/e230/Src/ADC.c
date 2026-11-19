/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
#include "ADC.h"

#include "functions.h"

#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
uint16_t ADCDataDMA[3];
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables

#ifdef USE_ADC_INPUT
    ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[1] / 2;
    ADC_raw_current = ADCDataDMA[2];
    ADC_raw_input = ADCDataDMA[0];

#else
    ADC_raw_temp = ADCDataDMA[2];
#ifdef PA6_VOLTAGE
    ADC_raw_volts = ADCDataDMA[1];
    ADC_raw_current = ADCDataDMA[0];
#else
    ADC_raw_volts = ADCDataDMA[0];
    ADC_raw_current = ADCDataDMA[1];
#endif
#endif
}

// same as adc_regular_channel_config() for ranks 0-5 with 55.5 cycle sample time
static inline void adcRegularChannel(uint32_t rank, uint32_t channel)
{
    ADC_RSQ2 = (ADC_RSQ2 & ~(ADC_RSQX_RSQN << (5U * rank))) | (channel << (5U * rank));
    if (channel < 10U) {
        ADC_SAMPT1 = (ADC_SAMPT1 & ~(ADC_SAMPTX_SPTN << (3U * channel)))
            | (ADC_SAMPLETIME_55POINT5 << (3U * channel));
    } else {
        ADC_SAMPT0 = (ADC_SAMPT0 & ~(ADC_SAMPTX_SPTN << (3U * (channel - 10U))))
            | (ADC_SAMPLETIME_55POINT5 << (3U * (channel - 10U)));
    }
}

void ADC_Init(void)
{
    RCU_CLOCK_ENABLE(RCU_GPIOA);
    RCU_CLOCK_ENABLE(RCU_ADC);
    RCU_CLOCK_ENABLE(RCU_DMA);
    /* ADC clock from APB2 / 6 */
    RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_ADCPSC) | RCU_ADC_CKAPB2_DIV6;
    RCU_CFG2 = (RCU_CFG2 & ~(RCU_CFG2_IRC28MDIV | RCU_CFG2_ADCPSC2)) | RCU_CFG2_ADCSEL;

    /* config the GPIO as analog mode */
    gpioSetMode(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
        VOLTAGE_ADC_PIN | CURRENT_ADC_PIN);

    /* ADC DMA channel: circular, 3 half words from ADC_RDATA, memory increment, low priority */
    dmaDeinit(DMA_CH0);
    DMA_CHPADDR(DMA_CH0) = (uint32_t)(&ADC_RDATA);
    DMA_CHMADDR(DMA_CH0) = (uint32_t)(&ADCDataDMA);
    DMA_CHCNT(DMA_CH0) = 3U;
    DMA_CHCTL(DMA_CH0) = DMA_PERIPHERAL_WIDTH_16BIT | DMA_MEMORY_WIDTH_16BIT | DMA_PRIORITY_LOW
        | DMA_CHXCTL_MNAGA | DMA_CHXCTL_CMEN;
    DMA_CHCTL(DMA_CH0) |= DMA_CHXCTL_CHEN;

    /* single scan of the regular group, right aligned, temp sensor on */
    ADC_CTL1 &= ~(ADC_CONTINUOUS_MODE | ADC_CTL1_DAL);
    ADC_CTL0 |= ADC_SCAN_MODE;
    ADC_RSQ0 = (ADC_RSQ0 & ~ADC_RSQ0_RL) | RSQ0_RL(3U - 1U);
    ADC_CTL1 |= ADC_CTL1_TSVREN;

    /* ADC regular channel config */
    adcRegularChannel(0, VOLTAGE_ADC_CHANNEL);
    adcRegularChannel(1, CURRENT_ADC_CHANNEL);
    adcRegularChannel(2, ADC_CHANNEL_16);

    /* software trigger for the regular group */
    ADC_CTL1 = (ADC_CTL1 & ~ADC_CTL1_ETSRC) | ADC_EXTTRIG_REGULAR_NONE | ADC_CTL1_ETERC;

    /* enable ADC interface */
    ADC_CTL1 |= ADC_CTL1_ADCON;
    delayMicros(1000);
    /* ADC calibration and reset calibration */
    ADC_CTL1 |= ADC_CTL1_RSTCLB;
    while (ADC_CTL1 & ADC_CTL1_RSTCLB) {
    }
    ADC_CTL1 |= ADC_CTL1_CLB;
    while (ADC_CTL1 & ADC_CTL1_CLB) {
    }

    /* ADC DMA function enable */
    ADC_CTL1 |= ADC_CTL1_DMA;
    /* ADC software trigger enable */
    ADC_CTL1 |= ADC_CTL1_SWRCST;
}
