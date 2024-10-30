#include "ADC.h"
#include "functions.h"
#include "stm32h563xx.h"

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

// #ifdef USE_ADC_INPUT
//     ADC_raw_temp = ADCDataDMA[3];
//     ADC_raw_volts = ADCDataDMA[1] / 2;
//     ADC_raw_current = ADCDataDMA[2];
//     ADC_raw_input = ADCDataDMA[0];

// #else
//     ADC_raw_temp = ADCDataDMA[2];
//     if (VOLTAGE_ADC_PIN > CURRENT_ADC_PIN) {
//         ADC_raw_volts = ADCDataDMA[1];
//         ADC_raw_current = ADCDataDMA[0];
//     } else {
//         ADC_raw_volts = ADCDataDMA[0];
//         ADC_raw_current = ADCDataDMA[1];
//     }
// #endif
}


void adc_set_regular_sequence(ADC_TypeDef* adc, uint8_t* channels, uint8_t length)
{
  uint32_t sqr1 = length-1;
  uint32_t sqr2 = 0;
  uint32_t sqr3 = 0;
  uint32_t sqr4 = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    if (i < 4) {
      sqr1 |= channels[i] << 6*(i + 1);
    } else if (i < 9) {
      sqr2 |= channels[i] << 6*(i - 4);
    } else if (i < 14) {
      sqr3 |= channels[i] << 6*(i - 9);
    } else if (i < 16) {
      sqr4 |= channels[i] << 6*(i - 14);
    }
  }
  adc->SQR1 = sqr1;
  adc->SQR2 = sqr2;
  adc->SQR3 = sqr3;
  adc->SQR4 = sqr4;
}

// ADEN bit cannot be set when ADCAL is set and during four ADC clock cycles after the
// ADCAL bit is cleared by hardware (end of the calibration).
void adc_enable(ADC_TypeDef* adc)
{
  adc->ISR |= ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  while (!(adc->ISR & ADC_ISR_ADRDY));
}

void adc_start(ADC_TypeDef* adc)
{
  adc->CR |= ADC_CR_ADSTART;
}

void adc_initialize(ADC_TypeDef* adc)
{
  // set adc input clock prescaler to divide by 2 (for a final clock of HCLK/4 = 42.5MHz)
  // ADC12_COMMON->CCR |= 0b0001 << ADC_CCR_PRESC_Pos;
  
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  delayMicros(100);

  // select the adc clock as HCLK/4
  ADC12_COMMON->CCR |= 0b11 << ADC_CCR_CKMODE_Pos;
  // // select the adc clock as HCLK/2
  // ADC12_COMMON->CCR |= 0b10 << ADC_CCR_CKMODE_Pos;
  
  // exit deep power down
  adc->CR &= ~ADC_CR_DEEPPWD;

  // enable adc voltage regulator
  // write 0b00 to ADVREGEN
  adc->CR = adc->CR & ~ADC_CR_ADVREGEN_Msk;
  // write 0b01 to ADVREGEN
  adc->CR |= 0b01 << ADC_CR_ADVREGEN_Pos;

  // worst case startup time for adc voltage regulator is 10us
  delayMicros(100);

  // adc calibration
  adc->CR |= ADC_CR_ADCAL;
  while (adc->CR & ADC_CR_ADCAL);

  // enable overwriting old data during overrun
  // (overruns should not happen, so we should not need this)
  adc->CFGR |= ADC_CFGR_OVRMOD;

  // enable temperature sensor
  ADC12_COMMON->CCR |= ADC_CCR_TSEN;

  // configure dma circular mode and endable dma request generation
  // adc->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

}

void adc_set_continuous_mode(ADC_TypeDef* adc, bool enabled)
{
  if (enabled) {
    adc->CFGR |= ADC_CFGR_CONT;
  } else {
    adc->CFGR &= ~ADC_CFGR_CONT;
  }
}

void adc_set_sample_time(ADC_TypeDef* adc, uint8_t channel, adcSampleTime_e sample_time)
{
  if (channel < 10) {
    adc->SMPR1 &= ~(0b111 << 3*channel);
    adc->SMPR1 |= sample_time << 3*channel;
  } else {
    channel -= 10;
    adc->SMPR2 &= ~(0b111 << 3*channel);
    adc->SMPR2 |= sample_time << 3*channel;
  }
}