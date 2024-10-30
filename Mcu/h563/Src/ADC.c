#include "ADC.h"
#include "dma.h"
#include "functions.h"
#include "stm32h563xx.h"
#include "stm32h5xx_ll_dma.h"
#include "targets.h"
#define ADC_DMA_LENGTH 3
uint16_t ADCDataDMA[ADC_DMA_LENGTH];

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables

  // CURRENT_ADC_CHANNEL,
  // VOLTAGE_ADC_CHANNEL,
  // DIE_TEMPERATURE_ADC_CHANNEL
  ADC_raw_temp = ADCDataDMA[2];
  ADC_raw_volts = ADCDataDMA[1];
  ADC_raw_current = ADCDataDMA[0];
}

void adc_dma_cb(dmaChannel_t* dma)
{
    dma->ref->CFCR |= DMA_CFCR_TCF;
    ADC_DMA_Callback();
}
void adc_dma_initialize(ADC_TypeDef* adc)
{
    dmaChannel_t* dma = &dmaChannels[15];
    
    NVIC_SetPriority(dma->irqn, 0);
    NVIC_EnableIRQ(dma->irqn);

    // set the source address
    dma->ref->CSAR = (uint32_t)&adc->DR;
    // set the destination address
    dma->ref->CDAR = (uint32_t)&ADCDataDMA[0];
    // set destination incrementing burst
    dma->ref->CTR1 |= DMA_CTR1_DINC;

    // set the transfer length
    dma->ref->CBR1 = 2*ADC_DMA_LENGTH;
    // set the block repeated destination address offset
    dma->ref->CBR2 |= 2*ADC_DMA_LENGTH << DMA_CBR2_BRDAO_Pos;
    dma->ref->CBR1 |= DMA_CBR1_BRDDEC;

    // configure single LLI to run repeatedly
    dma->ref->CLLR = 0x08000004 & DMA_CLLR_LA_Msk;

    // set the hardware request selections
    dma->ref->CTR2 |= LL_GPDMA1_REQUEST_ADC1;

    // enable transfer complete interrupt
    dma->ref->CCR |= DMA_CCR_TCIE;
    dma->callback = adc_dma_cb;
    dma->userParam = (uint32_t)adc;

    // set source data width to half word (16 bit)
    dma->ref->CTR1 |= 0b01 << DMA_CTR1_SDW_LOG2_Pos;
    // set destination data width to half word (16 bit)
    dma->ref->CTR1 |= 0b01 << DMA_CTR1_DDW_LOG2_Pos;

    // lastly enable the dma channel
    dma->ref->CCR |= DMA_CCR_EN;

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

  adc_dma_initialize(adc);
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
  adc->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

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