/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
#include "ADC.h"

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

void enableADC_DMA()
{ // enables channel

    //	NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
    //	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

//     LL_DMA_ConfigAddresses(
//         DMA1, LL_DMA_CHANNEL_1,
//         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
//         (uint32_t)&ADCDataDMA, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

//     /* Set DMA transfer size */
// #ifdef USE_ADC_INPUT
//     LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
// #else
//     LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 3);

// #endif
//     // LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_1);

//     // LL_DMA_EnableIT_TE(DMA1,LL_DMA_CHANNEL_1);

//     /*## Activation of DMA
//      * #####################################################*/
//     /* Enable the DMA transfer */
//     LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void activateADC()
{ // called right after enable regular conversions are
    // started by software and DMA interrupt happens at end of
    // transfer

    // __IO uint32_t wait_loop_index = 0;

    // LL_ADC_StartCalibration(ADC1);

    // /* Poll for ADC effectively calibrated */

    // while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
    // }
    // wait_loop_index = (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES >> 1);
    // while (wait_loop_index != 0) {
    //     wait_loop_index--;
    // }
    // LL_ADC_Enable(ADC1);
    // while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) {
    // }
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

void adc_enable(ADC_TypeDef* adc)
{
  adc->CR |= ADC_CR_ADEN;
  while (!adc->ISR & ADC_ISR_ADRDY);
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
  for (long i = 0; i < 10000; i++)
  {
    asm("nop");
  }
  // select the adc clock as HCLK/2
  ADC12_COMMON->CCR |= 0b11 << ADC_CCR_CKMODE_Pos;
  adc->CR &= ~ADC_CR_DEEPPWD;
  // enable temperature sensor
  ADC12_COMMON->CCR |= 1 << 23;

  // enable adc voltage regulator
  // write 0b00 to ADVREGEN
  adc->CR = adc->CR & ~ADC_CR_ADVREGEN_Msk;
  // write 0b01 to ADVREGEN
  adc->CR |= 0b01 << ADC_CR_ADVREGEN_Pos;

  // worst case startup time for adc voltage regulator is 10us
  for (long i = 0; i < 10000; i++)
  {
    asm("nop");
  }

  // adc calibration
  adc->CR |= ADC_CR_ADCAL;
  while (adc->CR & ADC_CR_ADCAL);

  // enable overwriting old data during overrun
  // (overruns should not happen, so we should not need this)
  adc->CFGR |= ADC_CFGR_OVRMOD;

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

void ADC_Init(void)
{




//     LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
//     LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };

//     LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

//     /* Peripheral clock enable */
//     LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

//     LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//     /**ADC GPIO Configuration
//     PA3   ------> ADC_IN3
//     PA6   ------> ADC_IN6
//     */
// #ifdef USE_ADC_INPUT
//     GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// #endif

//     GPIO_InitStruct.Pin = CURRENT_ADC_PIN;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin = VOLTAGE_ADC_PIN;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//     LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
//         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

//     LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

//     LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

//     LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

//     LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

//     LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

//     LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

// #ifdef USE_ADC_INPUT
//     LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
// #endif

//     LL_ADC_REG_SetSequencerChAdd(ADC1, VOLTAGE_ADC_CHANNEL);
//     /** Configure Regular Channel
//      */
//     LL_ADC_REG_SetSequencerChAdd(ADC1, CURRENT_ADC_CHANNEL);
//     /** Configure Regular Channel
//      */
//     LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
//     /** Configure Internal Channel
//      */
//     LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
//         LL_ADC_PATH_INTERNAL_TEMPSENSOR);
//     /** Configure the global features of the ADC (Clock, Resolution, Data
//      * Alignment and number of conversion)
//      */
//     ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
//     ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//     ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//     ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//     LL_ADC_Init(ADC1, &ADC_InitStruct);
//     ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//     ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//     ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//     ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
//     ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
//     LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//     // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_FALLING);
//     LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
//     LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_71CYCLES_5);
//     LL_ADC_DisableIT_EOC(ADC1);
//     LL_ADC_DisableIT_EOS(ADC1);
}

void adc_set_sample_time(ADC_TypeDef* adc, uint8_t channel, uint8_t sample_time)
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