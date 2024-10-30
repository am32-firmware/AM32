#include "stm32h563xx.h"
#include "targets.h"
#include "ADC.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"
#include "vref.h"

uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;

int main()
{
    mcu_setup();
    vref_enable();
    utility_timer_initialize();
    utility_timer_enable();
    adc_initialize(VOLTAGE_ADC);

    uint8_t channels[] = {
        CURRENT_ADC_CHANNEL,
        VOLTAGE_ADC_CHANNEL,
        DIE_TEMPERATURE_ADC_CHANNEL
    };
    adc_set_regular_sequence(VOLTAGE_ADC, channels, sizeof(channels));

    adc_set_sample_time(VOLTAGE_ADC, CURRENT_ADC_CHANNEL, ADC_SAMPLE_TIME_640_5);
    adc_set_sample_time(VOLTAGE_ADC, VOLTAGE_ADC_CHANNEL, ADC_SAMPLE_TIME_640_5);
    adc_set_sample_time(VOLTAGE_ADC, DIE_TEMPERATURE_ADC_CHANNEL, ADC_SAMPLE_TIME_640_5);
    adc_set_continuous_mode(VOLTAGE_ADC, true);
    adc_enable(VOLTAGE_ADC);
    adc_start(VOLTAGE_ADC);
    while(1) {
        // while (!(VOLTAGE_ADC->ISR & ADC_ISR_EOS))
        // {
        //     // wait
        // }
        // volatile uint32_t d = VOLTAGE_ADC->DR;

        // // clear end of sequence (EOS) flag
        // VOLTAGE_ADC->ISR |= ADC_ISR_EOS;
        // adc_start(VOLTAGE_ADC);
    }
}