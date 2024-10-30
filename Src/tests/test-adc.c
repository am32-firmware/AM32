#include "stm32h563xx.h"
#include "targets.h"
#include "ADC.h"
#include "functions.h"
#include "utility-timer.h"
#include "vref.h"

char comp_pwm = 1;
uint8_t i = 0;


int main()
{
    vref_enable();
    utility_timer_initialize();
    utility_timer_enable();
    adc_initialize(VOLTAGE_ADC);

    // uint8_t channels[] = {
    //     CURRENT_ADC_CHANNEL,
    //     VOLTAGE_ADC_CHANNEL};
    // adc_set_regular_sequence(VOLTAGE_ADC, channels, 2);
    // uint8_t channels[] = {
    // DIE_TEMPERATURE_ADC_CHANNEL};
        uint8_t channels[] = {
    VOLTAGE_ADC_CHANNEL};
    adc_set_regular_sequence(VOLTAGE_ADC, channels, 1);
    adc_set_sample_time(VOLTAGE_ADC, DIE_TEMPERATURE_ADC_CHANNEL, ADC_SAMPLE_TIME_640_5);
    // adc_set_continuous_mode(VOLTAGE_ADC, true);
    adc_enable(VOLTAGE_ADC);
    adc_start(VOLTAGE_ADC);
    while(1) {
        while (!(VOLTAGE_ADC->ISR & ADC_ISR_EOS))
        {
            // wait
        }
        volatile uint32_t d = VOLTAGE_ADC->DR;

        // clear end of sequence (EOS) flag
        VOLTAGE_ADC->ISR |= ADC_ISR_EOS;
        adc_start(VOLTAGE_ADC);
    }
}