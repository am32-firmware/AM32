#include "functions.h"
#include "targets.h"
#include "ADC.h"

char comp_pwm = 1;
uint8_t i = 0;

int main()
{
    adc_initialize(ADC1);
    uint8_t channels[] = {1, 2};
    adc_set_regular_sequence(ADC1, channels, 2);
    adc_enable(ADC1);
    adc_start(ADC1);
    adc_set_continuous_mode(ADC1, true);
    while(1) {
    }
}