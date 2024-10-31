#include "stm32h563xx.h"
#include "stm32h5xx_ll_adc.h"
#include "targets.h"
#include "ADC.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"
#include "vref.h"

uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
int32_t converted_degrees;

int main()
{
    mcu_setup();
    utility_timer_initialize();
    utility_timer_enable();
    ADC_setup();
    while(1) {
        converted_degrees = __LL_ADC_CALC_TEMPERATURE(2500, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
    }
}