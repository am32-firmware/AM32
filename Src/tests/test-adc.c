#include "stm32h563xx.h"
#include "stm32h5xx_ll_adc.h"
#include "targets.h"
#include "ADC.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"
#include "vref.h"

uint16_t ADC_raw_temp = 900;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
int32_t converted_degrees;



#define __LL_ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__,\
                                  __TEMPSENSOR_ADC_DATA__,\
                                  __ADC_RESOLUTION__)\
((((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) != 0) ?       \
 (((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),     \
                                                   (__ADC_RESOLUTION__),          \
                                                   LL_ADC_RESOLUTION_12B)         \
                  * (__VREFANALOG_VOLTAGE__))                                     \
                 / TEMPSENSOR_CAL_VREFANALOG)                                     \
       - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                         \
    ) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)                    \
   ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) \
  ) + TEMPSENSOR_CAL1_TEMP                                                        \
 )                                                                                \
 :                                                                                \
 ((int32_t)LL_ADC_TEMPERATURE_CALC_ERROR)                                         \
)

int main()
{
    mcu_setup();
    utility_timer_initialize();
    utility_timer_enable();
    ADC_setup();
    int32_t a = (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR);
    while(1) {
        int32_t a = __LL_ADC_CONVERT_DATA_RESOLUTION(ADC_raw_temp, LL_ADC_RESOLUTION_12B, LL_ADC_RESOLUTION_12B);
        converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
    }
}