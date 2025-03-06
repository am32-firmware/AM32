// this example tests adc reading on pin 2
// of the main 4-pin jst-gh connector
#include "stm32h563xx.h"
#include "stm32h5xx_ll_adc.h"
#include "targets.h"
#include "ADC.h"
#include "functions.h"
#include "gpio.h"
#include "mcu.h"
#include "utility-timer.h"
#include "vref.h"

// these are declared just for compatibility
// (compilation) with AM32
uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
int32_t converted_degrees;


int main()
{
    mcu_setup(250);
    utility_timer_initialize();
    utility_timer_enable();

    vref_enable();

    gpio_t gpioMainP1 = DEF_GPIO(MAIN_P1_PORT, MAIN_P1_PIN, 0, GPIO_OUTPUT);
    gpio_initialize(&gpioMainP1);
    gpio_set(&gpioMainP1);

    adc_initialize(ADC1);
    uint8_t channels[] = {
        2
    };
    adc_set_regular_sequence(ADC1, channels, sizeof(channels));
    adc_set_sample_time(ADC1, 2, ADC_SAMPLE_TIME_640_5);
    adc_set_continuous_mode(ADC1, true);
    adc_enable(ADC1);
    adc_start(ADC1);

    while(1) {
        // check ADC data register (DR) for adc value
    }
}