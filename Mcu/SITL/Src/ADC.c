/*
  ADC.c - SITL. Converts the simulation sensor values into the raw ADC
  counts the firmware expects, inverting the conversions in main.c
 */

#include "ADC.h"

#include "sitl.h"
#include "targets.h"

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;

void ADC_DMA_Callback(void)
{
    sitl_sensors_t s;
    sitl_sensors_read(&s);

    // main.c: battery_voltage(10mV) = raw * 3300 / 4095 * VOLTAGE_DIVIDER / 100
    const float pin_mv_volts = s.bus_voltage * 1000.0f * 10.0f / TARGET_VOLTAGE_DIVIDER;
    ADC_raw_volts = (uint16_t)(pin_mv_volts * 4095.0f / 3300.0f + 0.5f);

    // main.c: actual_current(10mA) = ((raw*3300/41) - CURRENT_OFFSET*100) / MILLIVOLT_PER_AMP
    float pin_mv_current = s.bus_current * MILLIVOLT_PER_AMP + CURRENT_OFFSET;
    if (pin_mv_current < 0) {
        pin_mv_current = 0;
    }
    ADC_raw_current = (uint16_t)(pin_mv_current * 4095.0f / 3300.0f + 0.5f);

    // __LL_ADC_CALC_TEMPERATURE is a pass through in SITL
    ADC_raw_temp = (uint16_t)(s.temperature_c + 0.5f);
}

void ADC_Init(void) { }
void enableADC_DMA(void) { }
void activateADC(void) { }
