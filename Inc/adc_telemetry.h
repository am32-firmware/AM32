/*
 * adc_telemetry.h
 *
 * ADC processing and telemetry functions
 */

#ifndef INC_ADC_TELEMETRY_H_
#define INC_ADC_TELEMETRY_H_

#include <stdint.h>
#include "main.h"

// ADC variables
extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;
extern uint16_t ADC_smoothed_input;
extern int16_t converted_degrees;
extern uint8_t degrees_celsius;
extern uint16_t battery_voltage;
extern int16_t actual_current;
extern int32_t smoothed_raw_current;
extern uint16_t low_voltage_count;
extern char LOW_VOLTAGE_CUTOFF;
extern char cell_count;
extern uint16_t VOLTAGE_DIVIDER;
extern int32_t consumed_current;
extern uint8_t temperature_offset;
extern char send_telemetry;
extern char send_esc_info_flag;
extern uint16_t tenkhzcounter;
extern char dshot_extended_telemetry;
extern uint16_t send_extended_dshot;

// Current sense reading buffer
extern const uint8_t numReadings;
extern uint8_t readIndex;
extern uint32_t total;
extern uint16_t readings[50];

// Telemetry
void makeTelemPackage(int8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm);
void makeInfoPacket(void);
void send_telem_DMA(uint8_t count);
uint16_t getSmoothedCurrent(void);

// Inline ADC processing functions
static inline void processADCReadings(void) {
#if defined(STMICRO)
    ADC_DMA_Callback();
    LL_ADC_REG_StartConversion(ADC1);
    converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
#endif
#ifdef MCU_GDE23
    ADC_DMA_Callback();
    converted_degrees = ((int32_t)(357.5581395348837f * (1 << 16)) - ADC_raw_temp * (int32_t)(0.18736373546511628f * (1 << 16))) >> 16;
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif
#ifdef ARTERY
    ADC_DMA_Callback();
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
#ifdef WCH
    startADCConversion();
    converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
    degrees_celsius = converted_degrees;
    battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
    smoothed_raw_current = getSmoothedCurrent();
    actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
    if (actual_current < 0) {
        actual_current = 0;
    }
}

static inline void checkLowVoltage(void) {
    if (eepromBuffer.low_voltage_cut_off == 1) {  
        if (battery_voltage < (cell_count * low_cell_volt_cutoff)) {
            low_voltage_count++;
        } else {
            if (!LOW_VOLTAGE_CUTOFF) {
                low_voltage_count = 0;
            }
        }
    }
    if (eepromBuffer.low_voltage_cut_off == 2) {
        if (battery_voltage < eepromBuffer.absolute_voltage_cutoff) {
            low_voltage_count++;    
        } else {
            if (!LOW_VOLTAGE_CUTOFF) {
                low_voltage_count = 0;
            }
        }
    }
    if (low_voltage_count > (10000 - (stepper_sine * 9900))) {
        LOW_VOLTAGE_CUTOFF = 1;
        input = 0;
        allOff();
        maskPhaseInterrupts();
        running = 0;
        zero_input_count = 0;
        armed = 0;
    }
}

static inline void updateTelemetryData(void) {
    consumed_current += (actual_current << 16) / 360;
    switch (dshot_extended_telemetry) {
    case 1:
        send_extended_dshot = 0b0010 << 8 | degrees_celsius;
        dshot_extended_telemetry = 2;
        break;
    case 2:
        send_extended_dshot = 0b0110 << 8 | (uint8_t)actual_current / 50;
        dshot_extended_telemetry = 3;
        break;
    case 3:
        send_extended_dshot = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
        dshot_extended_telemetry = 1;
        break;
    }
}

static inline void detectCellCount(void) {
    if (battery_voltage > 1100 && battery_voltage < 1350) {
        cell_count = 3;
    }
    if (battery_voltage > 1420 && battery_voltage < 1800) {
        cell_count = 4;
    }
    if (battery_voltage > 1850 && battery_voltage < 2150) {
        cell_count = 5;
    }
    if (battery_voltage > 2150 && battery_voltage < 2550) {
        cell_count = 6;
    }
    if (battery_voltage > 850 && battery_voltage < 1100) {
        cell_count = 2;
    }
    if (battery_voltage > 600 && battery_voltage < 850) {
        cell_count = 1;
    }
}

#endif /* INC_ADC_TELEMETRY_H_ */