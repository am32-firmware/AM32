// AUTO-GENERATED from eeprom.json - DO NOT EDIT
// Generated: 2026-01-05T21:14:07.301891
// Schema version: 1.0.1

#include "main.h"

#pragma once

typedef union EEprom_u {
    struct {
        uint8_t reserved_0; // 0
        uint8_t eeprom_version; // 1
        uint8_t reserved_1; // 2
        struct {
            uint8_t major; // 3
            uint8_t minor; // 4
        } version;
        uint8_t max_ramp; // 5
        uint8_t minimum_duty_cycle; // 6
        uint8_t disable_stick_calibration; // 7
        uint8_t absolute_voltage_cutoff; // 8
        uint8_t current_P; // 9
        uint8_t current_I; // 10
        uint8_t current_D; // 11
        uint8_t active_brake_power; // 12
        char reserved_eeprom_3[4]; // 13
        uint8_t dir_reversed; // 17
        uint8_t bi_direction; // 18
        uint8_t use_sine_start; // 19
        uint8_t comp_pwm; // 20
        uint8_t variable_pwm; // 21
        uint8_t stuck_rotor_protection; // 22
        uint8_t advance_level; // 23
        uint8_t pwm_frequency; // 24
        uint8_t startup_power; // 25
        uint8_t motor_kv; // 26
        uint8_t motor_poles; // 27
        uint8_t brake_on_stop; // 28
        uint8_t stall_protection; // 29
        uint8_t beep_volume; // 30
        uint8_t telemetry_on_interval; // 31
        struct {
            uint8_t low_threshold; // 32
            uint8_t high_threshold; // 33
            uint8_t neutral; // 34
            uint8_t dead_band; // 35
        } servo;
        uint8_t low_voltage_cut_off; // 36
        uint8_t low_cell_volt_cutoff; // 37
        uint8_t rc_car_reverse; // 38
        uint8_t use_hall_sensors; // 39
        uint8_t sine_mode_changeover_thottle_level; // 40
        uint8_t drag_brake_strength; // 41
        uint8_t driving_brake_strength; // 42
        struct {
            uint8_t temperature; // 43
            uint8_t current; // 44
        } limits;
        uint8_t sine_mode_power; // 45
        uint8_t input_type; // 46
        uint8_t auto_advance; // 47
        uint8_t tune[128]; // 48
        struct {
            uint8_t can_node; // 176
            uint8_t esc_index; // 177
            uint8_t require_arming; // 178
            uint8_t telem_rate; // 179
            uint8_t require_zero_throttle; // 180
            uint8_t filter_hz; // 181
            uint8_t debug_rate; // 182
            uint8_t term_enable; // 183
            uint8_t reserved[8]; // 184
        } can;
    };
    uint8_t buffer[192];
} EEprom_t;

extern EEprom_t eepromBuffer;

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t* data, int length, uint32_t add);
