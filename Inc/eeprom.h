#include "main.h"

#pragma once

typedef union EEprom_u {
    struct {
        uint8_t reserved_0; //0
        uint8_t eeprom_version; //1
        uint8_t reserved_1; //2
        struct {        
            uint8_t major; //3
            uint8_t minor; //4
        } version;
        char firmware_name[12]; //5-16
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
        uint8_t telementry_on_interval; // 31
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
        uint8_t reserved_2[5]; //47-51
        uint8_t tune[124]; // 52-175
    };
    uint8_t buffer[184];
} EEprom_t;

extern EEprom_t eepromBuffer;

// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t* data, int length, uint32_t add);