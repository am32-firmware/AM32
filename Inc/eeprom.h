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
        uint16_t motor_kv; // 26-27 (changed to uint16_t)
        uint8_t motor_poles; // 28
        uint8_t brake_on_stop; // 29
        uint8_t stall_protection; // 30
        uint8_t beep_volume; // 31
        uint8_t telementry_on_interval; // 32
        struct {
            uint8_t low_threshold; // 33
            uint8_t high_threshold; // 34
            uint8_t neutral; // 35
            uint8_t dead_band; // 36
        } servo;
        uint8_t low_voltage_cut_off; // 37
        uint8_t low_cell_volt_cutoff; // 38
        uint8_t rc_car_reverse; // 39
        uint8_t use_hall_sensors; // 40
        uint8_t sine_mode_changeover_thottle_level; // 41
        uint8_t drag_brake_strength; // 42
        uint8_t driving_brake_strength; // 43
        struct {
            uint8_t temperature; // 44
            uint8_t current; // 45
        } limits;
        uint8_t sine_mode_power; // 46
        uint8_t input_type; // 47
        uint8_t auto_advance; // 48
        uint8_t tune[128]; // 49-176
        struct {
            uint8_t can_node; // 177
            uint8_t esc_index; // 178
            uint8_t require_arming; // 179
            uint8_t telem_rate; // 180
            uint8_t require_zero_throttle; // 181
            uint8_t filter_hz; // 182
            uint8_t debug_rate; // 183
            uint8_t term_enable; // 184
            uint8_t reserved[8]; // 185-192
        } can;
    };
    uint8_t buffer[192];
} EEprom_t;

extern EEprom_t eepromBuffer;

// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t* data, int length, uint32_t add);
