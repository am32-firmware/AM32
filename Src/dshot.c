/*
 * dshot.c
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "dshot.h"
#include "IO.h"
#include "common.h"
#include "functions.h"
#include "sounds.h"
#include "targets.h"
#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#endif

int dpulse[16] = { 0 };

const char gcr_encode_table[16] = {
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111
};

typedef struct {
    uint16_t temp_count;
    uint16_t voltage_count;
    uint16_t current_count;
    uint8_t last_sent_extended;
} dshot_telem_scheduler_t;

static dshot_telem_scheduler_t telem_scheduler = {0};

// These divisors create ratios regardless of input rate:
// - Temperature: every 200 calls (4Hz at 800Hz input)
// - Voltage: every 200 calls (4Hz at 800Hz input)
// - Current: every 40 calls (20Hz at 800Hz input)
// - eRPM: fills all other slots

#define TEMP_EDT_RATE_DIVISOR    200
#define VOLTAGE_EDT_RATE_DIVISOR 200
#define CURRENT_EDT_RATE_DIVISOR 40


char send_EDT_init;
char send_EDT_deinit;
char EDT_ARM_ENABLE = 0;
char EDT_ARMED = 0;
int shift_amount = 0;
uint32_t gcrnumber;
extern int zero_crosses;
extern char send_telemetry;
extern uint8_t max_duty_cycle_change;
int dshot_full_number;
extern char play_tone_flag;
extern char send_esc_info_flag;
uint8_t command_count = 0;
uint8_t last_command = 0;
uint8_t high_pin_count = 0;
uint32_t gcr[37] = { 0 };
uint16_t dshot_frametime;
uint16_t dshot_goodcounts;
uint16_t dshot_badcounts;
uint8_t dshot_extended_telemetry = 0;
uint16_t processtime = 0;
uint16_t halfpulsetime = 0;

uint8_t programming_mode;
uint16_t position;
uint8_t  new_byte;

void computeDshotDMA()
{
    dshot_frametime = dma_buffer[31] - dma_buffer[0];
    halfpulsetime = dshot_frametime >> 5;
    if ((dshot_frametime > dshot_frametime_low) && (dshot_frametime < dshot_frametime_high)) {
			signaltimeout = 0;
        for (int i = 0; i < 16; i++) {
            // note that dma_buffer[] is uint32_t, we cast the difference to uint16_t to handle
            // timer wrap correctly
            const uint16_t pdiff = dma_buffer[(i << 1) + 1] - dma_buffer[(i << 1)];
            dpulse[i] = (pdiff > halfpulsetime);
        }
        uint8_t calcCRC = ((dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3 | (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2 | (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1 | (dpulse[3] ^ dpulse[7] ^ dpulse[11]));
        uint8_t checkCRC = (dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15]);

        if (!armed) {
            if (dshot_telemetry == 0) {
                if (getInputPinState()) { // if the pin is high for 100 checks between
                                          // signal pulses its inverted
                    high_pin_count++;
                    if (high_pin_count > 100) {
                        dshot_telemetry = 1;
                    }
                }
            }
        }
        if (dshot_telemetry) {
            checkCRC = ~checkCRC + 16;
        }

        int tocheck = (dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7 | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3 | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);

        if (calcCRC == checkCRC) {
            signaltimeout = 0;
            dshot_goodcounts++;
            if (dpulse[11] == 1) {
                send_telemetry = 1;
            }
            if(programming_mode > 0){  
                if(programming_mode == 1){ // begin programming mode
                    position = tocheck;    // eepromBuffer position
                    programming_mode = 2;
                    return;
                }
               if(programming_mode == 2){
                    new_byte = tocheck;   // new value of setting
                    programming_mode = 3;
                    return;
                }
                if(programming_mode == 3){
                    if(tocheck == 37){  // commit new values to eeprom. must use save settings to make permanent.
                    eepromBuffer.buffer[position] = new_byte;
                    programming_mode = 0;
                  }
                }
                return; // don't process dshot signal when in programming mode
            }
            if (tocheck > 47) {
                if (EDT_ARMED) {
                    newinput = tocheck;
                    dshotcommand = 0;
                    command_count = 0;
                    return;
                }
            }

            if ((tocheck <= 47) && (tocheck > 0)) {
                newinput = 0;
                dshotcommand = tocheck; //  todo
            }
            if (tocheck == 0) {
                if (EDT_ARM_ENABLE == 1) {
                    EDT_ARMED = 0;
                }
#if DRONECAN_SUPPORT
                if (DroneCAN_active()) {
                    // allow DroneCAN to override DShot input
                    return;
                }
#endif
                newinput = 0;
                dshotcommand = 0;
                command_count = 0;
            }

            if ((dshotcommand > 0) && (running == 0) && armed) {
                if (dshotcommand != last_command) {
                    last_command = dshotcommand;
                    command_count = 0;
                }
                if (dshotcommand <= 5) { // beacons
                    command_count = 6; // go on right away
                }
                command_count++;
                if (command_count >= 6) {
                    command_count = 0;
                    switch (dshotcommand) { // todo

                    case 1:
                        play_tone_flag = 1;
                        break;
                    case 2:
                        play_tone_flag = 2;
                        break;
                    case 3:
                        play_tone_flag = 3;
                        break;
                    case 4:
                        play_tone_flag = 4;
                        break;
                    case 5:
                        play_tone_flag = 5;
                        break;
                    case 6:
                        send_esc_info_flag = 1;
                        break;
                    case 7:
                        eepromBuffer.dir_reversed = 0;
                        forward = 1 - eepromBuffer.dir_reversed;
                        //	play_tone_flag = 1;
                        break;
                    case 8:
                        eepromBuffer.dir_reversed = 1;
                        forward = 1 - eepromBuffer.dir_reversed;
                        //	play_tone_flag = 2;
                        break;
                    case 9:
                        eepromBuffer.bi_direction = 0;
                        break;
                    case 10:
                        eepromBuffer.bi_direction = 1;
                        break;
                    case 12:
                        saveEEpromSettings();
                        play_tone_flag = 1 + eepromBuffer.dir_reversed;
                        //	NVIC_SystemReset();
                        break;
                    case 13:
                        dshot_extended_telemetry = 1;
                        send_EDT_init = 1;
                        if (EDT_ARM_ENABLE == 1) {
                            EDT_ARMED = 1;
                        }
                        break;
                    case 14:
                        dshot_extended_telemetry = 0;
                        send_EDT_deinit = 1;
                        break;
                    case 20:
                        forward = 1 - eepromBuffer.dir_reversed;
                        break;
                    case 21:
                        forward = eepromBuffer.dir_reversed;
                        break;
                    case 36:
                        programming_mode = 1;
              //          armed = 0;           // disarm when entering programming mode
                        break;
                    }
                    last_dshot_command = dshotcommand;
                    dshotcommand = 0;
                }
            }
        } else {
            dshot_badcounts++;
            programming_mode = 0;
        }
    }
}

void make_dshot_package(uint16_t com_time)
{
    uint16_t extended_frame_to_send = 0;

    if (dshot_extended_telemetry) {
        // Only send extended telemetry if last frame wasn't extended. This ensures eRPM interleaving.
        if (telem_scheduler.last_sent_extended) {
            telem_scheduler.last_sent_extended = 0;

        } else {
            telem_scheduler.current_count++;
            telem_scheduler.voltage_count++;
            telem_scheduler.temp_count++;

            if (telem_scheduler.current_count >= CURRENT_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0110 << 8 | (uint8_t)(actual_current / 50);
                telem_scheduler.current_count = 0;
            }
            else if (telem_scheduler.voltage_count >= VOLTAGE_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
                telem_scheduler.voltage_count = 0;
            }
            else if (telem_scheduler.temp_count >= TEMP_EDT_RATE_DIVISOR) {
                extended_frame_to_send = 0b0010 << 8 | degrees_celsius;
                telem_scheduler.temp_count = 0;
            }
        }
    }
      if(send_EDT_init){
        extended_frame_to_send = 0b111000000000;
        send_EDT_init = 0;
      }
      if(send_EDT_deinit){
        extended_frame_to_send = 0b111011111111;
        send_EDT_deinit = 0;
      }
    
    if (extended_frame_to_send > 0) {
        dshot_full_number = extended_frame_to_send;
        telem_scheduler.last_sent_extended = 1;

    } else {
        if (!running) {
            com_time = 65535;
        }
        //	calculate shift amount for data in format eee mmm mmm mmm, first 1 found
        // in first seven bits of data determines shift amount
        // this allows for a range of up to 65408 microseconds which would be
        // shifted 0b111 (eee) or 7 times.
        for (int i = 15; i >= 9; i--) {
            if (com_time >> i == 1) {
                shift_amount = i + 1 - 9;
                break;
            } else {
                shift_amount = 0;
            }
        }
        // shift the commutation time to allow for expanded range and put shift
        // amount in first three bits
        dshot_full_number = ((shift_amount << 9) | (com_time >> shift_amount));
    }
    // calculate checksum
    uint16_t csum = 0;
    uint16_t csum_data = dshot_full_number;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }
    csum = ~csum; // invert it
    csum &= 0xf;

    dshot_full_number = (dshot_full_number << 4) | csum; // put checksum at the end of 12 bit dshot number

    // GCR RLL encode 16 to 20 bit

    gcrnumber = gcr_encode_table[(dshot_full_number >> 12)]
            << 15 // first set of four digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 8))]
            << 10 // 2nd set of 4 digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))]
            << 5 // 3rd set of four digits
        | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))]; // last four digits
// GCR RLL encode 20 to 21bit output
#if defined(MCU_F051) || defined(MCU_F031) || defined(MCU_CH32V203)
    gcr[1 + buffer_padding] = 64;
    for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
        gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6))
            << 6; // exclusive ored with number before it multiplied by 64 to match
                  // output timer.
    }
    gcr[buffer_padding] = 0;
#else
    gcr[1 + buffer_padding] = 128;
    for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
        gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 7))
            << 7; // exclusive ored with number before it multiplied by 64 to match
                  // output timer.
    }
    gcr[buffer_padding] = 0;
#endif
}
