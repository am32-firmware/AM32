#pragma once

#include "spi.h"

#include <stdbool.h>

typedef union {
    struct {
        bool parity : 1;
        bool flag : 1;
        uint16_t data : 14;
    } __attribute__((__packed__)) pkg;
    uint16_t data;
} as5048_spi_pkg_t;

typedef struct {
    spi_t* spi;
} as5048_t;

typedef enum {
    AS5048_PARITY_ODD = 0,
    AS5048_PARITY_EVEN = 1,
} as5048_parity_e;

void as5048_initialize(as5048_t* as5048);
void as5048_initialize_gpio(as5048_t* as5048);
void as5048_initialize_spi(as5048_t* as5048);

// as5048 spi packages 15th bit, most significant bit,
// (MSB) is a parity bit (even parity)
// the parity is calculated over the 14 least significant
// bits (LSB), ie bits [0:14]
// the parity bit is 1 if there is an even number of `1` bits
// the parity bit is 0 if there is an odd number of `1` bits
as5048_parity_e as5048_parity(uint16_t word);

void as5048_read_all(as5048_t* as5048);

uint16_t as5048_spi_write_word(as5048_t* as5048, uint16_t word);

uint16_t as5048_read_reg(as5048_t* as5048, uint16_t word);
bool as5048_write_reg(as5048_t* as5048, uint16_t reg, uint16_t data);


uint16_t as5048_read_zero_position(as5048_t* as5048);
bool as5048_write_zero_position(as5048_t* as5048, uint16_t zero_position);

uint16_t as5048_read_angle(as5048_t* as5048);

