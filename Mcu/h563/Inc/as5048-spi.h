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

void as5048_initialize(as5048_t* as5048);
void as5048_initialize_gpio(as5048_t* as5048);
void as5048_initialize_spi(as5048_t* as5048);

void as5048_read_all(as5048_t* as5048);

uint16_t as5048_spi_write_word(as5048_t* as5048, uint16_t word);

uint16_t as5048_read_reg(as5048_t* as5048, uint16_t word);
bool as5048_write_reg(as5048_t* as5048, uint16_t word);

