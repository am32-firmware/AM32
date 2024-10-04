//white wire
#pragma once

#include <stdbool.h>

#include "gpio.h"
#include "spi.h"
#include "stm32h563xx.h"


#define GATE_DRIVER_SPI_NSS_PORT GPIOF
#define GATE_DRIVER_SPI_NSS_PIN 6
#define GATE_DRIVER_SPI_NSS_AF 5

// purple wire
#define GATE_DRIVER_SPI_SCK_PORT GPIOF
#define GATE_DRIVER_SPI_SCK_PIN 7
#define GATE_DRIVER_SPI_SCK_AF 5
// grey wire
#define GATE_DRIVER_SPI_MISO_PORT GPIOF
#define GATE_DRIVER_SPI_MISO_PIN 8
#define GATE_DRIVER_SPI_MISO_AF 5
// blue wire
#define GATE_DRIVER_SPI_MOSI_PORT GPIOF
#define GATE_DRIVER_SPI_MOSI_PIN 9
#define GATE_DRIVER_SPI_MOSI_AF 5

#define GATE_DRIVER_SPI_PERIPH SPI5

#define DRV8323_WRITE (0 << 15)
#define DRV8323_READ (1 << 15)

#define DRV8323_REG_FAULT_STATUS_1 (0x0 << 11)
#define DRV8323_REG_VGS_STATUS_2 (0x1 << 11)
#define DRV8323_REG_DRIVER_CONTROL (0x2 << 11)
#define DRV8323_REG_GATE_DRIVE_HS (0x3 << 11)
#define DRV8323_REG_GATE_DRIVE_LS (0x4 << 11)
#define DRV8323_REG_OCP_CONTROL (0x5 << 11)
#define DRV8323_REG_CSA_CONTROL (0x6 << 11)

// Driver control register bitfield setting definitions
#define DRV8323_DRIVER_CONTROL_COAST (1 << 2)

// OCP CONTROL register bitfield setting definitions
#define DRV8323_OCP_TRETRY (1 << 10)

#define DRV8323_OCP_DEADTIME_50ns (0b00 << 8)
#define DRV8323_OCP_DEADTIME_100ns (0b01 << 8)
#define DRV8323_OCP_DEADTIME_200ns (0b10 << 8)
#define DRV8323_OCP_DEADTIME_400ns (0b11 << 8)

#define DRV8323_OCP_MODE_LATCHED (0b00 << 6)
#define DRV8323_OCP_MODE_AUTORETRY (0b01 << 6)
#define DRV8323_OCP_MODE_REPORTONLY (0b10 << 6)
#define DRV8323_OCP_MODE_IGNORE (0b11 << 6)

#define DRV8323_OCP_DEGLITCH_2us (0b00 << 4)
#define DRV8323_OCP_DEGLITCH_4us (0b01 << 4)
#define DRV8323_OCP_DEGLITCH_6us (0b10 << 4)
#define DRV8323_OCP_DEGLITCH_8us (0b11 << 4)

#define DRV8323_OCP_VDSLVL_60mV (0b0000 << 0)
#define DRV8323_OCP_VDSLVL_130mV (0b0001 << 0)
#define DRV8323_OCP_VDSLVL_200mV (0b0010 << 0)
#define DRV8323_OCP_VDSLVL_260mV (0b0011 << 0)
#define DRV8323_OCP_VDSLVL_310mV (0b0100 << 0)
#define DRV8323_OCP_VDSLVL_450mV (0b0101 << 0)
#define DRV8323_OCP_VDSLVL_530mV (0b0110 << 0)
#define DRV8323_OCP_VDSLVL_600mV (0b0111 << 0)
#define DRV8323_OCP_VDSLVL_680mV (0b1000 << 0)
#define DRV8323_OCP_VDSLVL_750mV (0b1001 << 0)
#define DRV8323_OCP_VDSLVL_940mV (0b1010 << 0)
#define DRV8323_OCP_VDSLVL_1130mV (0b1011 << 0)
#define DRV8323_OCP_VDSLVL_1300mV (0b1100 << 0)
#define DRV8323_OCP_VDSLVL_1500mV (0b1101 << 0)
#define DRV8323_OCP_VDSLVL_1700mV (0b1110 << 0)
#define DRV8323_OCP_VDSLVL_1880mV (0b1111 << 0)

// CSA CONTROL register bitfield setting definitions
#define DRV8323_CSA_CSA_FET (1 << 10)

#define DRV8323_CSA_VREF_DIV (1 << 9)

#define DRV8323_CSA_LS_REF (1 << 8)
#define DRV8323_CSA_CSA_GAIN_5VV (0b00 << 6)
#define DRV8323_CSA_CSA_GAIN_10VV (0b01 << 6)
#define DRV8323_CSA_CSA_GAIN_20VV (0b10 << 6)
#define DRV8323_CSA_CSA_GAIN_40VV (0b11 << 6)
#define DRV8323_CSA_DIS_SEN (1 << 5)
#define DRV8323_CSA_CSA_CAL_A (1 << 4)
#define DRV8323_CSA_CSA_CAL_B (1 << 3)
#define DRV8323_CSA_CSA_CAL_C (1 << 2)
#define DRV8323_CSA_SEN_LVL_250mV (0b00 << 0)
#define DRV8323_CSA_SEN_LVL_500mV (0b01 << 0)
#define DRV8323_CSA_SEN_LVL_750mV (0b10 << 0)
#define DRV8323_CSA_SEN_LVL_1000mV (0b11 << 0)

#define DRV8323_TRETRY (1 << 10)

#define DRV8323_FRAME_REG_MASK  (0b0111100000000000)
// each register is 11 bits / 11 data bits per word or frame
#define DRV8323_FRAME_DATA_MASK (0b0000011111111111)

#define DRV8323_REG_CSA_CONTROL_VALUE 0b01011000001



typedef struct {
    gpio_t* gpioEnable;
    gpio_t* gpioFault;
    spi_t* spi;
} drv8323_t;

extern drv8323_t DRV8323;

typedef enum
{
    DRV8323_DEADTIME_50ns = 0b00,
    DRV8323_DEADTIME_100ns = 0b01,
    DRV8323_DEADTIME_200ns = 0b10,
    DRV8323_DEADTIME_400ns = 0b11,
    
} drv8323Deadtime_e;

typedef enum
{
    DRV8323_TDRIVE_500ns = 0b00,
    DRV8323_TDRIVE_1000ns = 0b01,
    DRV8323_TDRIVE_2000ns = 0b10,
    DRV8323_TDRIVE_4000ns = 0b11,
    
} drv8323TDrive_e;

void drv8323_initialize(drv8323_t* drv);

void drv8323_reset(drv8323_t* drv);
void drv8323_enable(drv8323_t* drv);
void drv8323_disable(drv8323_t* drv);

bool drv8323_write_reg(drv8323_t* drv, uint16_t word);
uint16_t drv8323_read_reg(drv8323_t* drv, uint16_t word);
uint16_t drv8323_spi_write_word(drv8323_t* drv, uint16_t word);

void drv8323_enable_offset_calibration();
void drv8323_disable_offset_calibration();
void drv8323_set_tdrive(drv8323TDrive_e tdrive);
void drv8323_set_deadtime(drv8323Deadtime_e deadtime);
