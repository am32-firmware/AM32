#include "main.h"

#ifndef KISS_TELEMETRY_H_
#define KISS_TELEMETRY_H_

typedef struct __attribute__((packed))
{
    uint8_t temperature; // temperature in Celcius
    uint8_t voltage_h; // voltage in centivolts
    uint8_t voltage_l;
    uint8_t current_h; // current in centiamps
    uint8_t current_l;
    uint8_t consumption_h; // accumulated current consumption in mAH
    uint8_t consumption_l;
    uint8_t erpm_h; // eRPM * 100, so 1 in the packet means 100 eRPM
    uint8_t erpm_l;
    uint8_t crc;
}
kiss_telem_pkt_t; // sizeof(kiss_telem_pkt_t) = 10

extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm);
void makeInfoPacket(void);

#endif