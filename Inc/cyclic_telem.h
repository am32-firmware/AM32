#ifndef CYCLIC_TELEM_H_
#define CYCLIC_TELEM_H_

#include "main.h"

#define CYCLIC_TELEM_INTERVAL 200  // 200 counts x 50us = 10ms at 20kHz ISR

typedef struct __attribute__((packed))
{
    uint8_t type;        // 0xCB - cyclic debug packet marker
    uint8_t gp_h;        // can_Gp MSB (int16, range [-8192, 8192])
    uint8_t gp_l;        // can_Gp LSB
    uint8_t gr_h;        // can_Gr MSB
    uint8_t gr_l;        // can_Gr LSB
    uint8_t base_duty_h; // base_duty_cycle MSB (uint16, range [0, 2000])
    uint8_t base_duty_l; // base_duty_cycle LSB
    uint8_t m_step_h;    // m_step MSB (int16, wraps 0-41)
    uint8_t m_step_l;    // m_step LSB
    uint8_t crc;         // CRC8 of bytes 0-8
} cyclic_telem_pkt_t;   // sizeof(cyclic_telem_pkt_t) = 10

void makeCyclicTelemPackage(int16_t gp, int16_t gr, uint16_t base_duty, int16_t step);

#endif /* CYCLIC_TELEM_H_ */