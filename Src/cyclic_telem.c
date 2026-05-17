#include "cyclic_telem.h"
#include "kiss_telemetry.h"

extern uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen);

void makeCyclicTelemPackage(int16_t gp, int16_t gr, uint16_t base_duty, int16_t step, uint16_t duty)
{
    cyclic_telem_pkt_t* pkt = (cyclic_telem_pkt_t*)aTxBuffer;

    pkt->type        = 0xCB;
    pkt->gp_h        = (uint8_t)((uint16_t)gp >> 8);
    pkt->gp_l        = (uint8_t)((uint16_t)gp & 0xFF);
    pkt->gr_h        = (uint8_t)((uint16_t)gr >> 8);
    pkt->gr_l        = (uint8_t)((uint16_t)gr & 0xFF);
    pkt->base_duty_h = (uint8_t)((uint16_t)base_duty >> 8);
    pkt->base_duty_l = (uint8_t)((uint16_t)base_duty & 0xFF);
    pkt->m_step_h    = (uint8_t)((uint16_t)step >> 8);
    pkt->m_step_l    = (uint8_t)((uint16_t)step & 0xFF);
    pkt->duty_h      = (uint8_t)(duty >> 8);
    pkt->duty_l      = (uint8_t)(duty & 0xFF);
    pkt->crc         = get_crc8((uint8_t*)pkt, sizeof(cyclic_telem_pkt_t) - 1);
}
