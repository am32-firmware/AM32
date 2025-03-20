#include "kiss_telemetry.h"
#include "eeprom.h"

extern uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen);

uint8_t aTxBuffer[49] __attribute__((aligned(4)));

void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm)
{
    kiss_telem_pkt_t* telem_pkt = (kiss_telem_pkt_t*)aTxBuffer;

    telem_pkt->temperature = temp; // temperature in Celcius

    // voltage in centivolts
    telem_pkt->voltage_h = (voltage >> 8) & 0xFF;
    telem_pkt->voltage_l = voltage & 0xFF;

    // current in centiamps
    telem_pkt->current_h = (current >> 8) & 0xFF;
    telem_pkt->current_l = current & 0xFF;

    // accumulated current consumption in mAH
    telem_pkt->consumption_h = (consumption >> 8) & 0xFF;
    telem_pkt->consumption_l = consumption & 0xFF;

    // eRPM * 100, so 1 in the packet means 100 eRPM
    telem_pkt->erpm_h = (e_rpm >> 8) & 0xFF;
    telem_pkt->erpm_l = e_rpm & 0xFF;

    telem_pkt->crc = get_crc8((uint8_t*)telem_pkt, sizeof(kiss_telem_pkt_t) - 1);
}

void makeInfoPacket()
{
    for(int i = 0; i < 48; i++) {
        aTxBuffer[i] = eepromBuffer.buffer[i];
    }
    aTxBuffer[48] = get_crc8(aTxBuffer, 48);
}
