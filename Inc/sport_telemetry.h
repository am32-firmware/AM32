#include "main.h"

#ifndef SPORT_TELEMETRY_H_
#define SPORT_TELEMETRY_H_

#define SPORT_TELEMETRY_MAX_PACKET_SIZE 18U
#define SPORT_TELEMETRY_BAUDRATE 57600U
#define SPORT_SENSOR_ID 0x0AU
#define SPORT_POLL_BYTE (0x60U | SPORT_SENSOR_ID)

extern uint8_t aTxBuffer[SPORT_TELEMETRY_MAX_PACKET_SIZE] __attribute__((aligned(4)));
extern uint8_t nbDataToTransmit;

uint8_t sport_prepare_response(uint8_t* buffer);
uint8_t sport_rx_byte(uint8_t byte);

#endif
