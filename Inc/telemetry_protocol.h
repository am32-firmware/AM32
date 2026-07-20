#include "main.h"

#ifndef TELEMETRY_PROTOCOL_H_
#define TELEMETRY_PROTOCOL_H_

#ifdef USE_SPORT_TELEMETRY
#include "sport_telemetry.h"
#define SERIAL_TELEMETRY_BAUDRATE_SELECTED SPORT_TELEMETRY_BAUDRATE
#else
#include "kiss_telemetry.h"
#define SERIAL_TELEMETRY_BAUDRATE_SELECTED 115200U
#endif

#endif
