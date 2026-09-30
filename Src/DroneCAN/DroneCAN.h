#pragma once

#include <stdbool.h>

#if DRONECAN_SUPPORT
void DroneCAN_Init(void);
void DroneCAN_update();
bool DroneCAN_active();
extern volatile uint16_t dronecan_beep_hz;   /* a BeepCommand note waiting to be played */
extern volatile uint16_t dronecan_beep_ms;

#endif // DRONECAN_SUPPORT
