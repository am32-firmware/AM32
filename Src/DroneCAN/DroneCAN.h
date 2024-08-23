#pragma once

#include <stdbool.h>

#if DRONECAN_SUPPORT
void DroneCAN_Init(void);
void DroneCAN_update();
bool DroneCAN_active();

#endif // DRONECAN_SUPPORT
