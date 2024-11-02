#pragma once

#include "targets.h"

#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)
#define SET_INTERVAL_TIMER_COUNT(intertime) (INTERVAL_TIMER->CNT = intertime)
