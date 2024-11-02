// This program tests the watchdog_initialize_period
// function
// a breakpoint before "while (1)" will retrigger
// by a watchdog reset
#include "mcu.h"
#include "watchdog.h"

int main()
{
    mcu_setup();
    watchdog_initialize_period(3000);
    watchdog_enable();

    while(1) {
    }
}
