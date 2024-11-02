// This program tests the watchdog_initialize_period
// function
// a breakpoint before "while (1)" will retrigger
// by a watchdog reset
#include "mcu.h"
#include "watchdog.h"
#include "debug.h"

int main()
{
    mcu_setup();
    debug_initialize();
    debug_set_1();
    // set watchdog to trigger after 500ms
    watchdog_initialize_period(500);
    watchdog_enable();
    debug_reset_1();
    while(1) {
    }
}
