// This program tests the watchdog_initialize_period
// function
// a breakpoint before "while (1)" will retrigger
// by a watchdog reset
#include "debug.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"
#include "watchdog.h"

int main()
{

    mcu_setup(250);

    utility_timer_initialize();
    utility_timer_enable();

    debug_initialize();
    debug_reset_1();

    for (int i = 0;i < 5;i++)
    {
        delayMillis(1);
        debug_toggle_1();
    }

    // set watchdog to trigger after 25ms
    watchdog_initialize_period(1000);
    watchdog_enable();
    // debug_reset_1();
    while(1) {
        // watchdog_reload();
        delayMillis(1);
        debug_toggle_1();
    }
}
