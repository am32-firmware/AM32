// #include "phaseouts.h"
#include "targets.h"
#include "gpio.h"
#include "bridge.h"
#include "drv8323-spi.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"

int main()
{
    mcu_setup(250);

    utility_timer_initialize();
    utility_timer_enable();
    drv8323_initialize(&DRV8323);


    bridge_initialize();
    bridge_set_deadtime_ns(500);
    bridge_set_mode_run();
    bridge_set_run_frequency(24000);
    bridge_set_run_duty(0x0400);
    bridge_enable();

    for (int n = 0; n < 1200; n++) {
        delayMillis(10);
        bridge_commutate();
    }
    bridge_disable();
    while (1) {
    }
}