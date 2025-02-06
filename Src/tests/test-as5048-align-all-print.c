// This example reads the as5048 using
// the as5048-spi.h driver
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "bridge.h"
#include "debug.h"
#include "drv8323-spi.h"
#include "functions.h"
#include "mcu.h"
#include "utility-timer.h"

as5048_t as5048;

uint16_t current_angle;

#define DEAD_ZONE 10

#define WAIT_MS 65 // 65 ms is max
int main()
{
    mcu_setup(250);

    debug_initialize();

    utility_timer_initialize();
    utility_timer_enable();

    as5048_initialize(&as5048);
    drv8323_initialize(&DRV8323);


    bridge_initialize();
    bridge_set_mode_run();
    bridge_set_run_frequency(24000);
    bridge_set_run_duty(0x0200);
    bridge_enable();

    delayMillis(WAIT_MS);
    as5048_set_zero_position(&as5048);

    do {
        current_angle = as5048_read_angle(&as5048);
    } while (current_angle != 0);

    uint16_t i = 0;
    do {
        bridge_commutate();
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);
        delayMillis(WAIT_MS);
        uint16_t last_angle = current_angle;
        current_angle = as5048_read_angle(&as5048);
        i++;
        debug_write_string("\n\rstep: ");
        debug_write_int(i);
        debug_write_string(" angle: ");
        debug_write_int(current_angle);
        debug_write_string(" diff: ");
        debug_write_int(current_angle - last_angle);


    } while (current_angle < (1<<14) - DEAD_ZONE  && current_angle > DEAD_ZONE);

    bridge_disable();

    while(1) {
        current_angle = as5048_read_angle(&as5048);
    }
}