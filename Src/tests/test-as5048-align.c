// This example reads the as5048 using
// the as5048-spi.h driver
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "bridge.h"
#include "drv8323-spi.h"
#include "mcu.h"
#include "utility-timer.h"

as5048_t as5048;

uint16_t current_angle;

int main()
{
    mcu_setup(250);

    utility_timer_initialize();
    utility_timer_enable();

    as5048_initialize(&as5048);
    drv8323_initialize(&DRV8323);


    // bridge_initialize();
    // bridge_set_mode_run();
    // bridge_set_run_frequency(8000);
    // bridge_set_run_duty(0x0080);
    // bridge_enable();


    while(1) {
        current_angle = as5048_read_angle(&as5048);
    }
}