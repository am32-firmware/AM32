// This example sets the zero position
// of the as5048 sensor using the
// as5048-spi.h driver
// the zero position setting is olitile in this example
// the sensor supports a one time programming (OTP)
// of the zero position registers, and the
// value of these registers is then kept in a permanent
// manner
// this example does not permanently program the zero position
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "debug.h"
#include "mcu.h"

as5048_t as5048;

uint32_t current_angle;

int main()
{
    mcu_setup(250);
    as5048_initialize(&as5048);
    debug_initialize();

    as5048_set_zero_position(&as5048);

    while(1) {
        current_angle = as5048_get_angle_degrees(&as5048);
        debug_write_int(current_angle);
    }
}
