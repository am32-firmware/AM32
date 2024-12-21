// This example sets the zero position
// of the as5048 sensor, then prints the
// angle reading of the as5048 in degrees*100
// (centi-degrees) to a usart

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
        debug_write_string("\r\n");
    }
}
