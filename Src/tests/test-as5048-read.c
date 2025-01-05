// This example reads the as5048 using
// the as5048-spi.h driver
// add the global variable `angle` to live watch
// to see the sensor angle reading

#include "as5048-spi.h"
#include "mcu.h"

as5048_t as5048;

uint16_t current_angle;

int main()
{
    mcu_setup(250);
    as5048_initialize(&as5048);

    while(1) {
        current_angle = as5048_read_angle(&as5048);
    }
}