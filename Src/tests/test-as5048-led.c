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
#include "led.h"
#include "mcu.h"

as5048_t as5048;

uint32_t current_angle;

static uint8_t map(uint32_t value, uint32_t in_low, uint32_t in_high, uint32_t out_low, uint32_t out_high)
{
    if (value < in_low) {
        return out_low;
    } else if (value > in_high) {
        return out_high;
    }

    uint32_t in_range = in_high - in_low;

    uint32_t difference = value - in_low;

    uint32_t out_range = out_high - out_low;

    uint32_t output = out_range * difference / in_range;
    return output;

}
uint32_t angle_to_color(uint16_t angle)
{
    uint32_t tmp = angle * 1000;
    uint32_t threshold = ((1<<14) * 1000) / 3;

    uint32_t r = 0;
    uint32_t g = 0;
    uint32_t b = 0;
    if (tmp < threshold) {
        r = map(tmp, 0, threshold, 0, 0xff);
        g = 0xff - r;
    // } else if (tmp < 2*threshold) {
    //     g = map(tmp - threshold, 0, threshold, 0, 0xff);
    //     b = 0xff - g;
    // } else {
    //     b = map(tmp - 2*threshold, 0, threshold, 0, 0xff);
    //     r = 0xff - b;
    }
    return (b << 16) | (r << 8) | g;
}

int main()
{
    mcu_setup(250);
    as5048_initialize(&as5048);
    led_initialize();

    as5048_set_zero_position(&as5048);

    while(1) {
        current_angle = as5048_read_angle(&as5048);
        uint32_t color = angle_to_color(current_angle);
        led_write(color);
    }
}
