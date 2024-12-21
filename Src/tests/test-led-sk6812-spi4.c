// This program tests the functions
// in "led.h"
#include "debug.h"
#include "led.h"
#include "mcu.h"
#include "targets.h"

void wait(uint32_t loops)
{
    for (int i = 0; i < loops; i++)
    {
        asm("nop");
    }
}
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
    } else if (tmp < 2*threshold) {
        g = map(tmp - threshold, 0, threshold, 0, 0xff);
        b = 0xff - g;
    } else {
        b = map(tmp - 2*threshold, 0, threshold, 0, 0xff);
        r = 0xff - b;
    }
    debug_write_string("r g b\r\n");
    debug_write_int(r);
    debug_write_string(" ");
    debug_write_int(g);
    debug_write_string(" ");
    debug_write_int(b);
    debug_write_string(" ");
    debug_write_string("\r\n");


    return (b << 16) | (r << 8) | g;
}

int main()
{
    mcu_setup(250);
    led_initialize();
    debug_initialize();

    while(1) {
        for (int i = 0; i < (1<<14); i++)
        {
            uint32_t color = angle_to_color(i);
            led_write(color);
            wait(0xfff);
        }
    }
}