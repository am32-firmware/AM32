// #include "functions.h"
#include "led.h"
// #include "peripherals.h"
// #include "debug.h"
#include "ten-khz-timer.h"

uint32_t i = 1;
uint32_t count = 0; // loop counter
uint8_t prescaler = 20; // loop trigger (ie prescaler)
void tenKhzRoutine()
{
    // debug_toggle_1();
    if ((count++ % prescaler) == 0)
    {
        i = i<<1;
        if (i & 0x800000) {
            i = 1;
        }
        led_write(i);
        count = 1;
    }

}

int main()
{
    led_initialize();
    led_off();

    ten_khz_timer_initialize();
    ten_khz_timer_enable();
    ten_khz_timer_interrupt_enable();

    while(1) {

    }
}
