#include "functions.h"
#include "led.h"
#include "peripherals.h"

int main()
{
    led_initialize();
    led_off();

    utility_timer_initialize();
    utility_timer_enable();

    while(1) {
        delayMillis(1000);
        led_toggle();
        delayMicros(25000);
        led_toggle();
    }
}
