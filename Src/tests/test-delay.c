#include "functions.h"
#include "led.h"
#include "peripherals.h"

int main()
{
    led_initialize();
    led_off();

    utility_timer_initialize();
    utility_timer_enable();

    // max value for micros is 0xffff
    // max value for millis is 0xffff/10 = 6553
    while(1) {
        // delayMillis(1000);
        delayMillis(1000);
        led_toggle();
        delayMicros(25000);
        led_toggle();
    }
}
