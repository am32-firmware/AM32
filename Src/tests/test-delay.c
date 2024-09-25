#include "functions.h"
#include "led.h"
#include "peripherals.h"

int main()
{
    utility_timer_initialize();
    led_initialize();
    led_on();
    utility_timer_enable();

    while(1) {
        delayMillis(1000);
        led_toggle();
    }
}
