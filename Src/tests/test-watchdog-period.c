#include "gpio.h"
#include "mcu.h"
#include "watchdog.h"

int main()
{
    mcu_setup();
    watchdog_initialize_period(1000);

    while(1) {
    }
}
