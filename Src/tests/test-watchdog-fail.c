#include "stm32h563xx.h"
#include "targets.h"
#include "peripherals.h"

int main()
{

    // set a breakpoint here to observe that the watchdog has triggered a system reset
    watchdog_initialize();

    while(1) {
    }
}