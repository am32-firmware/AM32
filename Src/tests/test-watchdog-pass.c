#include "stm32h563xx.h"
#include "targets.h"
#include "peripherals.h"

int main()
{

    // set a breakpoint here to observe that the watchdog DOES NOT trigger a system reset
    MX_IWDG_Init();

    while(1) {
        reloadWatchDogCounter();
    }
}