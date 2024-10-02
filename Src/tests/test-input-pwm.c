#include "functions.h"
// #include "mcu.h"
#include "peripherals.h"

int main()
{
    // mcu_setup();
    input_timer_initialize();
    TIM15->CCMR1 = 1;
    input_timer_enable();
    while(1) {
    }
}
