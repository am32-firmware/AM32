// this example tests the comp_timer interface
// it tests the COMP_TIMER and associated interrupts

#include "comp-timer.h"
#include "mcu.h"

int main()
{
    mcu_setup(250);
    comp_timer_initialize();
    comp_timer_interrupt_enable();
    comp_timer_enable();

    while(1) {
    }
}