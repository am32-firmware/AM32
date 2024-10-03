#include "functions.h"
// #include "mcu.h"
#include "peripherals.h"
#include "IO.h"
#include "dma.h"
#include "signal.h"


int main()
{
    dma_initialize();
    // mcu_setup();
    input_timer_initialize();
    TIM15->CCMR1 = 1;
    input_timer_enable();
    receiveDshotDma();
    // compute();
    while(1) {
    }
}
