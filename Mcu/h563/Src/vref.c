#include "vref.h"

#include "stm32h563xx.h"

void vref_enable()
{
    // enable the voltage reference buffer clock
    RCC->APB3ENR |= RCC_APB3ENR_VREFEN;
    // enable the internal voltage reference
    // reset HIZ bit (set by default)
    VREFBUF->CSR = VREFBUF_CSR_ENVR;
    // wait for voltage reference ready (VRR) flag
    while (!(VREFBUF->CSR & VREFBUF_CSR_VRR))
    {
        // wait for voltage reference output to settle
        // to selected level
    }
}
