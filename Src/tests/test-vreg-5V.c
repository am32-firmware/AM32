// this test enables the 5V regulator, then
// waits for the PGOOD signal

#include "targets.h"
#include "vreg.h"

int main()
{
    vreg5V_initialize();
    vreg5V_enable();

    // wait for power good (PGOOD) signal
    while (!vreg5V_pgood());

    // do nothing
    while (1);
}