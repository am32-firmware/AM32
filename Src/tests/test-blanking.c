#include "blanking.h"
#include "mcu.h"

int main()
{
    mcu_setup(250);
    blanking_initialize();
    blanking_enable();

    while(1) {
    }
}