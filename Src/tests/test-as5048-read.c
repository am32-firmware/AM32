// This example reads the as5048 using
// the as5048-spi.h driver

#include "as5048-spi.h"
#include "mcu.h"

uint16_t readData[7];
const uint16_t defaultData[7] = {
    0,
    0,
    0,
    1023,
    2047,
    345,
    643
};

bool compare()
{
    for (int i = 0; i < 7; i++) {
        if (readData[i] != defaultData[i]) {
            for (;;); // spin forever
        }
    }
    return true;
}

as5048_t as5048;

int main()
{
    mcu_setup(250);
    as5048_initialize(&as5048);

    while(1) {
    }
}