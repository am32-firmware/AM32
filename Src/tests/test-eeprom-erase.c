// this example will erase the flash region used
// for eeprom storage
// in AM32 typically one flash page is reserved for eeprom
// for the h5 target, the high-cycle flash area is
// used. This imposes additional constraints and
// configuration requirements
// the option bytes must be set to enable the high-cycle
// flash area


#include "stm32h563xx.h"
#include "targets.h"
#include "eeprom.h"

int main()
{

    flash_erase_sector(127);

    while(1) {
    }
}