// This example configures a couple
// of registers on the drv8323

#include "stm32h563xx.h"
#include "targets.h"
#include "spi.h"
#include "gpio.h"
#include "dma.h"
#include "drv8323-spi.h"
#include "mcu.h"
#include "utility-timer.h"

int main()
{
    mcu_setup();
    utility_timer_initialize();
    utility_timer_enable();
    drv8323_initialize(&DRV8323);

    while (!drv8323_write_reg(&DRV8323,
        DRV8323_REG_CSA_CONTROL |
        DRV8323_CSA_VREF_DIV |
        DRV8323_CSA_CSA_GAIN_40VV |
        DRV8323_CSA_SEN_LVL_250mV
    ));

    while (!drv8323_write_reg(&DRV8323,
        DRV8323_REG_OCP_CONTROL |
        DRV8323_OCP_DEADTIME_400ns |
        DRV8323_OCP_DEGLITCH_6us |
        DRV8323_OCP_VDSLVL_600mV
    ));

    while(1) {
    }
}