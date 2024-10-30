// This program tests the bridge and drv8323
// it will make the motor beep at 440Hz for 500ms
// if a fuse or current limiting device is in line
// with the power supply, the drv8323 may throw a fault
#include "functions.h"
#include "targets.h"
#include "bridge.h"
#include "drv8323-spi.h"
#include "mcu.h"
#include "utility-timer.h"

int main()
{
    mcu_setup();
    utility_timer_initialize();
    utility_timer_enable();
    drv8323_initialize(&DRV8323);
    bridge_initialize();

    bridge_set_mode_audio();
    bridge_set_audio_frequency(440);
    bridge_set_audio_duty(0x0f);
    bridge_enable();

    // beep for 0.5s
    delayMillis(500);

    bridge_disable();
    while(1) {
    }
}