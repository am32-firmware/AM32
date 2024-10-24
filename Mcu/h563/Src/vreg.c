#include "vreg.h"
#include "gpio.h"
#include "targets.h"

gpio_t gpioVreg5VEnable = DEF_GPIO(VREG5V_ENABLE_PORT, VREG5V_ENABLE_PIN, 0, GPIO_OUTPUT);
gpio_t gpioVreg5VPgood = DEF_GPIO(VREG5V_PGOOD_PORT, VREG5V_PGOOD_PIN, 0, GPIO_INPUT);
gpio_t gpioVreg5VSync = DEF_GPIO(VREG5V_SYNC_PORT, VREG5V_SYNC_PIN, VREG5V_SYNC_AF, GPIO_AF);

void vreg5V_initialize()
{
    gpio_initialize(&gpioVreg5VEnable);
    gpio_initialize(&gpioVreg5VPgood);
    // gpio_initialize(&gpioVreg5VSync);
}

void vreg5V_enable()
{
    gpio_set(&gpioVreg5VEnable);
}

void vreg5V_disable()
{
    gpio_reset(&gpioVreg5VEnable);
}

bool vreg5V_pgood()
{
    return gpio_read(&gpioVreg5VPgood);
}

void vreg5V_set_frequency()
{

}