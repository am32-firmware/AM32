#include "power.h"
#include "stm32h563xx.h"

void power_set_core_voltage(uint8_t vos)
{
    // Set core voltage regulator output scaling for maximum performance
    uint32_t voscr = PWR->VOSCR;
    voscr &= ~(PWR_VOSCR_VOS_Msk);
    voscr |= vos << PWR_VOSCR_VOS_Pos;
    PWR->VOSCR = voscr;
    while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY));
    while (!(PWR->VOSSR & PWR_VOSSR_ACTVOSRDY));
}
