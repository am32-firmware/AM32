#include "functions.h"
#include "phaseouts.h"
#include "targets.h"

char comp_pwm = 1;
uint8_t i = 0;

int main()
{
    while(1) {
        comStep(i++%6);
        delayMillis(500);
    }
}