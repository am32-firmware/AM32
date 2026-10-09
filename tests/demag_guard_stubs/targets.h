#pragma once
#define CPU_FREQUENCY_MHZ 64

#ifdef MCU_G431
#ifdef TEST_MIDRAIL
#define CURRENT_OFFSET 1650
#else
/* Sequre's 5 mV/A mid-rail sensor, with an incorrect configured offset. */
#define CURRENT_OFFSET 600
#endif // TEST_MIDRAIL
#define MILLIVOLT_PER_AMP 5
#else
#define CURRENT_OFFSET 0
#define MILLIVOLT_PER_AMP 20
#endif // MCU_G431

#define DEMAG_GUARD_ENABLED 1
#ifndef DEMAG_GUARD_CURRENT
#define DEMAG_GUARD_CURRENT 1
#endif // DEMAG_GUARD_CURRENT
