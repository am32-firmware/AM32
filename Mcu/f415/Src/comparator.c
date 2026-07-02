/*
 * comparator.c
 *
 * AT32F415 register mapping for the shared comparator/EXTI implementation.
 */

#include "comparator.h"

#include "targets.h"

#define COMPARATOR_READ() (CMP->ctrlsts1_bit.cmp1value)
#define COMP_EXTI_INTEN (EXINT->inten)
#define COMP_EXTI_PENDING (EXINT->intsts)
#define COMP_MUX_SELECT (CMP->ctrlsts1)
#define COMP_EXTI_RISING (EXINT->polcfg1)
#define COMP_EXTI_FALLING (EXINT->polcfg2)

#include "../../shared/comparator_exti.h"
