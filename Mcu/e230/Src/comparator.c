/*
 * comparator.c
 *
 * GD32E230 register mapping for the shared comparator/EXTI implementation.
 */

#include "comparator.h"

#include "targets.h"

#define COMPARATOR_READ() cmp_output_level_get()
#define COMP_EXTI_INTEN EXTI_INTEN
#define COMP_EXTI_PENDING EXTI_PD
#define COMP_MUX_SELECT CMP_CS
#define COMP_EXTI_RISING EXTI_RTEN
#define COMP_EXTI_FALLING EXTI_FTEN

#include "../../shared/comparator_exti.h"
