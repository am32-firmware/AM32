/*
 * phaseouts.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */

#ifndef INC_PHASEOUTS_H_
#define INC_PHASEOUTS_H_

#include "main.h"
#include "targets.h"
#include "common.h"

void allOff();
/*
 * comStep keeps the historical per-family signature: f051/l431/a153 define it
 * with a char parameter, the rest with int. Normalizing changes the generated
 * code on the char families (callee-side extension), so unification is
 * deferred to the phaseouts.c consolidation with bench verification.
 */
#if defined(MCU_F051) || defined(MCU_L431) || defined(MCU_A153)
void comStep(char newStep);
#else
void comStep(int newStep);
#endif
void fullBrake();
void allpwm();
void proportionalBrake();
void twoChannelForward();
void twoChannelReverse();

#endif /* INC_PHASEOUTS_H_ */
