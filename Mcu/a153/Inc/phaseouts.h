/*
 * phaseouts.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef INC_PHASEOUTS_H_
#define INC_PHASEOUTS_H_

#include "main.h"
#include "common.h"

void allOff();
void comStep(char newStep);
void fullBrake();
void allpwm();
void proportionalBrake();
void twoChannelForward();
void twoChannelReverse();

#endif /* INC_PHASEOUTS_H_ */
