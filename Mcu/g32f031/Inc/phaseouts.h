/*
 * phaseouts.h
 *
 *  Created on: Aug. 4, 2026
 *      Author: Nong jun
 */

#ifndef PHASEOUTS_H_
#define PHASEOUTS_H_

#include "main.h"
#include "common.h"

void allOff(void);
void comStep(int newStep);
void fullBrake(void);
void allpwm(void);
void proportionalBrake(void);
void twoChannelForward(void);
void twoChannelReverse(void);

#endif /* PHASEOUTS_H_ */
