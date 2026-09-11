/*
 * phaseouts.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
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

#ifndef DEMAG_DEADTIME_US
#define DEMAG_DEADTIME_US 1
#endif
void syncFetOn(int newStep, char forward);
void floatLeg(int newStep);

#endif /* INC_PHASEOUTS_H_ */
