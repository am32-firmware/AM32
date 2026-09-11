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
void comStep(int newStep);
void fullBrake();
void allpwm();
void proportionalBrake();
void twoChannelForward();
void twoChannelReverse();

void comStepWithFastDemag(int newStep, char forward, uint16_t demag_us);
void comStepWithActiveDemag(int newStep, char forward, uint16_t active_us,uint16_t tail_us);
void highSidesOff();

#ifndef DEMAG_DEADTIME_US
#define DEMAG_DEADTIME_US 1
#endif
// Split halves of comStepSyncDemag() so the active interval can be timed by
// a one-shot interrupt (COM_TIMER) instead of a blocking delay.
void syncFetOn(int newStep, char forward);
void floatLeg(int newStep);
#endif /* INC_PHASEOUTS_H_ */
