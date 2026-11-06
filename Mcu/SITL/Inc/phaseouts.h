/*
 * phaseouts.h - SITL
 */

#ifndef PHASEOUTS_H_
#define PHASEOUTS_H_

#include "main.h"

extern void allOff(void);
extern void comStep(int newStep);
extern void fullBrake(void);
extern void allpwm(void);
extern void proportionalBrake(void);
extern void twoChannelForward(void);
extern void twoChannelReverse(void);

#endif /* PHASEOUTS_H_ */
