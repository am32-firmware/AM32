/*
 * IO.h - SITL (dshot/servo input is not supported, stubs only)
 */

#ifndef IO_H_
#define IO_H_

#include "main.h"

void changeToOutput(void);
void changeToInput(void);
void receiveDshotDma(void);
void sendDshotDma(void);

uint8_t getInputPinState(void);
void setInputPolarityRising(void);
void setInputPullDown(void);
void setInputPullUp(void);
void enableHalfTransferInt(void);
void setInputPullNone(void);

extern volatile char inputSet;
extern char dshot;
extern volatile char servoPwm;
extern volatile char send_telemetry;

#endif /* IO_H_ */
