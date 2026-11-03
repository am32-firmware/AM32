/*
 * apa102.h
 *
 *  Created on: 18 Feb 2026
 *      Author: nxg09992
 */

#ifndef MCU_A153_INC_APA102_H_
#define MCU_A153_INC_APA102_H_

#include "main.h"

void init_apa102(void);
void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue);

#endif /* MCU_A153_INC_APA102_H_ */
