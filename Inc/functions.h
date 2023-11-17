/*
 * functions.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "main.h"
#include "targets.h"

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
	
	
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
#define SIGN(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  (_x > 0) - (_x < 0); })


int getAbsDif(int number1, int number2);
void delayMicros(uint32_t micros);
void delayMillis(uint32_t millis);
long map(long x, long in_min, long in_max, long out_min, long out_max);

//common functions which when used extensively will reduce code size, slight speed hit as a consequence
//however, given that the MCUs seem to be more space constrained, this should be OK
int32_t RotateMinMax(const int32_t startValue, const int32_t amount, const int32_t min, const int32_t max);
int32_t Clamp(const int32_t var, const int32_t min, const int32_t max);
int32_t Min(const int32_t var, const int32_t var1);
int32_t Max(const int32_t var, const int32_t var1);

#ifdef ARTERY
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin);
#endif
#endif /* FUNCTIONS_H_ */
