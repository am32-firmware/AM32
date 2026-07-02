/*
 * peripherals_mcu.h — MCXA153 private peripheral declarations.
 * Register macro sheet lives in timers.h/flexpwm.h (included via main.h).
 */

#ifndef PERIPHERALS_MCU_H_
#define PERIPHERALS_MCU_H_

void SystemInit(void);
void SystemClock_Config(void);
void initGPIO(void);
void initSPI(void);

#endif /* PERIPHERALS_MCU_H_ */
