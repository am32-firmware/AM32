/*
 * dshot.h
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "main.h"

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#define DSHOT_EDT_FRAME_STRESS 0x0C00U
#define DSHOT_EDT_FRAME_STATUS 0x0E00U

#define DSHOT_EDT_STATUS_ALERT   (1U << 7)
#define DSHOT_EDT_STATUS_WARNING (1U << 6)
#define DSHOT_EDT_STATUS_ERROR   (1U << 5)

void computeDshotDMA(void);
void make_dshot_package(uint16_t com_time);
void dshot_note_status_event(uint8_t event_mask);

extern void playInputTune(void);
extern void playInputTune2(void);
extern void playBeaconTune3(void);
extern void saveEEpromSettings(void);

extern volatile char dshot_telemetry;
extern volatile char armed;
extern char dir_reversed;
extern char buffer_divider;
extern uint8_t last_dshot_command;
extern volatile uint32_t commutation_interval;

// int e_com_time;

#endif /* INC_DSHOT_H_ */
