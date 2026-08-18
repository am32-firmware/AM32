/**
 *
 * @file        system_g32f031.h
 *
 * @brief       CMSIS Cortex-M0 Device System Source File for G32F031 devices.
 *
 * @version     V1.0.0
 *
 * @date        2026-01-15
 *
 * @attention
 *
 *  Copyright (C) 2026 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 *
 */

/* Define to prevent recursive inclusion */
#ifndef __SYSTEM_G32F031_H
#define __SYSTEM_G32F031_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

extern const uint8_t  HSIPrescTable[4];
extern const uint8_t  AHBPrescTable[8];
extern const uint8_t  APBPrescTable[4];

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_G32F031_H */
