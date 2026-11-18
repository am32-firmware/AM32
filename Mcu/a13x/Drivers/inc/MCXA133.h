/*
** ###################################################################
**     Processors:          MCXA133VFM
**                          MCXA133VFT
**                          MCXA133VLF
**
**     Compilers:           GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    MCXA1 User manual
**     Version:             rev. 2.0, 2024-10-29
**     Build:               b250521
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MCXA133
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2025 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2022-03-29)
**         Initial version based on v0.1UM
**     - rev. 2.0 (2024-10-29)
**         Change the device header file from single flat file to multiple files based on peripherals,
**         each peripheral with dedicated header file located in periphN folder.
**
** ###################################################################
*/

/*!
 * @file MCXA133.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for MCXA133
 *
 * CMSIS Peripheral Access Layer for MCXA133
 */

#if !defined(MCXA133_H_)  /* Check if memory map has not been already included */
#define MCXA133_H_

/* IP Header Files List */
#include "PERI_ADC.h"
#include "PERI_AOI.h"
#include "PERI_CDOG.h"
#include "PERI_CMC.h"
#include "PERI_CRC.h"
#include "PERI_CTIMER.h"
#include "PERI_DEBUGMAILBOX.h"
#include "PERI_DMA.h"
#include "PERI_EIM.h"
#include "PERI_EQDC.h"
#include "PERI_ERM.h"
#include "PERI_FMC.h"
#include "PERI_FMU.h"
#include "PERI_FMUTEST.h"
#include "PERI_FREQME.h"
#include "PERI_GLIKEY.h"
#include "PERI_GPIO.h"
#include "PERI_I3C.h"
#include "PERI_INPUTMUX.h"
#include "PERI_LPCMP.h"
#include "PERI_LPI2C.h"
#include "PERI_LPSPI.h"
#include "PERI_LPTMR.h"
#include "PERI_LPUART.h"
#include "PERI_MRCC.h"
#include "PERI_OSTIMER.h"
#include "PERI_PORT.h"
#include "PERI_PWM.h"
#include "PERI_SCG.h"
#include "PERI_SPC.h"
#include "PERI_SYSCON.h"
#include "PERI_TRDC.h"
#include "PERI_UTICK.h"
#include "PERI_VBAT.h"
#include "PERI_WAKETIMER.h"
#include "PERI_WUU.h"
#include "PERI_WWDT.h"

#endif  /* #if !defined(MCXA133_H_) */
