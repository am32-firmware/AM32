/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion
 * -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes
 * ------------------------------------------------------------------*/
#include "g32f0xx.h"
#include "g32f031xx.h"
#include "g32f031_ddl_adc.h"
#include "g32f031_ddl_atmr.h"
#include "g32f031_ddl_btmr.h"
#include "g32f031_ddl_bus.h"
#include "g32f031_ddl_comp0.h"
#include "g32f031_ddl_comp1.h"
#include "g32f031_ddl_cortex.h"
#include "g32f031_ddl_crc.h"
#include "g32f031_ddl_div.h"
#include "g32f031_ddl_dma.h"
#include "g32f031_ddl_eint.h"
#include "g32f031_ddl_flash.h"
#include "g32f031_ddl_gpio.h"
#include "g32f031_ddl_gtmr.h"
#include "g32f031_ddl_i2c.h"
#include "g32f031_ddl_iwdt.h"
#include "g32f031_ddl_lptmr.h"
#include "g32f031_ddl_opa.h"
#include "g32f031_ddl_pmu.h"
#include "g32f031_ddl_rcc.h"
#include "g32f031_ddl_scu.h"
#include "g32f031_ddl_spi.h"
#include "g32f031_ddl_uart.h"
#include "g32f031_ddl_usart.h"
#include "g32f031_ddl_utils.h"
#include "g32f031_ddl_wwdt.h"

void Error_Handler(void);
extern void PeriodElapsedCallback(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF
 * FILE****/
