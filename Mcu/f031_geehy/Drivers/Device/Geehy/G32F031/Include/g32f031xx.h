/**
 *
 * @file        g32f031xx.h
 *
 * @brief       CMSIS g32f031 Device Peripheral Access Layer Header File.
 *
 * @version     V1.0.0
 *
 * @date        2026-01-15
 *
 * @attention
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The original code has been modified by Geehy Semiconductor.
 *
 * Copyright (C) 2026 Geehy Semiconductor.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 */


/** @addtogroup CMSIS
  * @{
  */


/** @addtogroup G32F031
  * @{
  */


#ifndef G32F031_H
#define G32F031_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum
{
/* =======================================  ARM Cortex-M0+ Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */
/* ==========================================  G32F031 Specific Interrupt Numbers  =========================================== */
  WWDT_IRQn                 =   0,              /*!< 0  Window Watchdog global interrupt                                       */
  PVD_IRQn                  =   1,              /*!< 1  PVD global interrupt                                                   */
  FLASH_IRQn                =   3,              /*!< 3  FLASH global interrupt                                                 */
  RCC_IRQn                  =   4,              /*!< 4  RCM global interrupt                                                   */
  EINT0_1_IRQn              =   5,              /*!< 5  EINT Line 0 to 1 global interrupt                                      */
  EINT2_3_IRQn              =   6,              /*!< 6  EINT Line 2 to 3 global interrupt                                      */
  EINT4_15_IRQn             =   7,              /*!< 7  EINT Line 4 to 15 global interrupts                                    */
  DMA_CH0_IRQn              =   9,              /*!< 9  DMA_CH0 global interrupt                                               */
  DMA_CH1_IRQn              =  10,              /*!< 10 DMA_CH1 global interrupts                                              */
  ADC_IRQn                  =  12,              /*!< 12 ADC global interrupt                                                   */
  ATMR_BRK_UP_TRG_COM_IRQn  =  13,              /*!< 13 ATMR_BRK_UP_TRG_COM global interrupt                                   */
  ATMR_CC_IRQn              =  14,              /*!< 14 ATMR_CC global interrupt                                               */
  GTMR_IRQn                 =  15,              /*!< 15 GTMR interrupt                                                         */
  BTMR0_IRQn                =  17,              /*!< 17 BTMR0 global interrupt                                                 */
  BTMR1_IRQn                =  18,              /*!< 18 BTMR1 global interrupt                                                 */
  LPTMR_IRQn                =  19,              /*!< 19 LPTMR global interrupt                                                 */
  COMP0_IRQn                =  21,              /*!< 21 COMP0 global interrupt                                                 */
  COMP1_2_3_IRQn            =  22,              /*!< 22 COMP1/2/3 global interrupt                                             */
  I2C_IRQn                  =  23,              /*!< 23 I2C global interrupt                                                   */
  SPI_IRQn                  =  25,              /*!< 25 SPI global interrupt                                                   */
  USART_IRQn                =  27,              /*!< 27 USART global interrupt                                                 */
  UART_IRQn                 =  28               /*!< 28 UART global interrupt                                                  */
} IRQn_Type;

/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/* ==========================  Configuration of the ARM Cortex-M0+ Processor and Core Peripherals  =========================== */
#define __CM0PLUS_REV                  0x0001U  /*!< CM0PLUS Core Revision                                                     */
#define __NVIC_PRIO_BITS               2        /*!< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0        /*!< Set to 1 if different SysTick Config is used                              */
#define __VTOR_PRESENT                 1        /*!< Set to 1 if CPU supports Vector Table Offset Register                     */
#define __MPU_PRESENT                  0        /*!< MPU present                                                               */
#define __FPU_PRESENT                  0        /*!< FPU present                                                               */

/**
  * @}
  */

/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm0plus.h"                       /*!< ARM Cortex-M0+ processor and core peripherals                             */
#include "system_g32f031.h"                     /*!< G32F031 System                                                            */
#include <stdint.h>


/* =========================================================================================================================== */
/* ================                            Device Specific Peripheral Section                             ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripherals
  * @{
  */



/* =========================================================================================================================== */
/* ================                                            PMU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Power Manage Unit (PMU)
  */

typedef struct
{
  __IO uint32_t  KEY;                           /*!< (@0x00000000) Read-Write Protection Register                              */
  __IO uint32_t  LPCR;                          /*!< (@0x00000004) Low Power Mode Register                                     */
  __IO uint32_t  WKCR;                          /*!< (@0x00000008) Low-power wake-up control register                          */
  __IO uint32_t  WKSR;                          /*!< (@0x0000000C) Low Power Wake-up Status Register                           */
  __IO uint32_t  PVDCSR;                        /*!< (@0x00000010) PVD Control/Status Register                                 */
} PMU_TypeDef;



/* =========================================================================================================================== */
/* ================                                            SCU                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief System Control Unit (SCU)
  */

typedef struct
{
  __IO uint32_t  KEY;                           /*!< (@0x00000000) Read-Write Protection Register                              */
  __IO uint32_t  DBG;                           /*!< (@0x00000004) System DBG Control Register                                 */
} SCU_TypeDef;



/* =========================================================================================================================== */
/* ================                                           EINT                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief External interrupt/event controller (EINT)
  */

typedef struct
{
  __IO uint32_t  IMASK;                         /*!< (@0x00000000)                                                             */
  __IO uint32_t  EMASK;                         /*!< (@0x00000004)                                                             */
  __IO uint32_t  RTEN;                          /*!< (@0x00000008)                                                             */
  __IO uint32_t  FTEN;                          /*!< (@0x0000000C)                                                             */
  __IO uint32_t  SWIEN;                         /*!< (@0x00000010)                                                             */
  __IO uint32_t  IPEND;                         /*!< (@0x00000014)                                                             */
  __IO uint32_t  CFG[2];                        /*!< (@0x00000018)                                                             */
} EINT_TypeDef;



/* =========================================================================================================================== */
/* ================                                          ATMR                                             ================ */
/* =========================================================================================================================== */


/**
  * @brief Advance timer (ATMR)
  */

typedef struct
{
  __IO uint32_t  CR1;                           /*!< (@0x00000000) control register 1                                          */
  __IO uint32_t  CR2;                           /*!< (@0x00000004) control register 2                                          */
  __IO uint32_t  SMCR;                          /*!< (@0x00000008) slave mode control register                                 */
  __IO uint32_t  IER;                           /*!< (@0x0000000C) DMA/Interrupt enable register                               */
  __IO uint32_t  SR;                            /*!< (@0x00000010) status register                                             */
  __IO uint32_t  CEG;                           /*!< (@0x00000014) event generation register                                   */
  __IO uint32_t  CCM1;                          /*!< (@0x00000018) capture/compare mode register 1 (output mode)               */
  __IO uint32_t  CCM2;                          /*!< (@0x0000001C) capture/compare mode register 2 (output mode)               */
  __IO uint32_t  CCEN;                          /*!< (@0x00000020) capture/compare enable register                             */
  __IO uint32_t  CNT;                           /*!< (@0x00000024) counter                                                     */
  __IO uint32_t  PSC;                           /*!< (@0x00000028) prescaler                                                   */
  __IO uint32_t  AUTORLD;                       /*!< (@0x0000002C) auto-reload register                                        */
  __IO uint32_t  REPCNT;                        /*!< (@0x00000030) repetition counter register                                 */
  __IO uint32_t  CC0;                           /*!< (@0x00000034) capture/compare register 0                                  */
  __IO uint32_t  CC1;                           /*!< (@0x00000038) capture/compare register 1                                  */
  __IO uint32_t  CC2;                           /*!< (@0x0000003C) capture/compare register 2                                  */
  __IO uint32_t  CC3;                           /*!< (@0x00000040) capture/compare register 3                                  */
  __IO uint32_t  BDT;                           /*!< (@0x00000044) break and dead-time register                                */
  __IO uint32_t  OCR1;                          /*!< (@0x00000048) Output ctrl register 1                                      */
  __IO uint32_t  OCR2;                          /*!< (@0x0000004C) Output ctrl register 2                                      */
  __IO uint32_t  TRGOCR;                        /*!< (@0x00000050) TRGO control register                                       */
  __IO uint32_t  BREAK;                         /*!< (@0x00000054) BREAK Filter register                                       */
  __IO uint32_t  OCXACFG;                       /*!< (@0x00000058) Lower comparison register control register                  */
  __IO uint32_t  CC0C;                          /*!< (@0x0000005C) Channel 0 Compare Register                                  */
  __IO uint32_t  CC1C;                          /*!< (@0x00000060) Channel 1 Compare Register                                  */
  __IO uint32_t  CC2C;                          /*!< (@0x00000064) Channel 2 Compare Register                                  */
  __IO uint32_t  CC3C;                          /*!< (@0x00000068) Channel 3 Compare Register                                  */
} ATMR_TypeDef;


/* =========================================================================================================================== */
/* ================                                          GTMR                                             ================ */
/* =========================================================================================================================== */


/**
  * @brief General purpose timer (GTMR)
  */

typedef struct
{
  __IO uint32_t  CR1;                           /*!< (@0x00000000) control register 1                                          */
  __IO uint32_t  CR2;                           /*!< (@0x00000004) control register 2                                          */
  __IO uint32_t  SMCR;                          /*!< (@0x00000008) slave mode control register                                 */
  __IO uint32_t  DIER;                          /*!< (@0x0000000C) DMA/Interrupt enable register                               */
  __IO uint32_t  SR;                            /*!< (@0x00000010) status register                                             */
  __IO uint32_t  CEG;                           /*!< (@0x00000014) event generation register                                   */
  __IO uint32_t  CCM1;                          /*!< (@0x00000018) capture/compare mode register 1                             */
  __IO uint32_t  CCM2;                          /*!< (@0x0000001C) capture/compare mode register 2                             */
  __IO uint32_t  CCEN;                          /*!< (@0x00000020) capture/compare enable register                             */
  __IO uint32_t  CNT;                           /*!< (@0x00000024) counter                                                     */
  __IO uint32_t  PSC;                           /*!< (@0x00000028) prescaler                                                   */
  __IO uint32_t  AUTORLD;                       /*!< (@0x0000002C) auto-reload register                                        */
       uint32_t  RESERVED0;
  __IO uint32_t  CC0;                           /*!< (@0x00000034) capture/compare register 0                                  */
  __IO uint32_t  CC1;                           /*!< (@0x00000038) capture/compare register 1                                  */
  __IO uint32_t  CC2;                           /*!< (@0x0000003C) capture/compare register 2                                  */
  __IO uint32_t  CC3;                           /*!< (@0x00000040) capture/compare register 3                                  */
} GTMR_TypeDef;



/* =========================================================================================================================== */
/* ================                                          BTMR                                             ================ */
/* =========================================================================================================================== */


/**
  * @brief Base timer (BTMR)
  */

typedef struct
{
  __IO uint32_t  CR1;                           /*!< (@0x00000000) control register 1                                          */
  __IO uint32_t  CCXCR1;                        /*!< (@0x00000004) channel control register 1                                  */
  __IO uint32_t  CCXCR2;                        /*!< (@0x00000008) channel control register 2                                  */
  __IO uint32_t  CEG;                           /*!< (@0x0000000C) event generation register                                   */
  __IO uint32_t  IER;                           /*!< (@0x00000010) interrupt enable register                                   */
  __IO uint32_t  SR;                            /*!< (@0x00000014) status register                                             */
  __IO uint32_t  CNT;                           /*!< (@0x00000018) counter                                                     */
  __IO uint32_t  PSC;                           /*!< (@0x0000001C) prescaler                                                   */
  __IO uint32_t  AUTORLD;                       /*!< (@0x00000020) auto-reload register                                        */
  __IO uint32_t  CC0;                           /*!< (@0x00000024) capture/compare register 0                                  */
} BTMR_TypeDef;


/* =========================================================================================================================== */
/* ================                                          LPTMR                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Low-power timer (LPTMR)
  */

typedef struct
{
  __IO uint32_t  CR;                            /*!< (@0x00000000) Control register                                            */
  __IO uint32_t  PSC;                           /*!< (@0x00000004) Prescaler register                                          */
  __IO uint32_t  WKVAL;                         /*!< (@0x00000008) Wakeup Value register                                       */
  __IO uint32_t  SR;                            /*!< (@0x0000000C) Status register                                             */
  __IO uint32_t  CNT;                           /*!< (@0x00000010) Counter register                                            */
} LPTMR_TypeDef;



/* =========================================================================================================================== */
/* ================                                           IWDT                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Independent watchdog (IWDT)
  */

typedef struct
{
  __IO uint32_t  KEY;                           /*!< (@0x00000000) Key register                                                */
  __IO uint32_t  PSC;                           /*!< (@0x00000004) Prescaler register                                          */
  __IO uint32_t  RL;                            /*!< (@0x00000008) Reload register                                             */
  __IO uint32_t  SR;                            /*!< (@0x0000000C) Status register                                             */
} IWDT_TypeDef;



/* =========================================================================================================================== */
/* ================                                           WWDT                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Window watchdog (WWDT)
  */

typedef struct
{
  __IO uint32_t  CR;                            /*!< (@0x00000000) Control register                                            */
  __IO uint32_t  CFG;                           /*!< (@0x00000004) Configuration register                                      */
  __IO uint32_t  SR;                            /*!< (@0x00000008) Status register                                             */
} WWDT_TypeDef;



/* =========================================================================================================================== */
/* ================                                            SPI                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Serial peripheral interface (SPI)
  */

typedef struct
{
  __IO uint32_t  CR1;                           /*!< (@0x00000000) control register 1                                          */
  __IO uint32_t  CR2;                           /*!< (@0x00000004) control register 2                                          */
  __IO uint32_t  SR;                            /*!< (@0x00000008) status register                                             */
  __IO uint32_t  DR;                            /*!< (@0x0000000C) data register                                               */
} SPI_TypeDef;



/* =========================================================================================================================== */
/* ================                                           USART                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Universal asynchronous receiver transmitter (USART)
  */

typedef struct
{
  __IO uint32_t  SR;                            /*!< (@0x00000000)                                                             */
  __IO uint32_t  DR;                            /*!< (@0x00000004)                                                             */
  __IO uint32_t  BRR;                           /*!< (@0x00000008)                                                             */
  __IO uint32_t  CR1;                           /*!< (@0x0000000C)                                                             */
  __IO uint32_t  CR2;                           /*!< (@0x00000010)                                                             */
  __IO uint32_t  CR3;                           /*!< (@0x00000014)                                                             */
       uint32_t  RESERVED0;
  __IO uint32_t  RXTOR;                         /*!< (@0x0000001C)                                                             */
} USART_TypeDef;



/* =========================================================================================================================== */
/* ================                                            I2C                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Inter-Integrated Circuit (I2C)
  */

typedef struct
{
  __IO uint32_t  CR1;                           /*!< (@0x00000000) control register 1                                          */
  __IO uint32_t  CR2;                           /*!< (@0x00000004) control register 2                                          */
  __IO uint32_t  SADDR1;                        /*!< (@0x00000008) Slave Mode Address Register 1                               */
  __IO uint32_t  SADDR2;                        /*!< (@0x0000000C) Slave Mode Address Register 2                               */
  __IO uint32_t  DR;                            /*!< (@0x00000010) data register                                               */
  __IO uint32_t  SR1;                           /*!< (@0x00000014) Status register 1                                           */
  __IO uint32_t  SR2;                           /*!< (@0x00000018) Status register 2                                           */
  __IO uint32_t  CLKCR;                         /*!< (@0x0000001C) Host Clock Control Register                                 */
  __IO uint32_t  RISETMAX;                      /*!< (@0x00000020) Maximum Rise Time Register                                  */
  __IO uint32_t  FILTER;                        /*!< (@0x00000024) Filter Control Register                                     */
} I2C_TypeDef;



/* =========================================================================================================================== */
/* ================                                            ADC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Analog-to-digital converter (ADC)
  */

typedef struct
{
  __IO uint32_t  SR;                            /*!< (@0x00000000) ADC interrupt and status register                           */
  __IO uint32_t  IER;                           /*!< (@0x00000004) ADC interrupt enable register                               */
  __IO uint32_t  CR;                            /*!< (@0x00000008) ADC control register                                        */
  __IO uint32_t  CFG1;                          /*!< (@0x0000000C) ADC configuration register 1                                */
  __IO uint32_t  CFG2;                          /*!< (@0x00000010) ADC configuration register 2                                */
  __IO uint32_t  SMP1;                          /*!< (@0x00000014) ADC sampling time register 1                                */
  __IO uint32_t  SMP2;                          /*!< (@0x00000018) ADC sampling time register 2                                */
  __IO uint32_t  CFG3;                          /*!< (@0x0000001C) ADC configuration register 3                                */
       uint32_t  RESERVED0[3];
  __IO uint32_t  SQ1;                           /*!< (@0x0000002C) ADC group regular sequencer register 1                      */
  __IO uint32_t  SQ2;                           /*!< (@0x00000030) ADC group regular sequencer register 2                      */
  __IO uint32_t  SQ3;                           /*!< (@0x00000034) ADC group regular sequencer register 3                      */
  __IO uint32_t  DR;                            /*!< (@0x00000038) ADC group regular data register                             */
  __IO uint32_t  DR0;                           /*!< (@0x0000003C) Segmented sampling result data 0                            */
  __IO uint32_t  DR1;                           /*!< (@0x00000040) Segmented sampling result data 1                            */
  __IO uint32_t  DR2;                           /*!< (@0x00000044) Segmented sampling result data 2                            */
  __IO uint32_t  DR3;                           /*!< (@0x00000048) Segmented sampling result data 3                            */
  __IO uint32_t  DR4;                           /*!< (@0x0000004C) Segmented sampling result data 4                            */
  __IO uint32_t  DR5;                           /*!< (@0x00000050) Segmented sampling result data 5                            */
  __IO uint32_t  DR6;                           /*!< (@0x00000054) Segmented sampling result data 6                            */
  __IO uint32_t  DR7;                           /*!< (@0x00000058) Segmented sampling result data 7                            */
  __IO uint32_t  SEQNUM;                        /*!< (@0x0000005C) The Number of Sequencial Section                            */
} ADC_TypeDef;

/* =========================================================================================================================== */
/* ================                                           OPA                                             ================ */
/* =========================================================================================================================== */

/**
  * @brief Operational amplifier (OPA)
  */

typedef struct
{
  __IO uint32_t  CR;                            /*!< (@0x00000000) Control register                                            */
} OPA_TypeDef;


/* =========================================================================================================================== */
/* ================                                           COMP0                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Comparator (COMP0)
  */

typedef struct
{
  __IO uint32_t  CR;                            /*!< (@0x00000000) Control Register                                            */
  __IO uint32_t  ISR;                           /*!< (@0x00000004) Status register                                             */
} COMP0_TypeDef;



/* =========================================================================================================================== */
/* ================                                           COMP1                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Comparator (COMP1)
  */

typedef struct
{
  __IO uint32_t  CR;                            /*!< (@0x00000000) Control Register                                            */
        uint32_t  RESERVED[2];
  __IO uint32_t  ISR;                           /*!< (@0x0000000C) Status register                                             */
} COMP1_TypeDef;



/* =========================================================================================================================== */
/* ================                                            DMA                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief DMA controller (DMA)
  */

typedef struct
{
  __IO uint32_t  ISR;                           /*!< (@0x00000000) Interrupt Status Register                                   */
       uint32_t  RESERVED0;
  __IO uint32_t  IFCLR;                         /*!< (@0x00000008) Interrupt Flag Clear Register                               */
} DMA_TypeDef;


typedef struct
{
  __IO  uint32_t  SCFG;                         /*!< SCFG                                                                      */
  __IO  uint32_t  NDATA;                        /*!< NDATA                                                                     */
  __IO  uint32_t  PADDR;                        /*!< PADDR                                                                     */
  __IO  uint32_t  M0ADDR;                       /*!< M0ADDR                                                                    */
  __IO  uint32_t  M1ADDR;                       /*!< M1ADDR                                                                    */
  __IO  uint32_t  FIFOCR;                       /*!< FCTRL                                                                     */
} DMA_Channel_TypeDef;



/* =========================================================================================================================== */
/* ================                                            RCC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Reset and clock control (RCC)
  */

typedef struct
{
  __IO uint32_t  KEY;                           /*!< (@0x00000000) RCC KEY register                                            */
  __IO uint32_t  CR;                            /*!< (@0x00000004) RCC control register                                        */
  __IO uint32_t  CFG;                           /*!< (@0x00000008) RCC configuration register                                  */
  __IO uint32_t  IER;                           /*!< (@0x0000000C) RCC Interrupt Enable Register                               */
  __IO uint32_t  ISR;                           /*!< (@0x00000010) RCC Interrupt Flag Status Register                          */
  __IO uint32_t  AHBRST;                        /*!< (@0x00000014) RCC AHB Clock Reset Register                                */
  __IO uint32_t  APBRST;                        /*!< (@0x00000018) RCC APB Clock Reset Register                                */
  __IO uint32_t  AHBCLKEN;                      /*!< (@0x0000001C) RCC AHB Clock Enable Register                               */
  __IO uint32_t  APBCLKEN;                      /*!< (@0x00000020) RCC APB Clock Enable Register                               */
  __IO uint32_t  ADCCR;                         /*!< (@0x00000024) RCC ADC Clock Control Register                              */
  __IO uint32_t  RSTCSR;                        /*!< (@0x00000028) RCC Reset Control and Status Register                       */
  __IO uint32_t  AONCSR;                        /*!< (@0x0000002C) AON Control Register                                        */
} RCC_TypeDef;



/* =========================================================================================================================== */
/* ================                                           FLASH                                           ================ */
/* =========================================================================================================================== */


/**
  * @brief Flash memory controller (FLASH)
  */

typedef struct
{
  __IO uint32_t  WKEY;                          /*!< (@0x00000000) /                                                           */
  __IO uint32_t  MKEY;                          /*!< (@0x00000004) /                                                           */
       uint32_t  RESERVED0;
  __IO uint32_t  NVRCKEY;                       /*!< (@0x0000000C) /                                                           */
        uint32_t  RESERVED1;
  __IO uint32_t  CR;                            /*!< (@0x00000014) /                                                           */
  __IO uint32_t  IER;                           /*!< (@0x00000018) /                                                           */
  __IO uint32_t  SR;                            /*!< (@0x0000001C) /                                                           */
  __IO uint32_t  CR1;                           /*!< (@0x00000020) /                                                           */
} FLASH_TypeDef;



/* =========================================================================================================================== */
/* ================                                            CRC                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Cryptographic processor (CRC)
  */

typedef struct
{
  __IO uint32_t  DR;                            /*!< (@0x00000000) Data register                                               */
  __IO uint32_t  CR;                            /*!< (@0x00000004) Control Register                                            */
  __IO uint32_t  INIT;                          /*!< (@0x00000008) Initial value data register                                 */
} CRC_TypeDef;



/* =========================================================================================================================== */
/* ================                                            DIV                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief DIV (DIV)
  */

typedef struct
{
  __IO uint32_t  DVDR;                          /*!< (@0x00000000) Dividend register                                           */
  __IO uint32_t  DVSR;                          /*!< (@0x00000004) Divisor Register                                            */
  __IO uint32_t  QUOTR;                         /*!< (@0x00000008) Quotient register                                           */
  __IO uint32_t  RMDR;                          /*!< (@0x0000000C) Remainder register                                          */
  __IO uint32_t  SR;                            /*!< (@0x00000010) Stauts register                                             */
  __IO uint32_t  CR;                            /*!< (@0x00000014) Control register                                            */
} DIV_TypeDef;



/* =========================================================================================================================== */
/* ================                                           GPIO                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief General-purpose I/Os (GPIO)
  */

typedef struct
{
  __IO uint32_t  MDR;                           /*!< (@0x00000000)                                                             */
  __IO uint32_t  INENR;                         /*!< (@0x00000004)                                                             */
       uint32_t  RESERVED0;
  __IO uint32_t  PUPDR;                         /*!< (@0x0000000C)                                                             */
  __IO uint32_t  OTR;                           /*!< (@0x00000010)                                                             */
  __IO uint32_t  DSR;                           /*!< (@0x00000014)                                                             */
  __IO uint32_t  INDR;                          /*!< (@0x00000018)                                                             */
  __IO uint32_t  OUTDR;                         /*!< (@0x0000001C)                                                             */
  __IO uint32_t  BSRR;                          /*!< (@0x00000020)                                                             */
  __IO uint32_t  BRR;                           /*!< (@0x00000024)                                                             */
  __IO uint32_t  LOCK;                          /*!< (@0x00000028)                                                             */
  __IO uint32_t  AFSELR0;                       /*!< (@0x0000002C)                                                             */
  __IO uint32_t  AFSELR1;                       /*!< (@0x00000030)                                                             */
       uint32_t  RESERVED1[8];
  __IO uint32_t  AF3RMP;                        /*!< (@0x00000054) AF3RMP1R (Only for GPIOA)                                   */
} GPIO_TypeDef;



/** @} */ /* End of group Device_Peripheral_peripherals */


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */

#define FLASH_BASE            0x00000000UL      /*!< FLASH base address in the alias region                                     */
#define FLASH_END             0x0000FFFFUL      /*!< FLASH end address                                                          */
#define SRAM_BASE             0x20000000UL      /*!< SRAM base address in the alias region                                      */

/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

#define PMU_BASE                    0x40000000UL
#define SCU_BASE                    0x40000400UL
#define EINT_BASE                   0x40000800UL
#define ATMR_BASE                   0x40000C00UL
#define GTMR_BASE                   0x40001000UL
#define BTMR0_BASE                  0x40001800UL
#define BTMR1_BASE                  0x40001C00UL
#define LPTMR_BASE                  0x40002000UL
#define IWDT_BASE                   0x40002800UL
#define WWDT_BASE                   0x40002C00UL
#define SPI_BASE                    0x40003000UL
#define USART_BASE                  0x40003800UL
#define UART_BASE                   0x40003C00UL
#define I2C_BASE                    0x40004400UL
#define ADC_BASE                    0x40005400UL
#define OPA_BASE                    0x40005800UL
#define COMP0_BASE                  0x40005C00UL
#define COMP1_BASE                  0x40006000UL
#define COMP2_BASE                  0x40006004UL
#define COMP3_BASE                  0x40006008UL
#define DMA_BASE                    0x40020000UL
#define DMA_Channel0_BASE           0x40020010UL
#define DMA_Channel1_BASE           0x40020028UL
#define RCC_BASE                    0x40020400UL
#define FLASH_R_BASE                0x40020800UL
#define CRC_BASE                    0x40020C00UL
#define DIV_BASE                    0x40021000UL
#define GPIOA_BASE                  0x48000000UL
#define GPIOB_BASE                  0x48000400UL

#define UID_BASE                    0x00101E88UL           /*!< Unique device ID register base address */
#define PID_BASE                    0x00101D80UL           /*!< PID register base address */

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define PMU                     ((PMU_TypeDef*)               PMU_BASE)
#define SCU                     ((SCU_TypeDef*)               SCU_BASE)
#define EINT                    ((EINT_TypeDef*)              EINT_BASE)
#define ATMR                    ((ATMR_TypeDef*)              ATMR_BASE)
#define GTMR                    ((GTMR_TypeDef*)              GTMR_BASE)
#define BTMR0                   ((BTMR_TypeDef*)              BTMR0_BASE)
#define BTMR1                   ((BTMR_TypeDef*)              BTMR1_BASE)
#define LPTMR                   ((LPTMR_TypeDef*)             LPTMR_BASE)
#define IWDT                    ((IWDT_TypeDef*)              IWDT_BASE)
#define WWDT                    ((WWDT_TypeDef*)              WWDT_BASE)
#define SPI                     ((SPI_TypeDef*)               SPI_BASE)
#define USART                   ((USART_TypeDef*)             USART_BASE)
#define UART                    ((USART_TypeDef*)             UART_BASE)
#define I2C                     ((I2C_TypeDef*)               I2C_BASE)
#define ADC                     ((ADC_TypeDef*)               ADC_BASE)
#define OPA                     ((OPA_TypeDef*)               OPA_BASE)
#define COMP0                   ((COMP0_TypeDef*)             COMP0_BASE)
#define COMP1                   ((COMP1_TypeDef*)             COMP1_BASE)
#define COMP2                   ((COMP1_TypeDef*)             COMP2_BASE)
#define COMP3                   ((COMP1_TypeDef*)             COMP3_BASE)
#define DMA                     ((DMA_TypeDef*)               DMA_BASE)
#define DMA_Channel0            ((DMA_Channel_TypeDef *)      DMA_Channel0_BASE)
#define DMA_Channel1            ((DMA_Channel_TypeDef *)      DMA_Channel1_BASE)
#define RCC                     ((RCC_TypeDef*)               RCC_BASE)
#define FLASH                   ((FLASH_TypeDef*)             FLASH_R_BASE)
#define CRC                     ((CRC_TypeDef*)               CRC_BASE)
#define DIV                     ((DIV_TypeDef*)               DIV_BASE)
#define GPIOA                   ((GPIO_TypeDef*)              GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef*)              GPIOB_BASE)

/** @} */ /* End of group Device_Peripheral_declaration */


/* =========================================================================================================================== */
/* ================                                Pos/Mask Peripheral Section                                ================ */
/* =========================================================================================================================== */


/** @addtogroup PosMask_peripherals
  * @{
  */


/* ================================================================================================================================== */
/* =============================================================  PMU  ============================================================== */
/* ================================================================================================================================== */
/* =============================================  Bit definition for PMU_KEY register  ============================================== */
#define PMU_KEY_LOCKKEY_Pos         (0U)                                        /*!< Counter enable */
#define PMU_KEY_LOCKKEY_Msk         (0xFFFFUL << PMU_KEY_LOCKKEY_Pos)           /*!< 0x0000FFFF */
#define PMU_KEY_LOCKKEY             PMU_KEY_LOCKKEY_Msk
#define PMU_KEY_LOCKFLG_Pos         (16U)                                       /*!< Update disable */
#define PMU_KEY_LOCKFLG_Msk         (0x1UL << PMU_KEY_LOCKFLG_Pos)              /*!< 0x00010000 */
#define PMU_KEY_LOCKFLG             PMU_KEY_LOCKFLG_Msk

#define PMU_KEY_VALUE               (0x87E4U)

/* ============================================  Bit definition for PMU_LPCR register  ============================================= */
#define PMU_LPCR_LPCFG_Pos          (0U)                                        /*!< Low Power Mode */
#define PMU_LPCR_LPCFG_Msk          (0x1UL << PMU_LPCR_LPCFG_Pos)               /*!< 0x00000001 */
#define PMU_LPCR_LPCFG              PMU_LPCR_LPCFG_Msk
#define PMU_LPCR_STANDBYCFG_Pos     (1U)                                        /*!< STANDBY Operation Mode */
#define PMU_LPCR_STANDBYCFG_Msk     (0x1UL << PMU_LPCR_STANDBYCFG_Pos)          /*!< 0x00000002 */
#define PMU_LPCR_STANDBYCFG         PMU_LPCR_STANDBYCFG_Msk

/* ============================================  Bit definition for PMU_WKCR register  ============================================= */
#define PMU_WKCR_WKEN0_Pos          (0U)                                        /*!< Enable wake-up from STANDBY by pin 0 */
#define PMU_WKCR_WKEN0_Msk          (0x1UL << PMU_WKCR_WKEN0_Pos)               /*!< 0x00000001 */
#define PMU_WKCR_WKEN0              PMU_WKCR_WKEN0_Msk
#define PMU_WKCR_WKEN1_Pos          (1U)                                        /*!< Enable wake-up from STANDBY by pin 1 */
#define PMU_WKCR_WKEN1_Msk          (0x1UL << PMU_WKCR_WKEN1_Pos)               /*!< 0x00000002 */
#define PMU_WKCR_WKEN1              PMU_WKCR_WKEN1_Msk
#define PMU_WKCR_WKPOL0_Pos         (8U)                                        /*!< Wake-up polarity of pin 0 for STANDBY */
#define PMU_WKCR_WKPOL0_0           (0x1UL << PMU_WKCR_WKPOL0_Pos)              /*!< 0x00000100 */
#define PMU_WKCR_WKPOL0_1           (0x2UL << PMU_WKCR_WKPOL0_Pos)              /*!< 0x00000200 */
#define PMU_WKCR_WKPOL0_Msk         (0x3UL << PMU_WKCR_WKPOL0_Pos)              /*!< 0x00000300 */
#define PMU_WKCR_WKPOL0             PMU_WKCR_WKPOL0_Msk
#define PMU_WKCR_WKPOL1_Pos         (10U)                                       /*!< Wake-up polarity of pin 1 for STANDBY */
#define PMU_WKCR_WKPOL1_0           (0x1UL << PMU_WKCR_WKPOL1_Pos)              /*!< 0x00000400 */
#define PMU_WKCR_WKPOL1_1           (0x2UL << PMU_WKCR_WKPOL1_Pos)              /*!< 0x00000800 */
#define PMU_WKCR_WKPOL1_Msk         (0x3UL << PMU_WKCR_WKPOL1_Pos)              /*!< 0x00000C00 */
#define PMU_WKCR_WKPOL1             PMU_WKCR_WKPOL1_Msk
#define PMU_WKCR_WKPUS0_Pos         (16U)                                       /*!< Wake-up pin 0 pull-up/pull-down control in STANDBY mode */
#define PMU_WKCR_WKPUS0_0           (0x1UL << PMU_WKCR_WKPUS0_Pos)              /*!< 0x00010000 */
#define PMU_WKCR_WKPUS0_1           (0x2UL << PMU_WKCR_WKPUS0_Pos)              /*!< 0x00020000 */
#define PMU_WKCR_WKPUS0_Msk         (0x3UL << PMU_WKCR_WKPUS0_Pos)              /*!< 0x00030000 */
#define PMU_WKCR_WKPUS0             PMU_WKCR_WKPUS0_Msk
#define PMU_WKCR_WKPUS1_Pos         (18U)                                       /*!< Wake-up pin 1 pull-up/pull-down control in STANDBY mode */
#define PMU_WKCR_WKPUS1_0           (0x1UL << PMU_WKCR_WKPUS1_Pos)              /*!< 0x00040000 */
#define PMU_WKCR_WKPUS1_1           (0x2UL << PMU_WKCR_WKPUS1_Pos)              /*!< 0x00080000 */
#define PMU_WKCR_WKPUS1_Msk         (0x3UL << PMU_WKCR_WKPUS1_Pos)              /*!< 0x000C0000 */
#define PMU_WKCR_WKPUS1             PMU_WKCR_WKPUS1_Msk
#define PMU_WKCR_IWDTWKEN_Pos       (24U)                                       /*!< Enable IWDT wake-up in STANDBY mode */
#define PMU_WKCR_IWDTWKEN_Msk       (0x1UL << PMU_WKCR_IWDTWKEN_Pos)            /*!< 0x01000000 */
#define PMU_WKCR_IWDTWKEN           PMU_WKCR_IWDTWKEN_Msk
#define PMU_WKCR_LPTMRWKEN_Pos      (25U)                                       /*!< Enable LPTMR wake-up in STANDBY mode */
#define PMU_WKCR_LPTMRWKEN_Msk      (0x1UL << PMU_WKCR_LPTMRWKEN_Pos)           /*!< 0x02000000 */
#define PMU_WKCR_LPTMRWKEN          PMU_WKCR_LPTMRWKEN_Msk

/* ============================================  Bit definition for PMU_WKSR register  ============================================= */
#define PMU_WKSR_WKFLG0_Pos         (0U)                                        /*!< Wake-up pin 0 to STANDBY wake-up flag */
#define PMU_WKSR_WKFLG0_Msk         (0x1UL << PMU_WKSR_WKFLG0_Pos)              /*!< 0x00000001 */
#define PMU_WKSR_WKFLG0             PMU_WKSR_WKFLG0_Msk
#define PMU_WKSR_WKFLG1_Pos         (1U)                                        /*!< Wake-up pin 1 to STANDBY wake-up flag */
#define PMU_WKSR_WKFLG1_Msk         (0x1UL << PMU_WKSR_WKFLG1_Pos)              /*!< 0x00000002 */
#define PMU_WKSR_WKFLG1             PMU_WKSR_WKFLG1_Msk
#define PMU_WKSR_STANDBYFLG_Pos     (4U)                                        /*!< STANDBY Event Flag */
#define PMU_WKSR_STANDBYFLG_Msk     (0x1UL << PMU_WKSR_STANDBYFLG_Pos)          /*!< 0x00000010 */
#define PMU_WKSR_STANDBYFLG         PMU_WKSR_STANDBYFLG_Msk
#define PMU_WKSR_IWDTWKFLG_Pos      (24U)                                       /*!< IWDT Standby Wake-up Flag */
#define PMU_WKSR_IWDTWKFLG_Msk      (0x1UL << PMU_WKSR_IWDTWKFLG_Pos)           /*!< 0x01000000 */
#define PMU_WKSR_IWDTWKFLG          PMU_WKSR_IWDTWKFLG_Msk
#define PMU_WKSR_LPTMRWKFLG_Pos     (25U)                                       /*!< LPTMR to STANDBY wake-up flag */
#define PMU_WKSR_LPTMRWKFLG_Msk     (0x1UL << PMU_WKSR_LPTMRWKFLG_Pos)          /*!< 0x02000000 */
#define PMU_WKSR_LPTMRWKFLG         PMU_WKSR_LPTMRWKFLG_Msk

/* ============================================  Bit definition for PMU_PVDCSR register  ============================================ */
#define PMU_PVDCSR_PVDEN_Pos        (0U)                                        /*!< PVD Enable */
#define PMU_PVDCSR_PVDEN_Msk        (0x1UL << PMU_PVDCSR_PVDEN_Pos)             /*!< 0x00000001 */
#define PMU_PVDCSR_PVDEN            PMU_PVDCSR_PVDEN_Msk
#define PMU_PVDCSR_PVDTHSEL_Pos     (1U)                                        /*!< PVD threshold voltage configuration */
#define PMU_PVDCSR_PVDTHSEL_0       (0x1UL << PMU_PVDCSR_PVDTHSEL_Pos)          /*!< 0x00000002 */
#define PMU_PVDCSR_PVDTHSEL_1       (0x2UL << PMU_PVDCSR_PVDTHSEL_Pos)          /*!< 0x00000004 */
#define PMU_PVDCSR_PVDTHSEL_2       (0x4UL << PMU_PVDCSR_PVDTHSEL_Pos)          /*!< 0x00000008 */
#define PMU_PVDCSR_PVDTHSEL_Msk     (0x7UL << PMU_PVDCSR_PVDTHSEL_Pos)          /*!< 0x0000000E */
#define PMU_PVDCSR_PVDTHSEL         PMU_PVDCSR_PVDTHSEL_Msk
#define PMU_PVDCSR_PVDLT_Pos        (4U)                                        /*!< PVD below threshold voltage event detection. */
#define PMU_PVDCSR_PVDLT_Msk        (0x1UL << PMU_PVDCSR_PVDLT_Pos)             /*!< 0x00000010 */
#define PMU_PVDCSR_PVDLT            PMU_PVDCSR_PVDLT_Msk
#define PMU_PVDCSR_PVDHT_Pos        (5U)                                        /*!< PVD above threshold voltage event monitoring. */
#define PMU_PVDCSR_PVDHT_Msk        (0x1UL << PMU_PVDCSR_PVDHT_Pos)             /*!< 0x00000020 */
#define PMU_PVDCSR_PVDHT            PMU_PVDCSR_PVDHT_Msk
#define PMU_PVDCSR_PVDIEN_Pos       (8U)                                        /*!< PVD monitoring event interrupt enable */
#define PMU_PVDCSR_PVDIEN_Msk       (0x1UL << PMU_PVDCSR_PVDIEN_Pos)            /*!< 0x00000100 */
#define PMU_PVDCSR_PVDIEN           PMU_PVDCSR_PVDIEN_Msk
#define PMU_PVDCSR_PVDFLTEN_Pos     (9U)                                        /*!< PVD filter enable */
#define PMU_PVDCSR_PVDFLTEN_Msk     (0x1UL << PMU_PVDCSR_PVDFLTEN_Pos)          /*!< 0x00000200 */
#define PMU_PVDCSR_PVDFLTEN         PMU_PVDCSR_PVDFLTEN_Msk
#define PMU_PVDCSR_PVDFLTSEL_Pos    (10U)                                       /*!< PVD filter length select */
#define PMU_PVDCSR_PVDFLTSEL_0      (0x1UL << PMU_PVDCSR_PVDFLTSEL_Pos)         /*!< 0x00000400 */
#define PMU_PVDCSR_PVDFLTSEL_1      (0x2UL << PMU_PVDCSR_PVDFLTSEL_Pos)         /*!< 0x00000800 */
#define PMU_PVDCSR_PVDFLTSEL_2      (0x4UL << PMU_PVDCSR_PVDFLTSEL_Pos)         /*!< 0x00001000 */
#define PMU_PVDCSR_PVDFLTSEL_Msk    (0x7UL << PMU_PVDCSR_PVDFLTSEL_Pos)         /*!< 0x00001C00 */
#define PMU_PVDCSR_PVDFLTSEL        PMU_PVDCSR_PVDFLTSEL_Msk
#define PMU_PVDCSR_PVDFLG_Pos       (14U)                                       /*!< PVD Monitoring Event Flag */
#define PMU_PVDCSR_PVDFLG_Msk       (0x1UL << PMU_PVDCSR_PVDFLG_Pos)            /*!< 0x00004000 */
#define PMU_PVDCSR_PVDFLG           PMU_PVDCSR_PVDFLG_Msk
#define PMU_PVDCSR_PVDSTS_Pos       (15U)                                       /*!< PVD Monitoring Results Output */
#define PMU_PVDCSR_PVDSTS_Msk       (0x1UL << PMU_PVDCSR_PVDSTS_Pos)            /*!< 0x00008000 */
#define PMU_PVDCSR_PVDSTS           PMU_PVDCSR_PVDSTS_Msk
#define PMU_PVDCSR_PVDRDY_Pos       (16U)                                       /*!< PVD Ready */
#define PMU_PVDCSR_PVDRDY_Msk       (0x1UL << PMU_PVDCSR_PVDRDY_Pos)            /*!< 0x00010000 */
#define PMU_PVDCSR_PVDRDY           PMU_PVDCSR_PVDRDY_Msk

/* ================================================================================================================================== */
/* =============================================================  SCU  ============================================================== */
/* ================================================================================================================================== */
/* =============================================  Bit definition for SCU_KEY register  ============================================== */
#define SCU_KEY_LOCKKEY_Pos         (0U)                                        /*!< Counter enable */
#define SCU_KEY_LOCKKEY_Msk         (0xFFFFUL << SCU_KEY_LOCKKEY_Pos)           /*!< 0x0000FFFF */
#define SCU_KEY_LOCKKEY             SCU_KEY_LOCKKEY_Msk
#define SCU_KEY_LOCKFLG_Pos         (16U)                                       /*!< Update disable */
#define SCU_KEY_LOCKFLG_Msk         (0x1UL << SCU_KEY_LOCKFLG_Pos)              /*!< 0x00010000 */
#define SCU_KEY_LOCKFLG             SCU_KEY_LOCKFLG_Msk

#define SCU_KEY_VALUE               (0x87E4U)

/* =============================================  Bit definition for SCU_DBG register  ============================================== */
#define SCU_DBG_ATMRDBG_Pos         (0U)                                        /*!< In DEBUG mode, ATMR counting method */
#define SCU_DBG_ATMRDBG_Msk         (0x1UL << SCU_DBG_ATMRDBG_Pos)              /*!< 0x00000001 */
#define SCU_DBG_ATMRDBG             SCU_DBG_ATMRDBG_Msk
#define SCU_DBG_GTMRDBG_Pos         (1U)                                        /*!< In DEBUG mode, GTMR counting method */
#define SCU_DBG_GTMRDBG_Msk         (0x1UL << SCU_DBG_GTMRDBG_Pos)              /*!< 0x00000002 */
#define SCU_DBG_GTMRDBG             SCU_DBG_GTMRDBG_Msk
#define SCU_DBG_BTMR0DBG_Pos        (3U)                                        /*!< In DEBUG mode, BTMR0 counting method */
#define SCU_DBG_BTMR0DBG_Msk        (0x1UL << SCU_DBG_BTMR0DBG_Pos)             /*!< 0x00000008 */
#define SCU_DBG_BTMR0DBG            SCU_DBG_BTMR0DBG_Msk
#define SCU_DBG_BTMR1DBG_Pos        (4U)                                        /*!< In DEBUG mode, BTMR1 counting method */
#define SCU_DBG_BTMR1DBG_Msk        (0x1UL << SCU_DBG_BTMR1DBG_Pos)             /*!< 0x00000010 */
#define SCU_DBG_BTMR1DBG            SCU_DBG_BTMR1DBG_Msk
#define SCU_DBG_LPTMRDBG_Pos        (5U)                                        /*!< In DEBUG mode, LPTMR counting method */
#define SCU_DBG_LPTMRDBG_Msk        (0x1UL << SCU_DBG_LPTMRDBG_Pos)             /*!< 0x00000020 */
#define SCU_DBG_LPTMRDBG            SCU_DBG_LPTMRDBG_Msk
#define SCU_DBG_IWDTDBG_Pos         (6U)                                        /*!< In DEBUG mode, IWDT counting method */
#define SCU_DBG_IWDTDBG_Msk         (0x1UL << SCU_DBG_IWDTDBG_Pos)              /*!< 0x00000040 */
#define SCU_DBG_IWDTDBG             SCU_DBG_IWDTDBG_Msk
#define SCU_DBG_WWDTDBG_Pos         (7U)                                        /*!< In DEBUG mode, WWDT counting method */
#define SCU_DBG_WWDTDBG_Msk         (0x1UL << SCU_DBG_WWDTDBG_Pos)              /*!< 0x00000080 */
#define SCU_DBG_WWDTDBG             SCU_DBG_WWDTDBG_Msk

/* =========================================================================================================================== */
/* ================                                           EINT                                            ================ */
/* =========================================================================================================================== */

/* =========================================================  IMASK  ========================================================= */
#define EINT_IMASK_IMASK0_Pos       (0U)
#define EINT_IMASK_IMASK0_Msk       (0x1UL << EINT_IMASK_IMASK0_Pos)            /*!< 0x00000001 */
#define EINT_IMASK_IMASK0           EINT_IMASK_IMASK0_Msk                       /*!< Interrupt Mask on line 0 */
#define EINT_IMASK_IMASK1_Pos       (1U)
#define EINT_IMASK_IMASK1_Msk       (0x1UL << EINT_IMASK_IMASK1_Pos)            /*!< 0x00000002 */
#define EINT_IMASK_IMASK1           EINT_IMASK_IMASK1_Msk                       /*!< Interrupt Mask on line 1 */
#define EINT_IMASK_IMASK2_Pos       (2U)
#define EINT_IMASK_IMASK2_Msk       (0x1UL << EINT_IMASK_IMASK2_Pos)            /*!< 0x00000004 */
#define EINT_IMASK_IMASK2           EINT_IMASK_IMASK2_Msk                       /*!< Interrupt Mask on line 2 */
#define EINT_IMASK_IMASK3_Pos       (3U)
#define EINT_IMASK_IMASK3_Msk       (0x1UL << EINT_IMASK_IMASK3_Pos)            /*!< 0x00000008 */
#define EINT_IMASK_IMASK3           EINT_IMASK_IMASK3_Msk                       /*!< Interrupt Mask on line 3 */
#define EINT_IMASK_IMASK4_Pos       (4U)
#define EINT_IMASK_IMASK4_Msk       (0x1UL << EINT_IMASK_IMASK4_Pos)            /*!< 0x00000010 */
#define EINT_IMASK_IMASK4           EINT_IMASK_IMASK4_Msk                       /*!< Interrupt Mask on line 4 */
#define EINT_IMASK_IMASK5_Pos       (5U)
#define EINT_IMASK_IMASK5_Msk       (0x1UL << EINT_IMASK_IMASK5_Pos)            /*!< 0x00000020 */
#define EINT_IMASK_IMASK5           EINT_IMASK_IMASK5_Msk                       /*!< Interrupt Mask on line 5 */
#define EINT_IMASK_IMASK6_Pos       (6U)
#define EINT_IMASK_IMASK6_Msk       (0x1UL << EINT_IMASK_IMASK6_Pos)            /*!< 0x00000040 */
#define EINT_IMASK_IMASK6           EINT_IMASK_IMASK6_Msk                       /*!< Interrupt Mask on line 6 */
#define EINT_IMASK_IMASK7_Pos       (7U)
#define EINT_IMASK_IMASK7_Msk       (0x1UL << EINT_IMASK_IMASK7_Pos)            /*!< 0x00000080 */
#define EINT_IMASK_IMASK7           EINT_IMASK_IMASK7_Msk                       /*!< Interrupt Mask on line 7 */
#define EINT_IMASK_IMASK8_Pos       (8U)
#define EINT_IMASK_IMASK8_Msk       (0x1UL << EINT_IMASK_IMASK8_Pos)            /*!< 0x00000100 */
#define EINT_IMASK_IMASK8           EINT_IMASK_IMASK8_Msk                       /*!< Interrupt Mask on line 8 */
#define EINT_IMASK_IMASK9_Pos       (9U)
#define EINT_IMASK_IMASK9_Msk       (0x1UL << EINT_IMASK_IMASK9_Pos)            /*!< 0x00000200 */
#define EINT_IMASK_IMASK9           EINT_IMASK_IMASK9_Msk                       /*!< Interrupt Mask on line 9 */
#define EINT_IMASK_IMASK10_Pos      (10U)
#define EINT_IMASK_IMASK10_Msk      (0x1UL << EINT_IMASK_IMASK10_Pos)           /*!< 0x00000400 */
#define EINT_IMASK_IMASK10          EINT_IMASK_IMASK10_Msk                      /*!< Interrupt Mask on line 10 */
#define EINT_IMASK_IMASK11_Pos      (11U)
#define EINT_IMASK_IMASK11_Msk      (0x1UL << EINT_IMASK_IMASK11_Pos)           /*!< 0x00000800 */
#define EINT_IMASK_IMASK11          EINT_IMASK_IMASK11_Msk                      /*!< Interrupt Mask on line 11 */
#define EINT_IMASK_IMASK12_Pos      (12U)
#define EINT_IMASK_IMASK12_Msk      (0x1UL << EINT_IMASK_IMASK12_Pos)           /*!< 0x00001000 */
#define EINT_IMASK_IMASK12          EINT_IMASK_IMASK12_Msk                      /*!< Interrupt Mask on line 12 */
#define EINT_IMASK_IMASK13_Pos      (13U)
#define EINT_IMASK_IMASK13_Msk      (0x1UL << EINT_IMASK_IMASK13_Pos)           /*!< 0x00002000 */
#define EINT_IMASK_IMASK13          EINT_IMASK_IMASK13_Msk                      /*!< Interrupt Mask on line 13 */
#define EINT_IMASK_IMASK14_Pos      (14U)
#define EINT_IMASK_IMASK14_Msk      (0x1UL << EINT_IMASK_IMASK14_Pos)           /*!< 0x00004000 */
#define EINT_IMASK_IMASK14          EINT_IMASK_IMASK14_Msk                      /*!< Interrupt Mask on line 14 */
#define EINT_IMASK_IMASK15_Pos      (15U)
#define EINT_IMASK_IMASK15_Msk      (0x1UL << EINT_IMASK_IMASK15_Pos)           /*!< 0x00008000 */
#define EINT_IMASK_IMASK15          EINT_IMASK_IMASK15_Msk                      /*!< Interrupt Mask on line 15 */
#define EINT_IMASK_IMASK16_Pos      (16U)
#define EINT_IMASK_IMASK16_Msk      (0x1UL << EINT_IMASK_IMASK16_Pos)           /*!< 0x00010000 */
#define EINT_IMASK_IMASK16          EINT_IMASK_IMASK16_Msk                      /*!< Interrupt Mask on line 16 */
#define EINT_IMASK_IMASK17_Pos      (17U)
#define EINT_IMASK_IMASK17_Msk      (0x1UL << EINT_IMASK_IMASK17_Pos)           /*!< 0x00020000 */
#define EINT_IMASK_IMASK17          EINT_IMASK_IMASK17_Msk                      /*!< Interrupt Mask on line 17 */

/* Reference Defines */
#define EINT_IMASK_IM0              EINT_IMASK_IMASK0
#define EINT_IMASK_IM1              EINT_IMASK_IMASK1
#define EINT_IMASK_IM2              EINT_IMASK_IMASK2
#define EINT_IMASK_IM3              EINT_IMASK_IMASK3
#define EINT_IMASK_IM4              EINT_IMASK_IMASK4
#define EINT_IMASK_IM5              EINT_IMASK_IMASK5
#define EINT_IMASK_IM6              EINT_IMASK_IMASK6
#define EINT_IMASK_IM7              EINT_IMASK_IMASK7
#define EINT_IMASK_IM8              EINT_IMASK_IMASK8
#define EINT_IMASK_IM9              EINT_IMASK_IMASK9
#define EINT_IMASK_IM10             EINT_IMASK_IMASK10
#define EINT_IMASK_IM11             EINT_IMASK_IMASK11
#define EINT_IMASK_IM12             EINT_IMASK_IMASK12
#define EINT_IMASK_IM13             EINT_IMASK_IMASK13
#define EINT_IMASK_IM14             EINT_IMASK_IMASK14
#define EINT_IMASK_IM15             EINT_IMASK_IMASK15
#define EINT_IMASK_IM16             EINT_IMASK_IMASK16
#define EINT_IMASK_IM17             EINT_IMASK_IMASK17
#define EINT_IMASK_IM_Pos           (0U)
#define EINT_IMASK_IM_Msk           (0x3FFFFUL << EINT_IMASK_IM_Pos)            /*!< 0x007FFFFF */
#define EINT_IMASK_IM               EINT_IMASK_IM_Msk                           /*!< Interrupt Mask All */
/* =========================================================  EMASK  ========================================================= */
#define EINT_EMASK_EMASK0_Pos       (0U)
#define EINT_EMASK_EMASK0_Msk       (0x1UL << EINT_EMASK_EMASK0_Pos)            /*!< 0x00000001 */
#define EINT_EMASK_EMASK0           EINT_EMASK_EMASK0_Msk                       /*!< Event Mask on line 0 */
#define EINT_EMASK_EMASK1_Pos       (1U)
#define EINT_EMASK_EMASK1_Msk       (0x1UL << EINT_EMASK_EMASK1_Pos)            /*!< 0x00000002 */
#define EINT_EMASK_EMASK1           EINT_EMASK_EMASK1_Msk                       /*!< Event Mask on line 1 */
#define EINT_EMASK_EMASK2_Pos       (2U)
#define EINT_EMASK_EMASK2_Msk       (0x1UL << EINT_EMASK_EMASK2_Pos)            /*!< 0x00000004 */
#define EINT_EMASK_EMASK2           EINT_EMASK_EMASK2_Msk                       /*!< Event Mask on line 2 */
#define EINT_EMASK_EMASK3_Pos       (3U)
#define EINT_EMASK_EMASK3_Msk       (0x1UL << EINT_EMASK_EMASK3_Pos)            /*!< 0x00000008 */
#define EINT_EMASK_EMASK3           EINT_EMASK_EMASK3_Msk                       /*!< Event Mask on line 3 */
#define EINT_EMASK_EMASK4_Pos       (4U)
#define EINT_EMASK_EMASK4_Msk       (0x1UL << EINT_EMASK_EMASK4_Pos)            /*!< 0x00000010 */
#define EINT_EMASK_EMASK4           EINT_EMASK_EMASK4_Msk                       /*!< Event Mask on line 4 */
#define EINT_EMASK_EMASK5_Pos       (5U)
#define EINT_EMASK_EMASK5_Msk       (0x1UL << EINT_EMASK_EMASK5_Pos)            /*!< 0x00000020 */
#define EINT_EMASK_EMASK5           EINT_EMASK_EMASK5_Msk                       /*!< Event Mask on line 5 */
#define EINT_EMASK_EMASK6_Pos       (6U)
#define EINT_EMASK_EMASK6_Msk       (0x1UL << EINT_EMASK_EMASK6_Pos)            /*!< 0x00000040 */
#define EINT_EMASK_EMASK6           EINT_EMASK_EMASK6_Msk                       /*!< Event Mask on line 6 */
#define EINT_EMASK_EMASK7_Pos       (7U)
#define EINT_EMASK_EMASK7_Msk       (0x1UL << EINT_EMASK_EMASK7_Pos)            /*!< 0x00000080 */
#define EINT_EMASK_EMASK7           EINT_EMASK_EMASK7_Msk                       /*!< Event Mask on line 7 */
#define EINT_EMASK_EMASK8_Pos       (8U)
#define EINT_EMASK_EMASK8_Msk       (0x1UL << EINT_EMASK_EMASK8_Pos)            /*!< 0x00000100 */
#define EINT_EMASK_EMASK8           EINT_EMASK_EMASK8_Msk                       /*!< Event Mask on line 8 */
#define EINT_EMASK_EMASK9_Pos       (9U)
#define EINT_EMASK_EMASK9_Msk       (0x1UL << EINT_EMASK_EMASK9_Pos)            /*!< 0x00000200 */
#define EINT_EMASK_EMASK9           EINT_EMASK_EMASK9_Msk                       /*!< Event Mask on line 9 */
#define EINT_EMASK_EMASK10_Pos      (10U)
#define EINT_EMASK_EMASK10_Msk      (0x1UL << EINT_EMASK_EMASK10_Pos)           /*!< 0x00000400 */
#define EINT_EMASK_EMASK10          EINT_EMASK_EMASK10_Msk                      /*!< Event Mask on line 10 */
#define EINT_EMASK_EMASK11_Pos      (11U)
#define EINT_EMASK_EMASK11_Msk      (0x1UL << EINT_EMASK_EMASK11_Pos)           /*!< 0x00000800 */
#define EINT_EMASK_EMASK11          EINT_EMASK_EMASK11_Msk                      /*!< Event Mask on line 11 */
#define EINT_EMASK_EMASK12_Pos      (12U)
#define EINT_EMASK_EMASK12_Msk      (0x1UL << EINT_EMASK_EMASK12_Pos)           /*!< 0x00001000 */
#define EINT_EMASK_EMASK12          EINT_EMASK_EMASK12_Msk                      /*!< Event Mask on line 12 */
#define EINT_EMASK_EMASK13_Pos      (13U)
#define EINT_EMASK_EMASK13_Msk      (0x1UL << EINT_EMASK_EMASK13_Pos)           /*!< 0x00002000 */
#define EINT_EMASK_EMASK13          EINT_EMASK_EMASK13_Msk                      /*!< Event Mask on line 13 */
#define EINT_EMASK_EMASK14_Pos      (14U)
#define EINT_EMASK_EMASK14_Msk      (0x1UL << EINT_EMASK_EMASK14_Pos)           /*!< 0x00004000 */
#define EINT_EMASK_EMASK14          EINT_EMASK_EMASK14_Msk                      /*!< Event Mask on line 14 */
#define EINT_EMASK_EMASK15_Pos      (15U)
#define EINT_EMASK_EMASK15_Msk      (0x1UL << EINT_EMASK_EMASK15_Pos)           /*!< 0x00008000 */
#define EINT_EMASK_EMASK15          EINT_EMASK_EMASK15_Msk                      /*!< Event Mask on line 15 */
#define EINT_EMASK_EMASK16_Pos      (16U)
#define EINT_EMASK_EMASK16_Msk      (0x1UL << EINT_EMASK_EMASK16_Pos)           /*!< 0x00010000 */
#define EINT_EMASK_EMASK16          EINT_EMASK_EMASK16_Msk                      /*!< Event Mask on line 16 */
#define EINT_EMASK_EMASK17_Pos      (17U)
#define EINT_EMASK_EMASK17_Msk      (0x1UL << EINT_EMASK_EMASK17_Pos)           /*!< 0x00020000 */
#define EINT_EMASK_EMASK17          EINT_EMASK_EMASK17_Msk                      /*!< Event Mask on line 17 */

/* Reference Defines */
#define EINT_EMASK_EM0              EINT_EMASK_EMASK0
#define EINT_EMASK_EM1              EINT_EMASK_EMASK1
#define EINT_EMASK_EM2              EINT_EMASK_EMASK2
#define EINT_EMASK_EM3              EINT_EMASK_EMASK3
#define EINT_EMASK_EM4              EINT_EMASK_EMASK4
#define EINT_EMASK_EM5              EINT_EMASK_EMASK5
#define EINT_EMASK_EM6              EINT_EMASK_EMASK6
#define EINT_EMASK_EM7              EINT_EMASK_EMASK7
#define EINT_EMASK_EM8              EINT_EMASK_EMASK8
#define EINT_EMASK_EM9              EINT_EMASK_EMASK9
#define EINT_EMASK_EM10             EINT_EMASK_EMASK10
#define EINT_EMASK_EM11             EINT_EMASK_EMASK11
#define EINT_EMASK_EM12             EINT_EMASK_EMASK12
#define EINT_EMASK_EM13             EINT_EMASK_EMASK13
#define EINT_EMASK_EM14             EINT_EMASK_EMASK14
#define EINT_EMASK_EM15             EINT_EMASK_EMASK15
#define EINT_EMASK_EM16             EINT_EMASK_EMASK16
#define EINT_EMASK_EM17             EINT_EMASK_EMASK17
/* =========================================================  RTEN  ========================================================== */
#define EINT_RTEN_RTEN0_Pos         (0U)
#define EINT_RTEN_RTEN0_Msk         (0x1UL << EINT_RTEN_RTEN0_Pos)              /*!< 0x00000001 */
#define EINT_RTEN_RTEN0             EINT_RTEN_RTEN0_Msk                         /*!< Rising trigger event configuration bit of line 0 */
#define EINT_RTEN_RTEN1_Pos         (1U)
#define EINT_RTEN_RTEN1_Msk         (0x1UL << EINT_RTEN_RTEN1_Pos)              /*!< 0x00000002 */
#define EINT_RTEN_RTEN1             EINT_RTEN_RTEN1_Msk                         /*!< Rising trigger event configuration bit of line 1 */
#define EINT_RTEN_RTEN2_Pos         (2U)
#define EINT_RTEN_RTEN2_Msk         (0x1UL << EINT_RTEN_RTEN2_Pos)              /*!< 0x00000004 */
#define EINT_RTEN_RTEN2             EINT_RTEN_RTEN2_Msk                         /*!< Rising trigger event configuration bit of line 2 */
#define EINT_RTEN_RTEN3_Pos         (3U)
#define EINT_RTEN_RTEN3_Msk         (0x1UL << EINT_RTEN_RTEN3_Pos)              /*!< 0x00000008 */
#define EINT_RTEN_RTEN3             EINT_RTEN_RTEN3_Msk                         /*!< Rising trigger event configuration bit of line 3 */
#define EINT_RTEN_RTEN4_Pos         (4U)
#define EINT_RTEN_RTEN4_Msk         (0x1UL << EINT_RTEN_RTEN4_Pos)              /*!< 0x00000010 */
#define EINT_RTEN_RTEN4             EINT_RTEN_RTEN4_Msk                         /*!< Rising trigger event configuration bit of line 4 */
#define EINT_RTEN_RTEN5_Pos         (5U)
#define EINT_RTEN_RTEN5_Msk         (0x1UL << EINT_RTEN_RTEN5_Pos)              /*!< 0x00000020 */
#define EINT_RTEN_RTEN5             EINT_RTEN_RTEN5_Msk                         /*!< Rising trigger event configuration bit of line 5 */
#define EINT_RTEN_RTEN6_Pos         (6U)
#define EINT_RTEN_RTEN6_Msk         (0x1UL << EINT_RTEN_RTEN6_Pos)              /*!< 0x00000040 */
#define EINT_RTEN_RTEN6             EINT_RTEN_RTEN6_Msk                         /*!< Rising trigger event configuration bit of line 6 */
#define EINT_RTEN_RTEN7_Pos         (7U)
#define EINT_RTEN_RTEN7_Msk         (0x1UL << EINT_RTEN_RTEN7_Pos)              /*!< 0x00000080 */
#define EINT_RTEN_RTEN7             EINT_RTEN_RTEN7_Msk                         /*!< Rising trigger event configuration bit of line 7 */
#define EINT_RTEN_RTEN8_Pos         (8U)
#define EINT_RTEN_RTEN8_Msk         (0x1UL << EINT_RTEN_RTEN8_Pos)              /*!< 0x00000100 */
#define EINT_RTEN_RTEN8             EINT_RTEN_RTEN8_Msk                         /*!< Rising trigger event configuration bit of line 8 */
#define EINT_RTEN_RTEN9_Pos         (9U)
#define EINT_RTEN_RTEN9_Msk         (0x1UL << EINT_RTEN_RTEN9_Pos)              /*!< 0x00000200 */
#define EINT_RTEN_RTEN9             EINT_RTEN_RTEN9_Msk                         /*!< Rising trigger event configuration bit of line 9 */
#define EINT_RTEN_RTEN10_Pos        (10U)
#define EINT_RTEN_RTEN10_Msk        (0x1UL << EINT_RTEN_RTEN10_Pos)             /*!< 0x00000400 */
#define EINT_RTEN_RTEN10            EINT_RTEN_RTEN10_Msk                        /*!< Rising trigger event configuration bit of line 10 */
#define EINT_RTEN_RTEN11_Pos        (11U)
#define EINT_RTEN_RTEN11_Msk        (0x1UL << EINT_RTEN_RTEN11_Pos)             /*!< 0x00000800 */
#define EINT_RTEN_RTEN11            EINT_RTEN_RTEN11_Msk                        /*!< Rising trigger event configuration bit of line 11 */
#define EINT_RTEN_RTEN12_Pos        (12U)
#define EINT_RTEN_RTEN12_Msk        (0x1UL << EINT_RTEN_RTEN12_Pos)             /*!< 0x00001000 */
#define EINT_RTEN_RTEN12            EINT_RTEN_RTEN12_Msk                        /*!< Rising trigger event configuration bit of line 12 */
#define EINT_RTEN_RTEN13_Pos        (13U)
#define EINT_RTEN_RTEN13_Msk        (0x1UL << EINT_RTEN_RTEN13_Pos)             /*!< 0x00002000 */
#define EINT_RTEN_RTEN13            EINT_RTEN_RTEN13_Msk                        /*!< Rising trigger event configuration bit of line 13 */
#define EINT_RTEN_RTEN14_Pos        (14U)
#define EINT_RTEN_RTEN14_Msk        (0x1UL << EINT_RTEN_RTEN14_Pos)             /*!< 0x00004000 */
#define EINT_RTEN_RTEN14            EINT_RTEN_RTEN14_Msk                        /*!< Rising trigger event configuration bit of line 14 */
#define EINT_RTEN_RTEN15_Pos        (15U)
#define EINT_RTEN_RTEN15_Msk        (0x1UL << EINT_RTEN_RTEN15_Pos)             /*!< 0x00008000 */
#define EINT_RTEN_RTEN15            EINT_RTEN_RTEN15_Msk                        /*!< Rising trigger event configuration bit of line 15 */
#define EINT_RTEN_RTEN16_Pos        (16U)
#define EINT_RTEN_RTEN16_Msk        (0x1UL << EINT_RTEN_RTEN16_Pos)             /*!< 0x00010000 */
#define EINT_RTEN_RTEN16            EINT_RTEN_RTEN16_Msk                        /*!< Rising trigger event configuration bit of line 16 */
#define EINT_RTEN_RTEN17_Pos        (17U)
#define EINT_RTEN_RTEN17_Msk        (0x1UL << EINT_RTEN_RTEN17_Pos)             /*!< 0x00020000 */
#define EINT_RTEN_RTEN17            EINT_RTEN_RTEN17_Msk                        /*!< Rising trigger event configuration bit of line 17 */

/* =========================================================  FTEN  ========================================================== */
#define EINT_FTEN_FTEN0_Pos         (0U)
#define EINT_FTEN_FTEN0_Msk         (0x1UL << EINT_FTEN_FTEN0_Pos)              /*!< 0x00000001 */
#define EINT_FTEN_FTEN0             EINT_FTEN_FTEN0_Msk                         /*!< Falling trigger event configuration bit of line 0 */
#define EINT_FTEN_FTEN1_Pos         (1U)
#define EINT_FTEN_FTEN1_Msk         (0x1UL << EINT_FTEN_FTEN1_Pos)              /*!< 0x00000002 */
#define EINT_FTEN_FTEN1             EINT_FTEN_FTEN1_Msk                         /*!< Falling trigger event configuration bit of line 1 */
#define EINT_FTEN_FTEN2_Pos         (2U)
#define EINT_FTEN_FTEN2_Msk         (0x1UL << EINT_FTEN_FTEN2_Pos)              /*!< 0x00000004 */
#define EINT_FTEN_FTEN2             EINT_FTEN_FTEN2_Msk                         /*!< Falling trigger event configuration bit of line 2 */
#define EINT_FTEN_FTEN3_Pos         (3U)
#define EINT_FTEN_FTEN3_Msk         (0x1UL << EINT_FTEN_FTEN3_Pos)              /*!< 0x00000008 */
#define EINT_FTEN_FTEN3             EINT_FTEN_FTEN3_Msk                         /*!< Falling trigger event configuration bit of line 3 */
#define EINT_FTEN_FTEN4_Pos         (4U)
#define EINT_FTEN_FTEN4_Msk         (0x1UL << EINT_FTEN_FTEN4_Pos)              /*!< 0x00000010 */
#define EINT_FTEN_FTEN4             EINT_FTEN_FTEN4_Msk                         /*!< Falling trigger event configuration bit of line 4 */
#define EINT_FTEN_FTEN5_Pos         (5U)
#define EINT_FTEN_FTEN5_Msk         (0x1UL << EINT_FTEN_FTEN5_Pos)              /*!< 0x00000020 */
#define EINT_FTEN_FTEN5             EINT_FTEN_FTEN5_Msk                         /*!< Falling trigger event configuration bit of line 5 */
#define EINT_FTEN_FTEN6_Pos         (6U)
#define EINT_FTEN_FTEN6_Msk         (0x1UL << EINT_FTEN_FTEN6_Pos)              /*!< 0x00000040 */
#define EINT_FTEN_FTEN6             EINT_FTEN_FTEN6_Msk                         /*!< Falling trigger event configuration bit of line 6 */
#define EINT_FTEN_FTEN7_Pos         (7U)
#define EINT_FTEN_FTEN7_Msk         (0x1UL << EINT_FTEN_FTEN7_Pos)              /*!< 0x00000080 */
#define EINT_FTEN_FTEN7             EINT_FTEN_FTEN7_Msk                         /*!< Falling trigger event configuration bit of line 7 */
#define EINT_FTEN_FTEN8_Pos         (8U)
#define EINT_FTEN_FTEN8_Msk         (0x1UL << EINT_FTEN_FTEN8_Pos)              /*!< 0x00000100 */
#define EINT_FTEN_FTEN8             EINT_FTEN_FTEN8_Msk                         /*!< Falling trigger event configuration bit of line 8 */
#define EINT_FTEN_FTEN9_Pos         (9U)
#define EINT_FTEN_FTEN9_Msk         (0x1UL << EINT_FTEN_FTEN9_Pos)              /*!< 0x00000200 */
#define EINT_FTEN_FTEN9             EINT_FTEN_FTEN9_Msk                         /*!< Falling trigger event configuration bit of line 9 */
#define EINT_FTEN_FTEN10_Pos        (10U)
#define EINT_FTEN_FTEN10_Msk        (0x1UL << EINT_FTEN_FTEN10_Pos)             /*!< 0x00000400 */
#define EINT_FTEN_FTEN10            EINT_FTEN_FTEN10_Msk                        /*!< Falling trigger event configuration bit of line 10 */
#define EINT_FTEN_FTEN11_Pos        (11U)
#define EINT_FTEN_FTEN11_Msk        (0x1UL << EINT_FTEN_FTEN11_Pos)             /*!< 0x00000800 */
#define EINT_FTEN_FTEN11            EINT_FTEN_FTEN11_Msk                        /*!< Falling trigger event configuration bit of line 11 */
#define EINT_FTEN_FTEN12_Pos        (12U)
#define EINT_FTEN_FTEN12_Msk        (0x1UL << EINT_FTEN_FTEN12_Pos)             /*!< 0x00001000 */
#define EINT_FTEN_FTEN12            EINT_FTEN_FTEN12_Msk                        /*!< Falling trigger event configuration bit of line 12 */
#define EINT_FTEN_FTEN13_Pos        (13U)
#define EINT_FTEN_FTEN13_Msk        (0x1UL << EINT_FTEN_FTEN13_Pos)             /*!< 0x00002000 */
#define EINT_FTEN_FTEN13            EINT_FTEN_FTEN13_Msk                        /*!< Falling trigger event configuration bit of line 13 */
#define EINT_FTEN_FTEN14_Pos        (14U)
#define EINT_FTEN_FTEN14_Msk        (0x1UL << EINT_FTEN_FTEN14_Pos)             /*!< 0x00004000 */
#define EINT_FTEN_FTEN14            EINT_FTEN_FTEN14_Msk                        /*!< Falling trigger event configuration bit of line 14 */
#define EINT_FTEN_FTEN15_Pos        (15U)
#define EINT_FTEN_FTEN15_Msk        (0x1UL << EINT_FTEN_FTEN15_Pos)             /*!< 0x00008000 */
#define EINT_FTEN_FTEN15            EINT_FTEN_FTEN15_Msk                        /*!< Falling trigger event configuration bit of line 15 */
#define EINT_FTEN_FTEN16_Pos        (16U)
#define EINT_FTEN_FTEN16_Msk        (0x1UL << EINT_FTEN_FTEN16_Pos)             /*!< 0x00010000 */
#define EINT_FTEN_FTEN16            EINT_FTEN_FTEN16_Msk                        /*!< Falling trigger event configuration bit of line 16 */
#define EINT_FTEN_FTEN17_Pos        (17U)
#define EINT_FTEN_FTEN17_Msk        (0x1UL << EINT_FTEN_FTEN17_Pos)             /*!< 0x00020000 */
#define EINT_FTEN_FTEN17            EINT_FTEN_FTEN17_Msk                        /*!< Falling trigger event configuration bit of line 17 */

/* ========================================================  SWINTE  ========================================================= */
#define EINT_SWINTE_SWINTE0_Pos     (0U)
#define EINT_SWINTE_SWINTE0_Msk     (0x1UL << EINT_SWINTE_SWINTE0_Pos)          /*!< 0x00000001 */
#define EINT_SWINTE_SWINTE0         EINT_SWINTE_SWINTE0_Msk                     /*!< Software Interrupt on line 0 */
#define EINT_SWINTE_SWINTE1_Pos     (1U)
#define EINT_SWINTE_SWINTE1_Msk     (0x1UL << EINT_SWINTE_SWINTE1_Pos)          /*!< 0x00000002 */
#define EINT_SWINTE_SWINTE1         EINT_SWINTE_SWINTE1_Msk                     /*!< Software Interrupt on line 1 */
#define EINT_SWINTE_SWINTE2_Pos     (2U)
#define EINT_SWINTE_SWINTE2_Msk     (0x1UL << EINT_SWINTE_SWINTE2_Pos)          /*!< 0x00000004 */
#define EINT_SWINTE_SWINTE2         EINT_SWINTE_SWINTE2_Msk                     /*!< Software Interrupt on line 2 */
#define EINT_SWINTE_SWINTE3_Pos     (3U)
#define EINT_SWINTE_SWINTE3_Msk     (0x1UL << EINT_SWINTE_SWINTE3_Pos)          /*!< 0x00000008 */
#define EINT_SWINTE_SWINTE3         EINT_SWINTE_SWINTE3_Msk                     /*!< Software Interrupt on line 3 */
#define EINT_SWINTE_SWINTE4_Pos     (4U)
#define EINT_SWINTE_SWINTE4_Msk     (0x1UL << EINT_SWINTE_SWINTE4_Pos)          /*!< 0x00000010 */
#define EINT_SWINTE_SWINTE4         EINT_SWINTE_SWINTE4_Msk                     /*!< Software Interrupt on line 4 */
#define EINT_SWINTE_SWINTE5_Pos     (5U)
#define EINT_SWINTE_SWINTE5_Msk     (0x1UL << EINT_SWINTE_SWINTE5_Pos)          /*!< 0x00000020 */
#define EINT_SWINTE_SWINTE5         EINT_SWINTE_SWINTE5_Msk                     /*!< Software Interrupt on line 5 */
#define EINT_SWINTE_SWINTE6_Pos     (6U)
#define EINT_SWINTE_SWINTE6_Msk     (0x1UL << EINT_SWINTE_SWINTE6_Pos)          /*!< 0x00000040 */
#define EINT_SWINTE_SWINTE6         EINT_SWINTE_SWINTE6_Msk                     /*!< Software Interrupt on line 6 */
#define EINT_SWINTE_SWINTE7_Pos     (7U)
#define EINT_SWINTE_SWINTE7_Msk     (0x1UL << EINT_SWINTE_SWINTE7_Pos)          /*!< 0x00000080 */
#define EINT_SWINTE_SWINTE7         EINT_SWINTE_SWINTE7_Msk                     /*!< Software Interrupt on line 7 */
#define EINT_SWINTE_SWINTE8_Pos     (8U)
#define EINT_SWINTE_SWINTE8_Msk     (0x1UL << EINT_SWINTE_SWINTE8_Pos)          /*!< 0x00000100 */
#define EINT_SWINTE_SWINTE8         EINT_SWINTE_SWINTE8_Msk                     /*!< Software Interrupt on line 8 */
#define EINT_SWINTE_SWINTE9_Pos     (9U)
#define EINT_SWINTE_SWINTE9_Msk     (0x1UL << EINT_SWINTE_SWINTE9_Pos)          /*!< 0x00000200 */
#define EINT_SWINTE_SWINTE9         EINT_SWINTE_SWINTE9_Msk                     /*!< Software Interrupt on line 9 */
#define EINT_SWINTE_SWINTE10_Pos    (10U)
#define EINT_SWINTE_SWINTE10_Msk    (0x1UL << EINT_SWINTE_SWINTE10_Pos)         /*!< 0x00000400 */
#define EINT_SWINTE_SWINTE10        EINT_SWINTE_SWINTE10_Msk                    /*!< Software Interrupt on line 10 */
#define EINT_SWINTE_SWINTE11_Pos    (11U)
#define EINT_SWINTE_SWINTE11_Msk    (0x1UL << EINT_SWINTE_SWINTE11_Pos)         /*!< 0x00000800 */
#define EINT_SWINTE_SWINTE11        EINT_SWINTE_SWINTE11_Msk                    /*!< Software Interrupt on line 11 */
#define EINT_SWINTE_SWINTE12_Pos    (12U)
#define EINT_SWINTE_SWINTE12_Msk    (0x1UL << EINT_SWINTE_SWINTE12_Pos)         /*!< 0x00001000 */
#define EINT_SWINTE_SWINTE12        EINT_SWINTE_SWINTE12_Msk                    /*!< Software Interrupt on line 12 */
#define EINT_SWINTE_SWINTE13_Pos    (13U)
#define EINT_SWINTE_SWINTE13_Msk    (0x1UL << EINT_SWINTE_SWINTE13_Pos)         /*!< 0x00002000 */
#define EINT_SWINTE_SWINTE13        EINT_SWINTE_SWINTE13_Msk                    /*!< Software Interrupt on line 13 */
#define EINT_SWINTE_SWINTE14_Pos    (14U)
#define EINT_SWINTE_SWINTE14_Msk    (0x1UL << EINT_SWINTE_SWINTE14_Pos)         /*!< 0x00004000 */
#define EINT_SWINTE_SWINTE14        EINT_SWINTE_SWINTE14_Msk                    /*!< Software Interrupt on line 14 */
#define EINT_SWINTE_SWINTE15_Pos    (15U)
#define EINT_SWINTE_SWINTE15_Msk    (0x1UL << EINT_SWINTE_SWINTE15_Pos)         /*!< 0x00008000 */
#define EINT_SWINTE_SWINTE15        EINT_SWINTE_SWINTE15_Msk                    /*!< Software Interrupt on line 15 */
#define EINT_SWINTE_SWINTE16_Pos    (16U)
#define EINT_SWINTE_SWINTE16_Msk    (0x1UL << EINT_SWINTE_SWINTE16_Pos)         /*!< 0x00010000 */
#define EINT_SWINTE_SWINTE16        EINT_SWINTE_SWINTE16_Msk                    /*!< Software Interrupt on line 16 */
#define EINT_SWINTE_SWINTE17_Pos    (17U)
#define EINT_SWINTE_SWINTE17_Msk    (0x1UL << EINT_SWINTE_SWINTE17_Pos)         /*!< 0x00020000 */
#define EINT_SWINTE_SWINTE17        EINT_SWINTE_SWINTE17_Msk                    /*!< Software Interrupt on line 17 */

/* =========================================================  IPEND  ========================================================= */
#define EINT_IPEND_IPEND0_Pos       (0U)
#define EINT_IPEND_IPEND0_Msk       (0x1UL << EINT_IPEND_IPEND0_Pos)            /*!< 0x00000001 */
#define EINT_IPEND_IPEND0           EINT_IPEND_IPEND0_Msk                       /*!< Pending bit for line 0 */
#define EINT_IPEND_IPEND1_Pos       (1U)
#define EINT_IPEND_IPEND1_Msk       (0x1UL << EINT_IPEND_IPEND1_Pos)            /*!< 0x00000002 */
#define EINT_IPEND_IPEND1           EINT_IPEND_IPEND1_Msk                       /*!< Pending bit for line 1 */
#define EINT_IPEND_IPEND2_Pos       (2U)
#define EINT_IPEND_IPEND2_Msk       (0x1UL << EINT_IPEND_IPEND2_Pos)            /*!< 0x00000004 */
#define EINT_IPEND_IPEND2           EINT_IPEND_IPEND2_Msk                       /*!< Pending bit for line 2 */
#define EINT_IPEND_IPEND3_Pos       (3U)
#define EINT_IPEND_IPEND3_Msk       (0x1UL << EINT_IPEND_IPEND3_Pos)            /*!< 0x00000008 */
#define EINT_IPEND_IPEND3           EINT_IPEND_IPEND3_Msk                       /*!< Pending bit for line 3 */
#define EINT_IPEND_IPEND4_Pos       (4U)
#define EINT_IPEND_IPEND4_Msk       (0x1UL << EINT_IPEND_IPEND4_Pos)            /*!< 0x00000010 */
#define EINT_IPEND_IPEND4           EINT_IPEND_IPEND4_Msk                       /*!< Pending bit for line 4 */
#define EINT_IPEND_IPEND5_Pos       (5U)
#define EINT_IPEND_IPEND5_Msk       (0x1UL << EINT_IPEND_IPEND5_Pos)            /*!< 0x00000020 */
#define EINT_IPEND_IPEND5           EINT_IPEND_IPEND5_Msk                       /*!< Pending bit for line 5 */
#define EINT_IPEND_IPEND6_Pos       (6U)
#define EINT_IPEND_IPEND6_Msk       (0x1UL << EINT_IPEND_IPEND6_Pos)            /*!< 0x00000040 */
#define EINT_IPEND_IPEND6           EINT_IPEND_IPEND6_Msk                       /*!< Pending bit for line 6 */
#define EINT_IPEND_IPEND7_Pos       (7U)
#define EINT_IPEND_IPEND7_Msk       (0x1UL << EINT_IPEND_IPEND7_Pos)            /*!< 0x00000080 */
#define EINT_IPEND_IPEND7           EINT_IPEND_IPEND7_Msk                       /*!< Pending bit for line 7 */
#define EINT_IPEND_IPEND8_Pos       (8U)
#define EINT_IPEND_IPEND8_Msk       (0x1UL << EINT_IPEND_IPEND8_Pos)            /*!< 0x00000100 */
#define EINT_IPEND_IPEND8           EINT_IPEND_IPEND8_Msk                       /*!< Pending bit for line 8 */
#define EINT_IPEND_IPEND9_Pos       (9U)
#define EINT_IPEND_IPEND9_Msk       (0x1UL << EINT_IPEND_IPEND9_Pos)            /*!< 0x00000200 */
#define EINT_IPEND_IPEND9           EINT_IPEND_IPEND9_Msk                       /*!< Pending bit for line 9 */
#define EINT_IPEND_IPEND10_Pos      (10U)
#define EINT_IPEND_IPEND10_Msk      (0x1UL << EINT_IPEND_IPEND10_Pos)           /*!< 0x00000400 */
#define EINT_IPEND_IPEND10          EINT_IPEND_IPEND10_Msk                      /*!< Pending bit for line 10 */
#define EINT_IPEND_IPEND11_Pos      (11U)
#define EINT_IPEND_IPEND11_Msk      (0x1UL << EINT_IPEND_IPEND11_Pos)           /*!< 0x00000800 */
#define EINT_IPEND_IPEND11          EINT_IPEND_IPEND11_Msk                      /*!< Pending bit for line 11 */
#define EINT_IPEND_IPEND12_Pos      (12U)
#define EINT_IPEND_IPEND12_Msk      (0x1UL << EINT_IPEND_IPEND12_Pos)           /*!< 0x00001000 */
#define EINT_IPEND_IPEND12          EINT_IPEND_IPEND12_Msk                      /*!< Pending bit for line 12 */
#define EINT_IPEND_IPEND13_Pos      (13U)
#define EINT_IPEND_IPEND13_Msk      (0x1UL << EINT_IPEND_IPEND13_Pos)           /*!< 0x00002000 */
#define EINT_IPEND_IPEND13          EINT_IPEND_IPEND13_Msk                      /*!< Pending bit for line 13 */
#define EINT_IPEND_IPEND14_Pos      (14U)
#define EINT_IPEND_IPEND14_Msk      (0x1UL << EINT_IPEND_IPEND14_Pos)           /*!< 0x00004000 */
#define EINT_IPEND_IPEND14          EINT_IPEND_IPEND14_Msk                      /*!< Pending bit for line 14 */
#define EINT_IPEND_IPEND15_Pos      (15U)
#define EINT_IPEND_IPEND15_Msk      (0x1UL << EINT_IPEND_IPEND15_Pos)           /*!< 0x00008000 */
#define EINT_IPEND_IPEND15          EINT_IPEND_IPEND15_Msk                      /*!< Pending bit for line 15 */
#define EINT_IPEND_IPEND16_Pos      (16U)
#define EINT_IPEND_IPEND16_Msk      (0x1UL << EINT_IPEND_IPEND16_Pos)           /*!< 0x00010000 */
#define EINT_IPEND_IPEND16          EINT_IPEND_IPEND16_Msk                      /*!< Pending bit for line 16 */
#define EINT_IPEND_IPEND17_Pos      (17U)
#define EINT_IPEND_IPEND17_Msk      (0x1UL << EINT_IPEND_IPEND17_Pos)           /*!< 0x00020000 */
#define EINT_IPEND_IPEND17          EINT_IPEND_IPEND17_Msk                      /*!< Pending bit for line 17 */

/* =========================================================  CFG0  ========================================================== */
#define EINT_CFG0_EINT0_Pos         (0U)                                        /*!< CFG_EINT0 (Bit 0)                                     */
#define EINT_CFG0_EINT0_Msk         (0x1UL << EINT_CFG0_EINT0_Pos)              /*!< CFG_EINT0 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT0             EINT_CFG0_EINT0_Msk
#define EINT_CFG0_EINT1_Pos         (4U)                                        /*!< CFG_EINT1 (Bit 4)                                     */
#define EINT_CFG0_EINT1_Msk         (0x1UL << EINT_CFG0_EINT1_Pos)              /*!< CFG_EINT1 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT1             EINT_CFG0_EINT1_Msk
#define EINT_CFG0_EINT2_Pos         (8U)                                        /*!< CFG_EINT2 (Bit 8)                                     */
#define EINT_CFG0_EINT2_Msk         (0x1UL << EINT_CFG0_EINT2_Pos)              /*!< CFG_EINT2 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT2             EINT_CFG0_EINT2_Msk
#define EINT_CFG0_EINT3_Pos         (12U)                                       /*!< CFG_EINT3 (Bit 12)                                    */
#define EINT_CFG0_EINT3_Msk         (0x1UL << EINT_CFG0_EINT3_Pos)              /*!< CFG_EINT3 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT3             EINT_CFG0_EINT3_Msk
#define EINT_CFG0_EINT4_Pos         (16U)                                       /*!< CFG_EINT4 (Bit 16)                                    */
#define EINT_CFG0_EINT4_Msk         (0x1UL << EINT_CFG0_EINT4_Pos)              /*!< CFG_EINT4 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT4             EINT_CFG0_EINT4_Msk
#define EINT_CFG0_EINT5_Pos         (20U)                                       /*!< CFG_EINT5 (Bit 20)                                    */
#define EINT_CFG0_EINT5_Msk         (0x1UL << EINT_CFG0_EINT5_Pos)              /*!< CFG_EINT5 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT5             EINT_CFG0_EINT5_Msk
#define EINT_CFG0_EINT6_Pos         (24U)                                       /*!< CFG_EINT6 (Bit 24)                                    */
#define EINT_CFG0_EINT6_Msk         (0x1UL << EINT_CFG0_EINT6_Pos)              /*!< CFG_EINT6 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT6             EINT_CFG0_EINT6_Msk
#define EINT_CFG0_EINT7_Pos         (28U)                                       /*!< CFG_EINT7 (Bit 28)                                    */
#define EINT_CFG0_EINT7_Msk         (0x1UL << EINT_CFG0_EINT7_Pos)              /*!< CFG_EINT7 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG0_EINT7             EINT_CFG0_EINT7_Msk
/* =========================================================  CFG1  ========================================================== */
#define EINT_CFG1_EINT8_Pos         (0U)                                        /*!< CFG_EINT8 (Bit 0)                                     */
#define EINT_CFG1_EINT8_Msk         (0x1UL << EINT_CFG1_EINT8_Pos)              /*!< CFG_EINT8 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG1_EINT8             EINT_CFG1_EINT8_Msk
#define EINT_CFG1_EINT9_Pos         (4U)                                        /*!< CFG_EINT9 (Bit 4)                                     */
#define EINT_CFG1_EINT9_Msk         (0x1UL << EINT_CFG1_EINT9_Pos)              /*!< CFG_EINT9 (Bitfield-Mask: 0x0f)                       */
#define EINT_CFG1_EINT9             EINT_CFG1_EINT9_Msk
#define EINT_CFG1_EINT10_Pos        (8U)                                        /*!< CFG_EINT10 (Bit 8)                                    */
#define EINT_CFG1_EINT10_Msk        (0x1UL << EINT_CFG1_EINT10_Pos)             /*!< CFG_EINT10 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT10            EINT_CFG1_EINT10_Msk
#define EINT_CFG1_EINT11_Pos        (12U)                                       /*!< CFG_EINT11 (Bit 12)                                   */
#define EINT_CFG1_EINT11_Msk        (0x1UL << EINT_CFG1_EINT11_Pos)             /*!< CFG_EINT11 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT11            EINT_CFG1_EINT11_Msk
#define EINT_CFG1_EINT12_Pos        (16U)                                       /*!< CFG_EINT12 (Bit 16)                                   */
#define EINT_CFG1_EINT12_Msk        (0x1UL << EINT_CFG1_EINT12_Pos)             /*!< CFG_EINT12 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT12            EINT_CFG1_EINT12_Msk
#define EINT_CFG1_EINT13_Pos        (20U)                                       /*!< CFG_EINT13 (Bit 20)                                   */
#define EINT_CFG1_EINT13_Msk        (0x1UL << EINT_CFG1_EINT13_Pos)             /*!< CFG_EINT13 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT13            EINT_CFG1_EINT13_Msk
#define EINT_CFG1_EINT14_Pos        (24U)                                       /*!< CFG_EINT14 (Bit 24)                                   */
#define EINT_CFG1_EINT14_Msk        (0x1UL << EINT_CFG1_EINT14_Pos)             /*!< CFG_EINT14 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT14            EINT_CFG1_EINT14_Msk
#define EINT_CFG1_EINT15_Pos        (28U)                                       /*!< CFG_EINT15 (Bit 28)                                   */
#define EINT_CFG1_EINT15_Msk        (0x1UL << EINT_CFG1_EINT15_Pos)             /*!< CFG_EINT15 (Bitfield-Mask: 0x0f)                      */
#define EINT_CFG1_EINT15            EINT_CFG1_EINT15_Msk


/* =========================================================================================================================== */
/* ================                                          ATMR                                             ================ */
/* =========================================================================================================================== */

/* =========================================================  CTRL1  ========================================================= */
#define ATMR_CR1_CNTEN_Pos          (0U)                                        /*!< CNTEN (Bit 0)                             */
#define ATMR_CR1_CNTEN_Msk          (0x1UL << ATMR_CR1_CNTEN_Pos)               /*!< CNTEN (Bitfield-Mask: 0x01)               */
#define ATMR_CR1_CNTEN              ATMR_CR1_CNTEN_Msk
#define ATMR_CR1_UD_Pos             (1U)                                        /*!< UD (Bit 1)                                */
#define ATMR_CR1_UD_Msk             (0x1UL << ATMR_CR1_UD_Pos)                  /*!< UD (Bitfield-Mask: 0x01)                  */
#define ATMR_CR1_UD                 ATMR_CR1_UD_Msk
#define ATMR_CR1_UDISEN_Pos         (2U)                                        /*!< URSSEL (Bit 2)                            */
#define ATMR_CR1_UDISEN_Msk         (0x1UL << ATMR_CR1_UDISEN_Pos)              /*!< URSSEL (Bitfield-Mask: 0x01)              */
#define ATMR_CR1_UDISEN             ATMR_CR1_UDISEN_Msk
#define ATMR_CR1_SPMEN_Pos          (3U)                                        /*!< SPMEN (Bit 3)                             */
#define ATMR_CR1_SPMEN_Msk          (0x1UL << ATMR_CR1_SPMEN_Pos)               /*!< SPMEN (Bitfield-Mask: 0x01)               */
#define ATMR_CR1_SPMEN              ATMR_CR1_SPMEN_Msk
#define ATMR_CR1_CNTDIR_Pos         (4U)                                        /*!< CNTDIR (Bit 4)                            */
#define ATMR_CR1_CNTDIR_Msk         (0x1UL << ATMR_CR1_CNTDIR_Pos)              /*!< CNTDIR (Bitfield-Mask: 0x01)              */
#define ATMR_CR1_CNTDIR             ATMR_CR1_CNTDIR_Msk
#define ATMR_CR1_CAMSEL_Pos         (5U)                                        /*!< CAMSEL (Bit 5)                            */
#define ATMR_CR1_CAMSEL_Msk         (0x3UL << ATMR_CR1_CAMSEL_Pos)              /*!< CAMSEL (Bitfield-Mask: 0x03)              */
#define ATMR_CR1_CAMSEL             ATMR_CR1_CAMSEL_Msk
#define ATMR_CR1_CAMSEL_0           (0x1UL << ATMR_CR1_CAMSEL_Pos)
#define ATMR_CR1_CAMSEL_1           (0x2UL << ATMR_CR1_CAMSEL_Pos)

#define ATMR_CR1_ARPEN_Pos          (7U)                                        /*!< ARPEN (Bit 7)                             */
#define ATMR_CR1_ARPEN_Msk          (0x1UL << ATMR_CR1_ARPEN_Pos)               /*!< ARPEN (Bitfield-Mask: 0x01)               */
#define ATMR_CR1_ARPEN              ATMR_CR1_ARPEN_Msk
#define ATMR_CR1_CLKDIV_Pos         (8U)                                        /*!< CLKDIV (Bit 8)                            */
#define ATMR_CR1_CLKDIV_Msk         (0x3UL << ATMR_CR1_CLKDIV_Pos)              /*!< CLKDIV (Bitfield-Mask: 0x03)              */
#define ATMR_CR1_CLKDIV             ATMR_CR1_CLKDIV_Msk
#define ATMR_CR1_CLKDIV_0           (0x1UL << ATMR_CR1_CLKDIV_Pos)
#define ATMR_CR1_CLKDIV_1           (0x2UL << ATMR_CR1_CLKDIV_Pos)

#define ATMR_CR1_BCS_Pos            (10U)                                       /*!< BCS (Bit 10)                              */
#define ATMR_CR1_BCS_Msk            (0x7UL << ATMR_CR1_BCS_Pos)                 /*!< BCS (Bitfield-Mask: 0x07)                 */
#define ATMR_CR1_BCS                ATMR_CR1_BCS_Msk
#define ATMR_CR1_BCS_0              (0x1UL << ATMR_CR1_BCS_Pos)
#define ATMR_CR1_BCS_1              (0x2UL << ATMR_CR1_BCS_Pos)
#define ATMR_CR1_BCS_2              (0x4UL << ATMR_CR1_BCS_Pos)
/* =========================================================  CTRL2  ========================================================= */
#define ATMR_CR2_CCPEN_Pos          (0U)                                        /*!< CCPEN (Bit 0)                             */
#define ATMR_CR2_CCPEN_Msk          (0x1UL << ATMR_CR2_CCPEN_Pos)               /*!< CCPEN (Bitfield-Mask: 0x01)               */
#define ATMR_CR2_CCPEN              ATMR_CR2_CCPEN_Msk
#define ATMR_CR2_MMSZE_Pos          (1U)                                        /*!< MMSZE (Bit 1)                             */
#define ATMR_CR2_MMSZE_Msk          (0x1UL << ATMR_CR2_MMSZE_Pos)               /*!< MMSZE (Bitfield-Mask: 0x01)               */
#define ATMR_CR2_MMSZE              ATMR_CR2_MMSZE_Msk
#define ATMR_CR2_CCUSEL_Pos         (2U)                                        /*!< CCUSEL (Bit 2)                            */
#define ATMR_CR2_CCUSEL_Msk         (0x1UL << ATMR_CR2_CCUSEL_Pos)              /*!< CCUSEL (Bitfield-Mask: 0x01)              */
#define ATMR_CR2_CCUSEL             ATMR_CR2_CCUSEL_Msk
#define ATMR_CR2_MMSPE_Pos          (3U)                                        /*!< MMSPE (Bit 3)                             */
#define ATMR_CR2_MMSPE_Msk          (0x1UL << ATMR_CR2_MMSPE_Pos)               /*!< MMSPE (Bitfield-Mask: 0x01)               */
#define ATMR_CR2_MMSPE              ATMR_CR2_MMSPE_Msk
#define ATMR_CR2_MMSEL_Pos          (4U)                                        /*!< MMSEL (Bit 4)                             */
#define ATMR_CR2_MMSEL_Msk          (0xFUL << ATMR_CR2_MMSEL_Pos)               /*!< MMSEL (Bitfield-Mask: 0x0f)               */
#define ATMR_CR2_MMSEL              ATMR_CR2_MMSEL_Msk
#define ATMR_CR2_MMSEL_0            (0x1UL << ATMR_CR2_MMSEL_Pos)
#define ATMR_CR2_MMSEL_1            (0x2UL << ATMR_CR2_MMSEL_Pos)
#define ATMR_CR2_MMSEL_2            (0x4UL << ATMR_CR2_MMSEL_Pos)
#define ATMR_CR2_MMSEL_3            (0x8UL << ATMR_CR2_MMSEL_Pos)

#define ATMR_CR2_OC0OIS_Pos         (8U)                                        /*!< OC0OIS (Bit 8)                            */
#define ATMR_CR2_OC0OIS_Msk         (0x1UL << ATMR_CR2_OC0OIS_Pos)              /*!< OC0OIS (Bitfield-Mask: 0x01)              */
#define ATMR_CR2_OC0OIS             ATMR_CR2_OC0OIS_Msk
#define ATMR_CR2_OC0NOIS_Pos        (9U)                                        /*!< OC0NOIS (Bit 9)                           */
#define ATMR_CR2_OC0NOIS_Msk        (0x1UL << ATMR_CR2_OC0NOIS_Pos)             /*!< OC0NOIS (Bitfield-Mask: 0x01)             */
#define ATMR_CR2_OC0NOIS            ATMR_CR2_OC0NOIS_Msk
#define ATMR_CR2_OC1OIS_Pos         (10U)                                       /*!< OC1OIS (Bit 10)                           */
#define ATMR_CR2_OC1OIS_Msk         (0x1UL << ATMR_CR2_OC1OIS_Pos)              /*!< OC1OIS (Bitfield-Mask: 0x01)              */
#define ATMR_CR2_OC1OIS             ATMR_CR2_OC1OIS_Msk
#define ATMR_CR2_OC1NOIS_Pos        (11U)                                       /*!< OC1NOIS (Bit 11)                          */
#define ATMR_CR2_OC1NOIS_Msk        (0x1UL << ATMR_CR2_OC1NOIS_Pos)           /*!< OC1NOIS (Bitfield-Mask: 0x01)             */
#define ATMR_CR2_OC1NOIS            ATMR_CR2_OC1NOIS_Msk
#define ATMR_CR2_OC2OIS_Pos         (12U)                                       /*!< OC2OIS (Bit 12)                           */
#define ATMR_CR2_OC2OIS_Msk         (0x1UL << ATMR_CR2_OC2OIS_Pos)              /*!< OC2OIS (Bitfield-Mask: 0x01)              */
#define ATMR_CR2_OC2OIS             ATMR_CR2_OC2OIS_Msk
#define ATMR_CR2_OC2NOIS_Pos        (13U)                                       /*!< OC2NOIS (Bit 13)                          */
#define ATMR_CR2_OC2NOIS_Msk        (0x1UL << ATMR_CR2_OC2NOIS_Pos)             /*!< OC2NOIS (Bitfield-Mask: 0x01)             */
#define ATMR_CR2_OC2NOIS            ATMR_CR2_OC2NOIS_Msk
#define ATMR_CR2_OC3OIS_Pos         (14U)                                       /*!< OC3OIS (Bit 14)                           */
#define ATMR_CR2_OC3OIS_Msk         (0x1UL << ATMR_CR2_OC3OIS_Pos)              /*!< OC3OIS (Bitfield-Mask: 0x01)              */
#define ATMR_CR2_OC3OIS             ATMR_CR2_OC3OIS_Msk
#define ATMR_CR2_OC3NOIS_Pos        (15U)                                       /*!< OC3NOIS (Bit 15)                          */
#define ATMR_CR2_OC3NOIS_Msk        (0x1UL << ATMR_CR2_OC3NOIS_Pos)             /*!< OC3NOIS (Bitfield-Mask: 0x01)             */
#define ATMR_CR2_OC3NOIS            ATMR_CR2_OC3NOIS_Msk
/* ========================================================  SMCR  ========================================================= */
#define ATMR_SMCR_SMFSEL_Pos        (0U)                                        /*!< SMFSEL (Bit 0)                            */
#define ATMR_SMCR_SMFSEL_Msk        (0x7UL << ATMR_SMCR_SMFSEL_Pos)             /*!< SMFSEL (Bitfield-Mask: 0x07)              */
#define ATMR_SMCR_SMFSEL            ATMR_SMCR_SMFSEL_Msk
#define ATMR_SMCR_SMFSEL_0          (0x1UL << ATMR_SMCR_SMFSEL_Pos)
#define ATMR_SMCR_SMFSEL_1          (0x2UL << ATMR_SMCR_SMFSEL_Pos)
#define ATMR_SMCR_SMFSEL_2          (0x4UL << ATMR_SMCR_SMFSEL_Pos)

#define ATMR_SMCR_TRGSEL_Pos        (4U)                                        /*!< TRGSEL (Bit 4)                            */
#define ATMR_SMCR_TRGSEL_Msk        (0x7UL << ATMR_SMCR_TRGSEL_Pos)             /*!< TRGSEL (Bitfield-Mask: 0x07)              */
#define ATMR_SMCR_TRGSEL            ATMR_SMCR_TRGSEL_Msk
#define ATMR_SMCR_TRGSEL_0          (0x1UL << ATMR_SMCR_TRGSEL_Pos)
#define ATMR_SMCR_TRGSEL_1          (0x2UL << ATMR_SMCR_TRGSEL_Pos)
#define ATMR_SMCR_TRGSEL_2          (0x4UL << ATMR_SMCR_TRGSEL_Pos)

#define ATMR_SMCR_MSMEN_Pos         (7U)                                        /*!< MSMEN (Bit 7)                             */
#define ATMR_SMCR_MSMEN_Msk         (0x1UL << ATMR_SMCR_MSMEN_Pos)              /*!< MSMEN (Bitfield-Mask: 0x01)               */
#define ATMR_SMCR_MSMEN             ATMR_SMCR_MSMEN_Msk
#define ATMR_SMCR_ETFCFG_Pos        (8U)                                        /*!< ETFCFG (Bit 8)                            */
#define ATMR_SMCR_ETFCFG_Msk        (0xFUL << ATMR_SMCR_ETFCFG_Pos)             /*!< ETFCFG (Bitfield-Mask: 0x0f)              */
#define ATMR_SMCR_ETFCFG            ATMR_SMCR_ETFCFG_Msk
#define ATMR_SMCR_ETFCFG_0          (0x1UL << ATMR_SMCR_ETFCFG_Pos)
#define ATMR_SMCR_ETFCFG_1          (0x2UL << ATMR_SMCR_ETFCFG_Pos)
#define ATMR_SMCR_ETFCFG_2          (0x4UL << ATMR_SMCR_ETFCFG_Pos)
#define ATMR_SMCR_ETFCFG_3          (0x8UL << ATMR_SMCR_ETFCFG_Pos)

#define ATMR_SMCR_ETPCFG_Pos        (12U)                                       /*!< ETPCFG (Bit 12)                           */
#define ATMR_SMCR_ETPCFG_Msk        (0x3UL << ATMR_SMCR_ETPCFG_Pos)             /*!< ETPCFG (Bitfield-Mask: 0x03)              */
#define ATMR_SMCR_ETPCFG            ATMR_SMCR_ETPCFG_Msk
#define ATMR_SMCR_ETPCFG_0          (0x1UL << ATMR_SMCR_ETPCFG_Pos)
#define ATMR_SMCR_ETPCFG_1          (0x2UL << ATMR_SMCR_ETPCFG_Pos)

#define ATMR_SMCR_ETPOL_Pos         (15U)                                       /*!< ETPOL (Bit 15)                            */
#define ATMR_SMCR_ETPOL_Msk         (0x1UL << ATMR_SMCR_ETPOL_Pos)              /*!< ETPOL (Bitfield-Mask: 0x01)               */
#define ATMR_SMCR_ETPOL             ATMR_SMCR_ETPOL_Msk
/* =========================================================  DIEN  ========================================================== */
#define ATMR_IER_UIEN_Pos           (0U)                                        /*!< UIEN (Bit 0)                              */
#define ATMR_IER_UIEN_Msk           (0x1UL << ATMR_IER_UIEN_Pos)                /*!< UIEN (Bitfield-Mask: 0x01)                */
#define ATMR_IER_UIEN               ATMR_IER_UIEN_Msk
#define ATMR_IER_CC0IEN_Pos         (1U)                                        /*!< CC0IEN (Bit 1)                            */
#define ATMR_IER_CC0IEN_Msk         (0x1UL << ATMR_IER_CC0IEN_Pos)              /*!< CC0IEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_CC0IEN             ATMR_IER_CC0IEN_Msk
#define ATMR_IER_CC1IEN_Pos         (2U)                                        /*!< CC1IEN (Bit 2)                            */
#define ATMR_IER_CC1IEN_Msk         (0x1UL << ATMR_IER_CC1IEN_Pos)              /*!< CC1IEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_CC1IEN             ATMR_IER_CC1IEN_Msk
#define ATMR_IER_CC2IEN_Pos         (3U)                                        /*!< CC2IEN (Bit 3)                            */
#define ATMR_IER_CC2IEN_Msk         (0x1UL << ATMR_IER_CC2IEN_Pos)              /*!< CC2IEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_CC2IEN             ATMR_IER_CC2IEN_Msk
#define ATMR_IER_CC3IEN_Pos         (4U)                                        /*!< CC3IEN (Bit 4)                            */
#define ATMR_IER_CC3IEN_Msk         (0x1UL << ATMR_IER_CC3IEN_Pos)              /*!< CC3IEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_CC3IEN             ATMR_IER_CC3IEN_Msk
#define ATMR_IER_COMIEN_Pos         (5U)                                        /*!< COMIEN (Bit 5)                            */
#define ATMR_IER_COMIEN_Msk         (0x1UL << ATMR_IER_COMIEN_Pos)              /*!< COMIEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_COMIEN             ATMR_IER_COMIEN_Msk
#define ATMR_IER_TRGIEN_Pos         (6U)                                        /*!< TRGIEN (Bit 6)                            */
#define ATMR_IER_TRGIEN_Msk         (0x1UL << ATMR_IER_TRGIEN_Pos)              /*!< TRGIEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_TRGIEN             ATMR_IER_TRGIEN_Msk
#define ATMR_IER_BRKIEN_Pos         (7U)                                        /*!< BRKIEN (Bit 7)                            */
#define ATMR_IER_BRKIEN_Msk         (0x1UL << ATMR_IER_BRKIEN_Pos)              /*!< BRKIEN (Bitfield-Mask: 0x01)              */
#define ATMR_IER_BRKIEN             ATMR_IER_BRKIEN_Msk
/* ==========================================================  STS  ========================================================== */
#define ATMR_SR_UIFLG_Pos           (0U)                                        /*!< UIFLG (Bit 0)                             */
#define ATMR_SR_UIFLG_Msk           (0x1UL << ATMR_SR_UIFLG_Pos)                /*!< UIFLG (Bitfield-Mask: 0x01)               */
#define ATMR_SR_UIFLG               ATMR_SR_UIFLG_Msk
#define ATMR_SR_CC0IFLG_Pos         (1U)                                        /*!< CC0IFLG (Bit 1)                           */
#define ATMR_SR_CC0IFLG_Msk         (0x1UL << ATMR_SR_CC0IFLG_Pos)              /*!< CC0IFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_CC0IFLG             ATMR_SR_CC0IFLG_Msk
#define ATMR_SR_CC1IFLG_Pos         (2U)                                        /*!< CC1IFLG (Bit 2)                           */
#define ATMR_SR_CC1IFLG_Msk         (0x1UL << ATMR_SR_CC1IFLG_Pos)              /*!< CC1IFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_CC1IFLG             ATMR_SR_CC1IFLG_Msk
#define ATMR_SR_CC2IFLG_Pos         (3U)                                        /*!< CC2IFLG (Bit 3)                           */
#define ATMR_SR_CC2IFLG_Msk         (0x1UL << ATMR_SR_CC2IFLG_Pos)              /*!< CC2IFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_CC2IFLG             ATMR_SR_CC2IFLG_Msk
#define ATMR_SR_CC3IFLG_Pos         (4U)                                        /*!< CC3IFLG (Bit 4)                           */
#define ATMR_SR_CC3IFLG_Msk         (0x1UL << ATMR_SR_CC3IFLG_Pos)              /*!< CC3IFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_CC3IFLG             ATMR_SR_CC3IFLG_Msk
#define ATMR_SR_COMIFLG_Pos         (5U)                                        /*!< COMIFLG (Bit 5)                           */
#define ATMR_SR_COMIFLG_Msk         (0x1UL << ATMR_SR_COMIFLG_Pos)              /*!< COMIFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_COMIFLG             ATMR_SR_COMIFLG_Msk
#define ATMR_SR_TRGIFLG_Pos         (6U)                                        /*!< TRGIFLG (Bit 6)                           */
#define ATMR_SR_TRGIFLG_Msk         (0x1UL << ATMR_SR_TRGIFLG_Pos)              /*!< TRGIFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_TRGIFLG             ATMR_SR_TRGIFLG_Msk
#define ATMR_SR_BRKIFLG_Pos         (7U)                                        /*!< BRKIFLG (Bit 7)                           */
#define ATMR_SR_BRKIFLG_Msk         (0x1UL << ATMR_SR_BRKIFLG_Pos)              /*!< BRKIFLG (Bitfield-Mask: 0x01)             */
#define ATMR_SR_BRKIFLG             ATMR_SR_BRKIFLG_Msk
/* ==========================================================  CEG  ========================================================== */
#define ATMR_CEG_UEG_Pos            (0U)                                        /*!< UEG (Bit 0)                               */
#define ATMR_CEG_UEG_Msk            (0x1UL << ATMR_CEG_UEG_Pos)                 /*!< UEG (Bitfield-Mask: 0x01)                 */
#define ATMR_CEG_UEG                ATMR_CEG_UEG_Msk
#define ATMR_CEG_CC0EG_Pos          (1U)                                        /*!< CC0EG (Bit 1)                             */
#define ATMR_CEG_CC0EG_Msk          (0x1UL << ATMR_CEG_CC0EG_Pos)               /*!< CC0EG (Bitfield-Mask: 0x01)               */
#define ATMR_CEG_CC0EG              ATMR_CEG_CC0EG_Msk
#define ATMR_CEG_CC1EG_Pos          (2U)                                        /*!< CC1EG (Bit 2)                             */
#define ATMR_CEG_CC1EG_Msk          (0x1UL << ATMR_CEG_CC1EG_Pos)               /*!< CC1EG (Bitfield-Mask: 0x01)               */
#define ATMR_CEG_CC1EG              ATMR_CEG_CC1EG_Msk
#define ATMR_CEG_CC2EG_Pos          (3U)                                        /*!< CC2EG (Bit 3)                             */
#define ATMR_CEG_CC2EG_Msk          (0x1UL << ATMR_CEG_CC2EG_Pos)               /*!< CC2EG (Bitfield-Mask: 0x01)               */
#define ATMR_CEG_CC2EG              ATMR_CEG_CC2EG_Msk
#define ATMR_CEG_CC3EG_Pos          (4U)                                        /*!< CC3EG (Bit 4)                             */
#define ATMR_CEG_CC3EG_Msk          (0x1UL << ATMR_CEG_CC3EG_Pos)               /*!< CC3EG (Bitfield-Mask: 0x01)               */
#define ATMR_CEG_CC3EG              ATMR_CEG_CC3EG_Msk
#define ATMR_CEG_COMG_Pos           (5U)                                        /*!< COMG (Bit 5)                              */
#define ATMR_CEG_COMG_Msk           (0x1UL << ATMR_CEG_COMG_Pos)                /*!< COMG (Bitfield-Mask: 0x01)                */
#define ATMR_CEG_COMG               ATMR_CEG_COMG_Msk
#define ATMR_CEG_TEG_Pos            (6U)                                        /*!< TEG (Bit 6)                               */
#define ATMR_CEG_TEG_Msk            (0x1UL << ATMR_CEG_TEG_Pos)                 /*!< TEG (Bitfield-Mask: 0x01)                 */
#define ATMR_CEG_TEG                ATMR_CEG_TEG_Msk
#define ATMR_CEG_BEG_Pos            (7U)                                        /*!< BEG (Bit 7)                               */
#define ATMR_CEG_BEG_Msk            (0x1UL << ATMR_CEG_BEG_Pos)                 /*!< BEG (Bitfield-Mask: 0x01)                 */
#define ATMR_CEG_BEG                ATMR_CEG_BEG_Msk
/* ========================================================  CCM1  =========================================================== */
#define ATMR_CCM1_OC0PEN_Pos        (3U)                                        /*!< OC0PEN (Bit 3)                            */
#define ATMR_CCM1_OC0PEN_Msk        (0x1UL << ATMR_CCM1_OC0PEN_Pos)             /*!< OC0PEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCM1_OC0PEN            ATMR_CCM1_OC0PEN_Msk
#define ATMR_CCM1_OC0MOD_Pos        (4U)                                        /*!< OC0MOD (Bit 4)                            */
#define ATMR_CCM1_OC0MOD_Msk        (0x7UL << ATMR_CCM1_OC0MOD_Pos)             /*!< OC0MOD (Bitfield-Mask: 0x07)              */
#define ATMR_CCM1_OC0MOD            ATMR_CCM1_OC0MOD_Msk
#define ATMR_CCM1_OC0MOD_0          (0x1UL << ATMR_CCM1_OC0MOD_Pos)
#define ATMR_CCM1_OC0MOD_1          (0x2UL << ATMR_CCM1_OC0MOD_Pos)
#define ATMR_CCM1_OC0MOD_2          (0x4UL << ATMR_CCM1_OC0MOD_Pos)

#define ATMR_CCM1_OC0CEN_Pos        (7U)                                        /*!< OC0CEN (Bit 7)                            */
#define ATMR_CCM1_OC0CEN_Msk        (0x1UL << ATMR_CCM1_OC0CEN_Pos)             /*!< OC0CEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCM1_OC0CEN            ATMR_CCM1_OC0CEN_Msk
#define ATMR_CCM1_OC1PEN_Pos        (11U)                                       /*!< OC1PEN (Bit 11)                           */
#define ATMR_CCM1_OC1PEN_Msk        (0x1UL << ATMR_CCM1_OC1PEN_Pos)             /*!< OC1PEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCM1_OC1PEN            ATMR_CCM1_OC1PEN_Msk
#define ATMR_CCM1_OC1MOD_Pos        (12U)                                       /*!< OC1MOD (Bit 12)                           */
#define ATMR_CCM1_OC1MOD_Msk        (0x7UL << ATMR_CCM1_OC1MOD_Pos)             /*!< OC1MOD (Bitfield-Mask: 0x07)              */
#define ATMR_CCM1_OC1MOD            ATMR_CCM1_OC1MOD_Msk
#define ATMR_CCM1_OC1MOD_0          (0x1UL << ATMR_CCM1_OC1MOD_Pos)
#define ATMR_CCM1_OC1MOD_1          (0x2UL << ATMR_CCM1_OC1MOD_Pos)
#define ATMR_CCM1_OC1MOD_2          (0x4UL << ATMR_CCM1_OC1MOD_Pos)

#define ATMR_CCM1_OC1CEN_Pos        (15U)                                       /*!< OC1CEN (Bit 15)                           */
#define ATMR_CCM1_OC1CEN_Msk        (0x1UL << ATMR_CCM1_OC1CEN_Pos)             /*!< OC1CEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCM1_OC1CEN            ATMR_CCM1_OC1CEN_Msk
/* ========================================================  CCM2  ======================================================= */
#define ATMR_CCM2_OC2PEN_Pos        (3U)                                        /*!< OC2PEN (Bit 3)                        */
#define ATMR_CCM2_OC2PEN_Msk        (0x1UL << ATMR_CCM2_OC2PEN_Pos)             /*!< OC2PEN (Bitfield-Mask: 0x01)          */
#define ATMR_CCM2_OC2PEN            ATMR_CCM2_OC2PEN_Msk
#define ATMR_CCM2_OC2MOD_Pos        (4U)                                        /*!< OC2MOD (Bit 4)                        */
#define ATMR_CCM2_OC2MOD_Msk        (0x7UL << ATMR_CCM2_OC2MOD_Pos)             /*!< OC2MOD (Bitfield-Mask: 0x07)          */
#define ATMR_CCM2_OC2MOD            ATMR_CCM2_OC2MOD_Msk
#define ATMR_CCM2_OC2CEN_Pos        (7U)                                        /*!< OC2CEN (Bit 7)                        */
#define ATMR_CCM2_OC2CEN_Msk        (0x1UL << ATMR_CCM2_OC2CEN_Pos)             /*!< OC2CEN (Bitfield-Mask: 0x01)          */
#define ATMR_CCM2_OC2CEN            ATMR_CCM2_OC2CEN_Msk
#define ATMR_CCM2_OC3PEN_Pos        (11U)                                       /*!< OC3PEN (Bit 11)                       */
#define ATMR_CCM2_OC3PEN_Msk        (0x1UL << ATMR_CCM2_OC3PEN_Pos)             /*!< OC3PEN (Bitfield-Mask: 0x01)          */
#define ATMR_CCM2_OC3PEN            ATMR_CCM2_OC3PEN_Msk
#define ATMR_CCM2_OC3MOD_Pos        (12U)                                       /*!< OC3MOD (Bit 12)                       */
#define ATMR_CCM2_OC3MOD_Msk        (0x7UL << ATMR_CCM2_OC3MOD_Pos)             /*!< OC3MOD (Bitfield-Mask: 0x07)          */
#define ATMR_CCM2_OC3MOD            ATMR_CCM2_OC3MOD_Msk
#define ATMR_CCM2_OC3CEN_Pos        (15U)                                       /*!< OC3CEN (Bit 15)                       */
#define ATMR_CCM2_OC3CEN_Msk        (0x1UL << ATMR_CCM2_OC3CEN_Pos)             /*!< OC3CEN (Bitfield-Mask: 0x01)          */
#define ATMR_CCM2_OC3CEN            ATMR_CCM2_OC3CEN_Msk
/* =========================================================  CCEN  ========================================================== */
#define ATMR_CCEN_CC0EN_Pos         (0U)                                        /*!< CC0EN (Bit 0)                             */
#define ATMR_CCEN_CC0EN_Msk         (0x1UL << ATMR_CCEN_CC0EN_Pos)              /*!< CC0EN (Bitfield-Mask: 0x01)               */
#define ATMR_CCEN_CC0EN             ATMR_CCEN_CC0EN_Msk
#define ATMR_CCEN_CC0POL_Pos        (1U)                                        /*!< CC0POL (Bit 1)                            */
#define ATMR_CCEN_CC0POL_Msk        (0x1UL << ATMR_CCEN_CC0POL_Pos)             /*!< CC0POL (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC0POL            ATMR_CCEN_CC0POL_Msk
#define ATMR_CCEN_CC0NEN_Pos        (2U)                                        /*!< CC0NEN (Bit 2)                            */
#define ATMR_CCEN_CC0NEN_Msk        (0x1UL << ATMR_CCEN_CC0NEN_Pos)             /*!< CC0NEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC0NEN            ATMR_CCEN_CC0NEN_Msk
#define ATMR_CCEN_CC0NPOL_Pos       (3U)                                        /*!< CC0NPOL (Bit 3)                           */
#define ATMR_CCEN_CC0NPOL_Msk       (0x1UL << ATMR_CCEN_CC0NPOL_Pos)            /*!< CC0NPOL (Bitfield-Mask: 0x01)             */
#define ATMR_CCEN_CC0NPOL           ATMR_CCEN_CC0NPOL_Msk
#define ATMR_CCEN_CC1EN_Pos         (4U)                                        /*!< CC1EN (Bit 4)                             */
#define ATMR_CCEN_CC1EN_Msk         (0x1UL << ATMR_CCEN_CC1EN_Pos)              /*!< CC1EN (Bitfield-Mask: 0x01)               */
#define ATMR_CCEN_CC1EN             ATMR_CCEN_CC1EN_Msk
#define ATMR_CCEN_CC1POL_Pos        (5U)                                        /*!< CC1POL (Bit 5)                            */
#define ATMR_CCEN_CC1POL_Msk        (0x1UL << ATMR_CCEN_CC1POL_Pos)             /*!< CC1POL (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC1POL            ATMR_CCEN_CC1POL_Msk
#define ATMR_CCEN_CC1NEN_Pos        (6U)                                        /*!< CC1NEN (Bit 6)                            */
#define ATMR_CCEN_CC1NEN_Msk        (0x1UL << ATMR_CCEN_CC1NEN_Pos)             /*!< CC1NEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC1NEN            ATMR_CCEN_CC1NEN_Msk
#define ATMR_CCEN_CC1NPOL_Pos       (7U)                                        /*!< CC1NPOL (Bit 7)                           */
#define ATMR_CCEN_CC1NPOL_Msk       (0x1UL << ATMR_CCEN_CC1NPOL_Pos)            /*!< CC1NPOL (Bitfield-Mask: 0x01)             */
#define ATMR_CCEN_CC1NPOL           ATMR_CCEN_CC1NPOL_Msk
#define ATMR_CCEN_CC2EN_Pos         (8U)                                        /*!< CC2EN (Bit 8)                             */
#define ATMR_CCEN_CC2EN_Msk         (0x1UL << ATMR_CCEN_CC2EN_Pos)              /*!< CC2EN (Bitfield-Mask: 0x01)               */
#define ATMR_CCEN_CC2EN             ATMR_CCEN_CC2EN_Msk
#define ATMR_CCEN_CC2POL_Pos        (9U)                                        /*!< CC2POL (Bit 9)                            */
#define ATMR_CCEN_CC2POL_Msk        (0x1UL << ATMR_CCEN_CC2POL_Pos)             /*!< CC2POL (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC2POL            ATMR_CCEN_CC2POL_Msk
#define ATMR_CCEN_CC2NEN_Pos        (10U)                                       /*!< CC2NEN (Bit 10)                           */
#define ATMR_CCEN_CC2NEN_Msk        (0x1UL << ATMR_CCEN_CC2NEN_Pos)             /*!< CC2NEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC2NEN            ATMR_CCEN_CC2NEN_Msk
#define ATMR_CCEN_CC2NPOL_Pos       (11U)                                       /*!< CC2NPOL (Bit 11)                          */
#define ATMR_CCEN_CC2NPOL_Msk       (0x1UL << ATMR_CCEN_CC2NPOL_Pos)            /*!< CC2NPOL (Bitfield-Mask: 0x01)             */
#define ATMR_CCEN_CC2NPOL           ATMR_CCEN_CC2NPOL_Msk
#define ATMR_CCEN_CC3EN_Pos         (12U)                                       /*!< CC3EN (Bit 12)                            */
#define ATMR_CCEN_CC3EN_Msk         (0x1UL << ATMR_CCEN_CC3EN_Pos)              /*!< CC3EN (Bitfield-Mask: 0x01)               */
#define ATMR_CCEN_CC3EN             ATMR_CCEN_CC3EN_Msk
#define ATMR_CCEN_CC3POL_Pos        (13U)                                       /*!< CC3POL (Bit 13)                           */
#define ATMR_CCEN_CC3POL_Msk        (0x1UL << ATMR_CCEN_CC3POL_Pos)             /*!< CC3POL (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC3POL            ATMR_CCEN_CC3POL_Msk
#define ATMR_CCEN_CC3NEN_Pos        (14U)                                       /*!< CC3NEN (Bit 14)                           */
#define ATMR_CCEN_CC3NEN_Msk        (0x1UL << ATMR_CCEN_CC3NEN_Pos)             /*!< CC3NEN (Bitfield-Mask: 0x01)              */
#define ATMR_CCEN_CC3NEN            ATMR_CCEN_CC3NEN_Msk
#define ATMR_CCEN_CC3NPOL_Pos       (15U)                                       /*!< CC3NPOL (Bit 15)                          */
#define ATMR_CCEN_CC3NPOL_Msk       (0x1UL << ATMR_CCEN_CC3NPOL_Pos)            /*!< CC3NPOL (Bitfield-Mask: 0x01)             */
#define ATMR_CCEN_CC3NPOL           ATMR_CCEN_CC3NPOL_Msk
/* ==========================================================  CNT  ========================================================== */
#define ATMR_CNT_CNT_Pos            (0U)                                        /*!< CNT (Bit 0)                               */
#define ATMR_CNT_CNT_Msk            (0xFFFFUL << ATMR_CNT_CNT_Pos)              /*!< CNT (Bitfield-Mask: 0xffff)               */
#define ATMR_CNT_CNT                ATMR_CNT_CNT_Msk
/* ==========================================================  PSC  ========================================================== */
#define ATMR_PSC_PSC_Pos            (0U)                                        /*!< PSC (Bit 0)                               */
#define ATMR_PSC_PSC_Msk            (0xFFFFUL << ATMR_PSC_PSC_Pos)              /*!< PSC (Bitfield-Mask: 0xffff)               */
#define ATMR_PSC_PSC                ATMR_PSC_PSC_Msk
/* ========================================================  AUTORLD  ======================================================== */
#define ATMR_AUTORLD_AUTORLD_Pos    (0U)                                        /*!< AUTORLD (Bit 0)                       */
#define ATMR_AUTORLD_AUTORLD_Msk    (0xFFFFUL << ATMR_AUTORLD_AUTORLD_Pos)      /*!< AUTORLD (Bitfield-Mask: 0xffff)       */
#define ATMR_AUTORLD_AUTORLD        ATMR_AUTORLD_AUTORLD_Msk
/* ========================================================  REPCNT  ========================================================= */
#define ATMR_REPCNT_REPCNT_Pos      (0U)                                        /*!< REPCNT (Bit 0)                            */
#define ATMR_REPCNT_REPCNT_Msk      (0xFFUL << ATMR_REPCNT_REPCNT_Pos)          /*!< REPCNT (Bitfield-Mask: 0xff)              */
#define ATMR_REPCNT_REPCNT          ATMR_REPCNT_REPCNT_Msk
/* ==========================================================  CC0  ========================================================== */
#define ATMR_CC0_CC0_Pos            (0U)                                        /*!< CC0 (Bit 0)                               */
#define ATMR_CC0_CC0_Msk            (0xFFFFUL << ATMR_CC0_CC0_Pos)              /*!< CC0 (Bitfield-Mask: 0xffff)               */
#define ATMR_CC0_CC0                ATMR_CC0_CC0_Msk
/* ==========================================================  CC1  ========================================================== */
#define ATMR_CC1_CC1_Pos            (0U)                                        /*!< CC1 (Bit 0)                               */
#define ATMR_CC1_CC1_Msk            (0xFFFFUL << ATMR_CC1_CC1_Pos)              /*!< CC1 (Bitfield-Mask: 0xffff)               */
#define ATMR_CC1_CC1                ATMR_CC1_CC1_Msk
/* ==========================================================  CC2  ========================================================== */
#define ATMR_CC2_CC2_Pos            (0U)                                        /*!< CC2 (Bit 0)                               */
#define ATMR_CC2_CC2_Msk            (0xFFFFUL << ATMR_CC2_CC2_Pos)              /*!< CC2 (Bitfield-Mask: 0xffff)               */
#define ATMR_CC2_CC2                (ATMR_CC2_CC2_Msk)
/* ==========================================================  CC3  ========================================================== */
#define ATMR_CC3_CC3_Pos            (0U)                                        /*!< CC3 (Bit 0)                               */
#define ATMR_CC3_CC3_Msk            (0xFFFFUL << ATMR_CC3_CC3_Pos)              /*!< CC3 (Bitfield-Mask: 0xffff)               */
#define ATMR_CC3_CC3                (ATMR_CC3_CC3_Msk)
/* ==========================================================  BDT  ========================================================== */
#define ATMR_BDT_DTS0_Pos           (0U)                                        /*!< DTS (Bit 0)                               */
#define ATMR_BDT_DTS0_Msk           (0xFFUL << ATMR_BDT_DTS0_Pos)               /*!< DTS (Bitfield-Mask: 0xff)                 */
#define ATMR_BDT_DTS0               ATMR_BDT_DTS0_Msk
#define ATMR_BDT_LOCKCFG_Pos        (8U)                                        /*!< LOCKCFG (Bit 8)                           */
#define ATMR_BDT_LOCKCFG_Msk        (0x3UL << ATMR_BDT_LOCKCFG_Pos)             /*!< LOCKCFG (Bitfield-Mask: 0x03)             */
#define ATMR_BDT_LOCKCFG            ATMR_BDT_LOCKCFG_Msk
#define ATMR_BDT_LOCKCFG_0          (0x1UL << ATMR_BDT_LOCKCFG_Pos)
#define ATMR_BDT_LOCKCFG_1          (0x2UL << ATMR_BDT_LOCKCFG_Pos)

#define ATMR_BDT_IMOS_Pos           (10U)                                       /*!< IMOS (Bit 10)                             */
#define ATMR_BDT_IMOS_Msk           (0x1UL << ATMR_BDT_IMOS_Pos)                /*!< IMOS (Bitfield-Mask: 0x01)                */
#define ATMR_BDT_IMOS               ATMR_BDT_IMOS_Msk
#define ATMR_BDT_RMOS_Pos           (11U)                                       /*!< RMOS (Bit 11)                             */
#define ATMR_BDT_RMOS_Msk           (0x1UL << ATMR_BDT_RMOS_Pos)                /*!< RMOS (Bitfield-Mask: 0x01)                */
#define ATMR_BDT_RMOS               ATMR_BDT_RMOS_Msk
#define ATMR_BDT_BRKEN_Pos          (12U)                                       /*!< BRKEN (Bit 12)                            */
#define ATMR_BDT_BRKEN_Msk          (0x1UL << ATMR_BDT_BRKEN_Pos)               /*!< BRKEN (Bitfield-Mask: 0x01)               */
#define ATMR_BDT_BRKEN              ATMR_BDT_BRKEN_Msk
#define ATMR_BDT_BRKPOL_Pos         (13U)                                       /*!< BRKPOL (Bit 13)                           */
#define ATMR_BDT_BRKPOL_Msk         (0x1UL << ATMR_BDT_BRKPOL_Pos)              /*!< BRKPOL (Bitfield-Mask: 0x01)              */
#define ATMR_BDT_BRKPOL             ATMR_BDT_BRKPOL_Msk
#define ATMR_BDT_AOEN_Pos           (14U)                                       /*!< AOEN (Bit 14)                             */
#define ATMR_BDT_AOEN_Msk           (0x1UL << ATMR_BDT_AOEN_Pos)                /*!< AOEN (Bitfield-Mask: 0x01)                */
#define ATMR_BDT_AOEN               ATMR_BDT_AOEN_Msk
#define ATMR_BDT_MOEN_Pos           (15U)                                       /*!< MOEN (Bit 15)                             */
#define ATMR_BDT_MOEN_Msk           (0x1UL << ATMR_BDT_MOEN_Pos)                /*!< MOEN (Bitfield-Mask: 0x01)                */
#define ATMR_BDT_MOEN               ATMR_BDT_MOEN_Msk
#define ATMR_BDT_DTS1_Pos           (16U)                                       /*!< DTS1 (Bit 16)                             */
#define ATMR_BDT_DTS1_Msk           (0xFFUL << ATMR_BDT_DTS1_Pos)               /*!< DTS1 (Bitfield-Mask: 0xff)                */
#define ATMR_BDT_DTS1               ATMR_BDT_DTS1_Msk
/* ======================================================  OUTPUTCTRL1  ====================================================== */
#define ATMR_OCR1_CH0FORCEEN_Pos    (0U)                                        /*!< CH0FORCEEN (Bit 0)                 */
#define ATMR_OCR1_CH0FORCEEN_Msk    (0x1UL << ATMR_OCR1_CH0FORCEEN_Pos)         /*!< CH0FORCEEN (Bitfield-Mask: 0x01)   */
#define ATMR_OCR1_CH0FORCEEN        ATMR_OCR1_CH0FORCEEN_Msk
#define ATMR_OCR1_CH0NFORCEEN_Pos   (1U)                                        /*!< CH0NFORCEEN (Bit 1)                */
#define ATMR_OCR1_CH0NFORCEEN_Msk   (0x1UL << ATMR_OCR1_CH0NFORCEEN_Pos)        /*!< CH0NFORCEEN (Bitfield-Mask: 0x01)  */
#define ATMR_OCR1_CH0NFORCEEN       ATMR_OCR1_CH0NFORCEEN_Msk
#define ATMR_OCR1_CH1FORCEEN_Pos    (2U)                                        /*!< CH1FORCEEN (Bit 2)                 */
#define ATMR_OCR1_CH1FORCEEN_Msk    (0x1UL << ATMR_OCR1_CH1FORCEEN_Pos)         /*!< CH1FORCEEN (Bitfield-Mask: 0x01)   */
#define ATMR_OCR1_CH1FORCEEN        ATMR_OCR1_CH1FORCEEN_Msk
#define ATMR_OCR1_CH1NFORCEEN_Pos   (3U)                                        /*!< CH1NFORCEEN (Bit 3)                */
#define ATMR_OCR1_CH1NFORCEEN_Msk   (0x1UL << ATMR_OCR1_CH1NFORCEEN_Pos)        /*!< CH1NFORCEEN (Bitfield-Mask: 0x01)  */
#define ATMR_OCR1_CH1NFORCEEN       ATMR_OCR1_CH1NFORCEEN_Msk
#define ATMR_OCR1_CH2FORCEEN_Pos    (4U)                                        /*!< CH2FORCEEN (Bit 4)                 */
#define ATMR_OCR1_CH2FORCEEN_Msk    (0x1UL << ATMR_OCR1_CH2FORCEEN_Pos)         /*!< CH2FORCEEN (Bitfield-Mask: 0x01)   */
#define ATMR_OCR1_CH2FORCEEN        ATMR_OCR1_CH2FORCEEN_Msk
#define ATMR_OCR1_CH2NFORCEEN_Pos   (5U)                                        /*!< CH2NFORCEEN (Bit 5)                */
#define ATMR_OCR1_CH2NFORCEEN_Msk   (0x1UL << ATMR_OCR1_CH2NFORCEEN_Pos)        /*!< CH2NFORCEEN (Bitfield-Mask: 0x01)  */
#define ATMR_OCR1_CH2NFORCEEN       ATMR_OCR1_CH2NFORCEEN_Msk
#define ATMR_OCR1_CH3FORCEEN_Pos    (6U)                                        /*!< CH3FORCEEN (Bit 6)                 */
#define ATMR_OCR1_CH3FORCEEN_Msk    (0x1UL << ATMR_OCR1_CH3FORCEEN_Pos)         /*!< CH3FORCEEN (Bitfield-Mask: 0x01)   */
#define ATMR_OCR1_CH3FORCEEN        ATMR_OCR1_CH3FORCEEN_Msk
#define ATMR_OCR1_CH3NFORCEEN_Pos   (7U)                                        /*!< CH3NFORCEEN (Bit 7)                */
#define ATMR_OCR1_CH3NFORCEEN_Msk   (0x1UL << ATMR_OCR1_CH3NFORCEEN_Pos)        /*!< CH3NFORCEEN (Bitfield-Mask: 0x01)  */
#define ATMR_OCR1_CH3NFORCEEN       ATMR_OCR1_CH3NFORCEEN_Msk
#define ATMR_OCR1_BUFEN_Pos         (8U)                                        /*!< BUFEN (Bit 8)               */
#define ATMR_OCR1_BUFEN_Msk         (0x1UL << ATMR_OCR1_BUFEN_Pos)              /*!< BUFEN (Bitfield-Mask: 0x01) */
#define ATMR_OCR1_BUFEN             ATMR_OCR1_BUFEN_Msk
/* ======================================================  OUTPUTCTRL2  ====================================================== */
#define ATMR_OCR2_CH0FORCEVAL_Pos   (0U)                                        /*!< CH0FORCEVAL (Bit 0)                */
#define ATMR_OCR2_CH0FORCEVAL_Msk   (0x1UL << ATMR_OCR2_CH0FORCEVAL_Pos)        /*!< CH0FORCEVAL (Bitfield-Mask: 0x01)  */
#define ATMR_OCR2_CH0FORCEVAL       ATMR_OCR2_CH0FORCEVAL_Msk
#define ATMR_OCR2_CH0NFORCEVAL_Pos  (1U)                                        /*!< CH0NFORCEVAL (Bit 1)               */
#define ATMR_OCR2_CH0NFORCEVAL_Msk  (0x1UL << ATMR_OCR2_CH0NFORCEVAL_Pos)       /*!< CH0NFORCEVAL (Bitfield-Mask: 0x01) */
#define ATMR_OCR2_CH0NFORCEVAL      ATMR_OCR2_CH0NFORCEVAL_Msk
#define ATMR_OCR2_CH1FORCEVAL_Pos   (2U)                                        /*!< CH1FORCEVAL (Bit 2)                */
#define ATMR_OCR2_CH1FORCEVAL_Msk   (0x1UL << ATMR_OCR2_CH1FORCEVAL_Pos)        /*!< CH1FORCEVAL (Bitfield-Mask: 0x01)  */
#define ATMR_OCR2_CH1FORCEVAL       ATMR_OCR2_CH1FORCEVAL_Msk
#define ATMR_OCR2_CH1NFORCEVAL_Pos  (3U)                                        /*!< CH1NFORCEVAL (Bit 3)               */
#define ATMR_OCR2_CH1NFORCEVAL_Msk  (0x1UL << ATMR_OCR2_CH1NFORCEVAL_Pos)       /*!< CH1NFORCEVAL (Bitfield-Mask: 0x01) */
#define ATMR_OCR2_CH1NFORCEVAL      ATMR_OCR2_CH1NFORCEVAL_Msk
#define ATMR_OCR2_CH2FORCEVAL_Pos   (4U)                                        /*!< CH2FORCEVAL (Bit 4)                */
#define ATMR_OCR2_CH2FORCEVAL_Msk   (0x1UL << ATMR_OCR2_CH2FORCEVAL_Pos)        /*!< CH2FORCEVAL (Bitfield-Mask: 0x01)  */
#define ATMR_OCR2_CH2FORCEVAL       ATMR_OCR2_CH2FORCEVAL_Msk
#define ATMR_OCR2_CH2NFORCEVAL_Pos  (5U)                                        /*!< CH2NFORCEVAL (Bit 5)               */
#define ATMR_OCR2_CH2NFORCEVAL_Msk  (0x1UL << ATMR_OCR2_CH2NFORCEVAL_Pos)       /*!< CH2NFORCEVAL (Bitfield-Mask: 0x01) */
#define ATMR_OCR2_CH2NFORCEVAL      ATMR_OCR2_CH2NFORCEVAL_Msk
#define ATMR_OCR2_CH3FORCEVAL_Pos   (6U)                                        /*!< CH3FORCEVAL (Bit 6)                */
#define ATMR_OCR2_CH3FORCEVAL_Msk   (0x1UL << ATMR_OCR2_CH3FORCEVAL_Pos)        /*!< CH3FORCEVAL (Bitfield-Mask: 0x01)  */
#define ATMR_OCR2_CH3FORCEVAL       ATMR_OCR2_CH3FORCEVAL_Msk
#define ATMR_OCR2_CH3NFORCEVAL_Pos  (7U)                                        /*!< CH3NFORCEVAL (Bit 7)               */
#define ATMR_OCR2_CH3NFORCEVAL_Msk  (0x1UL << ATMR_OCR2_CH3NFORCEVAL_Pos)       /*!< CH3NFORCEVAL (Bitfield-Mask: 0x01) */
#define ATMR_OCR2_CH3NFORCEVAL      ATMR_OCR2_CH3NFORCEVAL_Msk
/* ========================================================  TRGOCR  ========================================================= */
#define ATMR_TRGOCR_MMS1_Pos        (0U)                                        /*!< MMS1 (Bit 0)                              */
#define ATMR_TRGOCR_MMS1_Msk        (0xFUL << ATMR_TRGOCR_MMS1_Pos)             /*!< MMS1 (Bitfield-Mask: 0x0f)                */
#define ATMR_TRGOCR_MMS1            ATMR_TRGOCR_MMS1_Msk
#define ATMR_TRGOCR_MMS2_Pos        (4U)                                        /*!< MMS2 (Bit 4)                              */
#define ATMR_TRGOCR_MMS2_Msk        (0xFUL << ATMR_TRGOCR_MMS2_Pos)             /*!< MMS2 (Bitfield-Mask: 0x0f)                */
#define ATMR_TRGOCR_MMS2            (ATMR_TRGOCR_MMS2_Msk)
#define ATMR_TRGOCR_MMS1ZE_Pos      (8U)                                        /*!< MMS1ZE (Bit 8)                            */
#define ATMR_TRGOCR_MMS1ZE_Msk      (0x1UL << ATMR_TRGOCR_MMS1ZE_Pos)           /*!< MMS1ZE (Bitfield-Mask: 0x01)              */
#define ATMR_TRGOCR_MMS1ZE          ATMR_TRGOCR_MMS1ZE_Msk
#define ATMR_TRGOCR_MMS1PE_Pos      (9U)                                        /*!< MMS1PE (Bit 9)                            */
#define ATMR_TRGOCR_MMS1PE_Msk      (0x1UL << ATMR_TRGOCR_MMS1PE_Pos)           /*!< MMS1PE (Bitfield-Mask: 0x01)              */
#define ATMR_TRGOCR_MMS1PE          ATMR_TRGOCR_MMS1PE_Msk
#define ATMR_TRGOCR_MMS2ZE_Pos      (10U)                                       /*!< MMS2ZE (Bit 10)                           */
#define ATMR_TRGOCR_MMS2ZE_Msk      (0x1UL << ATMR_TRGOCR_MMS2ZE_Pos)           /*!< MMS2ZE (Bitfield-Mask: 0x01)              */
#define ATMR_TRGOCR_MMS2ZE          ATMR_TRGOCR_MMS2ZE_Msk
#define ATMR_TRGOCR_MMS2PE_Pos      (11U)                                       /*!< MMS2PE (Bit 11)                           */
#define ATMR_TRGOCR_MMS2PE_Msk      (0x1UL << ATMR_TRGOCR_MMS2PE_Pos)           /*!< MMS2PE (Bitfield-Mask: 0x01)              */
#define ATMR_TRGOCR_MMS2PE          ATMR_TRGOCR_MMS2PE_Msk
/* =========================================================  BREAK  ========================================================= */
#define ATMR_BREAK_ANAFILTEN_Pos    (0U)                                        /*!< ANAFILTEN (Bit 0)               */
#define ATMR_BREAK_ANAFILTEN_Msk    (0x1UL << ATMR_BREAK_ANAFILTEN_Pos)         /*!< ANAFILTEN (Bitfield-Mask: 0x01) */
#define ATMR_BREAK_ANAFILTEN        ATMR_BREAK_ANAFILTEN_Msk
#define ATMR_BREAK_FILTEN_Pos       (1U)                                        /*!< FILTEN (Bit 1)                   */
#define ATMR_BREAK_FILTEN_Msk       (0x1UL << ATMR_BREAK_FILTEN_Pos)            /*!< FILTEN (Bitfield-Mask: 0x01)     */
#define ATMR_BREAK_FILTEN           ATMR_BREAK_FILTEN_Msk
#define ATMR_BREAK_FILT_Pos         (2U)                                        /*!< FILT (Bit 2)                      */
#define ATMR_BREAK_FILT_Msk         (0x3fUL << ATMR_BREAK_FILT_Pos)             /*!< FILT (Bitfield-Mask: 0x3f)        */
#define ATMR_BREAK_FILT             ATMR_BREAK_FILT_Msk
/* ========================================================  OCXAEN  ========================================================= */
#define ATMR_OCXACR_OC0AEN_Pos      (0U)                                        /*!< OC0AEN (Bit 0)                            */
#define ATMR_OCXACR_OC0AEN_Msk      (0x1UL << ATMR_OCXACR_OC0AEN_Pos)           /*!< OC0AEN (Bitfield-Mask: 0x01)              */
#define ATMR_OCXACR_OC0AEN          ATMR_OCXACR_OC0AEN_Msk
#define ATMR_OCXACR_OC1AEN_Pos      (1U)                                        /*!< OC1AEN (Bit 1)                            */
#define ATMR_OCXACR_OC1AEN_Msk      (0x1UL << ATMR_OCXACR_OC1AEN_Pos)           /*!< OC1AEN (Bitfield-Mask: 0x01)              */
#define ATMR_OCXACR_OC1AEN          ATMR_OCXACR_OC1AEN_Msk
#define ATMR_OCXACR_OC2AEN_Pos      (2U)                                        /*!< OC2AEN (Bit 2)                            */
#define ATMR_OCXACR_OC2AEN_Msk      (0x1UL << ATMR_OCXACR_OC2AEN_Pos)           /*!< OC2AEN (Bitfield-Mask: 0x01)              */
#define ATMR_OCXACR_OC2AEN          ATMR_OCXACR_OC2AEN_Msk
#define ATMR_OCXACR_OC3AEN_Pos      (3U)                                        /*!< OC3AEN (Bit 3)                            */
#define ATMR_OCXACR_OC3AEN_Msk      (0x1UL << ATMR_OCXACR_OC3AEN_Pos)           /*!< OC3AEN (Bitfield-Mask: 0x01)              */
#define ATMR_OCXACR_OC3AEN          ATMR_OCXACR_OC3AEN_Msk
#define ATMR_OCXACR_NONC0EN_Pos     (4U)                                        /*!< NONC0EN (Bit 4)                           */
#define ATMR_OCXACR_NONC0EN_Msk     (0x1UL << ATMR_OCXACR_NONC0EN_Pos)          /*!< NONC0EN (Bitfield-Mask: 0x01)             */
#define ATMR_OCXACR_NONC0EN         ATMR_OCXACR_NONC0EN_Msk
#define ATMR_OCXACR_NONC1EN_Pos     (5U)                                        /*!< NONC1EN (Bit 5)                           */
#define ATMR_OCXACR_NONC1EN_Msk     (0x1UL << ATMR_OCXACR_NONC1EN_Pos)          /*!< NONC1EN (Bitfield-Mask: 0x01)             */
#define ATMR_OCXACR_NONC1EN         ATMR_OCXACR_NONC1EN_Msk
#define ATMR_OCXACR_NONC2EN_Pos     (6U)                                        /*!< NONC2EN (Bit 6)                           */
#define ATMR_OCXACR_NONC2EN_Msk     (0x1UL << ATMR_OCXACR_NONC2EN_Pos)          /*!< NONC2EN (Bitfield-Mask: 0x01)             */
#define ATMR_OCXACR_NONC2EN         ATMR_OCXACR_NONC2EN_Msk
#define ATMR_OCXACR_NONC3EN_Pos     (7U)                                        /*!< NONC3EN (Bit 7)                           */
#define ATMR_OCXACR_NONC3EN_Msk     (0x1UL << ATMR_OCXACR_NONC3EN_Pos)          /*!< NONC3EN (Bitfield-Mask: 0x01)             */
#define ATMR_OCXACR_NONC3EN         ATMR_OCXACR_NONC3EN_Msk
/* =========================================================  CC0C  ========================================================== */
#define ATMR_CC0C_CCR0C_Pos         (0U)                                        /*!< CCR0C (Bit 0)                             */
#define ATMR_CC0C_CCR0C_Msk         (0xFFFFUL << ATMR_CC0C_CCR0C_Pos)           /*!< CCR0C (Bitfield-Mask: 0xffff)             */
#define ATMR_CC0C_CCR0C             ATMR_CC0C_CCR0C_Msk
/* =========================================================  CC1C  ========================================================== */
#define ATMR_CC1C_CCR1C_Pos         (0U)                                        /*!< CCR1C (Bit 0)                             */
#define ATMR_CC1C_CCR1C_Msk         (0xFFFFUL << ATMR_CC1C_CCR1C_Pos)           /*!< CCR1C (Bitfield-Mask: 0xffff)             */
#define ATMR_CC1C_CCR1C             ATMR_CC1C_CCR1C_Msk
/* =========================================================  CC2C  ========================================================== */
#define ATMR_CC2C_CCR2C_Pos         (0U)                                        /*!< CCR2C (Bit 0)                             */
#define ATMR_CC2C_CCR2C_Msk         (0xFFFFUL << ATMR_CC2C_CCR2C_Pos)           /*!< CCR2C (Bitfield-Mask: 0xffff)             */
#define ATMR_CC2C_CCR2C             ATMR_CC2C_CCR2C_Msk
/* =========================================================  CC3C  ========================================================== */
#define ATMR_CC3C_CCR3C_Pos         (0U)                                        /*!< CCR3C (Bit 0)                             */
#define ATMR_CC3C_CCR3C_Msk         (0xFFFFUL << ATMR_CC3C_CCR3C_Pos)           /*!< CCR3C (Bitfield-Mask: 0xffff)             */
#define ATMR_CC3C_CCR3C             ATMR_CC3C_CCR3C_Msk


/* =========================================================================================================================== */
/* ================                                          GTMR                                           ================ */
/* =========================================================================================================================== */

/* =========================================================  CTRL1  ========================================================= */
#define GTMR_CR1_CNTEN_Pos          (0U)                                        /*!< CNTEN (Bit 0)                             */
#define GTMR_CR1_CNTEN_Msk          (0x1UL << GTMR_CR1_CNTEN_Pos)               /*!< CNTEN (Bitfield-Mask: 0x01)               */
#define GTMR_CR1_CNTEN              GTMR_CR1_CNTEN_Msk
#define GTMR_CR1_UD_Pos             (1U)                                        /*!< UD (Bit 1)                                */
#define GTMR_CR1_UD_Msk             (0x1UL << GTMR_CR1_UD_Pos)                  /*!< UD (Bitfield-Mask: 0x01)                  */
#define GTMR_CR1_UD                 GTMR_CR1_UD_Msk
#define GTMR_CR1_URSSEL_Pos         (2U)                                        /*!< URSSEL (Bit 2)                            */
#define GTMR_CR1_URSSEL_Msk         (0x1UL << GTMR_CR1_URSSEL_Pos)              /*!< URSSEL (Bitfield-Mask: 0x01)              */
#define GTMR_CR1_URSSEL             GTMR_CR1_URSSEL_Msk
#define GTMR_CR1_SPMEN_Pos          (3U)                                        /*!< SPMEN (Bit 3)                             */
#define GTMR_CR1_SPMEN_Msk          (0x1UL << GTMR_CR1_SPMEN_Pos)               /*!< SPMEN (Bitfield-Mask: 0x01)               */
#define GTMR_CR1_SPMEN              GTMR_CR1_SPMEN_Msk
#define GTMR_CR1_CNTDIR_Pos         (4U)                                        /*!< CNTDIR (Bit 4)                            */
#define GTMR_CR1_CNTDIR_Msk         (0x1UL << GTMR_CR1_CNTDIR_Pos)              /*!< CNTDIR (Bitfield-Mask: 0x01)              */
#define GTMR_CR1_CNTDIR             GTMR_CR1_CNTDIR_Msk
#define GTMR_CR1_CAMSEL_Pos         (5U)                                        /*!< CAMSEL (Bit 5)                            */
#define GTMR_CR1_CAMSEL_Msk         (0x3UL << GTMR_CR1_CAMSEL_Pos)              /*!< CAMSEL (Bitfield-Mask: 0x03)              */
#define GTMR_CR1_CAMSEL             GTMR_CR1_CAMSEL_Msk
#define GTMR_CR1_CAMSEL_0           (0x1UL << GTMR_CR1_CAMSEL_Pos)
#define GTMR_CR1_CAMSEL_1           (0x2UL << GTMR_CR1_CAMSEL_Pos)

#define GTMR_CR1_ARPEN_Pos          (7U)                                        /*!< ARPEN (Bit 7)                             */
#define GTMR_CR1_ARPEN_Msk          (0x1UL << GTMR_CR1_ARPEN_Pos)               /*!< ARPEN (Bitfield-Mask: 0x01)               */
#define GTMR_CR1_ARPEN              GTMR_CR1_ARPEN_Msk
#define GTMR_CR1_CLKDIV_Pos         (8U)                                        /*!< CLKDIV (Bit 8)                            */
#define GTMR_CR1_CLKDIV_Msk         (0x3UL << GTMR_CR1_CLKDIV_Pos)              /*!< CLKDIV (Bitfield-Mask: 0x03)              */
#define GTMR_CR1_CLKDIV             GTMR_CR1_CLKDIV_Msk
#define GTMR_CR1_CLKDIV_0           (0x1UL << GTMR_CR1_CLKDIV_Pos)
#define GTMR_CR1_CLKDIV_1           (0x2UL << GTMR_CR1_CLKDIV_Pos)

/* =========================================================  CTRL2  ========================================================= */
#define GTMR_CR2_CCDSEL_Pos         (3U)                                        /*!< CCDSEL (Bit 3)                            */
#define GTMR_CR2_CCDSEL_Msk         (0x1UL << GTMR_CR2_CCDSEL_Pos)              /*!< CCDSEL (Bitfield-Mask: 0x01)              */
#define GTMR_CR2_CCDSEL             GTMR_CR2_CCDSEL_Msk
#define GTMR_CR2_MMSEL_Pos          (4U)                                        /*!< MMSEL (Bit 4)                             */
#define GTMR_CR2_MMSEL_Msk          (0x7UL << GTMR_CR2_MMSEL_Pos)               /*!< MMSEL (Bitfield-Mask: 0x07)               */
#define GTMR_CR2_MMSEL              GTMR_CR2_MMSEL_Msk
#define GTMR_CR2_MMSEL_0            (0x1UL << GTMR_CR2_MMSEL_Pos)
#define GTMR_CR2_MMSEL_1            (0x2UL << GTMR_CR2_MMSEL_Pos)
#define GTMR_CR2_MMSEL_2            (0x4UL << GTMR_CR2_MMSEL_Pos)

#define GTMR_CR2_TI0SEL_Pos         (7U)                                        /*!< TI1SEL (Bit 7)                            */
#define GTMR_CR2_TI0SEL_Msk         (0x1UL << GTMR_CR2_TI0SEL_Pos)              /*!< TI1SEL (Bitfield-Mask: 0x01)              */
#define GTMR_CR2_TI0SEL             GTMR_CR2_TI0SEL_Msk
#define GTMR_CR2_COMPCH0SEL_Pos     (8U)                                        /*!< COMPCH0SEL (Bit 8)                        */
#define GTMR_CR2_COMPCH0SEL_Msk     (0x7UL << GTMR_CR2_COMPCH0SEL_Pos)          /*!< COMPCH0SEL (Bitfield-Mask: 0x07)          */
#define GTMR_CR2_COMPCH0SEL         GTMR_CR2_COMPCH0SEL_Msk
#define GTMR_CR2_COMPCH0SEL_0       (0x1UL << GTMR_CR2_COMPCH0SEL_Pos)
#define GTMR_CR2_COMPCH0SEL_1       (0x2UL << GTMR_CR2_COMPCH0SEL_Pos)
#define GTMR_CR2_COMPCH0SEL_2       (0x4UL << GTMR_CR2_COMPCH0SEL_Pos)

#define GTMR_CR2_COMPCH1SEL_Pos     (11U)                                       /*!< COMPCH1SEL (Bit 11)                       */
#define GTMR_CR2_COMPCH1SEL_Msk     (0x7UL << GTMR_CR2_COMPCH1SEL_Pos)          /*!< COMPCH1SEL (Bitfield-Mask: 0x07)          */
#define GTMR_CR2_COMPCH1SEL         GTMR_CR2_COMPCH1SEL_Msk
#define GTMR_CR2_COMPCH1SEL_0       (0x1UL << GTMR_CR2_COMPCH1SEL_Pos)
#define GTMR_CR2_COMPCH1SEL_1       (0x2UL << GTMR_CR2_COMPCH1SEL_Pos)
#define GTMR_CR2_COMPCH1SEL_2       (0x4UL << GTMR_CR2_COMPCH1SEL_Pos)

#define GTMR_CR2_COMPCH2SEL_Pos     (14U)                                       /*!< COMPCH2SEL (Bit 14)                       */
#define GTMR_CR2_COMPCH2SEL_Msk     (0x7UL << GTMR_CR2_COMPCH2SEL_Pos)          /*!< COMPCH2SEL (Bitfield-Mask: 0x07)          */
#define GTMR_CR2_COMPCH2SEL         GTMR_CR2_COMPCH2SEL_Msk
#define GTMR_CR2_COMPCH2SEL_0       (0x1UL << GTMR_CR2_COMPCH2SEL_Pos)
#define GTMR_CR2_COMPCH2SEL_1       (0x2UL << GTMR_CR2_COMPCH2SEL_Pos)
#define GTMR_CR2_COMPCH2SEL_2       (0x4UL << GTMR_CR2_COMPCH2SEL_Pos)

#define GTMR_CR2_COMPCH3SEL_Pos     (17U)                                       /*!< COMPCH3SEL (Bit 17)                       */
#define GTMR_CR2_COMPCH3SEL_Msk     (0x7UL << GTMR_CR2_COMPCH3SEL_Pos)          /*!< COMPCH3SEL (Bitfield-Mask: 0x07)          */
#define GTMR_CR2_COMPCH3SEL         GTMR_CR2_COMPCH3SEL_Msk
#define GTMR_CR2_COMPCH3SEL_0       (0x1UL << GTMR_CR2_COMPCH3SEL_Pos)
#define GTMR_CR2_COMPCH3SEL_1       (0x2UL << GTMR_CR2_COMPCH3SEL_Pos)
#define GTMR_CR2_COMPCH3SEL_2       (0x4UL << GTMR_CR2_COMPCH3SEL_Pos)

#define GTMR_CR2_COMPETRSEL_Pos     (20U)                                       /*!< COMPETRSEL (Bit 20)                       */
#define GTMR_CR2_COMPETRSEL_Msk     (0x7UL << GTMR_CR2_COMPETRSEL_Pos)          /*!< COMPETRSEL (Bitfield-Mask: 0x07)          */
#define GTMR_CR2_COMPETRSEL         GTMR_CR2_COMPETRSEL_Msk
#define GTMR_CR2_COMPETRSEL_0       (0x1UL << GTMR_CR2_COMPETRSEL_Pos)
#define GTMR_CR2_COMPETRSEL_1       (0x2UL << GTMR_CR2_COMPETRSEL_Pos)
#define GTMR_CR2_COMPETRSEL_2       (0x4UL << GTMR_CR2_COMPETRSEL_Pos)
/* ========================================================  SMCTRL  ========================================================= */
#define GTMR_SMCR_SMFSEL_Pos        (0U)                                        /*!< SMFSEL (Bit 0)                            */
#define GTMR_SMCR_SMFSEL_Msk        (0x7UL << GTMR_SMCR_SMFSEL_Pos)             /*!< SMFSEL (Bitfield-Mask: 0x07)              */
#define GTMR_SMCR_SMFSEL            GTMR_SMCR_SMFSEL_Msk
#define GTMR_SMCR_SMFSEL_0          (0x1UL << GTMR_SMCR_SMFSEL_Pos)
#define GTMR_SMCR_SMFSEL_1          (0x2UL << GTMR_SMCR_SMFSEL_Pos)
#define GTMR_SMCR_SMFSEL_2          (0x4UL << GTMR_SMCR_SMFSEL_Pos)

#define GTMR_SMCR_TRGSEL_Pos        (4U)                                        /*!< TRGSEL (Bit 4)                            */
#define GTMR_SMCR_TRGSEL_Msk        (0x7UL << GTMR_SMCR_TRGSEL_Pos)             /*!< TRGSEL (Bitfield-Mask: 0x07)              */
#define GTMR_SMCR_TRGSEL            GTMR_SMCR_TRGSEL_Msk
#define GTMR_SMCR_TRGSEL_0          (0x1UL << GTMR_SMCR_TRGSEL_Pos)
#define GTMR_SMCR_TRGSEL_1          (0x2UL << GTMR_SMCR_TRGSEL_Pos)
#define GTMR_SMCR_TRGSEL_2          (0x4UL << GTMR_SMCR_TRGSEL_Pos)

#define GTMR_SMCR_MSMEN_Pos         (7U)                                        /*!< MSMEN (Bit 7)                             */
#define GTMR_SMCR_MSMEN_Msk         (0x1UL << GTMR_SMCR_MSMEN_Pos)              /*!< MSMEN (Bitfield-Mask: 0x01)               */
#define GTMR_SMCR_MSMEN             GTMR_SMCR_MSMEN_Msk
#define GTMR_SMCR_ETFCFG_Pos        (8U)                                        /*!< ETFCFG (Bit 8)                            */
#define GTMR_SMCR_ETFCFG_Msk        (0xFUL << GTMR_SMCR_ETFCFG_Pos)             /*!< ETFCFG (Bitfield-Mask: 0x0f)              */
#define GTMR_SMCR_ETFCFG            GTMR_SMCR_ETFCFG_Msk
#define GTMR_SMCR_ETFCFG_0          (0x1UL << GTMR_SMCR_ETFCFG_Pos)
#define GTMR_SMCR_ETFCFG_1          (0x2UL << GTMR_SMCR_ETFCFG_Pos)
#define GTMR_SMCR_ETFCFG_2          (0x4UL << GTMR_SMCR_ETFCFG_Pos)
#define GTMR_SMCR_ETFCFG_3          (0x8UL << GTMR_SMCR_ETFCFG_Pos)

#define GTMR_SMCR_ETPCFG_Pos        (12U)                                       /*!< ETPCFG (Bit 12)                           */
#define GTMR_SMCR_ETPCFG_Msk        (0x3UL << GTMR_SMCR_ETPCFG_Pos)             /*!< ETPCFG (Bitfield-Mask: 0x03)              */
#define GTMR_SMCR_ETPCFG            GTMR_SMCR_ETPCFG_Msk
#define GTMR_SMCR_ETPCFG_0          (0x1UL << GTMR_SMCR_ETPCFG_Pos)
#define GTMR_SMCR_ETPCFG_1          (0x2UL << GTMR_SMCR_ETPCFG_Pos)

#define GTMR_SMCR_ECEN_Pos          (14U)                                       /*!< ECEN (Bit 14)                             */
#define GTMR_SMCR_ECEN_Msk          (0x1UL << GTMR_SMCR_ECEN_Pos)               /*!< ECEN (Bitfield-Mask: 0x01)                */
#define GTMR_SMCR_ECEN              GTMR_SMCR_ECEN_Msk
#define GTMR_SMCR_ETPOL_Pos         (15U)                                       /*!< ETPOL (Bit 15)                            */
#define GTMR_SMCR_ETPOL_Msk         (0x1UL << GTMR_SMCR_ETPOL_Pos)              /*!< ETPOL (Bitfield-Mask: 0x01)               */
#define GTMR_SMCR_ETPOL             GTMR_SMCR_ETPOL_Msk
/* =========================================================  DIEN  ========================================================== */
#define GTMR_DIER_UIEN_Pos          (0U)                                        /*!< UIEN (Bit 0)                              */
#define GTMR_DIER_UIEN_Msk          (0x1UL << GTMR_DIER_UIEN_Pos)               /*!< UIEN (Bitfield-Mask: 0x01)                */
#define GTMR_DIER_UIEN              GTMR_DIER_UIEN_Msk
#define GTMR_DIER_CC0IEN_Pos        (1U)                                        /*!< CC0IEN (Bit 1)                            */
#define GTMR_DIER_CC0IEN_Msk        (0x1UL << GTMR_DIER_CC0IEN_Pos)             /*!< CC0IEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC0IEN            GTMR_DIER_CC0IEN_Msk
#define GTMR_DIER_CC1IEN_Pos        (2U)                                        /*!< CC1IEN (Bit 2)                            */
#define GTMR_DIER_CC1IEN_Msk        (0x1UL << GTMR_DIER_CC1IEN_Pos)             /*!< CC1IEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC1IEN            GTMR_DIER_CC1IEN_Msk
#define GTMR_DIER_CC2IEN_Pos        (3U)                                        /*!< CC2IEN (Bit 3)                            */
#define GTMR_DIER_CC2IEN_Msk        (0x1UL << GTMR_DIER_CC2IEN_Pos)             /*!< CC2IEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC2IEN            GTMR_DIER_CC2IEN_Msk
#define GTMR_DIER_CC3IEN_Pos        (4U)                                        /*!< CC3IEN (Bit 4)                            */
#define GTMR_DIER_CC3IEN_Msk        (0x1UL << GTMR_DIER_CC3IEN_Pos)             /*!< CC3IEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC3IEN            GTMR_DIER_CC3IEN_Msk
#define GTMR_DIER_TRGIEN_Pos        (6U)                                        /*!< TRGIEN (Bit 6)                            */
#define GTMR_DIER_TRGIEN_Msk        (0x1UL << GTMR_DIER_TRGIEN_Pos)             /*!< TRGIEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_TRGIEN            GTMR_DIER_TRGIEN_Msk
#define GTMR_DIER_UDIEN_Pos         (8U)                                        /*!< UDIEN (Bit 8)                             */
#define GTMR_DIER_UDIEN_Msk         (0x1UL << GTMR_DIER_UDIEN_Pos)              /*!< UDIEN (Bitfield-Mask: 0x01)               */
#define GTMR_DIER_UDIEN             GTMR_DIER_UDIEN_Msk
#define GTMR_DIER_CC0DEN_Pos        (9U)                                        /*!< CC0DEN (Bit 9)                            */
#define GTMR_DIER_CC0DEN_Msk        (0x1UL << GTMR_DIER_CC0DEN_Pos)             /*!< CC0DEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC0DEN            GTMR_DIER_CC0DEN_Msk
#define GTMR_DIER_CC1DEN_Pos        (10U)                                       /*!< CC1DEN (Bit 10)                           */
#define GTMR_DIER_CC1DEN_Msk        (0x1UL << GTMR_DIER_CC1DEN_Pos)             /*!< CC1DEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC1DEN            GTMR_DIER_CC1DEN_Msk
#define GTMR_DIER_CC2DEN_Pos        (11U)                                       /*!< CC2DEN (Bit 11)                           */
#define GTMR_DIER_CC2DEN_Msk        (0x1UL << GTMR_DIER_CC2DEN_Pos)             /*!< CC2DEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC2DEN            GTMR_DIER_CC2DEN_Msk
#define GTMR_DIER_CC3DEN_Pos        (12U)                                       /*!< CC3DEN (Bit 12)                           */
#define GTMR_DIER_CC3DEN_Msk        (0x1UL << GTMR_DIER_CC3DEN_Pos)             /*!< CC3DEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_CC3DEN            GTMR_DIER_CC3DEN_Msk
#define GTMR_DIER_TRGDEN_Pos        (14U)                                       /*!< TRGDEN (Bit 14)                           */
#define GTMR_DIER_TRGDEN_Msk        (0x1UL << GTMR_DIER_TRGDEN_Pos)             /*!< TRGDEN (Bitfield-Mask: 0x01)              */
#define GTMR_DIER_TRGDEN            GTMR_DIER_TRGDEN_Msk
/* ==========================================================  STS  ========================================================== */
#define GTMR_SR_UIFLG_Pos           (0U)                                        /*!< UIFLG (Bit 0)                             */
#define GTMR_SR_UIFLG_Msk           (0x1UL << GTMR_SR_UIFLG_Pos)                /*!< UIFLG (Bitfield-Mask: 0x01)               */
#define GTMR_SR_UIFLG               GTMR_SR_UIFLG_Msk
#define GTMR_SR_CC0IFLG_Pos         (1U)                                        /*!< CC0IFLG (Bit 1)                           */
#define GTMR_SR_CC0IFLG_Msk         (0x1UL << GTMR_SR_CC0IFLG_Pos)              /*!< CC0IFLG (Bitfield-Mask: 0x01)             */
#define GTMR_SR_CC0IFLG             GTMR_SR_CC0IFLG_Msk
#define GTMR_SR_CC1IFLG_Pos         (2U)                                        /*!< CC1IFLG (Bit 2)                           */
#define GTMR_SR_CC1IFLG_Msk         (0x1UL << GTMR_SR_CC1IFLG_Pos)              /*!< CC1IFLG (Bitfield-Mask: 0x01)             */
#define GTMR_SR_CC1IFLG             GTMR_SR_CC1IFLG_Msk
#define GTMR_SR_CC2IFLG_Pos         (3U)                                        /*!< CC2IFLG (Bit 3)                           */
#define GTMR_SR_CC2IFLG_Msk         (0x1UL << GTMR_SR_CC2IFLG_Pos)              /*!< CC2IFLG (Bitfield-Mask: 0x01)             */
#define GTMR_SR_CC2IFLG             GTMR_SR_CC2IFLG_Msk
#define GTMR_SR_CC3IFLG_Pos         (4U)                                        /*!< CC3IFLG (Bit 4)                           */
#define GTMR_SR_CC3IFLG_Msk         (0x1UL << GTMR_SR_CC3IFLG_Pos)              /*!< CC3IFLG (Bitfield-Mask: 0x01)             */
#define GTMR_SR_CC3IFLG             GTMR_SR_CC3IFLG_Msk
#define GTMR_SR_TRGIFLG_Pos         (6U)                                        /*!< TRGIFLG (Bit 6)                           */
#define GTMR_SR_TRGIFLG_Msk         (0x1UL << GTMR_SR_TRGIFLG_Pos)              /*!< TRGIFLG (Bitfield-Mask: 0x01)             */
#define GTMR_SR_TRGIFLG             GTMR_SR_TRGIFLG_Msk
#define GTMR_SR_CC0RCFLG_Pos        (9U)                                        /*!< CC0RCFLG (Bit 9)                          */
#define GTMR_SR_CC0RCFLG_Msk        (0x1UL << GTMR_SR_CC0RCFLG_Pos)             /*!< CC0RCFLG (Bitfield-Mask: 0x01)            */
#define GTMR_SR_CC0RCFLG            GTMR_SR_CC0RCFLG_Msk
#define GTMR_SR_CC1RCFLG_Pos        (10U)                                       /*!< CC1RCFLG (Bit 10)                         */
#define GTMR_SR_CC1RCFLG_Msk        (0x1UL << GTMR_SR_CC1RCFLG_Pos)             /*!< CC1RCFLG (Bitfield-Mask: 0x01)            */
#define GTMR_SR_CC1RCFLG            GTMR_SR_CC1RCFLG_Msk
#define GTMR_SR_CC2RCFLG_Pos        (11U)                                       /*!< CC2RCFLG (Bit 11)                         */
#define GTMR_SR_CC2RCFLG_Msk        (0x1UL << GTMR_SR_CC2RCFLG_Pos)             /*!< CC2RCFLG (Bitfield-Mask: 0x01)            */
#define GTMR_SR_CC2RCFLG            GTMR_SR_CC2RCFLG_Msk
#define GTMR_SR_CC3RCFLG_Pos        (12U)                                       /*!< CC3RCFLG (Bit 12)                         */
#define GTMR_SR_CC3RCFLG_Msk        (0x1UL << GTMR_SR_CC3RCFLG_Pos)             /*!< CC3RCFLG (Bitfield-Mask: 0x01)            */
#define GTMR_SR_CC3RCFLG            GTMR_SR_CC3RCFLG_Msk
/* ==========================================================  CEG  ========================================================== */
#define GTMR_CEG_UEG_Pos            (0U)                                        /*!< UEG (Bit 0)                               */
#define GTMR_CEG_UEG_Msk            (0x1UL << GTMR_CEG_UEG_Pos)                 /*!< UEG (Bitfield-Mask: 0x01)                 */
#define GTMR_CEG_UEG                GTMR_CEG_UEG_Msk
#define GTMR_CEG_CC0EG_Pos          (1U)                                        /*!< CC0EG (Bit 1)                             */
#define GTMR_CEG_CC0EG_Msk          (0x1UL << GTMR_CEG_CC0EG_Pos)               /*!< CC0EG (Bitfield-Mask: 0x01)               */
#define GTMR_CEG_CC0EG              GTMR_CEG_CC0EG_Msk
#define GTMR_CEG_CC1EG_Pos          (2U)                                        /*!< CC1EG (Bit 2)                             */
#define GTMR_CEG_CC1EG_Msk          (0x1UL << GTMR_CEG_CC1EG_Pos)               /*!< CC1EG (Bitfield-Mask: 0x01)               */
#define GTMR_CEG_CC1EG              GTMR_CEG_CC1EG_Msk
#define GTMR_CEG_CC2EG_Pos          (3U)                                        /*!< CC2EG (Bit 3)                             */
#define GTMR_CEG_CC2EG_Msk          (0x1UL << GTMR_CEG_CC2EG_Pos)               /*!< CC2EG (Bitfield-Mask: 0x01)               */
#define GTMR_CEG_CC2EG              GTMR_CEG_CC2EG_Msk
#define GTMR_CEG_CC3EG_Pos          (4U)                                        /*!< CC3EG (Bit 4)                             */
#define GTMR_CEG_CC3EG_Msk          (0x1UL << GTMR_CEG_CC3EG_Pos)               /*!< CC3EG (Bitfield-Mask: 0x01)               */
#define GTMR_CEG_CC3EG              GTMR_CEG_CC3EG_Msk
#define GTMR_CEG_TEG_Pos            (6U)                                        /*!< TEG (Bit 6)                               */
#define GTMR_CEG_TEG_Msk            (0x1UL << GTMR_CEG_TEG_Pos)                 /*!< TEG (Bitfield-Mask: 0x01)                 */
#define GTMR_CEG_TEG                GTMR_CEG_TEG_Msk
/* =====================================================  CCM1_COPARE  ====================================================== */
#define GTMR_CCM1_CC0SEL_Pos        (0U)                                        /*!< CC0SEL (Bit 0)                        */
#define GTMR_CCM1_CC0SEL_Msk        (0x3UL << GTMR_CCM1_CC0SEL_Pos)             /*!< CC0SEL (Bitfield-Mask: 0x03)          */
#define GTMR_CCM1_CC0SEL            GTMR_CCM1_CC0SEL_Msk
#define GTMR_CCM1_CC0SEL_0          (0x1UL << GTMR_CCM1_CC0SEL_Pos)
#define GTMR_CCM1_CC0SEL_1          (0x2UL << GTMR_CCM1_CC0SEL_Pos)

#define GTMR_CCM1_OC0PEN_Pos        (3U)                                        /*!< OC0PEN (Bit 3)                        */
#define GTMR_CCM1_OC0PEN_Msk        (0x1UL << GTMR_CCM1_OC0PEN_Pos)             /*!< OC0PEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM1_OC0PEN            GTMR_CCM1_OC0PEN_Msk
#define GTMR_CCM1_OC0MOD_Pos        (4U)                                        /*!< OC0MOD (Bit 4)                        */
#define GTMR_CCM1_OC0MOD_Msk        (0x7UL << GTMR_CCM1_OC0MOD_Pos)             /*!< OC0MOD (Bitfield-Mask: 0x07)          */
#define GTMR_CCM1_OC0MOD            GTMR_CCM1_OC0MOD_Msk
#define GTMR_CCM1_OC0MOD_0          (0x1UL << GTMR_CCM1_OC0MOD_Pos)
#define GTMR_CCM1_OC0MOD_1          (0x2UL << GTMR_CCM1_OC0MOD_Pos)
#define GTMR_CCM1_OC0MOD_2          (0x4UL << GTMR_CCM1_OC0MOD_Pos)

#define GTMR_CCM1_OC0CEN_Pos        (7U)                                        /*!< OC0CEN (Bit 7)                        */
#define GTMR_CCM1_OC0CEN_Msk        (0x1UL << GTMR_CCM1_OC0CEN_Pos)             /*!< OC0CEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM1_OC0CEN            GTMR_CCM1_OC0CEN_Msk
#define GTMR_CCM1_CC1SEL_Pos        (8U)                                        /*!< CC1SEL (Bit 8)                        */
#define GTMR_CCM1_CC1SEL_Msk        (0x3UL << GTMR_CCM1_CC1SEL_Pos)             /*!< CC1SEL (Bitfield-Mask: 0x03)          */
#define GTMR_CCM1_CC1SEL            GTMR_CCM1_CC1SEL_Msk
#define GTMR_CCM1_CC1SEL_0          (0x1UL << GTMR_CCM1_CC1SEL_Pos)
#define GTMR_CCM1_CC1SEL_1          (0x2UL << GTMR_CCM1_CC1SEL_Pos)

#define GTMR_CCM1_OC1PEN_Pos        (11U)                                       /*!< OC1PEN (Bit 11)                       */
#define GTMR_CCM1_OC1PEN_Msk        (0x1UL << GTMR_CCM1_OC1PEN_Pos)             /*!< OC1PEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM1_OC1PEN            GTMR_CCM1_OC1PEN_Msk
#define GTMR_CCM1_OC1MOD_Pos        (12U)                                       /*!< OC1MOD (Bit 12)                       */
#define GTMR_CCM1_OC1MOD_Msk        (0x7UL << GTMR_CCM1_OC1MOD_Pos)             /*!< OC1MOD (Bitfield-Mask: 0x07)          */
#define GTMR_CCM1_OC1MOD            GTMR_CCM1_OC1MOD_Msk
#define GTMR_CCM1_OC1MOD_0          (0x1UL << GTMR_CCM1_OC1MOD_Pos)
#define GTMR_CCM1_OC1MOD_1          (0x2UL << GTMR_CCM1_OC1MOD_Pos)
#define GTMR_CCM1_OC1MOD_2          (0x4UL << GTMR_CCM1_OC1MOD_Pos)

#define GTMR_CCM1_OC1CEN_Pos        (15U)                                       /*!< OC1CEN (Bit 15)                       */
#define GTMR_CCM1_OC1CEN_Msk        (0x1UL << GTMR_CCM1_OC1CEN_Pos)             /*!< OC1CEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM1_OC1CEN            GTMR_CCM1_OC1CEN_Msk
/* =====================================================  CCM1_CAPTURE  ====================================================== */
#define GTMR_CCM1_IC0PSC_Pos        (2U)                                        /*!< IC0PSC (Bit 2)                        */
#define GTMR_CCM1_IC0PSC_Msk        (0x3UL << GTMR_CCM1_IC0PSC_Pos)             /*!< IC0PSC (Bitfield-Mask: 0x03)          */
#define GTMR_CCM1_IC0PSC            GTMR_CCM1_IC0PSC_Msk
#define GTMR_CCM1_IC0PSC_0          (0x1UL << GTMR_CCM1_IC0PSC_Pos)
#define GTMR_CCM1_IC0PSC_1          (0x2UL << GTMR_CCM1_IC0PSC_Pos)

#define GTMR_CCM1_IC0F_Pos          (4U)                                        /*!< IC0F (Bit 4)                          */
#define GTMR_CCM1_IC0F_Msk          (0xFUL << GTMR_CCM1_IC0F_Pos)               /*!< IC0F (Bitfield-Mask: 0x0f)            */
#define GTMR_CCM1_IC0F              GTMR_CCM1_IC0F_Msk
#define GTMR_CCM1_IC0F_0            (0x1UL << GTMR_CCM1_IC0F_Pos)
#define GTMR_CCM1_IC0F_1            (0x2UL << GTMR_CCM1_IC0F_Pos)
#define GTMR_CCM1_IC0F_2            (0x4UL << GTMR_CCM1_IC0F_Pos)
#define GTMR_CCM1_IC0F_3            (0x8UL << GTMR_CCM1_IC0F_Pos)

#define GTMR_CCM1_IC1PSC_Pos        (10U)                                       /*!< IC1PSC (Bit 10)                       */
#define GTMR_CCM1_IC1PSC_Msk        (0x3UL << GTMR_CCM1_IC1PSC_Pos)             /*!< IC1PSC (Bitfield-Mask: 0x03)          */
#define GTMR_CCM1_IC1PSC            GTMR_CCM1_IC1PSC_Msk
#define GTMR_CCM1_IC1PSC_0          (0x1UL << GTMR_CCM1_IC1PSC_Pos)
#define GTMR_CCM1_IC1PSC_1          (0x2UL << GTMR_CCM1_IC1PSC_Pos)

#define GTMR_CCM1_IC1F_Pos          (12U)                                       /*!< IC1F (Bit 12)                         */
#define GTMR_CCM1_IC1F_Msk          (0xFUL << GTMR_CCM1_IC1F_Pos)               /*!< IC1F (Bitfield-Mask: 0x0f)            */
#define GTMR_CCM1_IC1F              GTMR_CCM1_IC1F_Msk
#define GTMR_CCM1_IC1F_0            (0x1UL << GTMR_CCM1_IC1F_Pos)
#define GTMR_CCM1_IC1F_1            (0x2UL << GTMR_CCM1_IC1F_Pos)
#define GTMR_CCM1_IC1F_2            (0x4UL << GTMR_CCM1_IC1F_Pos)
#define GTMR_CCM1_IC1F_3            (0x8UL << GTMR_CCM1_IC1F_Pos)
/* =====================================================  CCM2_COMPARE  ====================================================== */
#define GTMR_CCM2_CC2SEL_Pos        (0U)                                        /*!< CC2SEL (Bit 0)                        */
#define GTMR_CCM2_CC2SEL_Msk        (0x3UL << GTMR_CCM2_CC2SEL_Pos)             /*!< CC2SEL (Bitfield-Mask: 0x03)          */
#define GTMR_CCM2_CC2SEL            GTMR_CCM2_CC2SEL_Msk
#define GTMR_CCM2_OC2PEN_Pos        (3U)                                        /*!< OC2PEN (Bit 3)                        */
#define GTMR_CCM2_OC2PEN_Msk        (0x1UL << GTMR_CCM2_OC2PEN_Pos)             /*!< OC2PEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM2_OC2PEN            GTMR_CCM2_OC2PEN_Msk
#define GTMR_CCM2_OC2MOD_Pos        (4U)                                        /*!< OC2MOD (Bit 4)                        */
#define GTMR_CCM2_OC2MOD_Msk        (0x7UL << GTMR_CCM2_OC2MOD_Pos)             /*!< OC2MOD (Bitfield-Mask: 0x07)          */
#define GTMR_CCM2_OC2MOD            GTMR_CCM2_OC2MOD_Msk
#define GTMR_CCM2_OC2CEN_Pos        (7U)                                        /*!< OC2CEN (Bit 7)                        */
#define GTMR_CCM2_OC2CEN_Msk        (0x1UL << GTMR_CCM2_OC2CEN_Pos)             /*!< OC2CEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM2_OC2CEN            GTMR_CCM2_OC2CEN_Msk
#define GTMR_CCM2_CC3SEL_Pos        (8U)                                        /*!< CC3SEL (Bit 8)                        */
#define GTMR_CCM2_CC3SEL_Msk        (0x3UL << GTMR_CCM2_CC3SEL_Pos)             /*!< CC3SEL (Bitfield-Mask: 0x03)          */
#define GTMR_CCM2_CC3SEL            GTMR_CCM2_CC3SEL_Msk
#define GTMR_CCM2_OC3PEN_Pos        (11U)                                       /*!< OC3PEN (Bit 11)                       */
#define GTMR_CCM2_OC3PEN_Msk        (0x1UL << GTMR_CCM2_OC3PEN_Pos)             /*!< OC3PEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM2_OC3PEN            GTMR_CCM2_OC3PEN_Msk
#define GTMR_CCM2_OC3MOD_Pos        (12U)                                       /*!< OC3MOD (Bit 12)                       */
#define GTMR_CCM2_OC3MOD_Msk        (0x7UL << GTMR_CCM2_OC3MOD_Pos)             /*!< OC3MOD (Bitfield-Mask: 0x07)          */
#define GTMR_CCM2_OC3MOD            GTMR_CCM2_OC3MOD_Msk
#define GTMR_CCM2_OC3CEN_Pos        (15U)                                       /*!< OC3CEN (Bit 15)                       */
#define GTMR_CCM2_OC3CEN_Msk        (0x1UL << GTMR_CCM2_OC3CEN_Pos)             /*!< OC3CEN (Bitfield-Mask: 0x01)          */
#define GTMR_CCM2_OC3CEN            GTMR_CCM2_OC3CEN_Msk
/* =====================================================  CCM2_CAPTRUE  ====================================================== */
#define GTMR_CCM2_IC2PSC_Pos        (2U)                                        /*!< IC2PSC (Bit 2)                        */
#define GTMR_CCM2_IC2PSC_Msk        (0x3UL << GTMR_CCM2_IC2PSC_Pos)             /*!< IC2PSC (Bitfield-Mask: 0x03)          */
#define GTMR_CCM2_IC2PSC            GTMR_CCM2_IC2PSC_Msk
#define GTMR_CCM2_IC2F_Pos          (4U)                                        /*!< IC2F (Bit 4)                          */
#define GTMR_CCM2_IC2F_Msk          (0xFUL << GTMR_CCM2_IC2F_Pos)               /*!< IC2F (Bitfield-Mask: 0x0f)            */
#define GTMR_CCM2_IC2F              GTMR_CCM2_IC2F_Msk
#define GTMR_CCM2_IC3PSC_Pos        (10U)                                       /*!< IC3PSC (Bit 10)                       */
#define GTMR_CCM2_IC3PSC_Msk        (0x3UL << GTMR_CCM2_IC3PSC_Pos)             /*!< IC3PSC (Bitfield-Mask: 0x03)          */
#define GTMR_CCM2_IC3PSC            GTMR_CCM2_IC3PSC_Msk
#define GTMR_CCM2_IC3F_Pos          (12U)                                       /*!< IC3F (Bit 12)                         */
#define GTMR_CCM2_IC3F_Msk          (0xFUL << GTMR_CCM2_IC3F_Pos)               /*!< IC3F (Bitfield-Mask: 0x0f)            */
#define GTMR_CCM2_IC3F              GTMR_CCM2_IC3F_Msk
/* =========================================================  CCEN  ========================================================== */
#define GTMR_CCEN_CC0EN_Pos         (0U)                                        /*!< CC0EN (Bit 0)                             */
#define GTMR_CCEN_CC0EN_Msk         (0x1UL << GTMR_CCEN_CC0EN_Pos)              /*!< CC0EN (Bitfield-Mask: 0x01)               */
#define GTMR_CCEN_CC0EN             GTMR_CCEN_CC0EN_Msk
#define GTMR_CCEN_CC0POL_Pos        (1U)                                        /*!< CC0POL (Bit 1)                            */
#define GTMR_CCEN_CC0POL_Msk        (0x1UL << GTMR_CCEN_CC0POL_Pos)             /*!< CC0POL (Bitfield-Mask: 0x01)              */
#define GTMR_CCEN_CC0POL            GTMR_CCEN_CC0POL_Msk
#define GTMR_CCEN_CC1EN_Pos         (4U)                                        /*!< CC1EN (Bit 4)                             */
#define GTMR_CCEN_CC1EN_Msk         (0x1UL << GTMR_CCEN_CC1EN_Pos)              /*!< CC1EN (Bitfield-Mask: 0x01)               */
#define GTMR_CCEN_CC1EN             GTMR_CCEN_CC1EN_Msk
#define GTMR_CCEN_CC1POL_Pos        (5U)                                        /*!< CC1POL (Bit 5)                            */
#define GTMR_CCEN_CC1POL_Msk        (0x1UL << GTMR_CCEN_CC1POL_Pos)             /*!< CC1POL (Bitfield-Mask: 0x01)              */
#define GTMR_CCEN_CC1POL            GTMR_CCEN_CC1POL_Msk
#define GTMR_CCEN_CC2EN_Pos         (8U)                                        /*!< CC2EN (Bit 8)                             */
#define GTMR_CCEN_CC2EN_Msk         (0x1UL << GTMR_CCEN_CC2EN_Pos)              /*!< CC2EN (Bitfield-Mask: 0x01)               */
#define GTMR_CCEN_CC2EN             GTMR_CCEN_CC2EN_Msk
#define GTMR_CCEN_CC2POL_Pos        (9U)                                        /*!< CC2POL (Bit 9)                            */
#define GTMR_CCEN_CC2POL_Msk        (0x1UL << GTMR_CCEN_CC2POL_Pos)             /*!< CC2POL (Bitfield-Mask: 0x01)              */
#define GTMR_CCEN_CC2POL            GTMR_CCEN_CC2POL_Msk
#define GTMR_CCEN_CC3EN_Pos         (12U)                                       /*!< CC3EN (Bit 12)                            */
#define GTMR_CCEN_CC3EN_Msk         (0x1UL << GTMR_CCEN_CC3EN_Pos)              /*!< CC3EN (Bitfield-Mask: 0x01)               */
#define GTMR_CCEN_CC3EN             GTMR_CCEN_CC3EN_Msk
#define GTMR_CCEN_CC3POL_Pos        (13U)                                       /*!< CC3POL (Bit 13)                           */
#define GTMR_CCEN_CC3POL_Msk        (0x1UL << GTMR_CCEN_CC3POL_Pos)             /*!< CC3POL (Bitfield-Mask: 0x01)              */
#define GTMR_CCEN_CC3POL            GTMR_CCEN_CC3POL_Msk
/* ==========================================================  CNT  ========================================================== */
#define GTMR_CNT_CNT_Pos            (0U)                                        /*!< CNT (Bit 0)                               */
#define GTMR_CNT_CNT_Msk            (0xFFFFFFFFUL << GTMR_CNT_CNT_Pos)          /*!< CNT (Bitfield-Mask: 0xffffffff)           */
#define GTMR_CNT_CNT                GTMR_CNT_CNT_Msk
/* ==========================================================  PSC  ========================================================== */
#define GTMR_PSC_PSC_Pos            (0U)                                        /*!< PSC (Bit 0)                               */
#define GTMR_PSC_PSC_Msk            (0xFFFFUL << GTMR_PSC_PSC_Pos)              /*!< PSC (Bitfield-Mask: 0xffff)               */
#define GTMR_PSC_PSC                GTMR_PSC_PSC_Msk
/* ========================================================  AUTORLD  ======================================================== */
#define GTMR_AUTORLD_AUTORLD_Pos    (0U)                                        /*!< AUTORLD (Bit 0)                     */
#define GTMR_AUTORLD_AUTORLD_Msk    (0xFFFFFFFFUL << GTMR_AUTORLD_AUTORLD_Pos)  /*!< AUTORLD (Bitfield-Mask: 0xffffffff) */
#define GTMR_AUTORLD_AUTORLD        GTMR_AUTORLD_AUTORLD_Msk
/* ==========================================================  CC0  ========================================================== */
#define GTMR_CC0_CC0_Pos            (0U)                                        /*!< CC0 (Bit 0)                               */
#define GTMR_CC0_CC0_Msk            (0xFFFFFFFFUL << GTMR_CC0_CC0_Pos)          /*!< CC0 (Bitfield-Mask: 0xffffffff)           */
#define GTMR_CC0_CC0                GTMR_CC0_CC0_Msk
/* ==========================================================  CC1  ========================================================== */
#define GTMR_CC1_CC1_Pos            (0U)                                        /*!< CC1 (Bit 0)                               */
#define GTMR_CC1_CC1_Msk            (0xFFFFFFFFUL << GTMR_CC1_CC1_Pos)          /*!< CC1 (Bitfield-Mask: 0xffffffff)           */
#define GTMR_CC1_CC1                GTMR_CC1_CC1_Msk
/* ==========================================================  CC2  ========================================================== */
#define GTMR_CC2_CC2_Pos            (0U)                                        /*!< CC2 (Bit 0)                               */
#define GTMR_CC2_CC2_Msk            (0xFFFFFFFFUL << GTMR_CC2_CC2_Pos)          /*!< CC2 (Bitfield-Mask: 0xffffffff)           */
#define GTMR_CC2_CC2                (GTMR_CC2_CC2_Msk)
/* ==========================================================  CC3  ========================================================== */
#define GTMR_CC3_CC3_Pos            (0U)                                        /*!< CC3 (Bit 0)                               */
#define GTMR_CC3_CC3_Msk            (0xFFFFFFFFUL << GTMR_CC3_CC3_Pos)          /*!< CC3 (Bitfield-Mask: 0xffffffff)           */
#define GTMR_CC3_CC3                (GTMR_CC3_CC3_Msk)


/* =========================================================================================================================== */
/* ================                                          BTMR                                          ================ */
/* =========================================================================================================================== */

/* =========================================================  CTRL1  ========================================================= */
#define BTMR_CR1_CNTEN_Pos          (0U)                                        /*!< CNTEN (Bit 0)                         */
#define BTMR_CR1_CNTEN_Msk          (0x1UL << BTMR_CR1_CNTEN_Pos)               /*!< CNTEN (Bitfield-Mask: 0x01)           */
#define BTMR_CR1_CNTEN              BTMR_CR1_CNTEN_Msk
#define BTMR_CR1_CNTDIR_Pos         (1U)                                        /*!< DIR (Bit 1)                           */
#define BTMR_CR1_CNTDIR_Msk         (0x1UL << BTMR_CR1_CNTDIR_Pos)              /*!< DIR (Bitfield-Mask: 0x01)             */
#define BTMR_CR1_CNTDIR             BTMR_CR1_CNTDIR_Msk
#define BTMR_CR1_SPMEN_Pos          (2U)                                        /*!< OPMEM (Bit 2)                         */
#define BTMR_CR1_SPMEN_Msk          (0x1UL << BTMR_CR1_SPMEN_Pos)               /*!< OPMEM (Bitfield-Mask: 0x01)           */
#define BTMR_CR1_SPMEN              BTMR_CR1_SPMEN_Msk
#define BTMR_CR1_UDISEN_Pos         (3U)                                        /*!< UDISEN (Bit 3)                          */
#define BTMR_CR1_UDISEN_Msk         (0x1UL << BTMR_CR1_UDISEN_Pos)              /*!< UDISEN (Bitfield-Mask: 0x01)            */
#define BTMR_CR1_UDISEN             BTMR_CR1_UDISEN_Msk
#define BTMR_CR1_CAMSEL_Pos         (4U)                                        /*!< CMS (Bit 4)                           */
#define BTMR_CR1_CAMSEL_Msk         (0x3UL << BTMR_CR1_CAMSEL_Pos)              /*!< CMS (Bitfield-Mask: 0x03)             */
#define BTMR_CR1_CAMSEL             BTMR_CR1_CAMSEL_Msk
#define BTMR_CR1_CAMSEL_0           (0x1UL << BTMR_CR1_CAMSEL_Pos)
#define BTMR_CR1_CAMSEL_1           (0x2UL << BTMR_CR1_CAMSEL_Pos)

#define BTMR_CR1_ARPEN_Pos          (6U)                                        /*!< ALD_BUFF_EN (Bit 6)                   */
#define BTMR_CR1_ARPEN_Msk          (0x1UL << BTMR_CR1_ARPEN_Pos)               /*!< ALD_BUFF_EN (Bitfield-Mask: 0x01)     */
#define BTMR_CR1_ARPEN              BTMR_CR1_ARPEN_Msk
#define BTMR_CR1_PRPEN_Pos          (7U)                                        /*!< PSC_BUFF_EN (Bit 7)                   */
#define BTMR_CR1_PRPEN_Msk          (0x1UL << BTMR_CR1_PRPEN_Pos)               /*!< PSC_BUFF_EN (Bitfield-Mask: 0x01)     */
#define BTMR_CR1_PRPEN              BTMR_CR1_PRPEN_Msk
/* ========================================================  CCXCR1  ========================================================= */
#define BTMR_CCXCR1_CC0EN_Pos       (0U)                                        /*!< CC0EN (Bit 0)                         */
#define BTMR_CCXCR1_CC0EN_Msk       (0x1UL << BTMR_CCXCR1_CC0EN_Pos)            /*!< CC0EN (Bitfield-Mask: 0x01)           */
#define BTMR_CCXCR1_CC0EN           BTMR_CCXCR1_CC0EN_Msk
#define BTMR_CCXCR1_CC0POL_Pos      (1U)                                        /*!< CC0POL (Bit 1)                        */
#define BTMR_CCXCR1_CC0POL_Msk      (0x1UL << BTMR_CCXCR1_CC0POL_Pos)           /*!< CC0POL (Bitfield-Mask: 0x01)          */
#define BTMR_CCXCR1_CC0POL          BTMR_CCXCR1_CC0POL_Msk
#define BTMR_CCXCR1_CC0EDGESEL_Pos  (2U)                                        /*!< CC0_EDGE_SEL (Bit 2)                  */
#define BTMR_CCXCR1_CC0EDGESEL_Msk  (0x3UL << BTMR_CCXCR1_CC0EDGESEL_Pos)       /*!< CC0_EDGE_SEL (Bitfield-Mask: 0x03)    */
#define BTMR_CCXCR1_CC0EDGESEL      BTMR_CCXCR1_CC0EDGESEL_Msk
#define BTMR_CCXCR1_CC0EDGESEL_0    (0x1UL << BTMR_CCXCR1_CC0EDGESEL_Pos)
#define BTMR_CCXCR1_CC0EDGESEL_1    (0x2UL << BTMR_CCXCR1_CC0EDGESEL_Pos)

#define BTMR_CCXCR1_IC0F_Pos        (4U)                                        /*!< IC0F (Bit 4)                          */
#define BTMR_CCXCR1_IC0F_Msk        (0x7UL << BTMR_CCXCR1_IC0F_Pos)             /*!< IC0F (Bitfield-Mask: 0x07)            */
#define BTMR_CCXCR1_IC0F            BTMR_CCXCR1_IC0F_Msk
#define BTMR_CCXCR1_IC0F_0          (0x1UL << BTMR_CCXCR1_IC0F_Pos)
#define BTMR_CCXCR1_IC0F_1          (0x2UL << BTMR_CCXCR1_IC0F_Pos)
#define BTMR_CCXCR1_IC0F_2          (0x4UL << BTMR_CCXCR1_IC0F_Pos)

#define BTMR_CCXCR1_OC0MOD_Pos      (7U)                                        /*!< OC0MOD (Bit 7)                        */
#define BTMR_CCXCR1_OC0MOD_Msk      (0x7UL << BTMR_CCXCR1_OC0MOD_Pos)           /*!< OC0MOD (Bitfield-Mask: 0x07)          */
#define BTMR_CCXCR1_OC0MOD          BTMR_CCXCR1_OC0MOD_Msk
#define BTMR_CCXCR1_OC0MOD_0        (0x1UL << BTMR_CCXCR1_OC0MOD_Pos)
#define BTMR_CCXCR1_OC0MOD_1        (0x2UL << BTMR_CCXCR1_OC0MOD_Pos)
#define BTMR_CCXCR1_OC0MOD_2        (0x4UL << BTMR_CCXCR1_OC0MOD_Pos)
/* ========================================================  CCXCR2  ========================================================= */
#define BTMR_CCXCR2_CC0RPEN_Pos     (0U)                                        /*!< CC0_BUFF_EN (Bit 0)                   */
#define BTMR_CCXCR2_CC0RPEN_Msk     (0x1UL << BTMR_CCXCR2_CC0RPEN_Pos)          /*!< CC0_BUFF_EN (Bitfield-Mask: 0x01)     */
#define BTMR_CCXCR2_CC0RPEN         BTMR_CCXCR2_CC0RPEN_Msk
#define BTMR_CCXCR2_CC0SEL_Pos      (1U)                                        /*!< CC0_IOSEL (Bit 1)                     */
#define BTMR_CCXCR2_CC0SEL_Msk      (0x1UL << BTMR_CCXCR2_CC0SEL_Pos)           /*!< CC0_IOSEL (Bitfield-Mask: 0x01)       */
#define BTMR_CCXCR2_CC0SEL          BTMR_CCXCR2_CC0SEL_Msk
/* ==========================================================  CEG  ========================================================== */
#define BTMR_CEG_UEG_Pos            (0U)                                        /*!< UEG (Bit 0)                           */
#define BTMR_CEG_UEG_Msk            (0x1UL << BTMR_CEG_UEG_Pos)                 /*!< UEG (Bitfield-Mask: 0x01)             */
#define BTMR_CEG_UEG                BTMR_CEG_UEG_Msk
#define BTMR_CEG_CC0EG_Pos          (1U)                                        /*!< CC0EG (Bit 1)                         */
#define BTMR_CEG_CC0EG_Msk          (0x1UL << BTMR_CEG_CC0EG_Pos)               /*!< CC0EG (Bitfield-Mask: 0x01)           */
#define BTMR_CEG_CC0EG              BTMR_CEG_CC0EG_Msk
/* ==========================================================  IEN  ========================================================== */
#define BTMR_IER_UIEN_Pos           (0U)                                        /*!< UIEN (Bit 0)                          */
#define BTMR_IER_UIEN_Msk           (0x1UL << BTMR_IER_UIEN_Pos)                /*!< UIEN (Bitfield-Mask: 0x01)            */
#define BTMR_IER_UIEN               BTMR_IER_UIEN_Msk
#define BTMR_IER_CC0IEN_Pos         (1U)                                        /*!< CC0IEN (Bit 1)                        */
#define BTMR_IER_CC0IEN_Msk         (0x1UL << BTMR_IER_CC0IEN_Pos)              /*!< CC0IEN (Bitfield-Mask: 0x01)          */
#define BTMR_IER_CC0IEN             BTMR_IER_CC0IEN_Msk
/* ==========================================================  STS  ========================================================== */
#define BTMR_SR_CC0IFLG_Pos         (0U)                                        /*!< CC0IFLG (Bit 0)                       */
#define BTMR_SR_CC0IFLG_Msk         (0x1UL << BTMR_SR_CC0IFLG_Pos)              /*!< CC0IFLG (Bitfield-Mask: 0x01)         */
#define BTMR_SR_CC0IFLG             BTMR_SR_CC0IFLG_Msk
#define BTMR_SR_UIFLG_Pos           (1U)                                        /*!< UIFLG (Bit 1)                         */
#define BTMR_SR_UIFLG_Msk           (0x1UL << BTMR_SR_UIFLG_Pos)                /*!< UIFLG (Bitfield-Mask: 0x01)           */
#define BTMR_SR_UIFLG               BTMR_SR_UIFLG_Msk
#define BTMR_SR_CC0RCFLG_Pos        (2U)                                        /*!< CC0RCFLG (Bit 2)                      */
#define BTMR_SR_CC0RCFLG_Msk        (0x1UL << BTMR_SR_CC0RCFLG_Pos)             /*!< CC0RCFLG (Bitfield-Mask: 0x01)        */
#define BTMR_SR_CC0RCFLG            BTMR_SR_CC0RCFLG_Msk
/* ==========================================================  CNT  ========================================================== */
#define BTMR_CNT_CNT_Pos            (0U)                                        /*!< CNT (Bit 0)                           */
#define BTMR_CNT_CNT_Msk            (0xFFFFUL << BTMR_CNT_CNT_Pos)              /*!< CNT (Bitfield-Mask: 0xffff)           */
#define BTMR_CNT_CNT                BTMR_CNT_CNT_Msk
/* ==========================================================  PSC  ========================================================== */
#define BTMR_PSC_PSC_Pos            (0U)                                        /*!< PSC (Bit 0)                           */
#define BTMR_PSC_PSC_Msk            (0xFFFFUL << BTMR_PSC_PSC_Pos)              /*!< PSC (Bitfield-Mask: 0xffff)           */
#define BTMR_PSC_PSC                BTMR_PSC_PSC_Msk
/* ========================================================  AUTORLD  ======================================================== */
#define BTMR_AUTORLD_AUTORLD_Pos    (0U)                                        /*!< AUTORLD (Bit 0)                       */
#define BTMR_AUTORLD_AUTORLD_Msk    (0xFFFFUL << BTMR_AUTORLD_AUTORLD_Pos)      /*!< AUTORLD (Bitfield-Mask: 0xffff)       */
#define BTMR_AUTORLD_AUTORLD        BTMR_AUTORLD_AUTORLD_Msk
/* ==========================================================  CC0  ========================================================== */
#define BTMR_CC0_CC0_Pos            (0U)                                        /*!< CC0 (Bit 0)                           */
#define BTMR_CC0_CC0_Msk            (0xFFFFUL << BTMR_CC0_CC0_Pos)              /*!< CC0 (Bitfield-Mask: 0xffff)           */
#define BTMR_CC0_CC0                BTMR_CC0_CC0_Msk


/* =========================================================================================================================== */
/* ================                                          LPTMR                                            ================ */
/* =========================================================================================================================== */

/* ==========================================================  CR  =========================================================== */
#define LPTMR_CR_CNTEN_Pos          (0U)                                        /*!< CNTEN (Bit 0)                                */
#define LPTMR_CR_CNTEN_Msk          (0x1UL << LPTMR_CR_CNTEN_Pos)               /*!< CNTEN (Bitfield-Mask: 0x01)                  */
#define LPTMR_CR_CNTEN              LPTMR_CR_CNTEN_Msk
#define LPTMR_CR_LPTIEN_Pos         (1U)                                        /*!< IRQ EN (Bit 1)                            */
#define LPTMR_CR_LPTIEN_Msk         (0x1UL << LPTMR_CR_LPTIEN_Pos)              /*!< IRQ EN (Bitfield-Mask: 0x01)              */
#define LPTMR_CR_LPTIEN             LPTMR_CR_LPTIEN_Msk
/* ==========================================================  PS  =========================================================== */
#define LPTMR_PSC_PSC_Pos           (0U)                                        /*!< PS (Bit 0)                                */
#define LPTMR_PSC_PSC_Msk           (0xFFFFUL << LPTMR_PSC_PSC_Pos)             /*!< PS (Bitfield-Mask: 0xffff)                */
#define LPTMR_PSC_PSC               LPTMR_PSC_PSC_Msk
/* ==========================================================  RV  =========================================================== */
#define LPTMR_WKVAL_WKVAL_Pos       (0U)                                        /*!< RV (Bit 0)                                */
#define LPTMR_WKVAL_WKVAL_Msk       (0xFFFFUL << LPTMR_WKVAL_WKVAL_Pos)         /*!< RV (Bitfield-Mask: 0xffff)                */
#define LPTMR_WKVAL_WKVAL           LPTMR_WKVAL_WKVAL_Msk
/* ==========================================================  SR  =========================================================== */
#define LPTMR_SR_WKFLG_Pos          (0U)                                        /*!< WK_STS (Bit 0)                            */
#define LPTMR_SR_WKFLG_Msk          (0x1UL << LPTMR_SR_WKFLG_Pos)               /*!< WK_STS (Bitfield-Mask: 0x01)              */
#define LPTMR_SR_WKFLG              LPTMR_SR_WKFLG_Msk
#define LPTMR_SR_PVU_Pos            (1U)                                        /*!< PU (Bit 1)                                */
#define LPTMR_SR_PVU_Msk            (0x1UL << LPTMR_SR_PVU_Pos)                 /*!< PU (Bitfield-Mask: 0x01)                  */
#define LPTMR_SR_PVU                LPTMR_SR_PVU_Msk
#define LPTMR_SR_RVU_Pos            (2U)                                        /*!< RU (Bit 2)                                */
#define LPTMR_SR_RVU_Msk            (0x1UL << LPTMR_SR_RVU_Pos)                 /*!< RU (Bitfield-Mask: 0x01)                  */
#define LPTMR_SR_RVU                LPTMR_SR_RVU_Msk
/* ==========================================================  CNT  ========================================================== */
#define LPTMR_CNT_CNT_Pos           (0U)                                        /*!< CNT (Bit 0)                               */
#define LPTMR_CNT_CNT_Msk           (0xFFFFUL << LPTMR_CNT_CNT_Pos)             /*!< CNT (Bitfield-Mask: 0xffff)               */
#define LPTMR_CNT_CNT               LPTMR_CNT_CNT_Msk


/* =========================================================================================================================== */
/* ================                                           IWDT                                            ================ */
/* =========================================================================================================================== */

/* ==========================================================  KEY  ========================================================== */
#define IWDT_KEY_KEY_Pos            (0U)                                        /*!< KEY (Bit 0)                       */
#define IWDT_KEY_KEY_Msk            (0xFFFFUL << IWDT_KEY_KEY_Pos)              /*!< KEY (Bitfield-Mask: 0xffff)       */
#define IWDT_KEY_KEY                IWDT_KEY_KEY_Msk                            /*!<Key value (write only, read 0000h) */
/* ==========================================================  PR  =========================================================== */
#define IWDT_PSC_PSC_Pos            (0U)                                        /*!< PS (Bit 0)                        */
#define IWDT_PSC_PSC_0              (0x1UL << IWDT_PSC_PSC_Pos)                 /*!< 0x01 */
#define IWDT_PSC_PSC_1              (0x2UL << IWDT_PSC_PSC_Pos)                 /*!< 0x02 */
#define IWDT_PSC_PSC_2              (0x4UL << IWDT_PSC_PSC_Pos)                 /*!< 0x04 */
#define IWDT_PSC_PSC_Msk            (0x7UL << IWDT_PSC_PSC_Pos)                 /*!< PS (Bitfield-Mask: 0x07)          */
#define IWDT_PSC_PSC                IWDT_PSC_PSC_Msk                            /*!<PR[2:0] (Prescaler divider)        */

/* ==========================================================  RLR  ========================================================== */
#define IWDT_RLR_RLR_Pos            (0U)                                        /*!< RLR (Bit 0)                       */
#define IWDT_RLR_RLR_Msk            (0xFFFUL << IWDT_RLR_RLR_Pos)               /*!< RLR (Bitfield-Mask: 0xfff)        */
#define IWDT_RLR_RLR                IWDT_RLR_RLR_Msk                            /*!<Watchdog counter reload value      */
/* ==========================================================  SR  =========================================================== */
#define IWDT_SR_PVU_Pos             (0U)                                        /*!< PVU (Bit 0)                       */
#define IWDT_SR_PVU_Msk             (0x1UL << IWDT_SR_PVU_Pos)                  /*!< PVU (Bitfield-Mask: 0x01)         */
#define IWDT_SR_PVU                 IWDT_SR_PVU_Msk                             /*!< Watchdog prescaler value update   */
#define IWDT_SR_RVU_Pos             (1U)                                        /*!< RVU (Bit 1)                       */
#define IWDT_SR_RVU_Msk             (0x1UL << IWDT_SR_RVU_Pos)                  /*!< RVU (Bitfield-Mask: 0x01)         */
#define IWDT_SR_RVU                 IWDT_SR_RVU_Msk                             /*!<Watchdog counter reload value update*/


/* =========================================================================================================================== */
/* ================                                           WWDT                                            ================ */
/* =========================================================================================================================== */

/* ==========================================================  CR  =========================================================== */
#define WWDT_CR_TOUT_Pos            (0U)                                        /*!< TOUT (Bit 0)                         */
#define WWDT_CR_TOUT_Msk            (0x7fUL << WWDT_CR_TOUT_Pos)                /*!< TOUT (Bitfield-Mask: 0x7f)            */
#define WWDT_CR_TOUT                WWDT_CR_TOUT_Msk
#define WWDT_CR_WWDTEN_Pos          (7U)                                        /*!< WWDT_EN (Bit 7)                    */
#define WWDT_CR_WWDTEN_Msk          (0x1UL << WWDT_CR_WWDTEN_Pos)               /*!< WWDT_EN (Bitfield-Mask: 0x01)      */
#define WWDT_CR_WWDTEN              WWDT_CR_WWDTEN_Msk
/* ==========================================================  CFG  ========================================================== */
#define WWDT_CFG_WINDOW_Pos         (0U)                                        /*!< WOMDOW (Bit 0)                           */
#define WWDT_CFG_WINDOW_Msk         (0x7fUL << WWDT_CFG_WINDOW_Pos)             /*!< WINDOW (Bitfield-Mask: 0x7f)             */
#define WWDT_CFG_WINDOW             WWDT_CFG_WINDOW_Msk
#define WWDT_CFG_PSC_Pos            (7U)                                        /*!< TB (Bit 7)                          */
#define WWDT_CFG_PSC_0              (0x1UL << WWDT_CFG_PSC_Pos)
#define WWDT_CFG_PSC_1              (0x2UL << WWDT_CFG_PSC_Pos)
#define WWDT_CFG_PSC_Msk            (0x3UL << WWDT_CFG_PSC_Pos)                 /*!< TB (Bitfield-Mask: 0x03) */
#define WWDT_CFG_PSC                WWDT_CFG_PSC_Msk
#define WWDT_CFG_EWIEN_Pos          (9U)                                        /*!< EWIEN (Bit 9)                       */
#define WWDT_CFG_EWIEN_Msk          (0x1UL << WWDT_CFG_EWIEN_Pos)             /*!< EWIEN (Bitfield-Mask: 0x01)         */
#define WWDT_CFG_EWIEN              WWDT_CFG_EWIEN_Msk
/* ==========================================================  SR  =========================================================== */
#define WWDT_SR_EWIFLG_Pos          (0U)                                        /*!< EWIF (Bit 0)                           */
#define WWDT_SR_EWIFLG_Msk          (0x1UL << WWDT_SR_EWIFLG_Pos)               /*!< EWIF (Bitfield-Mask: 0x01)             */
#define WWDT_SR_EWIFLG              WWDT_SR_EWIFLG_Msk




/* ================================================================================================================================== */
/* =============================================================  SPI  ============================================================== */
/* ================================================================================================================================== */
/* =============================================  Bit definition for SPI_CR1 register  ============================================== */
#define SPI_CR1_CPHA_Pos            (0U)                                        /*!< Clock phase */
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk
#define SPI_CR1_CPOL_Pos            (1U)                                        /*!< Clock polarity */
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk
#define SPI_CR1_MSTR_Pos            (2U)                                        /*!< Master selection */
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk
#define SPI_CR1_BR_Pos              (3U)                                        /*!< Baud rate control */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk
#define SPI_CR1_SPIEN_Pos             (6U)                                        /*!< SPI enable */
#define SPI_CR1_SPIEN_Msk             (0x1UL << SPI_CR1_SPIEN_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPIEN                 SPI_CR1_SPIEN_Msk
#define SPI_CR1_LSBFIRST_Pos        (7U)                                        /*!< Frame format */
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk
#define SPI_CR1_SSI_Pos             (8U)                                        /*!< Internal slave select */
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk
#define SPI_CR1_SSM_Pos             (9U)                                        /*!< Software slave management */
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk
#define SPI_CR1_RXONLY_Pos          (10U)                                       /*!< Receive only */
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk
#define SPI_CR1_DFF_Pos             (11U)                                       /*!< Data frame format */
#define SPI_CR1_DFF_Msk             (0x1UL << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_DFF                 SPI_CR1_DFF_Msk
#define SPI_CR1_BIDIOEN_Pos         (14U)                                       /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIOEN_Msk         (0x1UL << SPI_CR1_BIDIOEN_Pos)              /*!< 0x00004000 */
#define SPI_CR1_BIDIOEN             SPI_CR1_BIDIOEN_Msk
#define SPI_CR1_BIDIMEN_Pos         (15U)                                       /*!< Bidirectional data mode enable */
#define SPI_CR1_BIDIMEN_Msk         (0x1UL << SPI_CR1_BIDIMEN_Pos)              /*!< 0x00008000 */
#define SPI_CR1_BIDIMEN             SPI_CR1_BIDIMEN_Msk

/* =============================================  Bit definition for SPI_CR2 register  ============================================== */
#define SPI_CR2_RXDMAEN_Pos         (0U)                                        /*!< Rx buffer DMA enable */
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk
#define SPI_CR2_TXDMAEN_Pos         (1U)                                        /*!< Tx buffer DMA enable */
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk
#define SPI_CR2_OVRIEN_Pos          (5U)                                        /*!< Error interrupt enable */
#define SPI_CR2_OVRIEN_Msk          (0x1UL << SPI_CR2_OVRIEN_Pos)               /*!< 0x00000020 */
#define SPI_CR2_OVRIEN              SPI_CR2_OVRIEN_Msk
#define SPI_CR2_RXFIFONEIEN_Pos     (6U)                                        /*!< RX buffer not empty interrupt enable */
#define SPI_CR2_RXFIFONEIEN_Msk     (0x1UL << SPI_CR2_RXFIFONEIEN_Pos)          /*!< 0x00000040 */
#define SPI_CR2_RXFIFONEIEN         SPI_CR2_RXFIFONEIEN_Msk
#define SPI_CR2_TXFIFOEIEN_Pos      (7U)                                        /*!< Tx buffer empty interrupt enable */
#define SPI_CR2_TXFIFOEIEN_Msk      (0x1UL << SPI_CR2_TXFIFOEIEN_Pos)           /*!< 0x00000080 */
#define SPI_CR2_TXFIFOEIEN          SPI_CR2_TXFIFOEIEN_Msk

/* ==============================================  Bit definition for SPI_SR register  ============================================== */
#define SPI_SR_RXFIFONEFLG_Pos      (0U)                                        /*!< Receive buffer not empty */
#define SPI_SR_RXFIFONEFLG_Msk      (0x1UL << SPI_SR_RXFIFONEFLG_Pos)           /*!< 0x00000001 */
#define SPI_SR_RXFIFONEFLG          SPI_SR_RXFIFONEFLG_Msk
#define SPI_SR_TXFIFOEFLG_Pos       (1U)                                        /*!< Transmit buffer empty */
#define SPI_SR_TXFIFOEFLG_Msk       (0x1UL << SPI_SR_TXFIFOEFLG_Pos)            /*!< 0x00000002 */
#define SPI_SR_TXFIFOEFLG           SPI_SR_TXFIFOEFLG_Msk
#define SPI_SR_OVRFLG_Pos           (6U)                                        /*!< Overrun flag */
#define SPI_SR_OVRFLG_Msk           (0x1UL << SPI_SR_OVRFLG_Pos)                /*!< 0x00000040 */
#define SPI_SR_OVRFLG               SPI_SR_OVRFLG_Msk
#define SPI_SR_BUSYFLG_Pos          (7U)                                        /*!< Busy flag */
#define SPI_SR_BUSYFLG_Msk          (0x1UL << SPI_SR_BUSYFLG_Pos)               /*!< 0x00000080 */
#define SPI_SR_BUSYFLG              SPI_SR_BUSYFLG_Msk
#define SPI_SR_RXFIFOVAL_Pos        (9U)                                        /*!< Receive FIFO Threshold */
#define SPI_SR_RXFIFOVAL_0          (0x1UL << SPI_SR_RXFIFOVAL_Pos)             /*!< 0x00000200 */
#define SPI_SR_RXFIFOVAL_1          (0x2UL << SPI_SR_RXFIFOVAL_Pos)             /*!< 0x00000400 */
#define SPI_SR_RXFIFOVAL_Msk        (0x3UL << SPI_SR_RXFIFOVAL_Pos)             /*!< 0x00000600 */
#define SPI_SR_RXFIFOVAL            SPI_SR_RXFIFOVAL_Msk
#define SPI_SR_TXFIFOVAL_Pos        (11U)                                       /*!< Transmit FIFO Threshold */
#define SPI_SR_TXFIFOVAL_0          (0x1UL << SPI_SR_TXFIFOVAL_Pos)             /*!< 0x00000800 */
#define SPI_SR_TXFIFOVAL_1          (0x2UL << SPI_SR_TXFIFOVAL_Pos)             /*!< 0x00001000 */
#define SPI_SR_TXFIFOVAL_Msk        (0x3UL << SPI_SR_TXFIFOVAL_Pos)             /*!< 0x00001800 */
#define SPI_SR_TXFIFOVAL            SPI_SR_TXFIFOVAL_Msk

/* ==============================================  Bit definition for SPI_DR register  ============================================== */
#define SPI_DR_DATA_Pos             (0U)                                        /*!< Data register */
#define SPI_DR_DATA_Msk             (0xFFFFUL << SPI_DR_DATA_Pos)               /*!< 0x0000FFFF */
#define SPI_DR_DATA                 SPI_DR_DATA_Msk


/* =========================================================================================================================== */
/* ============================================================  USART  ====================================================== */
/* =========================================================================================================================== */
/* ===========================================  Bit definition for USART_SR register  ======================================== */
#define USART_SR_PEFLG_Pos          (0U)
#define USART_SR_PEFLG_Msk          (0x1UL << USART_SR_PEFLG_Pos)               /*!< 0x00000001 */
#define USART_SR_PEFLG              USART_SR_PEFLG_Msk                          /*!<Parity Error                 */
#define USART_SR_FEFLG_Pos          (1U)
#define USART_SR_FEFLG_Msk          (0x1UL << USART_SR_FEFLG_Pos)               /*!< 0x00000002 */
#define USART_SR_FEFLG              USART_SR_FEFLG_Msk                          /*!<Framing Error                */
#define USART_SR_NEFLG_Pos          (2U)
#define USART_SR_NEFLG_Msk          (0x1UL << USART_SR_NEFLG_Pos)               /*!< 0x00000004 */
#define USART_SR_NEFLG              USART_SR_NEFLG_Msk                          /*!<Noise Error Flag             */
#define USART_SR_OREFLG_Pos         (3U)
#define USART_SR_OREFLG_Msk         (0x1UL << USART_SR_OREFLG_Pos)              /*!< 0x00000008 */
#define USART_SR_OREFLG             USART_SR_OREFLG_Msk                         /*!<OverRun Error                */
#define USART_SR_IDLEFLG_Pos        (4U)
#define USART_SR_IDLEFLG_Msk        (0x1UL << USART_SR_IDLEFLG_Pos)             /*!< 0x00000010 */
#define USART_SR_IDLEFLG            USART_SR_IDLEFLG_Msk                        /*!<IDLE line detected           */
#define USART_SR_RXNEFLG_Pos        (5U)
#define USART_SR_RXNEFLG_Msk        (0x1UL << USART_SR_RXNEFLG_Pos)             /*!< 0x00000020 */
#define USART_SR_RXNEFLG            USART_SR_RXNEFLG_Msk                        /*!<Read Data Register Not Empty */
#define USART_SR_TCFLG_Pos          (6U)
#define USART_SR_TCFLG_Msk          (0x1UL << USART_SR_TCFLG_Pos)               /*!< 0x00000040 */
#define USART_SR_TCFLG              USART_SR_TCFLG_Msk                          /*!<Transmission Complete        */
#define USART_SR_TXEFLG_Pos         (7U)
#define USART_SR_TXEFLG_Msk         (0x1UL << USART_SR_TXEFLG_Pos)              /*!< 0x00000080 */
#define USART_SR_TXEFLG             USART_SR_TXEFLG_Msk                         /*!<Transmit Data Register Empty */
#define USART_SR_LBDFLG_Pos         (8U)
#define USART_SR_LBDFLG_Msk         (0x1UL << USART_SR_LBDFLG_Pos)              /*!< 0x00000100 */
#define USART_SR_LBDFLG             USART_SR_LBDFLG_Msk                         /*!<LIN Break Detection Flag     */
#define USART_SR_CTSFLG_Pos         (9U)
#define USART_SR_CTSFLG_Msk         (0x1UL << USART_SR_CTSFLG_Pos)              /*!< 0x00000200 */
#define USART_SR_CTSFLG             USART_SR_CTSFLG_Msk                         /*!<CTS Flag                     */
#define USART_SR_RXTOFLG_Pos        (10U)
#define USART_SR_RXTOFLG_Msk        (0x1UL << USART_SR_RXTOFLG_Pos)             /*!< 0x00000400 */
#define USART_SR_RXTOFLG            USART_SR_RXTOFLG_Msk                        /*!<RXTO Flag                     */
#define USART_SR_ABCFLG_Pos         (11U)
#define USART_SR_ABCFLG_Msk         (0x1UL << USART_SR_ABCFLG_Pos)              /*!< 0x00000800 */
#define USART_SR_ABCFLG             USART_SR_ABCFLG_Msk                         /*!<ABC Flag                     */
#define USART_SR_ABERRFLG_Pos       (12U)
#define USART_SR_ABERRFLG_Msk       (0x1UL << USART_SR_ABERRFLG_Pos)            /*!< 0x00001000 */
#define USART_SR_ABERRFLG           USART_SR_ABERRFLG_Msk                       /*!<ABERR Flag                     */
#define USART_SR_CMFLG_Pos          (13U)
#define USART_SR_CMFLG_Msk          (0x1UL << USART_SR_CMFLG_Pos)               /*!< 0x00002000 */
#define USART_SR_CMFLG              USART_SR_CMFLG_Msk                          /*!<CMF Flag                     */

/* =============================================  Bit definition for USART_DR register  ============================================= */
#define USART_DR_DATA_Pos           (0U)
#define USART_DR_DATA_Msk           (0x1FFUL << USART_DR_DATA_Pos)              /*!< 0x000001FF */
#define USART_DR_DATA               USART_DR_DATA_Msk                           /*!<Data value */

/* ============================================  Bit definition for USART_BRR register  ============================================= */
#define USART_BRR_FBR_Pos           (0U)
#define USART_BRR_FBR_Msk           (0xFUL << USART_BRR_FBR_Pos)                /*!< 0x0000000F */
#define USART_BRR_FBR               USART_BRR_FBR_Msk                           /*!<Fraction of USARTDIV */
#define USART_BRR_IBR_Pos           (4U)
#define USART_BRR_IBR_Msk           (0xFFFUL << USART_BRR_IBR_Pos)              /*!< 0x0000FFF0 */
#define USART_BRR_IBR               USART_BRR_IBR_Msk                           /*!<Mantissa of USARTDIV */

/* ============================================  Bit definition for USART_CR1 register  ============================================= */
#define USART_CR1_SBK_Pos           (0U)
#define USART_CR1_SBK_Msk           (0x1UL << USART_CR1_SBK_Pos)                /*!< 0x00000001 */
#define USART_CR1_SBK               USART_CR1_SBK_Msk                           /*!<Send Break                             */
#define USART_CR1_RWU_Pos           (1U)
#define USART_CR1_RWU_Msk           (0x1UL << USART_CR1_RWU_Pos)                /*!< 0x00000002 */
#define USART_CR1_RWU               USART_CR1_RWU_Msk                           /*!<Receiver wakeup                        */
#define USART_CR1_REN_Pos           (2U)
#define USART_CR1_REN_Msk           (0x1UL << USART_CR1_REN_Pos)                /*!< 0x00000004 */
#define USART_CR1_REN               USART_CR1_REN_Msk                           /*!<Receiver Enable                        */
#define USART_CR1_TEN_Pos           (3U)
#define USART_CR1_TEN_Msk           (0x1UL << USART_CR1_TEN_Pos)                /*!< 0x00000008 */
#define USART_CR1_TEN               USART_CR1_TEN_Msk                           /*!<Transmitter Enable                     */
#define USART_CR1_IDLEIEN_Pos       (4U)
#define USART_CR1_IDLEIEN_Msk       (0x1UL << USART_CR1_IDLEIEN_Pos)            /*!< 0x00000010 */
#define USART_CR1_IDLEIEN           USART_CR1_IDLEIEN_Msk                       /*!<IDLE Interrupt Enable                  */
#define USART_CR1_RXNEIEN_Pos       (5U)
#define USART_CR1_RXNEIEN_Msk       (0x1UL << USART_CR1_RXNEIEN_Pos)            /*!< 0x00000020 */
#define USART_CR1_RXNEIEN           USART_CR1_RXNEIEN_Msk                       /*!<RXNE Interrupt Enable                  */
#define USART_CR1_TCIEN_Pos         (6U)
#define USART_CR1_TCIEN_Msk         (0x1UL << USART_CR1_TCIEN_Pos)              /*!< 0x00000040 */
#define USART_CR1_TCIEN             USART_CR1_TCIEN_Msk                         /*!<Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIEN_Pos        (7U)
#define USART_CR1_TXEIEN_Msk        (0x1UL << USART_CR1_TXEIEN_Pos)             /*!< 0x00000080 */
#define USART_CR1_TXEIEN            USART_CR1_TXEIEN_Msk                        /*!<TXE Interrupt Enable                   */
#define USART_CR1_PEIEN_Pos         (8U)
#define USART_CR1_PEIEN_Msk         (0x1UL << USART_CR1_PEIEN_Pos)              /*!< 0x00000100 */
#define USART_CR1_PEIEN             USART_CR1_PEIEN_Msk                         /*!<PE Interrupt Enable                    */
#define USART_CR1_PS_Pos            (9U)
#define USART_CR1_PS_Msk            (0x1UL << USART_CR1_PS_Pos)                 /*!< 0x00000200 */
#define USART_CR1_PS                USART_CR1_PS_Msk                            /*!<Parity Selection                       */
#define USART_CR1_PCEN_Pos          (10U)
#define USART_CR1_PCEN_Msk          (0x1UL << USART_CR1_PCEN_Pos)               /*!< 0x00000400 */
#define USART_CR1_PCEN              USART_CR1_PCEN_Msk                          /*!<Parity Control Enable                  */
#define USART_CR1_WKSEL_Pos         (11U)
#define USART_CR1_WKSEL_Msk         (0x1UL << USART_CR1_WKSEL_Pos)              /*!< 0x00000800 */
#define USART_CR1_WKSEL             USART_CR1_WKSEL_Msk                         /*!<Wakeup method                          */
#define USART_CR1_M_Pos             (12U)
#define USART_CR1_M_Msk             (0x3UL << USART_CR1_M_Pos)                  /*!< 0x00003000 */
#define USART_CR1_M_0               (0x1UL << USART_CR1_M_Pos)                  /*!< 0x00001000 */
#define USART_CR1_M_1               (0x2UL << USART_CR1_M_Pos)                  /*!< 0x00002000 */
#define USART_CR1_M                 USART_CR1_M_Msk                             /*!<Word length                            */
#define USART_CR1_UEN_Pos           (14U)
#define USART_CR1_UEN_Msk           (0x1UL << USART_CR1_UEN_Pos)                /*!< 0x00002000 */
#define USART_CR1_UEN               USART_CR1_UEN_Msk                           /*!<USART Enable                           */
#define USART_CR1_OSMCFG_Pos        (15U)
#define USART_CR1_OSMCFG_Msk        (0x1UL << USART_CR1_OSMCFG_Pos)             /*!< 0x00008000 */
#define USART_CR1_OSMCFG            USART_CR1_OSMCFG_Msk                        /*!<USART Oversampling by 8 enable         */
#define USART_CR1_DEDT_Pos          (16U)
#define USART_CR1_DEDT_Msk          (0x1FUL << USART_CR1_DEDT_Pos)              /*!< 0x03E00000 */
#define USART_CR1_DEDT              USART_CR1_DEDT_Msk                          /*!<USART Driver Enable deassertion time         */
#define USART_CR1_DEAT_Pos          (21U)
#define USART_CR1_DEAT_Msk          (0x1FUL << USART_CR1_DEAT_Pos)              /*!< 0x00008000 */
#define USART_CR1_DEAT              USART_CR1_DEAT_Msk                          /*!<USART Driver Enable assertion time        */
#define USART_CR1_RXTODEN_Pos       (26U)
#define USART_CR1_RXTODEN_Msk       (0x1UL << USART_CR1_RXTODEN_Pos)            /*!< 0x00008000 */
#define USART_CR1_RXTODEN           USART_CR1_RXTODEN_Msk                       /*!<USART Receive Timeout Detection Function Enable     */
#define USART_CR1_RXTOIEN_Pos       (27U)
#define USART_CR1_RXTOIEN_Msk       (0x1UL << USART_CR1_RXTOIEN_Pos)            /*!< 0x04000000 */
#define USART_CR1_RXTOIEN           USART_CR1_RXTOIEN_Msk                       /*!<USART Receive Timeout Detection Function Enable       */
#define USART_CR1_SWAPEN_Pos        (28U)
#define USART_CR1_SWAPEN_Msk        (0x1UL << USART_CR1_SWAPEN_Pos)             /*!< 0x08000000 */
#define USART_CR1_SWAPEN            USART_CR1_SWAPEN_Msk                        /*!<USART Swap TX/RX Pins Function Enable        */
#define USART_CR1_CMIEN_Pos         (29U)
#define USART_CR1_CMIEN_Msk         (0x1UL << USART_CR1_CMIEN_Pos)              /*!< 0x08000000 */
#define USART_CR1_CMIEN             USART_CR1_CMIEN_Msk                         /*!<USART Swap TX/RX Pins Function Enable        */

/* ============================================  Bit definition for USART_CR2 register  ============================================= */
#define USART_CR2_ADDRM7_Pos                   (4U)
#define USART_CR2_ADDRM7_Msk                   (0x1UL << USART_CR2_ADDRM7_Pos)                   /*!< 0x00000010 */
#define USART_CR2_ADDRM7                       USART_CR2_ADDRM7_Msk                              /*!<Address of the USART node            */
#define USART_CR2_LBDL_Pos                    (5U)
#define USART_CR2_LBDL_Msk                    (0x1UL << USART_CR2_LBDL_Pos)                    /*!< 0x00000020 */
#define USART_CR2_LBDL                        USART_CR2_LBDL_Msk                               /*!<LIN Break Detection Length           */
#define USART_CR2_LBDIEN_Pos                   (6U)
#define USART_CR2_LBDIEN_Msk                   (0x1UL << USART_CR2_LBDIEN_Pos)                   /*!< 0x00000040 */
#define USART_CR2_LBDIEN                       USART_CR2_LBDIEN_Msk                              /*!<LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos                    (8U)
#define USART_CR2_LBCL_Msk                    (0x1UL << USART_CR2_LBCL_Pos)                    /*!< 0x00000100 */
#define USART_CR2_LBCL                        USART_CR2_LBCL_Msk                               /*!<Last Bit Clock pulse                 */
#define USART_CR2_CPHA_Pos                    (9U)
#define USART_CR2_CPHA_Msk                    (0x1UL << USART_CR2_CPHA_Pos)                    /*!< 0x00000200 */
#define USART_CR2_CPHA                        USART_CR2_CPHA_Msk                               /*!<Clock Phase                          */
#define USART_CR2_CPOL_Pos                    (10U)
#define USART_CR2_CPOL_Msk                    (0x1UL << USART_CR2_CPOL_Pos)                    /*!< 0x00000400 */
#define USART_CR2_CPOL                        USART_CR2_CPOL_Msk                               /*!<Clock Polarity                       */
#define USART_CR2_CLKEN_Pos                   (11U)
#define USART_CR2_CLKEN_Msk                   (0x1UL << USART_CR2_CLKEN_Pos)                   /*!< 0x00000800 */
#define USART_CR2_CLKEN                       USART_CR2_CLKEN_Msk                              /*!<Clock Enable                         */
#define USART_CR2_STOP_Pos                    (12U)
#define USART_CR2_STOP_Msk                    (0x1UL << USART_CR2_STOP_Pos)                    /*!< 0x00001000 */
#define USART_CR2_STOP                        USART_CR2_STOP_Msk                               /*!<STOP[1:0] bits (STOP bits) */
#define USART_CR2_LINEN_Pos                   (14U)
#define USART_CR2_LINEN_Msk                   (0x1UL << USART_CR2_LINEN_Pos)                   /*!< 0x00004000 */
#define USART_CR2_LINEN                       USART_CR2_LINEN_Msk                              /*!<LIN mode enable */
#define USART_CR2_MSBFIRST_Pos                (15U)
#define USART_CR2_MSBFIRST_Msk                (0x1UL << USART_CR2_MSBFIRST_Pos)                /*!< 0x00008000 */
#define USART_CR2_MSBFIRST                    USART_CR2_MSBFIRST_Msk                           /*!<MSB select */
#define USART_CR2_ABEN_Pos                    (16U)
#define USART_CR2_ABEN_Msk                    (0x1UL << USART_CR2_ABEN_Pos)                     /*!< 0x00010000 */
#define USART_CR2_ABEN                        USART_CR2_ABEN_Msk                                /*!<Automatic porter rate mode enable */
#define USART_CR2_ABCIEN_Pos                  (17U)
#define USART_CR2_ABCIEN_Msk                  (0x1UL << USART_CR2_ABCIEN_Pos)                   /*!< 0x00020000 */
#define USART_CR2_ABCIEN                      USART_CR2_ABCIEN_Msk                              /*!<Automatic porter rate interrupt mode enable */
#define USART_CR2_ABEIEN_Pos                 (18U)
#define USART_CR2_ABEIEN_Msk                 (0x1UL << USART_CR2_ABEIEN_Pos)                   /*!< 0x00040000 */
#define USART_CR2_ABEIEN                     USART_CR2_ABEIEN_Msk                              /*!<Automatic porter rate error interrupt mode enable */
#define USART_CR2_ADDR_Pos                    (24U)
#define USART_CR2_ADDR_Msk                    (0xFFUL << USART_CR2_ADDR_Pos)                    /*!< 0xFF000000 */
#define USART_CR2_ADDR                        USART_CR2_ADDR_Msk                                /*!< Usart node address */

/* ============================================  Bit definition for USART_CR3 register  ============================================= */
#define USART_CR3_EIEN_Pos          (0U)
#define USART_CR3_EIEN_Msk          (0x1UL << USART_CR3_EIEN_Pos)               /*!< 0x00000001 */
#define USART_CR3_EIEN              USART_CR3_EIEN_Msk                          /*!<Error Interrupt Enable      */
#define USART_CR3_HDSEL_Pos         (3U)
#define USART_CR3_HDSEL_Msk         (0x1UL << USART_CR3_HDSEL_Pos)              /*!< 0x00000008 */
#define USART_CR3_HDSEL             USART_CR3_HDSEL_Msk                         /*!<Half-Duplex Selection       */
#define USART_CR3_DMAR_Pos          (6U)
#define USART_CR3_DMAR_Msk          (0x1UL << USART_CR3_DMAR_Pos)               /*!< 0x00000040 */
#define USART_CR3_DMAR              USART_CR3_DMAR_Msk                          /*!<DMA Enable Receiver         */
#define USART_CR3_DMAT_Pos          (7U)
#define USART_CR3_DMAT_Msk          (0x1UL << USART_CR3_DMAT_Pos)               /*!< 0x00000080 */
#define USART_CR3_DMAT              USART_CR3_DMAT_Msk                          /*!<DMA Enable Transmitter      */
#define USART_CR3_RTSEN_Pos         (8U)
#define USART_CR3_RTSEN_Msk         (0x1UL << USART_CR3_RTSEN_Pos)              /*!< 0x00000100 */
#define USART_CR3_RTSEN             USART_CR3_RTSEN_Msk                         /*!<RTS Enable                  */
#define USART_CR3_CTSEN_Pos         (9U)
#define USART_CR3_CTSEN_Msk         (0x1UL << USART_CR3_CTSEN_Pos)              /*!< 0x00000200 */
#define USART_CR3_CTSEN             USART_CR3_CTSEN_Msk                         /*!<CTS Enable                  */
#define USART_CR3_CTSIEN_Pos        (10U)
#define USART_CR3_CTSIEN_Msk        (0x1UL << USART_CR3_CTSIEN_Pos)             /*!< 0x00000400 */
#define USART_CR3_CTSIEN            USART_CR3_CTSIEN_Msk                        /*!<CTS Interrupt Enable        */
#define USART_CR3_ONEBIT_Pos        (11U)
#define USART_CR3_ONEBIT_Msk        (0x1UL << USART_CR3_ONEBIT_Pos)             /*!< 0x00000800 */
#define USART_CR3_ONEBIT            USART_CR3_ONEBIT_Msk                        /*!< Data sampling mode */
#define USART_CR3_DEMEN_Pos         (12U)
#define USART_CR3_DEMEN_Msk         (0x1UL << USART_CR3_DEMEN_Pos)              /*!< 0x00001000 */
#define USART_CR3_DEMEN             USART_CR3_DEMEN_Msk                         /*!<USART Driver enable mode) */

/* ===========================================  Bit definition for USART_RXTOR register  ============================================ */
#define USART_RXTOR_RXTO_Pos        (0U)
#define USART_RXTOR_RXTO_Msk        (0xFFFFFFUL << USART_RXTOR_RXTO_Pos)        /*!< 0x00FFFFFF */
#define USART_RXTOR_RXTO            USART_RXTOR_RXTO_Msk                        /*!<Data value */

/* =========================================================================================================================== */
/* ============================================================  UART  ======================================================= */
/* =========================================================================================================================== */
/* ===========================================  Bit definition for UART_SR register  ========================================= */
#define UART_SR_PEFLG_Pos           (0U)
#define UART_SR_PEFLG_Msk           (0x1UL << UART_SR_PEFLG_Pos)                /*!< 0x00000001 */
#define UART_SR_PEFLG               UART_SR_PEFLG_Msk                           /*!<Parity Error                 */
#define UART_SR_FEFLG_Pos           (1U)
#define UART_SR_FEFLG_Msk           (0x1UL << UART_SR_FEFLG_Pos)                /*!< 0x00000002 */
#define UART_SR_FEFLG               UART_SR_FEFLG_Msk                           /*!<Framing Error                */
#define UART_SR_NEFLG_Pos           (2U)
#define UART_SR_NEFLG_Msk           (0x1UL << UART_SR_NEFLG_Pos)                /*!< 0x00000004 */
#define UART_SR_NEFLG               UART_SR_NEFLG_Msk                           /*!<Noise Error Flag             */
#define UART_SR_OREFLG_Pos          (3U)
#define UART_SR_OREFLG_Msk          (0x1UL << UART_SR_OREFLG_Pos)               /*!< 0x00000008 */
#define UART_SR_OREFLG              UART_SR_OREFLG_Msk                          /*!<OverRun Error                */
#define UART_SR_IDLEFLG_Pos         (4U)
#define UART_SR_IDLEFLG_Msk         (0x1UL << UART_SR_IDLEFLG_Pos)              /*!< 0x00000010 */
#define UART_SR_IDLEFLG             UART_SR_IDLEFLG_Msk                         /*!<IDLE line detected           */
#define UART_SR_RXNEFLG_Pos         (5U)
#define UART_SR_RXNEFLG_Msk         (0x1UL << UART_SR_RXNEFLG_Pos)              /*!< 0x00000020 */
#define UART_SR_RXNEFLG             UART_SR_RXNEFLG_Msk                         /*!<Read Data Register Not Empty */
#define UART_SR_TCFLG_Pos           (6U)
#define UART_SR_TCFLG_Msk           (0x1UL << UART_SR_TCFLG_Pos)                /*!< 0x00000040 */
#define UART_SR_TCFLG               UART_SR_TCFLG_Msk                           /*!<Transmission Complete        */
#define UART_SR_TXEFLG_Pos          (7U)
#define UART_SR_TXEFLG_Msk          (0x1UL << UART_SR_TXEFLG_Pos)               /*!< 0x00000080 */
#define UART_SR_TXEFLG              UART_SR_TXEFLG_Msk                          /*!<Transmit Data Register Empty */
#define UART_SR_LBDFLG_Pos          (8U)
#define UART_SR_LBDFLG_Msk          (0x1UL << UART_SR_LBDFLG_Pos)               /*!< 0x00000100 */
#define UART_SR_LBDFLG              UART_SR_LBDFLG_Msk                          /*!<LIN Break Detection Flag     */
#define UART_SR_RXTOFLG_Pos         (10U)
#define UART_SR_RXTOFLG_Msk         (0x1UL << UART_SR_RXTOFLG_Pos)              /*!< 0x00000400 */
#define UART_SR_RXTOFLG             UART_SR_RXTOFLG_Msk                         /*!<RXTO Flag                     */
#define UART_SR_ABCFLG_Pos          (11U)
#define UART_SR_ABCFLG_Msk          (0x1UL << UART_SR_ABCFLG_Pos)               /*!< 0x00000800 */
#define UART_SR_ABCFLG              UART_SR_ABCFLG_Msk                          /*!<ABC Flag                     */
#define UART_SR_ABERRFLG_Pos        (12U)
#define UART_SR_ABERRFLG_Msk        (0x1UL << UART_SR_ABERRFLG_Pos)             /*!< 0x00001000 */
#define UART_SR_ABERRFLG            UART_SR_ABERRFLG_Msk                        /*!<ABERR Flag                     */

/* =============================================  Bit definition for UART_DR register  ============================================= */
#define UART_DR_DATA_Pos            (0U)
#define UART_DR_DATA_Msk            (0x1FFUL << UART_DR_DATA_Pos)               /*!< 0x000001FF */
#define UART_DR_DATA                UART_DR_DATA_Msk                            /*!<Data value */

/* ============================================  Bit definition for UART_BRR register  ============================================= */
#define UART_BRR_FBR_Pos            (0U)
#define UART_BRR_FBR_Msk            (0xFUL << UART_BRR_FBR_Pos)                 /*!< 0x0000000F */
#define UART_BRR_FBR                UART_BRR_FBR_Msk                            /*!<Fraction of UARTDIV */
#define UART_BRR_IBR_Pos            (4U)
#define UART_BRR_IBR_Msk            (0xFFFUL << UART_BRR_IBR_Pos)               /*!< 0x0000FFF0 */
#define UART_BRR_IBR                UART_BRR_IBR_Msk                            /*!<Mantissa of UARTDIV */

/* ============================================  Bit definition for UART_CR1 register  ============================================= */
#define UART_CR1_SBK_Pos            (0U)
#define UART_CR1_SBK_Msk            (0x1UL << UART_CR1_SBK_Pos)                 /*!< 0x00000001 */
#define UART_CR1_SBK                UART_CR1_SBK_Msk                            /*!<Send Break                             */
#define UART_CR1_REN_Pos            (2U)
#define UART_CR1_REN_Msk            (0x1UL << UART_CR1_REN_Pos)                 /*!< 0x00000004 */
#define UART_CR1_REN                UART_CR1_REN_Msk                            /*!<Receiver Enable                        */
#define UART_CR1_TEN_Pos            (3U)
#define UART_CR1_TEN_Msk            (0x1UL << UART_CR1_TEN_Pos)                 /*!< 0x00000008 */
#define UART_CR1_TEN                UART_CR1_TEN_Msk                            /*!<Transmitter Enable                     */
#define UART_CR1_IDLEIEN_Pos        (4U)
#define UART_CR1_IDLEIEN_Msk        (0x1UL << UART_CR1_IDLEIEN_Pos)             /*!< 0x00000010 */
#define UART_CR1_IDLEIEN            UART_CR1_IDLEIEN_Msk                        /*!<IDLE Interrupt Enable                  */
#define UART_CR1_RXNEIEN_Pos        (5U)
#define UART_CR1_RXNEIEN_Msk        (0x1UL << UART_CR1_RXNEIEN_Pos)             /*!< 0x00000020 */
#define UART_CR1_RXNEIEN            UART_CR1_RXNEIEN_Msk                        /*!<RXNE Interrupt Enable                  */
#define UART_CR1_TCIEN_Pos          (6U)
#define UART_CR1_TCIEN_Msk          (0x1UL << UART_CR1_TCIEN_Pos)               /*!< 0x00000040 */
#define UART_CR1_TCIEN              UART_CR1_TCIEN_Msk                          /*!<Transmission Complete Interrupt Enable */
#define UART_CR1_TXEIEN_Pos         (7U)
#define UART_CR1_TXEIEN_Msk         (0x1UL << UART_CR1_TXEIEN_Pos)              /*!< 0x00000080 */
#define UART_CR1_TXEIEN             UART_CR1_TXEIEN_Msk                         /*!<TXE Interrupt Enable                   */
#define UART_CR1_PEIEN_Pos          (8U)
#define UART_CR1_PEIEN_Msk          (0x1UL << UART_CR1_PEIEN_Pos)               /*!< 0x00000100 */
#define UART_CR1_PEIEN              UART_CR1_PEIEN_Msk                          /*!<PE Interrupt Enable                    */
#define UART_CR1_PSEL_Pos           (9U)
#define UART_CR1_PSEL_Msk           (0x1UL << UART_CR1_PSEL_Pos)                /*!< 0x00000200 */
#define UART_CR1_PSEL               UART_CR1_PSEL_Msk                           /*!<Parity Selection                       */
#define UART_CR1_PCEN_Pos           (10U)
#define UART_CR1_PCEN_Msk           (0x1UL << UART_CR1_PCEN_Pos)                /*!< 0x00000400 */
#define UART_CR1_PCEN               UART_CR1_PCEN_Msk                           /*!<Parity Control Enable                  */
#define UART_CR1_M_Pos              (12U)
#define UART_CR1_M_Msk              (0x3UL << UART_CR1_M_Pos)                   /*!< 0x00003000 */
#define UART_CR1_M_0                (0x1UL << UART_CR1_M_Pos)                   /*!< 0x00001000 */
#define UART_CR1_M_1                (0x2UL << UART_CR1_M_Pos)                   /*!< 0x00002000 */
#define UART_CR1_M                  UART_CR1_M_Msk                              /*!<Word length                            */
#define UART_CR1_UEN_Pos            (13U)
#define UART_CR1_UEN_Msk            (0x1UL << UART_CR1_UEN_Pos)                 /*!< 0x00002000 */
#define UART_CR1_UEN                UART_CR1_UEN_Msk                            /*!<UART Enable                           */
#define UART_CR1_RXTODEN_Pos        (26U)
#define UART_CR1_RXTODEN_Msk        (0x1UL << UART_CR1_RXTODEN_Pos)             /*!< 0x00008000 */
#define UART_CR1_RXTODEN            UART_CR1_RXTODEN_Msk                        /*!<UART Receive Timeout Detection Function Enable     */
#define UART_CR1_RXTOIEN_Pos        (27U)
#define UART_CR1_RXTOIEN_Msk        (0x1UL << UART_CR1_RXTOIEN_Pos)             /*!< 0x04000000 */
#define UART_CR1_RXTOIEN            UART_CR1_RXTOIEN_Msk                        /*!<UART Receive Timeout Detection Function Enable       */
#define UART_CR1_SWAPEN_Pos         (28U)
#define UART_CR1_SWAPEN_Msk         (0x1UL << UART_CR1_SWAPEN_Pos)              /*!< 0x08000000 */
#define UART_CR1_SWAPEN             UART_CR1_SWAPEN_Msk                         /*!<UART Swap TX/RX Pins Function Enable        */

/* ============================================  Bit definition for UART_CR2 register  ============================================= */
#define UART_CR2_LBDL_Pos           (5U)
#define UART_CR2_LBDL_Msk           (0x1UL << UART_CR2_LBDL_Pos)                /*!< 0x00000020 */
#define UART_CR2_LBDL               UART_CR2_LBDL_Msk                           /*!<LIN Break Detection Length           */
#define UART_CR2_LBDIEN_Pos         (6U)
#define UART_CR2_LBDIEN_Msk         (0x1UL << UART_CR2_LBDIEN_Pos)              /*!< 0x00000040 */
#define UART_CR2_LBDIEN             UART_CR2_LBDIEN_Msk                         /*!<LIN Break Detection Interrupt Enable */
#define UART_CR2_STOP_Pos           (13U)
#define UART_CR2_STOP_Msk           (0x1UL << UART_CR2_STOP_Pos)                /*!< 0x00001000 */
#define UART_CR2_STOP               UART_CR2_STOP_Msk                           /*!<STOP[1:0] bits (STOP bits) */
#define UART_CR2_LINEN_Pos          (14U)
#define UART_CR2_LINEN_Msk          (0x1UL << UART_CR2_LINEN_Pos)               /*!< 0x00004000 */
#define UART_CR2_LINEN              UART_CR2_LINEN_Msk                          /*!<LIN mode enable */
#define UART_CR2_ABEN_Pos           (16U)
#define UART_CR2_ABEN_Msk           (0x1UL << UART_CR2_ABEN_Pos)                /*!< 0x00010000 */
#define UART_CR2_ABEN               UART_CR2_ABEN_Msk                           /*!<Automatic porter rate mode enable */
#define UART_CR2_ABCIEN_Pos         (17U)
#define UART_CR2_ABCIEN_Msk         (0x1UL << UART_CR2_ABCIEN_Pos)              /*!< 0x00020000 */
#define UART_CR2_ABCIEN             UART_CR2_ABCIEN_Msk                         /*!<Automatic porter rate interrupt mode enable */
#define UART_CR2_ABEIEN_Pos         (18U)
#define UART_CR2_ABEIEN_Msk         (0x1UL << UART_CR2_ABEIEN_Pos)              /*!< 0x00040000 */
#define UART_CR2_ABEIEN             UART_CR2_ABEIEN_Msk                         /*!<Automatic porter rate error interrupt mode enable */

/* ============================================  Bit definition for UART_CR3 register  ============================================= */
#define UART_CR3_EIEN_Pos           (0U)
#define UART_CR3_EIEN_Msk           (0x1UL << UART_CR3_EIEN_Pos)                /*!< 0x00000001 */
#define UART_CR3_EIEN               UART_CR3_EIEN_Msk                           /*!<Error Interrupt Enable      */
#define UART_CR3_HDSEL_Pos          (3U)
#define UART_CR3_HDSEL_Msk          (0x1UL << UART_CR3_HDSEL_Pos)               /*!< 0x00000008 */
#define UART_CR3_HDSEL              UART_CR3_HDSEL_Msk                          /*!<Half-Duplex Selection       */
#define UART_CR3_DMAR_Pos           (6U)
#define UART_CR3_DMAR_Msk           (0x1UL << UART_CR3_DMAR_Pos)                /*!< 0x00000040 */
#define UART_CR3_DMAR               UART_CR3_DMAR_Msk                           /*!<DMA Enable Receiver         */
#define UART_CR3_DMAT_Pos           (7U)
#define UART_CR3_DMAT_Msk           (0x1UL << UART_CR3_DMAT_Pos)                /*!< 0x00000080 */
#define UART_CR3_DMAT               UART_CR3_DMAT_Msk                           /*!<DMA Enable Transmitter      */
#define UART_CR3_ONEBIT_Pos         (11U)
#define UART_CR3_ONEBIT_Msk         (0x1UL << UART_CR3_ONEBIT_Pos)              /*!< 0x00000800 */
#define UART_CR3_ONEBIT             UART_CR3_ONEBIT_Msk                         /*!< Data sampling mode */

/* ===========================================  Bit definition for UART_RXTOR register  ============================================ */
#define UART_RXTOR_RXTO_Pos         (0U)
#define UART_RXTOR_RXTO_Msk         (0xFFFFFFUL << UART_RXTOR_RXTO_Pos)         /*!< 0x00FFFFFF */
#define UART_RXTOR_RXTO             UART_RXTOR_RXTO_Msk                         /*!<Data value */

/* ================================================================================================================================== */
/* =============================================================  I2C  ============================================================== */
/* ================================================================================================================================== */
/* ============================================  Bit definition for I2C_CR1 register  ============================================= */
#define I2C_CR1_I2CEN_Pos           (0U)                                        /*!< I2C Enable */
#define I2C_CR1_I2CEN_Msk           (0x1UL << I2C_CR1_I2CEN_Pos)                /*!< 0x00000001 */
#define I2C_CR1_I2CEN               I2C_CR1_I2CEN_Msk
#define I2C_CR1_SMBEN_Pos           (1U)                                        /*!< SMBus Mode Enable */
#define I2C_CR1_SMBEN_Msk           (0x1UL << I2C_CR1_SMBEN_Pos)                /*!< 0x00000002 */
#define I2C_CR1_SMBEN               I2C_CR1_SMBEN_Msk
#define I2C_CR1_SMBTCFG_Pos         (3U)                                        /*!< SMBus Type Configure */
#define I2C_CR1_SMBTCFG_Msk         (0x1UL << I2C_CR1_SMBTCFG_Pos)              /*!< 0x00000008 */
#define I2C_CR1_SMBTCFG             I2C_CR1_SMBTCFG_Msk
#define I2C_CR1_ARPEN_Pos           (4U)                                        /*!< ARP Enable */
#define I2C_CR1_ARPEN_Msk           (0x1UL << I2C_CR1_ARPEN_Pos)                /*!< 0x00000010 */
#define I2C_CR1_ARPEN               I2C_CR1_ARPEN_Msk
#define I2C_CR1_PECEN_Pos           (5U)                                        /*!< PEC Enable */
#define I2C_CR1_PECEN_Msk           (0x1UL << I2C_CR1_PECEN_Pos)                /*!< 0x00000020 */
#define I2C_CR1_PECEN               I2C_CR1_PECEN_Msk
#define I2C_CR1_SRBEN_Pos           (6U)                                        /*!< Slave Responds Broadcast Enable */
#define I2C_CR1_SRBEN_Msk           (0x1UL << I2C_CR1_SRBEN_Pos)                /*!< 0x00000040 */
#define I2C_CR1_SRBEN               I2C_CR1_SRBEN_Msk
#define I2C_CR1_CLKSTRETCHD_Pos     (7U)                                        /*!< Slave Mode Clock Stretching Disable */
#define I2C_CR1_CLKSTRETCHD_Msk     (0x1UL << I2C_CR1_CLKSTRETCHD_Pos)          /*!< 0x00000080 */
#define I2C_CR1_CLKSTRETCHD         I2C_CR1_CLKSTRETCHD_Msk
#define I2C_CR1_START_Pos           (8U)                                        /*!< Start Bit Transfer */
#define I2C_CR1_START_Msk           (0x1UL << I2C_CR1_START_Pos)                /*!< 0x00000100 */
#define I2C_CR1_START               I2C_CR1_START_Msk
#define I2C_CR1_STOP_Pos            (9U)                                        /*!< Stop Bit Transfer */
#define I2C_CR1_STOP_Msk            (0x1UL << I2C_CR1_STOP_Pos)                 /*!< 0x00000200 */
#define I2C_CR1_STOP                I2C_CR1_STOP_Msk
#define I2C_CR1_ACKEN_Pos           (10U)                                       /*!< Acknowledge Transfer Enable */
#define I2C_CR1_ACKEN_Msk           (0x1UL << I2C_CR1_ACKEN_Pos)                /*!< 0x00000400 */
#define I2C_CR1_ACKEN               I2C_CR1_ACKEN_Msk
#define I2C_CR1_ACKPOS_Pos          (11U)                                       /*!< Acknowledge /PEC Position Configure */
#define I2C_CR1_ACKPOS_Msk          (0x1UL << I2C_CR1_ACKPOS_Pos)               /*!< 0x00000800 */
#define I2C_CR1_ACKPOS              I2C_CR1_ACKPOS_Msk
#define I2C_CR1_PEC_Pos             (12U)                                       /*!< Packet Error Check Transfer Enable */
#define I2C_CR1_PEC_Msk             (0x1UL << I2C_CR1_PEC_Pos)                  /*!< 0x00001000 */
#define I2C_CR1_PEC                 I2C_CR1_PEC_Msk
#define I2C_CR1_ALERTEN_Pos         (13U)                                       /*!< SMBus Alert Enable */
#define I2C_CR1_ALERTEN_Msk         (0x1UL << I2C_CR1_ALERTEN_Pos)              /*!< 0x00002000 */
#define I2C_CR1_ALERTEN             I2C_CR1_ALERTEN_Msk
#define I2C_CR1_SWRSTS_Pos          (15U)                                       /*!< Software Configure I2C under Reset State */
#define I2C_CR1_SWRSTS_Msk          (0x1UL << I2C_CR1_SWRSTS_Pos)               /*!< 0x00008000 */
#define I2C_CR1_SWRSTS              I2C_CR1_SWRSTS_Msk

/* ============================================  Bit definition for I2C_CR2 register  ============================================= */
#define I2C_CR2_CLKFCFG_Pos         (0U)                                        /*!< I2C Clock Frequency Configure */
#define I2C_CR2_CLKFCFG_Msk         (0xFFUL << I2C_CR2_CLKFCFG_Pos)             /*!< 0x000000FF */
#define I2C_CR2_CLKFCFG             I2C_CR2_CLKFCFG_Msk
#define I2C_CR2_ERRIEN_Pos          (8U)                                        /*!< Error Interrupt Enable */
#define I2C_CR2_ERRIEN_Msk          (0x1UL << I2C_CR2_ERRIEN_Pos)               /*!< 0x00000100 */
#define I2C_CR2_ERRIEN              I2C_CR2_ERRIEN_Msk
#define I2C_CR2_EVIEN_Pos           (9U)                                        /*!< Event Interrupt Enable */
#define I2C_CR2_EVIEN_Msk           (0x1UL << I2C_CR2_EVIEN_Pos)                /*!< 0x00000200 */
#define I2C_CR2_EVIEN               I2C_CR2_EVIEN_Msk
#define I2C_CR2_BUFIEN_Pos          (10U)                                       /*!< Error interrupt enable */
#define I2C_CR2_BUFIEN_Msk          (0x1UL << I2C_CR2_BUFIEN_Pos)               /*!< 0x00000400 */
#define I2C_CR2_BUFIEN              I2C_CR2_BUFIEN_Msk
#define I2C_CR2_DMAEN_Pos           (11U)                                       /*!< DMA Requests Enable */
#define I2C_CR2_DMAEN_Msk           (0x1UL << I2C_CR2_DMAEN_Pos)                /*!< 0x00000800 */
#define I2C_CR2_DMAEN               I2C_CR2_DMAEN_Msk
#define I2C_CR2_LTCFG_Pos           (12U)                                       /*!< DMA Last Transfer Configure */
#define I2C_CR2_LTCFG_Msk           (0x1UL << I2C_CR2_LTCFG_Pos)                /*!< 0x00001000 */
#define I2C_CR2_LTCFG               I2C_CR2_LTCFG_Msk

/* ============================================  Bit definition for I2C_SADDR1 register  ============================================ */
#define I2C_SADDR1_ADDR_Pos         (0U)                                        /*!< Slave Address Setup */
#define I2C_SADDR1_ADDR_Msk         (0x3FFUL << I2C_SADDR1_ADDR_Pos)            /*!< 0x000003FF */
#define I2C_SADDR1_ADDR             I2C_SADDR1_ADDR_Msk
#define I2C_SADDR1_ADDRLEN_Pos      (15U)                                       /*!< Slave Address Length Configure */
#define I2C_SADDR1_ADDRLEN_Msk      (0x1UL << I2C_SADDR1_ADDRLEN_Pos)           /*!< 0x00008000 */
#define I2C_SADDR1_ADDRLEN          I2C_SADDR1_ADDRLEN_Msk

/* ============================================  Bit definition for I2C_SADDR2 register  ============================================ */
#define I2C_SADDR2_ADDRNUM_Pos      (0U)                                        /*!< Slave Address Number Configure */
#define I2C_SADDR2_ADDRNUM_Msk      (0x1UL << I2C_SADDR2_ADDRNUM_Pos)           /*!< 0x00000001 */
#define I2C_SADDR2_ADDRNUM          I2C_SADDR2_ADDRNUM_Msk
#define I2C_SADDR2_ADDR2_Pos        (1U)                                        /*!< Slave Dual Address Mode Address Setup */
#define I2C_SADDR2_ADDR2_Msk        (0x7FUL << I2C_SADDR2_ADDR2_Pos)            /*!< 0x000000FE */
#define I2C_SADDR2_ADDR2            I2C_SADDR2_ADDR2_Msk

/* =============================================  Bit definition for I2C_DR register  ============================================= */
#define I2C_DR_DATA_Pos             (0U)                                        /*!< Data register */
#define I2C_DR_DATA_Msk             (0xFFUL << I2C_DR_DATA_Pos)                 /*!< 0x000000FF */
#define I2C_DR_DATA                 I2C_DR_DATA_Msk

/* =============================================  Bit definition for I2C_SR1 register  ============================================= */
#define I2C_SR1_STARTFLG_Pos        (0U)                                        /*!< Start Bit Sent Finished Flag */
#define I2C_SR1_STARTFLG_Msk        (0x1UL << I2C_SR1_STARTFLG_Pos)             /*!< 0x00000001 */
#define I2C_SR1_STARTFLG            I2C_SR1_STARTFLG_Msk
#define I2C_SR1_ADDRFLG_Pos         (1U)                                        /*!< Address Transfer Complete / Receive Match Flag */
#define I2C_SR1_ADDRFLG_Msk         (0x1UL << I2C_SR1_ADDRFLG_Pos)              /*!< 0x00000002 */
#define I2C_SR1_ADDRFLG             I2C_SR1_ADDRFLG_Msk
#define I2C_SR1_BTCFLG_Pos          (2U)                                        /*!< Byte Transfer Complete Flag */
#define I2C_SR1_BTCFLG_Msk          (0x1UL << I2C_SR1_BTCFLG_Pos)               /*!< 0x00000004 */
#define I2C_SR1_BTCFLG              I2C_SR1_BTCFLG_Msk
#define I2C_SR1_ADDR10FLG_Pos       (3U)                                        /*!< 10-Bit Address Header Transmit Flag */
#define I2C_SR1_ADDR10FLG_Msk       (0x1UL << I2C_SR1_ADDR10FLG_Pos)            /*!< 0x00000008 */
#define I2C_SR1_ADDR10FLG           I2C_SR1_ADDR10FLG_Msk
#define I2C_SR1_STOPFLG_Pos         (4U)                                        /*!< Stop Bit Detection Flag */
#define I2C_SR1_STOPFLG_Msk         (0x1UL << I2C_SR1_STOPFLG_Pos)              /*!< 0x00000010 */
#define I2C_SR1_STOPFLG             I2C_SR1_STOPFLG_Msk
#define I2C_SR1_RXBNEFLG_Pos        (6U)                                        /*!< Receive Buffer Not Empty Flag */
#define I2C_SR1_RXBNEFLG_Msk        (0x1UL << I2C_SR1_RXBNEFLG_Pos)             /*!< 0x00000040 */
#define I2C_SR1_RXBNEFLG            I2C_SR1_RXBNEFLG_Msk
#define I2C_SR1_TXBEFLG_Pos         (7U)                                        /*!< Transmit Buffer Empty Flag */
#define I2C_SR1_TXBEFLG_Msk         (0x1UL << I2C_SR1_TXBEFLG_Pos)              /*!< 0x00000080 */
#define I2C_SR1_TXBEFLG             I2C_SR1_TXBEFLG_Msk
#define I2C_SR1_BERRFLG_Pos         (8U)                                        /*!< Bus Error Flag */
#define I2C_SR1_BERRFLG_Msk         (0x1UL << I2C_SR1_BERRFLG_Pos)              /*!< 0x00000100 */
#define I2C_SR1_BERRFLG             I2C_SR1_BERRFLG_Msk
#define I2C_SR1_ALFLG_Pos           (9U)                                        /*!< Master Mode Arbitration Lost Flag */
#define I2C_SR1_ALFLG_Msk           (0x1UL << I2C_SR1_ALFLG_Pos)                /*!< 0x00000200 */
#define I2C_SR1_ALFLG               I2C_SR1_ALFLG_Msk
#define I2C_SR1_AEFLG_Pos           (10U)                                       /*!< Acknowledge Error Flag */
#define I2C_SR1_AEFLG_Msk           (0x1UL << I2C_SR1_AEFLG_Pos)                /*!< 0x00000400 */
#define I2C_SR1_AEFLG               I2C_SR1_AEFLG_Msk
#define I2C_SR1_OVRURFLG_Pos        (11U)                                       /*!< Overrun/Underrun Flag */
#define I2C_SR1_OVRURFLG_Msk        (0x1UL << I2C_SR1_OVRURFLG_Pos)             /*!< 0x00000800 */
#define I2C_SR1_OVRURFLG            I2C_SR1_OVRURFLG_Msk
#define I2C_SR1_PECEFLG_Pos         (12U)                                       /*!< PEC Error in Reception Flag */
#define I2C_SR1_PECEFLG_Msk         (0x1UL << I2C_SR1_PECEFLG_Pos)              /*!< 0x00001000 */
#define I2C_SR1_PECEFLG             I2C_SR1_PECEFLG_Msk
#define I2C_SR1_TTEFLG_Pos          (14U)                                       /*!< Timeout or Tlow Error Flag */
#define I2C_SR1_TTEFLG_Msk          (0x1UL << I2C_SR1_TTEFLG_Pos)               /*!< 0x00004000 */
#define I2C_SR1_TTEFLG              I2C_SR1_TTEFLG_Msk
#define I2C_SR1_SMBALTFLG_Pos       (15U)                                       /*!< SMBus Alert Occur Flag */
#define I2C_SR1_SMBALTFLG_Msk       (0x1UL << I2C_SR1_SMBALTFLG_Pos)            /*!< 0x00008000 */
#define I2C_SR1_SMBALTFLG           I2C_SR1_SMBALTFLG_Msk

/* =============================================  Bit definition for I2C_SR2 register  ============================================= */
#define I2C_SR2_MSFLG_Pos           (0U)                                        /*!< Master Slave Mode Flag */
#define I2C_SR2_MSFLG_Msk           (0x1UL << I2C_SR2_MSFLG_Pos)                /*!< 0x00000001 */
#define I2C_SR2_MSFLG               I2C_SR2_MSFLG_Msk
#define I2C_SR2_BUSBSYFLG_Pos       (1U)                                        /*!< Bus Busy Flag */
#define I2C_SR2_BUSBSYFLG_Msk       (0x1UL << I2C_SR2_BUSBSYFLG_Pos)            /*!< 0x00000002 */
#define I2C_SR2_BUSBSYFLG           I2C_SR2_BUSBSYFLG_Msk
#define I2C_SR2_TRFLG_Pos           (2U)                                        /*!< Transmitter / Receiver Mode Flag */
#define I2C_SR2_TRFLG_Msk           (0x1UL << I2C_SR2_TRFLG_Pos)                /*!< 0x00000004 */
#define I2C_SR2_TRFLG               I2C_SR2_TRFLG_Msk
#define I2C_SR2_GENCALLFLG_Pos      (4U)                                        /*!< Slave Mode Received General Call Address Flag */
#define I2C_SR2_GENCALLFLG_Msk      (0x1UL << I2C_SR2_GENCALLFLG_Pos)           /*!< 0x00000010 */
#define I2C_SR2_GENCALLFLG          I2C_SR2_GENCALLFLG_Msk
#define I2C_SR2_SMBDADDRFLG_Pos     (5U)                                        /*!< SMBus Device Received Default Address Flag in Slave Mode */
#define I2C_SR2_SMBDADDRFLG_Msk     (0x1UL << I2C_SR2_SMBDADDRFLG_Pos)          /*!< 0x00000020 */
#define I2C_SR2_SMBDADDRFLG         I2C_SR2_SMBDADDRFLG_Msk
#define I2C_SR2_SMMHADDRFLG_Pos     (6U)                                        /*!< SMBus Device Received Master Header Flag in Slave Mode */
#define I2C_SR2_SMMHADDRFLG_Msk     (0x1UL << I2C_SR2_SMMHADDRFLG_Pos)          /*!< 0x00000040 */
#define I2C_SR2_SMMHADDRFLG         I2C_SR2_SMMHADDRFLG_Msk
#define I2C_SR2_DUALADDRFLG_Pos     (7U)                                        /*!< Slave Mode Received Dual Address Match Flag */
#define I2C_SR2_DUALADDRFLG_Msk     (0x1UL << I2C_SR2_DUALADDRFLG_Pos)          /*!< 0x00000080 */
#define I2C_SR2_DUALADDRFLG         I2C_SR2_DUALADDRFLG_Msk
#define I2C_SR2_PECVAL_Pos          (8U)                                        /*!< Save Packet Error Checking Value */
#define I2C_SR2_PECVAL_Msk          (0xFFUL << I2C_SR2_PECVAL_Pos)              /*!< 0x0000FF00 */
#define I2C_SR2_PECVAL              I2C_SR2_PECVAL_Msk

/* ===========================================  Bit definition for I2C_CLKCR register  ============================================ */
#define I2C_CLKCR_CLKCFG_Pos          (0U)                                        /*!< Clock Setup in Fast/Standard Master Mode */
#define I2C_CLKCR_CLKCFG_Msk          (0xFFFUL << I2C_CLKCR_CLKCFG_Pos)             /*!< 0x00000FFF */
#define I2C_CLKCR_CLKCFG              I2C_CLKCR_CLKCFG_Msk
#define I2C_CLKCR_FDUTYCFG_Pos      (14U)                                       /*!< Fast Mode Duty Cycle Configure */
#define I2C_CLKCR_FDUTYCFG_Msk      (0x1UL << I2C_CLKCR_FDUTYCFG_Pos)           /*!< 0x00004000 */
#define I2C_CLKCR_FDUTYCFG          I2C_CLKCR_FDUTYCFG_Msk
#define I2C_CLKCR_SPEEDCFG_Pos      (15U)                                       /*!< Master Mode Speed Configure */
#define I2C_CLKCR_SPEEDCFG_Msk      (0x1UL << I2C_CLKCR_SPEEDCFG_Pos)           /*!< 0x00008000 */
#define I2C_CLKCR_SPEEDCFG          I2C_CLKCR_SPEEDCFG_Msk

/* ===========================================  Bit definition for I2C_RISETMAX register  =========================================== */
#define I2C_RISETMAX_RISETMAX_Pos   (0U)                                        /*!< Master Mode Maximum Rise Time in Fast/Standard Mode */
#define I2C_RISETMAX_RISETMAX_Msk   (0xFFUL << I2C_RISETMAX_RISETMAX_Pos)       /*!< 0x000000FF */
#define I2C_RISETMAX_RISETMAX       I2C_RISETMAX_RISETMAX_Msk

/* ============================================  Bit definition for I2C_FILTER register  ============================================ */
#define I2C_FILTER_DNFCFG_Pos       (0U)                                        /*!< Digital Noise Filter Filtering Capability Configure */
#define I2C_FILTER_DNFCFG_Msk       (0xFUL << I2C_FILTER_DNFCFG_Pos)            /*!< 0x0000000F */
#define I2C_FILTER_DNFCFG           I2C_FILTER_DNFCFG_Msk
#define I2C_FILTER_ANFDISEN_Pos     (4U)                                        /*!< Analog Noise Filter Disable */
#define I2C_FILTER_ANFDISEN_Msk     (0x1UL << I2C_FILTER_ANFDISEN_Pos)          /*!< 0x00000010 */
#define I2C_FILTER_ANFDISEN         I2C_FILTER_ANFDISEN_Msk

/* ================================================================================================================================== */
/* =============================================================  ADC  ============================================================== */
/* ================================================================================================================================== */
/* =============================================  Bit definition for ADC_SR register  ============================================== */
#define ADC_SR_RDYFLG_Pos           (0U)                                        /*!< ADC Ready Flag */
#define ADC_SR_RDYFLG_Msk           (0x1UL << ADC_SR_RDYFLG_Pos)                /*!< 0x00000001 */
#define ADC_SR_RDYFLG               ADC_SR_RDYFLG_Msk
#define ADC_SR_EOSFLG_Pos           (1U)                                        /*!< ADC group regular end of sampling flag */
#define ADC_SR_EOSFLG_Msk           (0x1UL << ADC_SR_EOSFLG_Pos)                /*!< 0x00000002 */
#define ADC_SR_EOSFLG               ADC_SR_EOSFLG_Msk
#define ADC_SR_EORCFLG_Pos          (2U)                                        /*!< ADC group regular end of unitary conversion flag */
#define ADC_SR_EORCFLG_Msk          (0x1UL << ADC_SR_EORCFLG_Pos)               /*!< 0x00000004 */
#define ADC_SR_EORCFLG              ADC_SR_EORCFLG_Msk
#define ADC_SR_EORSFLG_Pos          (3U)                                        /*!< ADC group regular end of sequence conversions flag */
#define ADC_SR_EORSFLG_Msk          (0x1UL << ADC_SR_EORSFLG_Pos)               /*!< 0x00000008 */
#define ADC_SR_EORSFLG              ADC_SR_EORSFLG_Msk
#define ADC_SR_OVRFLG_Pos           (4U)                                        /*!< ADC group regular overrun flag */
#define ADC_SR_OVRFLG_Msk           (0x1UL << ADC_SR_OVRFLG_Pos)                /*!< 0x00000010 */
#define ADC_SR_OVRFLG               ADC_SR_OVRFLG_Msk
#define ADC_SR_FINSEQNUM1_Pos       (8U)                                        /*!< NUM.1 of Sequential Section Sampling Finish */
#define ADC_SR_FINSEQNUM1_Msk       (0x1UL << ADC_SR_FINSEQNUM1_Pos)            /*!< 0x00000100 */
#define ADC_SR_FINSEQNUM1           ADC_SR_FINSEQNUM1_Msk
#define ADC_SR_FINSEQNUM2_Pos       (9U)                                        /*!< NUM.2 of Sequential Section Sampling Finish */
#define ADC_SR_FINSEQNUM2_Msk       (0x1UL << ADC_SR_FINSEQNUM2_Pos)            /*!< 0x00000200 */
#define ADC_SR_FINSEQNUM2           ADC_SR_FINSEQNUM2_Msk
#define ADC_SR_FINSEQNUM3_Pos       (10U)                                       /*!< NUM.3 of Sequential Section Sampling Finish */
#define ADC_SR_FINSEQNUM3_Msk       (0x1UL << ADC_SR_FINSEQNUM3_Pos)            /*!< 0x00000400 */
#define ADC_SR_FINSEQNUM3           ADC_SR_FINSEQNUM3_Msk

/* =============================================  Bit definition for ADC_IER register  ============================================== */
#define ADC_IER_RDYIEN_Pos            (0U)                                        /*!< ADC Ready Interrupt Enable */
#define ADC_IER_RDYIEN_Msk            (0x1UL << ADC_IER_RDYIEN_Pos)                 /*!< 0x00000001 */
#define ADC_IER_RDYIEN                ADC_IER_RDYIEN_Msk
#define ADC_IER_EOSFIEN_Pos         (1U)                                        /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOSFIEN_Msk         (0x1UL << ADC_IER_EOSFIEN_Pos)              /*!< 0x00000002 */
#define ADC_IER_EOSFIEN             ADC_IER_EOSFIEN_Msk
#define ADC_IER_EORCIEN_Pos         (2U)                                        /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EORCIEN_Msk         (0x1UL << ADC_IER_EORCIEN_Pos)              /*!< 0x00000004 */
#define ADC_IER_EORCIEN             ADC_IER_EORCIEN_Msk
#define ADC_IER_EORSIEN_Pos         (3U)                                        /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_EORSIEN_Msk         (0x1UL << ADC_IER_EORSIEN_Pos)              /*!< 0x00000008 */
#define ADC_IER_EORSIEN             ADC_IER_EORSIEN_Msk
#define ADC_IER_ORIEN_Pos           (4U)                                        /*!< ADC group regular overrun interrupt */
#define ADC_IER_ORIEN_Msk           (0x1UL << ADC_IER_ORIEN_Pos)                /*!< 0x00000010 */
#define ADC_IER_ORIEN               ADC_IER_ORIEN_Msk
#define ADC_IER_FINSEQNUM1IEN_Pos   (8U)                                        /*!< NUM.1 of Sequential Section Sampling Finish interrupt */
#define ADC_IER_FINSEQNUM1IEN_Msk   (0x1UL << ADC_IER_FINSEQNUM1IEN_Pos)        /*!< 0x00000100 */
#define ADC_IER_FINSEQNUM1IEN       ADC_IER_FINSEQNUM1IEN_Msk
#define ADC_IER_FINSEQNUM2IEN_Pos   (9U)                                        /*!< NUM.2 of Sequential Section Sampling Finish interrupt */
#define ADC_IER_FINSEQNUM2IEN_Msk   (0x1UL << ADC_IER_FINSEQNUM2IEN_Pos)        /*!< 0x00000200 */
#define ADC_IER_FINSEQNUM2IEN       ADC_IER_FINSEQNUM2IEN_Msk
#define ADC_IER_FINSEQNUM3IEN_Pos   (10U)                                       /*!< NUM.3 of Sequential Section Sampling Finish interrupt */
#define ADC_IER_FINSEQNUM3IEN_Msk   (0x1UL << ADC_IER_FINSEQNUM3IEN_Pos)        /*!< 0x00000400 */
#define ADC_IER_FINSEQNUM3IEN       ADC_IER_FINSEQNUM3IEN_Msk

/* ==========================================================  CR  =========================================================== */
#define ADC_CR_ADCEN_Pos            (0U)                                        /*!< ADEN (Bit 0)                       */
#define ADC_CR_ADCEN_Msk            (0x1UL << ADC_CR_ADCEN_Pos)                 /*!< ADEN (Bitfield-Mask: 0x01)         */
#define ADC_CR_ADCEN                ADC_CR_ADCEN_Msk
#define ADC_CR_START_Pos            (2U)                                        /*!< ADSTART (Bit 2)                    */
#define ADC_CR_START_Msk            (0x1UL << ADC_CR_START_Pos)                 /*!< ADSTART (Bitfield-Mask: 0x01)      */
#define ADC_CR_START                ADC_CR_START_Msk
#define ADC_CR_STOP_Pos             (4U)                                        /*!< ADSTP (Bit 4)                      */
#define ADC_CR_STOP_Msk             (0x1UL << ADC_CR_STOP_Pos)                  /*!< ADSTP (Bitfield-Mask: 0x01)        */
#define ADC_CR_STOP                 ADC_CR_STOP_Msk
#define ADC_CR_TSENP_Pos            (16U)                                       /*!< CONV_FAST_MD (Bit 30)              */
#define ADC_CR_TSENP_Msk            (0x1UL << ADC_CR_TSENP_Pos)                 /*!< CONV_FAST_MD (Bitfield-Mask: 0x01) */
#define ADC_CR_TSENP                ADC_CR_TSENP_Msk
#define ADC_CR_VREFSEL_Pos          (17U)                                       /*!< VREF_SEL (Bit 31)                  */
#define ADC_CR_VREFSEL_Msk          (0x1UL << ADC_CR_VREFSEL_Pos)               /*!< VREF_SEL (Bitfield-Mask: 0x01)     */
#define ADC_CR_VREFSEL              ADC_CR_VREFSEL_Msk
#define ADC_CR_MODESEL_Pos          (30U)
#define ADC_CR_MODESEL_Msk          (0x3UL << ADC_CR_MODESEL_Pos)
#define ADC_CR_MODESEL_0            (0x1UL << ADC_CR_MODESEL_Pos)
#define ADC_CR_MODESEL_1            (0x2UL << ADC_CR_MODESEL_Pos)
#define ADC_CR_MODESEL              ADC_CR_MODESEL_Msk
/* =========================================================  CFGR1  ========================================================= */
#define ADC_CFG1_DMAEN_Pos          (0U)                                        /*!< DMAEN (Bit 0)                      */
#define ADC_CFG1_DMAEN_Msk          (0x1UL << ADC_CFG1_DMAEN_Pos)               /*!< DMAEN (Bitfield-Mask: 0x01)        */
#define ADC_CFG1_DMAEN              ADC_CFG1_DMAEN_Msk
#define ADC_CFG1_DMACFG_Pos         (1U)                                        /*!< DMACFG (Bit 1)                     */
#define ADC_CFG1_DMACFG_Msk         (0x1UL << ADC_CFG1_DMACFG_Pos)              /*!< DMACFG (Bitfield-Mask: 0x01)       */
#define ADC_CFG1_DMACFG             ADC_CFG1_DMACFG_Msk
#define ADC_CFG1_ALIGN_Pos          (5U)                                        /*!< ALIGN (Bit 5)                      */
#define ADC_CFG1_ALIGN_Msk          (0x1UL << ADC_CFG1_ALIGN_Pos)               /*!< ALIGN (Bitfield-Mask: 0x01)        */
#define ADC_CFG1_ALIGN              ADC_CFG1_ALIGN_Msk
#define ADC_CFG1_EXTSEL1_Pos        (6U)                                        /*!< EXTSEL1 (Bit 6)                    */
#define ADC_CFG1_EXTSEL1_Msk        (0x7UL << ADC_CFG1_EXTSEL1_Pos)             /*!< EXTSEL1 (Bitfield-Mask: 0x0f)      */
#define ADC_CFG1_EXTSEL1_0          (0x1UL << ADC_CFG1_EXTSEL1_Pos)
#define ADC_CFG1_EXTSEL1_1          (0x2UL << ADC_CFG1_EXTSEL1_Pos)
#define ADC_CFG1_EXTSEL1_2          (0x4UL << ADC_CFG1_EXTSEL1_Pos)
#define ADC_CFG1_EXTSEL1            ADC_CFG1_EXTSEL1_Msk
#define ADC_CFG1_EXTEN1_Pos         (10U)                                       /*!< EXTEN (Bit 10)                     */
#define ADC_CFG1_EXTEN1_Msk         (0x3UL << ADC_CFG1_EXTEN1_Pos)              /*!< EXTEN (Bitfield-Mask: 0x03)        */
#define ADC_CFG1_EXTEN1_0           (0x1UL << ADC_CFG1_EXTEN1_Pos)
#define ADC_CFG1_EXTEN1_1           (0x2UL << ADC_CFG1_EXTEN1_Pos)
#define ADC_CFG1_EXTEN1             ADC_CFG1_EXTEN1_Msk
#define ADC_CFG1_OVRMOD_Pos         (12U)                                       /*!< OVRMOD (Bit 12)                    */
#define ADC_CFG1_OVRMOD_Msk         (0x1UL << ADC_CFG1_OVRMOD_Pos)              /*!< OVRMOD (Bitfield-Mask: 0x01)       */
#define ADC_CFG1_OVRMOD             ADC_CFG1_OVRMOD_Msk
#define ADC_CFG1_CONT_Pos           (13U)                                       /*!< CONT (Bit 13)                      */
#define ADC_CFG1_CONT_Msk           (0x1UL << ADC_CFG1_CONT_Pos)                /*!< CONT (Bitfield-Mask: 0x01)         */
#define ADC_CFG1_CONT               ADC_CFG1_CONT_Msk
#define ADC_CFG1_DLYCM_Pos          (14U)                                       /*!< AUTDLY (Bit 14)                    */
#define ADC_CFG1_DLYCM_Msk          (0x1UL << ADC_CFG1_DLYCM_Pos)               /*!< AUTDLY (Bitfield-Mask: 0x01)       */
#define ADC_CFG1_DLYCM              ADC_CFG1_DLYCM_Msk
#define ADC_CFG1_AUTOFF_Pos         (15U)                                       /*!< AUTOFF (Bit 15)                    */
#define ADC_CFG1_AUTOFF_Msk         (0x1UL << ADC_CFG1_AUTOFF_Pos)              /*!< AUTOFF (Bitfield-Mask: 0x01)       */
#define ADC_CFG1_AUTOFF             ADC_CFG1_AUTOFF_Msk
#define ADC_CFG1_DISCEN_Pos         (16U)                                       /*!< DISCEN (Bit 16)                    */
#define ADC_CFG1_DISCEN_Msk         (0x1UL << ADC_CFG1_DISCEN_Pos)              /*!< DISCEN (Bitfield-Mask: 0x01)       */
#define ADC_CFG1_DISCEN             ADC_CFG1_DISCEN_Msk
#define ADC_CFG1_DISCNUM_Pos        (17U)                                       /*!< DISCNUM (Bit 17)                   */
#define ADC_CFG1_DISCNUM_Msk        (0x7UL << ADC_CFG1_DISCNUM_Pos)             /*!< DISCNUM (Bitfield-Mask: 0x07)      */
#define ADC_CFG1_DISCNUM_0          (0x1UL << ADC_CFG1_DISCNUM_Pos)
#define ADC_CFG1_DISCNUM_1          (0x2UL << ADC_CFG1_DISCNUM_Pos)
#define ADC_CFG1_DISCNUM_2          (0x4UL << ADC_CFG1_DISCNUM_Pos)
#define ADC_CFG1_DISCNUM            ADC_CFG1_DISCNUM_Msk
/* =========================================================  CFGR2  ========================================================= */
#define ADC_CFG2_SEQEN_Pos          (0U)                                        /*!< SEQEN (Bit 0)                      */
#define ADC_CFG2_SEQEN_Msk          (0x1UL << ADC_CFG2_SEQEN_Pos)               /*!< SEQEN (Bitfield-Mask: 0x01)        */
#define ADC_CFG2_SEQEN              ADC_CFG2_SEQEN_Msk
#define ADC_CFG2_TGAP_Pos           (8U)                                        /*!< TGAP (Bit 8)                       */
#define ADC_CFG2_TGAP_Msk           (0xFUL << ADC_CFG2_TGAP_Pos)                /*!< TGAP (Bitfield-Mask: 0x0f)         */
#define ADC_CFG2_TGAP_0             (0x1UL << ADC_CFG2_TGAP_Pos)
#define ADC_CFG2_TGAP_1             (0x2UL << ADC_CFG2_TGAP_Pos)
#define ADC_CFG2_TGAP_2             (0x4UL << ADC_CFG2_TGAP_Pos)
#define ADC_CFG2_TGAP_3             (0x8UL << ADC_CFG2_TGAP_Pos)
#define ADC_CFG2_TGAP               ADC_CFG2_TGAP_Msk
/* =========================================================  SMPR1  ========================================================= */
#define ADC_SMP1_SMP0_Pos           (0U)                                        /*!< SMP0 (Bit 0)                       */
#define ADC_SMP1_SMP0_Msk           (0x7UL << ADC_SMP1_SMP0_Pos)                /*!< SMP0 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP0_0             (0x1UL << ADC_SMP1_SMP0_Pos)
#define ADC_SMP1_SMP0_1             (0x2UL << ADC_SMP1_SMP0_Pos)
#define ADC_SMP1_SMP0_2             (0x4UL << ADC_SMP1_SMP0_Pos)
#define ADC_SMP1_SMP0               ADC_SMP1_SMP0_Msk
#define ADC_SMP1_SMP1_Pos           (4U)                                        /*!< SMP1 (Bit 4)                       */
#define ADC_SMP1_SMP1_Msk           (0x7UL << ADC_SMP1_SMP1_Pos)                /*!< SMP1 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP1_0             (0x1UL << ADC_SMP1_SMP1_Msk)
#define ADC_SMP1_SMP1_1             (0x2UL << ADC_SMP1_SMP1_Msk)
#define ADC_SMP1_SMP1_2             (0x4UL << ADC_SMP1_SMP1_Msk)
#define ADC_SMP1_SMP1               ADC_SMP1_SMP1_Msk
#define ADC_SMP1_SMP2_Pos           (8U)                                        /*!< SMP2 (Bit 8)                       */
#define ADC_SMP1_SMP2_Msk           (0x7UL << ADC_SMP1_SMP2_Pos)                /*!< SMP2 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP2_0             (0x1UL << ADC_SMP1_SMP2_Pos)
#define ADC_SMP1_SMP2_1             (0x2UL << ADC_SMP1_SMP2_Pos)
#define ADC_SMP1_SMP2_2             (0x4UL << ADC_SMP1_SMP2_Pos)
#define ADC_SMP1_SMP2               ADC_SMP1_SMP2_Msk
#define ADC_SMP1_SMP3_Pos           (12U)                                       /*!< SMP3 (Bit 12)                      */
#define ADC_SMP1_SMP3_Msk           (0x7UL << ADC_SMP1_SMP3_Pos)                /*!< SMP3 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP3_0             (0x1UL << ADC_SMP1_SMP3_Pos)
#define ADC_SMP1_SMP3_1             (0x2UL << ADC_SMP1_SMP3_Pos)
#define ADC_SMP1_SMP3_2             (0x4UL << ADC_SMP1_SMP3_Pos)
#define ADC_SMP1_SMP3               ADC_SMP1_SMP3_Msk
#define ADC_SMP1_SMP4_Pos           (16U)                                       /*!< SMP4 (Bit 16)                      */
#define ADC_SMP1_SMP4_Msk           (0x7UL << ADC_SMP1_SMP4_Pos)                /*!< SMP4 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP4_0             (0x1UL << ADC_SMP1_SMP4_Pos)
#define ADC_SMP1_SMP4_1             (0x2UL << ADC_SMP1_SMP4_Pos)
#define ADC_SMP1_SMP4_2             (0x4UL << ADC_SMP1_SMP4_Pos)
#define ADC_SMP1_SMP4               ADC_SMP1_SMP4_Msk
#define ADC_SMP1_SMP5_Pos           (20U)                                       /*!< SMP5 (Bit 20)                      */
#define ADC_SMP1_SMP5_Msk           (0x7UL << ADC_SMP1_SMP5_Pos)                /*!< SMP5 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP5_0             (0x1UL << ADC_SMP1_SMP5_Pos)
#define ADC_SMP1_SMP5_1             (0x2UL << ADC_SMP1_SMP5_Pos)
#define ADC_SMP1_SMP5_2             (0x4UL << ADC_SMP1_SMP5_Pos)
#define ADC_SMP1_SMP5               ADC_SMP1_SMP5_Msk
#define ADC_SMP1_SMP6_Pos           (24U)                                       /*!< SMP6 (Bit 24)                      */
#define ADC_SMP1_SMP6_Msk           (0x7UL << ADC_SMP1_SMP6_Pos)                /*!< SMP6 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP6_0             (0x1UL << ADC_SMP1_SMP6_Pos)
#define ADC_SMP1_SMP6_1             (0x2UL << ADC_SMP1_SMP6_Pos)
#define ADC_SMP1_SMP6_2             (0x4UL << ADC_SMP1_SMP6_Pos)
#define ADC_SMP1_SMP6               ADC_SMP1_SMP6_Msk
#define ADC_SMP1_SMP7_Pos           (28U)                                       /*!< SMP7 (Bit 28)                      */
#define ADC_SMP1_SMP7_Msk           (0x7UL << ADC_SMP1_SMP7_Pos)                /*!< SMP7 (Bitfield-Mask: 0x07)         */
#define ADC_SMP1_SMP7_0             (0x1UL << ADC_SMP1_SMP7_Pos)
#define ADC_SMP1_SMP7_1             (0x2UL << ADC_SMP1_SMP7_Pos)
#define ADC_SMP1_SMP7_2             (0x4UL << ADC_SMP1_SMP7_Pos)
#define ADC_SMP1_SMP7               ADC_SMP1_SMP7_Msk
/* =========================================================  SMPR2  ========================================================= */
#define ADC_SMP2_SMP8_Pos           (0U)                                        /*!< SMP8 (Bit 0)                       */
#define ADC_SMP2_SMP8_Msk           (0x7UL << ADC_SMP2_SMP8_Pos)                /*!< SMP8 (Bitfield-Mask: 0x07)         */
#define ADC_SMP2_SMP8_0             (0x1UL << ADC_SMP2_SMP8_Pos)
#define ADC_SMP2_SMP8_1             (0x2UL << ADC_SMP2_SMP8_Pos)
#define ADC_SMP2_SMP8_2             (0x4UL << ADC_SMP2_SMP8_Pos)
#define ADC_SMP2_SMP8               ADC_SMP2_SMP8_Msk
#define ADC_SMP2_SMP9_Pos           (4U)                                        /*!< SMP9 (Bit 4)                       */
#define ADC_SMP2_SMP9_Msk           (0x7UL << ADC_SMP2_SMP9_Pos)                /*!< SMP9 (Bitfield-Mask: 0x07)         */
#define ADC_SMP2_SMP9_0             (0x1UL << ADC_SMP2_SMP9_Pos)
#define ADC_SMP2_SMP9_1             (0x2UL << ADC_SMP2_SMP9_Pos)
#define ADC_SMP2_SMP9_2             (0x4UL << ADC_SMP2_SMP9_Pos)
#define ADC_SMP2_SMP9               ADC_SMP2_SMP9_Msk
#define ADC_SMP2_SMP10_Pos          (8U)                                        /*!< SMP10 (Bit 8)                      */
#define ADC_SMP2_SMP10_Msk          (0x7UL << ADC_SMP2_SMP10_Pos)               /*!< SMP10 (Bitfield-Mask: 0x07)        */
#define ADC_SMP2_SMP10_0            (0x1UL << ADC_SMP2_SMP10_Pos)
#define ADC_SMP2_SMP10_1            (0x2UL << ADC_SMP2_SMP10_Pos)
#define ADC_SMP2_SMP10_2            (0x4UL << ADC_SMP2_SMP10_Pos)
#define ADC_SMP2_SMP10              ADC_SMP2_SMP10_Msk
#define ADC_SMP2_SMP11_Pos          (12U)                                       /*!< SMP11 (Bit 12)                     */
#define ADC_SMP2_SMP11_Msk          (0x7UL << ADC_SMP2_SMP11_Pos)               /*!< SMP11 (Bitfield-Mask: 0x07)        */
#define ADC_SMP2_SMP11_0            (0x1UL << ADC_SMP2_SMP11_Pos)
#define ADC_SMP2_SMP11_1            (0x2UL << ADC_SMP2_SMP11_Pos)
#define ADC_SMP2_SMP11_2            (0x4UL << ADC_SMP2_SMP11_Pos)
#define ADC_SMP2_SMP11              ADC_SMP2_SMP11_Msk
#define ADC_SMP2_SMP12_Pos          (16U)                                       /*!< SMP12 (Bit 16)                     */
#define ADC_SMP2_SMP12_Msk          (0x7UL << ADC_SMP2_SMP12_Pos)               /*!< SMP12 (Bitfield-Mask: 0x07)        */
#define ADC_SMP2_SMP12_0            (0x1UL << ADC_SMP2_SMP12_Pos)
#define ADC_SMP2_SMP12_1            (0x2UL << ADC_SMP2_SMP12_Pos)
#define ADC_SMP2_SMP12_2            (0x4UL << ADC_SMP2_SMP12_Pos)
#define ADC_SMP2_SMP12              ADC_SMP2_SMP12_Msk
/* =========================================================  CFGR3  ========================================================= */
#define ADC_CFG3_EXTEN2_Pos         (0U)                                        /*!< EXTEN2 (Bit 0)                     */
#define ADC_CFG3_EXTEN2_Msk         (0x3UL << ADC_CFG3_EXTEN2_Pos)              /*!< EXTEN2 (Bitfield-Mask: 0x03)       */
#define ADC_CFG3_EXTEN2_0           (0x1UL << ADC_CFG3_EXTEN2_Pos)
#define ADC_CFG3_EXTEN2_1           (0x2UL << ADC_CFG3_EXTEN2_Pos)
#define ADC_CFG3_EXTEN2             ADC_CFG3_EXTEN2_Msk
#define ADC_CFG3_EXTTRGSEL2_Pos     (2U)                                        /*!< EXTSEL2 (Bit 2)                    */
#define ADC_CFG3_EXTTRGSEL2_Msk     (0x7UL << ADC_CFG3_EXTTRGSEL2_Pos)          /*!< EXTSEL2 (Bitfield-Mask: 0x07)      */
#define ADC_CFG3_EXTTRGSEL2_0       (0x1UL << ADC_CFG3_EXTTRGSEL2_Pos)
#define ADC_CFG3_EXTTRGSEL2_1       (0x2UL << ADC_CFG3_EXTTRGSEL2_Pos)
#define ADC_CFG3_EXTTRGSEL2_2       (0x4UL << ADC_CFG3_EXTTRGSEL2_Pos)
#define ADC_CFG3_EXTTRGSEL2         ADC_CFG3_EXTTRGSEL2_Msk
#define ADC_CFG3_EXTEN3_Pos         (8U)                                        /*!< EXTEN3 (Bit 8)                     */
#define ADC_CFG3_EXTEN3_Msk         (0x3UL << ADC_CFG3_EXTEN3_Pos)              /*!< EXTEN3 (Bitfield-Mask: 0x03)       */
#define ADC_CFG3_EXTEN3_0           (0x1UL << ADC_CFG3_EXTEN3_Pos)
#define ADC_CFG3_EXTEN3_1           (0x2UL << ADC_CFG3_EXTEN3_Pos)
#define ADC_CFG3_EXTEN3             ADC_CFG3_EXTEN3_Msk
#define ADC_CFG3_EXTTRGSEL3_Pos     (10U)                                       /*!< EXTSEL3 (Bit 10)                   */
#define ADC_CFG3_EXTTRGSEL3_Msk     (0x7UL << ADC_CFG3_EXTTRGSEL3_Pos)          /*!< EXTSEL3 (Bitfield-Mask: 0x07)      */
#define ADC_CFG3_EXTTRGSEL3_0       (0x1UL << ADC_CFG3_EXTTRGSEL3_Pos)
#define ADC_CFG3_EXTTRGSEL3_1       (0x2UL << ADC_CFG3_EXTTRGSEL3_Pos)
#define ADC_CFG3_EXTTRGSEL3_2       (0x4UL << ADC_CFG3_EXTTRGSEL3_Pos)
#define ADC_CFG3_EXTTRGSEL3         ADC_CFG3_EXTTRGSEL3_Msk
/* =========================================================  SQR1  ========================================================== */
#define ADC_SQ1_LT3_Pos             (0U)                                        /*!< LT3 (Bit 0)                        */
#define ADC_SQ1_LT3_Msk             (0xFUL << ADC_SQ1_LT3_Pos)                  /*!< LT3 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_LT3_0               (0x1UL << ADC_SQ1_LT3_Pos)
#define ADC_SQ1_LT3_1               (0x2UL << ADC_SQ1_LT3_Pos)
#define ADC_SQ1_LT3_2               (0x4UL << ADC_SQ1_LT3_Pos)
#define ADC_SQ1_LT3_3               (0x8UL << ADC_SQ1_LT3_Pos)
#define ADC_SQ1_LT3                 ADC_SQ1_LT3_Msk
#define ADC_SQ1_SQ1_Pos             (4U)                                        /*!< SQ1 (Bit 4)                        */
#define ADC_SQ1_SQ1_Msk             (0xFUL << ADC_SQ1_SQ1_Pos)                  /*!< SQ1 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ1                 ADC_SQ1_SQ1_Msk
#define ADC_SQ1_SQ2_Pos             (8U)                                        /*!< SQ2 (Bit 8)                        */
#define ADC_SQ1_SQ2_Msk             (0xFUL << ADC_SQ1_SQ2_Pos)                  /*!< SQ2 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ2                 ADC_SQ1_SQ2_Msk
#define ADC_SQ1_SQ3_Pos             (12U)                                       /*!< SQ3 (Bit 12)                       */
#define ADC_SQ1_SQ3_Msk             (0xFUL << ADC_SQ1_SQ3_Pos)                  /*!< SQ3 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ3                 ADC_SQ1_SQ3_Msk
#define ADC_SQ1_SQ4_Pos             (16U)                                       /*!< SQ4 (Bit 16)                       */
#define ADC_SQ1_SQ4_Msk             (0xFUL << ADC_SQ1_SQ4_Pos)                  /*!< SQ4 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ4                 ADC_SQ1_SQ4_Msk
#define ADC_SQ1_SQ5_Pos             (20U)                                       /*!< SQ5 (Bit 20)                       */
#define ADC_SQ1_SQ5_Msk             (0xFUL << ADC_SQ1_SQ5_Pos)                  /*!< SQ5 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ5                 ADC_SQ1_SQ5_Msk
#define ADC_SQ1_SQ6_Pos             (24U)                                       /*!< SQ6 (Bit 24)                       */
#define ADC_SQ1_SQ6_Msk             (0xFUL << ADC_SQ1_SQ6_Pos)                  /*!< SQ6 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ6                 ADC_SQ1_SQ6_Msk
#define ADC_SQ1_SQ7_Pos             (28U)                                       /*!< SQ7 (Bit 28)                       */
#define ADC_SQ1_SQ7_Msk             (0xFUL << ADC_SQ1_SQ7_Pos)                  /*!< SQ7 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ1_SQ7                 ADC_SQ1_SQ7_Msk
/* =========================================================  SQR2  ========================================================== */
#define ADC_SQ2_SQ8_Pos             (0U)                                        /*!< SQ8 (Bit 0)                        */
#define ADC_SQ2_SQ8_Msk             (0xFUL << ADC_SQ2_SQ8_Pos)                  /*!< SQ8 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ2_SQ8                 ADC_SQ2_SQ8_Msk
#define ADC_SQ2_SQ9_Pos             (4U)                                        /*!< SQ9 (Bit 4)                        */
#define ADC_SQ2_SQ9_Msk             (0xFUL << ADC_SQ2_SQ9_Pos)                  /*!< SQ9 (Bitfield-Mask: 0x0f)          */
#define ADC_SQ2_SQ9                 ADC_SQ2_SQ9_Msk
#define ADC_SQ2_SQ10_Pos            (8U)                                        /*!< SQ10 (Bit 8)                       */
#define ADC_SQ2_SQ10_Msk            (0xFUL << ADC_SQ2_SQ10_Pos)                 /*!< SQ10 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ2_SQ10                ADC_SQ2_SQ10_Msk
#define ADC_SQ2_SQ11_Pos            (12U)                                       /*!< SQ11 (Bit 12)                      */
#define ADC_SQ2_SQ11_Msk            (0xFUL << ADC_SQ2_SQ11_Pos)                 /*!< SQ11 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ2_SQ11                ADC_SQ2_SQ11_Msk
#define ADC_SQ2_SQ12_Pos            (16U)                                       /*!< SQ12 (Bit 16)                      */
#define ADC_SQ2_SQ12_Msk            (0xFUL << ADC_SQ2_SQ12_Pos)                 /*!< SQ12 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ2_SQ12                ADC_SQ2_SQ12_Msk
#define ADC_SQ2_SQ13_Pos            (20U)                                       /*!< SQ13 (Bit 20)                      */
#define ADC_SQ2_SQ13_Msk            (0xFUL << ADC_SQ2_SQ13_Pos)                 /*!< SQ13 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ2_SQ13                ADC_SQ2_SQ13_Msk
#define ADC_SQ2_SQ14_Pos            (24U)                                       /*!< SQ14 (Bit 24)                      */
#define ADC_SQ2_SQ14_Msk            (0xFUL << ADC_SQ2_SQ14_Pos)                 /*!< SQ14 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ2_SQ14                ADC_SQ2_SQ14_Msk
/* =========================================================  SQR3  ========================================================== */
#define ADC_SQ3_SQ15_Pos            (0U)                                        /*!< SQ15 (Bit 0)                       */
#define ADC_SQ3_SQ15_Msk            (0xFUL << ADC_SQ3_SQ15_Pos)                 /*!< SQ15 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ3_SQ15                ADC_SQ3_SQ15_Msk
#define ADC_SQ3_SQ16_Pos            (4U)                                        /*!< SQ16 (Bit 4)                       */
#define ADC_SQ3_SQ16_Msk            (0xFUL << ADC_SQ3_SQ16_Pos)                 /*!< SQ16 (Bitfield-Mask: 0x0f)         */
#define ADC_SQ3_SQ16                ADC_SQ3_SQ16_Msk
/* ==========================================================  DR  =========================================================== */
#define ADC_DR_RDATA_Pos            (0U)                                        /*!< RDATA (Bit 0)                      */
#define ADC_DR_RDATA_Msk            (0xFFFFUL << ADC_DR_RDATA_Pos)              /*!< RDATA (Bitfield-Mask: 0xffff)      */
#define ADC_DR_RDATA                ADC_DR_RDATA_Msk
/* =========================================================  DATA0  ========================================================= */
#define ADC_DR0_SDATA0_Pos          (0U)                                        /*!< RDATA0 (Bit 0)                     */
#define ADC_DR0_SDATA0_Msk          (0xFFFFUL << ADC_DR0_SDATA0_Pos)            /*!< RDATA0 (Bitfield-Mask: 0xffff)     */
#define ADC_DR0_SDATA0              ADC_DR0_SDATA0_Msk
/* =========================================================  DATA1  ========================================================= */
#define ADC_DR1_SDATA1_Pos          (0U)                                        /*!< RDATA1 (Bit 0)                     */
#define ADC_DR1_SDATA1_Msk          (0xFFFFUL << ADC_DR1_SDATA1_Pos)            /*!< RDATA1 (Bitfield-Mask: 0xffff)     */
#define ADC_DR1_SDATA1              ADC_DR1_SDATA1_Msk
/* =========================================================  DATA2  ========================================================= */
#define ADC_DR2_SDATA2_Pos          (0U)                                        /*!< RDATA2 (Bit 0)                     */
#define ADC_DR2_SDATA2_Msk          (0xFFFFUL << ADC_DR2_SDATA2_Pos)            /*!< RDATA2 (Bitfield-Mask: 0xffff)     */
#define ADC_DR2_SDATA2              ADC_DR2_SDATA2_Msk
/* =========================================================  DATA3  ========================================================= */
#define ADC_DR3_SDATA3_Pos          (0U)                                        /*!< RDATA3 (Bit 0)                     */
#define ADC_DR3_SDATA3_Msk          (0xFFFFUL << ADC_DR3_SDATA3_Pos)            /*!< RDATA3 (Bitfield-Mask: 0xffff)     */
#define ADC_DR3_SDATA3              ADC_DR3_SDATA3_Msk
/* =========================================================  DATA4  ========================================================= */
#define ADC_DR4_SDATA4_Pos          (0U)                                        /*!< RDATA4 (Bit 0)                     */
#define ADC_DR4_SDATA4_Msk          (0xFFFFUL << ADC_DR4_SDATA4_Pos)            /*!< RDATA4 (Bitfield-Mask: 0xffff)     */
#define ADC_DR4_SDATA4              ADC_DR4_SDATA4_Msk
/* =========================================================  DATA5  ========================================================= */
#define ADC_DR5_SDATA5_Pos          (0U)                                        /*!< RDATA5 (Bit 0)                     */
#define ADC_DR5_SDATA5_Msk          (0xFFFFUL << ADC_DR5_SDATA5_Pos)            /*!< RDATA5 (Bitfield-Mask: 0xffff)     */
#define ADC_DR5_SDATA5              ADC_DR5_SDATA5_Msk
/* =========================================================  DATA6  ========================================================= */
#define ADC_DR6_SDATA6_Pos          (0U)                                        /*!< RDATA6 (Bit 0)                     */
#define ADC_DR6_SDATA6_Msk          (0xFFFFUL << ADC_DR6_SDATA6_Pos)            /*!< RDATA6 (Bitfield-Mask: 0xffff)     */
#define ADC_DR6_SDATA6              ADC_DR6_SDATA6_Msk
/* =========================================================  DATA7  ========================================================= */
#define ADC_DR7_SDATA7_Pos          (0U)                                        /*!< RDATA7 (Bit 0)                     */
#define ADC_DR7_SDATA7_Msk          (0xFFFFUL << ADC_DR7_SDATA7_Pos)            /*!< RDATA7 (Bitfield-Mask: 0xffff)     */
#define ADC_DR7_SDATA7              ADC_DR7_SDATA7_Msk
/* ========================================================  SEQ_NUM  ======================================================== */
#define ADC_SEQNUM_SEQNUM1_Pos      (0U)                                        /*!< SEQ_NUM1 (Bit 0)                   */
#define ADC_SEQNUM_SEQNUM1_Msk      (0x7UL << ADC_SEQNUM_SEQNUM1_Pos)           /*!< SEQ_NUM1 (Bitfield-Mask: 0x07)     */
#define ADC_SEQNUM_SEQNUM1_0        (0x1UL << ADC_SEQNUM_SEQNUM1_Pos)
#define ADC_SEQNUM_SEQNUM1_1        (0x2UL << ADC_SEQNUM_SEQNUM1_Pos)
#define ADC_SEQNUM_SEQNUM1_2        (0x4UL << ADC_SEQNUM_SEQNUM1_Pos)
#define ADC_SEQNUM_SEQNUM1          ADC_SEQNUM_SEQNUM1_Msk
#define ADC_SEQNUM_SEQNUM2_Pos      (4U)                                        /*!< SEQ_NUM2 (Bit 4)                   */
#define ADC_SEQNUM_SEQNUM2_Msk      (0x7UL << ADC_SEQNUM_SEQNUM2_Pos)           /*!< SEQ_NUM2 (Bitfield-Mask: 0x07)     */
#define ADC_SEQNUM_SEQNUM2_0        (0x1UL << ADC_SEQNUM_SEQNUM2_Pos)
#define ADC_SEQNUM_SEQNUM2_1        (0x2UL << ADC_SEQNUM_SEQNUM2_Pos)
#define ADC_SEQNUM_SEQNUM2_2        (0x4UL << ADC_SEQNUM_SEQNUM2_Pos)
#define ADC_SEQNUM_SEQNUM2          ADC_SEQNUM_SEQNUM2_Msk
#define ADC_SEQNUM_SEQNUM3_Pos      (8U)                                        /*!< SEQ_NUM3 (Bit 8)                   */
#define ADC_SEQNUM_SEQNUM3_Msk      (0x7UL << ADC_SEQNUM_SEQNUM3_Pos)           /*!< SEQ_NUM3 (Bitfield-Mask: 0x07)     */
#define ADC_SEQNUM_SEQNUM3_0        (0x1UL << ADC_SEQNUM_SEQNUM3_Pos)
#define ADC_SEQNUM_SEQNUM3_1        (0x2UL << ADC_SEQNUM_SEQNUM3_Pos)
#define ADC_SEQNUM_SEQNUM3_2        (0x4UL << ADC_SEQNUM_SEQNUM3_Pos)
#define ADC_SEQNUM_SEQNUM3          ADC_SEQNUM_SEQNUM3_Msk
#define ADC_SEQNUM_SGNUM_Pos        (16U)                                       /*!< SG_NUM (Bit 16)                    */
#define ADC_SEQNUM_SGNUM_Msk        (0x3UL << ADC_SEQNUM_SGNUM_Pos)             /*!< SG_NUM (Bitfield-Mask: 0x03)       */
#define ADC_SEQNUM_SGNUM_0          (0x1UL << ADC_SEQNUM_SGNUM_Pos)
#define ADC_SEQNUM_SGNUM_1          (0x2UL << ADC_SEQNUM_SGNUM_Pos)
#define ADC_SEQNUM_SGNUM            ADC_SEQNUM_SGNUM_Msk


/* =========================================================================================================================== */
/* ================                                           COMP                                            ================ */
/* =========================================================================================================================== */
/* =========================================================  CR  =========================================================== */
#define COMP_CR_COMPEN_Pos          (0UL)                                       /*!< EN (Bit 0)                           */
#define COMP_CR_COMPEN_Msk          (0x1UL << COMP_CR_COMPEN_Pos)               /*!< EN (Bitfield-Mask: 0x01)             */
#define COMP_CR_COMPEN              COMP_CR_COMPEN_Msk                          /*!< Comparator enable bit                */
#define COMP_CR_VNSEL_Pos           (4UL)                                       /*!< VN_EN (Bit 4)                        */
#define COMP_CR_VNSEL_0             (0x1UL << COMP_CR_VNSEL_Pos)
#define COMP_CR_VNSEL_1             (0x2UL << COMP_CR_VNSEL_Pos)
#define COMP_CR_VNSEL_Msk           (0x3UL << COMP_CR_VNSEL_Pos)                /*!< VN_EN (Bitfield-Mask: 0x03)            */
#define COMP_CR_VNSEL               COMP_CR_VNSEL_Msk                           /*!< Comparator VN source bit bit           */
#define COMP_CR_VPSEL_Pos           (7UL)                                       /*!< VP_EN (Bit 7)                          */
#define COMP_CR_VPSEL_0             (0x1UL << COMP_CR_VPSEL_Pos)
#define COMP_CR_VPSEL_1             (0x2UL << COMP_CR_VPSEL_Pos)
#define COMP_CR_VPSEL_Msk           (0x3UL << COMP_CR_VPSEL_Pos)                /*!< VP_EN (Bitfield-Mask: 0x03)            */
#define COMP_CR_VPSEL               COMP_CR_VPSEL_Msk                           /*!< Comparator VP source bit               */
#define COMP_CR_POL_Pos             (13UL)                                      /*!< POL (Bit 13)                           */
#define COMP_CR_POL_Msk             (0x1UL << COMP_CR_POL_Pos)                  /*!< POL (Bitfield-Mask: 0x01)              */
#define COMP_CR_POL                 COMP_CR_POL_Msk
#define COMP_CR_HYSNEN_Pos          (14UL)                                      /*!< HYSN_EN (Bit 14)                       */
#define COMP_CR_HYSNEN_0            (0x1UL << COMP_CR_HYSNEN_Pos)
#define COMP_CR_HYSNEN_1            (0x2UL << COMP_CR_HYSNEN_Pos)
#define COMP_CR_HYSNEN_Msk          (0x3UL << COMP_CR_HYSNEN_Pos)               /*!< HYS_EN (Bitfield-Mask: 0x03)            */
#define COMP_CR_HYSNEN              COMP_CR_HYSNEN_Msk
#define COMP_CR_HYSPEN_Pos          (16UL)                                      /*!< HYSN_EN (Bit 16)                       */
#define COMP_CR_HYSPEN_0            (0x1UL << COMP_CR_HYSPEN_Pos)
#define COMP_CR_HYSPEN_1            (0x2UL << COMP_CR_HYSPEN_Pos)
#define COMP_CR_HYSPEN_Msk          (0x3UL << COMP_CR_HYSPEN_Pos)               /*!< HYS_EN (Bitfield-Mask: 0x03)           */
#define COMP_CR_HYSPEN              COMP_CR_HYSPEN_Msk
#define COMP_CR_REN_Pos             (18U)                                       /*!< R_EN (Bit 18)                           */
#define COMP_CR_REN_Msk             (0x1UL << COMP_CR_REN_Pos)                  /*!< R_EN (Bitfield-Mask: 0x01)              */
#define COMP_CR_REN                 COMP_CR_REN_Msk
#define COMP_CR_FEN_Pos             (19U)                                       /*!< F_EN (Bit 19)                           */
#define COMP_CR_FEN_Msk             (0x1UL << COMP_CR_FEN_Pos)                  /*!< F_EN (Bitfield-Mask: 0x01)              */
#define COMP_CR_FEN                 COMP_CR_FEN_Msk
#define COMP_CR_RFEN_Pos            (20U)                                       /*!< RF_EN (Bit 20)                           */
#define COMP_CR_RFEN_Msk            (0x1UL << COMP_CR_RFEN_Pos)                 /*!< RF_EN (Bitfield-Mask: 0x01)              */
#define COMP_CR_RFEN                COMP_CR_RFEN_Msk
#define COMP_CR_SWEG_Pos            (21U)                                       /*!< SWINT (Bit 21)                           */
#define COMP_CR_SWEG_Msk            (0x1UL << COMP_CR_SWEG_Pos)                 /*!< SWINT (Bitfield-Mask: 0x01)              */
#define COMP_CR_SWEG                COMP_CR_SWEG_Msk
#define COMP_CR_CFG_Pos             (22U)                                       /*!< CFG (Bit 22)                           */
#define COMP_CR_CFG_0               (0x1UL << COMP_CR_CFG_Pos)                  /*!< CFG (Bitfield-Mask: 0x01)              */
#define COMP_CR_CFG_1               (0x2UL << COMP_CR_CFG_Pos)                  /*!< CFG (Bitfield-Mask: 0x02)              */
#define COMP_CR_CFG_2               (0x4UL << COMP_CR_CFG_Pos)                  /*!< CFG (Bitfield-Mask: 0x04)              */
#define COMP_CR_CFG_3               (0x8UL << COMP_CR_CFG_Pos)                  /*!< CFG (Bitfield-Mask: 0x08)              */
#define COMP_CR_CFG_Msk             (0xFUL << COMP_CR_CFG_Pos)                  /*!< CFG (Bitfield-Mask: 0x0F)              */
#define COMP_CR_CFG                 COMP_CR_CFG_Msk
#define COMP_CR_PSC_Pos             (26U)                                       /*!< PSC (Bit 26)                           */
#define COMP_CR_PSC_0               (0x1UL << COMP_CR_PSC_Pos)                  /*!< PSC (Bitfield-Mask: 0x01)              */
#define COMP_CR_PSC_1               (0x2UL << COMP_CR_PSC_Pos)                  /*!< PSC (Bitfield-Mask: 0x02)              */
#define COMP_CR_PSC_2               (0x4UL << COMP_CR_PSC_Pos)                  /*!< PSC (Bitfield-Mask: 0x04)              */
#define COMP_CR_PSC_Msk             (0xFUL << COMP_CR_PSC_Pos)                  /*!< PSC (Bitfield-Mask: 0x07)              */
#define COMP_CR_PSC                 COMP_CR_PSC_Msk
#define COMP_CR_VAL_Pos             (30U)                                       /*!< VALUE (Bit 30)                        */
#define COMP_CR_VAL_Msk             (0x1UL << COMP_CR_VAL_Pos)                  /*!< VALUE (Bitfield-Mask: 0x01)           */
#define COMP_CR_VAL                 COMP_CR_VAL_Msk
#define COMP_CR_LOCK_Pos            (31U)                                       /*!< LOCK (Bit 31)                         */
#define COMP_CR_LOCK_Msk            (0x1UL << COMP_CR_LOCK_Pos)                 /*!< LOCK (Bitfield-Mask: 0x01)            */
#define COMP_CR_LOCK                COMP_CR_LOCK_Msk
/* =========================================================  PEND  ========================================================= */
#define COMP_ISR_IFLG_Pos           (0U)                                        /*!< PEND (Bit 0)                           */
#define COMP_ISR_IFLG_Msk           (0x1UL << COMP_ISR_IFLG_Pos)                /*!< PEND (Bitfield-Mask: 0x01)               */
#define COMP_ISR_IFLG               COMP_ISR_IFLG_Msk


/* ================================================================================================================================== */
/* =============================================================  DMA  ============================================================== */
/* ================================================================================================================================== */
/* ===========================================  Bit definition for DMA_ISR register  ============================================ */
#define DMA_ISR_FEIFLG0_Pos         (0U)                                        /*!< Stream 0 FIFO error interrupt flag */
#define DMA_ISR_FEIFLG0_Msk         (0x1UL << DMA_ISR_FEIFLG0_Pos)              /*!< 0x00000001 */
#define DMA_ISR_FEIFLG0             DMA_ISR_FEIFLG0_Msk
#define DMA_ISR_DMEIFLG0_Pos        (2U)                                        /*!< Stream 0 direct mode error interrupt flag */
#define DMA_ISR_DMEIFLG0_Msk        (0x1UL << DMA_ISR_DMEIFLG0_Pos)             /*!< 0x00000004 */
#define DMA_ISR_DMEIFLG0            DMA_ISR_DMEIFLG0_Msk
#define DMA_ISR_TXEIFLG0_Pos        (3U)                                        /*!< Stream 0 transfer error interrupt flag */
#define DMA_ISR_TXEIFLG0_Msk        (0x1UL << DMA_ISR_TXEIFLG0_Pos)             /*!< 0x00000008 */
#define DMA_ISR_TXEIFLG0            DMA_ISR_TXEIFLG0_Msk
#define DMA_ISR_HTXIFLG0_Pos        (4U)                                        /*!< Stream 0 half transfer interrupt flag */
#define DMA_ISR_HTXIFLG0_Msk        (0x1UL << DMA_ISR_HTXIFLG0_Pos)             /*!< 0x00000010 */
#define DMA_ISR_HTXIFLG0            DMA_ISR_HTXIFLG0_Msk
#define DMA_ISR_TXCIFLG0_Pos        (5U)                                        /*!< Stream 0 transfer complete interrupt flag */
#define DMA_ISR_TXCIFLG0_Msk        (0x1UL << DMA_ISR_TXCIFLG0_Pos)             /*!< 0x00000020 */
#define DMA_ISR_TXCIFLG0            DMA_ISR_TXCIFLG0_Msk
#define DMA_ISR_FEIFLG1_Pos         (6U)                                        /*!< Stream 1 FIFO error interrupt flag */
#define DMA_ISR_FEIFLG1_Msk         (0x1UL << DMA_ISR_FEIFLG1_Pos)              /*!< 0x00000040 */
#define DMA_ISR_FEIFLG1             DMA_ISR_FEIFLG1_Msk
#define DMA_ISR_DMEIFLG1_Pos        (8U)                                        /*!< Stream 1 direct mode error interrupt flag */
#define DMA_ISR_DMEIFLG1_Msk        (0x1UL << DMA_ISR_DMEIFLG1_Pos)             /*!< 0x00000100 */
#define DMA_ISR_DMEIFLG1            DMA_ISR_DMEIFLG1_Msk
#define DMA_ISR_TXEIFLG1_Pos        (9U)                                        /*!< Stream 1 transfer error interrupt flag */
#define DMA_ISR_TXEIFLG1_Msk        (0x1UL << DMA_ISR_TXEIFLG1_Pos)             /*!< 0x00000200 */
#define DMA_ISR_TXEIFLG1            DMA_ISR_TXEIFLG1_Msk
#define DMA_ISR_HTXIFLG1_Pos        (10U)                                       /*!< Stream 1 half transfer interrupt flag */
#define DMA_ISR_HTXIFLG1_Msk        (0x1UL << DMA_ISR_HTXIFLG1_Pos)             /*!< 0x00000400 */
#define DMA_ISR_HTXIFLG1            DMA_ISR_HTXIFLG1_Msk
#define DMA_ISR_TXCIFLG1_Pos        (11U)                                       /*!< Stream 1 transfer complete interrupt flag */
#define DMA_ISR_TXCIFLG1_Msk        (0x1UL << DMA_ISR_TXCIFLG1_Pos)             /*!< 0x00000800 */
#define DMA_ISR_TXCIFLG1            DMA_ISR_TXCIFLG1_Msk

/* ============================================  Bit definition for DMA_IFCLR register  ============================================ */
#define DMA_IFCLR_CFEIFLG0_Pos      (0U)                                        /*!< Stream 0 clear FIFO error interrupt flag */
#define DMA_IFCLR_CFEIFLG0_Msk      (0x1UL << DMA_IFCLR_CFEIFLG0_Pos)           /*!< 0x00000001 */
#define DMA_IFCLR_CFEIFLG0          DMA_IFCLR_CFEIFLG0_Msk
#define DMA_IFCLR_CDMEIFLG0_Pos     (2U)                                        /*!< Stream 0 clear direct mode error interrupt flag */
#define DMA_IFCLR_CDMEIFLG0_Msk     (0x1UL << DMA_IFCLR_CDMEIFLG0_Pos)          /*!< 0x00000004 */
#define DMA_IFCLR_CDMEIFLG0         DMA_IFCLR_CDMEIFLG0_Msk
#define DMA_IFCLR_CTXEIFLG0_Pos     (3U)                                        /*!< Stream 0 clear transfer error interrupt flag */
#define DMA_IFCLR_CTXEIFLG0_Msk     (0x1UL << DMA_IFCLR_CTXEIFLG0_Pos)          /*!< 0x00000008 */
#define DMA_IFCLR_CTXEIFLG0         DMA_IFCLR_CTXEIFLG0_Msk
#define DMA_IFCLR_CHTXIFLG0_Pos     (4U)                                        /*!< Stream 0 clear half transfer interrupt flag */
#define DMA_IFCLR_CHTXIFLG0_Msk     (0x1UL << DMA_IFCLR_CHTXIFLG0_Pos)          /*!< 0x00000010 */
#define DMA_IFCLR_CHTXIFLG0         DMA_IFCLR_CHTXIFLG0_Msk
#define DMA_IFCLR_CTXCIFLG0_Pos     (5U)                                        /*!< Stream 0 clear transfer complete interrupt flag */
#define DMA_IFCLR_CTXCIFLG0_Msk     (0x1UL << DMA_IFCLR_CTXCIFLG0_Pos)          /*!< 0x00000020 */
#define DMA_IFCLR_CTXCIFLG0         DMA_IFCLR_CTXCIFLG0_Msk
#define DMA_IFCLR_CFEIFLG1_Pos      (6U)                                        /*!< Stream 1 clear FIFO error interrupt flag */
#define DMA_IFCLR_CFEIFLG1_Msk      (0x1UL << DMA_IFCLR_CFEIFLG1_Pos)           /*!< 0x00000040 */
#define DMA_IFCLR_CFEIFLG1          DMA_IFCLR_CFEIFLG1_Msk
#define DMA_IFCLR_CDMEIFLG1_Pos     (8U)                                        /*!< Stream 1 clear direct mode error interrupt flag */
#define DMA_IFCLR_CDMEIFLG1_Msk     (0x1UL << DMA_IFCLR_CDMEIFLG1_Pos)          /*!< 0x00000100 */
#define DMA_IFCLR_CDMEIFLG1         DMA_IFCLR_CDMEIFLG1_Msk
#define DMA_IFCLR_CTXEIFLG1_Pos     (9U)                                        /*!< Stream 1 clear transfer error interrupt flag */
#define DMA_IFCLR_CTXEIFLG1_Msk     (0x1UL << DMA_IFCLR_CTXEIFLG1_Pos)          /*!< 0x00000200 */
#define DMA_IFCLR_CTXEIFLG1         DMA_IFCLR_CTXEIFLG1_Msk
#define DMA_IFCLR_CHTXIFLG1_Pos     (10U)                                       /*!< Stream 1 clear half transfer interrupt flag */
#define DMA_IFCLR_CHTXIFLG1_Msk     (0x1UL << DMA_IFCLR_CHTXIFLG1_Pos)          /*!< 0x00000400 */
#define DMA_IFCLR_CHTXIFLG1         DMA_IFCLR_CHTXIFLG1_Msk
#define DMA_IFCLR_CTXCIFLG1_Pos     (11U)                                       /*!< Stream 1 clear transfer complete interrupt flag */
#define DMA_IFCLR_CTXCIFLG1_Msk     (0x1UL << DMA_IFCLR_CTXCIFLG1_Pos)          /*!< 0x00000800 */
#define DMA_IFCLR_CTXCIFLG1         DMA_IFCLR_CTXCIFLG1_Msk

/* ============================================  Bit definition for DMA_SCFGx register  ============================================= */
#define DMA_SCFGx_CHEN_Pos          (0U)                                        /*!< Stream enable / flag stream ready when read low */
#define DMA_SCFGx_CHEN_Msk          (0x1UL << DMA_SCFGx_CHEN_Pos)               /*!< 0x00000001 */
#define DMA_SCFGx_CHEN              DMA_SCFGx_CHEN_Msk
#define DMA_SCFGx_DMEIEN_Pos        (1U)                                        /*!< Direct mode error interrupt enable */
#define DMA_SCFGx_DMEIEN_Msk        (0x1UL << DMA_SCFGx_DMEIEN_Pos)             /*!< 0x00000002 */
#define DMA_SCFGx_DMEIEN            DMA_SCFGx_DMEIEN_Msk
#define DMA_SCFGx_TXEIEN_Pos        (2U)                                        /*!< Transfer error interrupt enable */
#define DMA_SCFGx_TXEIEN_Msk        (0x1UL << DMA_SCFGx_TXEIEN_Pos)             /*!< 0x00000004 */
#define DMA_SCFGx_TXEIEN            DMA_SCFGx_TXEIEN_Msk
#define DMA_SCFGx_HTXIEN_Pos        (3U)                                        /*!< Half transfer interrupt enable */
#define DMA_SCFGx_HTXIEN_Msk        (0x1UL << DMA_SCFGx_HTXIEN_Pos)             /*!< 0x00000008 */
#define DMA_SCFGx_HTXIEN            DMA_SCFGx_HTXIEN_Msk
#define DMA_SCFGx_TXCIEN_Pos        (4U)                                        /*!< Transfer complete interrupt enable */
#define DMA_SCFGx_TXCIEN_Msk        (0x1UL << DMA_SCFGx_TXCIEN_Pos)             /*!< 0x00000010 */
#define DMA_SCFGx_TXCIEN            DMA_SCFGx_TXCIEN_Msk
#define DMA_SCFGx_PFCFG_Pos         (5U)                                        /*!< Peripheral flow controller */
#define DMA_SCFGx_PFCFG_Msk         (0x1UL << DMA_SCFGx_PFCFG_Pos)              /*!< 0x00000020 */
#define DMA_SCFGx_PFCFG             DMA_SCFGx_PFCFG_Msk
#define DMA_SCFGx_DIRCFG_Pos        (6U)                                        /*!< Data transfer direction */
#define DMA_SCFGx_DIRCFG_0          (0x1UL << DMA_SCFGx_DIRCFG_Pos)             /*!< 0x00000040 */
#define DMA_SCFGx_DIRCFG_1          (0x2UL << DMA_SCFGx_DIRCFG_Pos)             /*!< 0x00000080 */
#define DMA_SCFGx_DIRCFG_Msk        (0x3UL << DMA_SCFGx_DIRCFG_Pos)             /*!< 0x000000C0 */
#define DMA_SCFGx_DIRCFG            DMA_SCFGx_DIRCFG_Msk
#define DMA_SCFGx_CIRCMEN_Pos       (8U)                                        /*!< Circular mode */
#define DMA_SCFGx_CIRCMEN_Msk       (0x1UL << DMA_SCFGx_CIRCMEN_Pos)            /*!< 0x00000100 */
#define DMA_SCFGx_CIRCMEN           DMA_SCFGx_CIRCMEN_Msk
#define DMA_SCFGx_PINCM_Pos         (9U)                                        /*!< Peripheral increment mode */
#define DMA_SCFGx_PINCM_Msk         (0x1UL << DMA_SCFGx_PINCM_Pos)              /*!< 0x00000200 */
#define DMA_SCFGx_PINCM             DMA_SCFGx_PINCM_Msk
#define DMA_SCFGx_MINCM_Pos         (10U)                                       /*!< Memory increment mode */
#define DMA_SCFGx_MINCM_Msk         (0x1UL << DMA_SCFGx_MINCM_Pos)              /*!< 0x00000400 */
#define DMA_SCFGx_MINCM             DMA_SCFGx_MINCM_Msk
#define DMA_SCFGx_PSIZECFG_Pos      (11U)                                       /*!< Peripheral data size */
#define DMA_SCFGx_PSIZECFG_0        (0x1UL << DMA_SCFGx_PSIZECFG_Pos)           /*!< 0x00000800 */
#define DMA_SCFGx_PSIZECFG_1        (0x2UL << DMA_SCFGx_PSIZECFG_Pos)           /*!< 0x00001000 */
#define DMA_SCFGx_PSIZECFG_Msk      (0x3UL << DMA_SCFGx_PSIZECFG_Pos)           /*!< 0x00001800 */
#define DMA_SCFGx_PSIZECFG          DMA_SCFGx_PSIZECFG_Msk
#define DMA_SCFGx_MSIZECFG_Pos      (13U)                                       /*!< Memory data size */
#define DMA_SCFGx_MSIZECFG_0        (0x1UL << DMA_SCFGx_MSIZECFG_Pos)           /*!< 0x00002000 */
#define DMA_SCFGx_MSIZECFG_1        (0x2UL << DMA_SCFGx_MSIZECFG_Pos)           /*!< 0x00004000 */
#define DMA_SCFGx_MSIZECFG_Msk      (0x3UL << DMA_SCFGx_MSIZECFG_Pos)           /*!< 0x00006000 */
#define DMA_SCFGx_MSIZECFG          DMA_SCFGx_MSIZECFG_Msk
#define DMA_SCFGx_PERISIZE_Pos      (15U)                                       /*!< Peripheral increment offset size */
#define DMA_SCFGx_PERISIZE_Msk      (0x1UL << DMA_SCFGx_PERISIZE_Pos)           /*!< 0x00008000 */
#define DMA_SCFGx_PERISIZE          DMA_SCFGx_PERISIZE_Msk
#define DMA_SCFGx_PRILCFG_Pos       (16U)                                       /*!< Priority level */
#define DMA_SCFGx_PRILCFG_Msk       (0x1UL << DMA_SCFGx_PRILCFG_Pos)            /*!< 0x00010000 */
#define DMA_SCFGx_PRILCFG           DMA_SCFGx_PRILCFG_Msk
#define DMA_SCFGx_DBM_Pos           (18U)                                       /*!< Double buffer mode */
#define DMA_SCFGx_DBM_Msk           (0x1UL << DMA_SCFGx_DBM_Pos)                /*!< 0x00040000 */
#define DMA_SCFGx_DBM               DMA_SCFGx_DBM_Msk
#define DMA_SCFGx_CTARG_Pos         (19U)                                       /*!< Current target (only in double buffer mode) */
#define DMA_SCFGx_CTARG_Msk         (0x1UL << DMA_SCFGx_CTARG_Pos)              /*!< 0x00080000 */
#define DMA_SCFGx_CTARG             DMA_SCFGx_CTARG_Msk
#define DMA_SCFGx_PBCFG_Pos         (21U)                                       /*!< Peripheral burst transfer configuration */
#define DMA_SCFGx_PBCFG_0           (0x1UL << DMA_SCFGx_PBCFG_Pos)              /*!< 0x00200000 */
#define DMA_SCFGx_PBCFG_1           (0x2UL << DMA_SCFGx_PBCFG_Pos)              /*!< 0x00400000 */
#define DMA_SCFGx_PBCFG_Msk         (0x3UL << DMA_SCFGx_PBCFG_Pos)              /*!< 0x00600000 */
#define DMA_SCFGx_PBCFG             DMA_SCFGx_PBCFG_Msk
#define DMA_SCFGx_MBCFG_Pos         (23U)                                       /*!< Memory burst transfer configuration */
#define DMA_SCFGx_MBCFG_0           (0x1UL << DMA_SCFGx_MBCFG_Pos)              /*!< 0x00800000 */
#define DMA_SCFGx_MBCFG_1           (0x2UL << DMA_SCFGx_MBCFG_Pos)              /*!< 0x01000000 */
#define DMA_SCFGx_MBCFG_Msk         (0x3UL << DMA_SCFGx_MBCFG_Pos)              /*!< 0x01800000 */
#define DMA_SCFGx_MBCFG             DMA_SCFGx_MBCFG_Msk
#define DMA_SCFGx_PSEL_Pos          (25U)                                       /*!< Channel selection */
#define DMA_SCFGx_PSEL_0            (0x1UL << DMA_SCFGx_PSEL_Pos)               /*!< 0x02000000 */
#define DMA_SCFGx_PSEL_1            (0x2UL << DMA_SCFGx_PSEL_Pos)               /*!< 0x04000000 */
#define DMA_SCFGx_PSEL_2            (0x4UL << DMA_SCFGx_PSEL_Pos)               /*!< 0x08000000 */
#define DMA_SCFGx_PSEL_3            (0x8UL << DMA_SCFGx_PSEL_Pos)               /*!< 0x10000000 */
#define DMA_SCFGx_PSEL_Msk          (0xFUL << DMA_SCFGx_PSEL_Pos)               /*!< 0x1E000000 */
#define DMA_SCFGx_PSEL              DMA_SCFGx_PSEL_Msk

/* ============================================  Bit definition for DMA_NDATAx register  ============================================ */
#define DMA_NDATAx_NDATA_Pos        (0U)                                        /*!< Number of data items to transfer */
#define DMA_NDATAx_NDATA_Msk        (0xFFFFUL << DMA_NDATAx_NDATA_Pos)          /*!< 0x0000FFFF */
#define DMA_NDATAx_NDATA            DMA_NDATAx_NDATA_Msk

/* ============================================  Bit definition for DMA_PADDRx register  ============================================ */
#define DMA_PADDRx_PADDR_Pos        (0U)                                        /*!< Peripheral address */
#define DMA_PADDRx_PADDR_Msk        (0xFFFFFFFFUL << DMA_PADDRx_PADDR_Pos)      /*!< 0xFFFFFFFF */
#define DMA_PADDRx_PADDR            DMA_PADDRx_PADDR_Msk

/* ===========================================  Bit definition for DMA_M0ADDRx register  ============================================ */
#define DMA_M0ADDRx_M0ADDR_Pos      (0U)                                        /*!< Memory 0 address */
#define DMA_M0ADDRx_M0ADDR_Msk      (0xFFFFFFFFUL << DMA_M0ADDRx_M0ADDR_Pos)    /*!< 0xFFFFFFFF */
#define DMA_M0ADDRx_M0ADDR          DMA_M0ADDRx_M0ADDR_Msk

/* ===========================================  Bit definition for DMA_M1ADDRx register  ============================================ */
#define DMA_M1ADDRx_M1ADDR_Pos      (0U)                                        /*!< Memory 1 address (used in case of Double buffer mode) */
#define DMA_M1ADDRx_M1ADDR_Msk      (0xFFFFFFFFUL << DMA_M1ADDRx_M1ADDR_Pos)    /*!< 0xFFFFFFFF */
#define DMA_M1ADDRx_M1ADDR          DMA_M1ADDRx_M1ADDR_Msk

/* ============================================  Bit definition for DMA_FIFOCRx register  ============================================ */
#define DMA_FIFOCRx_FTHSEL_Pos      (0U)                                        /*!< FIFO threshold selection */
#define DMA_FIFOCRx_FTHSEL_0        (0x1UL << DMA_FIFOCRx_FTHSEL_Pos)           /*!< 0x00000001 */
#define DMA_FIFOCRx_FTHSEL_1        (0x2UL << DMA_FIFOCRx_FTHSEL_Pos)           /*!< 0x00000002 */
#define DMA_FIFOCRx_FTHSEL_Msk      (0x3UL << DMA_FIFOCRx_FTHSEL_Pos)           /*!< 0x00000003 */
#define DMA_FIFOCRx_FTHSEL          DMA_FIFOCRx_FTHSEL_Msk
#define DMA_FIFOCRx_DMDEN_Pos       (2U)                                        /*!< Direct mode disable */
#define DMA_FIFOCRx_DMDEN_Msk       (0x1UL << DMA_FIFOCRx_DMDEN_Pos)            /*!< 0x00000004 */
#define DMA_FIFOCRx_DMDEN           DMA_FIFOCRx_DMDEN_Msk
#define DMA_FIFOCRx_FSTS_Pos        (3U)                                        /*!< FIFO status */
#define DMA_FIFOCRx_FSTS_0          (0x1UL << DMA_FIFOCRx_FSTS_Pos)             /*!< 0x00000008 */
#define DMA_FIFOCRx_FSTS_1          (0x2UL << DMA_FIFOCRx_FSTS_Pos)             /*!< 0x00000010 */
#define DMA_FIFOCRx_FSTS_2          (0x4UL << DMA_FIFOCRx_FSTS_Pos)             /*!< 0x00000020 */
#define DMA_FIFOCRx_FSTS_Msk        (0x7UL << DMA_FIFOCRx_FSTS_Pos)             /*!< 0x00000038 */
#define DMA_FIFOCRx_FSTS            DMA_FIFOCRx_FSTS_Msk
#define DMA_FIFOCRx_FEIEN_Pos       (7U)                                        /*!< FIFO error interrupt enable */
#define DMA_FIFOCRx_FEIEN_Msk       (0x1UL << DMA_FIFOCRx_FEIEN_Pos)            /*!< 0x00000080 */
#define DMA_FIFOCRx_FEIEN           DMA_FIFOCRx_FEIEN_Msk


/* ================================================================================================================================== */
/* =============================================================  RCC  ============================================================== */
/* ================================================================================================================================== */
/* =============================================  Bit definition for RCC_KEY register  ============================================== */
#define RCC_KEY_LOCKKEY_Pos         (0U)                                        /*!< Password protection configuration for write operations */
#define RCC_KEY_LOCKKEY_Msk         (0xFFFFUL << RCC_KEY_LOCKKEY_Pos)           /*!< 0x0000FFFF */
#define RCC_KEY_LOCKKEY             RCC_KEY_LOCKKEY_Msk
#define RCC_KEY_KEYST_Pos           (16U)                                       /*!< Register Write Protection Flag */
#define RCC_KEY_KEYST_Msk           (0x1UL << RCC_KEY_KEYST_Pos)                /*!< 0x00010000 */
#define RCC_KEY_KEYST               RCC_KEY_KEYST_Msk

#define RCC_KEY_VALUE               (0x87E4U)

/* =============================================  Bit definition for RCC_CR register  ============================================= */
#define RCC_CR_HSIEN_Pos            (0U)                                        /*!< HSI clock enable */
#define RCC_CR_HSIEN_Msk            (0x1UL << RCC_CR_HSIEN_Pos)                 /*!< 0x00000001 */
#define RCC_CR_HSIEN                RCC_CR_HSIEN_Msk
#define RCC_CR_HSIRDY_Pos           (1U)                                        /*!< HSI clock stabilization flag */
#define RCC_CR_HSIRDY_Msk           (0x1UL << RCC_CR_HSIRDY_Pos)                /*!< 0x00000002 */
#define RCC_CR_HSIRDY               RCC_CR_HSIRDY_Msk

/* =============================================  Bit definition for RCC_CFG register  ============================================= */
#define RCC_CFG_SWSEL_Pos           (0U)                                        /*!< System clock select */
#define RCC_CFG_SWSEL_Msk           (0x1UL << RCC_CFG_SWSEL_Pos)                /*!< 0x00000001 */
#define RCC_CFG_SWSEL               RCC_CFG_SWSEL_Msk
#define RCC_CFG_SW_HSI              0x00000000U
#define RCC_CFG_SW_LSI              0x00000001U
#define RCC_CFG_SWSTS_Pos           (1U)                                        /*!< System clock select flag */
#define RCC_CFG_SWSTS_Msk           (0x1UL << RCC_CFG_SWSTS_Pos)                /*!< 0x00000002 */
#define RCC_CFG_SWSTS               RCC_CFG_SWSTS_Msk
#define RCC_CFG_SWSTS_HSI           0x00000000U
#define RCC_CFG_SWSTS_LSI           0x00000001U
#define RCC_CFG_AHBCLKDIV_Pos       (4U)                                        /*!< AHB clock division */
#define RCC_CFG_AHBCLKDIV_0         (0x1UL << RCC_CFG_AHBCLKDIV_Pos)            /*!< 0x00000010 */
#define RCC_CFG_AHBCLKDIV_1         (0x2UL << RCC_CFG_AHBCLKDIV_Pos)            /*!< 0x00000020 */
#define RCC_CFG_AHBCLKDIV_2         (0x4UL << RCC_CFG_AHBCLKDIV_Pos)            /*!< 0x00000040 */
#define RCC_CFG_AHBCLKDIV_Msk       (0x7UL << RCC_CFG_AHBCLKDIV_Pos)            /*!< 0x00000070 */
#define RCC_CFG_AHBCLKDIV           RCC_CFG_AHBCLKDIV_Msk
#define RCC_CFG_AHBCLKDIV_DIV1      0x00000000U
#define RCC_CFG_AHBCLKDIV_DIV2      0x00000010U
#define RCC_CFG_AHBCLKDIV_DIV4      0x00000020U
#define RCC_CFG_AHBCLKDIV_DIV8      0x00000030U
#define RCC_CFG_AHBCLKDIV_DIV16     0x00000040U
#define RCC_CFG_AHBCLKDIV_DIV32     0x00000050U
#define RCC_CFG_AHBCLKDIV_DIV64     0x00000060U
#define RCC_CFG_AHBCLKDIV_DIV128    0x00000070U
#define RCC_CFG_APBCLKDIV_Pos       (8U)                                        /*!< APB clock division */
#define RCC_CFG_APBCLKDIV_0         (0x1UL << RCC_CFG_APBCLKDIV_Pos)            /*!< 0x00000100 */
#define RCC_CFG_APBCLKDIV_1         (0x2UL << RCC_CFG_APBCLKDIV_Pos)            /*!< 0x00000200 */
#define RCC_CFG_APBCLKDIV_Msk       (0x3UL << RCC_CFG_APBCLKDIV_Pos)            /*!< 0x00000300 */
#define RCC_CFG_APBCLKDIV           RCC_CFG_APBCLKDIV_Msk
#define RCC_CFG_APBCLKDIV_DIV1      0x00000000U
#define RCC_CFG_APBCLKDIV_DIV2      0x00000100U
#define RCC_CFG_APBCLKDIV_DIV4      0x00000200U
#define RCC_CFG_APBCLKDIV_DIV8      0x00000300U
#define RCC_CFG_HSIDIV_Pos          (12U)                                       /*!< System clock HSI branch division */
#define RCC_CFG_HSIDIV_0            (0x1UL << RCC_CFG_HSIDIV_Pos)               /*!< 0x00001000 */
#define RCC_CFG_HSIDIV_1            (0x2UL << RCC_CFG_HSIDIV_Pos)               /*!< 0x00002000 */
#define RCC_CFG_HSIDIV_Msk          (0x3UL << RCC_CFG_HSIDIV_Pos)               /*!< 0x00003000 */
#define RCC_CFG_HSIDIV              RCC_CFG_HSIDIV_Msk
#define RCC_CFG_HSIDIV_DIV1         0x00000000U
#define RCC_CFG_HSIDIV_DIV2         0x00001000U
#define RCC_CFG_HSIDIV_DIV4         0x00002000U
#define RCC_CFG_HSIDIV_DIV8         0x00003000U
#define RCC_CFG_CLKOUTSEL_Pos       (16U)                                       /*!< Clock output select */
#define RCC_CFG_CLKOUTSEL_0         (0x1UL << RCC_CFG_CLKOUTSEL_Pos)            /*!< 0x00010000 */
#define RCC_CFG_CLKOUTSEL_1         (0x2UL << RCC_CFG_CLKOUTSEL_Pos)            /*!< 0x00020000 */
#define RCC_CFG_CLKOUTSEL_Msk       (0x3UL << RCC_CFG_CLKOUTSEL_Pos)            /*!< 0x00030000 */
#define RCC_CFG_CLKOUTSEL           RCC_CFG_CLKOUTSEL_Msk
#define RCC_CFG_CLKOUTSEL_SYSCLK    0x00000000U
#define RCC_CFG_CLKOUTSEL_HSI       RCC_CFG_CLKOUTSEL_0
#define RCC_CFG_CLKOUTSEL_LSI       RCC_CFG_CLKOUTSEL_1
#define RCC_CFG_CLKOUTSEL_HCLK      (RCC_CFG_CLKOUTSEL_0 | RCC_CFG_CLKOUTSEL_1)
#define RCC_CFG_CLKOUTEN_Pos        (19U)                                       /*!< Clock output enable */
#define RCC_CFG_CLKOUTEN_Msk        (0x1UL << RCC_CFG_CLKOUTEN_Pos)             /*!< 0x00080000 */
#define RCC_CFG_CLKOUTEN            RCC_CFG_CLKOUTEN_Msk
#define RCC_CFG_CLKOUTDIV_Pos       (20U)                                       /*!< Clock output division */
#define RCC_CFG_CLKOUTDIV_0         (0x1UL << RCC_CFG_CLKOUTDIV_Pos)            /*!< 0x00100000 */
#define RCC_CFG_CLKOUTDIV_1         (0x2UL << RCC_CFG_CLKOUTDIV_Pos)            /*!< 0x00200000 */
#define RCC_CFG_CLKOUTDIV_2         (0x4UL << RCC_CFG_CLKOUTDIV_Pos)            /*!< 0x00400000 */
#define RCC_CFG_CLKOUTDIV_Msk       (0x7UL << RCC_CFG_CLKOUTDIV_Pos)            /*!< 0x00700000 */
#define RCC_CFG_CLKOUTDIV           RCC_CFG_CLKOUTDIV_Msk
#define RCC_CFG_CLKOUTDIV_DIV1      0x00000000U
#define RCC_CFG_CLKOUTDIV_DIV2      0x00100000U
#define RCC_CFG_CLKOUTDIV_DIV4      0x00200000U
#define RCC_CFG_CLKOUTDIV_DIV8      0x00300000U
#define RCC_CFG_CLKOUTDIV_DIV16     0x00400000U
#define RCC_CFG_CLKOUTDIV_DIV32     0x00500000U
#define RCC_CFG_CLKOUTDIV_DIV64     0x00600000U
#define RCC_CFG_CLKOUTDIV_DIV128    0x00700000U

/* =============================================  Bit definition for RCC_IER register  ============================================= */
#define RCC_IER_LSIRDYIE_Pos        (0U)                                        /*!< LSI clock stabilization interrupt enable */
#define RCC_IER_LSIRDYIE_Msk        (0x1UL << RCC_IER_LSIRDYIE_Pos)             /*!< 0x00000001 */
#define RCC_IER_LSIRDYIE            RCC_IER_LSIRDYIE_Msk
#define RCC_IER_HSIRDYIE_Pos        (2U)                                        /*!< HSI clock stabilization interrupt enable */
#define RCC_IER_HSIRDYIE_Msk        (0x1UL << RCC_IER_HSIRDYIE_Pos)             /*!< 0x00000004 */
#define RCC_IER_HSIRDYIE            RCC_IER_HSIRDYIE_Msk

/* =============================================  Bit definition for RCC_ISR register  ============================================= */
#define RCC_ISR_LSIRDYFLG_Pos       (0U)                                        /*!< LSI clock stabilization interrupt flag */
#define RCC_ISR_LSIRDYFLG_Msk       (0x1UL << RCC_ISR_LSIRDYFLG_Pos)            /*!< 0x00000001 */
#define RCC_ISR_LSIRDYFLG           RCC_ISR_LSIRDYFLG_Msk
#define RCC_ISR_HSIRDYFLG_Pos       (2U)                                        /*!< HSI clock stabilization interrupt flag */
#define RCC_ISR_HSIRDYFLG_Msk       (0x1UL << RCC_ISR_HSIRDYFLG_Pos)            /*!< 0x00000004 */
#define RCC_ISR_HSIRDYFLG           RCC_ISR_HSIRDYFLG_Msk

/* ============================================  Bit definition for RCC_AHBRST register  ============================================ */
#define RCC_AHBRST_DMARST_Pos       (0U)                                        /*!< DMA reset */
#define RCC_AHBRST_DMARST_Msk       (0x1UL << RCC_AHBRST_DMARST_Pos)            /*!< 0x00000001 */
#define RCC_AHBRST_DMARST           RCC_AHBRST_DMARST_Msk
#define RCC_AHBRST_CRCRST_Pos       (3U)                                        /*!< CRC reset */
#define RCC_AHBRST_CRCRST_Msk       (0x1UL << RCC_AHBRST_CRCRST_Pos)            /*!< 0x00000008 */
#define RCC_AHBRST_CRCRST           RCC_AHBRST_CRCRST_Msk
#define RCC_AHBRST_DIVRST_Pos       (4U)                                        /*!< DIV shift reset */
#define RCC_AHBRST_DIVRST_Msk       (0x1UL << RCC_AHBRST_DIVRST_Pos)            /*!< 0x00000010 */
#define RCC_AHBRST_DIVRST           RCC_AHBRST_DIVRST_Msk
#define RCC_AHBRST_GPIOARST_Pos     (16U)                                       /*!< GPIOA reset */
#define RCC_AHBRST_GPIOARST_Msk     (0x1UL << RCC_AHBRST_GPIOARST_Pos)          /*!< 0x00010000 */
#define RCC_AHBRST_GPIOARST         RCC_AHBRST_GPIOARST_Msk
#define RCC_AHBRST_GPIOBRST_Pos     (17U)                                       /*!< GPIOB reset */
#define RCC_AHBRST_GPIOBRST_Msk     (0x1UL << RCC_AHBRST_GPIOBRST_Pos)          /*!< 0x00020000 */
#define RCC_AHBRST_GPIOBRST         RCC_AHBRST_GPIOBRST_Msk

/* ============================================  Bit definition for RCC_APBRST register  ============================================ */
#define RCC_APBRST_EINTRST_Pos      (2U)                                        /*!< EINT reset */
#define RCC_APBRST_EINTRST_Msk      (0x1UL << RCC_APBRST_EINTRST_Pos)           /*!< 0x00000004 */
#define RCC_APBRST_EINTRST          RCC_APBRST_EINTRST_Msk
#define RCC_APBRST_ATMRRST_Pos      (3U)                                        /*!< ATMR resst */
#define RCC_APBRST_ATMRRST_Msk      (0x1UL << RCC_APBRST_ATMRRST_Pos)           /*!< 0x00000008 */
#define RCC_APBRST_ATMRRST          RCC_APBRST_ATMRRST_Msk
#define RCC_APBRST_GTMRRST_Pos      (4U)                                        /*!< GTMR reset */
#define RCC_APBRST_GTMRRST_Msk      (0x1UL << RCC_APBRST_GTMRRST_Pos)           /*!< 0x00000010 */
#define RCC_APBRST_GTMRRST          RCC_APBRST_GTMRRST_Msk
#define RCC_APBRST_BTMR0RST_Pos     (6U)                                        /*!< BTMR0 reset */
#define RCC_APBRST_BTMR0RST_Msk     (0x1UL << RCC_APBRST_BTMR0RST_Pos)          /*!< 0x00000040 */
#define RCC_APBRST_BTMR0RST         RCC_APBRST_BTMR0RST_Msk
#define RCC_APBRST_BTMR1RST_Pos     (7U)                                        /*!< BTMR1 reset */
#define RCC_APBRST_BTMR1RST_Msk     (0x1UL << RCC_APBRST_BTMR1RST_Pos)          /*!< 0x00000080 */
#define RCC_APBRST_BTMR1RST         RCC_APBRST_BTMR1RST_Msk
#define RCC_APBRST_WWDTRST_Pos      (11U)                                       /*!< WWDT reset */
#define RCC_APBRST_WWDTRST_Msk      (0x1UL << RCC_APBRST_WWDTRST_Pos)           /*!< 0x00000800 */
#define RCC_APBRST_WWDTRST          RCC_APBRST_WWDTRST_Msk
#define RCC_APBRST_SPIRST_Pos       (12U)                                       /*!< SPI reset */
#define RCC_APBRST_SPIRST_Msk       (0x1UL << RCC_APBRST_SPIRST_Pos)            /*!< 0x00001000 */
#define RCC_APBRST_SPIRST           RCC_APBRST_SPIRST_Msk
#define RCC_APBRST_USARTRST_Pos     (14U)                                       /*!< USART reset */
#define RCC_APBRST_USARTRST_Msk     (0x1UL << RCC_APBRST_USARTRST_Pos)          /*!< 0x00004000 */
#define RCC_APBRST_USARTRST         RCC_APBRST_USARTRST_Msk
#define RCC_APBRST_UARTRST_Pos      (15U)                                       /*!< UART reset */
#define RCC_APBRST_UARTRST_Msk      (0x1UL << RCC_APBRST_UARTRST_Pos)           /*!< 0x00008000 */
#define RCC_APBRST_UARTRST          RCC_APBRST_UARTRST_Msk
#define RCC_APBRST_I2CRST_Pos       (17U)                                       /*!< I2C reset */
#define RCC_APBRST_I2CRST_Msk       (0x1UL << RCC_APBRST_I2CRST_Pos)            /*!< 0x00020000 */
#define RCC_APBRST_I2CRST           RCC_APBRST_I2CRST_Msk
#define RCC_APBRST_ADCRST_Pos       (21U)                                       /*!< ADC reset */
#define RCC_APBRST_ADCRST_Msk       (0x1UL << RCC_APBRST_ADCRST_Pos)            /*!< 0x00200000 */
#define RCC_APBRST_ADCRST           RCC_APBRST_ADCRST_Msk
#define RCC_APBRST_OPARST_Pos       (22U)                                       /*!< OPA reset */
#define RCC_APBRST_OPARST_Msk       (0x1UL << RCC_APBRST_OPARST_Pos)            /*!< 0x00400000 */
#define RCC_APBRST_OPARST           RCC_APBRST_OPARST_Msk
#define RCC_APBRST_COMP0RST_Pos     (23U)                                       /*!< COMP0 reset */
#define RCC_APBRST_COMP0RST_Msk     (0x1UL << RCC_APBRST_COMP0RST_Pos)          /*!< 0x00800000 */
#define RCC_APBRST_COMP0RST         RCC_APBRST_COMP0RST_Msk
#define RCC_APBRST_COMP1RST_Pos     (24U)                                       /*!< COMP1 reset */
#define RCC_APBRST_COMP1RST_Msk     (0x1UL << RCC_APBRST_COMP1RST_Pos)          /*!< 0x01000000 */
#define RCC_APBRST_COMP1RST         RCC_APBRST_COMP1RST_Msk

/* ============================================  Bit definition for RCC_AHBCLKEN register  ============================================= */
#define RCC_AHBCLKEN_DMAEN_Pos      (0U)                                        /*!< DMA clock enable */
#define RCC_AHBCLKEN_DMAEN_Msk      (0x1UL << RCC_AHBCLKEN_DMAEN_Pos)           /*!< 0x00000001 */
#define RCC_AHBCLKEN_DMAEN          RCC_AHBCLKEN_DMAEN_Msk
#define RCC_AHBCLKEN_CRCEN_Pos      (3U)                                        /*!< CRC clock enable */
#define RCC_AHBCLKEN_CRCEN_Msk      (0x1UL << RCC_AHBCLKEN_CRCEN_Pos)           /*!< 0x00000008 */
#define RCC_AHBCLKEN_CRCEN          RCC_AHBCLKEN_CRCEN_Msk
#define RCC_AHBCLKEN_DIVEN_Pos      (4U)                                        /*!< DIV SHIFT clock enable */
#define RCC_AHBCLKEN_DIVEN_Msk      (0x1UL << RCC_AHBCLKEN_DIVEN_Pos)           /*!< 0x00000010 */
#define RCC_AHBCLKEN_DIVEN          RCC_AHBCLKEN_DIVEN_Msk
#define RCC_AHBCLKEN_GPIOAEN_Pos    (16U)                                       /*!< GPIOA clock enable */
#define RCC_AHBCLKEN_GPIOAEN_Msk    (0x1UL << RCC_AHBCLKEN_GPIOAEN_Pos)         /*!< 0x00010000 */
#define RCC_AHBCLKEN_GPIOAEN        RCC_AHBCLKEN_GPIOAEN_Msk
#define RCC_AHBCLKEN_GPIOBEN_Pos    (17U)                                       /*!< GPIOB clock enable */
#define RCC_AHBCLKEN_GPIOBEN_Msk    (0x1UL << RCC_AHBCLKEN_GPIOBEN_Pos)         /*!< 0x00020000 */
#define RCC_AHBCLKEN_GPIOBEN        RCC_AHBCLKEN_GPIOBEN_Msk

/* ============================================  Bit definition for RCC_APBCLKEN register  ============================================= */
#define RCC_APBCLKEN_EINTEN_Pos     (2U)                                        /*!< EINT enable */
#define RCC_APBCLKEN_EINTEN_Msk     (0x1UL << RCC_APBCLKEN_EINTEN_Pos)          /*!< 0x00000004 */
#define RCC_APBCLKEN_EINTEN         RCC_APBCLKEN_EINTEN_Msk
#define RCC_APBCLKEN_ATMREN_Pos     (3U)                                        /*!< ATMR enable */
#define RCC_APBCLKEN_ATMREN_Msk     (0x1UL << RCC_APBCLKEN_ATMREN_Pos)          /*!< 0x00000008 */
#define RCC_APBCLKEN_ATMREN         RCC_APBCLKEN_ATMREN_Msk
#define RCC_APBCLKEN_GTMREN_Pos     (4U)                                        /*!< GTMR enable */
#define RCC_APBCLKEN_GTMREN_Msk     (0x1UL << RCC_APBCLKEN_GTMREN_Pos)          /*!< 0x00000010 */
#define RCC_APBCLKEN_GTMREN         RCC_APBCLKEN_GTMREN_Msk
#define RCC_APBCLKEN_BTMR0EN_Pos    (6U)                                        /*!< BTMR0 enable */
#define RCC_APBCLKEN_BTMR0EN_Msk    (0x1UL << RCC_APBCLKEN_BTMR0EN_Pos)         /*!< 0x00000040 */
#define RCC_APBCLKEN_BTMR0EN        RCC_APBCLKEN_BTMR0EN_Msk
#define RCC_APBCLKEN_BTMR1EN_Pos    (7U)                                        /*!< BTMR1 enable */
#define RCC_APBCLKEN_BTMR1EN_Msk    (0x1UL << RCC_APBCLKEN_BTMR1EN_Pos)         /*!< 0x00000080 */
#define RCC_APBCLKEN_BTMR1EN        RCC_APBCLKEN_BTMR1EN_Msk
#define RCC_APBCLKEN_IWDTEN_Pos     (10U)                                       /*!< IWDT enable */
#define RCC_APBCLKEN_IWDTEN_Msk     (0x1UL << RCC_APBCLKEN_IWDTEN_Pos)          /*!< 0x00000400 */
#define RCC_APBCLKEN_IWDTEN         RCC_APBCLKEN_IWDTEN_Msk
#define RCC_APBCLKEN_WWDTEN_Pos     (11U)                                       /*!< WWDT enable */
#define RCC_APBCLKEN_WWDTEN_Msk     (0x1UL << RCC_APBCLKEN_WWDTEN_Pos)          /*!< 0x00000800 */
#define RCC_APBCLKEN_WWDTEN         RCC_APBCLKEN_WWDTEN_Msk
#define RCC_APBCLKEN_SPIEN_Pos      (12U)                                       /*!< SPI enable */
#define RCC_APBCLKEN_SPIEN_Msk      (0x1UL << RCC_APBCLKEN_SPIEN_Pos)           /*!< 0x00001000 */
#define RCC_APBCLKEN_SPIEN          RCC_APBCLKEN_SPIEN_Msk
#define RCC_APBCLKEN_USARTEN_Pos    (14U)                                       /*!< USART enable */
#define RCC_APBCLKEN_USARTEN_Msk    (0x1UL << RCC_APBCLKEN_USARTEN_Pos)         /*!< 0x00004000 */
#define RCC_APBCLKEN_USARTEN        RCC_APBCLKEN_USARTEN_Msk
#define RCC_APBCLKEN_UARTEN_Pos     (15U)                                       /*!< UART enable */
#define RCC_APBCLKEN_UARTEN_Msk     (0x1UL << RCC_APBCLKEN_UARTEN_Pos)          /*!< 0x00008000 */
#define RCC_APBCLKEN_UARTEN         RCC_APBCLKEN_UARTEN_Msk
#define RCC_APBCLKEN_I2CEN_Pos      (17U)                                       /*!< I2C enable */
#define RCC_APBCLKEN_I2CEN_Msk      (0x1UL << RCC_APBCLKEN_I2CEN_Pos)           /*!< 0x00020000 */
#define RCC_APBCLKEN_I2CEN          RCC_APBCLKEN_I2CEN_Msk
#define RCC_APBCLKEN_ADCEN_Pos      (21U)                                       /*!< ADC enable */
#define RCC_APBCLKEN_ADCEN_Msk      (0x1UL << RCC_APBCLKEN_ADCEN_Pos)           /*!< 0x00200000 */
#define RCC_APBCLKEN_ADCEN          RCC_APBCLKEN_ADCEN_Msk
#define RCC_APBCLKEN_OPAEN_Pos      (22U)                                       /*!< OPA enable */
#define RCC_APBCLKEN_OPAEN_Msk      (0x1UL << RCC_APBCLKEN_OPAEN_Pos)           /*!< 0x00400000 */
#define RCC_APBCLKEN_OPAEN          RCC_APBCLKEN_OPAEN_Msk
#define RCC_APBCLKEN_COMP0EN_Pos    (23U)                                       /*!< COMP0 enable */
#define RCC_APBCLKEN_COMP0EN_Msk    (0x1UL << RCC_APBCLKEN_COMP0EN_Pos)         /*!< 0x00800000 */
#define RCC_APBCLKEN_COMP0EN        RCC_APBCLKEN_COMP0EN_Msk
#define RCC_APBCLKEN_COMP1EN_Pos    (24U)                                       /*!< COMP1 enable */
#define RCC_APBCLKEN_COMP1EN_Msk    (0x1UL << RCC_APBCLKEN_COMP1EN_Pos)         /*!< 0x01000000 */
#define RCC_APBCLKEN_COMP1EN        RCC_APBCLKEN_COMP1EN_Msk

/* ============================================  Bit definition for RCC_ADCCR register  ============================================= */
#define RCC_ADCCR_ADCCLKDIV_Pos     (0U)                                        /*!< ADC clock division */
#define RCC_ADCCR_ADCCLKDIV_0       (0x1UL << RCC_ADCCR_ADCCLKDIV_Pos)          /*!< 0x00000001 */
#define RCC_ADCCR_ADCCLKDIV_1       (0x2UL << RCC_ADCCR_ADCCLKDIV_Pos)          /*!< 0x00000002 */
#define RCC_ADCCR_ADCCLKDIV_Msk     (0x3UL << RCC_ADCCR_ADCCLKDIV_Pos)          /*!< 0x00000003 */
#define RCC_ADCCR_ADCCLKDIV         RCC_ADCCR_ADCCLKDIV_Msk

/* ============================================  Bit definition for RCC_RSTCSR register  ============================================ */
#define RCC_RSTCSR_OPTRSTFLG_Pos    (0U)                                        /*!< OB reload system reset flag */
#define RCC_RSTCSR_OPTRSTFLG_Msk    (0x1UL << RCC_RSTCSR_OPTRSTFLG_Pos)         /*!< 0x00000001 */
#define RCC_RSTCSR_OPTRSTFLG        RCC_RSTCSR_OPTRSTFLG_Msk
#define RCC_RSTCSR_NRSTRSTFLG_Pos   (1U)                                        /*!< NRST reset flag */
#define RCC_RSTCSR_NRSTRSTFLG_Msk   (0x1UL << RCC_RSTCSR_NRSTRSTFLG_Pos)        /*!< 0x00000002 */
#define RCC_RSTCSR_NRSTRSTFLG       RCC_RSTCSR_NRSTRSTFLG_Msk
#define RCC_RSTCSR_PVDRSTFLG_Pos    (2U)                                        /*!< PVD system reset flag */
#define RCC_RSTCSR_PVDRSTFLG_Msk    (0x1UL << RCC_RSTCSR_PVDRSTFLG_Pos)         /*!< 0x00000004 */
#define RCC_RSTCSR_PVDRSTFLG        RCC_RSTCSR_PVDRSTFLG_Msk
#define RCC_RSTCSR_SWRSTFLG_Pos     (3U)                                        /*!< Soft system reset flag */
#define RCC_RSTCSR_SWRSTFLG_Msk     (0x1UL << RCC_RSTCSR_SWRSTFLG_Pos)          /*!< 0x00000008 */
#define RCC_RSTCSR_SWRSTFLG         RCC_RSTCSR_SWRSTFLG_Msk
#define RCC_RSTCSR_IWDTRSTFLG_Pos   (4U)                                        /*!< IWDT system reset flag */
#define RCC_RSTCSR_IWDTRSTFLG_Msk   (0x1UL << RCC_RSTCSR_IWDTRSTFLG_Pos)        /*!< 0x00000010 */
#define RCC_RSTCSR_IWDTRSTFLG       RCC_RSTCSR_IWDTRSTFLG_Msk
#define RCC_RSTCSR_WWDTRSTFLG_Pos   (5U)                                        /*!< WWDT system reset flag */
#define RCC_RSTCSR_WWDTRSTFLG_Msk   (0x1UL << RCC_RSTCSR_WWDTRSTFLG_Pos)        /*!< 0x00000020 */
#define RCC_RSTCSR_WWDTRSTFLG       RCC_RSTCSR_WWDTRSTFLG_Msk
#define RCC_RSTCSR_LOCKUPRSTFLG_Pos (6U)                                        /*!< LOCKUP system reset flag */
#define RCC_RSTCSR_LOCKUPRSTFLG_Msk (0x1UL << RCC_RSTCSR_LOCKUPRSTFLG_Pos)      /*!< 0x00000040 */
#define RCC_RSTCSR_LOCKUPRSTFLG     RCC_RSTCSR_LOCKUPRSTFLG_Msk
#define RCC_RSTCSR_PORRSTFLG_Pos    (7U)                                        /*!< POR/PDR reset flag */
#define RCC_RSTCSR_PORRSTFLG_Msk    (0x1UL << RCC_RSTCSR_PORRSTFLG_Pos)         /*!< 0x00000080 */
#define RCC_RSTCSR_PORRSTFLG        RCC_RSTCSR_PORRSTFLG_Msk
#define RCC_RSTCSR_NRSTFLTSEL_Pos   (8U)                                        /*!< NRST filter select */
#define RCC_RSTCSR_NRSTFLTSEL_0     (0x1UL << RCC_RSTCSR_NRSTFLTSEL_Pos)        /*!< 0x00000100 */
#define RCC_RSTCSR_NRSTFLTSEL_1     (0x2UL << RCC_RSTCSR_NRSTFLTSEL_Pos)        /*!< 0x00000200 */
#define RCC_RSTCSR_NRSTFLTSEL_Msk   (0x3UL << RCC_RSTCSR_NRSTFLTSEL_Pos)        /*!< 0x00000300 */
#define RCC_RSTCSR_NRSTFLTSEL       RCC_RSTCSR_NRSTFLTSEL_Msk
#define RCC_RSTCSR_PVDRSTEN_Pos     (14U)                                       /*!< PVD system reset enable */
#define RCC_RSTCSR_PVDRSTEN_Msk     (0x1UL << RCC_RSTCSR_PVDRSTEN_Pos)          /*!< 0x00004000 */
#define RCC_RSTCSR_PVDRSTEN         RCC_RSTCSR_PVDRSTEN_Msk
#define RCC_RSTCSR_LOCKUPRSTEN_Pos  (15U)                                       /*!< LOCKUP system reset enable */
#define RCC_RSTCSR_LOCKUPRSTEN_Msk  (0x1UL << RCC_RSTCSR_LOCKUPRSTEN_Pos)       /*!< 0x00008000 */
#define RCC_RSTCSR_LOCKUPRSTEN      RCC_RSTCSR_LOCKUPRSTEN_Msk

/* ============================================  Bit definition for RCC_AONCSR register  ============================================ */
#define RCC_AONCSR_LSIEN_Pos        (0U)                                        /*!< LSI clock enable */
#define RCC_AONCSR_LSIEN_Msk        (0x1UL << RCC_AONCSR_LSIEN_Pos)             /*!< 0x00000001 */
#define RCC_AONCSR_LSIEN            RCC_AONCSR_LSIEN_Msk
#define RCC_AONCSR_LSIRDY_Pos       (1U)                                        /*!< LSI clock ready flag */
#define RCC_AONCSR_LSIRDY_Msk       (0x1UL << RCC_AONCSR_LSIRDY_Pos)            /*!< 0x00000002 */
#define RCC_AONCSR_LSIRDY           RCC_AONCSR_LSIRDY_Msk
#define RCC_AONCSR_LPTMREN_Pos      (15U)                                       /*!< LPTMR clock enable */
#define RCC_AONCSR_LPTMREN_Msk      (0x1UL << RCC_AONCSR_LPTMREN_Pos)           /*!< 0x00008000 */
#define RCC_AONCSR_LPTMREN          RCC_AONCSR_LPTMREN_Msk
#define RCC_AONCSR_LPTMRRST_Pos     (16U)                                       /*!< LPTMR reset */
#define RCC_AONCSR_LPTMRRST_Msk     (0x1UL << RCC_AONCSR_LPTMRRST_Pos)          /*!< 0x00010000 */
#define RCC_AONCSR_LPTMRRST         RCC_AONCSR_LPTMRRST_Msk

/* =========================================================================================================================== */
/* ================                                           FLASH                                           ================ */
/* =========================================================================================================================== */

/* =========================================================  RKEY  ========================================================== */
#define FLASH_WKEY_WKEY_Pos         (0U)                                        /*!< RKEY (Bit 0)                              */
#define FLASH_WKEY_WKEY_Msk         (0x1UL << FLASH_WKEY_WKEY_Pos)              /*!< RKEY (Bitfield-Mask: 0x01)                */
#define FLASH_WKEY_WKEY             FLASH_WKEY_WKEY_Msk
/* =========================================================  MKEY  ========================================================== */
#define FLASH_MKEY_MKEY_Pos         (0U)                                        /*!< MKEY (Bit 0)                              */
#define FLASH_MKEY_MKEY_Msk         (0x1UL << FLASH_MKEY_MKEY_Pos)              /*!< MKEY (Bitfield-Mask: 0x01)                */
#define FLASH_MKEY_MKEY             FLASH_MKEY_MKEY_Msk
/* ========================================================  NVRCKEY  ======================================================== */
#define FLASH_NVRCKEY_NVRCKEY_Pos   (0U)                                        /*!< NVRCKEY (Bit 0)                           */
#define FLASH_NVRCKEY_NVRCKEY_Msk   (0x1UL << FLASH_NVRCKEY_NVRCKEY_Pos)        /*!< NVRCKEY (Bitfield-Mask: 0x01)             */
#define FLASH_NVRCKEY_NVRCKEY       FLASH_NVRCKEY_NVRCKEY_Msk
/* ==========================================================  CR  =========================================================== */
#define FLASH_CR_OPERATE_Pos        (0U)                                        /*!< OPERATE (Bit 0)                           */
#define FLASH_CR_OPERATE_Msk        (0x3UL << FLASH_CR_OPERATE_Pos)             /*!< OPERATE (Bitfield-Mask: 0x01)             */
#define FLASH_CR_OPERATE            FLASH_CR_OPERATE_Msk
#define FLASH_CR_OPERATE_0          (0x1UL << FLASH_CR_OPERATE_Pos)
#define FLASH_CR_OPERATE_1          (0x2UL << FLASH_CR_OPERATE_Pos)

#define FLASH_CR_READONLY_Pos       (4U)                                        /*!< READONLY (Bit 4)                          */
#define FLASH_CR_READONLY_Msk       (0x1UL << FLASH_CR_READONLY_Pos)            /*!< READONLY (Bitfield-Mask: 0x01)            */
#define FLASH_CR_READONLY           FLASH_CR_READONLY_Msk

#define FLASH_CR_PREEN_Pos          (10U)                                       /*!< PRE_EN (Bit 10)                           */
#define FLASH_CR_PREEN_Msk          (0x1UL << FLASH_CR_PREEN_Pos)               /*!< PRE_EN (Bitfield-Mask: 0x01)              */
#define FLASH_CR_PREEN              FLASH_CR_PREEN_Msk
#define FLASH_CR_OPTLOAD_Pos        (15U)                                       /*!< FORCE_OPTLOAD (Bit 15)                    */
#define FLASH_CR_OPTLOAD_Msk        (0x1UL << FLASH_CR_OPTLOAD_Pos)             /*!< FORCE_OPTLOAD (Bitfield-Mask: 0x01)       */
#define FLASH_CR_OPTLOAD            FLASH_CR_OPTLOAD_Msk
/* ==========================================================  ER  =========================================================== */
#define FLASH_IER_OPEIEN_Pos        (4U)                                        /*!< OPEIE (Bit 4)                             */
#define FLASH_IER_OPEIEN_Msk        (0x1UL << FLASH_IER_OPEIEN_Pos)             /*!< OPEIE (Bitfield-Mask: 0x01)               */
#define FLASH_IER_OPEIEN            FLASH_IER_OPEIEN_Msk
#define FLASH_IER_KEYIEN_Pos        (5U)                                        /*!< KEYIE (Bit 5)                             */
#define FLASH_IER_KEYIEN_Msk        (0x1UL << FLASH_IER_KEYIEN_Pos)             /*!< KEYIE (Bitfield-Mask: 0x01)               */
#define FLASH_IER_KEYIEN            FLASH_IER_KEYIEN_Msk
#define FLASH_IER_RPTIEN_Pos        (6U)                                        /*!< RPTIE (Bit 6)                             */
#define FLASH_IER_RPTIEN_Msk        (0x1UL << FLASH_IER_RPTIEN_Pos)             /*!< RPTIE (Bitfield-Mask: 0x01)               */
#define FLASH_IER_RPTIEN            FLASH_IER_RPTIEN_Msk
/* ==========================================================  SR  =========================================================== */
#define FLASH_SR_BUSYFLG_Pos        (0U)                                        /*!< BUSY (Bit 0)                              */
#define FLASH_SR_BUSYFLG_Msk        (0x1UL << FLASH_SR_BUSYFLG_Pos)             /*!< BUSY (Bitfield-Mask: 0x01)                */
#define FLASH_SR_BUSYFLG            FLASH_SR_BUSYFLG_Msk
#define FLASH_SR_OPENDFLG_Pos       (4U)                                        /*!< OPEIE (Bit 4)                             */
#define FLASH_SR_OPENDFLG_Msk       (0x1UL << FLASH_SR_OPENDFLG_Pos)            /*!< OPEIE (Bitfield-Mask: 0x01)               */
#define FLASH_SR_OPENDFLG           FLASH_SR_OPENDFLG_Msk
#define FLASH_SR_KEYERRFLG_Pos      (5U)                                        /*!< KEYIE (Bit 5)                             */
#define FLASH_SR_KEYERRFLG_Msk      (0x1UL << FLASH_SR_KEYERRFLG_Pos)           /*!< KEYIE (Bitfield-Mask: 0x01)               */
#define FLASH_SR_KEYERRFLG          FLASH_SR_KEYERRFLG_Msk
#define FLASH_SR_RPTERRFLG_Pos      (6U)                                        /*!< RPTIE (Bit 6)                             */
#define FLASH_SR_RPTERRFLG_Msk      (0x1UL << FLASH_SR_RPTERRFLG_Pos)           /*!< RPTIE (Bitfield-Mask: 0x01)               */
#define FLASH_SR_RPTERRFLG          FLASH_SR_RPTERRFLG_Msk
/* ==========================================================  CR1  ========================================================== */
#define FLASH_CR1_LATENCY_Pos       (0U)                                        /*!< LATENCY (Bit 0)                           */
#define FLASH_CR1_LATENCY_Msk       (0x3UL << FLASH_CR1_LATENCY_Pos)            /*!< LATENCY (Bitfield-Mask: 0x03)             */
#define FLASH_CR1_LATENCY           FLASH_CR1_LATENCY_Msk
#define FLASH_CR1_LATENCY_0         (0x1UL << FLASH_CR1_LATENCY_Pos)
#define FLASH_CR1_LATENCY_1         (0x2UL << FLASH_CR1_LATENCY_Pos)


/* =========================================================================================================================== */
/* ================                                            CRC                                            ================ */
/* =========================================================================================================================== */

/* =========================================================  DATA  ========================================================== */
#define CRC_DR_DATA_Pos             (0U)                                        /*!< data (Bit 0)                                          */
#define CRC_DR_DATA_Msk             (0xFFFFFFFFUL << CRC_DR_DATA_Pos)           /*!< data (Bitfield-Mask: 0xffffffff)                      */
#define CRC_DR_DATA                 CRC_DR_DATA_Msk
/* ==========================================================  CR  =========================================================== */
#define CRC_CR_CRCSEL_Pos           (0U)                                        /*!< CRC_SEL (Bit 0)                                       */
#define CRC_CR_CRCSEL_Msk           (0x1UL << CRC_CR_CRCSEL_Pos)                /*!< CRC_SEL (Bitfield-Mask: 0x01)                         */
#define CRC_CR_CRCSEL               CRC_CR_CRCSEL_Msk
#define CRC_CR_INFLIP_Pos           (1U)                                        /*!< INFLIP (Bit 1)                                        */
#define CRC_CR_INFLIP_0             (0x1UL << CRC_CR_INFLIP_Pos)
#define CRC_CR_INFLIP_1             (0x2UL << CRC_CR_INFLIP_Pos)
#define CRC_CR_INFLIP_Msk           (0x3UL << CRC_CR_INFLIP_Pos)                /*!< INFLIP (Bitfield-Mask: 0x03)                          */
#define CRC_CR_INFLIP               CRC_CR_INFLIP_Msk
#define CRC_CR_OUTFLIP_Pos          (3U)                                        /*!< OUTFLIP (Bit 3)                                       */
#define CRC_CR_OUTFLIP_Msk          (0x1UL << CRC_CR_OUTFLIP_Pos)               /*!< OUTFLIP (Bitfield-Mask: 0x01)                         */
#define CRC_CR_OUTFLIP              CRC_CR_OUTFLIP_Msk
/* ========================================================  RSTDATA  ======================================================== */
#define CRC_INIT_INIT_Pos           (0U)                                        /*!< RSTDATA (Bit 0)                                       */
#define CRC_INIT_INIT_Msk           (0xFFFFFFFFUL << CRC_INIT_INIT_Pos)         /*!< RSTDATA (Bitfield-Mask: 0xffffffff)                   */
#define CRC_INIT_INIT               CRC_INIT_INIT_Msk                           /*!< RSTDATA (Bitfield-Mask: 0xffffffff)                   */


/* =========================================================================================================================== */
/* ================                                            DIV                                            ================ */
/* =========================================================================================================================== */

/* =========================================================  DVDR  ========================================================== */
#define DIV_DVDR_DIVIDEND_Pos       (0U)                                        /*!< DIVIDEND (Bit 0)                                      */
#define DIV_DVDR_DIVIDEND_Msk       (0xFFFFFFFFUL << DIV_DVDR_DIVIDEND_Pos)     /*!< DIVIDEND (Bitfield-Mask: 0xffffffff)                  */
#define DIV_DVDR_DIVIDEND           DIV_DVDR_DIVIDEND_Msk
/* =========================================================  DVSR  ========================================================== */
#define DIV_DVSR_DIVISOR_Pos        (0U)                                        /*!< DIVISOR (Bit 0)                                       */
#define DIV_DVSR_DIVISOR_Msk        (0xFFFFFFFFUL << DIV_DVSR_DIVISOR_Pos)      /*!< DIVISOR (Bitfield-Mask: 0xffffffff)                   */
#define DIV_DVSR_DIVISOR            DIV_DVSR_DIVISOR_Msk
/* =========================================================  QUOTR  ========================================================= */
#define DIV_QUOTR_QUOTIENT_Pos      (0U)                                        /*!< QUOTIENT (Bit 0)                                      */
#define DIV_QUOTR_QUOTIENT_Msk      (0xFFFFFFFFUL << DIV_QUOTR_QUOTIENT_Pos)    /*!< QUOTIENT (Bitfield-Mask: 0xffffffff)                  */
#define DIV_QUOTR_QUOTIENT          DIV_QUOTR_QUOTIENT_Msk
/* =========================================================  RMDR  ========================================================== */
#define DIV_RMDR_REMAINDER_Pos      (0U)                                        /*!< REMAINDER (Bit 0)                                     */
#define DIV_RMDR_REMAINDER_Msk      (0xFFFFFFFFUL << DIV_RMDR_REMAINDER_Pos)    /*!< REMAINDER (Bitfield-Mask: 0xffffffff)                 */
#define DIV_RMDR_REMAINDER          DIV_RMDR_REMAINDER_Msk
/* ==========================================================  SR  =========================================================== */
#define DIV_SR_OVF_Pos              (0U)                                        /*!< OVF (Bit 0)                                           */
#define DIV_SR_OVF_Msk              (0x1UL << DIV_SR_OVF_Pos)                   /*!< OVF (Bitfield-Mask: 0x01)                             */
#define DIV_SR_OVF                  DIV_SR_OVF_Msk
/* ==========================================================  CR  =========================================================== */
#define DIV_CR_USIGNEN_Pos          (0U)                                        /*!< USIGN (Bit 0)                                         */
#define DIV_CR_USIGNEN_Msk          (0x1UL << DIV_CR_USIGNEN_Pos)               /*!< USIGN (Bitfield-Mask: 0x01)                           */
#define DIV_CR_USIGNEN              DIV_CR_USIGNEN_Msk

/* ================================================================================================================================== */
/* =============================================================  GPIO  ============================================================= */
/* ================================================================================================================================== */
/* ============================================  Bit definition for GPIO_MDR register  ============================================== */
#define GPIO_MDR_MD0_Pos            (0U)
#define GPIO_MDR_MD0_Msk            (0x3UL << GPIO_MDR_MD0_Pos)                 /*!< 0x00000003 */
#define GPIO_MDR_MD0                GPIO_MDR_MD0_Msk
#define GPIO_MDR_MD0_0              (0x1UL << GPIO_MDR_MD0_Pos)                 /*!< 0x00000001 */
#define GPIO_MDR_MD0_1              (0x2UL << GPIO_MDR_MD0_Pos)                 /*!< 0x00000002 */
#define GPIO_MDR_MD1_Pos            (2U)
#define GPIO_MDR_MD1_Msk            (0x3UL << GPIO_MDR_MD1_Pos)                 /*!< 0x0000000C */
#define GPIO_MDR_MD1                GPIO_MDR_MD1_Msk
#define GPIO_MDR_MD1_0              (0x1UL << GPIO_MDR_MD1_Pos)                 /*!< 0x00000004 */
#define GPIO_MDR_MD1_1              (0x2UL << GPIO_MDR_MD1_Pos)                 /*!< 0x00000008 */
#define GPIO_MDR_MD2_Pos            (4U)
#define GPIO_MDR_MD2_Msk            (0x3UL << GPIO_MDR_MD2_Pos)                 /*!< 0x00000030 */
#define GPIO_MDR_MD2                GPIO_MDR_MD2_Msk
#define GPIO_MDR_MD2_0              (0x1UL << GPIO_MDR_MD2_Pos)                 /*!< 0x00000010 */
#define GPIO_MDR_MD2_1              (0x2UL << GPIO_MDR_MD2_Pos)                 /*!< 0x00000020 */
#define GPIO_MDR_MD3_Pos            (6U)
#define GPIO_MDR_MD3_Msk            (0x3UL << GPIO_MDR_MD3_Pos)                 /*!< 0x000000C0 */
#define GPIO_MDR_MD3                GPIO_MDR_MD3_Msk
#define GPIO_MDR_MD3_0              (0x1UL << GPIO_MDR_MD3_Pos)                 /*!< 0x00000040 */
#define GPIO_MDR_MD3_1              (0x2UL << GPIO_MDR_MD3_Pos)                 /*!< 0x00000080 */
#define GPIO_MDR_MD4_Pos            (8U)
#define GPIO_MDR_MD4_Msk            (0x3UL << GPIO_MDR_MD4_Pos)                 /*!< 0x00000300 */
#define GPIO_MDR_MD4                GPIO_MDR_MD4_Msk
#define GPIO_MDR_MD4_0              (0x1UL << GPIO_MDR_MD4_Pos)                 /*!< 0x00000100 */
#define GPIO_MDR_MD4_1              (0x2UL << GPIO_MDR_MD4_Pos)                 /*!< 0x00000200 */
#define GPIO_MDR_MD5_Pos            (10U)
#define GPIO_MDR_MD5_Msk            (0x3UL << GPIO_MDR_MD5_Pos)                 /*!< 0x00000C00 */
#define GPIO_MDR_MD5                GPIO_MDR_MD5_Msk
#define GPIO_MDR_MD5_0              (0x1UL << GPIO_MDR_MD5_Pos)                 /*!< 0x00000400 */
#define GPIO_MDR_MD5_1              (0x2UL << GPIO_MDR_MD5_Pos)                 /*!< 0x00000800 */
#define GPIO_MDR_MD6_Pos            (12U)
#define GPIO_MDR_MD6_Msk            (0x3UL << GPIO_MDR_MD6_Pos)                 /*!< 0x00003000 */
#define GPIO_MDR_MD6                GPIO_MDR_MD6_Msk
#define GPIO_MDR_MD6_0              (0x1UL << GPIO_MDR_MD6_Pos)                 /*!< 0x00001000 */
#define GPIO_MDR_MD6_1              (0x2UL << GPIO_MDR_MD6_Pos)                 /*!< 0x00002000 */
#define GPIO_MDR_MD7_Pos            (14U)
#define GPIO_MDR_MD7_Msk            (0x3UL << GPIO_MDR_MD7_Pos)                 /*!< 0x0000C000 */
#define GPIO_MDR_MD7                GPIO_MDR_MD7_Msk
#define GPIO_MDR_MD7_0              (0x1UL << GPIO_MDR_MD7_Pos)                 /*!< 0x00004000 */
#define GPIO_MDR_MD7_1              (0x2UL << GPIO_MDR_MD7_Pos)                 /*!< 0x00008000 */
#define GPIO_MDR_MD8_Pos            (16U)
#define GPIO_MDR_MD8_Msk            (0x3UL << GPIO_MDR_MD8_Pos)                 /*!< 0x00030000 */
#define GPIO_MDR_MD8                GPIO_MDR_MD8_Msk
#define GPIO_MDR_MD8_0              (0x1UL << GPIO_MDR_MD8_Pos)                 /*!< 0x00010000 */
#define GPIO_MDR_MD8_1              (0x2UL << GPIO_MDR_MD8_Pos)                 /*!< 0x00020000 */
#define GPIO_MDR_MD9_Pos            (18U)
#define GPIO_MDR_MD9_Msk            (0x3UL << GPIO_MDR_MD9_Pos)                 /*!< 0x000C0000 */
#define GPIO_MDR_MD9                GPIO_MDR_MD9_Msk
#define GPIO_MDR_MD9_0              (0x1UL << GPIO_MDR_MD9_Pos)                 /*!< 0x00040000 */
#define GPIO_MDR_MD9_1              (0x2UL << GPIO_MDR_MD9_Pos)                 /*!< 0x00080000 */
#define GPIO_MDR_MD10_Pos           (20U)
#define GPIO_MDR_MD10_Msk           (0x3UL << GPIO_MDR_MD10_Pos)                /*!< 0x00300000 */
#define GPIO_MDR_MD10               GPIO_MDR_MD10_Msk
#define GPIO_MDR_MD10_0             (0x1UL << GPIO_MDR_MD10_Pos)                /*!< 0x00100000 */
#define GPIO_MDR_MD10_1             (0x2UL << GPIO_MDR_MD10_Pos)                /*!< 0x00200000 */
#define GPIO_MDR_MD11_Pos           (22U)
#define GPIO_MDR_MD11_Msk           (0x3UL << GPIO_MDR_MD11_Pos)                /*!< 0x00C00000 */
#define GPIO_MDR_MD11               GPIO_MDR_MD11_Msk
#define GPIO_MDR_MD11_0             (0x1UL << GPIO_MDR_MD11_Pos)                /*!< 0x00400000 */
#define GPIO_MDR_MD11_1             (0x2UL << GPIO_MDR_MD11_Pos)                /*!< 0x00800000 */
#define GPIO_MDR_MD12_Pos           (24U)
#define GPIO_MDR_MD12_Msk           (0x3UL << GPIO_MDR_MD12_Pos)                /*!< 0x03000000 */
#define GPIO_MDR_MD12               GPIO_MDR_MD12_Msk
#define GPIO_MDR_MD12_0             (0x1UL << GPIO_MDR_MD12_Pos)                /*!< 0x01000000 */
#define GPIO_MDR_MD12_1             (0x2UL << GPIO_MDR_MD12_Pos)                /*!< 0x02000000 */
#define GPIO_MDR_MD13_Pos           (26U)
#define GPIO_MDR_MD13_Msk           (0x3UL << GPIO_MDR_MD13_Pos)                /*!< 0x0C000000 */
#define GPIO_MDR_MD13               GPIO_MDR_MD13_Msk
#define GPIO_MDR_MD13_0             (0x1UL << GPIO_MDR_MD13_Pos)                /*!< 0x04000000 */
#define GPIO_MDR_MD13_1             (0x2UL << GPIO_MDR_MD13_Pos)                /*!< 0x08000000 */
#define GPIO_MDR_MD14_Pos           (28U)
#define GPIO_MDR_MD14_Msk           (0x3UL << GPIO_MDR_MD14_Pos)                /*!< 0x30000000 */
#define GPIO_MDR_MD14               GPIO_MDR_MD14_Msk
#define GPIO_MDR_MD14_0             (0x1UL << GPIO_MDR_MD14_Pos)                /*!< 0x10000000 */
#define GPIO_MDR_MD14_1             (0x2UL << GPIO_MDR_MD14_Pos)                /*!< 0x20000000 */
#define GPIO_MDR_MD15_Pos           (30U)
#define GPIO_MDR_MD15_Msk           (0x3UL << GPIO_MDR_MD15_Pos)                /*!< 0xC0000000 */
#define GPIO_MDR_MD15               GPIO_MDR_MD15_Msk
#define GPIO_MDR_MD15_0             (0x1UL << GPIO_MDR_MD15_Pos)                /*!< 0x40000000 */
#define GPIO_MDR_MD15_1             (0x2UL << GPIO_MDR_MD15_Pos)                /*!< 0x80000000 */

/* =============================================  Bit definition for GPIO_INENR register  ============================================= */
#define GPIO_INENR_INEN0_Pos        (0U)
#define GPIO_INENR_INEN0_Msk        (0x1UL << GPIO_INENR_INEN0_Pos)             /*!< 0x00000001 */
#define GPIO_INENR_INEN0            GPIO_INENR_INEN0_Msk
#define GPIO_INENR_INEN1_Pos        (1U)
#define GPIO_INENR_INEN1_Msk        (0x1UL << GPIO_INENR_INEN1_Pos)             /*!< 0x00000002 */
#define GPIO_INENR_INEN1            GPIO_INENR_INEN1_Msk
#define GPIO_INENR_INEN2_Pos        (2U)
#define GPIO_INENR_INEN2_Msk        (0x1UL << GPIO_INENR_INEN2_Pos)             /*!< 0x00000004 */
#define GPIO_INENR_INEN2            GPIO_INENR_INEN2_Msk
#define GPIO_INENR_INEN3_Pos        (3U)
#define GPIO_INENR_INEN3_Msk        (0x1UL << GPIO_INENR_INEN3_Pos)             /*!< 0x00000008 */
#define GPIO_INENR_INEN3            GPIO_INENR_INEN3_Msk
#define GPIO_INENR_INEN4_Pos        (4U)
#define GPIO_INENR_INEN4_Msk        (0x1UL << GPIO_INENR_INEN4_Pos)             /*!< 0x00000010 */
#define GPIO_INENR_INEN4            GPIO_INENR_INEN4_Msk
#define GPIO_INENR_INEN5_Pos        (5U)
#define GPIO_INENR_INEN5_Msk        (0x1UL << GPIO_INENR_INEN5_Pos)             /*!< 0x00000020 */
#define GPIO_INENR_INEN5            GPIO_INENR_INEN5_Msk
#define GPIO_INENR_INEN6_Pos        (6U)
#define GPIO_INENR_INEN6_Msk        (0x1UL << GPIO_INENR_INEN6_Pos)             /*!< 0x00000040 */
#define GPIO_INENR_INEN6            GPIO_INENR_INEN6_Msk
#define GPIO_INENR_INEN7_Pos        (7U)
#define GPIO_INENR_INEN7_Msk        (0x1UL << GPIO_INENR_INEN7_Pos)             /*!< 0x00000080 */
#define GPIO_INENR_INEN7            GPIO_INENR_INEN7_Msk
#define GPIO_INENR_INEN8_Pos        (8U)
#define GPIO_INENR_INEN8_Msk        (0x1UL << GPIO_INENR_INEN8_Pos)             /*!< 0x00000100 */
#define GPIO_INENR_INEN8            GPIO_INENR_INEN8_Msk
#define GPIO_INENR_INEN9_Pos        (9U)
#define GPIO_INENR_INEN9_Msk        (0x1UL << GPIO_INENR_INEN9_Pos)             /*!< 0x00000200 */
#define GPIO_INENR_INEN9            GPIO_INENR_INEN9_Msk
#define GPIO_INENR_INEN10_Pos       (10U)
#define GPIO_INENR_INEN10_Msk       (0x1UL << GPIO_INENR_INEN10_Pos)            /*!< 0x00000400 */
#define GPIO_INENR_INEN10           GPIO_INENR_INEN10_Msk
#define GPIO_INENR_INEN11_Pos       (11U)
#define GPIO_INENR_INEN11_Msk       (0x1UL << GPIO_INENR_INEN11_Pos)            /*!< 0x00000800 */
#define GPIO_INENR_INEN11           GPIO_INENR_INEN11_Msk
#define GPIO_INENR_INEN12_Pos       (12U)
#define GPIO_INENR_INEN12_Msk       (0x1UL << GPIO_INENR_INEN12_Pos)            /*!< 0x00001000 */
#define GPIO_INENR_INEN12           GPIO_INENR_INEN12_Msk
#define GPIO_INENR_INEN13_Pos       (13U)
#define GPIO_INENR_INEN13_Msk       (0x1UL << GPIO_INENR_INEN13_Pos)            /*!< 0x00002000 */
#define GPIO_INENR_INEN13           GPIO_INENR_INEN13_Msk
#define GPIO_INENR_INEN14_Pos       (14U)
#define GPIO_INENR_INEN14_Msk       (0x1UL << GPIO_INENR_INEN14_Pos)            /*!< 0x00004000 */
#define GPIO_INENR_INEN14           GPIO_INENR_INEN14_Msk
#define GPIO_INENR_INEN15_Pos       (15U)
#define GPIO_INENR_INEN15_Msk       (0x1UL << GPIO_INENR_INEN15_Pos)            /*!< 0x00008000 */
#define GPIO_INENR_INEN15           GPIO_INENR_INEN15_Msk
/* ============================================  Bit definition for GPIO_PUPDR rePDgister  ============================================= */
#define GPIO_PUPDR_PUEN0_Pos        (0U)                                        /*!< / */
#define GPIO_PUPDR_PUEN0_Msk        (0x1UL << GPIO_PUPDR_PUEN0_Pos)             /*!< 0x00000001 */
#define GPIO_PUPDR_PUEN0            GPIO_PUPDR_PUEN0_Msk
#define GPIO_PUPDR_PUS0_Pos         (1U)                                        /*!< / */
#define GPIO_PUPDR_PUS0_Msk         (0x1UL << GPIO_PUPDR_PUS0_Pos)              /*!< 0x00000002 */
#define GPIO_PUPDR_PUS0             GPIO_PUPDR_PUS0_Msk
#define GPIO_PUPDR_PUEN1_Pos        (2U)                                        /*!< / */
#define GPIO_PUPDR_PUEN1_Msk        (0x1UL << GPIO_PUPDR_PUEN1_Pos)             /*!< 0x00000002 */
#define GPIO_PUPDR_PUEN1            GPIO_PUPDR_PUEN1_Msk
#define GPIO_PUPDR_PUS1_Pos         (3U)                                        /*!< / */
#define GPIO_PUPDR_PUS1_Msk         (0x1UL << GPIO_PUPDR_PUS1_Pos)              /*!< 0x00000004 */
#define GPIO_PUPDR_PUS1             GPIO_PUPDR_PUS1_Msk
#define GPIO_PUPDR_PUEN2_Pos        (4U)                                        /*!< / */
#define GPIO_PUPDR_PUEN2_Msk        (0x1UL << GPIO_PUPDR_PUEN2_Pos)             /*!< 0x00000004 */
#define GPIO_PUPDR_PUEN2            GPIO_PUPDR_PUEN2_Msk
#define GPIO_PUPDR_PUS2_Pos         (5U)                                        /*!< / */
#define GPIO_PUPDR_PUS2_Msk         (0x1UL << GPIO_PUPDR_PUS2_Pos)              /*!< 0x00000008 */
#define GPIO_PUPDR_PUS2             GPIO_PUPDR_PUS2_Msk
#define GPIO_PUPDR_PUEN3_Pos        (6U)                                        /*!< / */
#define GPIO_PUPDR_PUEN3_Msk        (0x1UL << GPIO_PUPDR_PUEN3_Pos)             /*!< 0x00000008 */
#define GPIO_PUPDR_PUEN3            GPIO_PUPDR_PUEN3_Msk
#define GPIO_PUPDR_PUS3_Pos         (7U)                                        /*!< / */
#define GPIO_PUPDR_PUS3_Msk         (0x1UL << GPIO_PUPDR_PUS3_Pos)              /*!< 0x00000010 */
#define GPIO_PUPDR_PUS3             GPIO_PUPDR_PUS3_Msk
#define GPIO_PUPDR_PUEN4_Pos        (8U)                                        /*!< / */
#define GPIO_PUPDR_PUEN4_Msk        (0x1UL << GPIO_PUPDR_PUEN4_Pos)             /*!< 0x00000010 */
#define GPIO_PUPDR_PUEN4            GPIO_PUPDR_PUEN4_Msk
#define GPIO_PUPDR_PUS4_Pos         (9U)                                        /*!< / */
#define GPIO_PUPDR_PUS4_Msk         (0x1UL << GPIO_PUPDR_PUS4_Pos)              /*!< 0x00000020 */
#define GPIO_PUPDR_PUS4             GPIO_PUPDR_PUS4_Msk
#define GPIO_PUPDR_PUEN5_Pos        (10U)                                       /*!< / */
#define GPIO_PUPDR_PUEN5_Msk        (0x1UL << GPIO_PUPDR_PUEN5_Pos)             /*!< 0x00000020 */
#define GPIO_PUPDR_PUEN5            GPIO_PUPDR_PUEN5_Msk
#define GPIO_PUPDR_PUS5_Pos         (11U)                                       /*!< / */
#define GPIO_PUPDR_PUS5_Msk         (0x1UL << GPIO_PUPDR_PUS5_Pos)              /*!< 0x00000040 */
#define GPIO_PUPDR_PUS5             GPIO_PUPDR_PUS5_Msk
#define GPIO_PUPDR_PUEN6_Pos        (12U)                                       /*!< / */
#define GPIO_PUPDR_PUEN6_Msk        (0x1UL << GPIO_PUPDR_PUEN6_Pos)             /*!< 0x00000040 */
#define GPIO_PUPDR_PUEN6            GPIO_PUPDR_PUEN6_Msk
#define GPIO_PUPDR_PUS6_Pos         (13U)                                       /*!< / */
#define GPIO_PUPDR_PUS6_Msk         (0x1UL << GPIO_PUPDR_PUS6_Pos)              /*!< 0x00000080 */
#define GPIO_PUPDR_PUS6             GPIO_PUPDR_PUS6_Msk
#define GPIO_PUPDR_PUEN7_Pos        (14U)                                       /*!< / */
#define GPIO_PUPDR_PUEN7_Msk        (0x1UL << GPIO_PUPDR_PUEN7_Pos)             /*!< 0x00000080 */
#define GPIO_PUPDR_PUEN7            GPIO_PUPDR_PUEN7_Msk
#define GPIO_PUPDR_PUS7_Pos         (15U)                                       /*!< / */
#define GPIO_PUPDR_PUS7_Msk         (0x1UL << GPIO_PUPDR_PUS7_Pos)              /*!< 0x00000100 */
#define GPIO_PUPDR_PUS7             GPIO_PUPDR_PUS7_Msk
#define GPIO_PUPDR_PUEN8_Pos        (16U)                                       /*!< / */
#define GPIO_PUPDR_PUEN8_Msk        (0x1UL << GPIO_PUPDR_PUEN8_Pos)             /*!< 0x00000100 */
#define GPIO_PUPDR_PUEN8            GPIO_PUPDR_PUEN8_Msk
#define GPIO_PUPDR_PUS8_Pos         (17U)                                       /*!< / */
#define GPIO_PUPDR_PUS8_Msk         (0x1UL << GPIO_PUPDR_PUS8_Pos)              /*!< 0x00000200 */
#define GPIO_PUPDR_PUS8             GPIO_PUPDR_PUS8_Msk
#define GPIO_PUPDR_PUEN9_Pos        (18U)                                       /*!< / */
#define GPIO_PUPDR_PUEN9_Msk        (0x1UL << GPIO_PUPDR_PUEN9_Pos)             /*!< 0x00000200 */
#define GPIO_PUPDR_PUEN9            GPIO_PUPDR_PUEN9_Msk
#define GPIO_PUPDR_PUS9_Pos         (19U)                                       /*!< / */
#define GPIO_PUPDR_PUS9_Msk         (0x1UL << GPIO_PUPDR_PUS9_Pos)              /*!< 0x00000400 */
#define GPIO_PUPDR_PUS9             GPIO_PUPDR_PUS9_Msk
#define GPIO_PUPDR_PUEN10_Pos       (20U)                                       /*!< / */
#define GPIO_PUPDR_PUEN10_Msk       (0x1UL << GPIO_PUPDR_PUEN10_Pos)            /*!< 0x00000400 */
#define GPIO_PUPDR_PUEN10           GPIO_PUPDR_PUEN10_Msk
#define GPIO_PUPDR_PUS10_Pos        (21U)                                       /*!< / */
#define GPIO_PUPDR_PUS10_Msk        (0x1UL << GPIO_PUPDR_PUS10_Pos)             /*!< 0x00000800 */
#define GPIO_PUPDR_PUS10            GPIO_PUPDR_PUS10_Msk
#define GPIO_PUPDR_PUEN11_Pos       (22U)                                       /*!< / */
#define GPIO_PUPDR_PUEN11_Msk       (0x1UL << GPIO_PUPDR_PUEN11_Pos)            /*!< 0x00000800 */
#define GPIO_PUPDR_PUEN11           GPIO_PUPDR_PUEN11_Msk
#define GPIO_PUPDR_PUS11_Pos        (23U)                                       /*!< / */
#define GPIO_PUPDR_PUS11_Msk        (0x1UL << GPIO_PUPDR_PUS11_Pos)             /*!< 0x00001000 */
#define GPIO_PUPDR_PUS11            GPIO_PUPDR_PUS11_Msk
#define GPIO_PUPDR_PUEN12_Pos       (24U)                                       /*!< / */
#define GPIO_PUPDR_PUEN12_Msk       (0x1UL << GPIO_PUPDR_PUEN12_Pos)            /*!< 0x00001000 */
#define GPIO_PUPDR_PUEN12           GPIO_PUPDR_PUEN12_Msk
#define GPIO_PUPDR_PUS12_Pos        (25U)                                       /*!< / */
#define GPIO_PUPDR_PUS12_Msk        (0x1UL << GPIO_PUPDR_PUS12_Pos)             /*!< 0x00002000 */
#define GPIO_PUPDR_PUS12            GPIO_PUPDR_PUS12_Msk
#define GPIO_PUPDR_PUEN13_Pos       (26U)                                       /*!< / */
#define GPIO_PUPDR_PUEN13_Msk       (0x1UL << GPIO_PUPDR_PUEN13_Pos)            /*!< 0x00002000 */
#define GPIO_PUPDR_PUEN13           GPIO_PUPDR_PUEN13_Msk
#define GPIO_PUPDR_PUS13_Pos        (27U)                                       /*!< / */
#define GPIO_PUPDR_PUS13_Msk        (0x1UL << GPIO_PUPDR_PUS13_Pos)             /*!< 0x00004000 */
#define GPIO_PUPDR_PUS13            GPIO_PUPDR_PUS13_Msk
#define GPIO_PUPDR_PUEN14_Pos       (28U)                                       /*!< / */
#define GPIO_PUPDR_PUEN14_Msk       (0x1UL << GPIO_PUPDR_PUEN14_Pos)            /*!< 0x00004000 */
#define GPIO_PUPDR_PUEN14           GPIO_PUPDR_PUEN14_Msk
#define GPIO_PUPDR_PUS14_Pos        (29U)                                       /*!< / */
#define GPIO_PUPDR_PUS14_Msk        (0x1UL << GPIO_PUPDR_PUS14_Pos)             /*!< 0x00008000 */
#define GPIO_PUPDR_PUS14            GPIO_PUPDR_PUS14_Msk
#define GPIO_PUPDR_PUEN15_Pos       (30U)                                       /*!< / */
#define GPIO_PUPDR_PUEN15_Msk       (0x1UL << GPIO_PUPDR_PUEN15_Pos)            /*!< 0x00008000 */
#define GPIO_PUPDR_PUEN15           GPIO_PUPDR_PUEN15_Msk
#define GPIO_PUPDR_PUS15_Pos        (31U)                                       /*!< / */
#define GPIO_PUPDR_PUS15_Msk        (0x1UL << GPIO_PUPDR_PUS15_Pos)             /*!< 0x00010000 */
#define GPIO_PUPDR_PUS15            GPIO_PUPDR_PUS15_Msk

/* =============================================  Bit definition for GPIO_OTR register  ============================================= */
#define GPIO_OTR_OT0_Pos            (0U)
#define GPIO_OTR_OT0_Msk            (0x1UL << GPIO_OTR_OT0_Pos)                 /*!< 0x00000001 */
#define GPIO_OTR_OT0                GPIO_OTR_OT0_Msk
#define GPIO_OTR_OT1_Pos            (1U)
#define GPIO_OTR_OT1_Msk            (0x1UL << GPIO_OTR_OT1_Pos)                 /*!< 0x00000002 */
#define GPIO_OTR_OT1                GPIO_OTR_OT1_Msk
#define GPIO_OTR_OT2_Pos            (2U)
#define GPIO_OTR_OT2_Msk            (0x1UL << GPIO_OTR_OT2_Pos)                 /*!< 0x00000004 */
#define GPIO_OTR_OT2                GPIO_OTR_OT2_Msk
#define GPIO_OTR_OT3_Pos            (3U)
#define GPIO_OTR_OT3_Msk            (0x1UL << GPIO_OTR_OT3_Pos)                 /*!< 0x00000008 */
#define GPIO_OTR_OT3                GPIO_OTR_OT3_Msk
#define GPIO_OTR_OT4_Pos            (4U)
#define GPIO_OTR_OT4_Msk            (0x1UL << GPIO_OTR_OT4_Pos)                 /*!< 0x00000010 */
#define GPIO_OTR_OT4                GPIO_OTR_OT4_Msk
#define GPIO_OTR_OT5_Pos            (5U)
#define GPIO_OTR_OT5_Msk            (0x1UL << GPIO_OTR_OT5_Pos)                 /*!< 0x00000020 */
#define GPIO_OTR_OT5                GPIO_OTR_OT5_Msk
#define GPIO_OTR_OT6_Pos            (6U)
#define GPIO_OTR_OT6_Msk            (0x1UL << GPIO_OTR_OT6_Pos)                 /*!< 0x00000040 */
#define GPIO_OTR_OT6                GPIO_OTR_OT6_Msk
#define GPIO_OTR_OT7_Pos            (7U)
#define GPIO_OTR_OT7_Msk            (0x1UL << GPIO_OTR_OT7_Pos)                 /*!< 0x00000080 */
#define GPIO_OTR_OT7                GPIO_OTR_OT7_Msk
#define GPIO_OTR_OT8_Pos            (8U)
#define GPIO_OTR_OT8_Msk            (0x1UL << GPIO_OTR_OT8_Pos)                 /*!< 0x00000100 */
#define GPIO_OTR_OT8                GPIO_OTR_OT8_Msk
#define GPIO_OTR_OT9_Pos            (9U)
#define GPIO_OTR_OT9_Msk            (0x1UL << GPIO_OTR_OT9_Pos)                 /*!< 0x00000200 */
#define GPIO_OTR_OT9                GPIO_OTR_OT9_Msk
#define GPIO_OTR_OT10_Pos           (10U)
#define GPIO_OTR_OT10_Msk           (0x1UL << GPIO_OTR_OT10_Pos)                /*!< 0x00000400 */
#define GPIO_OTR_OT10               GPIO_OTR_OT10_Msk
#define GPIO_OTR_OT11_Pos           (11U)
#define GPIO_OTR_OT11_Msk           (0x1UL << GPIO_OTR_OT11_Pos)                /*!< 0x00000800 */
#define GPIO_OTR_OT11               GPIO_OTR_OT11_Msk
#define GPIO_OTR_OT12_Pos           (12U)
#define GPIO_OTR_OT12_Msk           (0x1UL << GPIO_OTR_OT12_Pos)                /*!< 0x00001000 */
#define GPIO_OTR_OT12               GPIO_OTR_OT12_Msk
#define GPIO_OTR_OT13_Pos           (13U)
#define GPIO_OTR_OT13_Msk           (0x1UL << GPIO_OTR_OT13_Pos)                /*!< 0x00002000 */
#define GPIO_OTR_OT13               GPIO_OTR_OT13_Msk
#define GPIO_OTR_OT14_Pos           (14U)
#define GPIO_OTR_OT14_Msk           (0x1UL << GPIO_OTR_OT14_Pos)                /*!< 0x00004000 */
#define GPIO_OTR_OT14               GPIO_OTR_OT14_Msk
#define GPIO_OTR_OT15_Pos           (15U)
#define GPIO_OTR_OT15_Msk           (0x1UL << GPIO_OTR_OT15_Pos)                /*!< 0x00008000 */
#define GPIO_OTR_OT15               GPIO_OTR_OT15_Msk

/* Legacy defines */
#define GPIO_OTR_OT_0               GPIO_OTR_OT0
#define GPIO_OTR_OT_1               GPIO_OTR_OT1
#define GPIO_OTR_OT_2               GPIO_OTR_OT2
#define GPIO_OTR_OT_3               GPIO_OTR_OT3
#define GPIO_OTR_OT_4               GPIO_OTR_OT4
#define GPIO_OTR_OT_5               GPIO_OTR_OT5
#define GPIO_OTR_OT_6               GPIO_OTR_OT6
#define GPIO_OTR_OT_7               GPIO_OTR_OT7
#define GPIO_OTR_OT_8               GPIO_OTR_OT8
#define GPIO_OTR_OT_9               GPIO_OTR_OT9
#define GPIO_OTR_OT_10              GPIO_OTR_OT10
#define GPIO_OTR_OT_11              GPIO_OTR_OT11
#define GPIO_OTR_OT_12              GPIO_OTR_OT12
#define GPIO_OTR_OT_13              GPIO_OTR_OT13
#define GPIO_OTR_OT_14              GPIO_OTR_OT14
#define GPIO_OTR_OT_15              GPIO_OTR_OT15
/* =============================================  Bit definition for GPIO_DSR register  ============================================= */
#define GPIO_DSR_DS0_Pos            (0U)
#define GPIO_DSR_DS0_Msk            (0x1UL << GPIO_DSR_DS0_Pos)                 /*!< 0x00000001 */
#define GPIO_DSR_DS0                GPIO_DSR_DS0_Msk
#define GPIO_DSR_DS1_Pos            (1U)
#define GPIO_DSR_DS1_Msk            (0x1UL << GPIO_DSR_DS1_Pos)                 /*!< 0x00000002 */
#define GPIO_DSR_DS1                GPIO_DSR_DS1_Msk
#define GPIO_DSR_DS2_Pos            (2U)
#define GPIO_DSR_DS2_Msk            (0x1UL << GPIO_DSR_DS2_Pos)                 /*!< 0x00000004 */
#define GPIO_DSR_DS2                GPIO_DSR_DS2_Msk
#define GPIO_DSR_DS3_Pos            (3U)
#define GPIO_DSR_DS3_Msk            (0x1UL << GPIO_DSR_DS3_Pos)                 /*!< 0x00000008 */
#define GPIO_DSR_DS3                GPIO_DSR_DS3_Msk
#define GPIO_DSR_DS4_Pos            (4U)
#define GPIO_DSR_DS4_Msk            (0x1UL << GPIO_DSR_DS4_Pos)                 /*!< 0x00000010 */
#define GPIO_DSR_DS4                GPIO_DSR_DS4_Msk
#define GPIO_DSR_DS5_Pos            (5U)
#define GPIO_DSR_DS5_Msk            (0x1UL << GPIO_DSR_DS5_Pos)                 /*!< 0x00000020 */
#define GPIO_DSR_DS5                GPIO_DSR_DS5_Msk
#define GPIO_DSR_DS6_Pos            (6U)
#define GPIO_DSR_DS6_Msk            (0x1UL << GPIO_DSR_DS6_Pos)                 /*!< 0x00000040 */
#define GPIO_DSR_DS6                GPIO_DSR_DS6_Msk
#define GPIO_DSR_DS7_Pos            (7U)
#define GPIO_DSR_DS7_Msk            (0x1UL << GPIO_DSR_DS7_Pos)                 /*!< 0x00000080 */
#define GPIO_DSR_DS7                GPIO_DSR_DS7_Msk
#define GPIO_DSR_DS8_Pos            (8U)
#define GPIO_DSR_DS8_Msk            (0x1UL << GPIO_DSR_DS8_Pos)                 /*!< 0x00000100 */
#define GPIO_DSR_DS8                GPIO_DSR_DS8_Msk
#define GPIO_DSR_DS9_Pos            (9U)
#define GPIO_DSR_DS9_Msk            (0x1UL << GPIO_DSR_DS9_Pos)                 /*!< 0x00000200 */
#define GPIO_DSR_DS9                GPIO_DSR_DS9_Msk
#define GPIO_DSR_DS10_Pos           (10U)
#define GPIO_DSR_DS10_Msk           (0x1UL << GPIO_DSR_DS10_Pos)                /*!< 0x00000400 */
#define GPIO_DSR_DS10               GPIO_DSR_DS10_Msk
#define GPIO_DSR_DS11_Pos           (11U)
#define GPIO_DSR_DS11_Msk           (0x1UL << GPIO_DSR_DS11_Pos)                /*!< 0x00000800 */
#define GPIO_DSR_DS11               GPIO_DSR_DS11_Msk
#define GPIO_DSR_DS12_Pos           (12U)
#define GPIO_DSR_DS12_Msk           (0x1UL << GPIO_DSR_DS12_Pos)                /*!< 0x00001000 */
#define GPIO_DSR_DS12               GPIO_DSR_DS12_Msk
#define GPIO_DSR_DS13_Pos           (13U)
#define GPIO_DSR_DS13_Msk           (0x1UL << GPIO_DSR_DS13_Pos)                /*!< 0x00002000 */
#define GPIO_DSR_DS13               GPIO_DSR_DS13_Msk
#define GPIO_DSR_DS14_Pos           (14U)
#define GPIO_DSR_DS14_Msk           (0x1UL << GPIO_DSR_DS14_Pos)                /*!< 0x00004000 */
#define GPIO_DSR_DS14               GPIO_DSR_DS14_Msk
#define GPIO_DSR_DS15_Pos           (15U)
#define GPIO_DSR_DS15_Msk           (0x1UL << GPIO_DSR_DS15_Pos)                /*!< 0x00008000 */
#define GPIO_DSR_DS15               GPIO_DSR_DS15_Msk
/* ============================================  Bit definition for GPIO_INDR register  ============================================= */
#define GPIO_INDR_IND0_Pos          (0U)
#define GPIO_INDR_IND0_Msk          (0x1UL << GPIO_INDR_IND0_Pos)               /*!< 0x00000001 */
#define GPIO_INDR_IND0              GPIO_INDR_IND0_Msk
#define GPIO_INDR_IND1_Pos          (1U)
#define GPIO_INDR_IND1_Msk          (0x1UL << GPIO_INDR_IND1_Pos)               /*!< 0x00000002 */
#define GPIO_INDR_IND1              GPIO_INDR_IND1_Msk
#define GPIO_INDR_IND2_Pos          (2U)
#define GPIO_INDR_IND2_Msk          (0x1UL << GPIO_INDR_IND2_Pos)               /*!< 0x00000004 */
#define GPIO_INDR_IND2              GPIO_INDR_IND2_Msk
#define GPIO_INDR_IND3_Pos          (3U)
#define GPIO_INDR_IND3_Msk          (0x1UL << GPIO_INDR_IND3_Pos)               /*!< 0x00000008 */
#define GPIO_INDR_IND3              GPIO_INDR_IND3_Msk
#define GPIO_INDR_IND4_Pos          (4U)
#define GPIO_INDR_IND4_Msk          (0x1UL << GPIO_INDR_IND4_Pos)               /*!< 0x00000010 */
#define GPIO_INDR_IND4              GPIO_INDR_IND4_Msk
#define GPIO_INDR_IND5_Pos          (5U)
#define GPIO_INDR_IND5_Msk          (0x1UL << GPIO_INDR_IND5_Pos)               /*!< 0x00000020 */
#define GPIO_INDR_IND5              GPIO_INDR_IND5_Msk
#define GPIO_INDR_IND6_Pos          (6U)
#define GPIO_INDR_IND6_Msk          (0x1UL << GPIO_INDR_IND6_Pos)               /*!< 0x00000040 */
#define GPIO_INDR_IND6              GPIO_INDR_IND6_Msk
#define GPIO_INDR_IND7_Pos          (7U)
#define GPIO_INDR_IND7_Msk          (0x1UL << GPIO_INDR_IND7_Pos)               /*!< 0x00000080 */
#define GPIO_INDR_IND7              GPIO_INDR_IND7_Msk
#define GPIO_INDR_IND8_Pos          (8U)
#define GPIO_INDR_IND8_Msk          (0x1UL << GPIO_INDR_IND8_Pos)               /*!< 0x00000100 */
#define GPIO_INDR_IND8              GPIO_INDR_IND8_Msk
#define GPIO_INDR_IND9_Pos          (9U)
#define GPIO_INDR_IND9_Msk          (0x1UL << GPIO_INDR_IND9_Pos)               /*!< 0x00000200 */
#define GPIO_INDR_IND9              GPIO_INDR_IND9_Msk
#define GPIO_INDR_IND10_Pos         (10U)
#define GPIO_INDR_IND10_Msk         (0x1UL << GPIO_INDR_IND10_Pos)              /*!< 0x00000400 */
#define GPIO_INDR_IND10             GPIO_INDR_IND10_Msk
#define GPIO_INDR_IND11_Pos         (11U)
#define GPIO_INDR_IND11_Msk         (0x1UL << GPIO_INDR_IND11_Pos)              /*!< 0x00000800 */
#define GPIO_INDR_IND11             GPIO_INDR_IND11_Msk
#define GPIO_INDR_IND12_Pos         (12U)
#define GPIO_INDR_IND12_Msk         (0x1UL << GPIO_INDR_IND12_Pos)              /*!< 0x00001000 */
#define GPIO_INDR_IND12             GPIO_INDR_IND12_Msk
#define GPIO_INDR_IND13_Pos         (13U)
#define GPIO_INDR_IND13_Msk         (0x1UL << GPIO_INDR_IND13_Pos)              /*!< 0x00002000 */
#define GPIO_INDR_IND13             GPIO_INDR_IND13_Msk
#define GPIO_INDR_IND14_Pos         (14U)
#define GPIO_INDR_IND14_Msk         (0x1UL << GPIO_INDR_IND14_Pos)              /*!< 0x00004000 */
#define GPIO_INDR_IND14             GPIO_INDR_IND14_Msk
#define GPIO_INDR_IND15_Pos         (15U)
#define GPIO_INDR_IND15_Msk         (0x1UL << GPIO_INDR_IND15_Pos)              /*!< 0x00008000 */
#define GPIO_INDR_IND15             GPIO_INDR_IND15_Msk

/* Legacy defines */
#define GPIO_IDATA_IDR_0            GPIO_INDR_IND0
#define GPIO_IDATA_IDR_1            GPIO_INDR_IND1
#define GPIO_IDATA_IDR_2            GPIO_INDR_IND2
#define GPIO_IDATA_IDR_3            GPIO_INDR_IND3
#define GPIO_IDATA_IDR_4            GPIO_INDR_IND4
#define GPIO_IDATA_IDR_5            GPIO_INDR_IND5
#define GPIO_IDATA_IDR_6            GPIO_INDR_IND6
#define GPIO_IDATA_IDR_7            GPIO_INDR_IND7
#define GPIO_IDATA_IDR_8            GPIO_INDR_IND8
#define GPIO_IDATA_IDR_9            GPIO_INDR_IND9
#define GPIO_IDATA_IDR_10           GPIO_INDR_IND10
#define GPIO_IDATA_IDR_11           GPIO_INDR_IND11
#define GPIO_IDATA_IDR_12           GPIO_INDR_IND12
#define GPIO_IDATA_IDR_13           GPIO_INDR_IND13
#define GPIO_IDATA_IDR_14           GPIO_INDR_IND14
#define GPIO_IDATA_IDR_15           GPIO_INDR_IND15
/* ============================================  Bit definition for GPIO_DOUTR register  ============================================ */
#define GPIO_OUTDR_OUTD0_Pos        (0U)
#define GPIO_OUTDR_OUTD0_Msk        (0x1UL << GPIO_OUTDR_OUTD0_Pos)             /*!< 0x00000001 */
#define GPIO_OUTDR_OUTD0            GPIO_OUTDR_OUTD0_Msk
#define GPIO_OUTDR_OUTD1_Pos        (1U)
#define GPIO_OUTDR_OUTD1_Msk        (0x1UL << GPIO_OUTDR_OUTD1_Pos)             /*!< 0x00000002 */
#define GPIO_OUTDR_OUTD1            GPIO_OUTDR_OUTD1_Msk
#define GPIO_OUTDR_OUTD2_Pos        (2U)
#define GPIO_OUTDR_OUTD2_Msk        (0x1UL << GPIO_OUTDR_OUTD2_Pos)             /*!< 0x00000004 */
#define GPIO_OUTDR_OUTD2            GPIO_OUTDR_OUTD2_Msk
#define GPIO_OUTDR_OUTD3_Pos        (3U)
#define GPIO_OUTDR_OUTD3_Msk        (0x1UL << GPIO_OUTDR_OUTD3_Pos)             /*!< 0x00000008 */
#define GPIO_OUTDR_OUTD3            GPIO_OUTDR_OUTD3_Msk
#define GPIO_OUTDR_OUTD4_Pos        (4U)
#define GPIO_OUTDR_OUTD4_Msk        (0x1UL << GPIO_OUTDR_OUTD4_Pos)             /*!< 0x00000010 */
#define GPIO_OUTDR_OUTD4            GPIO_OUTDR_OUTD4_Msk
#define GPIO_OUTDR_OUTD5_Pos        (5U)
#define GPIO_OUTDR_OUTD5_Msk        (0x1UL << GPIO_OUTDR_OUTD5_Pos)             /*!< 0x00000020 */
#define GPIO_OUTDR_OUTD5            GPIO_OUTDR_OUTD5_Msk
#define GPIO_OUTDR_OUTD6_Pos        (6U)
#define GPIO_OUTDR_OUTD6_Msk        (0x1UL << GPIO_OUTDR_OUTD6_Pos)             /*!< 0x00000040 */
#define GPIO_OUTDR_OUTD6            GPIO_OUTDR_OUTD6_Msk
#define GPIO_OUTDR_OUTD7_Pos        (7U)
#define GPIO_OUTDR_OUTD7_Msk        (0x1UL << GPIO_OUTDR_OUTD7_Pos)             /*!< 0x00000080 */
#define GPIO_OUTDR_OUTD7            GPIO_OUTDR_OUTD7_Msk
#define GPIO_OUTDR_OUTD8_Pos        (8U)
#define GPIO_OUTDR_OUTD8_Msk        (0x1UL << GPIO_OUTDR_OUTD8_Pos)             /*!< 0x00000100 */
#define GPIO_OUTDR_OUTD8            GPIO_OUTDR_OUTD8_Msk
#define GPIO_OUTDR_OUTD9_Pos        (9U)
#define GPIO_OUTDR_OUTD9_Msk        (0x1UL << GPIO_OUTDR_OUTD9_Pos)             /*!< 0x00000200 */
#define GPIO_OUTDR_OUTD9            GPIO_OUTDR_OUTD9_Msk
#define GPIO_OUTDR_OUTD10_Pos       (10U)
#define GPIO_OUTDR_OUTD10_Msk       (0x1UL << GPIO_OUTDR_OUTD10_Pos)            /*!< 0x00000400 */
#define GPIO_OUTDR_OUTD10           GPIO_OUTDR_OUTD10_Msk
#define GPIO_OUTDR_OUTD11_Pos       (11U)
#define GPIO_OUTDR_OUTD11_Msk       (0x1UL << GPIO_OUTDR_OUTD11_Pos)            /*!< 0x00000800 */
#define GPIO_OUTDR_OUTD11           GPIO_OUTDR_OUTD11_Msk
#define GPIO_OUTDR_OUTD12_Pos       (12U)
#define GPIO_OUTDR_OUTD12_Msk       (0x1UL << GPIO_OUTDR_OUTD12_Pos)            /*!< 0x00001000 */
#define GPIO_OUTDR_OUTD12           GPIO_OUTDR_OUTD12_Msk
#define GPIO_OUTDR_OUTD13_Pos       (13U)
#define GPIO_OUTDR_OUTD13_Msk       (0x1UL << GPIO_OUTDR_OUTD13_Pos)            /*!< 0x00002000 */
#define GPIO_OUTDR_OUTD13           GPIO_OUTDR_OUTD13_Msk
#define GPIO_OUTDR_OUTD14_Pos       (12U)
#define GPIO_OUTDR_OUTD14_Msk       (0x1UL << GPIO_OUTDR_OUTD14_Pos)            /*!< 0x00004000 */
#define GPIO_OUTDR_OUTD14           GPIO_OUTDR_OUTD14_Msk
#define GPIO_OUTDR_OUTD15_Pos       (13U)
#define GPIO_OUTDR_OUTD15_Msk       (0x1UL << GPIO_OUTDR_OUTD15_Pos)            /*!< 0x00008000 */
#define GPIO_OUTDR_OUTD15           GPIO_OUTDR_OUTD15_Msk

/* Legacy defines */
#define GPIO_ODATA_ODR_0            GPIO_OUTDR_OUTD0
#define GPIO_ODATA_ODR_1            GPIO_OUTDR_OUTD1
#define GPIO_ODATA_ODR_2            GPIO_OUTDR_OUTD2
#define GPIO_ODATA_ODR_3            GPIO_OUTDR_OUTD3
#define GPIO_ODATA_ODR_4            GPIO_OUTDR_OUTD4
#define GPIO_ODATA_ODR_5            GPIO_OUTDR_OUTD5
#define GPIO_ODATA_ODR_6            GPIO_OUTDR_OUTD6
#define GPIO_ODATA_ODR_7            GPIO_OUTDR_OUTD7
#define GPIO_ODATA_ODR_8            GPIO_OUTDR_OUTD8
#define GPIO_ODATA_ODR_9            GPIO_OUTDR_OUTD9
#define GPIO_ODATA_ODR_10           GPIO_OUTDR_OUTD10
#define GPIO_ODATA_ODR_11           GPIO_OUTDR_OUTD11
#define GPIO_ODATA_ODR_12           GPIO_OUTDR_OUTD12
#define GPIO_ODATA_ODR_13           GPIO_OUTDR_OUTD13
#define GPIO_ODATA_ODR_14           GPIO_OUTDR_OUTD14
#define GPIO_ODATA_ODR_15           GPIO_OUTDR_OUTD15
/* ============================================  Bit definition for GPIO_BSRR register  ============================================= */
#define GPIO_BSRR_BS0_Pos           (0U)
#define GPIO_BSRR_BS0_Msk           (0x1UL << GPIO_BSRR_BS0_Pos)                /*!< 0x00000001 */
#define GPIO_BSRR_BS0               GPIO_BSRR_BS0_Msk
#define GPIO_BSRR_BS1_Pos           (1U)
#define GPIO_BSRR_BS1_Msk           (0x1UL << GPIO_BSRR_BS1_Pos)                /*!< 0x00000002 */
#define GPIO_BSRR_BS1               GPIO_BSRR_BS1_Msk
#define GPIO_BSRR_BS2_Pos           (2U)
#define GPIO_BSRR_BS2_Msk           (0x1UL << GPIO_BSRR_BS2_Pos)                /*!< 0x00000004 */
#define GPIO_BSRR_BS2               GPIO_BSRR_BS2_Msk
#define GPIO_BSRR_BS3_Pos           (3U)
#define GPIO_BSRR_BS3_Msk           (0x1UL << GPIO_BSRR_BS3_Pos)                /*!< 0x00000008 */
#define GPIO_BSRR_BS3               GPIO_BSRR_BS3_Msk
#define GPIO_BSRR_BS4_Pos           (4U)
#define GPIO_BSRR_BS4_Msk           (0x1UL << GPIO_BSRR_BS4_Pos)                /*!< 0x00000010 */
#define GPIO_BSRR_BS4               GPIO_BSRR_BS4_Msk
#define GPIO_BSRR_BS5_Pos           (5U)
#define GPIO_BSRR_BS5_Msk           (0x1UL << GPIO_BSRR_BS5_Pos)                /*!< 0x00000020 */
#define GPIO_BSRR_BS5               GPIO_BSRR_BS5_Msk
#define GPIO_BSRR_BS6_Pos           (6U)
#define GPIO_BSRR_BS6_Msk           (0x1UL << GPIO_BSRR_BS6_Pos)                /*!< 0x00000040 */
#define GPIO_BSRR_BS6               GPIO_BSRR_BS6_Msk
#define GPIO_BSRR_BS7_Pos           (7U)
#define GPIO_BSRR_BS7_Msk           (0x1UL << GPIO_BSRR_BS7_Pos)                /*!< 0x00000080 */
#define GPIO_BSRR_BS7               GPIO_BSRR_BS7_Msk
#define GPIO_BSRR_BS8_Pos           (8U)
#define GPIO_BSRR_BS8_Msk           (0x1UL << GPIO_BSRR_BS8_Pos)                /*!< 0x00000100 */
#define GPIO_BSRR_BS8               GPIO_BSRR_BS8_Msk
#define GPIO_BSRR_BS9_Pos           (9U)
#define GPIO_BSRR_BS9_Msk           (0x1UL << GPIO_BSRR_BS9_Pos)                /*!< 0x00000200 */
#define GPIO_BSRR_BS9               GPIO_BSRR_BS9_Msk
#define GPIO_BSRR_BS10_Pos          (10U)
#define GPIO_BSRR_BS10_Msk          (0x1UL << GPIO_BSRR_BS10_Pos)               /*!< 0x00000400 */
#define GPIO_BSRR_BS10              GPIO_BSRR_BS10_Msk
#define GPIO_BSRR_BS11_Pos          (11U)
#define GPIO_BSRR_BS11_Msk          (0x1UL << GPIO_BSRR_BS11_Pos)               /*!< 0x00000800 */
#define GPIO_BSRR_BS11              GPIO_BSRR_BS11_Msk
#define GPIO_BSRR_BS12_Pos          (12U)
#define GPIO_BSRR_BS12_Msk          (0x1UL << GPIO_BSRR_BS12_Pos)               /*!< 0x00001000 */
#define GPIO_BSRR_BS12              GPIO_BSRR_BS12_Msk
#define GPIO_BSRR_BS13_Pos          (13U)
#define GPIO_BSRR_BS13_Msk          (0x1UL << GPIO_BSRR_BS13_Pos)               /*!< 0x00002000 */
#define GPIO_BSRR_BS13              GPIO_BSRR_BS13_Msk
#define GPIO_BSRR_BS14_Pos          (14U)
#define GPIO_BSRR_BS14_Msk          (0x1UL << GPIO_BSRR_BS14_Pos)               /*!< 0x00004000 */
#define GPIO_BSRR_BS14              GPIO_BSRR_BS14_Msk
#define GPIO_BSRR_BS15_Pos          (15U)
#define GPIO_BSRR_BS15_Msk          (0x1UL << GPIO_BSRR_BS15_Pos)               /*!< 0x00008000 */
#define GPIO_BSRR_BS15              GPIO_BSRR_BS15_Msk
#define GPIO_BSRR_BR0_Pos           (16U)
#define GPIO_BSRR_BR0_Msk           (0x1UL << GPIO_BSRR_BR0_Pos)                /*!< 0x00010000 */
#define GPIO_BSRR_BR0               GPIO_BSRR_BR0_Msk
#define GPIO_BSRR_BR1_Pos           (17U)
#define GPIO_BSRR_BR1_Msk           (0x1UL << GPIO_BSRR_BR1_Pos)                /*!< 0x00020000 */
#define GPIO_BSRR_BR1               GPIO_BSRR_BR1_Msk
#define GPIO_BSRR_BR2_Pos           (18U)
#define GPIO_BSRR_BR2_Msk           (0x1UL << GPIO_BSRR_BR2_Pos)                /*!< 0x00040000 */
#define GPIO_BSRR_BR2               GPIO_BSRR_BR2_Msk
#define GPIO_BSRR_BR3_Pos           (19U)
#define GPIO_BSRR_BR3_Msk           (0x1UL << GPIO_BSRR_BR3_Pos)                /*!< 0x00080000 */
#define GPIO_BSRR_BR3               GPIO_BSRR_BR3_Msk
#define GPIO_BSRR_BR4_Pos           (20U)
#define GPIO_BSRR_BR4_Msk           (0x1UL << GPIO_BSRR_BR4_Pos)                /*!< 0x00100000 */
#define GPIO_BSRR_BR4               GPIO_BSRR_BR4_Msk
#define GPIO_BSRR_BR5_Pos           (21U)
#define GPIO_BSRR_BR5_Msk           (0x1UL << GPIO_BSRR_BR5_Pos)                /*!< 0x00200000 */
#define GPIO_BSRR_BR5               GPIO_BSRR_BR5_Msk
#define GPIO_BSRR_BR6_Pos           (22U)
#define GPIO_BSRR_BR6_Msk           (0x1UL << GPIO_BSRR_BR6_Pos)                /*!< 0x00400000 */
#define GPIO_BSRR_BR6               GPIO_BSRR_BR6_Msk
#define GPIO_BSRR_BR7_Pos           (23U)
#define GPIO_BSRR_BR7_Msk           (0x1UL << GPIO_BSRR_BR7_Pos)                /*!< 0x00800000 */
#define GPIO_BSRR_BR7               GPIO_BSRR_BR7_Msk
#define GPIO_BSRR_BR8_Pos           (24U)
#define GPIO_BSRR_BR8_Msk           (0x1UL << GPIO_BSRR_BR8_Pos)                /*!< 0x01000000 */
#define GPIO_BSRR_BR8               GPIO_BSRR_BR8_Msk
#define GPIO_BSRR_BR9_Pos           (25U)
#define GPIO_BSRR_BR9_Msk           (0x1UL << GPIO_BSRR_BR9_Pos)                /*!< 0x02000000 */
#define GPIO_BSRR_BR9               GPIO_BSRR_BR9_Msk
#define GPIO_BSRR_BR10_Pos          (26U)
#define GPIO_BSRR_BR10_Msk          (0x1UL << GPIO_BSRR_BR10_Pos)               /*!< 0x04000000 */
#define GPIO_BSRR_BR10              GPIO_BSRR_BR10_Msk
#define GPIO_BSRR_BR11_Pos          (27U)
#define GPIO_BSRR_BR11_Msk          (0x1UL << GPIO_BSRR_BR11_Pos)               /*!< 0x08000000 */
#define GPIO_BSRR_BR11              GPIO_BSRR_BR11_Msk
#define GPIO_BSRR_BR12_Pos          (28U)
#define GPIO_BSRR_BR12_Msk          (0x1UL << GPIO_BSRR_BR12_Pos)               /*!< 0x10000000 */
#define GPIO_BSRR_BR12              GPIO_BSRR_BR12_Msk
#define GPIO_BSRR_BR13_Pos          (29U)
#define GPIO_BSRR_BR13_Msk          (0x1UL << GPIO_BSRR_BR13_Pos)               /*!< 0x20000000 */
#define GPIO_BSRR_BR13              GPIO_BSRR_BR13_Msk
#define GPIO_BSRR_BR14_Pos          (30U)
#define GPIO_BSRR_BR14_Msk          (0x1UL << GPIO_BSRR_BR14_Pos)               /*!< 0x40000000 */
#define GPIO_BSRR_BR14              GPIO_BSRR_BR14_Msk
#define GPIO_BSRR_BR15_Pos          (31U)
#define GPIO_BSRR_BR15_Msk          (0x1UL << GPIO_BSRR_BR15_Pos)               /*!< 0x80000000 */
#define GPIO_BSRR_BR15              GPIO_BSRR_BR15_Msk

/* Legacy defines */
#define GPIO_BSRR_BS_0              GPIO_BSRR_BS0
#define GPIO_BSRR_BS_1              GPIO_BSRR_BS1
#define GPIO_BSRR_BS_2              GPIO_BSRR_BS2
#define GPIO_BSRR_BS_3              GPIO_BSRR_BS3
#define GPIO_BSRR_BS_4              GPIO_BSRR_BS4
#define GPIO_BSRR_BS_5              GPIO_BSRR_BS5
#define GPIO_BSRR_BS_6              GPIO_BSRR_BS6
#define GPIO_BSRR_BS_7              GPIO_BSRR_BS7
#define GPIO_BSRR_BS_8              GPIO_BSRR_BS8
#define GPIO_BSRR_BS_9              GPIO_BSRR_BS9
#define GPIO_BSRR_BS_10             GPIO_BSRR_BS10
#define GPIO_BSRR_BS_11             GPIO_BSRR_BS11
#define GPIO_BSRR_BS_12             GPIO_BSRR_BS12
#define GPIO_BSRR_BS_13             GPIO_BSRR_BS13
#define GPIO_BSRR_BS_14             GPIO_BSRR_BS14
#define GPIO_BSRR_BS_15             GPIO_BSRR_BS15
#define GPIO_BSRR_BR_0              GPIO_BSRR_BR0
#define GPIO_BSRR_BR_1              GPIO_BSRR_BR1
#define GPIO_BSRR_BR_2              GPIO_BSRR_BR2
#define GPIO_BSRR_BR_3              GPIO_BSRR_BR3
#define GPIO_BSRR_BR_4              GPIO_BSRR_BR4
#define GPIO_BSRR_BR_5              GPIO_BSRR_BR5
#define GPIO_BSRR_BR_6              GPIO_BSRR_BR6
#define GPIO_BSRR_BR_7              GPIO_BSRR_BR7
#define GPIO_BSRR_BR_8              GPIO_BSRR_BR8
#define GPIO_BSRR_BR_9              GPIO_BSRR_BR9
#define GPIO_BSRR_BR_10             GPIO_BSRR_BR10
#define GPIO_BSRR_BR_11             GPIO_BSRR_BR11
#define GPIO_BSRR_BR_12             GPIO_BSRR_BR12
#define GPIO_BSRR_BR_13             GPIO_BSRR_BR13
#define GPIO_BSRR_BR_14             GPIO_BSRR_BR14
#define GPIO_BSRR_BR_15             GPIO_BSRR_BR15

/* =============================================  Bit definition for GPIO_BRR register  ============================================= */
#define GPIO_BRR_BRR0_Pos           (0U)
#define GPIO_BRR_BRR0_Msk           (0x1UL << GPIO_BRR_BRR0_Pos)                /*!< 0x00000001 */
#define GPIO_BRR_BRR0               GPIO_BRR_BRR0_Msk
#define GPIO_BRR_BRR1_Pos           (1U)
#define GPIO_BRR_BRR1_Msk           (0x1UL << GPIO_BRR_BRR1_Pos)                /*!< 0x00000002 */
#define GPIO_BRR_BRR1               GPIO_BRR_BRR1_Msk
#define GPIO_BRR_BRR2_Pos           (2U)
#define GPIO_BRR_BRR2_Msk           (0x1UL << GPIO_BRR_BRR2_Pos)                /*!< 0x00000004 */
#define GPIO_BRR_BRR2               GPIO_BRR_BRR2_Msk
#define GPIO_BRR_BRR3_Pos           (3U)
#define GPIO_BRR_BRR3_Msk           (0x1UL << GPIO_BRR_BRR3_Pos)                /*!< 0x00000008 */
#define GPIO_BRR_BRR3               GPIO_BRR_BRR3_Msk
#define GPIO_BRR_BRR4_Pos           (4U)
#define GPIO_BRR_BRR4_Msk           (0x1UL << GPIO_BRR_BRR4_Pos)                /*!< 0x00000010 */
#define GPIO_BRR_BRR4               GPIO_BRR_BRR4_Msk
#define GPIO_BRR_BRR5_Pos           (5U)
#define GPIO_BRR_BRR5_Msk           (0x1UL << GPIO_BRR_BRR5_Pos)                /*!< 0x00000020 */
#define GPIO_BRR_BRR5               GPIO_BRR_BRR5_Msk
#define GPIO_BRR_BRR6_Pos           (6U)
#define GPIO_BRR_BRR6_Msk           (0x1UL << GPIO_BRR_BRR6_Pos)                /*!< 0x00000040 */
#define GPIO_BRR_BRR6               GPIO_BRR_BRR6_Msk
#define GPIO_BRR_BRR7_Pos           (7U)
#define GPIO_BRR_BRR7_Msk           (0x1UL << GPIO_BRR_BRR7_Pos)                /*!< 0x00000080 */
#define GPIO_BRR_BRR7               GPIO_BRR_BRR7_Msk
#define GPIO_BRR_BRR8_Pos           (8U)
#define GPIO_BRR_BRR8_Msk           (0x1UL << GPIO_BRR_BRR8_Pos)                /*!< 0x00000100 */
#define GPIO_BRR_BRR8               GPIO_BRR_BRR8_Msk
#define GPIO_BRR_BRR9_Pos           (9U)
#define GPIO_BRR_BRR9_Msk           (0x1UL << GPIO_BRR_BRR9_Pos)                /*!< 0x00000200 */
#define GPIO_BRR_BRR9               GPIO_BRR_BRR9_Msk
#define GPIO_BRR_BRR10_Pos          (10U)
#define GPIO_BRR_BRR10_Msk          (0x1UL << GPIO_BRR_BRR10_Pos)               /*!< 0x00000400 */
#define GPIO_BRR_BRR10              GPIO_BRR_BRR10_Msk
#define GPIO_BRR_BRR11_Pos          (11U)
#define GPIO_BRR_BRR11_Msk          (0x1UL << GPIO_BRR_BRR11_Pos)               /*!< 0x00000800 */
#define GPIO_BRR_BRR11              GPIO_BRR_BRR11_Msk
#define GPIO_BRR_BRR12_Pos          (12U)
#define GPIO_BRR_BRR12_Msk          (0x1UL << GPIO_BRR_BRR12_Pos)               /*!< 0x00001000 */
#define GPIO_BRR_BRR12              GPIO_BRR_BRR12_Msk
#define GPIO_BRR_BRR13_Pos          (13U)
#define GPIO_BRR_BRR13_Msk          (0x1UL << GPIO_BRR_BRR13_Pos)               /*!< 0x00002000 */
#define GPIO_BRR_BRR13              GPIO_BRR_BRR13_Msk
#define GPIO_BRR_BRR14_Pos          (14U)
#define GPIO_BRR_BRR14_Msk          (0x1UL << GPIO_BRR_BRR14_Pos)               /*!< 0x00001000 */
#define GPIO_BRR_BRR14              GPIO_BRR_BRR14_Msk
#define GPIO_BRR_BRR15_Pos          (15U)
#define GPIO_BRR_BRR15_Msk          (0x1UL << GPIO_BRR_BRR15_Pos)               /*!< 0x00002000 */
#define GPIO_BRR_BRR15              GPIO_BRR_BRR15_Msk

/* Legacy defines */
#define GPIO_BRR_BRR_0              GPIO_BRR_BRR0
#define GPIO_BRR_BRR_1              GPIO_BRR_BRR1
#define GPIO_BRR_BRR_2              GPIO_BRR_BRR2
#define GPIO_BRR_BRR_3              GPIO_BRR_BRR3
#define GPIO_BRR_BRR_4              GPIO_BRR_BRR4
#define GPIO_BRR_BRR_5              GPIO_BRR_BRR5
#define GPIO_BRR_BRR_6              GPIO_BRR_BRR6
#define GPIO_BRR_BRR_7              GPIO_BRR_BRR7
#define GPIO_BRR_BRR_8              GPIO_BRR_BRR8
#define GPIO_BRR_BRR_9              GPIO_BRR_BRR9
#define GPIO_BRR_BRR_10             GPIO_BRR_BRR10
#define GPIO_BRR_BRR_11             GPIO_BRR_BRR11
#define GPIO_BRR_BRR_12             GPIO_BRR_BRR12
#define GPIO_BRR_BRR_13             GPIO_BRR_BRR13
#define GPIO_BRR_BRR_14             GPIO_BRR_BRR14
#define GPIO_BRR_BRR_15             GPIO_BRR_BRR15

/* ============================================  Bit definition for GPIO_LOCK register  ============================================= */
#define GPIO_LOCK_KEY_Pos           (0U)
#define GPIO_LOCK_KEY_Msk           (0xFFFFFFFFUL << GPIO_LOCK_KEY_Pos)         /*!< 0x00000001 */
#define GPIO_LOCK_KEY               GPIO_LOCK_KEY_Msk

/* ===========================================  Bit definition for GPIO_AFSELR0 register  =========================================== */
#define GPIO_AFSELR0_AFSEL0_Pos     (0U)
#define GPIO_AFSELR0_AFSEL0_Msk     (0x7UL << GPIO_AFSELR0_AFSEL0_Pos)          /*!< 0x0000000F */
#define GPIO_AFSELR0_AFSEL0         GPIO_AFSELR0_AFSEL0_Msk
#define GPIO_AFSELR0_AFSEL0_0       (0x1UL << GPIO_AFSELR0_AFSEL0_Pos)          /*!< 0x00000001 */
#define GPIO_AFSELR0_AFSEL0_1       (0x2UL << GPIO_AFSELR0_AFSEL0_Pos)          /*!< 0x00000002 */
#define GPIO_AFSELR0_AFSEL0_2       (0x4UL << GPIO_AFSELR0_AFSEL0_Pos)          /*!< 0x00000004 */
#define GPIO_AFSELR0_AFSEL1_Pos     (4U)
#define GPIO_AFSELR0_AFSEL1_Msk     (0x7UL << GPIO_AFSELR0_AFSEL1_Pos)          /*!< 0x000000F0 */
#define GPIO_AFSELR0_AFSEL1         GPIO_AFSELR0_AFSEL1_Msk
#define GPIO_AFSELR0_AFSEL1_0       (0x1UL << GPIO_AFSELR0_AFSEL1_Pos)          /*!< 0x00000010 */
#define GPIO_AFSELR0_AFSEL1_1       (0x2UL << GPIO_AFSELR0_AFSEL1_Pos)          /*!< 0x00000020 */
#define GPIO_AFSELR0_AFSEL1_2       (0x4UL << GPIO_AFSELR0_AFSEL1_Pos)          /*!< 0x00000040 */
#define GPIO_AFSELR0_AFSEL2_Pos     (8U)
#define GPIO_AFSELR0_AFSEL2_Msk     (0x7UL << GPIO_AFSELR0_AFSEL2_Pos)          /*!< 0x00000F00 */
#define GPIO_AFSELR0_AFSEL2         GPIO_AFSELR0_AFSEL2_Msk
#define GPIO_AFSELR0_AFSEL2_0       (0x1UL << GPIO_AFSELR0_AFSEL2_Pos)          /*!< 0x00000100 */
#define GPIO_AFSELR0_AFSEL2_1       (0x2UL << GPIO_AFSELR0_AFSEL2_Pos)          /*!< 0x00000200 */
#define GPIO_AFSELR0_AFSEL2_2       (0x4UL << GPIO_AFSELR0_AFSEL2_Pos)          /*!< 0x00000400 */
#define GPIO_AFSELR0_AFSEL3_Pos     (12U)
#define GPIO_AFSELR0_AFSEL3_Msk     (0x7UL << GPIO_AFSELR0_AFSEL3_Pos)          /*!< 0x0000F000 */
#define GPIO_AFSELR0_AFSEL3         GPIO_AFSELR0_AFSEL3_Msk
#define GPIO_AFSELR0_AFSEL3_0       (0x1UL << GPIO_AFSELR0_AFSEL3_Pos)          /*!< 0x00001000 */
#define GPIO_AFSELR0_AFSEL3_1       (0x2UL << GPIO_AFSELR0_AFSEL3_Pos)          /*!< 0x00002000 */
#define GPIO_AFSELR0_AFSEL3_2       (0x4UL << GPIO_AFSELR0_AFSEL3_Pos)          /*!< 0x00004000 */
#define GPIO_AFSELR0_AFSEL4_Pos     (16U)
#define GPIO_AFSELR0_AFSEL4_Msk     (0x7UL << GPIO_AFSELR0_AFSEL4_Pos)          /*!< 0x000F0000 */
#define GPIO_AFSELR0_AFSEL4         GPIO_AFSELR0_AFSEL4_Msk
#define GPIO_AFSELR0_AFSEL4_0       (0x1UL << GPIO_AFSELR0_AFSEL4_Pos)          /*!< 0x00010000 */
#define GPIO_AFSELR0_AFSEL4_1       (0x2UL << GPIO_AFSELR0_AFSEL4_Pos)          /*!< 0x00020000 */
#define GPIO_AFSELR0_AFSEL4_2       (0x4UL << GPIO_AFSELR0_AFSEL4_Pos)          /*!< 0x00040000 */
#define GPIO_AFSELR0_AFSEL5_Pos     (20U)
#define GPIO_AFSELR0_AFSEL5_Msk     (0x7UL << GPIO_AFSELR0_AFSEL5_Pos)          /*!< 0x00F00000 */
#define GPIO_AFSELR0_AFSEL5         GPIO_AFSELR0_AFSEL5_Msk
#define GPIO_AFSELR0_AFSEL5_0       (0x1UL << GPIO_AFSELR0_AFSEL5_Pos)          /*!< 0x00100000 */
#define GPIO_AFSELR0_AFSEL5_1       (0x2UL << GPIO_AFSELR0_AFSEL5_Pos)          /*!< 0x00200000 */
#define GPIO_AFSELR0_AFSEL5_2       (0x4UL << GPIO_AFSELR0_AFSEL5_Pos)          /*!< 0x00400000 */
#define GPIO_AFSELR0_AFSEL6_Pos     (24U)
#define GPIO_AFSELR0_AFSEL6_Msk     (0x7UL << GPIO_AFSELR0_AFSEL6_Pos)          /*!< 0x0F000000 */
#define GPIO_AFSELR0_AFSEL6         GPIO_AFSELR0_AFSEL6_Msk
#define GPIO_AFSELR0_AFSEL6_0       (0x1UL << GPIO_AFSELR0_AFSEL6_Pos)          /*!< 0x01000000 */
#define GPIO_AFSELR0_AFSEL6_1       (0x2UL << GPIO_AFSELR0_AFSEL6_Pos)          /*!< 0x02000000 */
#define GPIO_AFSELR0_AFSEL6_2       (0x4UL << GPIO_AFSELR0_AFSEL6_Pos)          /*!< 0x04000000 */
#define GPIO_AFSELR0_AFSEL7_Pos     (28U)
#define GPIO_AFSELR0_AFSEL7_Msk     (0x7UL << GPIO_AFSELR0_AFSEL7_Pos)          /*!< 0xF0000000 */
#define GPIO_AFSELR0_AFSEL7         GPIO_AFSELR0_AFSEL7_Msk
#define GPIO_AFSELR0_AFSEL7_0       (0x1UL << GPIO_AFSELR0_AFSEL7_Pos)          /*!< 0x10000000 */
#define GPIO_AFSELR0_AFSEL7_1       (0x2UL << GPIO_AFSELR0_AFSEL7_Pos)          /*!< 0x20000000 */
#define GPIO_AFSELR0_AFSEL7_2       (0x4UL << GPIO_AFSELR0_AFSEL7_Pos)          /*!< 0x40000000 */

/* Legacy defines */
#define GPIO_ALFL_AFRL0             GPIO_AFSELR0_AFSEL0
#define GPIO_ALFL_AFRL0_0           GPIO_AFSELR0_AFSEL0_0
#define GPIO_ALFL_AFRL0_1           GPIO_AFSELR0_AFSEL0_1
#define GPIO_ALFL_AFRL0_2           GPIO_AFSELR0_AFSEL0_2
#define GPIO_ALFL_AFRL1             GPIO_AFSELR0_AFSEL1
#define GPIO_ALFL_AFRL1_0           GPIO_AFSELR0_AFSEL1_0
#define GPIO_ALFL_AFRL1_1           GPIO_AFSELR0_AFSEL1_1
#define GPIO_ALFL_AFRL1_2           GPIO_AFSELR0_AFSEL1_2
#define GPIO_ALFL_AFRL2             GPIO_AFSELR0_AFSEL2
#define GPIO_ALFL_AFRL2_0           GPIO_AFSELR0_AFSEL2_0
#define GPIO_ALFL_AFRL2_1           GPIO_AFSELR0_AFSEL2_1
#define GPIO_ALFL_AFRL2_2           GPIO_AFSELR0_AFSEL2_2
#define GPIO_ALFL_AFRL3             GPIO_AFSELR0_AFSEL3
#define GPIO_ALFL_AFRL3_0           GPIO_AFSELR0_AFSEL3_0
#define GPIO_ALFL_AFRL3_1           GPIO_AFSELR0_AFSEL3_1
#define GPIO_ALFL_AFRL3_2           GPIO_AFSELR0_AFSEL3_2
#define GPIO_ALFL_AFRL4             GPIO_AFSELR0_AFSEL4
#define GPIO_ALFL_AFRL4_0           GPIO_AFSELR0_AFSEL4_0
#define GPIO_ALFL_AFRL4_1           GPIO_AFSELR0_AFSEL4_1
#define GPIO_ALFL_AFRL4_2           GPIO_AFSELR0_AFSEL4_2
#define GPIO_ALFL_AFRL5             GPIO_AFSELR0_AFSEL5
#define GPIO_ALFL_AFRL5_0           GPIO_AFSELR0_AFSEL5_0
#define GPIO_ALFL_AFRL5_1           GPIO_AFSELR0_AFSEL5_1
#define GPIO_ALFL_AFRL5_2           GPIO_AFSELR0_AFSEL5_2
#define GPIO_ALFL_AFRL6             GPIO_AFSELR0_AFSEL6
#define GPIO_ALFL_AFRL6_0           GPIO_AFSELR0_AFSEL6_0
#define GPIO_ALFL_AFRL6_1           GPIO_AFSELR0_AFSEL6_1
#define GPIO_ALFL_AFRL6_2           GPIO_AFSELR0_AFSEL6_2
#define GPIO_ALFL_AFRL7             GPIO_AFSELR0_AFSEL7
#define GPIO_ALFL_AFRL7_0           GPIO_AFSELR0_AFSEL7_0
#define GPIO_ALFL_AFRL7_1           GPIO_AFSELR0_AFSEL7_1
#define GPIO_ALFL_AFRL7_2           GPIO_AFSELR0_AFSEL7_2

/* ===========================================  Bit definition for GPIO_AFSELR1 register  =========================================== */
#define GPIO_AFSELR1_AFSEL8_Pos     (0U)
#define GPIO_AFSELR1_AFSEL8_Msk     (0x7UL << GPIO_AFSELR1_AFSEL8_Pos)          /*!< 0x0000000F */
#define GPIO_AFSELR1_AFSEL8         GPIO_AFSELR1_AFSEL8_Msk
#define GPIO_AFSELR1_AFSEL8_0       (0x1UL << GPIO_AFSELR1_AFSEL8_Pos)          /*!< 0x00000001 */
#define GPIO_AFSELR1_AFSEL8_1       (0x2UL << GPIO_AFSELR1_AFSEL8_Pos)          /*!< 0x00000002 */
#define GPIO_AFSELR1_AFSEL8_2       (0x4UL << GPIO_AFSELR1_AFSEL8_Pos)          /*!< 0x00000004 */
#define GPIO_AFSELR1_AFSEL9_Pos     (4U)
#define GPIO_AFSELR1_AFSEL9_Msk     (0x7UL << GPIO_AFSELR1_AFSEL9_Pos)          /*!< 0x000000F0 */
#define GPIO_AFSELR1_AFSEL9         GPIO_AFSELR1_AFSEL9_Msk
#define GPIO_AFSELR1_AFSEL9_0       (0x1UL << GPIO_AFSELR1_AFSEL9_Pos)          /*!< 0x00000010 */
#define GPIO_AFSELR1_AFSEL9_1       (0x2UL << GPIO_AFSELR1_AFSEL9_Pos)          /*!< 0x00000020 */
#define GPIO_AFSELR1_AFSEL9_2       (0x4UL << GPIO_AFSELR1_AFSEL9_Pos)          /*!< 0x00000040 */
#define GPIO_AFSELR1_AFSEL10_Pos    (8U)
#define GPIO_AFSELR1_AFSEL10_Msk    (0x7UL << GPIO_AFSELR1_AFSEL10_Pos)         /*!< 0x00000F00 */
#define GPIO_AFSELR1_AFSEL10        GPIO_AFSELR1_AFSEL10_Msk
#define GPIO_AFSELR1_AFSEL10_0      (0x1UL << GPIO_AFSELR1_AFSEL10_Pos)         /*!< 0x00000100 */
#define GPIO_AFSELR1_AFSEL10_1      (0x2UL << GPIO_AFSELR1_AFSEL10_Pos)         /*!< 0x00000200 */
#define GPIO_AFSELR1_AFSEL10_2      (0x4UL << GPIO_AFSELR1_AFSEL10_Pos)         /*!< 0x00000400 */
#define GPIO_AFSELR1_AFSEL11_Pos    (12U)
#define GPIO_AFSELR1_AFSEL11_Msk    (0x7UL << GPIO_AFSELR1_AFSEL11_Pos)         /*!< 0x0000F000 */
#define GPIO_AFSELR1_AFSEL11        GPIO_AFSELR1_AFSEL11_Msk
#define GPIO_AFSELR1_AFSEL11_0      (0x1UL << GPIO_AFSELR1_AFSEL11_Pos)         /*!< 0x00001000 */
#define GPIO_AFSELR1_AFSEL11_1      (0x2UL << GPIO_AFSELR1_AFSEL11_Pos)         /*!< 0x00002000 */
#define GPIO_AFSELR1_AFSEL11_2      (0x4UL << GPIO_AFSELR1_AFSEL11_Pos)         /*!< 0x00004000 */
#define GPIO_AFSELR1_AFSEL12_Pos    (16U)
#define GPIO_AFSELR1_AFSEL12_Msk    (0x7UL << GPIO_AFSELR1_AFSEL12_Pos)         /*!< 0x000F0000 */
#define GPIO_AFSELR1_AFSEL12        GPIO_AFSELR1_AFSEL12_Msk
#define GPIO_AFSELR1_AFSEL12_0      (0x1UL << GPIO_AFSELR1_AFSEL12_Pos)         /*!< 0x00010000 */
#define GPIO_AFSELR1_AFSEL12_1      (0x2UL << GPIO_AFSELR1_AFSEL12_Pos)         /*!< 0x00020000 */
#define GPIO_AFSELR1_AFSEL12_2      (0x4UL << GPIO_AFSELR1_AFSEL12_Pos)         /*!< 0x00040000 */
#define GPIO_AFSELR1_AFSEL13_Pos    (20U)
#define GPIO_AFSELR1_AFSEL13_Msk    (0x7UL << GPIO_AFSELR1_AFSEL13_Pos)         /*!< 0x00F00000 */
#define GPIO_AFSELR1_AFSEL13        GPIO_AFSELR1_AFSEL13_Msk
#define GPIO_AFSELR1_AFSEL13_0      (0x1UL << GPIO_AFSELR1_AFSEL13_Pos)         /*!< 0x00100000 */
#define GPIO_AFSELR1_AFSEL13_1      (0x2UL << GPIO_AFSELR1_AFSEL13_Pos)         /*!< 0x00200000 */
#define GPIO_AFSELR1_AFSEL13_2      (0x4UL << GPIO_AFSELR1_AFSEL13_Pos)         /*!< 0x00400000 */
#define GPIO_AFSELR1_AFSEL14_Pos    (24U)
#define GPIO_AFSELR1_AFSEL14_Msk    (0x7UL << GPIO_AFSELR1_AFSEL14_Pos)         /*!< 0x0F000000 */
#define GPIO_AFSELR1_AFSEL14        GPIO_AFSELR1_AFSEL14_Msk
#define GPIO_AFSELR1_AFSEL14_0      (0x1UL << GPIO_AFSELR1_AFSEL14_Pos)         /*!< 0x01000000 */
#define GPIO_AFSELR1_AFSEL14_1      (0x2UL << GPIO_AFSELR1_AFSEL14_Pos)         /*!< 0x02000000 */
#define GPIO_AFSELR1_AFSEL14_2      (0x4UL << GPIO_AFSELR1_AFSEL14_Pos)         /*!< 0x04000000 */
#define GPIO_AFSELR1_AFSEL15_Pos    (28U)
#define GPIO_AFSELR1_AFSEL15_Msk    (0x7UL << GPIO_AFSELR1_AFSEL15_Pos)         /*!< 0xF0000000 */
#define GPIO_AFSELR1_AFSEL15        GPIO_AFSELR1_AFSEL15_Msk
#define GPIO_AFSELR1_AFSEL15_0      (0x1UL << GPIO_AFSELR1_AFSEL15_Pos)         /*!< 0x10000000 */
#define GPIO_AFSELR1_AFSEL15_1      (0x2UL << GPIO_AFSELR1_AFSEL15_Pos)         /*!< 0x20000000 */
#define GPIO_AFSELR1_AFSEL15_2      (0x4UL << GPIO_AFSELR1_AFSEL15_Pos)         /*!< 0x40000000 */

/* Legacy defines */
#define GPIO_ALFH_AFRH0             GPIO_AFSELR1_AFSEL8
#define GPIO_ALFH_AFRH0_0           GPIO_AFSELR1_AFSEL8_0
#define GPIO_ALFH_AFRH0_1           GPIO_AFSELR1_AFSEL8_1
#define GPIO_ALFH_AFRH0_2           GPIO_AFSELR1_AFSEL8_2
#define GPIO_ALFH_AFRH1             GPIO_AFSELR1_AFSEL9
#define GPIO_ALFH_AFRH1_0           GPIO_AFSELR1_AFSEL9_0
#define GPIO_ALFH_AFRH1_1           GPIO_AFSELR1_AFSEL9_1
#define GPIO_ALFH_AFRH1_2           GPIO_AFSELR1_AFSEL9_2
#define GPIO_ALFH_AFRH2             GPIO_AFSELR1_AFSEL10
#define GPIO_ALFH_AFRH2_0           GPIO_AFSELR1_AFSEL10_0
#define GPIO_ALFH_AFRH2_1           GPIO_AFSELR1_AFSEL10_1
#define GPIO_ALFH_AFRH2_2           GPIO_AFSELR1_AFSEL10_2
#define GPIO_ALFH_AFRH3             GPIO_AFSELR1_AFSEL11
#define GPIO_ALFH_AFRH3_0           GPIO_AFSELR1_AFSEL11_0
#define GPIO_ALFH_AFRH3_1           GPIO_AFSELR1_AFSEL11_1
#define GPIO_ALFH_AFRH3_2           GPIO_AFSELR1_AFSEL11_2
#define GPIO_ALFH_AFRH4             GPIO_AFSELR1_AFSEL12
#define GPIO_ALFH_AFRH4_0           GPIO_AFSELR1_AFSEL12_0
#define GPIO_ALFH_AFRH4_1           GPIO_AFSELR1_AFSEL12_1
#define GPIO_ALFH_AFRH4_2           GPIO_AFSELR1_AFSEL12_2
#define GPIO_ALFH_AFRH5             GPIO_AFSELR1_AFSEL13
#define GPIO_ALFH_AFRH5_0           GPIO_AFSELR1_AFSEL13_0
#define GPIO_ALFH_AFRH5_1           GPIO_AFSELR1_AFSEL13_1
#define GPIO_ALFH_AFRH5_2           GPIO_AFSELR1_AFSEL13_2
#define GPIO_ALFH_AFRH6             GPIO_AFSELR1_AFSEL14
#define GPIO_ALFH_AFRH6_0           GPIO_AFSELR1_AFSEL14_0
#define GPIO_ALFH_AFRH6_1           GPIO_AFSELR1_AFSEL14_1
#define GPIO_ALFH_AFRH6_2           GPIO_AFSELR1_AFSEL14_2
#define GPIO_ALFH_AFRH7             GPIO_AFSELR1_AFSEL15
#define GPIO_ALFH_AFRH7_0           GPIO_AFSELR1_AFSEL15_0
#define GPIO_ALFH_AFRH7_1           GPIO_AFSELR1_AFSEL15_1
#define GPIO_ALFH_AFRH7_2           GPIO_AFSELR1_AFSEL15_2

/* ==========================================  Bit definition for GPIO_AF3RMP register  =========================================== */
#define GPIO_AF3RMP_AFRMP10_Pos     (8U)                                        /*!< / */
#define GPIO_AF3RMP_AFRMP10_0       (0x1UL << GPIO_AF3RMP_AFRMP10_Pos)          /*!< 0x00000100 */
#define GPIO_AF3RMP_AFRMP10_1       (0x2UL << GPIO_AF3RMP_AFRMP10_Pos)          /*!< 0x00000200 */
#define GPIO_AF3RMP_AFRMP10_2       (0x4UL << GPIO_AF3RMP_AFRMP10_Pos)          /*!< 0x00000400 */
#define GPIO_AF3RMP_AFRMP10_Msk     (0x7UL << GPIO_AF3RMP_AFRMP10_Pos)          /*!< 0x00000700 */
#define GPIO_AF3RMP_AFRMP10         GPIO_AF3RMP_AFRMP10_Msk
#define GPIO_AF3RMP_AFRMP11_Pos     (12U)                                       /*!< / */
#define GPIO_AF3RMP_AFRMP11_0       (0x1UL << GPIO_AF3RMP_AFRMP11_Pos)          /*!< 0x00001000 */
#define GPIO_AF3RMP_AFRMP11_1       (0x2UL << GPIO_AF3RMP_AFRMP11_Pos)          /*!< 0x00002000 */
#define GPIO_AF3RMP_AFRMP11_2       (0x4UL << GPIO_AF3RMP_AFRMP11_Pos)          /*!< 0x00004000 */
#define GPIO_AF3RMP_AFRMP11_Msk     (0x7UL << GPIO_AF3RMP_AFRMP11_Pos)          /*!< 0x00007000 */
#define GPIO_AF3RMP_AFRMP11         GPIO_AF3RMP_AFRMP11_Msk
#define GPIO_AF3RMP_AFRMP12_Pos     (16U)                                       /*!< / */
#define GPIO_AF3RMP_AFRMP12_0       (0x1UL << GPIO_AF3RMP_AFRMP12_Pos)          /*!< 0x00010000 */
#define GPIO_AF3RMP_AFRMP12_1       (0x2UL << GPIO_AF3RMP_AFRMP12_Pos)          /*!< 0x00020000 */
#define GPIO_AF3RMP_AFRMP12_2       (0x4UL << GPIO_AF3RMP_AFRMP12_Pos)          /*!< 0x00040000 */
#define GPIO_AF3RMP_AFRMP12_Msk     (0x7UL << GPIO_AF3RMP_AFRMP12_Pos)          /*!< 0x00070000 */
#define GPIO_AF3RMP_AFRMP12         GPIO_AF3RMP_AFRMP12_Msk
#define GPIO_AF3RMP_AFRMP13_Pos     (20U)                                       /*!< / */
#define GPIO_AF3RMP_AFRMP13_0       (0x1UL << GPIO_AF3RMP_AFRMP13_Pos)          /*!< 0x00100000 */
#define GPIO_AF3RMP_AFRMP13_1       (0x2UL << GPIO_AF3RMP_AFRMP13_Pos)          /*!< 0x00200000 */
#define GPIO_AF3RMP_AFRMP13_2       (0x4UL << GPIO_AF3RMP_AFRMP13_Pos)          /*!< 0x00400000 */
#define GPIO_AF3RMP_AFRMP13_Msk     (0x7UL << GPIO_AF3RMP_AFRMP13_Pos)          /*!< 0x00700000 */
#define GPIO_AF3RMP_AFRMP13         GPIO_AF3RMP_AFRMP13_Msk
#define GPIO_AF3RMP_AFRMP14_Pos     (24U)                                       /*!< / */
#define GPIO_AF3RMP_AFRMP14_0       (0x1UL << GPIO_AF3RMP_AFRMP14_Pos)          /*!< 0x01000000 */
#define GPIO_AF3RMP_AFRMP14_1       (0x2UL << GPIO_AF3RMP_AFRMP14_Pos)          /*!< 0x02000000 */
#define GPIO_AF3RMP_AFRMP14_2       (0x4UL << GPIO_AF3RMP_AFRMP14_Pos)          /*!< 0x04000000 */
#define GPIO_AF3RMP_AFRMP14_Msk     (0x7UL << GPIO_AF3RMP_AFRMP14_Pos)          /*!< 0x07000000 */
#define GPIO_AF3RMP_AFRMP14         GPIO_AF3RMP_AFRMP14_Msk
#define GPIO_AF3RMP_AFRMP15_Pos     (28U)                                       /*!< / */
#define GPIO_AF3RMP_AFRMP15_0       (0x1UL << GPIO_AF3RMP_AFRMP15_Pos)          /*!< 0x10000000 */
#define GPIO_AF3RMP_AFRMP15_1       (0x2UL << GPIO_AF3RMP_AFRMP15_Pos)          /*!< 0x20000000 */
#define GPIO_AF3RMP_AFRMP15_2       (0x4UL << GPIO_AF3RMP_AFRMP15_Pos)          /*!< 0x40000000 */
#define GPIO_AF3RMP_AFRMP15_Msk     (0x7UL << GPIO_AF3RMP_AFRMP15_Pos)          /*!< 0x70000000 */
#define GPIO_AF3RMP_AFRMP15         GPIO_AF3RMP_AFRMP15_Msk

/* ================================================================================================================================== */
/* =============================================================  OPA  ============================================================== */
/* ================================================================================================================================== */
/* ==============================================  Bit definition for OPA_CR register  ============================================== */
#define OPA_CR_OPA0EN_Pos           (0U)                                        /*!< OPA0_EN */
#define OPA_CR_OPA0EN_Msk           (0x1UL << OPA_CR_OPA0EN_Pos)                /*!< 0x00000001 */
#define OPA_CR_OPA0EN               OPA_CR_OPA0EN_Msk
#define OPA_CR_OPA0SELGAIN_Pos      (1U)                                        /*!< OPA0_SEL_GAIN */
#define OPA_CR_OPA0SELGAIN_0        (0x1UL << OPA_CR_OPA0SELGAIN_Pos)           /*!< 0x00000002 */
#define OPA_CR_OPA0SELGAIN_1        (0x2UL << OPA_CR_OPA0SELGAIN_Pos)           /*!< 0x00000004 */
#define OPA_CR_OPA0SELGAIN_2        (0x4UL << OPA_CR_OPA0SELGAIN_Pos)           /*!< 0x00000008 */
#define OPA_CR_OPA0SELGAIN_Msk      (0x7UL << OPA_CR_OPA0SELGAIN_Pos)           /*!< 0x0000000E */
#define OPA_CR_OPA0SELGAIN          OPA_CR_OPA0SELGAIN_Msk
#define OPA_CR_OPA0INSEL_Pos        (4U)                                        /*!< OPA0_IN_SEL */
#define OPA_CR_OPA0INSEL_Msk        (0x1UL << OPA_CR_OPA0INSEL_Pos)             /*!< 0x00000010 */
#define OPA_CR_OPA0INSEL            OPA_CR_OPA0INSEL_Msk
#define OPA_CR_OPA0OUTSEL_Pos       (5U)                                        /*!< OPA0_OUT_SEL */
#define OPA_CR_OPA0OUTSEL_Msk       (0x1UL << OPA_CR_OPA0OUTSEL_Pos)            /*!< 0x00000020 */
#define OPA_CR_OPA0OUTSEL           OPA_CR_OPA0OUTSEL_Msk
#define OPA_CR_OPA1EN_Pos           (8U)                                        /*!< OPA1_EN */
#define OPA_CR_OPA1EN_Msk           (0x1UL << OPA_CR_OPA1EN_Pos)                /*!< 0x00000100 */
#define OPA_CR_OPA1EN               OPA_CR_OPA1EN_Msk
#define OPA_CR_OPA1SELGAIN_Pos      (9U)                                        /*!< OPA1_SEL_GAIN */
#define OPA_CR_OPA1SELGAIN_0        (0x1UL << OPA_CR_OPA1SELGAIN_Pos)           /*!< 0x00000200 */
#define OPA_CR_OPA1SELGAIN_1        (0x2UL << OPA_CR_OPA1SELGAIN_Pos)           /*!< 0x00000400 */
#define OPA_CR_OPA1SELGAIN_2        (0x4UL << OPA_CR_OPA1SELGAIN_Pos)           /*!< 0x00000800 */
#define OPA_CR_OPA1SELGAIN_Msk      (0x7UL << OPA_CR_OPA1SELGAIN_Pos)           /*!< 0x00000E00 */
#define OPA_CR_OPA1SELGAIN          OPA_CR_OPA1SELGAIN_Msk
#define OPA_CR_OPA1INSEL_Pos        (12U)                                       /*!< OPA1_IN_SEL */
#define OPA_CR_OPA1INSEL_Msk        (0x1UL << OPA_CR_OPA1INSEL_Pos)             /*!< 0x00001000 */
#define OPA_CR_OPA1INSEL            OPA_CR_OPA1INSEL_Msk
#define OPA_CR_OPA1OUTSEL_Pos       (13U)                                       /*!< OPA1_OUT_SEL */
#define OPA_CR_OPA1OUTSEL_Msk       (0x1UL << OPA_CR_OPA1OUTSEL_Pos)            /*!< 0x00002000 */
#define OPA_CR_OPA1OUTSEL           OPA_CR_OPA1OUTSEL_Msk
#define OPA_CR_OPASELVCM_Pos        (16U)                                       /*!< OPA_SEL_VCM */
#define OPA_CR_OPASELVCM_0          (0x1UL << OPA_CR_OPASELVCM_Pos)             /*!< 0x00010000 */
#define OPA_CR_OPASELVCM_1          (0x2UL << OPA_CR_OPASELVCM_Pos)             /*!< 0x00020000 */
#define OPA_CR_OPASELVCM_2          (0x4UL << OPA_CR_OPASELVCM_Pos)             /*!< 0x00040000 */
#define OPA_CR_OPASELVCM_Msk        (0x7UL << OPA_CR_OPASELVCM_Pos)             /*!< 0x00070000 */
#define OPA_CR_OPASELVCM            OPA_CR_OPASELVCM_Msk

/** @} */ /* End of group PosMask_peripherals */

/** @addtogroup Exported_macro
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC))

#define IS_ADC_MULTMRODE_MASTER_INSTANCE(INSTANCE) ((INSTANCE) == ADC)

/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************** DMA Instances *******************************/
#define IS_DMA_STREAM_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Stream0) || \
                                              ((INSTANCE) == DMA1_Stream1))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB))

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C))

/******************************** LPTMR Instances *******************************/
#define IS_LPTMR_ALL_INSTANCE(INSTANCE) (((INSTANCE) == LPTMR))

/****************** TMR Instances : All supported instances *******************/
#define IS_ATMR_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ATMR)
#define IS_GTMR_ALL_INSTANCE(INSTANCE) ((INSTANCE) == GTMR)
#define IS_BTMR_ALL_INSTANCE(INSTANCE) ((INSTANCE) == BTMR0 || (INSTANCE) == BTMR1)
/************* TMR Instances : at least 0 capture/compare channel *************/
#define IS_ATMR_CC0_INSTANCE(INSTANCE)   ((INSTANCE) == ATMR)
#define IS_GTMR_CC0_INSTANCE(INSTANCE)   ((INSTANCE) == GTMR)
#define IS_BTMR_CC0_INSTANCE(INSTANCE)   ((INSTANCE) == BTMR0 || (INSTANCE) == BTMR1)
/************ TMR Instances : at least 1 capture/compare channels *************/
#define IS_ATMR_CC1_INSTANCE(INSTANCE)   ((INSTANCE) == ATMR)
#define IS_GTMR_CC1_INSTANCE(INSTANCE)   ((INSTANCE) == GTMR)
#define IS_BTMR_CC1_INSTANCE(INSTANCE)   ((INSTANCE) == BTMR0 || (INSTANCE) == BTMR1)
/************ TMR Instances : at least 2 capture/compare channels *************/
#define IS_ATMR_CC2_INSTANCE(INSTANCE)   ((INSTANCE) == ATMR)
#define IS_GTMR_CC2_INSTANCE(INSTANCE)   ((INSTANCE) == GTMR)
#define IS_BTMR_CC2_INSTANCE(INSTANCE)   ((INSTANCE) == BTMR0 || (INSTANCE) == BTMR1)
/************ TMR Instances : at least 3 capture/compare channels *************/
#define IS_ATMR_CC3_INSTANCE(INSTANCE)   ((INSTANCE) == ATMR)
#define IS_GTMR_CC3_INSTANCE(INSTANCE)   ((INSTANCE) == GTMR)

/******************** TMR Instances : Advanced-control timers *****************/
#define IS_TMR_ADVANCED_INSTANCE(INSTANCE) (((INSTANCE) == ATMR))

/******************* TMR Instances : Timer input XOR function *****************/
#define IS_TMR_XOR_INSTANCE(INSTANCE)   (((INSTANCE) == ATMR) || \
                                         ((INSTANCE) == GTMR))

/****** TMR Instances : master mode available (TMRx_CR2.MMS available )********/
#define IS_TMR_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == ATMR)  || \
                                          ((INSTANCE) == GTMR))

/*********** TMR Instances : Slave mode available (TMRx_SMCR available )*******/
#define IS_TMR_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == ATMR) || \
                                         ((INSTANCE) == GTMR))
/********************** TMR Instances : 32 bit Counter ************************/
#define IS_TMR_32B_COUNTER_INSTANCE(INSTANCE)(((INSTANCE) == GTMR))

/***************** TMR Instances : external trigger input available ************/
#define IS_TMR_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == ATMR) || \
                                        ((INSTANCE) == GTMR))

/****************** TMR Instances : supporting counting mode selection ********/
#define IS_TMR_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == ATMR) || \
                                                        ((INSTANCE) == GTMR))
#define IS_BTMR_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == BTMR0) || \
                                                         ((INSTANCE) == BTMR1))
/****************** TMR Instances : supporting clock division *****************/
#define IS_TMR_CLOCK_DIVISION_INSTANCE(INSTANCE) (((INSTANCE) == ATMR)  || \
                                                  ((INSTANCE) == GTMR))

/****************** TMR Instances : supporting commutation event generation ***/
#define IS_TMR_COMMUTATION_EVENT_INSTANCE(INSTANCE) (((INSTANCE) == ATMR))


/****************** TMR Instances : supporting OCxREF clear *******************/
#define IS_TMR_OCXREF_CLEAR_INSTANCE(INSTANCE)        (((INSTANCE) == ATMR) || \
                                                       ((INSTANCE) == GTMR))

/****** TMR Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TMR_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == ATMR) || \
                                                        ((INSTANCE) == GTMR))

/****** TMR Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == ATMR) || \
                                                        ((INSTANCE) == GTMR))

/****** TMR Instances : supporting external clock mode 1 for TIX inputs ******/
#define IS_TMR_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)      (((INSTANCE) == ATMR) || \
                                                        ((INSTANCE) == GTMR))

/********** TMR Instances : supporting internal trigger inputs(ITRX) *********/
#define IS_TMR_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)     (((INSTANCE) == ATMR) || \
                                                        ((INSTANCE) == GTMR))

/****************** TMR Instances : supporting repetition counter *************/
#define IS_TMR_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == ATMR))

/****************** TMR Instances : supporting encoder interface **************/
#define IS_GTMR_ENCODER_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == GTMR))
/****************** TMR Instances : supporting Hall sensor interface **********/
#define IS_GTMR_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == GTMR))
/****************** TMR Instances : supporting the break function *************/
#define IS_TMR_BREAK_INSTANCE(INSTANCE)  (((INSTANCE) == ATMR))

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == UART) || \
                                     ((INSTANCE) == USART))

/****************** USART Instances : Hardware Flow control ********************/
#define IS_USART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == UART) || \
                                           ((INSTANCE) == USART))
/******************** USART Instances : LIN mode **********************/
#define IS_USART_LIN_INSTANCE          IS_USART_HALFDUPLEX_INSTANCE

/********************* USART Instances : Smart card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == UART) || \
                                         ((INSTANCE) == USART))

/*********************** USART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == UART) || \
                                    ((INSTANCE) == USART))

/****************************** IWDT Instances ********************************/
#define IS_IWDT_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDT)

/****************************** WWDT Instances ********************************/
#define IS_WWDT_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDT)

/****************************** OPA Instances ********************************/
#define IS_OPA_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == OPA)



#define RCC_MAX_FREQUENCY             64000000U           /*!< Max frequency of family in Hz*/
#define RCC_MAX_FREQUENCY_SCALE1      RCC_MAX_FREQUENCY   /*!< Maximum frequency for system clock at power scale1, in Hz */

#define FLASH_SCALE1_LATENCY1_FREQ    32000000U           /*!< HCLK frequency to set FLASH latency 1 in power scale 1  */

#ifdef __cplusplus
}
#endif

#endif /* G32F031_H */


/** @} */ /* End of group G32F031 */

/** @} */ /* End of group  */
