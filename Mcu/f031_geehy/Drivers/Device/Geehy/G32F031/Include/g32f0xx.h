/**
 *
 * @file        g32f0xx.h
 *
 * @brief       CMSIS G32F031 Device Peripheral Access Layer Header File.
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
 * Copyright (c) 2017 STMicroelectronics.
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

/** @addtogroup g32f0xx
  * @{
  */

#ifndef __G32F0xx_H
#define __G32F0xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief G32 Family
  */
#if !defined  (G32F0)
#define G32F0
#endif /* G32M3 */

/* Uncomment the line below according to the target G32 device used in your
   application
  */
#if !defined (G32F031xx)
    /* #define G32F031xx */   /*!< G32F031K8, G32F031G8 and G32F031E8 Devices */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_DAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
  /*#define USE_DAL_DRIVER */
#endif /* USE_DAL_DRIVER */

/**
  * @brief CMSIS version number V1.0.0
  */
#define __G32F0xx_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __G32F0xx_CMSIS_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __G32F0xx_CMSIS_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __G32F0xx_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __G32F0xx_CMSIS_VERSION        ((__G32F0xx_CMSIS_VERSION_MAIN << 24)\
                                           |(__G32F0xx_CMSIS_VERSION_SUB1 << 16)\
                                           |(__G32F0xx_CMSIS_VERSION_SUB2 << 8 )\
                                           |(__G32F0xx_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(G32F031xx)
  #include "g32f031xx.h"
#else
 #error "Please select first the target G32F0xx device used in your application (in g32f0xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0U,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macro
  * @{
  */

/* Define one bit mask */
#define BIT0                    ((uint32_t)0x00000001)
#define BIT1                    ((uint32_t)0x00000002)
#define BIT2                    ((uint32_t)0x00000004)
#define BIT3                    ((uint32_t)0x00000008)
#define BIT4                    ((uint32_t)0x00000010)
#define BIT5                    ((uint32_t)0x00000020)
#define BIT6                    ((uint32_t)0x00000040)
#define BIT7                    ((uint32_t)0x00000080)
#define BIT8                    ((uint32_t)0x00000100)
#define BIT9                    ((uint32_t)0x00000200)
#define BIT10                   ((uint32_t)0x00000400)
#define BIT11                   ((uint32_t)0x00000800)
#define BIT12                   ((uint32_t)0x00001000)
#define BIT13                   ((uint32_t)0x00002000)
#define BIT14                   ((uint32_t)0x00004000)
#define BIT15                   ((uint32_t)0x00008000)
#define BIT16                   ((uint32_t)0x00010000)
#define BIT17                   ((uint32_t)0x00020000)
#define BIT18                   ((uint32_t)0x00040000)
#define BIT19                   ((uint32_t)0x00080000)
#define BIT20                   ((uint32_t)0x00100000)
#define BIT21                   ((uint32_t)0x00200000)
#define BIT22                   ((uint32_t)0x00400000)
#define BIT23                   ((uint32_t)0x00800000)
#define BIT24                   ((uint32_t)0x01000000)
#define BIT25                   ((uint32_t)0x02000000)
#define BIT26                   ((uint32_t)0x04000000)
#define BIT27                   ((uint32_t)0x08000000)
#define BIT28                   ((uint32_t)0x10000000)
#define BIT29                   ((uint32_t)0x20000000)
#define BIT30                   ((uint32_t)0x40000000)
#define BIT31                   ((uint32_t)0x80000000)


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/**
  * @}
  */

#if defined (USE_DAL_DRIVER)
 #include "g32f031_dal.h"
#endif /* USE_DAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __G32F031_H */
/**
  * @}
  */

/**
  * @}
  */
