/*
** ###################################################################
**     Processors:          MCXA132VFM
**                          MCXA132VFT
**                          MCXA132VLF
**                          MCXA133VFM
**                          MCXA133VFT
**                          MCXA133VLF
**                          MCXA142VFM
**                          MCXA142VFT
**                          MCXA142VLF
**                          MCXA142VLH
**                          MCXA143VFM
**                          MCXA143VFT
**                          MCXA143VLF
**                          MCXA143VLH
**                          MCXA152VFM
**                          MCXA152VFT
**                          MCXA152VLF
**                          MCXA152VLH
**                          MCXA153VFM
**                          MCXA153VFT
**                          MCXA153VLF
**                          MCXA153VLH
**
**     Version:             rev. 2.0, 2024-10-29
**     Build:               b250521
**
**     Abstract:
**         CMSIS Peripheral Access Layer for ADC
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
 * @file PERI_ADC.h
 * @version 2.0
 * @date 2024-10-29
 * @brief CMSIS Peripheral Access Layer for ADC
 *
 * CMSIS Peripheral Access Layer for ADC
 */

#if !defined(PERI_ADC_H_)
#define PERI_ADC_H_                              /**< Symbol preventing repeated inclusion */

#if (defined(CPU_MCXA132VFM) || defined(CPU_MCXA132VFT) || defined(CPU_MCXA132VLF))
#include "MCXA132_COMMON.h"
#elif (defined(CPU_MCXA133VFM) || defined(CPU_MCXA133VFT) || defined(CPU_MCXA133VLF))
#include "MCXA133_COMMON.h"
#elif (defined(CPU_MCXA142VFM) || defined(CPU_MCXA142VFT) || defined(CPU_MCXA142VLF) || defined(CPU_MCXA142VLH))
#include "MCXA142_COMMON.h"
#elif (defined(CPU_MCXA143VFM) || defined(CPU_MCXA143VFT) || defined(CPU_MCXA143VLF) || defined(CPU_MCXA143VLH))
#include "MCXA143_COMMON.h"
#elif (defined(CPU_MCXA152VFM) || defined(CPU_MCXA152VFT) || defined(CPU_MCXA152VLF) || defined(CPU_MCXA152VLH))
#include "MCXA152_COMMON.h"
#elif (defined(CPU_MCXA153VFM) || defined(CPU_MCXA153VFT) || defined(CPU_MCXA153VLF) || defined(CPU_MCXA153VLH))
#include "MCXA153_COMMON.h"
#else
  #error "No valid CPU defined!"
#endif

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Size of Registers Arrays */
#define ADC_TCTRL_COUNT                           4u
#define ADC_GCC_COUNT                             1u
#define ADC_GCR_COUNT                             1u
#define ADC_CMD_COUNT                             7u
#define ADC_CV_COUNT                              15u
#define ADC_CAL_GAR_COUNT                         33u

/** ADC - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
       uint8_t RESERVED_0[8];
  __IO uint32_t CTRL;                              /**< Control Register, offset: 0x10 */
  __IO uint32_t STAT;                              /**< Status Register, offset: 0x14 */
  __IO uint32_t IE;                                /**< Interrupt Enable Register, offset: 0x18 */
  __IO uint32_t DE;                                /**< DMA Enable Register, offset: 0x1C */
  __IO uint32_t CFG;                               /**< Configuration Register, offset: 0x20 */
  __IO uint32_t PAUSE;                             /**< Pause Register, offset: 0x24 */
       uint8_t RESERVED_1[12];
  __IO uint32_t SWTRIG;                            /**< Software Trigger Register, offset: 0x34 */
  __IO uint32_t TSTAT;                             /**< Trigger Status Register, offset: 0x38 */
       uint8_t RESERVED_2[4];
  __IO uint32_t OFSTRIM;                           /**< Offset Trim Register, offset: 0x40 */
       uint8_t RESERVED_3[4];
  __IO uint32_t HSTRIM;                            /**< High Speed Trim Register, offset: 0x48 */
       uint8_t RESERVED_4[84];
  __IO uint32_t TCTRL[ADC_TCTRL_COUNT];            /**< Trigger Control Register, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_5[48];
  __IO uint32_t FCTRL;                             /**< FIFO Control Register, offset: 0xE0 */
       uint8_t RESERVED_6[12];
  __I  uint32_t GCC[ADC_GCC_COUNT];                /**< Gain Calibration Control, array offset: 0xF0, array step: 0x4 */
       uint8_t RESERVED_7[4];
  __IO uint32_t GCR[ADC_GCR_COUNT];                /**< Gain Calculation Result, array offset: 0xF8, array step: 0x4 */
       uint8_t RESERVED_8[4];
  struct {                                         /* offset: 0x100, array step: 0x8 */
    __IO uint32_t CMDL;                              /**< Command Low Buffer Register, array offset: 0x100, array step: 0x8 */
    __IO uint32_t CMDH;                              /**< Command High Buffer Register, array offset: 0x104, array step: 0x8 */
  } CMD[ADC_CMD_COUNT];
       uint8_t RESERVED_9[200];
  __IO uint32_t CV[ADC_CV_COUNT];                  /**< Compare Value Register, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_10[196];
  __I  uint32_t RESFIFO;                           /**< Data Result FIFO Register, offset: 0x300 */
       uint8_t RESERVED_11[252];
  __IO uint32_t CAL_GAR[ADC_CAL_GAR_COUNT];        /**< Calibration General A-Side Registers, array offset: 0x400, array step: 0x4 */
       uint8_t RESERVED_12[2932];
  __IO uint32_t CFG2;                              /**< Configuration 2 Register, offset: 0xFF8 */
} ADC_Type;

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */

#define ADC_VERID_RES_MASK                       (0x1U)
#define ADC_VERID_RES_SHIFT                      (0U)
/*! RES - Resolution
 *  0b0..Up to 12-bit single ended resolution supported (and 13-bit differential resolution if VERID[DIFFEN] = 1b).
 *  0b1..Up to 16-bit single ended resolution supported (and 16-bit differential resolution if VERID[DIFFEN] = 1b).
 */
#define ADC_VERID_RES(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_VERID_RES_SHIFT)) & ADC_VERID_RES_MASK)

#define ADC_VERID_DIFFEN_MASK                    (0x2U)
#define ADC_VERID_DIFFEN_SHIFT                   (1U)
/*! DIFFEN - Differential Supported
 *  0b0..Differential operation not supported.
 *  0b1..Differential operation supported.
 */
#define ADC_VERID_DIFFEN(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_VERID_DIFFEN_SHIFT)) & ADC_VERID_DIFFEN_MASK)

#define ADC_VERID_MVI_MASK                       (0x8U)
#define ADC_VERID_MVI_SHIFT                      (3U)
/*! MVI - Multi Vref Implemented
 *  0b0..Single voltage reference high (VREFH) input supported.
 *  0b1..Multiple voltage reference high (VREFH) inputs supported.
 */
#define ADC_VERID_MVI(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_VERID_MVI_SHIFT)) & ADC_VERID_MVI_MASK)

#define ADC_VERID_CSW_MASK                       (0x70U)
#define ADC_VERID_CSW_SHIFT                      (4U)
/*! CSW - Channel Scale Width
 *  0b000..Channel scaling not supported.
 *  0b001..Channel scaling supported. 1-bit CSCALE control field.
 *  0b110..Channel scaling supported. 6-bit CSCALE control field.
 */
#define ADC_VERID_CSW(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_VERID_CSW_SHIFT)) & ADC_VERID_CSW_MASK)

#define ADC_VERID_VR1RNGI_MASK                   (0x100U)
#define ADC_VERID_VR1RNGI_SHIFT                  (8U)
/*! VR1RNGI - Voltage Reference 1 Range Control Bit Implemented
 *  0b0..Range control not required. CFG[VREF1RNG] is not implemented.
 *  0b1..Range control required. CFG[VREF1RNG] is implemented.
 */
#define ADC_VERID_VR1RNGI(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_VERID_VR1RNGI_SHIFT)) & ADC_VERID_VR1RNGI_MASK)

#define ADC_VERID_IADCKI_MASK                    (0x200U)
#define ADC_VERID_IADCKI_SHIFT                   (9U)
/*! IADCKI - Internal ADC Clock Implemented
 *  0b0..Internal clock source not implemented.
 *  0b1..Internal clock source (and CFG[ADCKEN]) implemented.
 */
#define ADC_VERID_IADCKI(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_VERID_IADCKI_SHIFT)) & ADC_VERID_IADCKI_MASK)

#define ADC_VERID_CALOFSI_MASK                   (0x400U)
#define ADC_VERID_CALOFSI_SHIFT                  (10U)
/*! CALOFSI - Calibration Function Implemented
 *  0b0..Calibration Not Implemented.
 *  0b1..Calibration Implemented.
 */
#define ADC_VERID_CALOFSI(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_VERID_CALOFSI_SHIFT)) & ADC_VERID_CALOFSI_MASK)

#define ADC_VERID_NUM_SEC_MASK                   (0x800U)
#define ADC_VERID_NUM_SEC_SHIFT                  (11U)
/*! NUM_SEC - Number of Single Ended Outputs Supported
 *  0b0..This design supports one single ended conversion at a time.
 *  0b1..This design supports two simultaneous single ended conversions.
 */
#define ADC_VERID_NUM_SEC(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_VERID_NUM_SEC_SHIFT)) & ADC_VERID_NUM_SEC_MASK)

#define ADC_VERID_NUM_FIFO_MASK                  (0x7000U)
#define ADC_VERID_NUM_FIFO_SHIFT                 (12U)
/*! NUM_FIFO - Number of FIFOs
 *  0b000..N/A
 *  0b001..This design supports one result FIFO.
 *  0b010..This design supports two result FIFOs.
 *  0b011..This design supports three result FIFOs.
 *  0b100..This design supports four result FIFOs.
 */
#define ADC_VERID_NUM_FIFO(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_VERID_NUM_FIFO_SHIFT)) & ADC_VERID_NUM_FIFO_MASK)

#define ADC_VERID_MINOR_MASK                     (0xFF0000U)
#define ADC_VERID_MINOR_SHIFT                    (16U)
/*! MINOR - Minor Version Number */
#define ADC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_VERID_MINOR_SHIFT)) & ADC_VERID_MINOR_MASK)

#define ADC_VERID_MAJOR_MASK                     (0xFF000000U)
#define ADC_VERID_MAJOR_SHIFT                    (24U)
/*! MAJOR - Major Version Number */
#define ADC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_VERID_MAJOR_SHIFT)) & ADC_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */

#define ADC_PARAM_TRIG_NUM_MASK                  (0xFFU)
#define ADC_PARAM_TRIG_NUM_SHIFT                 (0U)
/*! TRIG_NUM - Trigger Number */
#define ADC_PARAM_TRIG_NUM(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_PARAM_TRIG_NUM_SHIFT)) & ADC_PARAM_TRIG_NUM_MASK)

#define ADC_PARAM_FIFOSIZE_MASK                  (0xFF00U)
#define ADC_PARAM_FIFOSIZE_SHIFT                 (8U)
/*! FIFOSIZE - Result FIFO Depth
 *  0b00000001..Result FIFO depth = 2 dataword.
 *  0b00000100..Result FIFO depth = 4 datawords.
 *  0b00001000..Result FIFO depth = 8 datawords.
 *  0b00010000..Result FIFO depth = 16 datawords.
 *  0b00100000..Result FIFO depth = 32 datawords.
 *  0b01000000..Result FIFO depth = 64 datawords.
 */
#define ADC_PARAM_FIFOSIZE(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_PARAM_FIFOSIZE_SHIFT)) & ADC_PARAM_FIFOSIZE_MASK)

#define ADC_PARAM_CV_NUM_MASK                    (0xFF0000U)
#define ADC_PARAM_CV_NUM_SHIFT                   (16U)
/*! CV_NUM - Compare Value Number */
#define ADC_PARAM_CV_NUM(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_PARAM_CV_NUM_SHIFT)) & ADC_PARAM_CV_NUM_MASK)

#define ADC_PARAM_CMD_NUM_MASK                   (0xFF000000U)
#define ADC_PARAM_CMD_NUM_SHIFT                  (24U)
/*! CMD_NUM - Command Buffer Number */
#define ADC_PARAM_CMD_NUM(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_PARAM_CMD_NUM_SHIFT)) & ADC_PARAM_CMD_NUM_MASK)
/*! @} */

/*! @name CTRL - Control Register */
/*! @{ */

#define ADC_CTRL_ADCEN_MASK                      (0x1U)
#define ADC_CTRL_ADCEN_SHIFT                     (0U)
/*! ADCEN - ADC Enable
 *  0b0..ADC is disabled.
 *  0b1..ADC is enabled.
 */
#define ADC_CTRL_ADCEN(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_ADCEN_SHIFT)) & ADC_CTRL_ADCEN_MASK)

#define ADC_CTRL_RST_MASK                        (0x2U)
#define ADC_CTRL_RST_SHIFT                       (1U)
/*! RST - Software Reset
 *  0b0..ADC logic is not reset.
 *  0b1..ADC logic is reset.
 */
#define ADC_CTRL_RST(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_RST_SHIFT)) & ADC_CTRL_RST_MASK)

#define ADC_CTRL_DOZEN_MASK                      (0x4U)
#define ADC_CTRL_DOZEN_SHIFT                     (2U)
/*! DOZEN - Doze Enable
 *  0b0..ADC is enabled in low power mode.
 *  0b1..ADC is disabled in low power mode.
 */
#define ADC_CTRL_DOZEN(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_DOZEN_SHIFT)) & ADC_CTRL_DOZEN_MASK)

#define ADC_CTRL_CAL_REQ_MASK                    (0x8U)
#define ADC_CTRL_CAL_REQ_SHIFT                   (3U)
/*! CAL_REQ - Auto-Calibration Request
 *  0b0..No request for hardware calibration has been made
 *  0b1..A request for hardware calibration has been made
 */
#define ADC_CTRL_CAL_REQ(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_CAL_REQ_SHIFT)) & ADC_CTRL_CAL_REQ_MASK)

#define ADC_CTRL_CALOFS_MASK                     (0x10U)
#define ADC_CTRL_CALOFS_SHIFT                    (4U)
/*! CALOFS - Offset Calibration Request
 *  0b0..No request for offset calibration has been made
 *  0b1..Request for offset calibration function
 */
#define ADC_CTRL_CALOFS(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_CALOFS_SHIFT)) & ADC_CTRL_CALOFS_MASK)

#define ADC_CTRL_CALHS_MASK                      (0x40U)
#define ADC_CTRL_CALHS_SHIFT                     (6U)
/*! CALHS - High Speed Mode Trim Request
 *  0b0..No request for high speed mode trim has been made
 *  0b1..Request for high speed mode trim has been made
 */
#define ADC_CTRL_CALHS(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_CALHS_SHIFT)) & ADC_CTRL_CALHS_MASK)

#define ADC_CTRL_RSTFIFO0_MASK                   (0x100U)
#define ADC_CTRL_RSTFIFO0_SHIFT                  (8U)
/*! RSTFIFO0 - Reset FIFO 0
 *  0b0..No effect.
 *  0b1..FIFO 0 is reset.
 */
#define ADC_CTRL_RSTFIFO0(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_RSTFIFO0_SHIFT)) & ADC_CTRL_RSTFIFO0_MASK)

#define ADC_CTRL_CAL_AVGS_MASK                   (0xF0000U)
#define ADC_CTRL_CAL_AVGS_SHIFT                  (16U)
/*! CAL_AVGS - Auto-Calibration Averages
 *  0b0000..Single conversion.
 *  0b0001..2 conversions averaged.
 *  0b0010..4 conversions averaged.
 *  0b0011..8 conversions averaged.
 *  0b0100..16 conversions averaged.
 *  0b0101..32 conversions averaged.
 *  0b0110..64 conversions averaged.
 *  0b0111..128 conversions averaged.
 *  0b1000..256 conversions averaged.
 *  0b1001..512 conversions averaged.
 *  0b1010..1024 conversions averaged.
 */
#define ADC_CTRL_CAL_AVGS(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_CAL_AVGS_SHIFT)) & ADC_CTRL_CAL_AVGS_MASK)
/*! @} */

/*! @name STAT - Status Register */
/*! @{ */

#define ADC_STAT_RDY0_MASK                       (0x1U)
#define ADC_STAT_RDY0_SHIFT                      (0U)
/*! RDY0 - Result FIFO 0 Ready Flag
 *  0b0..Result FIFO 0 data level not above watermark level.
 *  0b1..Result FIFO 0 holding data above watermark level.
 */
#define ADC_STAT_RDY0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_STAT_RDY0_SHIFT)) & ADC_STAT_RDY0_MASK)

#define ADC_STAT_FOF0_MASK                       (0x2U)
#define ADC_STAT_FOF0_SHIFT                      (1U)
/*! FOF0 - Result FIFO 0 Overflow Flag
 *  0b0..No result FIFO 0 overflow has occurred since the last time the flag was cleared.
 *  0b1..At least one result FIFO 0 overflow has occurred since the last time the flag was cleared.
 */
#define ADC_STAT_FOF0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_STAT_FOF0_SHIFT)) & ADC_STAT_FOF0_MASK)

#define ADC_STAT_TEXC_INT_MASK                   (0x100U)
#define ADC_STAT_TEXC_INT_SHIFT                  (8U)
/*! TEXC_INT - Interrupt Flag For High Priority Trigger Exception
 *  0b0..No trigger exceptions have occurred.
 *  0b1..A trigger exception has occurred and is pending acknowledgement.
 */
#define ADC_STAT_TEXC_INT(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_STAT_TEXC_INT_SHIFT)) & ADC_STAT_TEXC_INT_MASK)

#define ADC_STAT_TCOMP_INT_MASK                  (0x200U)
#define ADC_STAT_TCOMP_INT_SHIFT                 (9U)
/*! TCOMP_INT - Interrupt Flag For Trigger Completion
 *  0b0..Either IE[TCOMP_IE] is set to 0, or no trigger sequences have run to completion.
 *  0b1..Trigger sequence has been completed and all data is stored in the associated FIFO.
 */
#define ADC_STAT_TCOMP_INT(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_STAT_TCOMP_INT_SHIFT)) & ADC_STAT_TCOMP_INT_MASK)

#define ADC_STAT_CAL_RDY_MASK                    (0x400U)
#define ADC_STAT_CAL_RDY_SHIFT                   (10U)
/*! CAL_RDY - Calibration Ready
 *  0b0..Calibration is incomplete or hasn't been ran.
 *  0b1..The ADC is calibrated.
 */
#define ADC_STAT_CAL_RDY(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_STAT_CAL_RDY_SHIFT)) & ADC_STAT_CAL_RDY_MASK)

#define ADC_STAT_ADC_ACTIVE_MASK                 (0x800U)
#define ADC_STAT_ADC_ACTIVE_SHIFT                (11U)
/*! ADC_ACTIVE - ADC Active
 *  0b0..The ADC is IDLE. There are no pending triggers to service and no active commands are being processed.
 *  0b1..The ADC is processing a conversion, running through the power up delay, or servicing a trigger.
 */
#define ADC_STAT_ADC_ACTIVE(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_STAT_ADC_ACTIVE_SHIFT)) & ADC_STAT_ADC_ACTIVE_MASK)

#define ADC_STAT_TRGACT_MASK                     (0x30000U)
#define ADC_STAT_TRGACT_SHIFT                    (16U)
/*! TRGACT - Trigger Active
 *  0b00..Command (sequence) associated with Trigger 0 currently being executed.
 *  0b01..Command (sequence) associated with Trigger 1 currently being executed.
 *  0b10..Command (sequence) associated with Trigger 2 currently being executed.
 *  0b11..Command (sequence) associated with Trigger 3 currently being executed.
 */
#define ADC_STAT_TRGACT(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_STAT_TRGACT_SHIFT)) & ADC_STAT_TRGACT_MASK)

#define ADC_STAT_CMDACT_MASK                     (0x7000000U)
#define ADC_STAT_CMDACT_SHIFT                    (24U)
/*! CMDACT - Command Active
 *  0b000..No command is currently in progress.
 *  0b001..Command 1 currently being executed.
 *  0b010..Command 2 currently being executed.
 *  0b011-0b111..Associated command number is currently being executed.
 */
#define ADC_STAT_CMDACT(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_STAT_CMDACT_SHIFT)) & ADC_STAT_CMDACT_MASK)
/*! @} */

/*! @name IE - Interrupt Enable Register */
/*! @{ */

#define ADC_IE_FWMIE0_MASK                       (0x1U)
#define ADC_IE_FWMIE0_SHIFT                      (0U)
/*! FWMIE0 - FIFO 0 Watermark Interrupt Enable
 *  0b0..FIFO 0 watermark interrupts are not enabled.
 *  0b1..FIFO 0 watermark interrupts are enabled.
 */
#define ADC_IE_FWMIE0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_IE_FWMIE0_SHIFT)) & ADC_IE_FWMIE0_MASK)

#define ADC_IE_FOFIE0_MASK                       (0x2U)
#define ADC_IE_FOFIE0_SHIFT                      (1U)
/*! FOFIE0 - Result FIFO 0 Overflow Interrupt Enable
 *  0b0..FIFO 0 overflow interrupts are not enabled.
 *  0b1..FIFO 0 overflow interrupts are enabled.
 */
#define ADC_IE_FOFIE0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_IE_FOFIE0_SHIFT)) & ADC_IE_FOFIE0_MASK)

#define ADC_IE_TEXC_IE_MASK                      (0x100U)
#define ADC_IE_TEXC_IE_SHIFT                     (8U)
/*! TEXC_IE - Trigger Exception Interrupt Enable
 *  0b0..Trigger exception interrupts are disabled.
 *  0b1..Trigger exception interrupts are enabled.
 */
#define ADC_IE_TEXC_IE(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_IE_TEXC_IE_SHIFT)) & ADC_IE_TEXC_IE_MASK)

#define ADC_IE_TCOMP_IE_MASK                     (0xF0000U)
#define ADC_IE_TCOMP_IE_SHIFT                    (16U)
/*! TCOMP_IE - Trigger Completion Interrupt Enable
 *  0b0000..Trigger completion interrupts are disabled.
 *  0b0001..Trigger completion interrupts are enabled for trigger source 0 only.
 *  0b0010..Trigger completion interrupts are enabled for trigger source 1 only.
 *  0b0011-0b1110..Associated trigger completion interrupts are enabled.
 *  0b1111..Trigger completion interrupts are enabled for every trigger source.
 */
#define ADC_IE_TCOMP_IE(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_IE_TCOMP_IE_SHIFT)) & ADC_IE_TCOMP_IE_MASK)
/*! @} */

/*! @name DE - DMA Enable Register */
/*! @{ */

#define ADC_DE_FWMDE0_MASK                       (0x1U)
#define ADC_DE_FWMDE0_SHIFT                      (0U)
/*! FWMDE0 - FIFO 0 Watermark DMA Enable
 *  0b0..DMA request disabled.
 *  0b1..DMA request enabled.
 */
#define ADC_DE_FWMDE0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_DE_FWMDE0_SHIFT)) & ADC_DE_FWMDE0_MASK)
/*! @} */

/*! @name CFG - Configuration Register */
/*! @{ */

#define ADC_CFG_TPRICTRL_MASK                    (0x3U)
#define ADC_CFG_TPRICTRL_SHIFT                   (0U)
/*! TPRICTRL - ADC Trigger Priority Control
 *  0b00..If a higher priority trigger is detected during command processing, the current conversion is aborted
 *        and the new command specified by the trigger is started.
 *  0b01..If a higher priority trigger is received during command processing, the current command is stopped after
 *        completing the current conversion. If averaging is enabled, the averaging loop will be completed.
 *        However, CMDHa[LOOP] will be ignored and the higher priority trigger will be serviced.
 *  0b10..If a higher priority trigger is received during command processing, the current command will be
 *        completed (averaging, looping, compare) before servicing the higher priority trigger.
 *  0b11..
 */
#define ADC_CFG_TPRICTRL(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CFG_TPRICTRL_SHIFT)) & ADC_CFG_TPRICTRL_MASK)

#define ADC_CFG_PWRSEL_MASK                      (0x20U)
#define ADC_CFG_PWRSEL_SHIFT                     (5U)
/*! PWRSEL - Power Configuration Select
 *  0b0..Low power
 *  0b1..High power
 */
#define ADC_CFG_PWRSEL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CFG_PWRSEL_SHIFT)) & ADC_CFG_PWRSEL_MASK)

#define ADC_CFG_REFSEL_MASK                      (0xC0U)
#define ADC_CFG_REFSEL_SHIFT                     (6U)
/*! REFSEL - Voltage Reference Selection
 *  0b00..(Default) Option 1 setting.
 *  0b01..Option 2 setting.
 *  0b10..Option 3 setting.
 *  0b11..Reserved
 */
#define ADC_CFG_REFSEL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CFG_REFSEL_SHIFT)) & ADC_CFG_REFSEL_MASK)

#define ADC_CFG_TRES_MASK                        (0x100U)
#define ADC_CFG_TRES_SHIFT                       (8U)
/*! TRES - Trigger Resume Enable
 *  0b0..Trigger sequences interrupted by a high priority trigger exception are not automatically resumed or restarted.
 *  0b1..Trigger sequences interrupted by a high priority trigger exception are automatically resumed or restarted.
 */
#define ADC_CFG_TRES(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_CFG_TRES_SHIFT)) & ADC_CFG_TRES_MASK)

#define ADC_CFG_TCMDRES_MASK                     (0x200U)
#define ADC_CFG_TCMDRES_SHIFT                    (9U)
/*! TCMDRES - Trigger Command Resume
 *  0b0..Trigger sequences interrupted by a high priority trigger exception is automatically restarted.
 *  0b1..Trigger sequences interrupted by a high priority trigger exception is resumed from the command executing before the exception.
 */
#define ADC_CFG_TCMDRES(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG_TCMDRES_SHIFT)) & ADC_CFG_TCMDRES_MASK)

#define ADC_CFG_HPT_EXDI_MASK                    (0x400U)
#define ADC_CFG_HPT_EXDI_SHIFT                   (10U)
/*! HPT_EXDI - High Priority Trigger Exception Disable
 *  0b0..High priority trigger exceptions are enabled.
 *  0b1..High priority trigger exceptions are disabled.
 */
#define ADC_CFG_HPT_EXDI(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CFG_HPT_EXDI_SHIFT)) & ADC_CFG_HPT_EXDI_MASK)

#define ADC_CFG_PUDLY_MASK                       (0xFF0000U)
#define ADC_CFG_PUDLY_SHIFT                      (16U)
/*! PUDLY - Power Up Delay */
#define ADC_CFG_PUDLY(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG_PUDLY_SHIFT)) & ADC_CFG_PUDLY_MASK)

#define ADC_CFG_PWREN_MASK                       (0x10000000U)
#define ADC_CFG_PWREN_SHIFT                      (28U)
/*! PWREN - ADC Analog Pre-Enable
 *  0b0..ADC analog circuits are only enabled while conversions are active. Performance is affected due to analog startup delays.
 *  0b1..ADC analog circuits are pre-enabled and ready to execute conversions without startup delays (at the cost
 *       of higher DC current consumption). Note that a single power up delay (CFG[PUDLY]) is executed immediately
 *       once PWREN is set, and any detected trigger does not begin ADC operation until the power up delay time has
 *       passed. After this initial delay expires the analog remains pre-enabled and no additional delays are
 *       executed.
 */
#define ADC_CFG_PWREN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG_PWREN_SHIFT)) & ADC_CFG_PWREN_MASK)
/*! @} */

/*! @name PAUSE - Pause Register */
/*! @{ */

#define ADC_PAUSE_PAUSEDLY_MASK                  (0x1FFU)
#define ADC_PAUSE_PAUSEDLY_SHIFT                 (0U)
/*! PAUSEDLY - Pause Delay */
#define ADC_PAUSE_PAUSEDLY(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_PAUSE_PAUSEDLY_SHIFT)) & ADC_PAUSE_PAUSEDLY_MASK)

#define ADC_PAUSE_PAUSEEN_MASK                   (0x80000000U)
#define ADC_PAUSE_PAUSEEN_SHIFT                  (31U)
/*! PAUSEEN - PAUSE Option Enable
 *  0b0..Pause operation disabled
 *  0b1..Pause operation enabled
 */
#define ADC_PAUSE_PAUSEEN(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_PAUSE_PAUSEEN_SHIFT)) & ADC_PAUSE_PAUSEEN_MASK)
/*! @} */

/*! @name SWTRIG - Software Trigger Register */
/*! @{ */

#define ADC_SWTRIG_SWT0_MASK                     (0x1U)
#define ADC_SWTRIG_SWT0_SHIFT                    (0U)
/*! SWT0 - Software Trigger 0 Event
 *  0b0..No trigger 0 event generated.
 *  0b1..Trigger 0 event generated.
 */
#define ADC_SWTRIG_SWT0(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_SWTRIG_SWT0_SHIFT)) & ADC_SWTRIG_SWT0_MASK)

#define ADC_SWTRIG_SWT1_MASK                     (0x2U)
#define ADC_SWTRIG_SWT1_SHIFT                    (1U)
/*! SWT1 - Software Trigger 1 Event
 *  0b0..No trigger 1 event generated.
 *  0b1..Trigger 1 event generated.
 */
#define ADC_SWTRIG_SWT1(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_SWTRIG_SWT1_SHIFT)) & ADC_SWTRIG_SWT1_MASK)

#define ADC_SWTRIG_SWT2_MASK                     (0x4U)
#define ADC_SWTRIG_SWT2_SHIFT                    (2U)
/*! SWT2 - Software Trigger 2 Event
 *  0b0..No trigger 2 event generated.
 *  0b1..Trigger 2 event generated.
 */
#define ADC_SWTRIG_SWT2(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_SWTRIG_SWT2_SHIFT)) & ADC_SWTRIG_SWT2_MASK)

#define ADC_SWTRIG_SWT3_MASK                     (0x8U)
#define ADC_SWTRIG_SWT3_SHIFT                    (3U)
/*! SWT3 - Software Trigger 3 Event
 *  0b0..No trigger 3 event generated.
 *  0b1..Trigger 3 event generated.
 */
#define ADC_SWTRIG_SWT3(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_SWTRIG_SWT3_SHIFT)) & ADC_SWTRIG_SWT3_MASK)
/*! @} */

/*! @name TSTAT - Trigger Status Register */
/*! @{ */

#define ADC_TSTAT_TEXC_NUM_MASK                  (0xFU)
#define ADC_TSTAT_TEXC_NUM_SHIFT                 (0U)
/*! TEXC_NUM - Trigger Exception Number
 *  0b0000..No triggers have been interrupted by a high priority exception. Or CFG[TRES] = 1.
 *  0b0001..Trigger 0 has been interrupted by a high priority exception.
 *  0b0010..Trigger 1 has been interrupted by a high priority exception.
 *  0b0011-0b1110..Associated trigger sequence has interrupted by a high priority exception.
 *  0b1111..Every trigger sequence has been interrupted by a high priority exception.
 */
#define ADC_TSTAT_TEXC_NUM(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_TSTAT_TEXC_NUM_SHIFT)) & ADC_TSTAT_TEXC_NUM_MASK)

#define ADC_TSTAT_TCOMP_FLAG_MASK                (0xF0000U)
#define ADC_TSTAT_TCOMP_FLAG_SHIFT               (16U)
/*! TCOMP_FLAG - Trigger Completion Flag
 *  0b0000..No triggers have been completed. Trigger completion interrupts are disabled.
 *  0b0001..Trigger 0 has been completed and trigger 0 has enabled completion interrupts.
 *  0b0010..Trigger 1 has been completed and trigger 1 has enabled completion interrupts.
 *  0b0011-0b1110..Associated trigger sequence has completed and has enabled completion interrupts.
 *  0b1111..Every trigger sequence has been completed and every trigger has enabled completion interrupts.
 */
#define ADC_TSTAT_TCOMP_FLAG(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_TSTAT_TCOMP_FLAG_SHIFT)) & ADC_TSTAT_TCOMP_FLAG_MASK)
/*! @} */

/*! @name OFSTRIM - Offset Trim Register */
/*! @{ */

#define ADC_OFSTRIM_OFSTRIM_MASK                 (0x3FFU)
#define ADC_OFSTRIM_OFSTRIM_SHIFT                (0U)
/*! OFSTRIM - Trim for Offset */
#define ADC_OFSTRIM_OFSTRIM(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_OFSTRIM_OFSTRIM_SHIFT)) & ADC_OFSTRIM_OFSTRIM_MASK)
/*! @} */

/*! @name HSTRIM - High Speed Trim Register */
/*! @{ */

#define ADC_HSTRIM_HSTRIM_MASK                   (0x1FU)
#define ADC_HSTRIM_HSTRIM_SHIFT                  (0U)
/*! HSTRIM - Trim for High Speed Conversions */
#define ADC_HSTRIM_HSTRIM(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_HSTRIM_HSTRIM_SHIFT)) & ADC_HSTRIM_HSTRIM_MASK)
/*! @} */

/*! @name TCTRL - Trigger Control Register */
/*! @{ */

#define ADC_TCTRL_HTEN_MASK                      (0x1U)
#define ADC_TCTRL_HTEN_SHIFT                     (0U)
/*! HTEN - Trigger Enable
 *  0b0..Hardware trigger source disabled
 *  0b1..Hardware trigger source enabled
 */
#define ADC_TCTRL_HTEN(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_HTEN_SHIFT)) & ADC_TCTRL_HTEN_MASK)

#define ADC_TCTRL_TPRI_MASK                      (0x300U)
#define ADC_TCTRL_TPRI_SHIFT                     (8U)
/*! TPRI - Trigger Priority Setting
 *  0b00..Set to highest priority, Level 1
 *  0b01-0b10..Set to corresponding priority level
 *  0b11..Set to lowest priority, Level 4
 */
#define ADC_TCTRL_TPRI(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_TPRI_SHIFT)) & ADC_TCTRL_TPRI_MASK)

#define ADC_TCTRL_RSYNC_MASK                     (0x8000U)
#define ADC_TCTRL_RSYNC_SHIFT                    (15U)
/*! RSYNC - Trigger Resync */
#define ADC_TCTRL_RSYNC(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_RSYNC_SHIFT)) & ADC_TCTRL_RSYNC_MASK)

#define ADC_TCTRL_TDLY_MASK                      (0xF0000U)
#define ADC_TCTRL_TDLY_SHIFT                     (16U)
/*! TDLY - Trigger Delay Select */
#define ADC_TCTRL_TDLY(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_TDLY_SHIFT)) & ADC_TCTRL_TDLY_MASK)

#define ADC_TCTRL_TSYNC_MASK                     (0x800000U)
#define ADC_TCTRL_TSYNC_SHIFT                    (23U)
/*! TSYNC - Trigger Synchronous Select */
#define ADC_TCTRL_TSYNC(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_TSYNC_SHIFT)) & ADC_TCTRL_TSYNC_MASK)

#define ADC_TCTRL_TCMD_MASK                      (0x7000000U)
#define ADC_TCTRL_TCMD_SHIFT                     (24U)
/*! TCMD - Trigger Command Select
 *  0b000..Not a valid selection from the command buffer. Trigger event is ignored.
 *  0b001..CMD1 is executed
 *  0b010-0b110..Corresponding CMD is executed
 *  0b111..CMD7 is executed
 */
#define ADC_TCTRL_TCMD(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_TCTRL_TCMD_SHIFT)) & ADC_TCTRL_TCMD_MASK)
/*! @} */

/*! @name FCTRL - FIFO Control Register */
/*! @{ */

#define ADC_FCTRL_FCOUNT_MASK                    (0xFU)
#define ADC_FCTRL_FCOUNT_SHIFT                   (0U)
/*! FCOUNT - Result FIFO Counter */
#define ADC_FCTRL_FCOUNT(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FCTRL_FCOUNT_SHIFT)) & ADC_FCTRL_FCOUNT_MASK)

#define ADC_FCTRL_FWMARK_MASK                    (0x70000U)
#define ADC_FCTRL_FWMARK_SHIFT                   (16U)
/*! FWMARK - Watermark Level Selection */
#define ADC_FCTRL_FWMARK(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FCTRL_FWMARK_SHIFT)) & ADC_FCTRL_FWMARK_MASK)
/*! @} */

/*! @name GCC - Gain Calibration Control */
/*! @{ */

#define ADC_GCC_GAIN_CAL_MASK                    (0xFFFFU)
#define ADC_GCC_GAIN_CAL_SHIFT                   (0U)
/*! GAIN_CAL - Gain Calibration Value */
#define ADC_GCC_GAIN_CAL(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_GCC_GAIN_CAL_SHIFT)) & ADC_GCC_GAIN_CAL_MASK)

#define ADC_GCC_RDY_MASK                         (0x1000000U)
#define ADC_GCC_RDY_SHIFT                        (24U)
/*! RDY - Gain Calibration Value Valid
 *  0b0..The GAIN_CAL value is invalid. Run the hardware calibration routine for this value to be set.
 *  0b1..The GAIN_CAL value is valid. GAIN_CAL should be used by software to derive GCRa[GCALR].
 */
#define ADC_GCC_RDY(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_GCC_RDY_SHIFT)) & ADC_GCC_RDY_MASK)
/*! @} */

/*! @name GCR - Gain Calculation Result */
/*! @{ */

#define ADC_GCR_GCALR_MASK                       (0x1FFFFU)
#define ADC_GCR_GCALR_SHIFT                      (0U)
/*! GCALR - Gain Calculation Result */
#define ADC_GCR_GCALR(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_GCR_GCALR_SHIFT)) & ADC_GCR_GCALR_MASK)

#define ADC_GCR_RDY_MASK                         (0x1000000U)
#define ADC_GCR_RDY_SHIFT                        (24U)
/*! RDY - Gain Calculation Ready
 *  0b0..The GCALR value is invalid.
 *  0b1..The GCALR value is valid.
 */
#define ADC_GCR_RDY(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_GCR_RDY_SHIFT)) & ADC_GCR_RDY_MASK)
/*! @} */

/*! @name CMDL - Command Low Buffer Register */
/*! @{ */

#define ADC_CMDL_ADCH_MASK                       (0x1FU)
#define ADC_CMDL_ADCH_SHIFT                      (0U)
/*! ADCH - Input Channel Select
 *  0b00000..Select CH0A.
 *  0b00001..Select CH1A.
 *  0b00010..Select CH2A.
 *  0b00011..Select CH3A.
 *  0b00100-0b11101..Select corresponding channel CHnA.
 *  0b11110..Select CH30A.
 *  0b11111..Select CH31A.
 */
#define ADC_CMDL_ADCH(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CMDL_ADCH_SHIFT)) & ADC_CMDL_ADCH_MASK)

#define ADC_CMDL_CTYPE_MASK                      (0x60U)
#define ADC_CMDL_CTYPE_SHIFT                     (5U)
/*! CTYPE - Conversion Type
 *  0b00..Single-Ended Mode. Only A side channel is converted.
 *  0b01-0b11..Reserved.
 */
#define ADC_CMDL_CTYPE(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CMDL_CTYPE_SHIFT)) & ADC_CMDL_CTYPE_MASK)

#define ADC_CMDL_MODE_MASK                       (0x80U)
#define ADC_CMDL_MODE_SHIFT                      (7U)
/*! MODE - Select Resolution of Conversions
 *  0b0..Standard resolution. Single-ended 12-bit conversion.
 *  0b1..High resolution. Single-ended 16-bit conversion.
 */
#define ADC_CMDL_MODE(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CMDL_MODE_SHIFT)) & ADC_CMDL_MODE_MASK)
/*! @} */

/* The count of ADC_CMDL */
#define ADC_CMDL_COUNT                           (7U)

/*! @name CMDH - Command High Buffer Register */
/*! @{ */

#define ADC_CMDH_CMPEN_MASK                      (0x3U)
#define ADC_CMDH_CMPEN_SHIFT                     (0U)
/*! CMPEN - Compare Function Enable
 *  0b00..Compare disabled.
 *  0b01..Reserved
 *  0b10..Compare enabled. Store on true.
 *  0b11..Compare enabled. Repeat channel acquisition (sample/convert/compare) until true.
 */
#define ADC_CMDH_CMPEN(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_CMPEN_SHIFT)) & ADC_CMDH_CMPEN_MASK)

#define ADC_CMDH_WAIT_TRIG_MASK                  (0x4U)
#define ADC_CMDH_WAIT_TRIG_SHIFT                 (2U)
/*! WAIT_TRIG - Wait for Trigger Assertion before Execution.
 *  0b0..This command will be automatically executed.
 *  0b1..The active trigger must be asserted again before executing this command.
 */
#define ADC_CMDH_WAIT_TRIG(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_WAIT_TRIG_SHIFT)) & ADC_CMDH_WAIT_TRIG_MASK)

#define ADC_CMDH_LWI_MASK                        (0x80U)
#define ADC_CMDH_LWI_SHIFT                       (7U)
/*! LWI - Loop with Increment
 *  0b0..Auto channel increment disabled
 *  0b1..Auto channel increment enabled
 */
#define ADC_CMDH_LWI(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_LWI_SHIFT)) & ADC_CMDH_LWI_MASK)

#define ADC_CMDH_STS_MASK                        (0x700U)
#define ADC_CMDH_STS_SHIFT                       (8U)
/*! STS - Sample Time Select
 *  0b000..Minimum sample time of 3.5 ADCK cycles.
 *  0b001..3.5 + 21 ADCK cycles; 5.5 ADCK cycles total sample time.
 *  0b010..3.5 + 22 ADCK cycles; 7.5 ADCK cycles total sample time.
 *  0b011..3.5 + 23 ADCK cycles; 11.5 ADCK cycles total sample time.
 *  0b100..3.5 + 24 ADCK cycles; 19.5 ADCK cycles total sample time.
 *  0b101..3.5 + 25 ADCK cycles; 35.5 ADCK cycles total sample time.
 *  0b110..3.5 + 26 ADCK cycles; 67.5 ADCK cycles total sample time.
 *  0b111..3.5 + 27 ADCK cycles; 131.5 ADCK cycles total sample time.
 */
#define ADC_CMDH_STS(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_STS_SHIFT)) & ADC_CMDH_STS_MASK)

#define ADC_CMDH_AVGS_MASK                       (0xF000U)
#define ADC_CMDH_AVGS_SHIFT                      (12U)
/*! AVGS - Hardware Average Select
 *  0b0000..Single conversion.
 *  0b0001..2 conversions averaged.
 *  0b0010..4 conversions averaged.
 *  0b0011..8 conversions averaged.
 *  0b0100..16 conversions averaged.
 *  0b0101..32 conversions averaged.
 *  0b0110..64 conversions averaged.
 *  0b0111..128 conversions averaged.
 *  0b1000..256 conversions averaged.
 *  0b1001..512 conversions averaged.
 *  0b1010..1024 conversions averaged.
 */
#define ADC_CMDH_AVGS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_AVGS_SHIFT)) & ADC_CMDH_AVGS_MASK)

#define ADC_CMDH_LOOP_MASK                       (0xF0000U)
#define ADC_CMDH_LOOP_SHIFT                      (16U)
/*! LOOP - Loop Count Select
 *  0b0000..Looping not enabled. Command executes 1 time.
 *  0b0001..Loop 1 time. Command executes 2 times.
 *  0b0010..Loop 2 times. Command executes 3 times.
 *  0b0011-0b1110..Loop corresponding number of times. Command executes LOOP+1 times.
 *  0b1111..Loop 15 times. Command executes 16 times.
 */
#define ADC_CMDH_LOOP(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_LOOP_SHIFT)) & ADC_CMDH_LOOP_MASK)

#define ADC_CMDH_NEXT_MASK                       (0x7000000U)
#define ADC_CMDH_NEXT_SHIFT                      (24U)
/*! NEXT - Next Command Select
 *  0b000..No next command defined. Terminate conversions at completion of current command. If lower priority
 *         trigger pending, begin command associated with lower priority trigger.
 *  0b001..Select CMD1 command buffer register as next command.
 *  0b010-0b110..Select corresponding CMD command buffer register as next command
 *  0b111..Select CMD7 command buffer register as next command.
 */
#define ADC_CMDH_NEXT(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CMDH_NEXT_SHIFT)) & ADC_CMDH_NEXT_MASK)
/*! @} */

/* The count of ADC_CMDH */
#define ADC_CMDH_COUNT                           (7U)

/*! @name CV - Compare Value Register */
/*! @{ */

#define ADC_CV_CVL_MASK                          (0xFFFFU)
#define ADC_CV_CVL_SHIFT                         (0U)
/*! CVL - Compare Value Low */
#define ADC_CV_CVL(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_CV_CVL_SHIFT)) & ADC_CV_CVL_MASK)

#define ADC_CV_CVH_MASK                          (0xFFFF0000U)
#define ADC_CV_CVH_SHIFT                         (16U)
/*! CVH - Compare Value High */
#define ADC_CV_CVH(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_CV_CVH_SHIFT)) & ADC_CV_CVH_MASK)
/*! @} */

/*! @name RESFIFO - Data Result FIFO Register */
/*! @{ */

#define ADC_RESFIFO_D_MASK                       (0xFFFFU)
#define ADC_RESFIFO_D_SHIFT                      (0U)
/*! D - Data Result */
#define ADC_RESFIFO_D(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_RESFIFO_D_SHIFT)) & ADC_RESFIFO_D_MASK)

#define ADC_RESFIFO_TSRC_MASK                    (0x30000U)
#define ADC_RESFIFO_TSRC_SHIFT                   (16U)
/*! TSRC - Trigger Source
 *  0b00..Trigger source 0 initiated this conversion.
 *  0b01..Trigger source 1 initiated this conversion.
 *  0b10-0b10..Corresponding trigger source initiated this conversion.
 *  0b11..Trigger source 3 initiated this conversion.
 */
#define ADC_RESFIFO_TSRC(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_RESFIFO_TSRC_SHIFT)) & ADC_RESFIFO_TSRC_MASK)

#define ADC_RESFIFO_LOOPCNT_MASK                 (0xF00000U)
#define ADC_RESFIFO_LOOPCNT_SHIFT                (20U)
/*! LOOPCNT - Loop Count Value
 *  0b0000..Result is from initial conversion in command.
 *  0b0001..Result is from second conversion in command.
 *  0b0010-0b1110..Result is from LOOPCNT+1 conversion in command.
 *  0b1111..Result is from 16th conversion in command.
 */
#define ADC_RESFIFO_LOOPCNT(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_RESFIFO_LOOPCNT_SHIFT)) & ADC_RESFIFO_LOOPCNT_MASK)

#define ADC_RESFIFO_CMDSRC_MASK                  (0x7000000U)
#define ADC_RESFIFO_CMDSRC_SHIFT                 (24U)
/*! CMDSRC - Command Buffer Source
 *  0b000..Not a valid value CMDSRC value for a dataword in RESFIFO. 0x0 is only found in initial FIFO state prior
 *         to an ADC conversion result dataword being stored to a RESFIFO buffer.
 *  0b001..CMD1 buffer used as control settings for this conversion.
 *  0b010-0b110..Corresponding command buffer used as control settings for this conversion.
 *  0b111..CMD7 buffer used as control settings for this conversion.
 */
#define ADC_RESFIFO_CMDSRC(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_RESFIFO_CMDSRC_SHIFT)) & ADC_RESFIFO_CMDSRC_MASK)

#define ADC_RESFIFO_VALID_MASK                   (0x80000000U)
#define ADC_RESFIFO_VALID_SHIFT                  (31U)
/*! VALID - FIFO Entry is Valid
 *  0b0..FIFO is empty. Discard any read from RESFIFO.
 *  0b1..FIFO record read from RESFIFO is valid.
 */
#define ADC_RESFIFO_VALID(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_RESFIFO_VALID_SHIFT)) & ADC_RESFIFO_VALID_MASK)
/*! @} */

/*! @name CAL_GAR - Calibration General A-Side Registers */
/*! @{ */

#define ADC_CAL_GAR_CAL_GAR_VAL_MASK             (0xFFFFU)
#define ADC_CAL_GAR_CAL_GAR_VAL_SHIFT            (0U)
/*! CAL_GAR_VAL - Calibration General A Side Register Element */
#define ADC_CAL_GAR_CAL_GAR_VAL(x)               (((uint32_t)(((uint32_t)(x)) << ADC_CAL_GAR_CAL_GAR_VAL_SHIFT)) & ADC_CAL_GAR_CAL_GAR_VAL_MASK)
/*! @} */

/*! @name CFG2 - Configuration 2 Register */
/*! @{ */

#define ADC_CFG2_JLEFT_MASK                      (0x100U)
#define ADC_CFG2_JLEFT_SHIFT                     (8U)
/*! JLEFT - Justified Left Enable register */
#define ADC_CFG2_JLEFT(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_JLEFT_SHIFT)) & ADC_CFG2_JLEFT_MASK)

#define ADC_CFG2_HS_MASK                         (0x200U)
#define ADC_CFG2_HS_SHIFT                        (9U)
/*! HS - High Speed Enable register
 *  0b0..High speed conversion mode disabled
 *  0b1..High speed conversion mode enabled
 */
#define ADC_CFG2_HS(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_HS_SHIFT)) & ADC_CFG2_HS_MASK)

#define ADC_CFG2_HSEXTRA_MASK                    (0x400U)
#define ADC_CFG2_HSEXTRA_SHIFT                   (10U)
/*! HSEXTRA - High Speed Extra register
 *  0b0..No extra cycle added
 *  0b1..Extra cycle added
 */
#define ADC_CFG2_HSEXTRA(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_HSEXTRA_SHIFT)) & ADC_CFG2_HSEXTRA_MASK)

#define ADC_CFG2_TUNE_MASK                       (0x3000U)
#define ADC_CFG2_TUNE_SHIFT                      (12U)
/*! TUNE - Tune Mode register */
#define ADC_CFG2_TUNE(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_TUNE_SHIFT)) & ADC_CFG2_TUNE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group ADC_Register_Masks */

/* Backward compatibility */
#define ADC_CTRL_RSTFIFO_MASK                     ADC_CTRL_RSTFIFO0_MASK
#define ADC_CTRL_RSTFIFO_SHIFT                    ADC_CTRL_RSTFIFO0_SHIFT
#define ADC_CTRL_RSTFIFO(x)                       ADC_CTRL_RSTFIFO0(x)
#define ADC_STAT_RDY_MASK                         ADC_STAT_RDY0_MASK
#define ADC_STAT_RDY_SHIFT                        ADC_STAT_RDY0_SHIFT
#define ADC_STAT_RDY(x)                           ADC_STAT_RDY0(x)
#define ADC_STAT_FOF_MASK                         ADC_STAT_FOF0_MASK
#define ADC_STAT_FOF_SHIFT                        ADC_STAT_FOF0_SHIFT
#define ADC_STAT_FOF(x)                           ADC_STAT_FOF0(x)
#define ADC_IE_FWMIE_MASK                         ADC_IE_FWMIE0_MASK
#define ADC_IE_FWMIE_SHIFT                        ADC_IE_FWMIE0_SHIFT
#define ADC_IE_FWMIE(x)                           ADC_IE_FWMIE0(x)
#define ADC_IE_FOFIE_MASK                         ADC_IE_FOFIE0_MASK
#define ADC_IE_FOFIE_SHIFT                        ADC_IE_FOFIE0_SHIFT
#define ADC_IE_FOFIE(x)                           ADC_IE_FOFIE0(x)
#define ADC_DE_FWMDE_MASK                         ADC_DE_FWMDE0_MASK
#define ADC_DE_FWMDE_SHIFT                        ADC_DE_FWMDE0_SHIFT
#define ADC_DE_FWMDE(x)                           ADC_DE_FWMDE0(x)


/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


#endif  /* PERI_ADC_H_ */

