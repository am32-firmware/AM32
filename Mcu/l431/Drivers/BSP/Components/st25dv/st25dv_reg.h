/**
  ******************************************************************************
  * @file    st25dv_reg.h
  * @author  MMY Application Team
  * @brief   This file provides set of functions to access st25dv-i2c registers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */ 
#ifndef ST25DV_REG_H
#define ST25DV_REG_H

#include <cmsis_compiler.h>

/* Registers i2c address */
/** @brief ST25DV GPO register address. */
#define ST25DV_GPO_REG                       0x0000
/** @brief ST25DV IT duration register address. */
#define ST25DV_ITTIME_REG                    0x0001
/** @brief ST25DV Energy Harvesting register address. */
#define ST25DV_EH_MODE_REG                   0x0002
/** @brief ST25DV RF management register address. */
#define ST25DV_RF_MNGT_REG                   0x0003
/** @brief ST25DV Area 1 security register address. */
#define ST25DV_RFA1SS_REG                    0x0004
/** @brief ST25DV Area 1 end address register address. */
#define ST25DV_ENDA1_REG                      0x0005
/** @brief ST25DV Area 2 security register address. */
#define ST25DV_RFA2SS_REG                    0x0006
/** @brief ST25DV Area 2 end address register address. */
#define ST25DV_ENDA2_REG                      0x0007
/** @brief ST25DV Area 3 security register address. */
#define ST25DV_RFA3SS_REG                    0x0008
/** @brief ST25DV Area 3 end address register address. */
#define ST25DV_ENDA3_REG                      0x0009
/** @brief ST25DV Area 4 security register address. */
#define ST25DV_RFA4SS_REG                    0x000A
/** @brief ST25DV I2C security register address. */
#define ST25DV_I2CSS_REG                    0x000B
/** @brief ST25DV Capability Container lock register address. */
#define ST25DV_LOCKCCFILE_REG                0x000C
/** @brief ST25DV Mailbox mode register address. */
#define ST25DV_MB_MODE_REG                   0x000D
/** @brief ST25DV Mailbox Watchdog register address. */
#define ST25DV_MB_WDG_REG                    0x000E
/** @brief ST25DV Configuration lock register address. */
#define ST25DV_LOCKCFG_REG                   0x000F
/** @brief ST25DV DSFID lock register address. */
#define ST25DV_LOCKDSFID_REG                 0x0010
/** @brief ST25DV AFI lock register address. */
#define ST25DV_LOCKAFI_REG                   0x0011
/** @brief ST25DV DSFID register address. */
#define ST25DV_DSFID_REG                     0x0012
/** @brief ST25DV AFI register address. */
#define ST25DV_AFI_REG                       0x0013
/** @brief ST25DV Memory size register address. */
#define ST25DV_MEM_SIZE_LSB_REG              0x0014
/** @brief ST25DV Memory size register address. */
#define ST25DV_MEM_SIZE_MSB_REG              0x0015
/** @brief ST25DV Block size register address. */
#define ST25DV_BLK_SIZE_REG                  0x0016
//** @brief ST25DV ICref register address. */
#define ST25DV_ICREF_REG                     0x0017
/** @brief ST25DV UID register address. */
#define ST25DV_UID_REG                       0x0018
/** @brief ST25DV IC revision register address. */
#define ST25DV_ICREV_REG                     0x0020
/** @brief ST25DV I2C password register address. */
#define ST25DV_I2CPASSWD_REG                 0x0900

/* Dynamic Registers i2c address */
/** @brief ST25DV GPO dynamic register address. */
#define ST25DV_GPO_DYN_REG                   0x2000
/** @brief ST25DV Energy Harvesting control dynamic register address. */
#define ST25DV_EH_CTRL_DYN_REG               0x2002
/** @brief ST25DV RF management dynamic register address. */
#define ST25DV_RF_MNGT_DYN_REG               0x2003
/** @brief ST25DV I2C secure session opened dynamic register address. */
#define ST25DV_I2C_SSO_DYN_REG               0x2004
/** @brief ST25DV Interrupt status dynamic register address. */
#define ST25DV_ITSTS_DYN_REG                 0x2005
/** @brief ST25DV Mailbox control dynamic register address. */
#define ST25DV_MB_CTRL_DYN_REG               0x2006
/** @brief ST25DV Mailbox message length dynamic register address. */
#define ST25DV_MBLEN_DYN_REG                 0x2007
/** @brief ST25DV Mailbox buffer address. */
#define ST25DV_MAILBOX_RAM_REG               0x2008

/* Registers fields definitions */
/* MB_MODE */
#define ST25DV_MB_MODE_RW_SHIFT              (0)
#define ST25DV_MB_MODE_RW_FIELD              0xFE
#define ST25DV_MB_MODE_RW_MASK               0x01

/* MB_LEN_Dyn */
#define ST25DV_MBLEN_DYN_MBLEN_SHIFT         (0)
#define ST25DV_MBLEN_DYN_MBLEN_FIELD         0x00
#define ST25DV_MBLEN_DYN_MBLEN_MASK          0xFF

/* MB_CTRL_Dyn */
#define ST25DV_MB_CTRL_DYN_MBEN_SHIFT        (0)
#define ST25DV_MB_CTRL_DYN_MBEN_FIELD        0xFE
#define ST25DV_MB_CTRL_DYN_MBEN_MASK         0x01
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_SHIFT  (1)
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_FIELD  0xFD
#define ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK   0x02
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_SHIFT    (2)
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_FIELD    0xFB
#define ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK     0x04
#define ST25DV_MB_CTRL_DYN_STRESERVED_SHIFT  (3)
#define ST25DV_MB_CTRL_DYN_STRESERVED_FIELD  0xF7
#define ST25DV_MB_CTRL_DYN_STRESERVED_MASK   0x08
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_SHIFT (4)
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_FIELD 0xEF
#define ST25DV_MB_CTRL_DYN_HOSTMISSMSG_MASK  0x10
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_SHIFT   (5)
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_FIELD   0xDF
#define ST25DV_MB_CTRL_DYN_RFMISSMSG_MASK    0x20
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_SHIFT  (6)
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_FIELD  0x3F
#define ST25DV_MB_CTRL_DYN_CURRENTMSG_MASK   0xC0


/* MB_WDG */
#define ST25DV_MB_WDG_DELAY_SHIFT            (0)
#define ST25DV_MB_WDG_DELAY_FIELD            0xF8
#define ST25DV_MB_WDG_DELAY_MASK             0x07

/* GPO */
#define ST25DV_GPO_RFUSERSTATE_SHIFT         (0)
#define ST25DV_GPO_RFUSERSTATE_FIELD         0xFE
#define ST25DV_GPO_RFUSERSTATE_MASK          0x01
#define ST25DV_GPO_RFACTIVITY_SHIFT          (1)
#define ST25DV_GPO_RFACTIVITY_FIELD          0xFD
#define ST25DV_GPO_RFACTIVITY_MASK           0x02
#define ST25DV_GPO_RFINTERRUPT_SHIFT         (2)
#define ST25DV_GPO_RFINTERRUPT_FIELD         0xFB
#define ST25DV_GPO_RFINTERRUPT_MASK          0x04
#define ST25DV_GPO_FIELDCHANGE_SHIFT         (3)
#define ST25DV_GPO_FIELDCHANGE_FIELD         0xF7
#define ST25DV_GPO_FIELDCHANGE_MASK          0x08
#define ST25DV_GPO_RFPUTMSG_SHIFT            (4)
#define ST25DV_GPO_RFPUTMSG_FIELD            0xEF
#define ST25DV_GPO_RFPUTMSG_MASK             0x10
#define ST25DV_GPO_RFGETMSG_SHIFT            (5)
#define ST25DV_GPO_RFGETMSG_FIELD            0xDF
#define ST25DV_GPO_RFGETMSG_MASK             0x20
#define ST25DV_GPO_RFWRITE_SHIFT             (6)
#define ST25DV_GPO_RFWRITE_FIELD             0xBF
#define ST25DV_GPO_RFWRITE_MASK              0x40
#define ST25DV_GPO_ENABLE_SHIFT              (7)
#define ST25DV_GPO_ENABLE_FIELD              0x7F
#define ST25DV_GPO_ENABLE_MASK               0x80
#define ST25DV_GPO_ALL_SHIFT                 (0)
#define ST25DV_GPO_ALL_MASK                  0xFF

/* GPO_Dyn */
#define ST25DV_GPO_DYN_RFUSERSTATE_SHIFT     (0)
#define ST25DV_GPO_DYN_RFUSERSTATE_FIELD     0xFE
#define ST25DV_GPO_DYN_RFUSERSTATE_MASK      0x01
#define ST25DV_GPO_DYN_RFACTIVITY_SHIFT      (1)
#define ST25DV_GPO_DYN_RFACTIVITY_FIELD      0xFD
#define ST25DV_GPO_DYN_RFACTIVITY_MASK       0x02
#define ST25DV_GPO_DYN_RFINTERRUPT_SHIFT     (2)
#define ST25DV_GPO_DYN_RFINTERRUPT_FIELD     0xFB
#define ST25DV_GPO_DYN_RFINTERRUPT_MASK      0x04
#define ST25DV_GPO_DYN_FIELDCHANGE_SHIFT     (3)
#define ST25DV_GPO_DYN_FIELDCHANGE_FIELD     0xF7
#define ST25DV_GPO_DYN_FIELDCHANGE_MASK      0x08
#define ST25DV_GPO_DYN_RFPUTMSG_SHIFT        (4)
#define ST25DV_GPO_DYN_RFPUTMSG_FIELD        0xEF
#define ST25DV_GPO_DYN_RFPUTMSG_MASK         0x10
#define ST25DV_GPO_DYN_RFGETMSG_SHIFT        (5)
#define ST25DV_GPO_DYN_RFGETMSG_FIELD        0xDF
#define ST25DV_GPO_DYN_RFGETMSG_MASK         0x20
#define ST25DV_GPO_DYN_RFWRITE_SHIFT         (6)
#define ST25DV_GPO_DYN_RFWRITE_FIELD         0xBF
#define ST25DV_GPO_DYN_RFWRITE_MASK          0x40
#define ST25DV_GPO_DYN_ENABLE_SHIFT          (7)
#define ST25DV_GPO_DYN_ENABLE_FIELD          0x7F
#define ST25DV_GPO_DYN_ENABLE_MASK           0x80
#define ST25DV_GPO_DYN_ALL_SHIFT             (0)
#define ST25DV_GPO_DYN_ALL_MASK              0xFF

/* ITTIME */
#define ST25DV_ITTIME_DELAY_SHIFT            (0)
#define ST25DV_ITTIME_DELAY_FIELD            0xFC
#define ST25DV_ITTIME_DELAY_MASK             0x03

/* ITSTS_Dyn */
#define ST25DV_ITSTS_DYN_RFUSERSTATE_SHIFT       (0)
#define ST25DV_ITSTS_DYN_RFUSERSTATE_FIELD       0xFE
#define ST25DV_ITSTS_DYN_RFUSERSTATE_MASK        0x01
#define ST25DV_ITSTS_DYN_RFACTIVITY_SHIFT        (1)
#define ST25DV_ITSTS_DYN_RFACTIVITY_FIELD        0xFD
#define ST25DV_ITSTS_DYN_RFACTIVITY_MASK         0x02
#define ST25DV_ITSTS_DYN_RFINTERRUPT_SHIFT       (2)
#define ST25DV_ITSTS_DYN_RFINTERRUPT_FIELD       0xFB
#define ST25DV_ITSTS_DYN_RFINTERRUPT_MASK        0x04
#define ST25DV_ITSTS_DYN_FIELDFALLING_SHIFT      (3)
#define ST25DV_ITSTS_DYN_FIELDFALLING_FIELD      0xF7
#define ST25DV_ITSTS_DYN_FIELDFALLING_MASK       0x08
#define ST25DV_ITSTS_DYN_FIELDRISING_SHIFT       (4)
#define ST25DV_ITSTS_DYN_FIELDRISING_FIELD       0xEF
#define ST25DV_ITSTS_DYN_FIELDRISING_MASK        0x10
#define ST25DV_ITSTS_DYN_RFPUTMSG_SHIFT          (5)
#define ST25DV_ITSTS_DYN_RFPUTMSG_FIELD          0xDF
#define ST25DV_ITSTS_DYN_RFPUTMSG_MASK           0x20
#define ST25DV_ITSTS_DYN_RFGETMSG_SHIFT          (6)
#define ST25DV_ITSTS_DYN_RFGETMSG_FIELD          0xBF
#define ST25DV_ITSTS_DYN_RFGETMSG_MASK           0x40
#define ST25DV_ITSTS_DYN_RFWRITE_SHIFT           (7)
#define ST25DV_ITSTS_DYN_RFWRITE_FIELD           0x7F
#define ST25DV_ITSTS_DYN_RFWRITE_MASK            0x80

/* EH_MODE */
#define ST25DV_EH_MODE_SHIFT                 (0)
#define ST25DV_EH_MODE_FIELD                 0xFE
#define ST25DV_EH_MODE_MASK                  0x01

/* EH_CTRL_Dyn */
#define ST25DV_EH_CTRL_DYN_EH_EN_SHIFT       (0)
#define ST25DV_EH_CTRL_DYN_EH_EN_FIELD       0xFE
#define ST25DV_EH_CTRL_DYN_EH_EN_MASK        0x01
#define ST25DV_EH_CTRL_DYN_EH_ON_SHIFT       (1)
#define ST25DV_EH_CTRL_DYN_EH_ON_FIELD       0xFD
#define ST25DV_EH_CTRL_DYN_EH_ON_MASK        0x02
#define ST25DV_EH_CTRL_DYN_FIELD_ON_SHIFT    (2)
#define ST25DV_EH_CTRL_DYN_FIELD_ON_FIELD    0xFB
#define ST25DV_EH_CTRL_DYN_FIELD_ON_MASK     0x04
#define ST25DV_EH_CTRL_DYN_VCC_ON_SHIFT      (3)
#define ST25DV_EH_CTRL_DYN_VCC_ON_FIELD      0xF7
#define ST25DV_EH_CTRL_DYN_VCC_ON_MASK       0x08
#define ST25DV_EH_CTRL_DYN_ALL_SHIFT         (0)
#define ST25DV_EH_CTRL_DYN_ALL_MASK          0x0F

/* RF_MNGT */
#define ST25DV_RF_MNGT_RFDIS_SHIFT           (0)
#define ST25DV_RF_MNGT_RFDIS_FIELD           0xFE
#define ST25DV_RF_MNGT_RFDIS_MASK            0x01
#define ST25DV_RF_MNGT_RFSLEEP_SHIFT         (1)
#define ST25DV_RF_MNGT_RFSLEEP_FIELD         0xFD
#define ST25DV_RF_MNGT_RFSLEEP_MASK          0x02
#define ST25DV_RF_MNGT_ALL_SHIFT             (0)
#define ST25DV_RF_MNGT_ALL_MASK              0x03

/* RF_MNGT_Dyn */
#define ST25DV_RF_MNGT_DYN_RFDIS_SHIFT       (0)
#define ST25DV_RF_MNGT_DYN_RFDIS_FIELD       0xFE
#define ST25DV_RF_MNGT_DYN_RFDIS_MASK        0x01
#define ST25DV_RF_MNGT_DYN_RFSLEEP_SHIFT     (1)
#define ST25DV_RF_MNGT_DYN_RFSLEEP_FIELD     0xFD
#define ST25DV_RF_MNGT_DYN_RFSLEEP_MASK      0x02
#define ST25DV_RF_MNGT_DYN_ALL_SHIFT         (0)
#define ST25DV_RF_MNGT_DYN_ALL_MASK          0x03

/* RFA1SS */
#define ST25DV_RFA1SS_PWDCTRL_SHIFT           (0)
#define ST25DV_RFA1SS_PWDCTRL_FIELD           0xFC
#define ST25DV_RFA1SS_PWDCTRL_MASK            0x03
#define ST25DV_RFA1SS_RWPROT_SHIFT            (2)
#define ST25DV_RFA1SS_RWPROT_FIELD            0xF3
#define ST25DV_RFA1SS_RWPROT_MASK             0x0C
#define ST25DV_RFA1SS_ALL_SHIFT               (0)
#define ST25DV_RFA1SS_ALL_MASK                0x0F

/* RFA2SS */
#define ST25DV_RFA2SS_PWDCTRL_SHIFT           (0)
#define ST25DV_RFA2SS_PWDCTRL_FIELD           0xFC
#define ST25DV_RFA2SS_PWDCTRL_MASK            0x03
#define ST25DV_RFA2SS_RWPROT_SHIFT            (2)
#define ST25DV_RFA2SS_RWPROT_FIELD            0xF3
#define ST25DV_RFA2SS_RWPROT_MASK             0x0C
#define ST25DV_RFA2SS_ALL_SHIFT               (0)
#define ST25DV_RFA2SS_ALL_MASK                0x0F

/* RFA3SS */
#define ST25DV_RFA3SS_PWDCTRL_SHIFT           (0)
#define ST25DV_RFA3SS_PWDCTRL_FIELD           0xFC
#define ST25DV_RFA3SS_PWDCTRL_MASK            0x03
#define ST25DV_RFA3SS_RWPROT_SHIFT            (2)
#define ST25DV_RFA3SS_RWPROT_FIELD            0xF3
#define ST25DV_RFA3SS_RWPROT_MASK             0x0C
#define ST25DV_RFA3SS_ALL_SHIFT               (0)
#define ST25DV_RFA3SS_ALL_MASK                0x0F

/* RFA4SS */
#define ST25DV_RFA4SS_PWDCTRL_SHIFT           (0)
#define ST25DV_RFA4SS_PWDCTRL_FIELD           0xFC
#define ST25DV_RFA4SS_PWDCTRL_MASK            0x03
#define ST25DV_RFA4SS_RWPROT_SHIFT            (2)
#define ST25DV_RFA4SS_RWPROT_FIELD            0xF3
#define ST25DV_RFA4SS_RWPROT_MASK             0x0C
#define ST25DV_RFA4SS_ALL_SHIFT               (0)
#define ST25DV_RFA4SS_ALL_MASK                0x0F

/* I2CSS */
#define ST25DV_I2CSS_PZ1_SHIFT              (0)
#define ST25DV_I2CSS_PZ1_FIELD              0xFC
#define ST25DV_I2CSS_PZ1_MASK               0x03
#define ST25DV_I2CSS_PZ2_SHIFT              (2)
#define ST25DV_I2CSS_PZ2_FIELD              0xF3
#define ST25DV_I2CSS_PZ2_MASK               0x0C
#define ST25DV_I2CSS_PZ3_SHIFT              (4)
#define ST25DV_I2CSS_PZ3_FIELD              0xCF
#define ST25DV_I2CSS_PZ3_MASK               0x30
#define ST25DV_I2CSS_PZ4_SHIFT              (6)
#define ST25DV_I2CSS_PZ4_FIELD              0x3F
#define ST25DV_I2CSS_PZ4_MASK               0xC0

/* LOCKCCFILE */
#define ST25DV_LOCKCCFILE_BLCK0_SHIFT        (0)
#define ST25DV_LOCKCCFILE_BLCK0_FIELD        0xFE
#define ST25DV_LOCKCCFILE_BLCK0_MASK         0x01
#define ST25DV_LOCKCCFILE_BLCK1_SHIFT        (1)
#define ST25DV_LOCKCCFILE_BLCK1_FIELD        0xFD
#define ST25DV_LOCKCCFILE_BLCK1_MASK         0x02
#define ST25DV_LOCKCCFILE_ALL_SHIFT          (0)
#define ST25DV_LOCKCCFILE_ALL_MASK           0x03

/* LOCKCFG */
#define ST25DV_LOCKCFG_B0_SHIFT              (0)
#define ST25DV_LOCKCFG_B0_FIELD              0xFE
#define ST25DV_LOCKCFG_B0_MASK               0x01

/* I2C_SSO_Dyn */
#define ST25DV_I2C_SSO_DYN_I2CSSO_SHIFT      (0)
#define ST25DV_I2C_SSO_DYN_I2CSSO_FIELD      0xFE
#define ST25DV_I2C_SSO_DYN_I2CSSO_MASK       0x01

/**
 * @brief  ST25DV status enumerator definition.
 */
#define ST25DV_OK      (0)
#define ST25DV_ERROR   (-1)
#define ST25DV_BUSY    (-2)
#define ST25DV_TIMEOUT (-3)
#define ST25DV_NACK    (-102)


typedef int32_t (*ST25DV_WriteReg_Func)(void *, uint16_t, const uint8_t*, uint16_t);
typedef int32_t (*ST25DV_ReadReg_Func) (void *, uint16_t, uint8_t*, uint16_t);

typedef struct {
  ST25DV_WriteReg_Func WriteReg;
  ST25DV_ReadReg_Func ReadReg;
  void *handle;
} st25dv_ctx_t;

int32_t st25dv_get_icref (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_enda1 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_enda1 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_enda2 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_enda2 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_enda3 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_enda3 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_dsfid (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_afi (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mem_size_msb (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_blk_size (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mem_size_lsb (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_icrev (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_uid (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_i2cpasswd (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2cpasswd (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_lockdsfid (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_lockafi (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_mode_rw (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_mb_mode_rw (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_mblen_dyn_mblen (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_mben (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_mb_ctrl_dyn_mben (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_hostputmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_streserved (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_hostmissmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_rfmissmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_currentmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_ctrl_dyn_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_mb_wdg_delay (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_mb_wdg_delay (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfuserstate (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfactivity (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfactivity (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfinterrupt (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_fieldchange (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_fieldchange (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfputmsg (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfgetmsg (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_rfwrite (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_rfwrite (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_enable (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_enable (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfuserstate (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfactivity (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfactivity (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfinterrupt (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_fieldchange (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_fieldchang (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfputmsg (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfgetmsg (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_rfwrite (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_rfwrite (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_enable (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_enable (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_gpo_dyn_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_gpo_dyn_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_ittime_delay (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_ittime_delay (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfactivity (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_fieldfalling (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_fieldrising (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_rfwrite (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_itsts_dyn_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_eh_mode (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_eh_mode (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_eh_ctrl_dyn_eh_en (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_eh_ctrl_dyn_eh_en (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_eh_ctrl_dyn_eh_on (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_eh_ctrl_dyn_field_on (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_eh_ctrl_dyn_vcc_on (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_eh_ctrl_dyn_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_get_rf_mngt_rfdis (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_rfdis (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rf_mngt_rfsleep (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_rfsleep (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rf_mngt_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rf_mngt_dyn_rfdis (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_dyn_rfdis (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rf_mngt_dyn_rfsleep (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_dyn_rfsleep (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rf_mngt_dyn_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rf_mngt_dyn_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa1ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa1ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa1ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa1ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa1ss_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa1ss_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa2ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa2ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa2ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa2ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa2ss_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa2ss_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa3ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa3ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa3ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa3ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa3ss_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa3ss_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa4ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa4ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa4ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa4ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_rfa4ss_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_rfa4ss_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2css_pz1 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2css_pz1 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2css_pz2 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2css_pz2 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2css_pz3 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2css_pz3 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2css_pz4 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2css_pz4 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2css_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_i2css_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_lockccfile_blck0 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_lockccfile_blck0 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_lockccfile_blck1 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_lockccfile_blck1 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_lockccfile_all (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_lockccfile_all (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_lockcfg_b0 (st25dv_ctx_t *ctx, uint8_t *value);
int32_t st25dv_set_lockcfg_b0 (st25dv_ctx_t *ctx, const uint8_t *value);
int32_t st25dv_get_i2c_sso_dyn_i2csso (st25dv_ctx_t *ctx, uint8_t *value);

#endif
