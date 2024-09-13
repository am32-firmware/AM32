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

#include "st25dv_reg.h"

int32_t st25dv_readreg (st25dv_ctx_t *ctx, uint16_t reg, uint8_t* data, uint16_t len)
{
  return ctx->ReadReg(ctx->handle, reg, data, len);
}

int32_t st25dv_WriteReg (st25dv_ctx_t *ctx, uint16_t reg, uint8_t const *data, uint16_t len)
{
  return ctx->WriteReg(ctx->handle, reg, data, len);
}


/**** Copy generated code hereafter ****/
int32_t st25dv_get_icref (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ICREF_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_enda1 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ENDA1_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_enda1 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  return st25dv_WriteReg(ctx, (ST25DV_ENDA1_REG), value, 1);
}

int32_t st25dv_get_enda2 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ENDA2_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_enda2 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  return st25dv_WriteReg(ctx, (ST25DV_ENDA2_REG), value, 1);
}

int32_t st25dv_get_enda3 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ENDA3_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_enda3 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  return st25dv_WriteReg(ctx, (ST25DV_ENDA3_REG), value, 1);
}

int32_t st25dv_get_dsfid (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_DSFID_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_afi (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_AFI_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mem_size_msb (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MEM_SIZE_MSB_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_blk_size (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_BLK_SIZE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mem_size_lsb (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MEM_SIZE_LSB_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_icrev (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ICREV_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_uid (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_UID_REG), (uint8_t *)value, 8))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2cpasswd (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CPASSWD_REG), (uint8_t *)value, 8))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_i2cpasswd (st25dv_ctx_t *ctx, const uint8_t *value)
{
  if( st25dv_WriteReg(ctx, (ST25DV_I2CPASSWD_REG), value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockdsfid (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKDSFID_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockafi (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKAFI_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mb_mode_rw (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_MODE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_MODE_RW_MASK);
  *value = *value >> (ST25DV_MB_MODE_RW_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_mb_mode_rw (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_MB_MODE_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_MB_MODE_RW_SHIFT)) & (ST25DV_MB_MODE_RW_MASK)) |
                (reg_value & ~(ST25DV_MB_MODE_RW_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_MB_MODE_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mblen_dyn_mblen (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MBLEN_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_mben (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_MBEN_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_MBEN_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_mb_ctrl_dyn_mben (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_MB_CTRL_DYN_MBEN_SHIFT)) & (ST25DV_MB_CTRL_DYN_MBEN_MASK)) |
                (reg_value & ~(ST25DV_MB_CTRL_DYN_MBEN_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_MB_CTRL_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_hostputmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_HOSTPUTMSG_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_HOSTPUTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_RFPUTMSG_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_RFPUTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_streserved (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_STRESERVED_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_STRESERVED_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_hostmissmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_HOSTMISSMSG_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_HOSTMISSMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_rfmissmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_RFMISSMSG_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_RFMISSMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_currentmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_CTRL_DYN_CURRENTMSG_MASK);
  *value = *value >> (ST25DV_MB_CTRL_DYN_CURRENTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_mb_ctrl_dyn_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_mb_wdg_delay (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_MB_WDG_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_MB_WDG_DELAY_MASK);
  *value = *value >> (ST25DV_MB_WDG_DELAY_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_mb_wdg_delay (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_MB_WDG_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_MB_WDG_DELAY_SHIFT)) & (ST25DV_MB_WDG_DELAY_MASK)) |
                (reg_value & ~(ST25DV_MB_WDG_DELAY_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_MB_WDG_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFUSERSTATE_MASK);
  *value = *value >> (ST25DV_GPO_RFUSERSTATE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfuserstate (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFUSERSTATE_SHIFT)) & (ST25DV_GPO_RFUSERSTATE_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFUSERSTATE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfactivity (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFACTIVITY_MASK);
  *value = *value >> (ST25DV_GPO_RFACTIVITY_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfactivity (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFACTIVITY_SHIFT)) & (ST25DV_GPO_RFACTIVITY_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFACTIVITY_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFINTERRUPT_MASK);
  *value = *value >> (ST25DV_GPO_RFINTERRUPT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfinterrupt (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFINTERRUPT_SHIFT)) & (ST25DV_GPO_RFINTERRUPT_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFINTERRUPT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_fieldchange (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_FIELDCHANGE_MASK);
  *value = *value >> (ST25DV_GPO_FIELDCHANGE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_fieldchange (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_FIELDCHANGE_SHIFT)) & (ST25DV_GPO_FIELDCHANGE_MASK)) |
                (reg_value & ~(ST25DV_GPO_FIELDCHANGE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFPUTMSG_MASK);
  *value = *value >> (ST25DV_GPO_RFPUTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfputmsg (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFPUTMSG_SHIFT)) & (ST25DV_GPO_RFPUTMSG_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFPUTMSG_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFGETMSG_MASK);
  *value = *value >> (ST25DV_GPO_RFGETMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfgetmsg (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFGETMSG_SHIFT)) & (ST25DV_GPO_RFGETMSG_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFGETMSG_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_rfwrite (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_RFWRITE_MASK);
  *value = *value >> (ST25DV_GPO_RFWRITE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_rfwrite (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_RFWRITE_SHIFT)) & (ST25DV_GPO_RFWRITE_MASK)) |
                (reg_value & ~(ST25DV_GPO_RFWRITE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_enable (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_ENABLE_MASK);
  *value = *value >> (ST25DV_GPO_ENABLE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_enable (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_ENABLE_SHIFT)) & (ST25DV_GPO_ENABLE_MASK)) |
                (reg_value & ~(ST25DV_GPO_ENABLE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  if( st25dv_WriteReg(ctx, (ST25DV_GPO_REG), value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFUSERSTATE_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFUSERSTATE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfuserstate (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFUSERSTATE_SHIFT)) & (ST25DV_GPO_DYN_RFUSERSTATE_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFUSERSTATE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfactivity (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFACTIVITY_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFACTIVITY_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfactivity (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFACTIVITY_SHIFT)) & (ST25DV_GPO_DYN_RFACTIVITY_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFACTIVITY_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFINTERRUPT_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFINTERRUPT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfinterrupt (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFINTERRUPT_SHIFT)) & (ST25DV_GPO_DYN_RFINTERRUPT_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFINTERRUPT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_fieldchange (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_FIELDCHANGE_MASK);
  *value = *value >> (ST25DV_GPO_DYN_FIELDCHANGE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_fieldchange (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_FIELDCHANGE_SHIFT)) & (ST25DV_GPO_DYN_FIELDCHANGE_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_FIELDCHANGE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFPUTMSG_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFPUTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfputmsg (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFPUTMSG_SHIFT)) & (ST25DV_GPO_DYN_RFPUTMSG_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFPUTMSG_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFGETMSG_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFGETMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfgetmsg (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFGETMSG_SHIFT)) & (ST25DV_GPO_DYN_RFGETMSG_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFGETMSG_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_rfwrite (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_RFWRITE_MASK);
  *value = *value >> (ST25DV_GPO_DYN_RFWRITE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_rfwrite (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_RFWRITE_SHIFT)) & (ST25DV_GPO_DYN_RFWRITE_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_RFWRITE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_enable (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_GPO_DYN_ENABLE_MASK);
  *value = *value >> (ST25DV_GPO_DYN_ENABLE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_enable (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_GPO_DYN_ENABLE_SHIFT)) & (ST25DV_GPO_DYN_ENABLE_MASK)) |
                (reg_value & ~(ST25DV_GPO_DYN_ENABLE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_gpo_dyn_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_GPO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_gpo_dyn_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  if( st25dv_WriteReg(ctx, (ST25DV_GPO_DYN_REG), value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_ittime_delay (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITTIME_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITTIME_DELAY_MASK);
  *value = *value >> (ST25DV_ITTIME_DELAY_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_ittime_delay (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_ITTIME_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_ITTIME_DELAY_SHIFT)) & (ST25DV_ITTIME_DELAY_MASK)) |
                (reg_value & ~(ST25DV_ITTIME_DELAY_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_ITTIME_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfuserstate (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFUSERSTATE_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFUSERSTATE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfactivity (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFACTIVITY_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFACTIVITY_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfinterrupt (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFINTERRUPT_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFINTERRUPT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_fieldfalling (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_FIELDFALLING_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_FIELDFALLING_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_fieldrising (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_FIELDRISING_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_FIELDRISING_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfputmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFPUTMSG_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFPUTMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfgetmsg (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFGETMSG_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFGETMSG_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_rfwrite (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_ITSTS_DYN_RFWRITE_MASK);
  *value = *value >> (ST25DV_ITSTS_DYN_RFWRITE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_itsts_dyn_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_ITSTS_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_eh_mode (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_MODE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_MODE_MASK);
  *value = *value >> (ST25DV_EH_MODE_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_eh_mode (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_EH_MODE_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_EH_MODE_SHIFT)) & (ST25DV_EH_MODE_MASK)) |
                (reg_value & ~(ST25DV_EH_MODE_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_EH_MODE_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_eh_ctrl_dyn_eh_en (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_CTRL_DYN_EH_EN_MASK);
  *value = *value >> (ST25DV_EH_CTRL_DYN_EH_EN_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_eh_ctrl_dyn_eh_en (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_EH_CTRL_DYN_EH_EN_SHIFT)) & (ST25DV_EH_CTRL_DYN_EH_EN_MASK)) |
                (reg_value & ~(ST25DV_EH_CTRL_DYN_EH_EN_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_EH_CTRL_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_eh_ctrl_dyn_eh_on (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_CTRL_DYN_EH_ON_MASK);
  *value = *value >> (ST25DV_EH_CTRL_DYN_EH_ON_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_eh_ctrl_dyn_field_on (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_CTRL_DYN_FIELD_ON_MASK);
  *value = *value >> (ST25DV_EH_CTRL_DYN_FIELD_ON_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_eh_ctrl_dyn_vcc_on (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_CTRL_DYN_VCC_ON_MASK);
  *value = *value >> (ST25DV_EH_CTRL_DYN_VCC_ON_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_eh_ctrl_dyn_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_EH_CTRL_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_EH_CTRL_DYN_ALL_MASK);
  *value = *value >> (ST25DV_EH_CTRL_DYN_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_rfdis (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_RFDIS_MASK);
  *value = *value >> (ST25DV_RF_MNGT_RFDIS_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_rfdis (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_RFDIS_SHIFT)) & (ST25DV_RF_MNGT_RFDIS_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_RFDIS_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_rfsleep (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_RFSLEEP_MASK);
  *value = *value >> (ST25DV_RF_MNGT_RFSLEEP_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_rfsleep (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_RFSLEEP_SHIFT)) & (ST25DV_RF_MNGT_RFSLEEP_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_RFSLEEP_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_ALL_MASK);
  *value = *value >> (ST25DV_RF_MNGT_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_ALL_SHIFT)) & (ST25DV_RF_MNGT_ALL_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_dyn_rfdis (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_DYN_RFDIS_MASK);
  *value = *value >> (ST25DV_RF_MNGT_DYN_RFDIS_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_dyn_rfdis (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_DYN_RFDIS_SHIFT)) & (ST25DV_RF_MNGT_DYN_RFDIS_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_DYN_RFDIS_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_dyn_rfsleep (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_DYN_RFSLEEP_MASK);
  *value = *value >> (ST25DV_RF_MNGT_DYN_RFSLEEP_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_dyn_rfsleep (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_DYN_RFSLEEP_SHIFT)) & (ST25DV_RF_MNGT_DYN_RFSLEEP_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_DYN_RFSLEEP_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rf_mngt_dyn_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RF_MNGT_DYN_ALL_MASK);
  *value = *value >> (ST25DV_RF_MNGT_DYN_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rf_mngt_dyn_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RF_MNGT_DYN_ALL_SHIFT)) & (ST25DV_RF_MNGT_DYN_ALL_MASK)) |
                (reg_value & ~(ST25DV_RF_MNGT_DYN_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RF_MNGT_DYN_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa1ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA1SS_PWDCTRL_MASK);
  *value = *value >> (ST25DV_RFA1SS_PWDCTRL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa1ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA1SS_PWDCTRL_SHIFT)) & (ST25DV_RFA1SS_PWDCTRL_MASK)) |
                (reg_value & ~(ST25DV_RFA1SS_PWDCTRL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa1ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA1SS_RWPROT_MASK);
  *value = *value >> (ST25DV_RFA1SS_RWPROT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa1ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA1SS_RWPROT_SHIFT)) & (ST25DV_RFA1SS_RWPROT_MASK)) |
                (reg_value & ~(ST25DV_RFA1SS_RWPROT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa1ss_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA1SS_ALL_MASK);
  *value = *value >> (ST25DV_RFA1SS_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa1ss_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA1SS_ALL_SHIFT)) & (ST25DV_RFA1SS_ALL_MASK)) |
                (reg_value & ~(ST25DV_RFA1SS_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA1SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa2ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA2SS_PWDCTRL_MASK);
  *value = *value >> (ST25DV_RFA2SS_PWDCTRL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa2ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA2SS_PWDCTRL_SHIFT)) & (ST25DV_RFA2SS_PWDCTRL_MASK)) |
                (reg_value & ~(ST25DV_RFA2SS_PWDCTRL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa2ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA2SS_RWPROT_MASK);
  *value = *value >> (ST25DV_RFA2SS_RWPROT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa2ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA2SS_RWPROT_SHIFT)) & (ST25DV_RFA2SS_RWPROT_MASK)) |
                (reg_value & ~(ST25DV_RFA2SS_RWPROT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa2ss_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA2SS_ALL_MASK);
  *value = *value >> (ST25DV_RFA2SS_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa2ss_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA2SS_ALL_SHIFT)) & (ST25DV_RFA2SS_ALL_MASK)) |
                (reg_value & ~(ST25DV_RFA2SS_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA2SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa3ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA3SS_PWDCTRL_MASK);
  *value = *value >> (ST25DV_RFA3SS_PWDCTRL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa3ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA3SS_PWDCTRL_SHIFT)) & (ST25DV_RFA3SS_PWDCTRL_MASK)) |
                (reg_value & ~(ST25DV_RFA3SS_PWDCTRL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa3ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA3SS_RWPROT_MASK);
  *value = *value >> (ST25DV_RFA3SS_RWPROT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa3ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA3SS_RWPROT_SHIFT)) & (ST25DV_RFA3SS_RWPROT_MASK)) |
                (reg_value & ~(ST25DV_RFA3SS_RWPROT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa3ss_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA3SS_ALL_MASK);
  *value = *value >> (ST25DV_RFA3SS_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa3ss_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA3SS_ALL_SHIFT)) & (ST25DV_RFA3SS_ALL_MASK)) |
                (reg_value & ~(ST25DV_RFA3SS_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA3SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa4ss_pwdctrl (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA4SS_PWDCTRL_MASK);
  *value = *value >> (ST25DV_RFA4SS_PWDCTRL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa4ss_pwdctrl (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA4SS_PWDCTRL_SHIFT)) & (ST25DV_RFA4SS_PWDCTRL_MASK)) |
                (reg_value & ~(ST25DV_RFA4SS_PWDCTRL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa4ss_rwprot (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA4SS_RWPROT_MASK);
  *value = *value >> (ST25DV_RFA4SS_RWPROT_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa4ss_rwprot (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA4SS_RWPROT_SHIFT)) & (ST25DV_RFA4SS_RWPROT_MASK)) |
                (reg_value & ~(ST25DV_RFA4SS_RWPROT_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_rfa4ss_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_RFA4SS_ALL_MASK);
  *value = *value >> (ST25DV_RFA4SS_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_rfa4ss_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_RFA4SS_ALL_SHIFT)) & (ST25DV_RFA4SS_ALL_MASK)) |
                (reg_value & ~(ST25DV_RFA4SS_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_RFA4SS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2css_pz1 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_I2CSS_PZ1_MASK);
  *value = *value >> (ST25DV_I2CSS_PZ1_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_i2css_pz1 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_I2CSS_PZ1_SHIFT)) & (ST25DV_I2CSS_PZ1_MASK)) |
                (reg_value & ~(ST25DV_I2CSS_PZ1_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2css_pz2 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_I2CSS_PZ2_MASK);
  *value = *value >> (ST25DV_I2CSS_PZ2_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_i2css_pz2 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_I2CSS_PZ2_SHIFT)) & (ST25DV_I2CSS_PZ2_MASK)) |
                (reg_value & ~(ST25DV_I2CSS_PZ2_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2css_pz3 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_I2CSS_PZ3_MASK);
  *value = *value >> (ST25DV_I2CSS_PZ3_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_i2css_pz3 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_I2CSS_PZ3_SHIFT)) & (ST25DV_I2CSS_PZ3_MASK)) |
                (reg_value & ~(ST25DV_I2CSS_PZ3_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2css_pz4 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_I2CSS_PZ4_MASK);
  *value = *value >> (ST25DV_I2CSS_PZ4_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_i2css_pz4 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_I2CSS_PZ4_SHIFT)) & (ST25DV_I2CSS_PZ4_MASK)) |
                (reg_value & ~(ST25DV_I2CSS_PZ4_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_I2CSS_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2css_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2CSS_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_set_i2css_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  if( st25dv_WriteReg(ctx, (ST25DV_I2CSS_REG), value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockccfile_blck0 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_LOCKCCFILE_BLCK0_MASK);
  *value = *value >> (ST25DV_LOCKCCFILE_BLCK0_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_lockccfile_blck0 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_LOCKCCFILE_BLCK0_SHIFT)) & (ST25DV_LOCKCCFILE_BLCK0_MASK)) |
                (reg_value & ~(ST25DV_LOCKCCFILE_BLCK0_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockccfile_blck1 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_LOCKCCFILE_BLCK1_MASK);
  *value = *value >> (ST25DV_LOCKCCFILE_BLCK1_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_lockccfile_blck1 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_LOCKCCFILE_BLCK1_SHIFT)) & (ST25DV_LOCKCCFILE_BLCK1_MASK)) |
                (reg_value & ~(ST25DV_LOCKCCFILE_BLCK1_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockccfile_all (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_LOCKCCFILE_ALL_MASK);
  *value = *value >> (ST25DV_LOCKCCFILE_ALL_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_lockccfile_all (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_LOCKCCFILE_ALL_SHIFT)) & (ST25DV_LOCKCCFILE_ALL_MASK)) |
                (reg_value & ~(ST25DV_LOCKCCFILE_ALL_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_LOCKCCFILE_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_lockcfg_b0 (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_LOCKCFG_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_LOCKCFG_B0_MASK);
  *value = *value >> (ST25DV_LOCKCFG_B0_SHIFT);
  return ST25DV_OK;
}

int32_t st25dv_set_lockcfg_b0 (st25dv_ctx_t *ctx, const uint8_t *value)
{
  uint8_t reg_value;
  if( st25dv_readreg(ctx, (ST25DV_LOCKCFG_REG), &reg_value, 1))
    return ST25DV_ERROR;

  reg_value = ( (*value << (ST25DV_LOCKCFG_B0_SHIFT)) & (ST25DV_LOCKCFG_B0_MASK)) |
                (reg_value & ~(ST25DV_LOCKCFG_B0_MASK));

  if( st25dv_WriteReg(ctx, (ST25DV_LOCKCFG_REG), &reg_value, 1))
    return ST25DV_ERROR;
  return ST25DV_OK;
}

int32_t st25dv_get_i2c_sso_dyn_i2csso (st25dv_ctx_t *ctx, uint8_t *value)
{
  if( st25dv_readreg(ctx, (ST25DV_I2C_SSO_DYN_REG), (uint8_t *)value, 1))
    return ST25DV_ERROR;
  *value &= (ST25DV_I2C_SSO_DYN_I2CSSO_MASK);
  *value = *value >> (ST25DV_I2C_SSO_DYN_I2CSSO_SHIFT);
  return ST25DV_OK;
}



