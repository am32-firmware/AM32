/*
 * mcxa153_common.h
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#ifndef MCU_A153_INC_MCXA153_COMMON_H_
#define MCU_A153_INC_MCXA153_COMMON_H_

#include "main.h"

static inline void modifyReg32(volatile uint32_t *regAddr, uint32_t clearbits, uint32_t setbits)
{
	uint32_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

static inline void modifyReg16(volatile uint16_t *regAddr, uint16_t clearbits, uint16_t setbits)
{
	uint16_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

static inline void modifyReg8(volatile uint8_t *regAddr, uint8_t clearbits, uint8_t setbits)
{
	uint8_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

#endif /* MCU_A153_INC_MCXA153_COMMON_H_ */
