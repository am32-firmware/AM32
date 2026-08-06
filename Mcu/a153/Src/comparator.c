/*
 * comparator.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "comparator.h"
#include "functions.h"

LPCMP_Type *MAIN_COMP = CMP0;

void initComp0(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_CMP0(1);

	//Select functional clock for CMP0 to FRO_12M clock
	modifyReg32(&MRCC0->MRCC_CMP0_RR_CLKSEL, MRCC_MRCC_CMP0_RR_CLKSEL_MUX_MASK, MRCC_MRCC_CMP0_RR_CLKSEL_MUX(0));

	//Enable CMP0 functional clock
	modifyReg32(&MRCC0->MRCC_CMP0_FUNC_CLKDIV, MRCC_MRCC_CMP0_FUNC_CLKDIV_HALT_MASK, 0);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Enable analog module CMP0
	modifyReg32(&SPC0->ACTIVE_CFG1, 0, (1 << 16));

	//Select functional clock source
	//clock source 1: FRO_16K
	//clock source 3: CMPn function clock as defined in MRCC_CMP0_RR_CLKSEL
	modifyReg32(&CMP0->CCR1, LPCMP_CCR1_FUNC_CLK_SEL_MASK, LPCMP_CCR1_FUNC_CLK_SEL(3));

	//Select BEMF_COMMON as plus input
	modifyReg32(&CMP0->CCR2, LPCMP_CCR2_PSEL_MASK, LPCMP_CCR2_PSEL(COMMON_COMP0_INP));

	//Enable high speed comparator mode
	modifyReg32(&CMP0->CCR2, LPCMP_CCR2_CMP_HPMD_MASK, LPCMP_CCR2_CMP_HPMD(1));
}

void initComp1(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_CMP1(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_CMP1(1);

	//Select functional clock for CMP1 to FRO_12M clock
	modifyReg32(&MRCC0->MRCC_CMP1_RR_CLKSEL, MRCC_MRCC_CMP1_RR_CLKSEL_MUX_MASK, MRCC_MRCC_CMP1_RR_CLKSEL_MUX(0));

	//Enable CMP1 functional clock
	modifyReg32(&MRCC0->MRCC_CMP1_FUNC_CLKDIV, MRCC_MRCC_CMP1_FUNC_CLKDIV_HALT_MASK, 0);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Enable analog module CMP1
	modifyReg32(&SPC0->ACTIVE_CFG1, 0, (1 << 17));

	//Select functional clock source
	//clock source 1: FRO_16K
	//clock source 3: CMPn function clock as defined in MRCC_CMP1_RR_CLKSEL
	modifyReg32(&CMP1->CCR1, LPCMP_CCR1_FUNC_CLK_SEL_MASK, LPCMP_CCR1_FUNC_CLK_SEL(3));

	//Select BEMF_COMMON as plus input
	modifyReg32(&CMP1->CCR2, LPCMP_CCR2_PSEL_MASK, LPCMP_CCR2_PSEL(COMMON_COMP1_INP));

	//Enable high speed comparator mode
	modifyReg32(&CMP1->CCR2, LPCMP_CCR2_CMP_HPMD_MASK, LPCMP_CCR2_CMP_HPMD(1));
}

/*
 * @brief 	Enables only the main selected comparator unit
 */
void enableComparator(void)
{
	//Enable the selected comparator unit
	modifyReg32(&MAIN_COMP->CCR0, 0, LPCMP_CCR0_CMP_EN(1));
}

/*
 * @brief 	Disables both comparator units
 */
void disableComparators(void)
{
	//Disable comparator 0 and 1
	modifyReg32(&CMP0->CCR0, LPCMP_CCR0_CMP_EN(1), 0);
	modifyReg32(&CMP1->CCR0, LPCMP_CCR0_CMP_EN(1), 0);
}

uint8_t getCompOutputLevel()
{
	return ((MAIN_COMP->CSR & LPCMP_CSR_COUT_MASK) >> LPCMP_CSR_COUT_SHIFT);
}

/*
 * @brief 	Disables the comparator interrupt
 */
void maskPhaseInterrupts()
{
	//Disable comparator interrupt
	if (MAIN_COMP == CMP0)
	{
		__NVIC_DisableIRQ(CMP0_IRQn);
	}
	else
	{
		__NVIC_DisableIRQ(CMP1_IRQn);
	}
}

/*
 * @brief 	Enables the comparator interrupt
 */
void enableCompInterrupts()
{
	//Enable comparator interrupt
	if (MAIN_COMP == CMP0)
	{
		__NVIC_EnableIRQ(CMP0_IRQn);
	}
	else
	{
		__NVIC_EnableIRQ(CMP1_IRQn);
	}
}

/*
 * @brief	Changes the main comparator unit and copies the correct settings to that one
 */
void changeMainComp(LPCMP_Type *CMPx)
{
	//Get previous main comparator unit
	LPCMP_Type *prev_MAIN_COMP 	= MAIN_COMP;
	MAIN_COMP 					= CMPx;

	if (MAIN_COMP != prev_MAIN_COMP)
	{
		//Enable and disable correct comparator interrupt
		if (MAIN_COMP == CMP0)
		{
			if (__NVIC_GetEnableIRQ(CMP1_IRQn))
			{
				__NVIC_EnableIRQ(CMP0_IRQn);
				__NVIC_DisableIRQ(CMP1_IRQn);
			}
		}
		else
		{
			if (__NVIC_GetEnableIRQ(CMP0_IRQn))
			{
				__NVIC_EnableIRQ(CMP1_IRQn);
				__NVIC_DisableIRQ(CMP0_IRQn);
			}
		}
	}
}

/*
 * @brief 	Changes which phase will be compared to BEMF_COMMON
 * 			Also changes the comparator unit being used if necessary
 * 			And sets to get an interrupt on the rising or falling edge of the comparator output
 */
void changeCompInput()
{
	//Disable the comparator unit.
	//Necessary to prevent any noise on the comparator output when changing the control register fields.
	disableComparators();

	//Check commutation step
	if (step == 1 || step == 4)
	{
		//Set minus input to PHASE C, COMP0_IN1
		modifyReg32(&PHASE_C_COMP_UNIT->CCR2, LPCMP_CCR2_MSEL_MASK, LPCMP_CCR2_MSEL(PHASE_C_COMP_INP));
		changeMainComp(PHASE_C_COMP_UNIT);
	}
	if (step == 2 || step == 5)
	{
		//Set minus input to PHASE A, COMP0_IN3
		modifyReg32(&PHASE_A_COMP_UNIT->CCR2, LPCMP_CCR2_MSEL_MASK, LPCMP_CCR2_MSEL(PHASE_A_COMP_INP));
		changeMainComp(PHASE_A_COMP_UNIT);
	}
	if (step == 3 || step == 6)
	{
		//Set minus input to PHASE B, COMP1_IN3
		modifyReg32(&PHASE_B_COMP_UNIT->CCR2, LPCMP_CCR2_MSEL_MASK, LPCMP_CCR2_MSEL(PHASE_B_COMP_INP));
		changeMainComp(PHASE_B_COMP_UNIT);
	}

	modifyReg32(&CMP0->IER,
			LPCMP_IER_CFF_IE_MASK | LPCMP_IER_CFR_IE_MASK,
			0);
	modifyReg32(&CMP1->IER,
			LPCMP_IER_CFF_IE_MASK | LPCMP_IER_CFR_IE_MASK,
			0);

	//Enable main comparator unit after control register fields have been adjusted
	enableComparator();

	//Check if BEMF is rising or falling
	if (rising)
	{
		//Disable rising interrupt, enable falling interrupt
		modifyReg32(&MAIN_COMP->IER,
				LPCMP_IER_CFF_IE_MASK | LPCMP_IER_CFR_IE_MASK,
				LPCMP_IER_CFF_IE(1) | LPCMP_IER_CFR_IE(0));
	}
	else
	{
		//Disable falling interrupt, enable rising interrupt
		modifyReg32(&MAIN_COMP->IER,
				LPCMP_IER_CFF_IE_MASK | LPCMP_IER_CFR_IE_MASK,
				LPCMP_IER_CFF_IE(0) | LPCMP_IER_CFR_IE(1));
	}
}

