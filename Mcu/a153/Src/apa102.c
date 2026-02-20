/*
 * apa102.c
 *
 *  Created on: 18 Feb 2026
 *      Author: Youri Tils
 */

#include "apa102.h"

#define NUM_LEDS 1
#define BRIGHTNESS 31	//Can be from 0 to 31

/*
 * @brief 	Initialize SPI1 for RGB led use.
 * 			SPI uses functional clock of 12MHz
 */
void init_apa102(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Select FRO_12M as clock for LPSPI1, which is 12MHz
	modifyReg32(&MRCC0->MRCC_LPSPI1_CLKSEL, MRCC_MRCC_LPSPI1_CLKSEL_MUX_MASK, MRCC_MRCC_LPSPI1_CLKSEL_MUX(0));

	//Enable LPSPI1 and set divider to 0, so clock frequency is 12MHz
	modifyReg32(&MRCC0->MRCC_LPSPI1_CLKDIV,
			MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK | MRCC_MRCC_LPSPI1_CLKDIV_DIV_MASK,
			MRCC_MRCC_LPSPI1_CLKDIV_DIV(0));

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_LPSPI1(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_LPSPI1(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set that SIN is used for input and SOUT for output data
	//Set LPSPI master operating mode
	modifyReg32(&LPSPI1->CFGR1,
			LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_MASTER_MASK,
			LPSPI_CFGR1_PINCFG(0) | LPSPI_CFGR1_MASTER(1));

	//Set CCR
	LPSPI1->CCR = 0;

	//Set baud rate to 6MHz by setting prescaler to 0.
	//Baud rate = function clock ÷ (2^PRESCALE × (SCKSET + SCKHLD + 2))
	//Set frame size to 32.
	modifyReg32(&LPSPI1->TCR,
			LPSPI_TCR_PRESCALE_MASK | LPSPI_TCR_FRAMESZ_MASK,
			LPSPI_TCR_PRESCALE(0) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK(1));

	//Configure LPSPI pins, P2_12 = SCK, P2_13 = SDO
	modifyReg32(&PORT2->PCR[12], PORT_PCR_MUX_MASK, PORT_PCR_MUX(2));
	modifyReg32(&PORT2->PCR[13], PORT_PCR_MUX_MASK, PORT_PCR_MUX(2));

	//Enable debug mode
	modifyReg32(&LPSPI1->CR, 0, LPSPI_CR_DBGEN(1));

	//Enable SPI module
	modifyReg32(&LPSPI1->CR, LPSPI_CR_MEN_MASK, LPSPI_CR_MEN(1));
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
	//Create RGB data frame
	uint32_t rgb_data = (0x7 << 29) | (BRIGHTNESS << 24) | (blue << 16) | (green << 8) | red;

	//Initialize dataframes by 0. Start frame is all zeros
	uint32_t dataframe[NUM_LEDS + 2] = {0};

	//Add RGB data for each led
	for (int i = 0; i < NUM_LEDS; i++) {
		dataframe[i + 1] = rgb_data;
	}

	//Add end frame
	dataframe[NUM_LEDS + 1] = 0xffffffff;

	//Send all dataframes
	for (int i = 0; i < (NUM_LEDS + 2); i++) {
		//Send data frame
		LPSPI1->TDR = dataframe[i];

		//Wait while LPSPI is busy
		while(LPSPI1->SR & LPSPI_SR_MBF_MASK) {}
	}
}


