/*
 * apa102.c
 *
 *  Created on: 18 Feb 2026
 *      Author: Youri Tils
 */

#include "apa102.h"

#define NUM_LEDS 1
#define BRIGHTNESS 16	//Can be from 0 to 31

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

	//Enable LPSPI0 and set divider to 1, so clock frequency is 12MHz
	modifyReg32(&MRCC0->MRCC_LPSPI1_CLKDIV,
			MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK | MRCC_MRCC_LPSPI1_CLKDIV_DIV_MASK,
			MRCC_MRCC_LPSPI1_CLKDIV_DIV(0));

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_LPSPI1(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_LPSPI1(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set that transfer complete flag generates interrupt
//	modifyReg32(&LPSPI0->IER, 0, LPSPI_IER_TCIE(1));

	//Set that SIN is used for input and SOUT for output data
	//Set LPSPI master operating mode
	modifyReg32(&LPSPI0->CFGR1,
			LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_MASTER_MASK,
			LPSPI_CFGR1_PINCFG(0) | LPSPI_CFGR1_MASTER(1));

	//Set CCR
	LPSPI0->CCR = 0;

	//Set TX watermark to 1 word
//	LPSPI0->FCR = LPSPI_FCR_TXWATER(0);

	//Set prescaler to 8, in the end it will be 16 due to an additional value doubling.
	//Baud rate will be 750kbit/s at 12MHz functional clock
	//Set baud rate to 6MHz
	//Set frame size to 32.
	modifyReg32(&LPSPI0->TCR,
			LPSPI_TCR_PRESCALE_MASK | LPSPI_TCR_FRAMESZ_MASK,
			LPSPI_TCR_PRESCALE(1) | LPSPI_TCR_FRAMESZ(31));

	//Configure LPSPI pins, P2_12 = SCK, P2_13 = SDO
	modifyReg32(&PORT2->PCR[12], PORT_PCR_MUX_MASK, PORT_PCR_MUX(2));
	modifyReg32(&PORT2->PCR[13], PORT_PCR_MUX_MASK, PORT_PCR_MUX(2));

	//Enable SPI module
	modifyReg32(&LPSPI0->CR, LPSPI_CR_MEN_MASK, LPSPI_CR_MEN(1));

	//Enable interrupt
	__NVIC_SetPriority(LPSPI0_IRQn, 1);	//set interrupt priority to 1
	__NVIC_EnableIRQ(LPSPI0_IRQn);
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
	//Send start frame
	LPSPI1->TDR = 0;

	//Create data frame
	uint32_t dataframe = (0x7 << 29) | (BRIGHTNESS << 24) | (blue << 16) | (green << 8) | red;

	//Send data frame for each RGB LED
	for (int i = 0; i < NUM_LEDS; i++) {
		LPSPI1->TDR = dataframe;
	}

	//Send end frame
	LPSPI1->TDR = 0xffff;

	//Start SPI transfer
	modifyReg32(&LPSPI0->TCR, 0, LPSPI_TCR_TXMSK(1));
}


