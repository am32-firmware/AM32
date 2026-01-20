/*
 * IO.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "IO.h"

char ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 5);	//Used to detect input type (Dshot300, Dshot600 and PWM400). Divide by 8 is best for MCXA153 at 96MHz CPU
uint32_t dma_buffer[64] = { 0 };
char out_put = 0;
uint8_t buffer_padding = 0;

extern volatile uint32_t gcrnumber;
volatile char sync_dshot = 0;

void receiveDshotDma()
{
	//Set output variable for state machine
	out_put = 0;

	//Set prescaler
	CTIMER0->PR = ic_timer_prescaler;

	//Set sync_dshot to enable syncing with dshot frame
	sync_dshot = 1;

	if (buffersize > 3) {
		//Resets PWM/Dshot timer to 0. Needed for Dshot to work properly.
		resetInputCaptureTimer();

		//Set match1 value to higher then the minimum Dshot300 frame time which is around 53us, so take at least 53us.
		//Set timeout value to 5500 clock ticks at Dshot300 (Prescaler is 1 then)
		CTIMER0->MR[1] = 11000 / (CTIMER0->PR + 1);

		//Reset timer and enable interrupt on Match1 event
		modifyReg32(&CTIMER0->MCR, CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR1R_MASK, CTIMER_MCR_MR1I(1));

	} else {
		//Disable interrupt and reset on Match1 event
		modifyReg32(&CTIMER0->MCR, CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR1R_MASK, 0);
	}

	//Set the major loop count and addresses again to prevent unintended DMA request from CTIMER match register
	//Set current and beginning major loop count to 8
	DMA0->CH[DMA_CH_DshotPWM].TCD_CITER_ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(buffersize / 2);

	//Sets the amount of major loop counts after a DMA transfer completes
	//i.e. the CITER register gets this BITER value after DMA transfer complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_BITER_ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(buffersize / 2);

	//Set last destination address adjustment to -8 bytes * the number of DMA requests before transfer complete
	//Adds this value to the destination address when CITER reaches 0 / DMA transfer is complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_DLAST_SGA = -(8 * (buffersize / 2));

	//Set source address
	DMA0->CH[DMA_CH_DshotPWM].TCD_SADDR = (uint32_t)&CTIMER0->CR[1];

	//Set destination address
	DMA0->CH[DMA_CH_DshotPWM].TCD_DADDR = (uint32_t)&dma_buffer;

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));
}

void sendDshotDma()
{
	//Set output variable for state machine
	out_put = 1;

	//Change Dshot pin to SPI0_SDI
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(2) | PORT_PCR_IBE(0) | PORT_PCR_PE(1) | PORT_PCR_PS(1));

	//Functional LPSPI0 clock is 12MHz. To get 5/4 x Dshot bit rate, SPI prescaler should be:
	//Dshot600 => prescaler = 8
	//Dshot300 => prescaler = 16

	//Check if Dshot600
	if (ic_timer_prescaler) {
		//Set transmit prescaler to 8 for Dshot600
		//Set framesize to 21 bits
		modifyReg32(&LPSPI0->TCR,
				LPSPI_TCR_PRESCALE_MASK | LPSPI_TCR_FRAMESZ_MASK,
				LPSPI_TCR_PRESCALE(3) | LPSPI_TCR_FRAMESZ(20) | LPSPI_TCR_RXMSK(1));
	//Is Dshot300
	} else {
		//Set transmit prescaler to 16 for Dshot300
		//Set framesize to 21 bits
		modifyReg32(&LPSPI0->TCR,
				LPSPI_TCR_PRESCALE_MASK | LPSPI_TCR_FRAMESZ_MASK,
				LPSPI_TCR_PRESCALE(4) | LPSPI_TCR_FRAMESZ(20) | LPSPI_TCR_RXMSK(1));
	}

	//Send GCR data to LPSPI FIFO
	LPSPI0->TDR = gcrnumber;
}

uint8_t getInputPinState()
{
	//Set INPUT_PIN to a GPIO pin
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

	//Read INPUT_PIN value
	uint8_t readPinData = INPUT_PIN_GPIO->PDR[INPUT_PIN];

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

	return readPinData;
}

void setInputPullDown()
{
	//Enable internal pull-down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_PE(1) | PORT_PCR_PS(0));
}

void setInputPullUp()
{
	//Enable internal pull-up resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_PE(1) | PORT_PCR_PS(1));
}

void setInputPullNone()
{
	//Disable internal pull resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN], PORT_PCR_PE_MASK, 0);
}
