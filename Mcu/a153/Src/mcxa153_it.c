/*
 * mcxa153_it.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "mcxa153_it.h"

//TODO check what is not necessary
extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void doPWMChanges();
extern void tenKhzRoutine();
extern void sendDshotDma();
extern void receiveDshotDma();
extern void processDshot();
extern char send_telemetry;
extern char telemetry_done;
extern char servoPwm;
extern char dshot_telemetry;
extern char armed;
extern char out_put;
extern uint8_t compute_dshot_flag;
extern uint32_t average_interval;

extern volatile char input_ready;

volatile int signal_pin_high_count = 0;
volatile int is_inverted_dshot = 0;
extern char inputSet;
extern uint16_t zero_input_count;
extern uint32_t average_packet_length;
extern uint8_t average_count;

extern char dshot;
extern volatile char sync_dshot;

/*
 * @brief 	Comparator 0 interrupt handler. Is called after a compare event has occurred.
 */
void CMP0_IRQHandler(void)
{
	//Do extra check to prevent false positive interrupts
	if((INTERVAL_TIMER_COUNT) > ((average_interval>>1)))
	{
		CMP0->CSR = 0x7;
		interruptRoutine();
	}
	else if (getCompOutputLevel() == rising)
	{
		CMP0->CSR = 0x7;
	}
}

/*
 * @brief 	Comparator 1 interrupt handler. Is called after a compare event has occurred.
 */
void CMP1_IRQHandler(void)
{
	//Do extra check to prevent false positive interrupts
	if((INTERVAL_TIMER_COUNT) > ((average_interval>>1)))
	{
		CMP1->CSR = 0x7;
		interruptRoutine();
	}
	else if (getCompOutputLevel() == rising)
	{
		CMP1->CSR = 0x7;
	}
}

/*
 * @brief 	DSHOT CTIMER0 interrupt called after a timeout in which no Dshot frame has been detected.
 * 			This is to reset the DMA before a new Dshot frame happens.
 */
void CTIMER0_IRQHandler(void)
{
	uint32_t flags = CTIMER0->IR;

	//If input type is not PWM and sync_dshot is enabled, reset Dshot timer and DMA to sync with Dshot frame correctly
	if (!servoPwm && sync_dshot) {
		//Disable sync dshot
		sync_dshot = 0;

		//Reset timer counter
		resetInputCaptureTimer();

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
	}

	//Check for inverted Dshot. If inverted Dshot, configure timer capture for inverted Dshot
	//If not armed and not PWM input
	if (!armed && !servoPwm) {
		if (is_inverted_dshot == 0) {
			//Check if signal pin is high
			if (getInputPinState()) {
				//Do average filter
				signal_pin_high_count++;

				if (signal_pin_high_count > 100) {
					is_inverted_dshot = 1;

					//Configure CTIMER
					//Configure capture control register so capture value register is loaded on CR1 falling edge and CR2 rising edge
					modifyReg32(&CTIMER0->CCR,
							CTIMER_CCR_CAP1RE_MASK | CTIMER_CCR_CAP2RE_MASK | CTIMER_CCR_CAP1FE_MASK | CTIMER_CCR_CAP2FE_MASK,
							CTIMER_CCR_CAP2RE(1) | CTIMER_CCR_CAP1FE(1));

					//Clear timer counter on capture channel 2 rising edge
					modifyReg32(&CTIMER0->CTCR,
							CTIMER_CTCR_ENCC_MASK | CTIMER_CTCR_SELCC_MASK,
							CTIMER_CTCR_ENCC(1) | CTIMER_CTCR_SELCC(4));

					//Reset input detect state so it detects inverted Dshot correctly
					inputSet = 0;
					armed = 0;
					average_packet_length = 0;
					average_count = 0;
					zero_input_count = 0;
					ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 5);
					dshot_frametime_high = 50000;
					dshot_frametime_low = 0;
				}
			}
		}
	}

	//Clear interrupt flags
	CTIMER0->IR = flags;
}

/*
 * @brief COM_TIMER interrupt
 */
void CTIMER1_IRQHandler(void)
{
	uint32_t flags = CTIMER1->IR;

	if(flags & CTIMER_IR_MR0INT_MASK) {
		//Call function from main.c
		PeriodElapsedCallback();
	}

	//Clear interrupt flags
	CTIMER1->IR = flags;
}

/*
 * @brief 	TenKhzTimer interrupt
 */
void LPTMR0_IRQHandler(void)
{
	//Call loop function from main.c
	tenKhzRoutine();

	//Clear timer compare flag
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TCF_MASK, LPTMR_CSR_TCF(1));
}

/*
 * @brief 	Dshot/PWM interrupt handler. Calls when DMA0 has transfered the captured timing data
 */
void DMA_CH0_IRQHandler(void)
{
	//Clear DMA channel 0 interrupt flag
	DMA0->CH[DMA_CH_DshotPWM].CH_INT = DMA_CH_INT_INT(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, 0, DMA_CH_CSR_DONE(1));

	//Clear DMA error flag
	DMA0->CH[DMA_CH_DshotPWM].CH_ES = DMA_CH_ES_ERR(1);

	//Disable DMA hardware request
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, DMA_CH_CSR_ERQ_MASK, 0);

	//Convert to correct Dshot/PWM timing data format
	doDshotCorrection();

	//Call transfercomplete
	transfercomplete();

	//Set input_ready so processDshot is called in main loop
	input_ready = 1;

	//Enable DMA hardware request
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, DMA_CH_CSR_ERQ_MASK, DMA_CH_CSR_ERQ(1));
}

/*
 * @brief 	ADC interrupt request called after the DMA has transferred the ADC data
 */
void DMA_CH1_IRQHandler(void)
{
	//Call adc callback
	ADC_DMA_Callback();

	//Clear DMA channel 0 interrupt flag
	DMA0->CH[DMA_CH_ADC].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_ADC].CH_ES = DMA_CH_ES_ERR(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_ADC].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));
}

/*
 * @brief 	DMA interrupt after all UART data has been transferred
 */
void DMA_CH2_IRQHandler(void)
{
	//Clear DMA channel 2 interrupt flag
	DMA0->CH[DMA_CH_UART].CH_INT = DMA_CH_INT_INT(1);

	//Clear DMA error flag
	DMA0->CH[DMA_CH_UART].CH_ES = DMA_CH_ES_ERR(1);

	//Clear done flag
	modifyReg32(&DMA0->CH[DMA_CH_UART].CH_CSR, DMA_CH_CSR_DONE_MASK, DMA_CH_CSR_DONE(1));
}

/*
 * @brief 	LPSPI interrupt after SPI transfer complete
 */
void LPSPI0_IRQHandler(void)
{
	//Clear Transfer complete flag
	LPSPI0->SR = LPSPI_SR_TCF_MASK | LPSPI_SR_FCF_MASK | LPSPI_SR_WCF_MASK;

	//Call transfercomplete
	transfercomplete();

	//Set input_ready so processDshot is called in main loop
	input_ready = 1;
}

