/*
 * mcxa153_it.c
 *
 *  Created on: 12 Nov 2024
 *      Author: nxg09992
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

	//TODO remove this
	GPIO3->PTOR = (1 << 27);	//ENC_A

	//If second rising/falling edge of the dshot frame is expected. Do not clear the timer.
	//Second rising/falling edge will be written to the dma_buffer address + 4 bytes offset.
//	if (DMA0->CH[DMA_CH_DshotPWM].TCD_DADDR != (uint32_t)&dma_buffer + 4) {
//		CTIMER0->TC = 0;

//		//Set latest capture 1 value to counter value to prevent missing the first falling/rising edge of a Dshot frame
//		//This is because the DMA triggers after seeing a rising and falling or a falling and rising after each other.
//		//This means that the timeout could happen in between the first bit of a Dshot frame,
//		//causing the first rising/falling time to be missing.
//		modifyReg32(&CTIMER0->TC, CTIMER_TC_TCVAL_MASK, CTIMER_TC_TCVAL(CTIMER0->CR[1]));
//	}

	//Check if CR[1] > 0 and CR[2] > 0
	//If only CR[1] > 0 then the timeout is happening in the first Dshot bit

	//Check if capture 1 is larger than capture 2.
	//If this is the case, it must be that this is an invalid timeout in the first Dshot bit
	//So if this is not the case, clear the timer interrupt as the timeout is valid
	//Also check if the destination address is the first element of the dma_buffer
	if (!((CTIMER0->CR[1] > CTIMER0->CR[2]) && (DMA0->CH[DMA_CH_DshotPWM].TCD_DADDR == (uint32_t)&dma_buffer))) {
//		resetInputCaptureTimer();
		CTIMER0->TC = 0;
	}
	else
	{
		//TODO remove this
		GPIO3->PTOR = (1 << 28);	//ENC_I
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

	//Set latest capture 1 value to counter value to prevent missing the first falling/rising edge of a Dshot frame
	//This is because the DMA triggers after seeing a rising and falling or a falling and rising after each other.
	//This means that the timeout could happen in between the first bit of a Dshot frame,
	//causing the first rising/falling time to be missing.
//	modifyReg32(&CTIMER0->TC, CTIMER_TC_TCVAL_MASK, CTIMER_TC_TCVAL(CTIMER0->CR[1]));

	//Check for inverted Dshot
	if (!armed && dshot) {
//	if (!armed) {
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
					ic_timer_prescaler = (CPU_FREQUENCY_MHZ / 8);
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

//    if (armed && dshot_telemetry) {
//        if (out_put) {
//            receiveDshotDma();
//            compute_dshot_flag = 2;
//        } else {
//            sendDshotDma();
//            compute_dshot_flag = 1;
//        }
//    }
//    else
//    {
		//Call transfercomplete
		transfercomplete();
//    }

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

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
//	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
//			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

//	out_put = 0;

	//Reset Dshot timer to prevent a timeout from happening right after transfer
//	resetInputCaptureTimer();

	//Resets PWM/Dshot timer to 0. Needed for Dshot to work properly.
//	resetInputCaptureTimer();
//	CTIMER0->TC = 0;

	//Set match1 value to higher then the minimum Dshot300 frame time which is around 53us, so take at least 53us.
//	CTIMER0->MR[1] = 10000 / (CTIMER0->PR + 1);

	//Reset timer and enable interrupt on Match1 event
//	modifyReg32(&CTIMER0->MCR, CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR1R_MASK, CTIMER_MCR_MR1I(1));

	transfercomplete();

//	input_ready = 1;

//	receiveDshotDma();

//	DMA_CH0_IRQHandler();
}

