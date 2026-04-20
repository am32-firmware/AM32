/*
 * DMA.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "DMA.h"

extern uint16_t ADCDataDMA[];

extern uint8_t buffersize;
extern uint32_t dma_buffer[64];

//extern int8_t aTxBuffer[10];
extern int8_t aTxBuffer[18];
extern uint8_t nbDataToTransmit;

/*
 * @brief 	Initializes DMA channel 0 for transferring the Dshot / PWM timing data
 */
void initDMA_DshotPWM(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Sets the amount of major loop counts to handle before DMA transfer complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_CITER_ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(buffersize / 2);

	//Sets the amount of major loop counts after a DMA transfer completes
	//i.e. the CITER register gets this BITER value after DMA transfer complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_BITER_ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(buffersize / 2);

	//Set number of bytes to 8
	//We are reading CR[1] and CR[2]
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].TCD_NBYTES_MLOFFNO,
			DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK,
			DMA_TCD_NBYTES_MLOFFNO_NBYTES(8));

	//Set source address to CR[1] to make the DMA circulate between CR[1] and CR[2] properly
	DMA0->CH[DMA_CH_DshotPWM].TCD_SADDR = (uint32_t)&CTIMER0->CR[1];

	//Set Source Address offset to 4 bytes
	//Increments the source address by this value after each read transaction
	DMA0->CH[DMA_CH_DshotPWM].TCD_SOFF = 4;

	//Set source size to 32-bit
	modifyReg16(&DMA0->CH[DMA_CH_DshotPWM].TCD_ATTR, DMA_TCD_ATTR_SSIZE_MASK, DMA_TCD_ATTR_SSIZE(2));

	//Set source modulo to three LSB that may change.
	modifyReg16(&DMA0->CH[DMA_CH_DshotPWM].TCD_ATTR, DMA_TCD_ATTR_SMOD_MASK, DMA_TCD_ATTR_SMOD(3));

	//Set last source address adjustment to 0 bytes
	//Adds this value to the source address when the major loop is complete.
	DMA0->CH[DMA_CH_DshotPWM].TCD_SLAST_SDA = 0;

	//Set destination address
	DMA0->CH[DMA_CH_DshotPWM].TCD_DADDR = (uint32_t)&dma_buffer;

	//Set Destination offset to 4 bytes
	//Increments the destination address by this value after each write transaction
	DMA0->CH[DMA_CH_DshotPWM].TCD_DOFF = 4;

	//Set destination size to 32-bit
	modifyReg16(&DMA0->CH[DMA_CH_DshotPWM].TCD_ATTR, DMA_TCD_ATTR_DSIZE_MASK, DMA_TCD_ATTR_DSIZE(2));

	//Set last destination address adjustment to -8 bytes * the number of DMA requests before transfer complete
	//Adds this value to the destination address when CITER reaches 0 / DMA transfer is complete
	DMA0->CH[DMA_CH_DshotPWM].TCD_DLAST_SGA = -(8 * (buffersize / 2));

	//Set the CTIMER0 MATCH0 request to initiate a DMA request
	DMA0->CH[DMA_CH_DshotPWM].CH_MUX = kDma0RequestMuxCtimer0M0;

	//Enable DMA0 CH0 interrupts
	__NVIC_SetPriority(DMA_CH_DshotPWM_IRQ, 1);		//Set Dshot/PWM interrupt priority to 1
	__NVIC_EnableIRQ(DMA_CH_DshotPWM_IRQ);
}

/*
 * @brief 	Initializes DMA channel 1 for transferring the ADC data from the ADC FIFO to the ADC data buffer
 */
void initDMA_ADC(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set current and beginning major loop count to ADC data size
	modifyReg16(&DMA0->CH[DMA_CH_ADC].TCD_CITER_ELINKNO, DMA_TCD_CITER_ELINKNO_CITER_MASK, DMA_TCD_CITER_ELINKNO_CITER(ADCDataDMA_size));
	modifyReg16(&DMA0->CH[DMA_CH_ADC].TCD_BITER_ELINKNO, DMA_TCD_BITER_ELINKNO_BITER_MASK, DMA_TCD_BITER_ELINKNO_BITER(ADCDataDMA_size));

	//Set number of bytes to 2
	modifyReg32(&DMA0->CH[DMA_CH_ADC].TCD_NBYTES_MLOFFNO, DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK, DMA_TCD_NBYTES_MLOFFNO_NBYTES(2));

	//Set source address
	DMA0->CH[DMA_CH_ADC].TCD_SADDR = (uint32_t)&ADC0->RESFIFO;

	//Set Source Address offset to 0
	//Increments the source address by this value after each read transaction
	DMA0->CH[DMA_CH_ADC].TCD_SOFF = 0;

	//Set source size to 16-bit
	modifyReg16(&DMA0->CH[DMA_CH_ADC].TCD_ATTR, DMA_TCD_ATTR_SSIZE_MASK, DMA_TCD_ATTR_SSIZE(1));

	//Set last source address adjustment.
	//Adds this value to the source address when the major loop is complete.
	DMA0->CH[DMA_CH_ADC].TCD_SLAST_SDA = 0;

	//Set destination address
	DMA0->CH[DMA_CH_ADC].TCD_DADDR = (uint32_t)&ADCDataDMA;

	//Set Destination offset to 2 bytes
	//Increments the destination address by this value after each write transaction
	DMA0->CH[DMA_CH_ADC].TCD_DOFF = 2;

	//Set destination size to 16-bit
	modifyReg16(&DMA0->CH[DMA_CH_ADC].TCD_ATTR, DMA_TCD_ATTR_DSIZE_MASK, DMA_TCD_ATTR_DSIZE(1));

	//Set last destination address adjustment
	//Adds this value to the destination address when the major loop is complete.
	DMA0->CH[DMA_CH_ADC].TCD_DLAST_SGA = -(2 * ADCDataDMA_size);

	//Set the ADC FIFO request to initiate a DMA request
	DMA0->CH[DMA_CH_ADC].CH_MUX = kDma0RequestMuxAdc0FifoRequest;

	//Enable DMA0 CH1 interrupt
	__NVIC_SetPriority(DMA_CH_ADC_IRQ, 1);	//Set ADC interrupt priority to 1
	__NVIC_EnableIRQ(DMA_CH_ADC_IRQ);
}

/*
 * @brief 	Initializes DMA for transferring aTxBuffer data to the TX FIFO
 * 			Gets DMA request when TX FIFO is empty.
 * 			After all data has been transferred the DMA disables the DMA hardware request
 */
void initDMA_UART(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_DMA(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Sets the amount of major loop counts to handle before DMA transfer complete
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_CITER_ELINKNO, DMA_TCD_CITER_ELINKNO_CITER_MASK,
			DMA_TCD_CITER_ELINKNO_CITER(nbDataToTransmit));

	//Sets the amount of major loop counts after a DMA transfer completes
	//i.e. the CITER register gets this BITER value after DMA transfer complete
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_BITER_ELINKNO, DMA_TCD_BITER_ELINKNO_BITER_MASK,
			DMA_TCD_BITER_ELINKNO_BITER(nbDataToTransmit));

	//Set number of bytes to 1
	modifyReg32(&DMA0->CH[DMA_CH_UART].TCD_NBYTES_MLOFFNO, DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK, DMA_TCD_NBYTES_MLOFFNO_NBYTES(1));

	//Set source address to txBuffer
	DMA0->CH[DMA_CH_UART].TCD_SADDR = (uint32_t)&aTxBuffer;

	//Set Source Address offset to 1 bytes
	//Increments the source address by this value after each read transaction
	DMA0->CH[DMA_CH_UART].TCD_SOFF = 1;

	//Set source size to 8-bit
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_ATTR, DMA_TCD_ATTR_SSIZE_MASK, DMA_TCD_ATTR_SSIZE(0));

	//Set last source address adjustment to 0 bytes
	//Adds this value to the source address when the major loop is complete.
	DMA0->CH[DMA_CH_UART].TCD_SLAST_SDA = -nbDataToTransmit;

	//Set destination address
	DMA0->CH[DMA_CH_UART].TCD_DADDR = (uint32_t)&SERIAL_TELEMETRY->DATA;

	//Set Destination offset to 0 bytes
	//Increments the destination address by this value after each write transaction
	DMA0->CH[DMA_CH_UART].TCD_DOFF = 0;

	//Set destination size to 8-bit
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_ATTR, DMA_TCD_ATTR_DSIZE_MASK, DMA_TCD_ATTR_DSIZE(0));

	//Set last destination address adjustment to -8 bytes * the number of DMA requests before transfer complete
	//Adds this value to the destination address when CITER reaches 0 / DMA transfer is complete
	DMA0->CH[DMA_CH_UART].TCD_DLAST_SGA = 0;

	//Set DMA request address to UART1 TX
	DMA0->CH[DMA_CH_UART].CH_MUX = kDma0RequestLPUART0Tx + (2 * SERIAL_TELEMETRY_MODULE);

	//Set that after major count completion the hardware request bit is cleared automatically
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_CSR, DMA_TCD_CSR_DREQ_MASK, DMA_TCD_CSR_DREQ(1));

	//Enable DMA0 CH2 interrupt
	__NVIC_SetPriority(DMA_CH_UART_IRQ, 2);	//Set UART interrupt priority to 2
	__NVIC_EnableIRQ(DMA_CH_UART_IRQ);
}

/*
 * @brief 	Enables the DMA and re-initializes the addresses and major loop count
 * 			(this is needed due to unwanted triggering)
 */
void enableDMA_DshotPWM(void)
{
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

	//Enable major count complete interrupt
	modifyReg16(&DMA0->CH[DMA_CH_DshotPWM].TCD_CSR, DMA_TCD_CSR_INTMAJOR_MASK, DMA_TCD_CSR_INTMAJOR(1));

	//Clear DMA channel 0 interrupt flag
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_INT, 0, DMA_CH_INT_INT(1));

	//Enable DMA hardware request
	modifyReg32(&DMA0->CH[DMA_CH_DshotPWM].CH_CSR, DMA_CH_CSR_ERQ_MASK, DMA_CH_CSR_ERQ(1));

}

void enableDMA_ADC(void)
{
	//Enable major count complete interrupt
	modifyReg16(&DMA0->CH[DMA_CH_ADC].TCD_CSR, DMA_TCD_CSR_INTMAJOR_MASK, DMA_TCD_CSR_INTMAJOR(1));

	//Enable DMA hardware request
	modifyReg32(&DMA0->CH[DMA_CH_ADC].CH_CSR, DMA_CH_CSR_ERQ_MASK, DMA_CH_CSR_ERQ(1));
}

void enableDMA_UART(void)
{
	//Enable major count complete interrupt
	modifyReg16(&DMA0->CH[DMA_CH_UART].TCD_CSR, DMA_TCD_CSR_INTMAJOR_MASK, DMA_TCD_CSR_INTMAJOR(1));
}

/*
 * @brief 	Does a correction on the Dshot timing data to make it in the format expected by computeDshotDMA().
 * 			The timing is made up of small timing counter ramps as this is how our MCU is able to capture the Dshot.
 * 			These small ramps need to be added up to each other to reproduce to one timing counter ramp as computeDshotDMA() expects.
 */
void doDshotCorrection(void)
{
	//If larger than 3 it must be Dshot. PWM input does not need correction
	if (buffersize > 3) {
		int sum = dma_buffer[1];
		int temp_sum = sum;

		for (int i = 1; i < (buffersize / 2); i++) {
			temp_sum += dma_buffer[(i << 1) + 1];

			dma_buffer[(i << 1)] += sum;
			dma_buffer[(i << 1) + 1] += sum;

			sum = temp_sum;
		}
	}
}

