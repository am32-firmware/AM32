/*
 * ADC.c
 *
 *  Created on: 14 Nov 2024
 *      Author: Youri
 */

#include "ADC.h"

uint16_t ADCDataDMA[ADCDataDMA_size];

extern uint16_t ADC_raw_temp[2];
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

//MCXA13x/14x/15x ADC temperature calculation coefficients (see datasheet ADC electrical characteristics)
const float slopeFactor 		= 738;
const float offsetFactor 		= 287.5;
const float bandgapCoefficient 	= 10.06;

/*
 * @brief 	Compute gain adjustment
 */
uint32_t LPADC_GetGainConvResult(float gainAdjustment)
{
    uint16_t i        = 0U;
    uint32_t tmp32    = 0U;
    uint32_t GCRa[17] = {0};
    uint32_t GCALR    = 0U;

    for (i = 0x11U; i > 0U; i--)
    {
        tmp32          = (uint32_t)((gainAdjustment) / ((float)(1.0 / (double)(1U << (0x10U - (i - 1U))))));
        GCRa[i - 1U]   = tmp32;
        gainAdjustment = gainAdjustment - ((float)tmp32) * ((float)(1.0 / (double)(1U << (0x10U - (i - 1U)))));
    }
    /* Get GCALR value calculated */
    for (i = 0x11U; i > 0U; i--)
    {
        GCALR += GCRa[i - 1U] * ((uint32_t)(1UL << (uint32_t)(i - 1UL)));
    }

    /* to return GCALR value calculated */
    return GCALR;
}

/*
 * @brief 	Calibrates the ADC
 */
void calibADC(void)
{
	//Set calibration averaging to 256
	modifyReg32(&ADC0->CTRL, ADC_CTRL_CAL_AVGS_MASK, ADC_CTRL_CAL_AVGS(8));

	//Initiate offset calibration
	modifyReg32(&ADC0->CTRL, ADC_CTRL_CALOFS_MASK, ADC_CTRL_CALOFS(1));

	//Poll for offset calibration to complete
	while ( !((ADC0->STAT & ADC_STAT_CAL_RDY_MASK) >> ADC_STAT_CAL_RDY_SHIFT))
	{
		//Wait for calibration
		__asm volatile ("nop");
	}

	//Initiate calibration routine
	modifyReg32(&ADC0->CTRL, ADC_CTRL_CAL_REQ_MASK, ADC_CTRL_CAL_REQ(1));

	//Poll for calibration routine to complete
	while ( !((ADC0->GCC[0] & ADC_GCC_RDY_MASK) >> ADC_GCC_RDY_SHIFT))
	{
		//Wait for calibration
		__asm volatile ("nop");
	}

	//Get gain calibration value
	uint32_t gain_cal = (ADC0->GCC[0] & ADC_GCC_GAIN_CAL_MASK);

	//Calculate gain offset
	float gainAdjustment = (float)((131072.0) / (131072.0 - (double)gain_cal)); // Gain_CalA = (131072.0 / (131072-(ADC_GCC_GAIN_CAL(ADC->GCC[0]))
	ADC0->GCR[0] = LPADC_GetGainConvResult(gainAdjustment);      				// write A side GCALR

	//Set GCR0[RDY] flag to indicate entered gain_adjustment is valid
	modifyReg32(&ADC0->GCR[0], ADC_GCR_RDY_MASK, ADC_GCR_RDY(1));
}

/*
 * @brief	Initializes the ADC which measures the data as described below.
 * 			Every time data is available the DMA is called which transfers the ADC FIFO data to the ADCDataDMA buffer.
 * 			Sets sample time per channel conversion to 625ns.
 * 			Resolution = 12-bit
 * 			Conversion sequence order: Current, voltage, temperature, (optional) input
 * 			Channel 5 = current
 * 			Channel 6 = voltage
 * 			Internal  = temperature
 * 			Channel x = (optional) input
 */
void initADC(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Set ADC clock to 12MHz (FRO_12M)
	modifyReg32(&MRCC0->MRCC_ADC0_CLKSEL, MRCC_MRCC_ADC0_CLKSEL_MUX_MASK, MRCC_MRCC_ADC0_CLKSEL_MUX(0));

	//Enable ADC clock
	modifyReg32(&MRCC0->MRCC_ADC0_CLKDIV,
				MRCC_MRCC_ADC0_CLKDIV_DIV_MASK | MRCC_MRCC_ADC0_CLKDIV_HALT_MASK,
				MRCC_MRCC_ADC0_CLKDIV_DIV(0));

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_ADC0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT2(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_ADC0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT2(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Clear PCR registers for sense pins to enable analog functionality
	modifyReg32(&SENSE_ADC_PORT->PCR[VOLTAGE_SENSE_ADC_PIN],
			0xffff,
			0);
	modifyReg32(&SENSE_ADC_PORT->PCR[CURRENT_SENSE_ADC_PIN],
			0xffff,
			0);

	//Set Trigger0 to execute CMD1
	modifyReg32(&ADC0->TCTRL[0], ADC_TCTRL_TCMD_MASK, ADC_TCTRL_TCMD(1));

	//Set ADC CMD1 for measured current
	modifyReg32(&ADC0->CMD[0].CMDL,
			ADC_CMDL_MODE_MASK | ADC_CMDL_CTYPE_MASK | ADC_CMDL_ADCH_MASK,
			ADC_CMDL_MODE(1) | ADC_CMDL_CTYPE(0) | ADC_CMDL_ADCH(CURRENT_ADC_CHANNEL));
	modifyReg32(&ADC0->CMD[0].CMDH,
			ADC_CMDH_NEXT_MASK | ADC_CMDH_STS_MASK,
			ADC_CMDH_NEXT(2) | ADC_CMDH_STS(2));

	//Set CMD2 for measured voltage
	modifyReg32(&ADC0->CMD[1].CMDL,
			ADC_CMDL_MODE_MASK | ADC_CMDL_CTYPE_MASK | ADC_CMDL_ADCH_MASK,
			ADC_CMDL_MODE(1) | ADC_CMDL_CTYPE(0) | ADC_CMDL_ADCH(VOLTAGE_ADC_CHANNEL));
	modifyReg32(&ADC0->CMD[1].CMDH,
			ADC_CMDH_NEXT_MASK | ADC_CMDH_STS_MASK,
			ADC_CMDH_NEXT(3) | ADC_CMDH_STS(2));

	//Set CMD3 for measured temperature
	/* This sensor should be configured with the following parameters:
	 * resolution = 16-bit
	 * max averaging = 0xa = 1024 conversions averaged
	 * max sample time = 0x7 = 131.5 ADCK
	 * loop is set to 1 so CMD is executed 2 times
	 * loop increment is disabled
	 * compare function is disabled
	 */
	modifyReg32(&ADC0->CMD[2].CMDL,
			ADC_CMDL_MODE_MASK | ADC_CMDL_CTYPE_MASK | ADC_CMDL_ADCH_MASK,
			ADC_CMDL_MODE(1) | ADC_CMDL_CTYPE(0) | ADC_CMDL_ADCH(TEMP_ADC_CHANNEL));
#ifndef USE_ADC_INPUT
	modifyReg32(&ADC0->CMD[2].CMDH,
			ADC_CMDH_NEXT_MASK | ADC_CMDH_STS_MASK,
			ADC_CMDH_NEXT(0) | ADC_CMDH_LOOP(1) | ADC_CMDH_AVGS(8) | ADC_CMDH_STS(6));
#else
	modifyReg32(&ADC0->CMD[2].CMDH,
			ADC_CMDH_NEXT_MASK | ADC_CMDH_STS_MASK,
			ADC_CMDH_NEXT(4) | ADC_CMDH_STS(2));

	//Set CMD4 for measured optional input
	modifyReg32(&ADC0->CMD[3].CMDL,
			ADC_CMDL_MODE_MASK | ADC_CMDL_CTYPE_MASK | ADC_CMDL_ADCH_MASK,
			ADC_CMDL_MODE(1) | ADC_CMDL_CTYPE(0) | ADC_CMDL_ADCH(0));
	modifyReg32(&ADC0->CMD[3].CMDH,
			ADC_CMDH_NEXT_MASK | ADC_CMDH_STS_MASK,
			ADC_CMDH_NEXT(0) | ADC_CMDH_STS(2));
#endif

	//Set watermark event to cause when more than 0 amount of ADC results are in result FIFO
	modifyReg32(&ADC0->FCTRL, ADC_FCTRL_FWMARK_MASK, ADC_FCTRL_FWMARK(0));

	//Enable DMA request when a watermark event occurs
	modifyReg32(&ADC0->DE, ADC_DE_FWMDE0_MASK, ADC_DE_FWMDE0(1));
}

/*
 * @brief 	Enables the ADC and calibrates it
 */
void enableADC(void)
{
	//Enable ADC
	modifyReg32(&ADC0->CTRL, ADC_CTRL_ADCEN_MASK, ADC_CTRL_ADCEN(1));

	//Calibrate the ADC. This is necessary for it to work!
	calibADC();
}

/*
 * @brief 	Starts an ADC conversion via software trigger
 */
void startADCConversion(void)
{
	//Start ADC conversion via software trigger
	ADC0->SWTRIG = ADC_SWTRIG_SWT0(1);
}

/*
 * @brief	Writes the ADCDataDMA buffer to globally used variables
 */
void ADC_DMA_Callback()
{
#ifdef USE_ADC_INPUT
	ADC_raw_input 	= ADCDataDMA[4];
	ADC_raw_temp[1] = ADCDataDMA[3];
	ADC_raw_temp[0] = ADCDataDMA[2];
	ADC_raw_volts  	= ADCDataDMA[1];
	ADC_raw_current = ADCDataDMA[0];
#else
	ADC_raw_temp[1] = ADCDataDMA[3];
	ADC_raw_temp[0]	= ADCDataDMA[2];
	ADC_raw_volts 	= ADCDataDMA[1];
	ADC_raw_current	= ADCDataDMA[0];
#endif
}

int16_t computeTemperature(uint16_t raw_temp_val1, uint16_t raw_temp_val2)
{
	//Compute temperature with equation found in datasheet
	float tmp = (float)slopeFactor * ((bandgapCoefficient * ((float)raw_temp_val2 - (float)raw_temp_val1)) / ((float)raw_temp_val2 + (bandgapCoefficient * ((float)raw_temp_val2 - (float)raw_temp_val1)))) - offsetFactor;

	return (int16_t)tmp;
}

