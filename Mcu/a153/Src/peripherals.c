/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "peripherals.h"

void initCorePeripherals(void)
{
	SystemInit();
    SystemClock_Config();

    initGPIO();

	initFlexPWM();

#ifndef USE_ADC_INPUT
	initDshotPWMTimer();
	initDMA_DshotPWM();
	initSPI();
#endif
#ifndef BRUSHED_MODE
	initComTimer();
#endif
	initIntervalTimer();
	initDelayTimer();
	initTenKHzTimer();

#ifdef USE_ADC
	initADC();
	initDMA_ADC();
#endif

	initComp0();
	initComp1();

#ifdef USE_SERIAL_TELEMETRY
	telem_UART_Init();
	initDMA_UART();
#endif

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//TODO check if this is needed or what it does

}

void initAfterJump()
{
    __enable_irq();
}

/*
 * @brief	Initiate system. Enables read/write/execute access for flash.
 */
void SystemInit(void)
{
	SCB->CPACR |= ((3UL << 0*2) | (3UL << 1*2));    /* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
	SCB->NSACR |= ((3UL << 0) | (3UL << 10));   /* enable CP0, CP1, CP10, CP11 Non-secure Access */

	extern void *__Vectors;
	SCB->VTOR = (uint32_t) &__Vectors;

	/* Enable the LPCAC */
	SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_MASK;
	SYSCON->LPCAC_CTRL &= ~SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK;

	/* Enable flash RWX when FLASH_ACL in IFR0 is invalid */
	if ((*((volatile const uint32_t *)(0x1000000)) == 0xFFFFFFFFU) ||
		((*((volatile const uint32_t *)(0x1000000)) == 0x59630000U) &&
		 (*((volatile const uint32_t *)(0x1000040)) == 0xFFFFFFFFU) &&
		 (*((volatile const uint32_t *)(0x1000044)) == 0xFFFFFFFFU)))
	{
		/* Enable MBC register written with GLIKEY index15 */
		GLIKEY0->CTRL_0 = 0x00060000U;
		GLIKEY0->CTRL_0 = 0x0002000FU;
		GLIKEY0->CTRL_0 = 0x0001000FU;
		GLIKEY0->CTRL_1 = 0x00290000U;
		GLIKEY0->CTRL_0 = 0x0002000FU;
		GLIKEY0->CTRL_1 = 0x00280000U;
		GLIKEY0->CTRL_0 = 0x0000000FU;

		/* Enable RWX for GLBAC0 */
		MBC0->MBC_INDEX[0].MBC_MEMN_GLBAC[0] = 0x7700U;

		/* Use GLBAC0 for all flash block */
		for (uint8_t i = 0; i < 2U; i++)
		{
			MBC0->MBC_INDEX[0].MBC_DOM0_MEM0_BLK_CFG_W[i] = 0x00000000U;
		}

		/* Disable MBC register written */
		GLIKEY0->CTRL_0 = 0x0002000FU;
	}

	/* Route the PMC bandgap buffer signal to the ADC */
	SPC0->CORELDO_CFG |= (1U << 24U);

	uint32_t status = 0;
	memset(&s_flashDriver, 0, sizeof(flash_config_t));

	//Check if init went successful
	status = FLASH_API->flash_init(&s_flashDriver);
	if (status) {
		__asm volatile ("nop");
	}

	//Disable cache function
	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_DIS_LPCAC(1));
}

/*
 * @brief 	Configures the core voltage to 1.1V. Sets the Fast Internal Reference Clock (FIRC) to 192MHz.
 * 			Sets MUX to select FIRC as MAIN_CLK. Sets system clock divider to /2, so CPU_CLK and SYSTEM_CLK are 96MHz.
 * 			Also enables SIRC so 12MHz and 1MHz can be sourced to peripherals.
 */
void SystemClock_Config(void)
{
	//Set VDD_CORE voltage level to 1.1V to set it in Standard Drive mode
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_LVL(2));

	//Set VDD_CORE drive strength to normal (1.1V)
	modifyReg32(&SPC0->ACTIVE_CFG, SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK, SPC_ACTIVE_CFG_CORELDO_VDD_DS(1));

	//Wait for the SPC to finish its transition to 1.1V, i.e. SPC is not busy
	while ((SPC0->SC & SPC_SC_BUSY_MASK) >> SPC_SC_BUSY_SHIFT) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Set flash memory to support higher voltage level and frequency
	//This sets the number of additional wait-states.
	//These need to be set according to the set FIRC frequency:
	//48MHz = 0
	//64MHz = 0
	//96MHz = 1
	//192MHz = 2
	//Set to 192MHz
	modifyReg32(&FMU0->FCTRL, FMU_FCTRL_RWSC_MASK, FMU_FCTRL_RWSC(2));

	//Set SRAM to support higher voltage levels
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_VSM_MASK, SPC_SRAMCTL_VSM(2));

	//Request SRAM voltage update
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, SPC_SRAMCTL_REQ(1));

	//Wait for the SRAM voltage change to complete
	while (!((SPC0->SRAMCTL & SPC_SRAMCTL_ACK_MASK) >> SPC_SRAMCTL_ACK_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Clear the SRAM voltage update request
	modifyReg32(&SPC0->SRAMCTL, SPC_SRAMCTL_REQ_MASK, 0);

	//Set System clock divider to 2 to make the CPU and SYSTEM clock 96MHz (this is the max clock) divider value = DIV + 1
	modifyReg32(&SYSCON->AHBCLKDIV, SYSCON_AHBCLKDIV_DIV_MASK, SYSCON_AHBCLKDIV_DIV(1));

	/* Config FIRC */
	//Set the Fast Internal Reference Clock (FIRC) to 192MHz
	modifyReg32(&SCG0->FIRCCFG, SCG_FIRCCFG_FREQ_SEL_MASK, SCG_FIRCCFG_FREQ_SEL(7));

	//Unlock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_LK_MASK, 0);

	//Enable FRO_HF clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN(1));

	//Enable FIRC 48MHz clock to peripherals
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK, SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN(1));

	//Set that FIRC is disabled when in deep sleep mode
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCSTEN_MASK, SCG_FIRCCSR_FIRCSTEN(0));

	//Wait for the FIRC clock source to be valid
	while (!((SCG0->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Select FIRC as MAIN_CLK clock source
	modifyReg32(&SCG0->RCCR, SCG_RCCR_SCS_MASK, SCG_RCCR_SCS(3));

	//Wait for the MAIN_CLK clock source mux to be set correctly
	while (((SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Enable FIRC clock source
	modifyReg32(&SCG0->FIRCCSR, SCG_FIRCCSR_FIRCEN_MASK, SCG_FIRCCSR_FIRCEN(1));

	//Lock FIRC control status register
	modifyReg32(&SCG0->FIRCCSR, 0, SCG_FIRCCSR_LK_MASK);

	/* Config SIRC */
	//Unlock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_LK_MASK, 0);

	//Set that SIRC is disabled in deep sleep mode
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRCSTEN_MASK, 0);

	//Enable SIRC clock to peripherals
	modifyReg32(&SCG0->SIRCCSR, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK, SCG_SIRCCSR_SIRC_CLK_PERIPH_EN(1));

	//Wait for SIRC clock source to be valid
	while (!((SCG0->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT)) {
		//Do nothing
		__asm volatile ("nop");
	}

	//Lock SIRC control status register
	modifyReg32(&SCG0->SIRCCSR, 0, SCG_SIRCCSR_LK_MASK);
}

/*
 * @brief 	Enables all GPIO and PORT peripherals
 */
void initGPIO(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable PORT peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT0(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT1(1);
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_PORT2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_PORT3(1);

	//Enable GPIO peripheral clocks
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO0(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO1(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO2(1);
	MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_GPIO3(1);

	//Enable INPUTMUX peripheral clock
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Release PORT peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT0(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT1(1);
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_PORT2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3(1);

	//Release GPIO peripherals from reset
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO0(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO1(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO2(1);
	MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_GPIO3(1);

	//Release INPUTMUX peripheral from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_INPUTMUX0(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Enable GPIO pins for testing/debugging. P3.27, P2.17, P3.28. Set them to output
	modifyReg32(&PORT3->PCR[27],	//ENC_A
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 27));
	GPIO3->PCOR = (1 << 27);

	//pin 12, 13 and 14 cannot be used as these are occupied by USB FS on the MCXA14x and MCXA15x
//	modifyReg32(&PORT2->PCR[17],	//ENC_B
//			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
//			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
//	modifyReg32(&GPIO2->PDDR, 0, (1 << 17));
//	GPIO2->PCOR = (1 << 17);

	modifyReg32(&PORT3->PCR[28],	//ENC_I
			PORT_PCR_MUX_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(0) | PORT_PCR_PE(1) | PORT_PCR_PS(0));
	modifyReg32(&GPIO3->PDDR, 0, (1 << 28));
	GPIO3->PCOR = (1 << 28);
}

/*
 * @brief 	Initializes LPSPI0 module for sending Dshot telemetry.
 * 			Set LPSPI functional clock to 12MHz.
 * 			SPI bitrate should be 5/4 x Dshot bit rate.
 * 			So on DSHOT600 the 21 GCR bits are sent with a bitrate of 750kbit/s.
 */
void initSPI(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Select FRO_HF as clock for LPSPI0, which is 192MHz, see SystemClock_Config()
//	modifyReg32(&MRCC0->MRCC_LPSPI0_CLKSEL, MRCC_MRCC_LPSPI0_CLKSEL_MUX_MASK, MRCC_MRCC_LPSPI0_CLKSEL_MUX(1));
	modifyReg32(&MRCC0->MRCC_LPSPI0_CLKSEL, MRCC_MRCC_LPSPI0_CLKSEL_MUX_MASK, MRCC_MRCC_LPSPI0_CLKSEL_MUX(0));

	//Enable LPSPI0 and set divider to 4, so clock frequency is 48MHz
//	modifyReg32(&MRCC0->MRCC_LPSPI0_CLKDIV,
//			MRCC_MRCC_LPSPI0_CLKDIV_HALT_MASK | MRCC_MRCC_LPSPI0_CLKDIV_DIV_MASK,
//			MRCC_MRCC_LPSPI0_CLKDIV_DIV(3));
	modifyReg32(&MRCC0->MRCC_LPSPI0_CLKDIV,
			MRCC_MRCC_LPSPI0_CLKDIV_HALT_MASK | MRCC_MRCC_LPSPI0_CLKDIV_DIV_MASK,
			MRCC_MRCC_LPSPI0_CLKDIV_DIV(0));

	//Enable peripheral clocks
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_LPSPI0(1);

	//Release peripherals from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_LPSPI0(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set that transfer complete flag generates interrupt
	modifyReg32(&LPSPI0->IER, 0, LPSPI_IER_TCIE(1));

	//Set that SIN is used for input and output data
	//Set LPSPI master operating mode
	//Set output data is 3-stated
	modifyReg32(&LPSPI0->CFGR1,
			LPSPI_CFGR1_PINCFG_MASK | LPSPI_CFGR1_MASTER_MASK | LPSPI_CFGR1_OUTCFG_MASK,
			LPSPI_CFGR1_PINCFG(3) | LPSPI_CFGR1_MASTER(1) | LPSPI_CFGR1_OUTCFG(1));

	//Set CCR
//	modifyReg32(&LPSPI0->CCR, 0, LPSPI_CCR_SCKDIV(1));
//	LPSPI0->CCR = LPSPI_CCR_SCKDIV(1) | LPSPI_CCR_DBT(1) | LPSPI_CCR_PCSSCK(1) | LPSPI_CCR_SCKPCS(1);
	LPSPI0->CCR = 0;

	//Set TX watermark to 1 word
//	modifyReg32(&LPSPI0->FCR, LPSPI_FCR_TXWATER_MASK, LPSPI_FCR_TXWATER(1));
	LPSPI0->FCR = LPSPI_FCR_TXWATER(0);

	//Set prescaler to 8, in the end it will be 16 due to an additional value doubling.
	//Baud rate will be 750kbit/s at 12MHz functional clock
	//Set frame size to 32.
	//Mask RX so we do not receive any data.
	modifyReg32(&LPSPI0->TCR,
			LPSPI_TCR_PRESCALE_MASK | LPSPI_TCR_FRAMESZ_MASK,
			LPSPI_TCR_PRESCALE(3) | LPSPI_TCR_FRAMESZ(20) | LPSPI_TCR_RXMSK(1));

	//Enable SPI module
	modifyReg32(&LPSPI0->CR, LPSPI_CR_MEN_MASK, LPSPI_CR_MEN(1));

	//Enable interrupt
	__NVIC_SetPriority(LPSPI0_IRQn, 1);	//set interrupt priority to 1
	__NVIC_EnableIRQ(LPSPI0_IRQn);
}

void enableCorePeripherals()
{
	//Enable PWM
	enableFlexPWM();

	//Enable the timers
#ifndef USE_ADC_INPUT
	enableDMA_DshotPWM();
	enableDshotPWMTimer();
#endif

#ifndef BRUSHED_MODE
	enableComTimer();
#endif
	enableTenKHzTimer();
	enableIntervalTimer();

	//Enable ADC
#ifdef USE_ADC
	enableADC();
	enableDMA_ADC();
#endif

	//Enable UART DMA
#ifdef USE_SERIAL_TELEMETRY
	enableDMA_UART();
	enable_telem_UART();
#endif
}
