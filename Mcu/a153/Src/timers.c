/*
 * timers.c
 *
 *  Created on: 13 Nov 2024
 *      Author: nxg09992
 */

#include "timers.h"
#include "functions.h"

/*
 * @brief 	Initializes the Watchdog which resets the chip after a timeout of 2 seconds
 * 			By default the Watchdog timer runs on a 1MHz clock
 */
void MX_IWDG_Init(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clock
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_CC0_WWDT0(1);

	//Enable Watchdog timer and set Watchdog clock divider
	modifyReg32(&MRCC0->MRCC_WWDT0_CLKDIV,
			MRCC_MRCC_WWDT0_CLKDIV_HALT_MASK | MRCC_MRCC_WWDT0_CLKDIV_DIV_MASK,
			MRCC_MRCC_WWDT0_CLKDIV_DIV(0));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set the watchdog timeout to 2s
	WWDT0->TC = WWDT_TC_COUNT(2000000 >> 2);	//divide by 4 cause of fixed clock divider of 4 inside WWDT

	//Set watchdog timeout to cause a reset
	modifyReg32(&WWDT0->MOD, WWDT_MOD_WDRESET_MASK, WWDT_MOD_WDRESET(1));

	//Enable watchdog timer
	modifyReg32(&WWDT0->MOD, WWDT_MOD_WDEN_MASK, WWDT_MOD_WDEN(1));

	//Write the correct feed sequence to enable the watchdog timer
	WWDT0->FEED = WWDT_FEED_FEED(0xaa);
	WWDT0->FEED = WWDT_FEED_FEED(0x55);
}

/*
 * @brief	Initializes CTIMER0 as a capture/compare timer to decode input PWM or input Dshot for setting the throttle.
 * 			This timer calls a DMA request at a match0 event.
 * 			The timer captures the rising and falling edge of the input pin.
 * 			On the falling edge the timer is cleared, which causes a match0 event that triggers a DMA0 request.
 * 			The DMA then transfers the capture 1 and capture 2 counter values to the DMA buffer.
 * 			To reconstruct the Dshot/PWM data a correction is done after the DMA buffer is fully filled.
 * 			CTIMER0 is configured to run at functional clock of 96MHz
 */
void initDshotPWMTimer(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Select FRO_HF as clock for CTIMER0, which is 192MHz, see SystemClock_Config()
	modifyReg32(&MRCC0->MRCC_CTIMER0_CLKSEL, MRCC_MRCC_CTIMER0_CLKSEL_MUX_MASK, MRCC_MRCC_CTIMER0_CLKSEL_MUX(1));

	//Enable CTIMER0 and set divider to 2, so clock frequency is 96MHz
	modifyReg32(&MRCC0->MRCC_CTIMER0_CLKDIV,
			MRCC_MRCC_CTIMER0_CLKDIV_HALT_MASK | MRCC_MRCC_CTIMER0_CLKDIV_DIV_MASK,
			MRCC_MRCC_CTIMER0_CLKDIV_DIV(1));

	//Enable CTIMER0 peripheral clock
	MRCC0->MRCC_GLB_CC0_SET = MRCC_MRCC_GLB_RST0_CTIMER0(1);

	//Release CTIMER0 from reset
	MRCC0->MRCC_GLB_RST0_SET = MRCC_MRCC_GLB_RST0_CTIMER0(1);

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set PWM/Dshot input pin to timer capture/compare input
	//Enable input buffer and disable pull-up/down resistor
	modifyReg32(&INPUT_PIN_PORT->PCR[INPUT_PIN],
			PORT_PCR_MUX_MASK | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK,
			PORT_PCR_MUX(INPUT_PIN_ALT_FUNC) | PORT_PCR_IBE(1) | PORT_PCR_PE(0) | PORT_PCR_PS(1));

	//Set PWM/Dshot input pin as TIMER0 capture input for capture register 1 and 2
	INPUTMUX0->CTIMER0CAP[1] = INPUT_PIN_CAPTURE_INP;
	INPUTMUX0->CTIMER0CAP[2] = INPUT_PIN_CAPTURE_INP;

	//Set prescaler value
	CTIMER0->PR = 0;

	//Set match0 value to zero
	CTIMER0->MR[0] = 0;

	//Set match1 value to higher then the minimum Dshot300 frame time which is around 53us, so take at least 53us.
	//Functional clock is 96MHz. So Dshot300 frame is around 5000 clock ticks. So set timeout value at 5500 clock ticks (57us).
	//Set match event to trigger at twice the frame time.
	//And correct for prescaler.
	CTIMER0->MR[1] = 11000 / (CTIMER0->PR + 1);

	//Enable interrupt on Match1 event
	modifyReg32(&CTIMER0->MCR, 0, CTIMER_MCR_MR1I(1));

	//Configure capture control register so capture value register is loaded on CR1 rising edge and CR2 falling edge
	modifyReg32(&CTIMER0->CCR,
			CTIMER_CCR_CAP1RE_MASK | CTIMER_CCR_CAP2RE_MASK | CTIMER_CCR_CAP1FE_MASK | CTIMER_CCR_CAP2FE_MASK,
			CTIMER_CCR_CAP1RE(1) | CTIMER_CCR_CAP2FE(1));

	//Clear timer counter on capture channel 2 falling edge
	modifyReg32(&CTIMER0->CTCR,
			CTIMER_CTCR_ENCC_MASK | CTIMER_CTCR_SELCC_MASK,
			CTIMER_CTCR_ENCC(1) | CTIMER_CTCR_SELCC(5));

	//Enable interrupt
	__NVIC_SetPriority(CTIMER0_IRQn, 1);	//set interrupt priority to 1
	__NVIC_EnableIRQ(CTIMER0_IRQn);
}

/*
 * @brief	Initializes CTIMER1 for use as the comparator time difference timer at 2MHz.
 * 			Initializes the reload frequency at 30Hz. Reload time is reconfigured through-out the program.
 */
void initComTimer(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	modifyReg32(&MRCC0->MRCC_GLB_CC0, MRCC_MRCC_GLB_CC0_CTIMER1_MASK, MRCC_MRCC_GLB_CC0_CTIMER1(1));

	//Release peripherals from reset
	modifyReg32(&MRCC0->MRCC_GLB_RST0, MRCC_MRCC_GLB_RST0_CTIMER1_MASK, MRCC_MRCC_GLB_RST0_CTIMER1(1));

	//Select functional clock for CTIMER1 to FRO_12M clock
	modifyReg32(&MRCC0->MRCC_CTIMER1_CLKSEL, MRCC_MRCC_CTIMER1_CLKSEL_MUX_MASK, MRCC_MRCC_CTIMER1_CLKSEL_MUX(0));

	//Enable CTIMER1 timer
	//And set CTIMER1 clock divider to /1
	modifyReg32(&MRCC0->MRCC_CTIMER1_CLKDIV,
			MRCC_MRCC_SYSTICK_CLKDIV_DIV_MASK | MRCC_MRCC_SYSTICK_CLKDIV_HALT(1),
			MRCC_MRCC_SYSTICK_CLKDIV_DIV(0));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set CTIMER1 prescaler to /6
	CTIMER1->PR = 5;

	//Enable reset on match0 event.
	//Do not yet enable interrupt on match0 event, as this causes the commutation to start and stagnate, drawing lots of current!
	modifyReg32(&CTIMER1->MCR,
				CTIMER_MCR_MR0R_MASK | CTIMER_MCR_MR0I_MASK,
				CTIMER_MCR_MR0R(1));

	//Set shadow match0 event at 30Hz
	CTIMER1->MR[0] = 65535;

	//Enable interrupt
	__NVIC_SetPriority(CTIMER1_IRQn, 0);	//set interrupt priority to 0
	__NVIC_EnableIRQ(CTIMER1_IRQn);
}

/*
 * @brief	Initializes CTIMER2 for use as an interval timer at 2MHz
 */
void initIntervalTimer(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	modifyReg32(&MRCC0->MRCC_GLB_CC0, MRCC_MRCC_GLB_CC0_CTIMER2_MASK, MRCC_MRCC_GLB_CC0_CTIMER2(1));

	//Release peripherals from reset
	modifyReg32(&MRCC0->MRCC_GLB_RST0, MRCC_MRCC_GLB_RST0_CTIMER2_MASK, MRCC_MRCC_GLB_RST0_CTIMER2(1));

	//Select functional clock for CTIMER2 to FRO_12M clock
	modifyReg32(&MRCC0->MRCC_CTIMER2_CLKSEL, MRCC_MRCC_CTIMER2_CLKSEL_MUX_MASK, MRCC_MRCC_CTIMER2_CLKSEL_MUX(0));

	//Enable CTIMER2 timer
	//And set CTIMER2 clock divider to /1
	modifyReg32(&MRCC0->MRCC_CTIMER2_CLKDIV,
			MRCC_MRCC_SYSTICK_CLKDIV_DIV_MASK | MRCC_MRCC_SYSTICK_CLKDIV_HALT(1),
			MRCC_MRCC_SYSTICK_CLKDIV_DIV(0));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set CTIMER2 prescaler to /6
	CTIMER2->PR = 5;
}

/*
 * @brief 	Initializes 32-bit micro-tick timer used for delay functions
 * 			Runs at 1MHz (clock source cannot be chosen), 1 tick = 1us
 * 			micro-tick timer counts down
 */
void initDelayTimer(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Enable peripheral clocks
	modifyReg32(&MRCC0->MRCC_GLB_CC0, MRCC_MRCC_GLB_CC0_UTICK0_MASK, MRCC_MRCC_GLB_CC0_UTICK0(1));

	//Release peripherals from reset
	modifyReg32(&MRCC0->MRCC_GLB_RST0, MRCC_MRCC_GLB_RST0_UTICK0_MASK, MRCC_MRCC_GLB_RST0_UTICK0(1));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));
}

/*
 * @brief 	Initializes the Low-power timer to generate a interrupt which calls the optional control loop with PID control etc.
 */
void initTenKHzTimer(void)
{
	//Unlock clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);

	//Select functional clock for Low-power timer to 1MHz clock (CLK_1M)
	modifyReg32(&MRCC0->MRCC_LPTMR0_CLKSEL, MRCC_MRCC_LPTMR0_CLKSEL_MUX_MASK, MRCC_MRCC_LPTMR0_CLKSEL_MUX(5));

	//Enable Low-power timer
	//And set Low-power timer clock divider to /1
	modifyReg32(&MRCC0->MRCC_LPTMR0_CLKDIV,
			MRCC_MRCC_LPTMR0_CLKDIV_DIV_MASK | MRCC_MRCC_LPTMR0_CLKDIV_HALT(1),
			MRCC_MRCC_LPTMR0_CLKDIV_DIV(0));

	//Freeze clock configuration registers access
	modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));

	//Set LPTMR clock to the one configured in MRCC_LPTMR0_CLKSEL
	modifyReg32(&LPTMR0->PSR, LPTMR_PSR_PCS_MASK, LPTMR_PSR_PCS(3));

	//Bypass the prescaler and glitch filter
	modifyReg32(&LPTMR0->PSR, LPTMR_PSR_PBYP_MASK, LPTMR_PSR_PBYP(1));

	//Set that low-power timer resets at compare event
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TFC_MASK, LPTMR_CSR_TFC(0));

	//Set compare/reload value such that it reloads at the loop frequency
	LPTMR0->CMR = 1000000 / LOOP_FREQUENCY_HZ;

	//Enable Low-power timer interrupt
	__NVIC_SetPriority(LPTMR0_IRQn, 3);	//set interrupt priority to 3
	__NVIC_EnableIRQ(LPTMR0_IRQn);
}

void enableDshotPWMTimer(void)
{
	//Reset timer counter
	modifyReg32(&CTIMER0->TCR, 0, CTIMER_TCR_CRST(1));
	delayMicros(2);
	modifyReg32(&CTIMER0->TCR, CTIMER_TCR_CRST_MASK, 0);

	//Enable timer counter
	modifyReg32(&CTIMER0->TCR, CTIMER_TCR_CEN_MASK, CTIMER_TCR_CEN(1));
}

void enableComTimer(void)
{
	//Reset timer counter
	modifyReg32(&CTIMER1->TCR, 0, CTIMER_TCR_CRST(1));
	delayMicros(2);
	modifyReg32(&CTIMER1->TCR, CTIMER_TCR_CRST_MASK, 0);

	//Enable CTIMER1
	modifyReg32(&CTIMER1->TCR, CTIMER_TCR_CEN_MASK, CTIMER_TCR_CEN(1));
}

void enableIntervalTimer(void)
{
	//Reset timer counter
	modifyReg32(&CTIMER2->TCR, 0, CTIMER_TCR_CRST(1));
	delayMicros(2);
	modifyReg32(&CTIMER2->TCR, CTIMER_TCR_CRST_MASK, 0);

	//Enable CTIMER2
	modifyReg32(&CTIMER2->TCR, CTIMER_TCR_CEN_MASK, CTIMER_TCR_CEN(1));
}

void enableTenKHzTimer(void)
{
	//Enable LPTMR interrupt
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TIE_MASK, LPTMR_CSR_TIE(1));

	//Enable low-power timer
	modifyReg32(&LPTMR0->CSR, LPTMR_CSR_TEN_MASK, LPTMR_CSR_TEN(1));
}

/*
 * @brief 	Resets the capture/compare timer used for Dshot/PWM (DshotPWMTimer)
 */
inline void resetInputCaptureTimer(void)
{
	//Reset timer counter
	//Synchronously resets Timer Counter and Prescaler after next positive edge of CTIMER function clock.
	//So, delay at least one operation when CPU clock is 96MHz
	modifyReg32(&CTIMER0->TCR, 0, CTIMER_TCR_CRST(1));
//	delayMicros(2);
	__asm volatile ("nop");
	modifyReg32(&CTIMER0->TCR, CTIMER_TCR_CRST_MASK, 0);
}

