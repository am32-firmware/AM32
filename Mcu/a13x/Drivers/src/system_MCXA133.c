/*
** ###################################################################
**     Processors:          MCXA133VFM
**                          MCXA133VFT
**                          MCXA133VLF
**
**     Compilers:           GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    MCXA1 User manual
**     Version:             rev. 2.0, 2024-10-29
**     Build:               b260603
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2026 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2022-03-29)
**         Initial version based on v0.1UM
**     - rev. 2.0 (2024-10-29)
**         Change the device header file from single flat file to multiple files based on peripherals,
**         each peripheral with dedicated header file located in periphN folder.
**
** ###################################################################
*/

/*!
 * @file MCXA133
 * @version 2.0
 * @date 2024-10-29
 * @brief Device specific configuration file for MCXA133 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"

#if __has_include("fsl_clock.h")
#include "fsl_clock.h"
#endif


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInit (void) {

    // Initialize SRAM ECC - bypassed when running from RAM (debugger pre-initializes ECC).
    // Detection: (PC & 0x24000000) == 0 means flash address space.
    //   Flash:  0x00/0x08/0x10/0x18/0x80/0x90xxxxxx  -> bit29=0, bit26=0
    //   SRAM X: 0x04/0x14xxxxxx                       -> bit26 (0x04000000) set
    //   SRAM AB: 0x20/0x30xxxxxx                      -> bit29 (0x20000000) set
#if !defined(BYPASS_ECC_RAM_INIT)
    __asm volatile(
        "MOV     R1, PC                 \n"
        "TST     R1, #0x24000000        \n"
        "BNE     ecc_sram_skip         \n"
        "LDR     R0, =0x20000000        \n"
        "LDR     R1, =0x20002000        \n"
        "MOV     R2, #0                 \n"
        "MOV     R3, #0                 \n"
        "MOV     R4, #0                 \n"
        "MOV     R5, #0                 \n"
        "ecc_sram_loop:                \n"
        "STMIA   R0!, {R2-R5}           \n"
        "CMP     R0, R1                 \n"
        "BCC     ecc_sram_loop         \n"
        "ecc_sram_skip:                \n"
        :
        :
        : "r0", "r1", "r2", "r3", "r4", "r5");
#endif
  SCB->CPACR |= ((3UL << 0*2) | (3UL << 1*2));    /* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  SCB_NS->CPACR |= ((3UL << 0*2) | (3UL << 1*2));    /* set CP0, CP1 Full Access in Normal mode (enable PowerQuad) */
#endif /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

  SCB->NSACR |= ((3UL << 0) | (3UL << 10));   /* enable CP0, CP1, CP10, CP11 Non-secure Access */

#if !defined(__ZEPHYR__)
#if defined(__MCUXPRESSO)
    extern void(*const g_pfnVectors[]) (void);
    SCB->VTOR = (uint32_t) &g_pfnVectors;
#else
    extern void *__Vectors;
    SCB->VTOR = (uint32_t) &__Vectors;
#endif
#endif

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

    /* Enables flash speculation */
    SYSCON->NVM_CTRL &= ~(SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_MASK | SYSCON_NVM_CTRL_DIS_MBECC_ERR_INST_MASK);
    SYSCON->NVM_CTRL &= ~SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK;
  SystemInitHook();
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
#if __has_include("fsl_clock.h")
    /* Get frequency of Core System */
    SystemCoreClock = CLOCK_GetCoreSysClkFreq();
#endif
}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}
