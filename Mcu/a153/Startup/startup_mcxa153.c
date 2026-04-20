//*****************************************************************************
// MCXA153 startup code for use with MCUXpresso IDE
//
// Version : 040724
//*****************************************************************************
//
// Copyright 2016-2024 NXP
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause
//*****************************************************************************

#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("Og")
#endif // (DEBUG)

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define WEAK_AV __attribute__ ((weak, section(".after_vectors")))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
//*****************************************************************************
//*****************************************************************************
// Declaration of external SystemInit function
//*****************************************************************************
#if defined (__USE_CMSIS)
extern void SystemInit(void);
#endif // (__USE_CMSIS)

//*****************************************************************************
// Forward declaration of the core exception handlers.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions.
// If your application is a C++ one, then any interrupt handlers defined
// in C++ files within in your main application will need to have C linkage
// rather than C++ linkage. To do this, make sure that you are using extern "C"
// { .... } around the interrupt handler within your main application code.
//*****************************************************************************
     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SecureFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
// Forward declaration of the application IRQ handlers. When the application
// defines a handler (with the same name), this will automatically take
// precedence over weak definitions below
//*****************************************************************************
WEAK void Reserved16_IRQHandler(void);
WEAK void CMC_IRQHandler(void);
WEAK void DMA_CH0_IRQHandler(void);
WEAK void DMA_CH1_IRQHandler(void);
WEAK void DMA_CH2_IRQHandler(void);
WEAK void DMA_CH3_IRQHandler(void);
WEAK void Reserved22_IRQHandler(void);
WEAK void Reserved23_IRQHandler(void);
WEAK void Reserved24_IRQHandler(void);
WEAK void Reserved25_IRQHandler(void);
WEAK void ERM0_SINGLE_BIT_IRQHandler(void);
WEAK void ERM0_MULTI_BIT_IRQHandler(void);
WEAK void FMU0_IRQHandler(void);
WEAK void GLIKEY0_IRQHandler(void);
WEAK void MBC0_IRQHandler(void);
WEAK void SCG0_IRQHandler(void);
WEAK void SPC0_IRQHandler(void);
WEAK void Reserved33_IRQHandler(void);
WEAK void WUU0_IRQHandler(void);
WEAK void Reserved35_IRQHandler(void);
WEAK void Reserved36_IRQHandler(void);
WEAK void Reserved37_IRQHandler(void);
WEAK void Reserved38_IRQHandler(void);
WEAK void Reserved39_IRQHandler(void);
WEAK void I3C0_IRQHandler(void);
WEAK void Reserved41_IRQHandler(void);
WEAK void LPI2C0_IRQHandler(void);
WEAK void Reserved43_IRQHandler(void);
WEAK void LPSPI0_IRQHandler(void);
WEAK void LPSPI1_IRQHandler(void);
WEAK void Reserved46_IRQHandler(void);
WEAK void LPUART0_IRQHandler(void);
WEAK void LPUART1_IRQHandler(void);
WEAK void LPUART2_IRQHandler(void);
WEAK void Reserved50_IRQHandler(void);
WEAK void Reserved51_IRQHandler(void);
WEAK void USB0_IRQHandler(void);
WEAK void Reserved53_IRQHandler(void);
WEAK void CDOG0_IRQHandler(void);
WEAK void CTIMER0_IRQHandler(void);
WEAK void CTIMER1_IRQHandler(void);
WEAK void CTIMER2_IRQHandler(void);
WEAK void Reserved58_IRQHandler(void);
WEAK void Reserved59_IRQHandler(void);
WEAK void FLEXPWM0_RELOAD_ERROR_IRQHandler(void);
WEAK void FLEXPWM0_FAULT_IRQHandler(void);
WEAK void FLEXPWM0_SUBMODULE0_IRQHandler(void);
WEAK void FLEXPWM0_SUBMODULE1_IRQHandler(void);
WEAK void FLEXPWM0_SUBMODULE2_IRQHandler(void);
WEAK void Reserved65_IRQHandler(void);
WEAK void QDC0_COMPARE_IRQHandler(void);
WEAK void QDC0_HOME_IRQHandler(void);
WEAK void QDC0_WATCHDOG_IRQHandler(void);
WEAK void QDC0_INDEX_IRQHandler(void);
WEAK void FREQME0_IRQHandler(void);
WEAK void LPTMR0_IRQHandler(void);
WEAK void Reserved72_IRQHandler(void);
WEAK void OS_EVENT_IRQHandler(void);
WEAK void WAKETIMER0_IRQHandler(void);
WEAK void UTICK0_IRQHandler(void);
WEAK void WWDT0_IRQHandler(void);
WEAK void Reserved77_IRQHandler(void);
WEAK void ADC0_IRQHandler(void);
WEAK void Reserved79_IRQHandler(void);
WEAK void CMP0_IRQHandler(void);
WEAK void CMP1_IRQHandler(void);
WEAK void Reserved82_IRQHandler(void);
WEAK void Reserved83_IRQHandler(void);
WEAK void Reserved84_IRQHandler(void);
WEAK void Reserved85_IRQHandler(void);
WEAK void Reserved86_IRQHandler(void);
WEAK void GPIO0_IRQHandler(void);
WEAK void GPIO1_IRQHandler(void);
WEAK void GPIO2_IRQHandler(void);
WEAK void GPIO3_IRQHandler(void);

WEAK void __libc_init_array(void);

//*****************************************************************************
// Forward declaration of the driver IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the driver
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//*****************************************************************************
void Reserved16_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_CH0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_CH1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_CH2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_CH3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved22_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved23_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved24_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved25_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ERM0_SINGLE_BIT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ERM0_MULTI_BIT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FMU0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GLIKEY0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void MBC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SCG0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved33_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WUU0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved35_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved36_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved37_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved38_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved39_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I3C0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved41_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPI2C0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved43_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPSPI0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPSPI1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved46_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved50_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved51_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved53_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CDOG0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved58_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved59_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXPWM0_RELOAD_ERROR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXPWM0_FAULT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXPWM0_SUBMODULE0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXPWM0_SUBMODULE1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXPWM0_SUBMODULE2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved65_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDC0_COMPARE_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDC0_HOME_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDC0_WATCHDOG_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDC0_INDEX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FREQME0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPTMR0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved72_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void OS_EVENT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WAKETIMER0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UTICK0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WWDT0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved77_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved79_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved82_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved83_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved84_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved85_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved86_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#endif
extern int main(void);

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
extern void _vStackTop(void);
extern void _vStackBase(void);
//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
// The vector table.
// This relies on the linker script to place at correct location in memory.
//*****************************************************************************

extern void (* const g_pfnVectors[])(void);
extern void * __Vectors __attribute__ ((alias ("g_pfnVectors")));

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM33
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
    NMI_Handler,                       // NMI Handler
    HardFault_Handler,                 // Hard Fault Handler
    MemManage_Handler,                 // MPU Fault Handler
    BusFault_Handler,                  // Bus Fault Handler
    UsageFault_Handler,                // Usage Fault Handler
    SecureFault_Handler,               // Secure Fault Handler
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    SVC_Handler,                       // SVCall Handler
    DebugMon_Handler,                  // Debug Monitor Handler
    0,                                 // Reserved
    PendSV_Handler,                    // PendSV Handler
    SysTick_Handler,                   // SysTick Handler

    // Chip Level - MCXA153
    Reserved16_IRQHandler,             // 16: Reserved interrupt
    CMC_IRQHandler,                    // 17: Core Mode Controller interrupt
    DMA_CH0_IRQHandler,                // 18: DMA3_0_CH0 error or transfer complete
    DMA_CH1_IRQHandler,                // 19: DMA3_0_CH1 error or transfer complete
    DMA_CH2_IRQHandler,                // 20: DMA3_0_CH2 error or transfer complete
    DMA_CH3_IRQHandler,                // 21: DMA3_0_CH3 error or transfer complete
    Reserved22_IRQHandler,             // 22: Reserved interrupt
    Reserved23_IRQHandler,             // 23: Reserved interrupt
    Reserved24_IRQHandler,             // 24: Reserved interrupt
    Reserved25_IRQHandler,             // 25: Reserved interrupt
    ERM0_SINGLE_BIT_IRQHandler,        // 26: ERM Single Bit error interrupt
    ERM0_MULTI_BIT_IRQHandler,         // 27: ERM Multi Bit error interrupt
    FMU0_IRQHandler,                   // 28: Flash Management Unit interrupt
    GLIKEY0_IRQHandler,                // 29: GLIKEY Interrupt
    MBC0_IRQHandler,                   // 30: MBC secure violation interrupt
    SCG0_IRQHandler,                   // 31: System Clock Generator interrupt
    SPC0_IRQHandler,                   // 32: System Power Controller interrupt
    Reserved33_IRQHandler,             // 33: Reserved interrupt
    WUU0_IRQHandler,                   // 34: Wake Up Unit interrupt
    Reserved35_IRQHandler,             // 35: Reserved interrupt
    Reserved36_IRQHandler,             // 36: Reserved interrupt
    Reserved37_IRQHandler,             // 37: Reserved interrupt
    Reserved38_IRQHandler,             // 38: Reserved interrupt
    Reserved39_IRQHandler,             // 39: Reserved interrupt
    I3C0_IRQHandler,                   // 40: Improved Inter Integrated Circuit interrupt 0
    Reserved41_IRQHandler,             // 41: Reserved interrupt
    LPI2C0_IRQHandler,                 // 42: Low-Power Inter Integrated Circuit interrupt
    Reserved43_IRQHandler,             // 43: Reserved interrupt
    LPSPI0_IRQHandler,                 // 44: Low-Power Serial Peripheral Interface interrupt
    LPSPI1_IRQHandler,                 // 45: Low-Power Serial Peripheral Interface interrupt
    Reserved46_IRQHandler,             // 46: Reserved interrupt
    LPUART0_IRQHandler,                // 47: Low-Power Universal Asynchronous Receive/Transmit interrupt
    LPUART1_IRQHandler,                // 48: Low-Power Universal Asynchronous Receive/Transmit interrupt
    LPUART2_IRQHandler,                // 49: Low-Power Universal Asynchronous Receive/Transmit interrupt
    Reserved50_IRQHandler,             // 50: Reserved interrupt
    Reserved51_IRQHandler,             // 51: Reserved interrupt
    USB0_IRQHandler,                   // 52: Universal Serial Bus - Full Speed interrupt
    Reserved53_IRQHandler,             // 53: Reserved interrupt
    CDOG0_IRQHandler,                  // 54: Code Watchdog Timer 0 interrupt
    CTIMER0_IRQHandler,                // 55: Standard counter/timer 0 interrupt
    CTIMER1_IRQHandler,                // 56: Standard counter/timer 1 interrupt
    CTIMER2_IRQHandler,                // 57: Standard counter/timer 2 interrupt
    Reserved58_IRQHandler,             // 58: Reserved interrupt
    Reserved59_IRQHandler,             // 59: Reserved interrupt
    FLEXPWM0_RELOAD_ERROR_IRQHandler,  // 60: FlexPWM0_reload_error interrupt
    FLEXPWM0_FAULT_IRQHandler,         // 61: FlexPWM0_fault interrupt
    FLEXPWM0_SUBMODULE0_IRQHandler,    // 62: FlexPWM0 Submodule 0 capture/compare/reload interrupt
    FLEXPWM0_SUBMODULE1_IRQHandler,    // 63: FlexPWM0 Submodule 1 capture/compare/reload interrupt
    FLEXPWM0_SUBMODULE2_IRQHandler,    // 64: FlexPWM0 Submodule 2 capture/compare/reload interrupt
    Reserved65_IRQHandler,             // 65: Reserved interrupt
    QDC0_COMPARE_IRQHandler,           // 66: Compare
    QDC0_HOME_IRQHandler,              // 67: Home
    QDC0_WATCHDOG_IRQHandler,          // 68: Watchdog / Simultaneous A and B Change
    QDC0_INDEX_IRQHandler,             // 69: Index / Roll Over / Roll Under
    FREQME0_IRQHandler,                // 70: Frequency Measurement interrupt
    LPTMR0_IRQHandler,                 // 71: Low Power Timer 0 interrupt
    Reserved72_IRQHandler,             // 72: Reserved interrupt
    OS_EVENT_IRQHandler,               // 73: OS event timer interrupt
    WAKETIMER0_IRQHandler,             // 74: Wake Timer Interrupt
    UTICK0_IRQHandler,                 // 75: Micro-Tick Timer interrupt
    WWDT0_IRQHandler,                  // 76: Windowed Watchdog Timer 0 interrupt
    Reserved77_IRQHandler,             // 77: Reserved interrupt
    ADC0_IRQHandler,                   // 78: Analog-to-Digital Converter interrupt
    Reserved79_IRQHandler,             // 79: Reserved interrupt
    CMP0_IRQHandler,                   // 80: Comparator interrupt
    CMP1_IRQHandler,                   // 81: Comparator interrupt
    Reserved82_IRQHandler,             // 82: Reserved interrupt
    Reserved83_IRQHandler,             // 83: Reserved interrupt
    Reserved84_IRQHandler,             // 84: Reserved interrupt
    Reserved85_IRQHandler,             // 85: Reserved interrupt
    Reserved86_IRQHandler,             // 86: Reserved interrupt
    GPIO0_IRQHandler,                  // 87: General Purpose Input/Output interrupt 0
    GPIO1_IRQHandler,                  // 88: General Purpose Input/Output interrupt 1
    GPIO2_IRQHandler,                  // 89: General Purpose Input/Output interrupt 2
    GPIO3_IRQHandler,                  // 90: General Purpose Input/Output interrupt 3
}; /* End of g_pfnVectors */

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors.init_data")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors.init_bss")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((naked, section(".after_vectors.reset")))
void ResetISR(void) {
    // Disable interrupts
    __asm volatile ("cpsid i");
    // Config VTOR & MSPLIM register
    __asm volatile ("LDR R0, =0xE000ED08  \n"
                    "STR %0, [R0]         \n"
                    "LDR R1, [%0]         \n"
                    "MSR MSP, R1          \n"
                    "MSR MSPLIM, %1       \n"
                    :
                    : "r"(g_pfnVectors), "r"(_vStackBase)
                    : "r0", "r1");

#if defined (__USE_CMSIS)
// If __USE_CMSIS defined, then call CMSIS SystemInit code
    SystemInit();

#endif // (__USE_CMSIS)

    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }

    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }

#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will setup the VTOR register

    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *)g_pfnVectors!=(unsigned int *) 0x00000000) {
        *pSCB_VTOR = (unsigned int)g_pfnVectors;
    }
#endif // (__USE_CMSIS)
#if defined (__cplusplus) //TODO TBD if C++ init is needed because technically AM32 isn't C++
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

    // Reenable interrupts
    __asm volatile ("cpsie i");

#if defined (__REDLIB__)
    // Call the Redlib library, which in turn calls main()
    __main();
#else
    main();
#endif

    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default core exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
WEAK_AV void NMI_Handler(void)
{ while(1) {}
}

WEAK_AV void HardFault_Handler(void)
{ while(1) {}
}

WEAK_AV void MemManage_Handler(void)
{ while(1) {}
}

WEAK_AV void BusFault_Handler(void)
{ while(1) {}
}

WEAK_AV void UsageFault_Handler(void)
{ while(1) {}
}

WEAK_AV void SecureFault_Handler(void)
{ while(1) {}
}

WEAK_AV void SVC_Handler(void)
{ while(1) {}
}

WEAK_AV void DebugMon_Handler(void)
{ while(1) {}
}

WEAK_AV void PendSV_Handler(void)
{ while(1) {}
}

WEAK_AV void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
WEAK_AV void IntDefaultHandler(void)
{ while(1) {}
}

//*****************************************************************************
// Default application exception handlers. Override the ones here by defining
// your own handler routines in your application code. These routines call
// driver exception handlers or IntDefaultHandler() if no driver exception
// handler is included.
//*****************************************************************************
WEAK void Reserved16_IRQHandler(void)
{   Reserved16_DriverIRQHandler();
}

WEAK void CMC_IRQHandler(void)
{   CMC_DriverIRQHandler();
}

WEAK void DMA_CH0_IRQHandler(void)
{   DMA_CH0_DriverIRQHandler();
}

WEAK void DMA_CH1_IRQHandler(void)
{   DMA_CH1_DriverIRQHandler();
}

WEAK void DMA_CH2_IRQHandler(void)
{   DMA_CH2_DriverIRQHandler();
}

WEAK void DMA_CH3_IRQHandler(void)
{   DMA_CH3_DriverIRQHandler();
}

WEAK void Reserved22_IRQHandler(void)
{   Reserved22_DriverIRQHandler();
}

WEAK void Reserved23_IRQHandler(void)
{   Reserved23_DriverIRQHandler();
}

WEAK void Reserved24_IRQHandler(void)
{   Reserved24_DriverIRQHandler();
}

WEAK void Reserved25_IRQHandler(void)
{   Reserved25_DriverIRQHandler();
}

WEAK void ERM0_SINGLE_BIT_IRQHandler(void)
{   ERM0_SINGLE_BIT_DriverIRQHandler();
}

WEAK void ERM0_MULTI_BIT_IRQHandler(void)
{   ERM0_MULTI_BIT_DriverIRQHandler();
}

WEAK void FMU0_IRQHandler(void)
{   FMU0_DriverIRQHandler();
}

WEAK void GLIKEY0_IRQHandler(void)
{   GLIKEY0_DriverIRQHandler();
}

WEAK void MBC0_IRQHandler(void)
{   MBC0_DriverIRQHandler();
}

WEAK void SCG0_IRQHandler(void)
{   SCG0_DriverIRQHandler();
}

WEAK void SPC0_IRQHandler(void)
{   SPC0_DriverIRQHandler();
}

WEAK void Reserved33_IRQHandler(void)
{   Reserved33_DriverIRQHandler();
}

WEAK void WUU0_IRQHandler(void)
{   WUU0_DriverIRQHandler();
}

WEAK void Reserved35_IRQHandler(void)
{   Reserved35_DriverIRQHandler();
}

WEAK void Reserved36_IRQHandler(void)
{   Reserved36_DriverIRQHandler();
}

WEAK void Reserved37_IRQHandler(void)
{   Reserved37_DriverIRQHandler();
}

WEAK void Reserved38_IRQHandler(void)
{   Reserved38_DriverIRQHandler();
}

WEAK void Reserved39_IRQHandler(void)
{   Reserved39_DriverIRQHandler();
}

WEAK void I3C0_IRQHandler(void)
{   I3C0_DriverIRQHandler();
}

WEAK void Reserved41_IRQHandler(void)
{   Reserved41_DriverIRQHandler();
}

WEAK void LPI2C0_IRQHandler(void)
{   LPI2C0_DriverIRQHandler();
}

WEAK void Reserved43_IRQHandler(void)
{   Reserved43_DriverIRQHandler();
}

WEAK void LPSPI0_IRQHandler(void)
{   LPSPI0_DriverIRQHandler();
}

WEAK void LPSPI1_IRQHandler(void)
{   LPSPI1_DriverIRQHandler();
}

WEAK void Reserved46_IRQHandler(void)
{   Reserved46_DriverIRQHandler();
}

WEAK void LPUART0_IRQHandler(void)
{   LPUART0_DriverIRQHandler();
}

WEAK void LPUART1_IRQHandler(void)
{   LPUART1_DriverIRQHandler();
}

WEAK void LPUART2_IRQHandler(void)
{   LPUART2_DriverIRQHandler();
}

WEAK void Reserved50_IRQHandler(void)
{   Reserved50_DriverIRQHandler();
}

WEAK void Reserved51_IRQHandler(void)
{   Reserved51_DriverIRQHandler();
}

WEAK void USB0_IRQHandler(void)
{   USB0_DriverIRQHandler();
}

WEAK void Reserved53_IRQHandler(void)
{   Reserved53_DriverIRQHandler();
}

WEAK void CDOG0_IRQHandler(void)
{   CDOG0_DriverIRQHandler();
}

WEAK void CTIMER0_IRQHandler(void)
{   CTIMER0_DriverIRQHandler();
}

WEAK void CTIMER1_IRQHandler(void)
{   CTIMER1_DriverIRQHandler();
}

WEAK void CTIMER2_IRQHandler(void)
{   CTIMER2_DriverIRQHandler();
}

WEAK void Reserved58_IRQHandler(void)
{   Reserved58_DriverIRQHandler();
}

WEAK void Reserved59_IRQHandler(void)
{   Reserved59_DriverIRQHandler();
}

WEAK void FLEXPWM0_RELOAD_ERROR_IRQHandler(void)
{   FLEXPWM0_RELOAD_ERROR_DriverIRQHandler();
}

WEAK void FLEXPWM0_FAULT_IRQHandler(void)
{   FLEXPWM0_FAULT_DriverIRQHandler();
}

WEAK void FLEXPWM0_SUBMODULE0_IRQHandler(void)
{   FLEXPWM0_SUBMODULE0_DriverIRQHandler();
}

WEAK void FLEXPWM0_SUBMODULE1_IRQHandler(void)
{   FLEXPWM0_SUBMODULE1_DriverIRQHandler();
}

WEAK void FLEXPWM0_SUBMODULE2_IRQHandler(void)
{   FLEXPWM0_SUBMODULE2_DriverIRQHandler();
}

WEAK void Reserved65_IRQHandler(void)
{   Reserved65_DriverIRQHandler();
}

WEAK void QDC0_COMPARE_IRQHandler(void)
{   QDC0_COMPARE_DriverIRQHandler();
}

WEAK void QDC0_HOME_IRQHandler(void)
{   QDC0_HOME_DriverIRQHandler();
}

WEAK void QDC0_WATCHDOG_IRQHandler(void)
{   QDC0_WATCHDOG_DriverIRQHandler();
}

WEAK void QDC0_INDEX_IRQHandler(void)
{   QDC0_INDEX_DriverIRQHandler();
}

WEAK void FREQME0_IRQHandler(void)
{   FREQME0_DriverIRQHandler();
}

WEAK void LPTMR0_IRQHandler(void)
{   LPTMR0_DriverIRQHandler();
}

WEAK void Reserved72_IRQHandler(void)
{   Reserved72_DriverIRQHandler();
}

WEAK void OS_EVENT_IRQHandler(void)
{   OS_EVENT_DriverIRQHandler();
}

WEAK void WAKETIMER0_IRQHandler(void)
{   WAKETIMER0_DriverIRQHandler();
}

WEAK void UTICK0_IRQHandler(void)
{   UTICK0_DriverIRQHandler();
}

WEAK void WWDT0_IRQHandler(void)
{   WWDT0_DriverIRQHandler();
}

WEAK void Reserved77_IRQHandler(void)
{   Reserved77_DriverIRQHandler();
}

WEAK void ADC0_IRQHandler(void)
{   ADC0_DriverIRQHandler();
}

WEAK void Reserved79_IRQHandler(void)
{   Reserved79_DriverIRQHandler();
}

WEAK void CMP0_IRQHandler(void)
{   CMP0_DriverIRQHandler();
}

WEAK void CMP1_IRQHandler(void)
{   CMP1_DriverIRQHandler();
}

WEAK void Reserved82_IRQHandler(void)
{   Reserved82_DriverIRQHandler();
}

WEAK void Reserved83_IRQHandler(void)
{   Reserved83_DriverIRQHandler();
}

WEAK void Reserved84_IRQHandler(void)
{   Reserved84_DriverIRQHandler();
}

WEAK void Reserved85_IRQHandler(void)
{   Reserved85_DriverIRQHandler();
}

WEAK void Reserved86_IRQHandler(void)
{   Reserved86_DriverIRQHandler();
}

WEAK void GPIO0_IRQHandler(void)
{   GPIO0_DriverIRQHandler();
}

WEAK void GPIO1_IRQHandler(void)
{   GPIO1_DriverIRQHandler();
}

WEAK void GPIO2_IRQHandler(void)
{   GPIO2_DriverIRQHandler();
}

WEAK void GPIO3_IRQHandler(void)
{   GPIO3_DriverIRQHandler();
}

//*****************************************************************************

#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)
