/*!
 * @file       startup_g32f031.S
 *
 * @brief       G32F031 Devices vector table for GCC based toolchains.
 *              This module performs:
 *                  - Set the initial SP
 *                  - Set the initial PC == Reset_Handler,
 *                  - Set the vector table entries with the exceptions ISR address
 *                  - Branches to main in the C library (which eventually
 *                  calls main()).
 *              After Reset the Cortex-M4 processor is in Thread mode,
 *              priority is Privileged, and the Stack is set to Main.
 *
 * @version    V1.0.0
 *
 * @date       2026-01-15
 *
 * @attention
 *
 *  Copyright (C) 2026 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

  .syntax unified
  .cpu cortex-m0
  .fpu softvfp
  .thumb

.global  g_Vectors
.global  Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word  _start_address_init_data
/* start address for the .data section. defined in linker script */
.word  _start_address_data
/* end address for the .data section. defined in linker script */
.word  _end_address_data
/* start address for the .bss section. defined in linker script */
.word  _start_address_bss
/* end address for the .bss section. defined in linker script */
.word  _end_address_bss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
// Reset handler routine
Reset_Handler:
  ldr   r0, =_end_stack
  mov   sp, r0

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_start_address_data
  ldr r1, =_end_address_data
  ldr r2, =_start_address_init_data
  movs r3, #0
  b L_loop0_0

L_loop0:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

L_loop0_0:
  adds r4, r0, r3
  cmp r4, r1
  bcc L_loop0

  ldr r2, =_start_address_bss
  ldr r4, =_end_address_bss
  movs r3, #0
  b L_loop1

L_loop2:
  str  r3, [r2]
  adds r2, r2, #4

L_loop1:
  cmp r2, r4
  bcc L_loop2

  bl  SystemInit
  bl __libc_init_array
  bl  main
  bx  lr
.size  Reset_Handler, .-Reset_Handler

// This is the code that gets called when the processor receives an unexpected interrupt.
  .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
// The minimal vector table for a Cortex M0 Plus.
  .section  .isr_vector,"a",%progbits
  .type  g_Vectors, %object
  .size  g_Vectors, .-g_Vectors

// Vector Table Mapped to Address 0 at Reset
g_Vectors:
  .word  _end_stack                         // Top of Stack
  .word  Reset_Handler                      // Reset Handler
  .word  NMI_Handler                        // NMI Handler
  .word  HardFault_Handler                  // Hard Fault Handler
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  SVC_Handler                        // SVCall Handler
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  PendSV_Handler                     // PendSV Handler
  .word  SysTick_Handler                    // SysTick Handler

  /* External Interrupts */
  .word  WWDT_IRQHandler                    // Window Watchdog
  .word  PVD_IRQHandler                     // PVD through EINT Line detect
  .word  0                                  // Reserved
  .word  FLASH_IRQHandler                   // FLASH
  .word  RCC_IRQHandler                     // RCM
  .word  EINT0_1_IRQHandler                 // EINT Line 0 and 1
  .word  EINT2_3_IRQHandler                 // EINT Line 2 and 3
  .word  EINT4_15_IRQHandler                // EINT Line 4 to 15
  .word  0                                  // Reserved
  .word  DMA_CH0_IRQHandler                 // DMA Channel 0
  .word  DMA_CH1_IRQHandler                 // DMA Channel 1
  .word  0                                  // Reserved
  .word  ADC_IRQHandler                     // ADC
  .word  ATMR_BRK_UP_TRG_COM_IRQHandler     // Advanced Timer Break, Update, Trigger and Commutation
  .word  ATMR_CC_IRQHandler                 // Advanced Timer Capture Compare
  .word  GTMR_IRQHandler                    // General Timer
  .word  0                                  // 0
  .word  BTMR0_IRQHandler                   // Basic timer 0
  .word  BTMR1_IRQHandler                   // Basic timer 1
  .word  LPTMR_IRQHandler                   // Low power timer 1
  .word  0                                  // Reserved
  .word  COMP0_IRQHandler                   // COMP0
  .word  COMP1_2_3_IRQHandler               // COMP1/2/3
  .word  I2C_IRQHandler                     // I2C
  .word  0                                  // Reserved
  .word  SPI_IRQHandler                     // SPI
  .word  0                                  // Reserved
  .word  USART_IRQHandler                   // USART0
  .word  UART_IRQHandler                    // UART0
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved

// Default exception/interrupt handler

   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDT_IRQHandler
   .thumb_set WWDT_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler

   .weak      EINT0_1_IRQHandler
   .thumb_set EINT0_1_IRQHandler,Default_Handler

   .weak      EINT2_3_IRQHandler
   .thumb_set EINT2_3_IRQHandler,Default_Handler

   .weak      EINT4_15_IRQHandler
   .thumb_set EINT4_15_IRQHandler,Default_Handler

   .weak      DMA_CH0_IRQHandler
   .thumb_set DMA_CH0_IRQHandler,Default_Handler

   .weak      DMA_CH1_IRQHandler
   .thumb_set DMA_CH1_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      ATMR_BRK_UP_TRG_COM_IRQHandler
   .thumb_set ATMR_BRK_UP_TRG_COM_IRQHandler,Default_Handler

   .weak      ATMR_CC_IRQHandler
   .thumb_set ATMR_CC_IRQHandler,Default_Handler

   .weak      GTMR_IRQHandler
   .thumb_set GTMR_IRQHandler,Default_Handler

   .weak      BTMR0_IRQHandler
   .thumb_set BTMR0_IRQHandler,Default_Handler

   .weak      BTMR1_IRQHandler
   .thumb_set BTMR1_IRQHandler,Default_Handler

   .weak      LPTMR_IRQHandler
   .thumb_set LPTMR_IRQHandler,Default_Handler

   .weak      COMP0_IRQHandler
   .thumb_set COMP0_IRQHandler,Default_Handler

   .weak      COMP1_2_3_IRQHandler
   .thumb_set COMP1_2_3_IRQHandler,Default_Handler

   .weak      I2C_IRQHandler
   .thumb_set I2C_IRQHandler,Default_Handler

   .weak      SPI_IRQHandler
   .thumb_set SPI_IRQHandler,Default_Handler

   .weak      USART_IRQHandler
   .thumb_set USART_IRQHandler,Default_Handler

   .weak      UART_IRQHandler
   .thumb_set UART_IRQHandler,Default_Handler
