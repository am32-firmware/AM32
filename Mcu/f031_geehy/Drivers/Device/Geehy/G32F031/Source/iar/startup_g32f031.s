;/*!
; * @file       startup_g32f031xx.s
; *
; * @brief      CMSIS Cortex-M0 PLUS based Core Device Startup File
; *
; * @version    V1.0.0
; *
; * @date       2026-01-15
; *
; * @attention
; *
; *  Copyright (C) 2026 Geehy Semiconductor
; *
; *  You may not use this file except in compliance with the
; *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
; *
; *  The program is only for reference, which is distributed in the hope
; *  that it will be useful and instructional for customers to develop
; *  their software. Unless required by applicable law or agreed to in
; *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
; *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
; *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
; *  and limitations under the License.
; */

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)

                DCD     Reset_Handler                   ; Reset Handler
                DCD     NMI_Handler                     ; NMI Handler
                DCD     HardFault_Handler               ; Hard Fault Handler
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     SVC_Handler                     ; SVCall Handler
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     PendSV_Handler                  ; PendSV Handler
                DCD     SysTick_Handler                 ; SysTick Handler

                ; External Interrupts
                DCD     WWDT_IRQHandler                 ; Window Watchdog
                DCD     PVD_IRQHandler                  ; PVD through EINT Line detect
                DCD     0                               ; Reserved
                DCD     FLASH_IRQHandler                ; FLASH
                DCD     RCC_IRQHandler                  ; RCM
                DCD     EINT0_1_IRQHandler              ; EINT Line 0 and 1
                DCD     EINT2_3_IRQHandler              ; EINT Line 2 and 3
                DCD     EINT4_15_IRQHandler             ; EINT Line 4 to 15
                DCD     0                               ; Reserved
                DCD     DMA_CH0_IRQHandler              ; DMA Channel 0
                DCD     DMA_CH1_IRQHandler              ; DMA Channel 1
                DCD     0                               ; Reserved
                DCD     ADC_IRQHandler                  ; ADC
                DCD     ATMR_BRK_UP_TRG_COM_IRQHandler  ; Advanced Timer Break, Update, Trigger and Commutation
                DCD     ATMR_CC_IRQHandler              ; Advanced Timer Capture Compare
                DCD     GTMR_IRQHandler                 ; General Timer
                DCD     0                               ; 0
                DCD     BTMR0_IRQHandler                ; Basic timer 0
                DCD     BTMR1_IRQHandler                ; Basic timer 1
                DCD     LPTMR_IRQHandler                ; Low power timer 1
                DCD     0                               ; Reserved
                DCD     COMP0_IRQHandler                ; COMP0
                DCD     COMP1_2_3_IRQHandler            ; COMP1/2/3
                DCD     I2C_IRQHandler                  ; I2C
                DCD     0                               ; Reserved
                DCD     SPI_IRQHandler                  ; SPI
                DCD     0                               ; Reserved
                DCD     USART_IRQHandler                ; USART0
                DCD     UART_IRQHandler                 ; UART0
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers
;;
        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK  HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK  SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK  PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK  SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK  WWDT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WWDT_IRQHandler
        B WWDT_IRQHandler

        PUBWEAK  PVD_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PVD_IRQHandler
        B PVD_IRQHandler

        PUBWEAK  FLASH_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLASH_IRQHandler
        B FLASH_IRQHandler

        PUBWEAK  RCC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RCC_IRQHandler
        B RCC_IRQHandler

        PUBWEAK  EINT0_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EINT0_1_IRQHandler
        B EINT0_1_IRQHandler

        PUBWEAK  EINT2_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EINT2_3_IRQHandler
        B EINT2_3_IRQHandler

        PUBWEAK  EINT4_15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EINT4_15_IRQHandler
        B EINT4_15_IRQHandler

        PUBWEAK  DMA_CH0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_CH0_IRQHandler
        B DMA_CH0_IRQHandler

        PUBWEAK  DMA_CH1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA_CH1_IRQHandler
        B DMA_CH1_IRQHandler

        PUBWEAK  ADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_IRQHandler
        B ADC_IRQHandler

        PUBWEAK  ATMR_BRK_UP_TRG_COM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ATMR_BRK_UP_TRG_COM_IRQHandler
        B ATMR_BRK_UP_TRG_COM_IRQHandler

        PUBWEAK  ATMR_CC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ATMR_CC_IRQHandler
        B ATMR_CC_IRQHandler

        PUBWEAK  GTMR_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GTMR_IRQHandler
        B GTMR_IRQHandler

        PUBWEAK  BTMR0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTMR0_IRQHandler
        B BTMR0_IRQHandler

        PUBWEAK  BTMR1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BTMR1_IRQHandler
        B BTMR1_IRQHandler

        PUBWEAK  LPTMR_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LPTMR_IRQHandler
        B LPTMR_IRQHandler

        PUBWEAK  COMP0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP0_IRQHandler
        B COMP0_IRQHandler

        PUBWEAK  COMP1_2_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP1_2_3_IRQHandler
        B COMP1_2_3_IRQHandler

        PUBWEAK  I2C_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C_IRQHandler
        B I2C_IRQHandler

        PUBWEAK  SPI_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI_IRQHandler
        B SPI_IRQHandler

        PUBWEAK  USART_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USART_IRQHandler
        B USART_IRQHandler

        PUBWEAK  UART_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART_IRQHandler
        B UART_IRQHandler

        END
