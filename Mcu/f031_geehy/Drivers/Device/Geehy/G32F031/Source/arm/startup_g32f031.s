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

; <h> Stack Configuration
;  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp                    ; Top of Stack
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

__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler routine
Reset_Handler   PROC
                EXPORT  Reset_Handler                 [WEAK]
                IMPORT  __main
                IMPORT  SystemInit
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                    [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler              [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                    [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler                 [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler                [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDT_IRQHandler                 [WEAK]
                EXPORT  PVD_IRQHandler                  [WEAK]
                EXPORT  FLASH_IRQHandler                [WEAK]
                EXPORT  RCC_IRQHandler                  [WEAK]
                EXPORT  EINT0_1_IRQHandler              [WEAK]
                EXPORT  EINT2_3_IRQHandler              [WEAK]
                EXPORT  EINT4_15_IRQHandler             [WEAK]
                EXPORT  DMA_CH0_IRQHandler              [WEAK]
                EXPORT  DMA_CH1_IRQHandler              [WEAK]
                EXPORT  ADC_IRQHandler                  [WEAK]
                EXPORT  ATMR_BRK_UP_TRG_COM_IRQHandler  [WEAK]
                EXPORT  ATMR_CC_IRQHandler              [WEAK]
                EXPORT  GTMR_IRQHandler                 [WEAK]
                EXPORT  BTMR0_IRQHandler                [WEAK]
                EXPORT  BTMR1_IRQHandler                [WEAK]
                EXPORT  LPTMR_IRQHandler                [WEAK]
                EXPORT  COMP0_IRQHandler                [WEAK]
                EXPORT  COMP1_2_3_IRQHandler            [WEAK]
                EXPORT  I2C_IRQHandler                  [WEAK]
                EXPORT  SPI_IRQHandler                  [WEAK]
                EXPORT  USART_IRQHandler                [WEAK]
                EXPORT  UART_IRQHandler                 [WEAK]


WWDT_IRQHandler
PVD_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EINT0_1_IRQHandler
EINT2_3_IRQHandler
EINT4_15_IRQHandler
DMA_CH0_IRQHandler
DMA_CH1_IRQHandler
ADC_IRQHandler
ATMR_BRK_UP_TRG_COM_IRQHandler
ATMR_CC_IRQHandler
GTMR_IRQHandler
BTMR0_IRQHandler
BTMR1_IRQHandler
LPTMR_IRQHandler
COMP0_IRQHandler
COMP1_2_3_IRQHandler
I2C_IRQHandler
SPI_IRQHandler
USART_IRQHandler
UART_IRQHandler

                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB

                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit

                 ELSE

                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap

__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END
