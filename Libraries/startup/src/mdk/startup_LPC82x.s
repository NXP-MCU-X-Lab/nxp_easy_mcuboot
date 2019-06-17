;/**************************************************************************//**
; * @file     startup_LPC82x.s
; * @brief    CMSIS Core Device Startup File for
; *           NXP LPC82x Device Series
; * @version  V1.00
; * @date     19. August 2014
; *
; * @note
; * Copyright (C) 2014 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/

; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     SPI0_IRQHandler           ; 16+ 0  SPI0
                DCD     SPI1_IRQHandler           ; 16+ 1  SPI1
                DCD     0                         ; 16+ 2  Reserved
                DCD     UART0_IRQHandler          ; 16+ 3  UART0
                DCD     UART1_IRQHandler          ; 16+ 4  UART1
                DCD     UART2_IRQHandler          ; 16+ 5  UART2
                DCD     0                         ; 16+ 6  Reserved
                DCD     I2C1_IRQHandler           ; 16+ 7  I2C1
                DCD     I2C0_IRQHandler           ; 16+ 8  I2C0
                DCD     SCT_IRQHandler            ; 16+ 9  State configurable timer
                DCD     MRT_IRQHandler            ; 16+10  Multi-rate timer
                DCD     CMP_IRQHandler            ; 16+11  Analog comparator
                DCD     WDT_IRQHandler            ; 16+12  Windowed watchdog timer
                DCD     BOD_IRQHandler            ; 16+13  BOD
                DCD     FLASH_IRQHandler          ; 16+14  flash
                DCD     WKT_IRQHandler            ; 16+15  Self wake-up timer
                DCD     ADC_SEQA_IRQHandler       ; 16+16  ADC sequence A completion
                DCD     ADC_SEQB_IRQHandler       ; 16+17  ADC sequence B completion
                DCD     ADC_THCMP_IRQ             ; 16+18  ADC threshold compare
                DCD     ADC_OVR_IRQHandler        ; 16+19  ADC overrun
                DCD     DMA_IRQHandler            ; 16+20  DMA
                DCD     I2C2_IRQHandler           ; 16+21  I2C2
                DCD     I2C3_IRQHandler           ; 16+22  I2C3
                DCD     0                         ; 16+23  Reserved
                DCD     PIN_INT0_IRQHandler       ; 16+24  PIO INT0
                DCD     PIN_INT1_IRQHandler       ; 16+25  PIO INT1
                DCD     PIN_INT2_IRQHandler       ; 16+26  PIO INT2
                DCD     PIN_INT3_IRQHandler       ; 16+27  PIO INT3
                DCD     PIN_INT4_IRQHandler       ; 16+28  PIO INT4
                DCD     PIN_INT5_IRQHandler       ; 16+29  PIO INT5
                DCD     PIN_INT6_IRQHandler       ; 16+30  PIO INT6
                DCD     PIN_INT7_IRQHandler       ; 16+31  PIO INT7

; <h> Code Read Protection
;   <o> Code Read Protection  <0xFFFFFFFF=>CRP Disabled
;                             <0x12345678=>CRP Level 1
;                             <0x87654321=>CRP Level 2
;                             <0x43218765=>CRP Level 3 (ARE YOU SURE?)
;                             <0x4E697370=>NO ISP (ARE YOU SURE?)
; </h>
                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
                DCD     0xFFFFFFFF
                ENDIF

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  SCT_IRQHandler            [WEAK]
                EXPORT  MRT_IRQHandler            [WEAK]
                EXPORT  CMP_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  FLASH_IRQHandler          [WEAK]
                EXPORT  WKT_IRQHandler            [WEAK]
                EXPORT  ADC_SEQA_IRQHandler       [WEAK]
                EXPORT  ADC_SEQB_IRQHandler       [WEAK]
                EXPORT  ADC_THCMP_IRQ             [WEAK]
                EXPORT  ADC_OVR_IRQHandler        [WEAK]
                EXPORT  DMA_IRQHandler            [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  I2C3_IRQHandler           [WEAK]
                EXPORT  PIN_INT0_IRQHandler       [WEAK]
                EXPORT  PIN_INT1_IRQHandler       [WEAK]
                EXPORT  PIN_INT2_IRQHandler       [WEAK]
                EXPORT  PIN_INT3_IRQHandler       [WEAK]
                EXPORT  PIN_INT4_IRQHandler       [WEAK]
                EXPORT  PIN_INT5_IRQHandler       [WEAK]
                EXPORT  PIN_INT6_IRQHandler       [WEAK]
                EXPORT  PIN_INT7_IRQHandler       [WEAK]

SPI0_IRQHandler
SPI1_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
I2C1_IRQHandler
I2C0_IRQHandler
SCT_IRQHandler
MRT_IRQHandler
CMP_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FLASH_IRQHandler
WKT_IRQHandler
ADC_SEQA_IRQHandler
ADC_SEQB_IRQHandler
ADC_THCMP_IRQ
ADC_OVR_IRQHandler
DMA_IRQHandler
I2C2_IRQHandler
I2C3_IRQHandler
PIN_INT0_IRQHandler
PIN_INT1_IRQHandler
PIN_INT2_IRQHandler
PIN_INT3_IRQHandler
PIN_INT4_IRQHandler
PIN_INT5_IRQHandler
PIN_INT6_IRQHandler
PIN_INT7_IRQHandler

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

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
