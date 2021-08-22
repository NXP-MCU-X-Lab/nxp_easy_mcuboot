; * ---------------------------------------------------------------------------------------
; *  @file:    startup_MKE18F16.s
; *  @purpose: CMSIS Cortex-M4 Core Device Startup File
; *            MKE18F16
; *  @version: 2.0
; *  @date:    2015-12-3
; *  @build:   b160125
; * ---------------------------------------------------------------------------------------
; *
; * Copyright (c) 1997 - 2016 , Freescale Semiconductor, Inc.
; * All rights reserved.
; *
; * Redistribution and use in source and binary forms, with or without modification,
; * are permitted provided that the following conditions are met:
; *
; * o Redistributions of source code must retain the above copyright notice, this list
; *   of conditions and the following disclaimer.
; *
; * o Redistributions in binary form must reproduce the above copyright notice, this
; *   list of conditions and the following disclaimer in the documentation and/or
; *   other materials provided with the distribution.
; *
; * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
; *   contributors may be used to endorse or promote products derived from this
; *   software without specific prior written permission.
; *
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; *
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

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
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp  ; Top of Stack
                DCD     Reset_Handler  ; Reset Handler
                DCD     NMI_Handler                         ;NMI Handler
                DCD     HardFault_Handler                   ;Hard Fault Handler
                DCD     MemManage_Handler                   ;MPU Fault Handler
                DCD     BusFault_Handler                    ;Bus Fault Handler
                DCD     UsageFault_Handler                  ;Usage Fault Handler
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     SVC_Handler                         ;SVCall Handler
                DCD     DebugMon_Handler                    ;Debug Monitor Handler
                DCD     0                                   ;Reserved
                DCD     PendSV_Handler                      ;PendSV Handler
                DCD     SysTick_Handler                     ;SysTick Handler

                                                            ;External Interrupts
                DCD     DMA0_IRQHandler                     ;DMA channel 0 transfer complete
                DCD     DMA1_IRQHandler                     ;DMA channel 1 transfer complete
                DCD     DMA2_IRQHandler                     ;DMA channel 2 transfer complete
                DCD     DMA3_IRQHandler                     ;DMA channel 3 transfer complete
                DCD     DMA4_IRQHandler                     ;DMA channel 4 transfer complete
                DCD     DMA5_IRQHandler                     ;DMA channel 5 transfer complete
                DCD     DMA6_IRQHandler                     ;DMA channel 6 transfer complete
                DCD     DMA7_IRQHandler                     ;DMA channel 7 transfer complete
                DCD     DMA8_IRQHandler                     ;DMA channel 8 transfer complete
                DCD     DMA9_IRQHandler                     ;DMA channel 9 transfer complete
                DCD     DMA10_IRQHandler                    ;DMA channel 10 transfer complete
                DCD     DMA11_IRQHandler                    ;DMA channel 11 transfer complete
                DCD     DMA12_IRQHandler                    ;DMA channel 12 transfer complete
                DCD     DMA13_IRQHandler                    ;DMA channel 13 transfer complete
                DCD     DMA14_IRQHandler                    ;DMA channel 14 transfer complete
                DCD     DMA15_IRQHandler                    ;DMA channel 15 transfer complete
                DCD     DMA_Error_IRQHandler                ;DMA error interrupt channels 0-15
                DCD     MCM_IRQHandler                      ;FPU sources
                DCD     FTFE_IRQHandler                     ;FTFE command complete
                DCD     Read_Collision_IRQHandler           ;FTFE read collision
                DCD     LVD_LVW_IRQHandler                  ;PMC controller low-voltage detect, low-voltage warning
                DCD     Doublebit_Fault_IRQHandler          ;FTFE double bit fault detect
                DCD     WDOG_EWM_IRQHandler                 ;Single interrupt vector for WDOG and EWM
                DCD     Reserved39_IRQHandler               ;Reserved interrupt
                DCD     LPI2C0_IRQHandler                   ;Inter-integrated circuit 0
                DCD     LPI2C1_IRQHandler                   ;Inter-integrated circuit 1
                DCD     LPSPI0_IRQHandler                   ;Serial peripheral Interface 0
                DCD     LPSPI1_IRQHandler                   ;Serial peripheral Interface 1
                DCD     Reserved44_IRQHandler               ;Reserved interrupt
                DCD     PWT_IRQHandler                      ;PWT interrupt
                DCD     Reserved46_IRQHandler               ;Reserved interrupt
                DCD     LPUART0_TX_IRQHandler               ;LPUART0 transmit interrupt
                DCD     LPUART0_RX_IRQHandler               ;LPUART0 receive interrupt
                DCD     LPUART1_TX_IRQHandler               ;LPUART1 transmit interrupt
                DCD     LPUART1_RX_IRQHandler               ;LPUART1 receive interrupt
                DCD     LPUART2_TX_IRQHandler               ;LPUART2 transmit interrupt
                DCD     LPUART2_RX_IRQHandler               ;LPUART2 receive interrupt
                DCD     Reserved53_IRQHandler               ;Reserved interrupt
                DCD     Reserved54_IRQHandler               ;Reserved interrupt
                DCD     ADC0_IRQHandler                     ;ADC conversion complete interrupt
                DCD     CMP0_IRQHandler                     ;CMP0 interrupt
                DCD     CMP1_IRQHandler                     ;CMP1 interrupt
                DCD     FTM0_IRQHandler                     ;FTM0 single interrupt vector for all sources
                DCD     FTM1_IRQHandler                     ;FTM1 single interrupt vector for all sources
                DCD     FTM2_IRQHandler                     ;FTM2 single interrupt vector for all sources
                DCD     Reserved61_IRQHandler               ;Reserved interrupt
                DCD     RTC_IRQHandler                      ;RTC alarm interrupt
                DCD     RTC_Seconds_IRQHandler              ;RTC seconds interrupt
                DCD     LPIT0_IRQHandler                    ;LPIT overflow
                DCD     Reserved65_IRQHandler               ;Reserved interrupt
                DCD     Reserved66_IRQHandler               ;Reserved interrupt
                DCD     Reserved67_IRQHandler               ;Reserved interrupt
                DCD     PDB0_IRQHandler                     ;Programmable delay block
                DCD     Reserved69_IRQHandler               ;Reserved interrupt
                DCD     Reserved70_IRQHandler               ;Reserved interrupt
                DCD     Reserved71_IRQHandler               ;Reserved interrupt
                DCD     DAC0_IRQHandler                     ;Digital-to-analog converter 0
                DCD     SCG_RCM_IRQHandler                  ;SCG_RCM interrupt
                DCD     LPTMR0_IRQHandler                   ;Single interrupt vector for  Low Power Timer 0
                DCD     PORTA_IRQHandler                    ;Port A pin detect interrupt
                DCD     PORTB_IRQHandler                    ;Port B pin detect interrupt
                DCD     PORTC_IRQHandler                    ;Port C pin detect interrupt
                DCD     PORTD_IRQHandler                    ;Port D pin detect interrupt
                DCD     PORTE_IRQHandler                    ;Port E pin detect interrupt
                DCD     SWI_IRQHandler                      ;Software interrupt
                DCD     Reserved81_IRQHandler               ;Reserved interrupt
                DCD     Reserved82_IRQHandler               ;Reserved interrupt
                DCD     Reserved83_IRQHandler               ;Reserved interrupt
                DCD     PDB1_IRQHandler                     ;Programmable delay block
                DCD     FLEXIO_IRQHandler                   ;FLEXIO
                DCD     CMP2_IRQHandler                     ;CMP2 interrupt
                DCD     FTM3_IRQHandler                     ;FlexTimer module 3 fault, overflow and channels interrupt
                DCD     Reserved88_IRQHandler               ;Reserved interrupt
                DCD     ADC1_IRQHandler                     ;ADC conversion complete interrupt
                DCD     ADC2_IRQHandler                     ;ADC conversion complete interrupt
                DCD     Reserved91_IRQHandler               ;Reserved interrupt
                DCD     Reserved92_IRQHandler               ;Reserved interrupt
                DCD     PDB2_IRQHandler                     ;Programmable delay block
                DCD     CAN0_ORed_IRQHandler                ;can
                DCD     CAN0_Error_IRQHandler               ;can
                DCD     CAN0_Wake_Up_IRQHandler             ;can
                DCD     CAN0_ORed_Message_buffer_IRQHandler ;can
                DCD     CAN0_Reserved1_IRQHandler           ;can
                DCD     CAN0_Reserved2_IRQHandler           ;can
                DCD     CAN0_Reserved3_IRQHandler           ;can
                DCD     CAN1_ORed_IRQHandler                ;can
                DCD     CAN1_Error_IRQHandler               ;can
                DCD     CAN1_Wake_Up_IRQHandler             ;can
                DCD     CAN1_ORed_Message_buffer_IRQHandler ;can
                DCD     CAN1_Reserved1_IRQHandler           ;can
                DCD     CAN1_Reserved2_IRQHandler           ;can
                DCD     CAN1_Reserved3_IRQHandler           ;can
                DCD     DefaultISR                          ;108
                DCD     DefaultISR                          ;109
                DCD     DefaultISR                          ;110
                DCD     DefaultISR                          ;111
                DCD     DefaultISR                          ;112
                DCD     DefaultISR                          ;113
                DCD     DefaultISR                          ;114
                DCD     DefaultISR                          ;115
                DCD     DefaultISR                          ;116
                DCD     DefaultISR                          ;117
                DCD     DefaultISR                          ;118
                DCD     DefaultISR                          ;119
                DCD     DefaultISR                          ;120
                DCD     DefaultISR                          ;121
                DCD     DefaultISR                          ;122
                DCD     DefaultISR                          ;123
                DCD     DefaultISR                          ;124
                DCD     DefaultISR                          ;125
                DCD     DefaultISR                          ;126
                DCD     DefaultISR                          ;127
                DCD     DefaultISR                          ;128
                DCD     DefaultISR                          ;129
                DCD     DefaultISR                          ;130
                DCD     DefaultISR                          ;131
                DCD     DefaultISR                          ;132
                DCD     DefaultISR                          ;133
                DCD     DefaultISR                          ;134
                DCD     DefaultISR                          ;135
                DCD     DefaultISR                          ;136
                DCD     DefaultISR                          ;137
                DCD     DefaultISR                          ;138
                DCD     DefaultISR                          ;139
                DCD     DefaultISR                          ;140
                DCD     DefaultISR                          ;141
                DCD     DefaultISR                          ;142
                DCD     DefaultISR                          ;143
                DCD     DefaultISR                          ;144
                DCD     DefaultISR                          ;145
                DCD     DefaultISR                          ;146
                DCD     DefaultISR                          ;147
                DCD     DefaultISR                          ;148
                DCD     DefaultISR                          ;149
                DCD     DefaultISR                          ;150
                DCD     DefaultISR                          ;151
                DCD     DefaultISR                          ;152
                DCD     DefaultISR                          ;153
                DCD     DefaultISR                          ;154
                DCD     DefaultISR                          ;155
                DCD     DefaultISR                          ;156
                DCD     DefaultISR                          ;157
                DCD     DefaultISR                          ;158
                DCD     DefaultISR                          ;159
                DCD     DefaultISR                          ;160
                DCD     DefaultISR                          ;161
                DCD     DefaultISR                          ;162
                DCD     DefaultISR                          ;163
                DCD     DefaultISR                          ;164
                DCD     DefaultISR                          ;165
                DCD     DefaultISR                          ;166
                DCD     DefaultISR                          ;167
                DCD     DefaultISR                          ;168
                DCD     DefaultISR                          ;169
                DCD     DefaultISR                          ;170
                DCD     DefaultISR                          ;171
                DCD     DefaultISR                          ;172
                DCD     DefaultISR                          ;173
                DCD     DefaultISR                          ;174
                DCD     DefaultISR                          ;175
                DCD     DefaultISR                          ;176
                DCD     DefaultISR                          ;177
                DCD     DefaultISR                          ;178
                DCD     DefaultISR                          ;179
                DCD     DefaultISR                          ;180
                DCD     DefaultISR                          ;181
                DCD     DefaultISR                          ;182
                DCD     DefaultISR                          ;183
                DCD     DefaultISR                          ;184
                DCD     DefaultISR                          ;185
                DCD     DefaultISR                          ;186
                DCD     DefaultISR                          ;187
                DCD     DefaultISR                          ;188
                DCD     DefaultISR                          ;189
                DCD     DefaultISR                          ;190
                DCD     DefaultISR                          ;191
                DCD     DefaultISR                          ;192
                DCD     DefaultISR                          ;193
                DCD     DefaultISR                          ;194
                DCD     DefaultISR                          ;195
                DCD     DefaultISR                          ;196
                DCD     DefaultISR                          ;197
                DCD     DefaultISR                          ;198
                DCD     DefaultISR                          ;199
                DCD     DefaultISR                          ;200
                DCD     DefaultISR                          ;201
                DCD     DefaultISR                          ;202
                DCD     DefaultISR                          ;203
                DCD     DefaultISR                          ;204
                DCD     DefaultISR                          ;205
                DCD     DefaultISR                          ;206
                DCD     DefaultISR                          ;207
                DCD     DefaultISR                          ;208
                DCD     DefaultISR                          ;209
                DCD     DefaultISR                          ;210
                DCD     DefaultISR                          ;211
                DCD     DefaultISR                          ;212
                DCD     DefaultISR                          ;213
                DCD     DefaultISR                          ;214
                DCD     DefaultISR                          ;215
                DCD     DefaultISR                          ;216
                DCD     DefaultISR                          ;217
                DCD     DefaultISR                          ;218
                DCD     DefaultISR                          ;219
                DCD     DefaultISR                          ;220
                DCD     DefaultISR                          ;221
                DCD     DefaultISR                          ;222
                DCD     DefaultISR                          ;223
                DCD     DefaultISR                          ;224
                DCD     DefaultISR                          ;225
                DCD     DefaultISR                          ;226
                DCD     DefaultISR                          ;227
                DCD     DefaultISR                          ;228
                DCD     DefaultISR                          ;229
                DCD     DefaultISR                          ;230
                DCD     DefaultISR                          ;231
                DCD     DefaultISR                          ;232
                DCD     DefaultISR                          ;233
                DCD     DefaultISR                          ;234
                DCD     DefaultISR                          ;235
                DCD     DefaultISR                          ;236
                DCD     DefaultISR                          ;237
                DCD     DefaultISR                          ;238
                DCD     DefaultISR                          ;239
                DCD     DefaultISR                          ;240
                DCD     DefaultISR                          ;241
                DCD     DefaultISR                          ;242
                DCD     DefaultISR                          ;243
                DCD     DefaultISR                          ;244
                DCD     DefaultISR                          ;245
                DCD     DefaultISR                          ;246
                DCD     DefaultISR                          ;247
                DCD     DefaultISR                          ;248
                DCD     DefaultISR                          ;249
                DCD     DefaultISR                          ;250
                DCD     DefaultISR                          ;251
                DCD     DefaultISR                          ;252
                DCD     DefaultISR                          ;253
                DCD     DefaultISR                          ;254
                DCD     0xFFFFFFFF                          ; Reserved for user TRIM value
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

; <h> Flash Configuration
;   <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
;   <i> and security information that allows the MCU to restrict access to the FTFL module.
;   <h> Backdoor Comparison Key
;     <o0>  Backdoor Comparison Key 0.  <0x0-0xFF:2>
;     <o1>  Backdoor Comparison Key 1.  <0x0-0xFF:2>
;     <o2>  Backdoor Comparison Key 2.  <0x0-0xFF:2>
;     <o3>  Backdoor Comparison Key 3.  <0x0-0xFF:2>
;     <o4>  Backdoor Comparison Key 4.  <0x0-0xFF:2>
;     <o5>  Backdoor Comparison Key 5.  <0x0-0xFF:2>
;     <o6>  Backdoor Comparison Key 6.  <0x0-0xFF:2>
;     <o7>  Backdoor Comparison Key 7.  <0x0-0xFF:2>
BackDoorK0      EQU     0xFF
BackDoorK1      EQU     0xFF
BackDoorK2      EQU     0xFF
BackDoorK3      EQU     0xFF
BackDoorK4      EQU     0xFF
BackDoorK5      EQU     0xFF
BackDoorK6      EQU     0xFF
BackDoorK7      EQU     0xFF
;   </h>
;   <h> Program flash protection bytes (FPROT)
;     <i> Each program flash region can be protected from program and erase operation by setting the associated PROT bit.
;     <i> Each bit protects a 1/32 region of the program flash memory.
;     <h> FPROT0
;       <i> Program Flash Region Protect Register 0
;       <i> 1/32 - 8/32 region
;       <o.0>   FPROT0.0
;       <o.1>   FPROT0.1
;       <o.2>   FPROT0.2
;       <o.3>   FPROT0.3
;       <o.4>   FPROT0.4
;       <o.5>   FPROT0.5
;       <o.6>   FPROT0.6
;       <o.7>   FPROT0.7
nFPROT0         EQU     0x00
FPROT0          EQU     nFPROT0:EOR:0xFF
;     </h>
;     <h> FPROT1
;       <i> Program Flash Region Protect Register 1
;       <i> 9/32 - 16/32 region
;       <o.0>   FPROT1.0
;       <o.1>   FPROT1.1
;       <o.2>   FPROT1.2
;       <o.3>   FPROT1.3
;       <o.4>   FPROT1.4
;       <o.5>   FPROT1.5
;       <o.6>   FPROT1.6
;       <o.7>   FPROT1.7
nFPROT1         EQU     0x00
FPROT1          EQU     nFPROT1:EOR:0xFF
;     </h>
;     <h> FPROT2
;       <i> Program Flash Region Protect Register 2
;       <i> 17/32 - 24/32 region
;       <o.0>   FPROT2.0
;       <o.1>   FPROT2.1
;       <o.2>   FPROT2.2
;       <o.3>   FPROT2.3
;       <o.4>   FPROT2.4
;       <o.5>   FPROT2.5
;       <o.6>   FPROT2.6
;       <o.7>   FPROT2.7
nFPROT2         EQU     0x00
FPROT2          EQU     nFPROT2:EOR:0xFF
;     </h>
;     <h> FPROT3
;       <i> Program Flash Region Protect Register 3
;       <i> 25/32 - 32/32 region
;       <o.0>   FPROT3.0
;       <o.1>   FPROT3.1
;       <o.2>   FPROT3.2
;       <o.3>   FPROT3.3
;       <o.4>   FPROT3.4
;       <o.5>   FPROT3.5
;       <o.6>   FPROT3.6
;       <o.7>   FPROT3.7
nFPROT3         EQU     0x00
FPROT3          EQU     nFPROT3:EOR:0xFF
;     </h>
;   </h>
;   <h> Data flash protection byte (FDPROT)
;     <i> Each bit protects a 1/8 region of the data flash memory.
;     <i> (Program flash only devices: Reserved)
;       <o.0>   FDPROT.0
;       <o.1>   FDPROT.1
;       <o.2>   FDPROT.2
;       <o.3>   FDPROT.3
;       <o.4>   FDPROT.4
;       <o.5>   FDPROT.5
;       <o.6>   FDPROT.6
;       <o.7>   FDPROT.7
nFDPROT         EQU     0x00
FDPROT          EQU     nFDPROT:EOR:0xFF
;   </h>
;   <h> EEPROM protection byte (FEPROT)
;     <i> FlexNVM devices: Each bit protects a 1/8 region of the EEPROM.
;     <i> (Program flash only devices: Reserved)
;       <o.0>   FEPROT.0
;       <o.1>   FEPROT.1
;       <o.2>   FEPROT.2
;       <o.3>   FEPROT.3
;       <o.4>   FEPROT.4
;       <o.5>   FEPROT.5
;       <o.6>   FEPROT.6
;       <o.7>   FEPROT.7
nFEPROT         EQU     0x00
FEPROT          EQU     nFEPROT:EOR:0xFF
;   </h>
;   <h> Flash nonvolatile option byte (FOPT)
;     <i> Allows the user to customize the operation of the MCU at boot time.
;     <o.0> LPBOOT
;       <0=> Low-power boot
;       <1=> Normal boot
;     <o.1> BOOTPIN_OPT
;       <0=> Force Boot from ROM if BOOTCFG0 asserted, where BOOTCFG0 is the boot config function which is muxed with NMI pin
;       <1=> Boot source configured by FOPT (BOOTSRC_SEL) bits
;     <o.2> NMI_DIS
;       <0=> NMI interrupts are always blocked
;       <1=> NMI_b pin/interrupts reset default to enabled
;     <o.5> FAST_INIT
;       <0=> Slower initialization
;       <1=> Fast Initialization
;     <o.6..7> BOOTSRC_SEL
;       <0=> Boot from Flash
;       <2=> Boot from ROM
;       <3=> Boot from ROM
;         <i> Boot source selection
FOPT          EQU     0x3D
;   </h>
;   <h> Flash security byte (FSEC)
;     <i> WARNING: If SEC field is configured as "MCU security status is secure" and MEEN field is configured as "Mass erase is disabled",
;     <i> MCU's security status cannot be set back to unsecure state since Mass erase via the debugger is blocked !!!
;     <o.0..1> SEC
;       <2=> MCU security status is unsecure
;       <3=> MCU security status is secure
;         <i> Flash Security
;     <o.2..3> FSLACC
;       <2=> Freescale factory access denied
;       <3=> Freescale factory access granted
;         <i> Freescale Failure Analysis Access Code
;     <o.4..5> MEEN
;       <2=> Mass erase is disabled
;       <3=> Mass erase is enabled
;     <o.6..7> KEYEN
;       <2=> Backdoor key access enabled
;       <3=> Backdoor key access disabled
;         <i> Backdoor Key Security Enable
FSEC          EQU     0xFE
;   </h>
; </h>
                IF      :LNOT::DEF:RAM_TARGET
                AREA    |.ARM.__at_0x400|, CODE, READONLY
                DCB     BackDoorK0, BackDoorK1, BackDoorK2, BackDoorK3
                DCB     BackDoorK4, BackDoorK5, BackDoorK6, BackDoorK7
                DCB     FPROT0    , FPROT1    , FPROT2    , FPROT3
                DCB     FSEC      , FOPT      , FEPROT    , FDPROT
                ENDIF


                AREA    |.text|, CODE, READONLY

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                CPSID   I               ; Mask interrupts
                LDR     R0, =0xE000ED08
                LDR     R1, =__Vectors
                STR     R1, [R0]
                LDR     R0, =SystemInit
                BLX     R0
                CPSIE   i               ; Unmask interrupts
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler\
                PROC
                EXPORT  NMI_Handler         [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler         [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler\
                PROC
                EXPORT  SVC_Handler         [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler         [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler         [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler         [WEAK]
                B       .
                ENDP
DMA0_IRQHandler\
                PROC
                EXPORT  DMA0_IRQHandler         [WEAK]
                LDR     R0, =DMA0_DriverIRQHandler
                BX      R0
                ENDP

DMA1_IRQHandler\
                PROC
                EXPORT  DMA1_IRQHandler         [WEAK]
                LDR     R0, =DMA1_DriverIRQHandler
                BX      R0
                ENDP

DMA2_IRQHandler\
                PROC
                EXPORT  DMA2_IRQHandler         [WEAK]
                LDR     R0, =DMA2_DriverIRQHandler
                BX      R0
                ENDP

DMA3_IRQHandler\
                PROC
                EXPORT  DMA3_IRQHandler         [WEAK]
                LDR     R0, =DMA3_DriverIRQHandler
                BX      R0
                ENDP

DMA4_IRQHandler\
                PROC
                EXPORT  DMA4_IRQHandler         [WEAK]
                LDR     R0, =DMA4_DriverIRQHandler
                BX      R0
                ENDP

DMA5_IRQHandler\
                PROC
                EXPORT  DMA5_IRQHandler         [WEAK]
                LDR     R0, =DMA5_DriverIRQHandler
                BX      R0
                ENDP

DMA6_IRQHandler\
                PROC
                EXPORT  DMA6_IRQHandler         [WEAK]
                LDR     R0, =DMA6_DriverIRQHandler
                BX      R0
                ENDP

DMA7_IRQHandler\
                PROC
                EXPORT  DMA7_IRQHandler         [WEAK]
                LDR     R0, =DMA7_DriverIRQHandler
                BX      R0
                ENDP

DMA8_IRQHandler\
                PROC
                EXPORT  DMA8_IRQHandler         [WEAK]
                LDR     R0, =DMA8_DriverIRQHandler
                BX      R0
                ENDP

DMA9_IRQHandler\
                PROC
                EXPORT  DMA9_IRQHandler         [WEAK]
                LDR     R0, =DMA9_DriverIRQHandler
                BX      R0
                ENDP

DMA10_IRQHandler\
                PROC
                EXPORT  DMA10_IRQHandler         [WEAK]
                LDR     R0, =DMA10_DriverIRQHandler
                BX      R0
                ENDP

DMA11_IRQHandler\
                PROC
                EXPORT  DMA11_IRQHandler         [WEAK]
                LDR     R0, =DMA11_DriverIRQHandler
                BX      R0
                ENDP

DMA12_IRQHandler\
                PROC
                EXPORT  DMA12_IRQHandler         [WEAK]
                LDR     R0, =DMA12_DriverIRQHandler
                BX      R0
                ENDP

DMA13_IRQHandler\
                PROC
                EXPORT  DMA13_IRQHandler         [WEAK]
                LDR     R0, =DMA13_DriverIRQHandler
                BX      R0
                ENDP

DMA14_IRQHandler\
                PROC
                EXPORT  DMA14_IRQHandler         [WEAK]
                LDR     R0, =DMA14_DriverIRQHandler
                BX      R0
                ENDP

DMA15_IRQHandler\
                PROC
                EXPORT  DMA15_IRQHandler         [WEAK]
                LDR     R0, =DMA15_DriverIRQHandler
                BX      R0
                ENDP

DMA_Error_IRQHandler\
                PROC
                EXPORT  DMA_Error_IRQHandler         [WEAK]
                LDR     R0, =DMA_Error_DriverIRQHandler
                BX      R0
                ENDP

LPI2C0_IRQHandler\
                PROC
                EXPORT  LPI2C0_IRQHandler         [WEAK]
                LDR     R0, =LPI2C0_DriverIRQHandler
                BX      R0
                ENDP

LPI2C1_IRQHandler\
                PROC
                EXPORT  LPI2C1_IRQHandler         [WEAK]
                LDR     R0, =LPI2C1_DriverIRQHandler
                BX      R0
                ENDP

LPSPI0_IRQHandler\
                PROC
                EXPORT  LPSPI0_IRQHandler         [WEAK]
                LDR     R0, =LPSPI0_DriverIRQHandler
                BX      R0
                ENDP

LPSPI1_IRQHandler\
                PROC
                EXPORT  LPSPI1_IRQHandler         [WEAK]
                LDR     R0, =LPSPI1_DriverIRQHandler
                BX      R0
                ENDP

LPUART0_TX_IRQHandler\
                PROC
                EXPORT  LPUART0_TX_IRQHandler         [WEAK]
                LDR     R0, =LPUART0_TX_DriverIRQHandler
                BX      R0
                ENDP

LPUART0_RX_IRQHandler\
                PROC
                EXPORT  LPUART0_RX_IRQHandler         [WEAK]
                LDR     R0, =LPUART0_RX_DriverIRQHandler
                BX      R0
                ENDP

LPUART1_TX_IRQHandler\
                PROC
                EXPORT  LPUART1_TX_IRQHandler         [WEAK]
                LDR     R0, =LPUART1_TX_DriverIRQHandler
                BX      R0
                ENDP

LPUART1_RX_IRQHandler\
                PROC
                EXPORT  LPUART1_RX_IRQHandler         [WEAK]
                LDR     R0, =LPUART1_RX_DriverIRQHandler
                BX      R0
                ENDP

LPUART2_TX_IRQHandler\
                PROC
                EXPORT  LPUART2_TX_IRQHandler         [WEAK]
                LDR     R0, =LPUART2_TX_DriverIRQHandler
                BX      R0
                ENDP

LPUART2_RX_IRQHandler\
                PROC
                EXPORT  LPUART2_RX_IRQHandler         [WEAK]
                LDR     R0, =LPUART2_RX_DriverIRQHandler
                BX      R0
                ENDP

FLEXIO_IRQHandler\
                PROC
                EXPORT  FLEXIO_IRQHandler         [WEAK]
                LDR     R0, =FLEXIO_DriverIRQHandler
                BX      R0
                ENDP

CAN0_ORed_IRQHandler\
                PROC
                EXPORT  CAN0_ORed_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_Error_IRQHandler\
                PROC
                EXPORT  CAN0_Error_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_Wake_Up_IRQHandler\
                PROC
                EXPORT  CAN0_Wake_Up_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_ORed_Message_buffer_IRQHandler\
                PROC
                EXPORT  CAN0_ORed_Message_buffer_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_Reserved1_IRQHandler\
                PROC
                EXPORT  CAN0_Reserved1_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_Reserved2_IRQHandler\
                PROC
                EXPORT  CAN0_Reserved2_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN0_Reserved3_IRQHandler\
                PROC
                EXPORT  CAN0_Reserved3_IRQHandler         [WEAK]
                LDR     R0, =CAN0_DriverIRQHandler
                BX      R0
                ENDP

CAN1_ORed_IRQHandler\
                PROC
                EXPORT  CAN1_ORed_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_Error_IRQHandler\
                PROC
                EXPORT  CAN1_Error_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_Wake_Up_IRQHandler\
                PROC
                EXPORT  CAN1_Wake_Up_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_ORed_Message_buffer_IRQHandler\
                PROC
                EXPORT  CAN1_ORed_Message_buffer_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_Reserved1_IRQHandler\
                PROC
                EXPORT  CAN1_Reserved1_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_Reserved2_IRQHandler\
                PROC
                EXPORT  CAN1_Reserved2_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

CAN1_Reserved3_IRQHandler\
                PROC
                EXPORT  CAN1_Reserved3_IRQHandler         [WEAK]
                LDR     R0, =CAN1_DriverIRQHandler
                BX      R0
                ENDP

Default_Handler\
                PROC
                EXPORT  DMA0_DriverIRQHandler         [WEAK]
                EXPORT  DMA1_DriverIRQHandler         [WEAK]
                EXPORT  DMA2_DriverIRQHandler         [WEAK]
                EXPORT  DMA3_DriverIRQHandler         [WEAK]
                EXPORT  DMA4_DriverIRQHandler         [WEAK]
                EXPORT  DMA5_DriverIRQHandler         [WEAK]
                EXPORT  DMA6_DriverIRQHandler         [WEAK]
                EXPORT  DMA7_DriverIRQHandler         [WEAK]
                EXPORT  DMA8_DriverIRQHandler         [WEAK]
                EXPORT  DMA9_DriverIRQHandler         [WEAK]
                EXPORT  DMA10_DriverIRQHandler         [WEAK]
                EXPORT  DMA11_DriverIRQHandler         [WEAK]
                EXPORT  DMA12_DriverIRQHandler         [WEAK]
                EXPORT  DMA13_DriverIRQHandler         [WEAK]
                EXPORT  DMA14_DriverIRQHandler         [WEAK]
                EXPORT  DMA15_DriverIRQHandler         [WEAK]
                EXPORT  DMA_Error_DriverIRQHandler         [WEAK]
                EXPORT  MCM_IRQHandler         [WEAK]
                EXPORT  FTFE_IRQHandler         [WEAK]
                EXPORT  Read_Collision_IRQHandler         [WEAK]
                EXPORT  LVD_LVW_IRQHandler         [WEAK]
                EXPORT  Doublebit_Fault_IRQHandler         [WEAK]
                EXPORT  WDOG_EWM_IRQHandler         [WEAK]
                EXPORT  Reserved39_IRQHandler         [WEAK]
                EXPORT  LPI2C0_DriverIRQHandler         [WEAK]
                EXPORT  LPI2C1_DriverIRQHandler         [WEAK]
                EXPORT  LPSPI0_DriverIRQHandler         [WEAK]
                EXPORT  LPSPI1_DriverIRQHandler         [WEAK]
                EXPORT  Reserved44_IRQHandler         [WEAK]
                EXPORT  PWT_IRQHandler         [WEAK]
                EXPORT  Reserved46_IRQHandler         [WEAK]
                EXPORT  LPUART0_TX_DriverIRQHandler         [WEAK]
                EXPORT  LPUART0_RX_DriverIRQHandler         [WEAK]
                EXPORT  LPUART1_TX_DriverIRQHandler         [WEAK]
                EXPORT  LPUART1_RX_DriverIRQHandler         [WEAK]
                EXPORT  LPUART2_TX_DriverIRQHandler         [WEAK]
                EXPORT  LPUART2_RX_DriverIRQHandler         [WEAK]
                EXPORT  Reserved53_IRQHandler         [WEAK]
                EXPORT  Reserved54_IRQHandler         [WEAK]
                EXPORT  ADC0_IRQHandler         [WEAK]
                EXPORT  CMP0_IRQHandler         [WEAK]
                EXPORT  CMP1_IRQHandler         [WEAK]
                EXPORT  FTM0_IRQHandler         [WEAK]
                EXPORT  FTM1_IRQHandler         [WEAK]
                EXPORT  FTM2_IRQHandler         [WEAK]
                EXPORT  Reserved61_IRQHandler         [WEAK]
                EXPORT  RTC_IRQHandler         [WEAK]
                EXPORT  RTC_Seconds_IRQHandler         [WEAK]
                EXPORT  LPIT0_IRQHandler         [WEAK]
                EXPORT  Reserved65_IRQHandler         [WEAK]
                EXPORT  Reserved66_IRQHandler         [WEAK]
                EXPORT  Reserved67_IRQHandler         [WEAK]
                EXPORT  PDB0_IRQHandler         [WEAK]
                EXPORT  Reserved69_IRQHandler         [WEAK]
                EXPORT  Reserved70_IRQHandler         [WEAK]
                EXPORT  Reserved71_IRQHandler         [WEAK]
                EXPORT  DAC0_IRQHandler         [WEAK]
                EXPORT  SCG_RCM_IRQHandler         [WEAK]
                EXPORT  LPTMR0_IRQHandler         [WEAK]
                EXPORT  PORTA_IRQHandler         [WEAK]
                EXPORT  PORTB_IRQHandler         [WEAK]
                EXPORT  PORTC_IRQHandler         [WEAK]
                EXPORT  PORTD_IRQHandler         [WEAK]
                EXPORT  PORTE_IRQHandler         [WEAK]
                EXPORT  SWI_IRQHandler         [WEAK]
                EXPORT  Reserved81_IRQHandler         [WEAK]
                EXPORT  Reserved82_IRQHandler         [WEAK]
                EXPORT  Reserved83_IRQHandler         [WEAK]
                EXPORT  PDB1_IRQHandler         [WEAK]
                EXPORT  FLEXIO_DriverIRQHandler         [WEAK]
                EXPORT  CMP2_IRQHandler         [WEAK]
                EXPORT  FTM3_IRQHandler         [WEAK]
                EXPORT  Reserved88_IRQHandler         [WEAK]
                EXPORT  ADC1_IRQHandler         [WEAK]
                EXPORT  ADC2_IRQHandler         [WEAK]
                EXPORT  Reserved91_IRQHandler         [WEAK]
                EXPORT  Reserved92_IRQHandler         [WEAK]
                EXPORT  PDB2_IRQHandler         [WEAK]
                EXPORT  CAN0_DriverIRQHandler         [WEAK]
                EXPORT  CAN1_DriverIRQHandler         [WEAK]
                EXPORT  DefaultISR         [WEAK]
DMA0_DriverIRQHandler
DMA1_DriverIRQHandler
DMA2_DriverIRQHandler
DMA3_DriverIRQHandler
DMA4_DriverIRQHandler
DMA5_DriverIRQHandler
DMA6_DriverIRQHandler
DMA7_DriverIRQHandler
DMA8_DriverIRQHandler
DMA9_DriverIRQHandler
DMA10_DriverIRQHandler
DMA11_DriverIRQHandler
DMA12_DriverIRQHandler
DMA13_DriverIRQHandler
DMA14_DriverIRQHandler
DMA15_DriverIRQHandler
DMA_Error_DriverIRQHandler
MCM_IRQHandler
FTFE_IRQHandler
Read_Collision_IRQHandler
LVD_LVW_IRQHandler
Doublebit_Fault_IRQHandler
WDOG_EWM_IRQHandler
Reserved39_IRQHandler
LPI2C0_DriverIRQHandler
LPI2C1_DriverIRQHandler
LPSPI0_DriverIRQHandler
LPSPI1_DriverIRQHandler
Reserved44_IRQHandler
PWT_IRQHandler
Reserved46_IRQHandler
LPUART0_TX_DriverIRQHandler
LPUART0_RX_DriverIRQHandler
LPUART1_TX_DriverIRQHandler
LPUART1_RX_DriverIRQHandler
LPUART2_TX_DriverIRQHandler
LPUART2_RX_DriverIRQHandler
Reserved53_IRQHandler
Reserved54_IRQHandler
ADC0_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
Reserved61_IRQHandler
RTC_IRQHandler
RTC_Seconds_IRQHandler
LPIT0_IRQHandler
Reserved65_IRQHandler
Reserved66_IRQHandler
Reserved67_IRQHandler
PDB0_IRQHandler
Reserved69_IRQHandler
Reserved70_IRQHandler
Reserved71_IRQHandler
DAC0_IRQHandler
SCG_RCM_IRQHandler
LPTMR0_IRQHandler
PORTA_IRQHandler
PORTB_IRQHandler
PORTC_IRQHandler
PORTD_IRQHandler
PORTE_IRQHandler
SWI_IRQHandler
Reserved81_IRQHandler
Reserved82_IRQHandler
Reserved83_IRQHandler
PDB1_IRQHandler
FLEXIO_DriverIRQHandler
CMP2_IRQHandler
FTM3_IRQHandler
Reserved88_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
Reserved91_IRQHandler
Reserved92_IRQHandler
PDB2_IRQHandler
CAN0_DriverIRQHandler
CAN1_DriverIRQHandler
DefaultISR
                B      DefaultISR
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
