; <<< Use Configuration Wizard in Context Menu >>>
;******************************************************************************
;
; startup_rvmdk.S - Startup code for use with Keil's uVision.
;
; Copyright (c) 2005-2011 Texas Instruments Incorporated.  All rights reserved.
; Software License Agreement
; 
; Texas Instruments (TI) is supplying this software for use solely and
; exclusively on TI's microcontroller products. The software is owned by
; TI and/or its suppliers, and is protected under applicable copyright
; laws. You may not combine this software with "viral" open-source
; software in order to form a larger program.
; 
; THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
; NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
; NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
; CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
; DAMAGES, FOR ANY REASON WHATSOEVER.
; 
; This is part of revision 8264 of the EK-LM3S1968 Firmware Package.
;
;******************************************************************************

;******************************************************************************
;
; <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Stack   EQU     0x00001000

;******************************************************************************
;
; <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;
;******************************************************************************
Heap    EQU     0x00001000

;******************************************************************************
;
; Allocate space for the stack.
;
;******************************************************************************
        AREA    STACK, NOINIT, READWRITE, ALIGN=3
StackMem
        SPACE   Stack
__initial_sp

;******************************************************************************
;
; Allocate space for the heap.
;
;******************************************************************************
        AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
HeapMem
        SPACE   Heap
__heap_limit

;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************
        PRESERVE8

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************
        AREA    RESET, CODE, READONLY
        THUMB

;******************************************************************************
;
; External declaration for the interrupt handler used by the application.
;
;******************************************************************************
        EXTERN  UARTIntHandler
		EXTERN	SYSTICKIntHandler
		EXTERN 	TIMER0AIntHandler

;******************************************************************************
;
; The vector table.
;
;******************************************************************************
        EXPORT  __Vectors
__Vectors
        DCD     StackMem + Stack            ; Top of Stack				0
        DCD     Reset_Handler               ; Reset Handler				1
        DCD     NmiSR                       ; NMI Handler				2
        DCD     FaultISR                    ; Hard Fault Handler		3	
        DCD     IntDefaultHandler           ; The MPU fault handler		4
        DCD     IntDefaultHandler           ; The bus fault handler		5
        DCD     IntDefaultHandler           ; The usage fault handler	6
        DCD     0                           ; Reserved					7
        DCD     0                           ; Reserved					8
        DCD     0                           ; Reserved					9
        DCD     0                           ; Reserved					10
        DCD     IntDefaultHandler           ; SVCall handler			11
        DCD     IntDefaultHandler           ; Debug monitor handler		12
        DCD     0                           ; Reserved					13
        DCD     IntDefaultHandler           ; The PendSV handler		14
        DCD     SYSTICKIntHandler           ; The SysTick handler		15
        DCD     IntDefaultHandler           ; GPIO Port A				16
        DCD     IntDefaultHandler           ; GPIO Port B				17
        DCD     IntDefaultHandler           ; GPIO Port C				18
        DCD     IntDefaultHandler           ; GPIO Port D				19
        DCD     IntDefaultHandler           ; GPIO Port E				20
        DCD     IntDefaultHandler              ; UART0 Rx and Tx			21
        DCD     IntDefaultHandler           ; UART1 Rx and Tx			22	
        DCD     IntDefaultHandler           ; SSI0 Rx and Tx			23		
        DCD     IntDefaultHandler           ; I2C0 Master and Slave		24
        DCD     IntDefaultHandler           ; PWM Fault					25
        DCD     IntDefaultHandler           ; PWM Generator 0			26
        DCD     IntDefaultHandler           ; PWM Generator 1			27
        DCD     IntDefaultHandler           ; PWM Generator 2			28
        DCD     IntDefaultHandler           ; Quadrature Encoder 0		29
        DCD     IntDefaultHandler           ; ADC Sequence 0			30
        DCD     IntDefaultHandler           ; ADC Sequence 1			31
        DCD     IntDefaultHandler           ; ADC Sequence 2			32	
        DCD     IntDefaultHandler           ; ADC Sequence 3			33
        DCD     IntDefaultHandler           ; Watchdog timer			34
        DCD     TIMER0AIntHandler           ; Timer 0 subtimer A		35
        DCD     IntDefaultHandler           ; Timer 0 subtimer B		36
        DCD     IntDefaultHandler           ; Timer 1 subtimer A		37
        DCD     IntDefaultHandler           ; Timer 1 subtimer B		38
        DCD     IntDefaultHandler           ; Timer 2 subtimer A		39
        DCD     IntDefaultHandler           ; Timer 2 subtimer B		40
        DCD     IntDefaultHandler           ; Analog Comparator 0		41
        DCD     IntDefaultHandler           ; Analog Comparator 1		42	
        DCD     IntDefaultHandler           ; Analog Comparator 2		43
        DCD     IntDefaultHandler           ; System Control (PLL, OSC, BO)	44
        DCD     IntDefaultHandler           ; FLASH Control				45
        DCD     IntDefaultHandler           ; GPIO Port F				46
        DCD     IntDefaultHandler           ; GPIO Port G				47
        DCD     IntDefaultHandler           ; GPIO Port H				48
        DCD     IntDefaultHandler           ; UART2 Rx and Tx			49
        DCD     IntDefaultHandler           ; SSI1 Rx and Tx			50
        DCD     IntDefaultHandler           ; Timer 3 subtimer A		51
        DCD     IntDefaultHandler           ; Timer 3 subtimer B		52
        DCD     IntDefaultHandler           ; I2C1 Master and Slave		53
        DCD     IntDefaultHandler           ; Quadrature Encoder 1		54
        DCD     IntDefaultHandler           ; CAN0						55
        DCD     IntDefaultHandler           ; CAN1						56
        DCD     IntDefaultHandler           ; CAN2						57
        DCD     IntDefaultHandler           ; Ethernet					58
        DCD     IntDefaultHandler           ; Hibernate					59

;******************************************************************************
;
; This is the code that gets called when the processor first starts execution
; following a reset event.
;
;******************************************************************************
        EXPORT  Reset_Handler
Reset_Handler
        ;
        ; Call the C library enty point that handles startup.  This will copy
        ; the .data section initializers from flash to SRAM and zero fill the
        ; .bss section.
        ;
        IMPORT  __main
        B       __main

;******************************************************************************
;
; This is the code that gets called when the processor receives a NMI.  This
; simply enters an infinite loop, preserving the system state for examination
; by a debugger.
;
;******************************************************************************
NmiSR
        B       NmiSR

;******************************************************************************
;
; This is the code that gets called when the processor receives a fault
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
FaultISR
        B       FaultISR

;******************************************************************************
;
; This is the code that gets called when the processor receives an unexpected
; interrupt.  This simply enters an infinite loop, preserving the system state
; for examination by a debugger.
;
;******************************************************************************
IntDefaultHandler
        B       IntDefaultHandler

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Some code in the normal code section for initializing the heap and stack.
;
;******************************************************************************
        AREA    |.text|, CODE, READONLY


;******************************************************************************
;
; Useful functions.
;
;******************************************************************************
        EXPORT  DisableInterrupts
        EXPORT  EnableInterrupts
        EXPORT  StartCritical
        EXPORT  EndCritical
        EXPORT  WaitForInterrupt

;*********** DisableInterrupts ***************
; disable interrupts
; inputs:  none
; outputs: none
DisableInterrupts
        CPSID  I
        BX     LR

;*********** EnableInterrupts ***************
; disable interrupts
; inputs:  none
; outputs: none
EnableInterrupts
        CPSIE  I
        BX     LR

;*********** StartCritical ************************
; make a copy of previous I bit, disable interrupts
; inputs:  none
; outputs: previous I bit
StartCritical
        MRS    R0, PRIMASK  ; save old status
        CPSID  I            ; mask all (except faults)
        BX     LR

;*********** EndCritical ************************
; using the copy of previous I bit, restore I bit to previous value
; inputs:  previous I bit
; outputs: none
EndCritical
        MSR    PRIMASK, R0
        BX     LR

;*********** WaitForInterrupt ************************
; go to low power mode while waiting for the next interrupt
; inputs:  none
; outputs: none
WaitForInterrupt
        WFI
        BX     LR
		
		
;******************************************************************************
;
; The function expected of the C library startup code for defining the stack
; and heap memory locations.  For the C library version of the startup code,
; provide this function so that the C library initialization code can find out
; the location of the stack and heap.
;
;******************************************************************************
    IF :DEF: __MICROLIB
        EXPORT  __initial_sp
        EXPORT  __heap_base
        EXPORT  __heap_limit
    ELSE
        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap
__user_initial_stackheap
        LDR     R0, =HeapMem
        LDR     R1, =(StackMem + Stack)
        LDR     R2, =(HeapMem + Heap)
        LDR     R3, =StackMem
        BX      LR
    ENDIF

;******************************************************************************
;
; Make sure the end of this section is aligned.
;
;******************************************************************************
        ALIGN

;******************************************************************************
;
; Tell the assembler that we're done.
;
;******************************************************************************
        END
