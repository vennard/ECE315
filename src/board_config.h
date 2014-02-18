#ifndef __BOARD_CONFIG__
#define __BOARD_CONFIG__

#include <stdint.h>
#include <stdio.h>
#include "lm4f120h5qr.h"
#include "inc/hw_types.h"
#include "gpio.h"
#include "UART.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

#define PIN_0     (1 << 0)
#define PIN_1     (1 << 1)
#define PIN_2     (1 << 2)
#define PIN_3     (1 << 3)
#define PIN_4     (1 << 4)
#define PIN_5     (1 << 5)
#define PIN_6     (1 << 6)
#define PIN_7     (1 << 7)

#define NULL      0
#define DISABLED  0
#define NONE      0
#define PORT_CONTROL_DEFAULT


// ULTRA SONIC RANGE FINDERS
#define PB0_TRIG_0	PIN_0
#define PE1_ECHO_0	PIN_1

#define PE2_TRIG_1	PIN_2
#define PE3_ECHO_1	PIN_3


// IR RANGE FINDER
#define PE0_RANGE_0_IN	PIN_0

#define A0C3_RANGE_0_IN 3


// PS2 INPUT
#define PB1_PS2_BUTTON PIN_1

#define PB4_YPOS_IN     PIN_4
#define PB5_XPOS_IN     PIN_5

#define A0C10_YPOS_IN 10
#define A0C11_XPOS_IN 11


// UARTS
#define PA0_U0_RX  PIN_0
#define PA1_U0_TX  PIN_1


// ENCODERS
#define PB2_MOTOR_0_SA	PIN_2
#define PB3_MOTOR_0_SB	PIN_3

#define PB6_MOTOR_1_SA	PIN_6
#define PB7_MOTOR_1_SB	PIN_7


// MOTOR DRIVES
#define PF1_MOTOR_0_DIR PIN_1
#define PF2_MOTOR_0_EN  PIN_2
#define PF3_MOTOR_1_DIR PIN_3
#define PF4_MOTOR_1_EN  PIN_4


// JTAG
#define PC0_TCK       PIN_0
#define PC1_TMS       PIN_1
#define PC2_TDI       PIN_2
#define PC3_TDO       PIN_3

/******************************************************************************
 * Initialization Structures
 *****************************************************************************/

/******************************************************************************
 * PORT A
 *****************************************************************************/
GPIO_CONFIG portA_config = {
	(PA0_U0_RX | PA1_U0_TX),	// DigitalEnable
	NONE,			// Input
	NONE,			// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	DISABLED,		// AnalogEnable
	(PA0_U0_RX | PA1_U0_TX),	// AlternateFunctionEnable
	(GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX)	// PortControl
};

 /******************************************************************************
 * PORT B
 *****************************************************************************/
GPIO_CONFIG portB_config = {
	PB0_TRIG_0 | PB1_PS2_BUTTON | PB2_MOTOR_0_SA | PB3_MOTOR_0_SB | PB6_MOTOR_1_SA | PB7_MOTOR_1_SB,		// DigitalEnable
	PB1_PS2_BUTTON | PB2_MOTOR_0_SA | PB3_MOTOR_0_SB | PB4_YPOS_IN | PB5_XPOS_IN | PB6_MOTOR_1_SA | PB7_MOTOR_1_SB,	// Input
	PB0_TRIG_0,			// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	PB4_YPOS_IN | PB5_XPOS_IN,	// AnalogEnable
	PB4_YPOS_IN | PB5_XPOS_IN,	// AlternateFunctionEnable
	PORT_CONTROL_DEFAULT	// PortControl
};

 /******************************************************************************
 * PORT C
 *****************************************************************************/
GPIO_CONFIG portC_config = {
	(PC0_TCK | PC1_TMS | PC2_TDI | PC3_TDO),	// DigitalEnable
	NONE,			// Input
	NONE,			// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	(PC0_TCK | PC1_TMS | PC2_TDI | PC3_TDO)
	    ,			// PullUp
	DISABLED,		// AnalogEnable
	(PC0_TCK | PC1_TMS | PC2_TDI | PC3_TDO),	// AlternateFunctionEnable
	(GPIO_PCTL_PC0_TCK | GPIO_PCTL_PC1_TMS | GPIO_PCTL_PC2_TDI | GPIO_PCTL_PC3_TDO)	// PortControl
};

 /******************************************************************************
 * PORT D
 *****************************************************************************/
GPIO_CONFIG portD_config = {
	NONE,			// DigitalEnable
	NONE,			// Input
	NONE,			// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	DISABLED,		// AnalogEnable
	DISABLED,		// AlternateFunctionEnable
	PORT_CONTROL_DEFAULT	// PortControl
};

 /******************************************************************************
 * PORT E
 *****************************************************************************/
GPIO_CONFIG portE_config = {
	PE1_ECHO_0 | PE2_TRIG_1 | PE3_ECHO_1,			// DigitalEnable
	PE0_RANGE_0_IN | PE1_ECHO_0 | PE3_ECHO_1,			// Input
	PE2_TRIG_1,			// Output
	PE1_ECHO_0 | PE3_ECHO_1,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	PE1_ECHO_0 | PE3_ECHO_1,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	PE1_ECHO_0 | PE3_ECHO_1,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	PE0_RANGE_0_IN,		// AnalogEnable
	PE0_RANGE_0_IN,		// AlternateFunctionEnable
	PORT_CONTROL_DEFAULT	// PortControl
};

/******************************************************************************
 * PORT F
 *****************************************************************************/
GPIO_CONFIG portF_config = {
	PF1_MOTOR_0_DIR | PF2_MOTOR_0_EN | PF3_MOTOR_1_DIR | PF4_MOTOR_1_EN,	// DigitalEnable
	NONE,			// Input
	PF1_MOTOR_0_DIR | PF2_MOTOR_0_EN | PF3_MOTOR_1_DIR | PF4_MOTOR_1_EN,	// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	DISABLED,		// AnalogEnable
	DISABLED,		// AlternateFunctionEnable
	PORT_CONTROL_DEFAULT	// PortControl
};


/******************************************************************************
 * Initialization Structure for UART0 Comms
 *****************************************************************************/
UART_CONFIG UART0_config = {
	ENABLED,		// UART
	ENABLED,		// UARTRx
	ENABLED,		// UARTTx
	_8BITS,			// WordLen
	9600,			// BaudRate
	ENABLED,		// FIFO
	DISABLED,		// _2Stop
	DISABLED,		// Parity
	DISABLED,		// EvenParity
	DISABLED,		// HighSpeed
	RX_ONE_EIGHTH,		// RxIntFIFO
	TX_ONE_EIGHTH,		// TxIntFIFO
	ENABLED,		// RxIntMask
	ENABLED,		// TxIntMask
	DISABLED,		// RxTimeOutIntMask
	DISABLED,		// OvrrIntMask
	DISABLED,		// ParityErrIntMask
	DISABLED		// FramingErrIntMask  
};




#endif
