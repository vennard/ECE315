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

#define NULL                  0

#define PIN_0     (1 << 0)
#define PIN_1     (1 << 1)
#define PIN_2     (1 << 2)
#define PIN_3     (1 << 3)
#define PIN_4     (1 << 4)
#define PIN_5     (1 << 5)
#define PIN_6     (1 << 6)
#define PIN_7     (1 << 7)

#define A0C10_XPOS_IN 10
#define A0C11_YPOS_IN 11

#define PA0_U0_RX  PIN_0
#define PA1_U0_TX  PIN_1

#define PB1_PS2_BUTTON PIN_1

#define PB4_XPOS_IN     PIN_4
#define PB5_YPOS_IN     PIN_5

#define PF1_MOTOR_0_DIR PIN_1
#define PF2_MOTOR_0_EN  PIN_2
#define PF3_MOTOR_1_DIR PIN_3
#define PF4_MOTOR_1_EN  PIN_4

#define PC0_TCK       PIN_0
#define PC1_TMS       PIN_1
#define PC2_TDI       PIN_2
#define PC3_TDO       PIN_3

#define DISABLED  0
#define NONE      0
#define PORT_CONTROL_DEFAULT

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
	PB1_PS2_BUTTON,		// DigitalEnable
	PB1_PS2_BUTTON | PB4_XPOS_IN | PB5_YPOS_IN,	// Input
	NONE,			// Output
	DISABLED,		// InterruptEnable
	DISABLED,		// InterruptLevel
	DISABLED,		// InterruptLevelActiveHigh
	DISABLED,		// InterruptEdge
	DISABLED,		// InterruptEdgeRising
	DISABLED,		// InterruptEdgeBoth
	DISABLED,		// PullDown
	DISABLED,		// PullUp
	PB4_XPOS_IN | PB5_YPOS_IN,	// AnalogEnable
	PB4_XPOS_IN | PB5_YPOS_IN,	// AlternateFunctionEnable
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
