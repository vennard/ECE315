#include <stdint.h>
#include "board_config.h"
#include "gpio.h"
#include "extern.h"


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
	PE1_ECHO_0 | PE2_TRIG_1 | PE3_ECHO_1 | PE4_U5_RX | PE5_U5_TX,			// DigitalEnable
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
	PE0_RANGE_0_IN | PE4_U5_RX | PE5_U5_TX,		// AlternateFunctionEnable
	GPIO_PCTL_PE4_U5RX | GPIO_PCTL_PE5_U5TX	// PortControl
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
