#ifndef __BOARD_CONFIG__
#define __BOARD_CONFIG__

#include <stdint.h>
#include <stdio.h>
#include "lm4f120h5qr.h"
#include "inc/hw_types.h"

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

#define PA0_U0_RX  PIN_0
#define PA1_U0_TX  PIN_1

#define PE4_U5_RX  PIN_4
#define PE5_U5_TX  PIN_5


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


#endif
