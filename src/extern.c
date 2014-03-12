#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "extern.h"

volatile struct MotorData motor0 = {0, 0, 0, 0}, motor1 = {0, 0, 0, 0};
volatile struct EncoderData enc0 = {0, 0}, enc1 = {0, 0};
volatile struct UrfData urf0 = {0, 0, 9999}, urf1 = {0, 0, 9999};
volatile struct IrData ir0 = {0};
volatile struct JoystickData jstick0 = {5, 5, 0, 0};
volatile uint16_t adc0read[3] = {0, 0, 0};
volatile struct NavData nav0 = {0, 0};

//*****************************************************************************
// TABLES
//*****************************************************************************
//NOTE: Top and Bottom of joystick motion is stuck at max value 
//(ie we cannot get response past 70% of controller forward motion)
struct JsRange jsranges[11] = {
	{0, 30},	// Rev 5 -- When at full reverse position fluxuates up to 20
	{3, 500},	// Rev 4 -- Increase very quickly
	{420, 1100}, // Rev 3
	{980, 1450},	// Rev 2
	{1400, 2028},	// Rev 1
	{2000, 2168}, // Stop
	{2130, 2650}, // Fwd 1
	{2550, 3200}, // Fwd 2
	{3050, 3500}, // Fwd 3
	{3600, 4094}, // Fwd 4
	{4093, 4096} // Fwd 5
};
// inverse grey code map
char invgrey[4] = {0, 1, 3, 2};
