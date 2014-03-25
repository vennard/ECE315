//*****************************************************************************
//
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "lm4f120h5qr.h"
#include "board_config.h"
#include "extern.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define ABS(x) (((x) > 0) ? (x) : -(x))
#define CLIP(x, y, z) (((y) > (z)) ? (z) : ((y) < (x)) ? (x) : (y))

#define PORTA   0x40004000
#define PORTB   0x40005000
#define PORTC   0x40006000
#define PORTD   0x40007000
#define PORTE   0x40024000
#define PORTF   0x40025000

#define UTICKS 232
#define PWM_WIDTH 100
#define NUM_SAMPLES 3
#define TEN_FEET_TICKS 3200

// 80MHz = 80000000Hz
#define CPU_HZ 80000000
// 1 / (200 uS) = 5kHz
#define SYSTICK_US 200
// 1 / (25 mS) = 40 Hz
#define TIMER0_MS 4
// 1 / (50 mS) = 20 Hz
#define TIMER1_MS 50

#define PID_MS 100

enum PIDparams {
	Pfactor = 500,
	Ifactor = 0,
	PidNorm = 1000
};

static volatile uint32_t systickdelay = 0;
static volatile char isrobot;

void resetEncs(void)
{
	DisableInterrupts();
	enc0.odo = enc1.odo = 0;
	EnableInterrupts();
}

void resetNav(void)
{
	DisableInterrupts();
	nav0.left = nav0.right = 0;
	motor0.targ =  motor1.targ = 0;
	motor0.errsum = motor1.errsum = 0;
	EnableInterrupts();
}

void handlePS2(void)
{
	uint16_t t;
	static uint16_t debounce = 0;
	
	debounce = (debounce << 1) | (GPIO_PORTB_DATA_R & PB1_PS2_BUTTON ? 1 : 0);
	if (debounce == 0x8000)
		jstick0.press = 1;
	else if (debounce == 0x7FFF)
		jstick0.release = 1;
	
	t = adc0read[0];
	if (t < jsranges[jstick0.y].bot)
		jstick0.y--;
	else if (t > jsranges[jstick0.y].top)
		jstick0.y++;
	
	t = adc0read[2];
	if (t < jsranges[jstick0.x].bot)
		jstick0.x--;
	else if (t > jsranges[jstick0.x].top)
		jstick0.x++;
	
	//jstick0.y = adc0read[0] - (0xFFF >> 1);
	//jstick0.x = adc0read[2] - (0xFFF >> 1);
}

void txPS2Struct(void)
{
	char *p;
	
	txXbUART("\x02");	// send STX
	for (p = (char *)&jstick0; p < (char *)(&jstick0 + 1); p++) {
		if (*p == '\x17')
			txXbUART("\x17");	// stuff ETB
		txDataXbUART(p, 1);
	}
	txXbUART("\x17");		// send ETB
}

void rxPS2Struct(void)
{
	static struct JoystickData shadow;
	static char *p = 0;
	static char gotETB = 0;
	char *q;
	int16_t r;
	
	// read all available data
	while ((r = rxCharXbUART()) != -1) {
		// on 1st ETB, set ETB flag for next iteration
		if (r == '\x17' && !gotETB) {
			gotETB = 1;
			continue;
		}
		
		// if '\x17' follows an ETB, it was a stuffing byte
		if (r == '\x17' && gotETB)
			gotETB = 0;
		
		// end of valid packet
		if (gotETB && p == (char *)(&shadow + 1)) {
			DisableInterrupts();
			p = (char *)&shadow;
			q = (char *)&jstick0;
			while (p < (char *)(&shadow + 1))
				*q++ = *p++;
			EnableInterrupts();
		}
		
		// end of any packet
		if (gotETB || p >= (char *)(&shadow + 1)) {
			gotETB = 0;
			p = 0;
		}
		
		// new packet begin on STX
		if (!p && r == '\x02') {
			p = (char *)&shadow;
			continue;
		}
		
		// wait for STX
		if (!p)
			continue;
		
		
		*p++ = r;
	}
}

void ADC0IntHandler(void)
{
	// Acknowledge completion
	ADC0_ISC_R = ~0;
	
	adc0read[0] = ADC0_SSFIFO0_R & 0x0FFF;
	adc0read[1] = ADC0_SSFIFO0_R & 0x0FFF;
	adc0read[2] = ADC0_SSFIFO0_R & 0x0FFF;
}

void initADC(void)
{
	uint32_t delay;

	// Enable ADC 
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
	for (delay = 10; delay--;);
	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;	// Activate ADC0
	SYSCTL_RCGC0_R &= ~SYSCTL_RCGC0_ADC0SPD_M;	// set ADC0 to default 125k
	// Sequence 0 is highest priority
	ADC0_SSPRI_R =
	    ADC_SSPRI_SS0_1ST |
	    ADC_SSPRI_SS1_2ND |
	    ADC_SSPRI_SS2_3RD |
	    ADC_SSPRI_SS3_4TH;
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;	// Disable sample sequencer 0
	ADC0_IM_R = ADC_IM_MASK0;		// Interrupt on SS0
	ADC0_EMUX_R = ADC_EMUX_EM0_ALWAYS;	// Continuous
	ADC0_SSCTL0_R = ADC_SSCTL0_END2 | ADC_SSCTL0_IE2;
	ADC0_SSMUX0_R = (A0C10_YPOS_IN << 8) | (A0C11_XPOS_IN << 4) | A0C3_RANGE_0_IN;
	
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;	// Enable SS0
	NVIC_EN0_R |= NVIC_EN0_INT14; // Enable interrupt in NVIC
}

void handleMotors(void)
{
	static uint32_t MotorDutyCycle = 0;
	static uint32_t c = 0;

	int32_t m0d, m1d;

	MotorDutyCycle = (MotorDutyCycle + 1) % PWM_WIDTH;
	c = (c + 1) % (PID_MS * 1000000 / 1000 / SYSTICK_US);
	
	/* handle PID stuff every PID_MS milliseconds */
	if (!c) {
		// update position
		//nav.theta += (enc0.odo - enc1.odo) / (360 * 232);
		
		motor0.targ -= enc0.odo;
		motor1.targ -= enc1.odo;
		
		motor0.errsum += motor0.targ * 1000 / PID_MS;
		motor1.errsum += motor1.targ * 1000 / PID_MS;
		
		m0d = (motor0.targ * Pfactor + motor0.errsum * Ifactor) / PidNorm;
		m1d = (motor1.targ * Pfactor + motor1.errsum * Ifactor) / PidNorm;

		motor0.pwm = CLIP(-PWM_WIDTH, m0d, PWM_WIDTH);
		motor1.pwm = CLIP(-PWM_WIDTH, m1d, PWM_WIDTH);
		
		nav0.left += enc0.odo;
		nav0.right += enc1.odo;
		
		motor0.targ += motor0.speed * PID_MS / 1000;
		motor1.targ += motor1.speed * PID_MS / 1000;
		resetEncs();
	}

	if (motor0.pwm < 0)
		GPIO_PORTF_DATA_R |= PF1_MOTOR_0_DIR;
	else
		GPIO_PORTF_DATA_R &= ~PF1_MOTOR_0_DIR;

	if (MotorDutyCycle >= ABS(motor0.pwm))
		GPIO_PORTF_DATA_R &= ~PF2_MOTOR_0_EN;
	else
		GPIO_PORTF_DATA_R |= PF2_MOTOR_0_EN;

	if (motor1.pwm > 0)
		GPIO_PORTF_DATA_R |= PF3_MOTOR_1_DIR;
	else
		GPIO_PORTF_DATA_R &= ~PF3_MOTOR_1_DIR;

	if (MotorDutyCycle >= ABS(motor1.pwm))
		GPIO_PORTF_DATA_R &= ~PF4_MOTOR_1_EN;
	else
		GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
}

void handleEncoders(void)
{
	uint8_t p0 = invgrey[(GPIO_PORTB_DATA_R & (PB2_MOTOR_0_SA | PB3_MOTOR_0_SB)) >> 2];
	uint8_t p1 = invgrey[(GPIO_PORTB_DATA_R & (PB6_MOTOR_1_SA | PB7_MOTOR_1_SB)) >> 6];
	
	DisableInterrupts();
	switch ((p0 - enc0.pos) % sizeof (invgrey)) {
	case 1: enc0.odo++; break;
	case 3: enc0.odo--; break;
	}
	
	switch ((p1 - enc1.pos) % sizeof (invgrey)) {
	case 1: enc1.odo--; break;
	case 3: enc1.odo++; break;
	}
	EnableInterrupts();
	
	enc0.pos = p0;
	enc1.pos = p1;
}

void handleURFs(void)
{
	static char isLo = 0;
	
	// Ensure GPIO port E interrupts are enabled
	NVIC_EN0_R |= NVIC_EN0_INT4;
	
	if (!isLo) {
		DisableInterrupts();
		if (urf0.s && urf0.e) {
			urf0.dist += (urf0.e - urf0.s) / (80 * 58);
			urf0.dist >>= 1;
		}
		if (urf1.s && urf1.e) {
			urf1.dist += (urf1.e - urf1.s) / (80 * 58);
			urf1.dist >>= 1;
		}
		urf0.s = urf0.e = urf1.s = urf1.e = 0;
		EnableInterrupts();
		
		//start a new ping
		GPIO_PORTB_DATA_R &= ~PB0_TRIG_0;
		GPIO_PORTE_DATA_R &= ~PE2_TRIG_1;
	} else {
		//raise the line in prep for next ping
		GPIO_PORTB_DATA_R |= PB0_TRIG_0;
		GPIO_PORTE_DATA_R |= PE2_TRIG_1;
	}
	isLo = !isLo;
}

void handleIR()
{
	static int n = 0;
	static uint32_t sum = 0;
	
	sum += adc0read[1];
	
	if (n++ == NUM_SAMPLES) {
		ir0.dist = sum / n;
		n = 0;
		sum = 0;
	}
}

void initSYSTICK(uint32_t count)
{
	NVIC_ST_CTRL_R = 0;	// disable SysTick timer
	NVIC_ST_RELOAD_R = count - 1;	// Set reload to 1ms
	NVIC_ST_CURRENT_R = 0;	// clear the current count
	// use system clock, enable timer, enable interrupts
	NVIC_ST_CTRL_R =
	    NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE |
	    NVIC_ST_CTRL_CLK_SRC;
}

void systickDelay(int ms)
{
	DisableInterrupts();
	systickdelay = 0;
	EnableInterrupts();
	while (systickdelay < ms * 1000000 / 1000 / SYSTICK_US);
}

void SYSTICKIntHandler(void)
{
	char str[512];
	
	DisableInterrupts();
	systickdelay++;
	EnableInterrupts();
	
	handleMotors();
	handleEncoders();
}

void initTIMER0A(uint32_t count)
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
	TIMER0_CTL_R = 0; //disable
	TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER; //32-bit timer
	TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; //Enables and sets to periodic
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;
	TIMER0_IMR_R = TIMER_IMR_TATOIM;
	TIMER0_TAILR_R = count - 1;
	TIMER0_TAPR_R = 0;
	TIMER0_CTL_R = TIMER_CTL_TAEN; //enable
	NVIC_EN0_R = NVIC_EN0_INT19; //enable in nvic
	
}

void TIMER0AIntHandler(void)
{
	handleURFs();
	
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
}

void PORTEIntHandler(void)
{
	if (GPIO_PORTE_RIS_R & PE1_ECHO_0) {
		if (GPIO_PORTE_DATA_R & PE1_ECHO_0)
			urf0.s = TIMER0_TAR_R;
		else
			urf0.e = TIMER0_TAR_R;
	}
	
	if (GPIO_PORTE_RIS_R & PE3_ECHO_1) {
		if (GPIO_PORTE_DATA_R & PE3_ECHO_1)
			urf1.s = TIMER0_TAR_R;
		else
			urf1.e = TIMER0_TAR_R;
	}
	
	GPIO_PORTE_ICR_R = ~0;
}

void initTIMER1A(uint32_t count)
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // enable clock
	TIMER1_CTL_R = 0; // disable module
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; //32-bit timer
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; //Enables and sets to periodic
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;
	TIMER1_IMR_R = TIMER_IMR_TATOIM;
	TIMER1_TAILR_R = count - 1;
	TIMER1_TAPR_R = 0;
	TIMER1_CTL_R = TIMER_CTL_TAEN; // enable module
	NVIC_EN0_R = NVIC_EN0_INT21; //enable in nvic
}

void TIMER1AIntHandler(void)
{
	handleIR();
	
	TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}

void drive(void)
{
	int dif = jstick0.x - 5;
	int com = jstick0.y - 5;
	if (dif < 1 && dif > -1) {
		motor0.speed = com * 100;
		motor1.speed = com * 100;
	} else {
		motor0.speed = 0 - dif * 42;
		motor1.speed = 0 + dif * 42;
	}
}

/* Decide if forward motion should be disallowed */
void emergencyStop(void)
{
	// if forward motion isn't happening, then just don't mess around with anything
	if (motor0.speed + motor1.speed <= 0)
		return;
	
	// if no violation, OK
	if (urf0.dist >= 20 && urf1.dist >= 20)
		return;
	
	// keep the robot still
	motor0.speed = 0;
	motor1.speed = 0;
}

void foo()
{
	uint8_t a = 1;
	uint8_t b = 1;
	uint8_t c = 1;
	uint8_t d = 1;
	uint8_t e = 1;
	uint8_t f = 1;
}

void bar()
{
	uint8_t a = 1;
	uint8_t b = 1;
	uint8_t c = 1;
	uint8_t d = 1;
	uint8_t e = 1;
}

void foobar()
{
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
	uint8_t e;
	uint8_t f;
}

void runrobot()
{
	int i;
	char str[255];
	
	// joystick control
	while (!jstick0.press) {
		for (i = 10; i--;) {
			systickDelay(20);
			rxPS2Struct();
			//handlePS2();
			drive();
			emergencyStop();
		}
		
		sprintf(str, "MOTOR SPEEDS %d %d\n\r", motor0.speed, motor1.speed);
		txDbgUART(str);
		//sprintf(str, "MOTOR ERR %d %d\n\r", motor0.errsum, motor1.errsum);
		//txDbgUART(str);
// 		sprintf(str, "MOTOR TARG %d %d\n\r", motor0.targ, motor1.targ);
// 		txDbgUART(str);
// 		sprintf(str, "JS %d %d\n\r", jstick0.x, jstick0.y);
// 		txDbgUART(str);
		//sprintf(str, "URF %d %d\n\r", urf0.dist, urf1.dist);
		//txDbgUART(str);
		//sprintf(str, "MOTOR0 %d %d\n\r", motor0.errsum, motor0.speed);
		//txDbgUART(str);
	}
	
	// full stop
	motor0.speed = 0;
	motor1.speed = 0;
	resetNav();
	systickDelay(1000);
	resetNav();
	
	// move 10 ft
	motor0.speed = 288;
	motor1.speed = 288;
	while (nav0.left < TEN_FEET_TICKS && nav0.right < TEN_FEET_TICKS) {
	}
	
	// full stop
	motor0.speed = 0;
	motor1.speed = 0;
}

void runcontroller()
{
	char str[255];
	
	while (1) {
		systickDelay(20);
		handlePS2();
		txPS2Struct();

		sprintf(str, "JS %d %d\n\r", jstick0.x, jstick0.y);
		txDbgUART(str);
	}
}

int main(void)
{
	char str[255];
	int16_t i;
	
	// Initialize the PLLs so the the main CPU frequency is 80MHz
	PLL_Init();

	initializeGPIOPort(PORTA, &portA_config);
	initializeGPIOPort(PORTB, &portB_config);
	initializeGPIOPort(PORTC, &portC_config);
	initializeGPIOPort(PORTD, &portD_config);
	initializeGPIOPort(PORTE, &portE_config);
	initializeGPIOPort(PORTF, &portF_config);
	
	initADC();
	initDbgUART(9600);
	initXbUART(9600);
	//UART5_CTL_R |= UART_CTL_LBE;
	
	// ticks = (CPU ticks/second) * (periods/time unit) / (time units/second)
	initSYSTICK(CPU_HZ / 1000000 * SYSTICK_US);
	initTIMER0A(CPU_HZ / 1000 * TIMER0_MS);
	initTIMER1A(CPU_HZ / 1000 * TIMER1_MS);
	
	isrobot = 1;
	
	if (isrobot)
		runrobot();
	else
		runcontroller();
	
	while (1);
}