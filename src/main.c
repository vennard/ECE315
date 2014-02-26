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

#define NUM_SAMPLES 10
#define UTICKS 232
#define PWM_WIDTH 100

// 80MHz = 80000000Hz
#define CPU_HZ 80000000
// 1 / (200 uS) = 5kHz
#define SYSTICK_US 200
// 1 / (25 mS) = 40 Hz
#define TIMER0_MS 25
// 1 / (50 mS) = 20 Hz
#define TIMER1_MS 50

#define PID_MS 100

enum PIDparams {
	Pfactor = 60,
	Ifactor = 5,
	PidNorm = 2000
};

static volatile uint32_t systickdelay = 0;
static volatile char isrobot;

void resetEncs(void)
{
	DisableInterrupts();
	enc0.odo = enc1.odo = 0;
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
}

void txPS2Struct(void)
{
	char *p;
	
	txXbUART("\x02", true);	// send STX
	for (p = (char *)&jstick0; p < (char *)(&jstick0 + 1); p++) {
		if (*p == '\x17')
			txXbUART("\x17", true);	// stuff ETB
		txDataXbUART(p, 1, true);
	}
	txXbUART("\x17", true);		// send ETB
}

/*void initXb(bool flipdlmy)
{
	int16_t r;
	
	txDbgUART("begin init\r\n", true);
	
	txXbUART("+++\r", true);
	while ((r = rxCharXbUART(true)) != 'K') {
		txDataDbgUART((char *)&r, 1, true);
	}
	txDbgUART("got +++ OK\r\n", true);
	
	txXbUART("ATCH=10\r\n", true);
	while ((r = rxCharXbUART(false)) != 'K');
	txDbgUART("got ATCH OK\r\n", true);
	
	txXbUART("ATID=1000\r\n", true);
	while ((r = rxCharXbUART(false)) != 'K');
	txXbUART(flipdlmy ? "ATDL=1001\r\n" : "ATDL=1001\r\n", true);
	while ((r = rxCharXbUART(false)) != 'K');
	txXbUART(!flipdlmy ? "ATDL=1001\r\n" : "ATDL=1001\r\n", true);
	while ((r = rxCharXbUART(false)) != 'K');
	//txXbUART("ATWR\r\n", true);
	//while ((r = rxCharXbUART(false)) != 'K');
	txXbUART("ATSR\r\n", true);
	while ((r = rxCharXbUART(false)) != 'K');
}*/

void rxPS2Struct(void)
{
	static struct JoystickData shadow;
	static char *p = 0;
	static char gotETB = 0;
	char *q;
	int16_t r;
	
	// read all available data
	while ((r = rxCharXbUART(false)) != -1) {
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
		
		// speed = dist * 1 / time
		motor0.speed = enc0.odo * 1000 / PID_MS;
		motor1.speed = enc1.odo * 1000 / PID_MS;
		
		motor0.errsum += (motor0.targ - motor0.speed) * 1000 / PID_MS;
		motor1.errsum += (motor1.targ - motor1.speed) * 1000 / PID_MS;
	
		m0d = ((motor0.targ - motor0.speed) * Pfactor + motor0.errsum * Ifactor) / PidNorm;
		m1d = ((motor1.targ - motor1.speed) * Pfactor + motor1.errsum * Ifactor) / PidNorm;

		motor0.pwm = CLIP(-PWM_WIDTH, m0d, PWM_WIDTH);
		motor1.pwm = CLIP(-PWM_WIDTH, m1d, PWM_WIDTH);
		
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
	
	static int pings = 0;
	static int u0n = 0, u1n = 0;
	static uint32_t u0sum = 0.0, u1sum = 0.0;
	
	// Ensure GPIO port E interrupts are enabled
	NVIC_EN0_R |= NVIC_EN0_INT4;
	
	if (!isLo) {
		DisableInterrupts();
		if (urf0.s && urf0.e) {
			u0sum += urf0.e - urf0.s;
			u0n++;
		}
		if (urf1.s && urf1.e) {
			u1sum += urf1.e - urf1.s;
			u1n++;
		}
		urf0.s = urf0.e = urf1.s = urf1.e = 0;
		EnableInterrupts();
		
		//start a new ping
		GPIO_PORTB_DATA_R &= ~PB0_TRIG_0;
		GPIO_PORTE_DATA_R &= ~PE2_TRIG_1;
		
		pings++;
	} else {
		//raise the line in prep for next ping
		GPIO_PORTB_DATA_R |= PB0_TRIG_0;
		GPIO_PORTE_DATA_R |= PE2_TRIG_1;
	}
	isLo = !isLo;
	
	if (pings >= NUM_SAMPLES) {
		DisableInterrupts();
		urf0.dist = u0n ? u0sum / (u0n * 80 * 58) : 0;
		urf1.dist = u1n ? u1sum / (u1n * 80 * 58) : 0;
		EnableInterrupts();
		
		u0n = u0sum = u1n = u1sum = 0;
		pings = 0;
	}
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

int main(void)
{
	char str[255];
	int16_t r;
	
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
	
	isrobot = 0;

	while (isrobot) {
		systickDelay(100);
		rxPS2Struct();
		
		sprintf(str, "JS %d %d\n\r", jstick0.x, jstick0.y);
		txDbgUART(str, true);
		
		txDbgUART(".\r\n", true);
	}
	
	while (!isrobot) {
		systickDelay(100);
		handlePS2();
		txPS2Struct();

		sprintf(str, "JS %d %d\n\r", jstick0.x, jstick0.y);
		txDbgUART(str, true);
	}
	
	while (1);
}