//*****************************************************************************
//
//*****************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "lm4f120h5qr.h"
#include "board_config.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define PORTA   0x40004000
#define PORTB   0x40005000
#define PORTC   0x40006000
#define PORTD   0x40007000
#define PORTE   0x40024000
#define PORTF   0x40025000

#define NUM_SAMPLES 10
#define REG_POW 0

//*****************************************************************************
// TABLES
//*****************************************************************************
//NOTE: Top and Bottom of joystick motion is stuck at max value 
//(ie we cannot get response past 70% of controller forward motion)
struct jsrange {
	int bot;
	int top;
} jsranges[] = {
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

enum MotorDir {
	MotorFwd = 0,
	MotorRev = 1
};

//*****************************************************************************
// GLOBALS
//*****************************************************************************
volatile struct MotorData {
	uint32_t pow;
	char dir;

	int32_t odo;
	double rate;
} motor0 = {REG_POW, MotorFwd, 0, 0}, motor1 = {REG_POW, MotorFwd, 0, 0};

volatile struct URFdata {
	uint32_t s;
	uint32_t e;
	
	uint32_t dist;
} urf0 = {0, 0, 9999}, urf1 = {0, 0, 9999};

volatile struct IRdata {
	uint32_t dist;
} ir0 = {0};

volatile struct joystick {
	int x;
	int y;
	int press;
	int release;
} jstick = {5, 5, 0, 0};

volatile uint16_t adc0read[3];
uint32_t systick_delay = 0;

//*****************************************************************************
// LIBRARY FUNCTIONS
//*****************************************************************************
extern void PLL_Init(void);
extern bool InitializeUART(uint32_t base, UART_CONFIG * init);
extern char uartRxPoll(uint32_t base);
extern void uartTxPoll(uint32_t base, char *data);
extern bool initializeGPIOPort(uint32_t base, GPIO_CONFIG * init);
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);

void handlePS2(void)
{
	uint16_t t;
	static uint16_t debounce = 0;

	debounce = (debounce << 1) | (GPIO_PORTB_DATA_R & PB1_PS2_BUTTON ? 1 : 0);
	if (debounce == 0x8000)
		jstick.press = 1;
	else if (debounce == 0x7FFF)
		jstick.release = 1;
	
	t = adc0read[0];
	if (t < jsranges[jstick.y].bot)
		jstick.y--;
	else if (t > jsranges[jstick.y].top)
		jstick.y++;
	
	t = adc0read[2];
	if (t < jsranges[jstick.x].bot)
		jstick.x--;
	else if (t > jsranges[jstick.x].top)
		jstick.x++;
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

	MotorDutyCycle = (MotorDutyCycle + 1) % 100;
	
	if (motor0.dir)
		GPIO_PORTF_DATA_R |= PF1_MOTOR_0_DIR;
	else
		GPIO_PORTF_DATA_R &= ~PF1_MOTOR_0_DIR;

	if (MotorDutyCycle >= motor0.pow)
		GPIO_PORTF_DATA_R &= ~PF2_MOTOR_0_EN;
	else
		GPIO_PORTF_DATA_R |= PF2_MOTOR_0_EN;


	if (!motor1.dir)
		GPIO_PORTF_DATA_R |= PF3_MOTOR_1_DIR;
	else
		GPIO_PORTF_DATA_R &= ~PF3_MOTOR_1_DIR;

	if (MotorDutyCycle >= motor1.pow)
		GPIO_PORTF_DATA_R &= ~PF4_MOTOR_1_EN;
	else
		GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
}

void handleEncoders(void)
{
	static char pe0 = 0;
	static char pe1 = 0;
	char e0 = (GPIO_PORTB_DATA_R & (PB2_MOTOR_0_SA | PB3_MOTOR_0_SB)) >> 2;
	char e1 = (GPIO_PORTB_DATA_R & (PB6_MOTOR_1_SA | PB7_MOTOR_1_SB)) >> 6;
	
	DisableInterrupts();
	if (e0 != pe0)
		motor0.odo++;
	if (e1 != pe1)
		motor1.odo++;
	EnableInterrupts();
	
	pe0 = e0;
	pe1 = e1;
}

void handleURFs(void)
{
	static char isLo = 0;
	
	static int pings = 0;
	static int u0n = 0, u1n = 0;
	static uint32_t u0sum = 0.0, u1sum = 0.0;
	
	char str[255];
	
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
	static uint16_t v;
	
	static int n = 0;
	static uint32_t sum = 0;
	
	v = adc0read[1];
	sum += v;
	n++;
	
	if (n == NUM_SAMPLES) {
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

/* TODO: fix... this is horrible*/
void systickDelay(int delay)
{
	systick_delay = 0;
	while (systick_delay < delay * 5);
}

void SYSTICKIntHandler(void)
{
	char str[512];
	
	systick_delay++;
	
	handleMotors();
	handleEncoders();
	
	//handlePS2();
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
	char str[255];
	handleURFs();
	
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
}

void PORTEIntHandler(void)
{
	if (GPIO_PORTE_RIS_R & PE1_ECHO_0) {
		DisableInterrupts();
		
		if (GPIO_PORTE_DATA_R & PE1_ECHO_0)
			urf0.s = TIMER0_TAR_R;
		else
			urf0.e = TIMER0_TAR_R;
		
		EnableInterrupts();
	}
	
	if (GPIO_PORTE_RIS_R & PE3_ECHO_1) {
		DisableInterrupts();
		
		if (GPIO_PORTE_DATA_R & PE3_ECHO_1)
			urf1.s = TIMER0_TAR_R;
		else
			urf1.e = TIMER0_TAR_R;
		
		EnableInterrupts();
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
	int t;
	uint32_t delay;
	
	// Initialize the PLLs so the the main CPU frequency is 80MHz
	PLL_Init();

	initializeGPIOPort(PORTA, &portA_config);
	initializeGPIOPort(PORTB, &portB_config);
	initializeGPIOPort(PORTC, &portC_config);
	initializeGPIOPort(PORTD, &portD_config);
	initializeGPIOPort(PORTE, &portE_config);
	initializeGPIOPort(PORTF, &portF_config);
	InitializeUART(UART0, &UART0_config);
	
	initADC();
	initSYSTICK(11429);   //5kHz
	initTIMER0A(1000000);
	initTIMER1A(1000000);
	
	uartTxPoll(UART0, "=============================\n\r");
	uartTxPoll(UART0, "ECE315 Lab3  \n\r");
	uartTxPoll(UART0, "=============================\n\r");
	
	/*while (1) {
		sprintf(str, "S %d %d %d\n\r", adc0read[0], adc0read[1], adc0read[2]);
		uartTxPoll(UART0, str);
	}*/
	
	
	while (1) {
		sprintf(str, "S %d %d %d\n\r", urf0.dist, urf1.dist, ir0.dist);
		uartTxPoll(UART0, str);
		
		if (ir0.dist > 1200 || (urf0.dist < 30 && urf1.dist < 30)) {
			uartTxPoll(UART0, "HEAD ON\r\n");
			sprintf(str, "S %d %d %d\n\r", urf0.dist, urf1.dist, ir0.dist);
			uartTxPoll(UART0, str);
			
			
			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			// turn 180
			motor0.dir = MotorFwd; motor1.dir = MotorRev;
			motor0.odo = motor1.odo = 0;
			motor0.pow = motor1.pow = REG_POW;
			while (motor0.odo < 285 / 2);

			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			//resume going fwd
			motor0.dir = MotorFwd;
			motor1.dir = MotorFwd;
			motor0.pow = motor1.pow = REG_POW;
			
			//halt after 3 ft
			while (motor0.odo < 1000);
			motor0.pow = motor1.pow = 0;
			
			//don't move
			while(1);
		}
		
		if (urf0.dist < 30) {
			uartTxPoll(UART0, "RIGHT SIDE\r\n");
			sprintf(str, "S %d %d %d\n\r", urf0.dist, urf1.dist, ir0.dist);
			uartTxPoll(UART0, str);
			
			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			// turn until ready
			motor0.dir = MotorFwd; motor1.dir = MotorRev;
			motor0.odo = motor1.odo = 0;
			motor0.pow = motor1.pow = REG_POW;
			while (motor0.odo < 285 / 4);

			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			//resume going fwd
			motor0.dir = MotorFwd;
			motor1.dir = MotorFwd;
			motor0.pow = motor1.pow = REG_POW;
		}
		
		if (urf1.dist < 30) {
			uartTxPoll(UART0, "LEFT SIDE\r\n");
			sprintf(str, "S %d %d %d\n\r", urf0.dist, urf1.dist, ir0.dist);
			uartTxPoll(UART0, str);
			
			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			// turn until ready
			motor0.dir = MotorRev; motor1.dir = MotorFwd;
			motor0.odo = motor1.odo = 0;
			motor0.pow = motor1.pow = REG_POW;
			while (motor0.odo < 285 / 4);

			// stop for a second
			motor0.pow = motor1.pow = 0;
			systickDelay(1000);
			
			//resume going fwd
			motor0.dir = MotorFwd;
			motor1.dir = MotorFwd;
			motor0.pow = motor1.pow = REG_POW;
		}
		
		
		
		/*if (such collision) {
			reset odometers;
			change direction;
			while (!odo where they should be);
			restore direction
		}*/
	}
}
