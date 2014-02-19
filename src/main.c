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

//*****************************************************************************
// Global Variables
//*****************************************************************************
struct jsrange {
	int bot;
	int top;
	//NOTE: Top and Bottom of joystick motion is stuck at max value 
	//(ie we cannot get response past 70% of controller forward motion)
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

int motorspeeds[] = {
	-100,
	-80,
	-60,
	-40,
	-20,
	0,
	20,
	40,
	60,
	80,
	100
};

volatile int MotorDutyCycle = 0;
int response_time = 0;
int count = 0;
int time_result = 0;
int distance = 0;
char str[255];

volatile struct joystick {
	int x;
	int y;
	int press;
	int release;
} jstick = {5, 5, 0, 0};

//*****************************************************************************
// External Functions
//*****************************************************************************
extern void PLL_Init(void);
extern bool InitializeUART(uint32_t base, UART_CONFIG * init);
extern char uartRxPoll(uint32_t base);
extern void uartTxPoll(uint32_t base, char *data);
extern bool initializeGPIOPort(uint32_t base, GPIO_CONFIG * init);
void initADC(void)
{
	uint32_t delay;

	// Enable ADC 
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
	for (delay = 10; delay--;);
	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;	// Activate ADC0
	SYSCTL_RCGC0_R &= ~SYSCTL_RCGC0_ADC0SPD_M;	// set ADC0 to default 125k
	// Sequence 3 is highest priority
	ADC0_SSPRI_R =
	    ADC_SSPRI_SS3_1ST |
	    ADC_SSPRI_SS2_2ND |
	    ADC_SSPRI_SS1_3RD |
	    ADC_SSPRI_SS0_4TH;
	ADC0_IM_R = 0;		// Don't send any interrupts
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;	// Disable sample sequencer 3
	ADC0_EMUX_R &= ~ADC_EMUX_EM3_M;	// Use default PROCESSOR trigger
	ADC0_SSCTL3_R = ADC_SSCTL3_END0 | ADC_SSCTL3_IE0;	// sequence ends after sample 0.
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;	// Enable SS3
}


/* non-blocking function called periodically in systick to update adcVals */
uint32_t readADC(int chan)
{
	ADC0_SSMUX3_R = chan;	// Select channel to sample
	ADC0_ISC_R = ADC_ISC_IN3;	// Acknowledge completion
	ADC0_PSSI_R = ADC_PSSI_SS3;	// Initiate SS3
	while (!(ADC0_RIS_R & ADC_RIS_INR3));	// wait for sample
	ADC0_ISC_R = ADC_ISC_IN3;	// Acknowledge completion
	return ADC0_SSFIFO3_R & 0x0FFF;
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

void SYSTICKIntHandler(void)
{
	int s;
	
	MotorDutyCycle++;
	MotorDutyCycle %= 100;
	
	s = motorspeeds[jstick.y];
	// backwards speeds
	if (s < 0) {
		GPIO_PORTF_DATA_R &= ~PF1_MOTOR_0_DIR;
		GPIO_PORTF_DATA_R |= PF3_MOTOR_1_DIR;
		s = -s;
	// forwards speeds
	} else {
		GPIO_PORTF_DATA_R |= PF1_MOTOR_0_DIR;
		GPIO_PORTF_DATA_R &= ~PF3_MOTOR_1_DIR;
	}

	if (MotorDutyCycle >= s) {
		GPIO_PORTF_DATA_R &= ~PF2_MOTOR_0_EN;
		GPIO_PORTF_DATA_R &= ~PF4_MOTOR_1_EN;
	} else {
		GPIO_PORTF_DATA_R |= PF2_MOTOR_0_EN;
		GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
	}
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
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
	//Send pulse at 20Hz
	count++;
	response_time++;
	if (count == 5000) {
		//Send short >10us pulse -- TODO
		//start timer
		GPIO_PORTB_DATA_R |= PB0_TRIG_0;
		time_result = 1;
		response_time = 0;
	}
	if (count >= 5020) {
	//if (count >= 5100) {
		//End >10us pulse
		GPIO_PORTB_DATA_R &= ~PB0_TRIG_0;	
		count = 0;
	}
	
	//Timing until ECHO signal STOPS (Falling Edge)
	if (time_result == 1) {
		//give time for response
		if (response_time >= 6) {
				//Watch for falling edge
						//TODO MUST ADD EDGE DETECTION (aka detect rising and falling edge and make sure one is in front of the other)
				if ((PE1_ECHO_0 & GPIO_PORTE_DATA_R)==2) {
					//detected start
					distance = (response_time*10) / 58;
					time_result = 0;
					response_time = 0;
				}

			//Seen response signal
			//else {
				//distance = response_time / 58;
				

			}
	}

	
}


//*****************************************************************************
//*****************************************************************************
int main(void)
{
	volatile unsigned long delay;
	int t;
	static uint16_t debounce = 0;
	
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
	//initTIMER0A(4000000); //20Hz
	initTIMER0A(800); //Interrupts every 10us
	
	uartTxPoll(UART0, "=============================\n\r");
	uartTxPoll(UART0, "ECE315 Lab1  \n\r");
	uartTxPoll(UART0, "=============================\n\r");
	
	
	while (1) {
		//
				sprintf(str, "DISTANCE -> %d \r\n",distance);
		uartTxPoll(UART0, str);
		/* this "works", but should be rethought */
		debounce = (debounce << 1) | (GPIO_PORTB_DATA_R & PB1_PS2_BUTTON ? 1 : 0);
		if (debounce == 0x8000) {
			jstick.press = 1;
			uartTxPoll(UART0, "B: PRESSED\n\r");
			
		} else if (debounce == 0x7FFF) {
			jstick.release = 1;
			uartTxPoll(UART0, "B: RELEASED\n\r");
		
		}
		
		t = readADC(A0C10_YPOS_IN);
		/*c++;
		if (c > 30000) {
			sprintf(str, "READ IN VALUE -> %d",t);
			uartTxPoll(UART0, str);
			sprintf(str, " --- Speed = %d\n\r",ypos);
			uartTxPoll(UART0, str);
			c=0;
		}*/
		if (t < jsranges[jstick.y].bot) {
			jstick.y--;
			sprintf(str, "Y: moved left to %x (ADC: %d)\n\r", jstick.y, t);
			uartTxPoll(UART0, str);
		} else if (t > jsranges[jstick.y].top) {
			jstick.y++;
			sprintf(str, "Y: moved right to %x (ADC: %d)\n\r", jstick.y, t);
			uartTxPoll(UART0, str);
		}
		
		t = readADC(A0C11_XPOS_IN);
		if (t < jsranges[jstick.x].bot) {
			jstick.x--;
			sprintf(str, "X: moved left to %x (ADC: %d)\n\r", jstick.x, t);
			uartTxPoll(UART0, str);
		} else if (t > jsranges[jstick.x].top) {
			jstick.x++;
			sprintf(str, "X: moved right to %x (ADC: %d)\n\r", jstick.x, t);
			uartTxPoll(UART0, str);
		}
	}
}
