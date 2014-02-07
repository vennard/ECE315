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

#define OVERLAP 15

#define CENTER 5

//*****************************************************************************
// Global Variables
//*****************************************************************************
int xranges[] = { 99999 };

int yranges[] = { 372.36,	// rev 5
	744.72,			// rev 4
	1117.09,		// rev 3
	1489.45,		// rev 2
	1861.81,		// rev 1
	2234.18,		// off
	2606.54,		// fwd 1
	2978.90,		// fwd 2
	3351.27,		// fwd 3
	3723.63,		// fwd 4
	4096.0			// fwd 5
};


/*int pwm[] = {
	
};*/
int xpos = 0, ypos = 0, bs = 0;

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
	ADC0_SSPRI_R = ADC_SSPRI_SS3_1ST | ADC_SSPRI_SS2_2ND | ADC_SSPRI_SS1_3RD | ADC_SSPRI_SS0_4TH;	// Sequence 3 is highest priority
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
	static int pwm_phase = 0;
	pwm_phase++;
	pwm_phase %= 100;
	if (ypos > 1) {
		GPIO_PORTF_DATA_R &= ~PF1_MOTOR_0_DIR;
		GPIO_PORTF_DATA_R |= PF2_MOTOR_0_EN;
		GPIO_PORTF_DATA_R &= ~PF3_MOTOR_1_DIR;
		GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
	} else {
		GPIO_PORTF_DATA_R |= PF1_MOTOR_0_DIR;
		GPIO_PORTF_DATA_R |= PF2_MOTOR_0_EN;
		GPIO_PORTF_DATA_R |= PF3_MOTOR_1_DIR;
		GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
	}

	// GPIO_PORTF_DATA_R |= PF3_MOTOR_1_DIR;
	// GPIO_PORTF_DATA_R |= PF4_MOTOR_1_EN;
	uartTxPoll(UART0, "=\n\r");
}


//*****************************************************************************
//*****************************************************************************
int main(void)
{
	volatile unsigned long delay;
	char str[255];
	int t;

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
	initSYSTICK(11429);
	uartTxPoll(UART0, "=============================\n\r");
	uartTxPoll(UART0, "ECE315 Lab1  \n\r");
	uartTxPoll(UART0, "=============================\n\r");
	while (1) {
		t = PB1_PS2_BUTTON & GPIO_PORTB_DATA_R ? 1 : 0;
		if (t != bs) {
			uartTxPoll(UART0,
				   t ? "B: ON -> OFF\n\r" :
				   "B: OFF -> ON\n\r");
			bs = t;
		}
		t = readADC(A0C10_XPOS_IN);
		if (xpos != 0 && t < xranges[xpos - 1] - OVERLAP) {
			xpos--;
			sprintf(str, "X: moved up to %x\n\r", xpos);
			uartTxPoll(UART0, str);
		} else if (t > xranges[xpos]) {
			xpos++;
			sprintf(str, "X: moved down to %x\n\r", xpos);
			uartTxPoll(UART0, str);
		}
		t = readADC(A0C11_YPOS_IN);
		if (ypos != 0 && t < yranges[ypos - 1] - OVERLAP) {
			ypos--;
			sprintf(str, "Y: moved left to %x\n\r", ypos);
			uartTxPoll(UART0, str);
		} else if (t > yranges[ypos]) {
			ypos++;
			sprintf(str, "Y: moved right to %x\n\r", ypos);
			uartTxPoll(UART0, str);
		}
	}
}
