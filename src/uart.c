#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "lm4f120h5qr.h"
#include "gpio.h"
#include "board_config.h"


#define	UART0 0x4000C000
#define UART1 0x4000D000
#define UART2 0x4000E000
#define UART3 0x4000F000
#define UART4 0x40010000
#define UART5 0x40011000
#define UART6 0x40012000
#define UART7 0x40013000


typedef struct {

	volatile uint32_t Data;	// + 0x000
	volatile uint32_t RxStatus;	// + 0x004
	volatile uint32_t Unused0[4];

	volatile uint32_t Flag;	// + 0x018
	volatile uint32_t Unused1;

	volatile uint32_t IrDALowPower;	// + 0x020
	volatile uint32_t IntegerBaudRateDiv;	// + 0x024
	volatile uint32_t FracBaudRateDiv;	// + 0x028
	volatile uint32_t LineControl;	// + 0x02C
	volatile uint32_t UARTControl;	// + 0x030
	volatile uint32_t IntFIFOLevelSel;	// + 0x034
	volatile uint32_t IntMask;	// + 0x038 
	volatile uint32_t RawIntStatus;	// + 0x03C
	volatile uint32_t MaskedIntStatus;	// + 0x040
	volatile uint32_t IntClear;	// + 0x044
} UART_PERIPH;

enum UartBufParams {
	UART_BUFSIZ = 512
};


struct UartBuf {
	char buf[UART_BUFSIZ];
	int h;
	int t;
};


struct UartBuf u0tx, u0rx;
struct UartBuf u5tx, u5rx;



extern void EnableInterrupts(void);
extern void DisableInterrupts(void);


//*****************************************************************************
// Handle UART interrupts
//*****************************************************************************
static void _UARTIntHandler(volatile UART_PERIPH * p,
			    volatile struct UartBuf *tx,
			    volatile struct UartBuf *rx)
{

	//begin critical section
	DisableInterrupts();

	//TX part
	if (p->RawIntStatus & UART_IM_TXIM) {
		//Write until HW buf is full or SW buf is empty.
		while (tx->h != tx->t && !(p->Flag & UART_FR_TXFF)) {
			p->Data = tx->buf[tx->h];
			tx->h = (tx->h + 1) % UART_BUFSIZ;
		}
		
		//if SW buffer is empty, disable TX interrupts.
		if (tx->h == tx->t)
			p->IntMask &= ~UART_IM_TXIM;
	}
	
	//RX part
	if (p->RawIntStatus & UART_IM_RXIM) {
		//read until the HW buf is empty or until SW buf is full
		while (!(p->Flag & UART_FR_RXFE)) {
			rx->buf[rx->t] = p->Data;
			rx->t = (rx->t + 1) % UART_BUFSIZ;

			//stop if buffer is full
			if (rx->t != (rx->h + UART_BUFSIZ - 1) % UART_BUFSIZ)
				break;
		}
	}
	
	p->IntClear |= UART_IM_RXIM | UART_IM_TXIM;
	
	//end critical section
	EnableInterrupts();
}



void UART0IntHandler()
{
	_UARTIntHandler((UART_PERIPH *) UART0, &u0tx, &u0rx);
}

void UART5IntHandler()
{
	_UARTIntHandler((UART_PERIPH *) UART5, &u5tx, &u5rx);
}

//*****************************************************************************
// TX
//*****************************************************************************
static int _txUART(volatile UART_PERIPH * p, volatile struct UartBuf *tx,
		   char *d)
{

	char *d0 = d;

	// load all data or fill SW buffer
	while (*d) {
		DisableInterrupts();
		while (*d && (tx->t + 1) % UART_BUFSIZ != tx->h) {
			tx->buf[tx->t] = *d++;
			tx->t = (tx->t + 1) % UART_BUFSIZ;
		}
		EnableInterrupts();
	}

	// Ensure HW buffer is primed and interrupts are enabled
	DisableInterrupts();
	while (tx->h != tx->t && !(p->Flag & UART_FR_TXFF)) {
		p->Data = tx->buf[tx->h];
		tx->h = (tx->h + 1) % UART_BUFSIZ;
	}
	p->IntMask |= UART_IM_TXIM;
	EnableInterrupts();

	//return # bytes written.
	return d - d0;
}

int txDbgUART(char *d)
	{return _txUART((UART_PERIPH *) UART0, &u0tx, d);}
int txXbUART(char *d)
	{return _txUART((UART_PERIPH *) UART5, &u5tx, d);}


static int _txDataUART(volatile UART_PERIPH * p,
		       volatile struct UartBuf *tx, char *d, uint32_t n)
{
	char *d0 = d;

	// load all dato a fill SW buffer
	while (n) {
		DisableInterrupts();
		while (n && (tx->t + 1) % UART_BUFSIZ != tx->h) {
			tx->buf[tx->t] = *d++;
			tx->t = (tx->t + 1) % UART_BUFSIZ;
			n--;
		}
		EnableInterrupts();
	}

	// Ensure HW buffer is primed and interrupts are enabled
	DisableInterrupts();
	while (tx->h != tx->t && !(p->Flag & UART_FR_TXFF)) {
		p->Data = tx->buf[tx->h];
		tx->h = (tx->h + 1) % UART_BUFSIZ;
	}
	p->IntMask |= UART_IM_TXIM;
	EnableInterrupts();

	//return # bytes written.
	return d - d0;
}

int txDataDbgUART(char *d, uint32_t n)
{
	return _txDataUART((UART_PERIPH *) UART0, &u0tx, d, n);
}

int txDataXbUART(char *d, uint32_t n)
{
	return _txDataUART((UART_PERIPH *) UART5, &u5tx, d, n);
}


//*****************************************************************************
// RX
//*****************************************************************************
static int _rxCharUART(volatile UART_PERIPH * p,
		       volatile struct UartBuf *rx)
{
	int rv = -1;

	DisableInterrupts();
	//check on SW FIFO
	if (rx->h != rx->t) {
		rv = rx->buf[rx->h];
		rx->h = (rx->h + 1) % UART_BUFSIZ;
	}
	//check on HW FIFO
	if (!(p->Flag & UART_FR_RXFE)) {
		rv = p->Data;
	}
	EnableInterrupts();

	return rv;
}

int rxCharDbgUART(void)
{
	return _rxCharUART((UART_PERIPH *) UART0, &u0rx);
}

int rxCharXbUART(void)
{
	return _rxCharUART((UART_PERIPH *) UART5, &u5rx);
}


//*****************************************************************************
// Config
//*****************************************************************************
static bool _initUART(volatile UART_PERIPH * p,
		      volatile struct UartBuf *tx,
		      volatile struct UartBuf *rx, uint32_t baudrate)
{

	//init SW buffer
	tx->h = tx->t = 0;
	rx->h = rx->t = 0;

	//2. Disable UART during config
	p->UARTControl = 0;

	//3. Set baud
	switch (baudrate) {

	case  9600:
		p->IntegerBaudRateDiv = 520;
		p->FracBaudRateDiv = 53;
		break;
	case 115200:
		p->IntegerBaudRateDiv = 43;
		p->FracBaudRateDiv = 26;
		break;
	default:
		return false;
	}

	//4. FIFO interrupt levels
	p->IntFIFOLevelSel = UART_IFLS_TX1_8 | UART_IFLS_RX1_8;

	//5. IRQ on RX, Don't IRQ on TX until you have something to send.
	p->IntMask = UART_IM_RXIM;

	//6. Configure the Line Control for 8-n-1 w/ FIFOs
	p->LineControl = UART_LCRH_WLEN_8 | UART_LCRH_FEN;

	//7. Enable UART w/ TX and RX
	p->UARTControl = UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;

	return true;
}

bool initDbgUART(uint32_t baudrate)
{
	uint32_t delay;

	//1. Enable clock. delay for power up
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
	for (delay = 10; delay--;);

	//[2-7]. generic configuration
	if (!_initUART((UART_PERIPH *) UART0, &u0tx, &u0rx, baudrate))
		return false;

	//9, 10. Configure NVIC
	NVIC_PRI1_R &= ~(0xFF << (1 * 4));
	NVIC_PRI1_R |= 2 << (1 * 4);
	NVIC_EN0_R |= 1 << 5;

	// Wait until the UART is avaiable
	while (!(SYSCTL_PRUART_R & SYSCTL_PRUART_R0));

	return true;
}

bool initXbUART(uint32_t baudrate)
{
	uint32_t delay;

	//1. Enable clock. delay for power up
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5;
	for (delay = 10; delay--;);

	//[2-7]. generic configuration
	if (!_initUART((UART_PERIPH *) UART5, &u5tx, &u5rx, baudrate))
		return false;

	//9, 10. Configure NVIC
	NVIC_PRI15_R &= ~(0xFF << (1 * 4));
	NVIC_PRI15_R |= 2 << (1 * 4);
	NVIC_EN1_R |= 1 << (61 - 32);


	// Wait until the UART is avaiable
	while (!(SYSCTL_PRUART_R & SYSCTL_PRUART_R5));

	return true;
}
