/************************************************************************/
/* FILE NAME    - UART.h												*/
/* AUTHOR       - ECE353 Staff                                          */
/* DATE CREATED - 03-Jun-2013                                           */
/* DESCRIPTION  - Header file for UART.c 								*/
/* 														                */
/* (c) ECE Department, University of Wisconsin - Madison                */
/************************************************************************/
#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include "lm4f120h5qr.h"

#define	UART0 0x4000C000
#define UART1 0x4000D000
#define UART2 0x4000E000
#define UART3 0x4000F000
#define UART4 0x40010000
#define UART5 0x40011000
#define UART6 0x40012000
#define UART7 0x40013000

#define	_8BITS		0x3	
#define	ENABLED		1
#define	DISABLED	0

/*************************************************************************
 * RX FIFO LENGTHS (RxIntFIFO)
 ************************************************************************/
#define	RX_ONE_EIGHTH  	0x0
#define	RX_ONE_FOURTH	0x1
#define	RX_ONE_HALF		0x2
#define	RX_THREE_FOURTH	0x3
#define	RX_SEVEN_EIGHTH	0x4

/*************************************************************************
 * TX FIFO LENGTHS (TxIntFIFO)
 ************************************************************************/
#define	TX_ONE_EIGHTH  	0x4
#define	TX_ONE_FOURTH	0x3
#define	TX_ONE_HALF		0x2
#define	TX_THREE_FOURTH	0x1
#define	TX_SEVEN_EIGHTH	0x0

/*************************************************************************
 ************************************************************************/
#define EN_TX_FIFO		1

typedef struct {
  volatile uint32_t    Data;                    // + 0x000
  volatile uint32_t    RxStatus;                // + 0x004
  volatile uint32_t    Unused0[4];    			
  volatile uint32_t    Flag;                    // + 0x018
  volatile uint32_t    Unused1;  				
  volatile uint32_t    IrDALowPower;            // + 0x020
  volatile uint32_t    IntegerBaudRateDiv;      // + 0x024
  volatile uint32_t    FracBaudRateDiv;         // + 0x028
  volatile uint32_t    LineControl;             // + 0x02C
  volatile uint32_t    UARTControl;             // + 0x030
  volatile uint32_t    IntFIFOLevelSel;         // + 0x034
  volatile uint32_t    IntMask;                 // + 0x038 
  volatile uint32_t    RawIntStatus;            // + 0x03C
  volatile uint32_t    MaskedIntStatus;         // + 0x040
  volatile uint32_t    IntClear;                // + 0x044
} UART_PERIPH;
 
/*
  Todo: Get the Clk val & Baud rate
  and calculate the baud rate register's
  values inside the init function.
*/
typedef struct {
  uint8_t     UartEn;						// Enable/Disable	   
  uint8_t     UARTRxEn;						// Enable/Disable
  uint8_t     UARTTxEn;						// Enable/Disable
  uint8_t     WordLen;
  uint32_t	  BaudRate;
  uint8_t     FifoEn;						// Enable/Disable
  uint8_t     _2StopEn;						// Enable/Disable
  uint8_t     ParityEn;						// Enable/Disable
  uint8_t     EvenParityEn;					// Enable/Disable
  uint8_t     HighSpeedEn;					// Enable/Disable
  uint8_t     RxIntFIFO;
  uint8_t     TxIntFIFO; 
  uint8_t     RxIntMaskEn;					// Enable/Disable
  uint8_t     TxIntMaskEn;					// Enable/Disable
  uint8_t     RxTimeOutIntMaskEn;			// Enable/Disable
  uint8_t     OvrrIntMaskEn;				// Enable/Disable
  uint8_t     ParityErrIntMaskEn;			// Enable/Disable
  uint8_t     FramingErrIntMaskEn;			// Enable/Disable
} UART_CONFIG;


/* Exported functions --------------------------------------------------*/

#endif
