#include <p18f26k80.h>
#include "can.h"
#include "uart.h"

/*
// Configuration Bits
#pragma config XINST = OFF	// disable extended instructions
#pragma config FOSC = XT	// using external oscillator
#pragma config PLLCFG = OFF	// disable 4x pll
#pragma config FCMEN = OFF	// disable fail-safe clock monitor
#pragma config IESO = OFF	// disable internal oscillator switch over
#pragma config PWRTEN = OFF	// disable power up timer
#pragma config BOREN = OFF	// disable brown out protect
#pragma config BORV = 2		// set brownout threshold at 2V
#pragma config BORPWR = MEDIUM	// set BORMV to medium power level
#pragma config WDTEN = OFF	// disable watchdog
#pragma config CANMX = PORTB	// use port c pins for CAN
#pragma config MCLRE = ON	// enable MCLR (needed for debug)
#pragma config CPB = OFF	// disable boot code protect
#pragma config CPD = OFF	// disable ee read protect
#pragma config CP1 = OFF	// disable code protect
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config WRT1 = OFF	// disable table write protect
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
*/
#define FCY 8000000

// SETTINGS
#define CAN_ID 0x100		// device CAN ID
#define CAN_BAUD 500000		// CAN bus baud rate (250000, 500000, 1000000)
#define UART_BAUD 115200	// UART baud rate
#define DEBUG 1				// enable UART debug messages
#define BOOTLOAD 1			// use bootloader

#ifdef BOOTLOAD
#pragma code main=0x920
#endif
void main()
{
	// set output pins
	LATA = 0;
	LATB = 0;
	LATC = 0;
	TRISA = 0b11010000;
	TRISB = 0b00001100;
	TRISC = 0b11000000;

    UART1Init(UART_BAUD);
#ifdef DEBUG
	UART1TxROMString("Starting CAN...\r\n");
#endif

	// set CAN filter to device CAN ID
    RXF0SIDH = CAN_ID >> 3;
    RXF0SIDL = CAN_ID << 5;

	// initalize CAN perpherial 
	CANInit(CAN_BAUD);

#ifdef DEBUG
	UART1TxROMString("On CAN bus.\r\n");
#endif

	// enable global interrupts
	INTCONbits.GIE = 1;
	// enable perpherial interrupts
	INTCONbits.PEIE = 1;

	// loop forever
	for(;;);
}

// interrupt handling
#pragma interrupt isr
void isr()
{
	// CAN receive buffer 0 full
	if (PIR5bits.RXB0IF)
	{
		PIR5bits.RXB0IF = 0;
#ifdef DEBUG		
		UART1TxROMString("Received CAN message.\r\n");
#endif
		switch (RXB0D0)
		{
		case 0x0:
			break;
		// cmd 0x1: set output
		case 0x1:
			// set outputs
			// RA0 - RA3
			LATA = RXB0D1 & 0b00001111;
			// RA5	
			LATA |= (RXB0D1 & 0b00010000) << 1;
			// RB0, RB1
			LATB = ((RXB0D1 & 0b01100000) >> 5); 
			// RB4
			LATB |= ((RXB0D1 & 0b10000000) >> 3);
			// RB5 - RB7
			LATB |= ((RXB0D2 & 0b00000111) << 5);
			// RC0 - RC5
			LATC = RXB0D2 >> 3;
			break;
		// cmd 0x2: sleep
		case 0x2:
			LATA = 0;
			LATB = 0;
			LATC = 0;
			_asm sleep _endasm
			break;
		}
		
		CANTx(0xA3, 8, (PORTA&0b00111111),
			PORTC<<2,0,0,0,0,0,0);
		RXB0CONbits.RXFUL = 0;	// clear the buffer
	}
	return;
}

// interrupt mapping
#ifdef BOOTLOAD
#pragma code high_vector=0x908
#else
#pragma code high_vector=0x08
#endif
void high_isr()
{
	_asm goto isr _endasm
}
#ifdef BOOTLOAD
#pragma code low_vector=0x918
#else
#pragma code low_vector=0x18
#endif
void low_isr()
{
	_asm goto isr _endasm
}
