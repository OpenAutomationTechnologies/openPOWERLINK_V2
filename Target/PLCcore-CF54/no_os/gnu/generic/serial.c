/****************************************************************************/

/*
 *	serial.c -- serial driver for Coldfire internal UARTS.
 *
 *	Copyright (C) 1999 Greg Ungerer (gerg@moreton.com.au)
 */

/****************************************************************************/
 
#include "arch.h"
#include "mcfuart.h"
//#include "prototypes.h"

unsigned int consoleBase;

void configureSerial(unsigned int uartBase, unsigned int baudRate, unsigned int systemClock)
{
	volatile unsigned char	*uartp;
    unsigned long  div = 32 * baudRate;
    unsigned short clock = MCF_CLK / div;
	//double clock, fraction;

	/*
	 *	Reset UART, get it into known state...
	 */
	uartp = (volatile unsigned char *) (MCF_MBAR + uartBase);

	uartp[MCFUART_UCR] = MCFUART_UCR_CMDRESETRX;  /* reset RX */
	uartp[MCFUART_UCR] = MCFUART_UCR_CMDRESETTX;  /* reset TX */
	uartp[MCFUART_UCR] = MCFUART_UCR_CMDRESETMRPTR;  /* reset MR pointer */
	uartp[MCFUART_UCR] = MCFUART_UCR_CMDRESETERR;  /* reset Error pointer */

	/*
	 * Set port for specified baud rate, 8 data bits, 1 stop bit, no parity.
	 */
	uartp[MCFUART_UMR] = MCFUART_MR1_PARITYNONE | MCFUART_MR1_CS8;
	uartp[MCFUART_UMR] = MCFUART_MR2_STOP1;

	//clock = ((double)systemClock / 32.0) / (double)(baudRate);  /* Set baud above */

	//fraction = ((clock - (int)clock) * 16.0) + 0.5;

	uartp[MCFUART_UBG1] = ((clock >> 8) & 0xff);  /* set msb baud */
	uartp[MCFUART_UBG2] = (clock & 0xff);  /* set lsb baud */

	/*  Note: This register is not in the 5249 docs. I am assuming that Greg knew 
	 *        what he was doing, so I'm leaving it here.  ~Jeremy */
	//uartp[MCFUART_UFPD] = ((int)fraction & 0xf);  /* set baud fraction adjust */

	uartp[MCFUART_UCSR] = MCFUART_UCSR_RXCLKTIMER | MCFUART_UCSR_TXCLKTIMER;
	uartp[MCFUART_UCR] = MCFUART_UCR_RXENABLE | MCFUART_UCR_TXENABLE;

	return;
}

/****************************************************************************/

/*
 *	Output a single character, using UART polled mode.
 */
void nextPut(unsigned int uartBase, char ch)
{
	volatile unsigned char *uartp;
	int i;

	uartp = (volatile unsigned char *)(MCF_MBAR + uartBase);
	for (i = 0; (i < 0x10000); i++)
	{
		if (uartp[MCFUART_USR] & MCFUART_USR_TXREADY)
			break;
	}
	uartp[MCFUART_UTB] = ch;
}

void nextPutCRLF(unsigned int uartBase, char ch)
{
	if(ch == '\n')
		nextPut(uartBase, '\r');
	nextPut(uartBase, ch);
}

void console_nextPut(char ch)
{
	nextPutCRLF(consoleBase, ch);
}

/****************************************************************************/

void console_nextPutAll(char *s)
{
	char ch;
	
	while ((ch = *(s++)) != 0)
	{
		console_nextPut(ch);
	}
	return;
}

/****************************************************************************/

/*
 *	Return if a receive character is ready or not.
 */

int rs_is_char(void)
{
	volatile unsigned char *uartp;

	uartp = (volatile unsigned char *)(MCF_MBAR + consoleBase);
	return((uartp[MCFUART_USR] & MCFUART_USR_RXREADY) ? 1 : 0);
}

int rs_is_char_port(int port)
{
	volatile unsigned char *uartp;

	if (port == 0) 
	  uartp = (volatile unsigned char *) (MCF_MBAR + MCFUART_BASE1);
	else
	  uartp = (volatile unsigned char *) (MCF_MBAR + MCFUART_BASE2);

	return((uartp[MCFUART_USR] & MCFUART_USR_RXREADY) ? 1 : 0);
}

/****************************************************************************/
/*
 *	Input a single character from UART.
 */

int rs_get_char(void)
{
	volatile unsigned char *uartp;
	
	uartp = (volatile unsigned char *)(MCF_MBAR + consoleBase);
	return(uartp[MCFUART_URB]);
}

int rs_get_char_port(int port)
{
	volatile unsigned char	*uartp;

	if (port == 0) 
	  uartp = (volatile unsigned char *) (MCF_MBAR + MCFUART_BASE1);
	else
	  uartp = (volatile unsigned char *) (MCF_MBAR + MCFUART_BASE2);

	return(uartp[MCFUART_URB]);
}

/****************************************************************************/
