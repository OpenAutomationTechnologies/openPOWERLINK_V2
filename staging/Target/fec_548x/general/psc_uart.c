/*
 * File:		psc_uart.c
 * Purpose:		Simple UART routines 
 *
 * Notes:		
 */

#include "common.h"
#include "assert.h"
#include "mcf548x_psc.h"
//#include "mcf548x_gpio.h"
#include "arch.h"
/********************************************************************/
void
psc_uart_init (int port, int baud)
{
	register uint16 divider;
	int t;

	/* 
	 * On Verdi, only PSC 0 & 1 are brought out to RS232 transceivers
	 */
	ASSERT(port >= 0 && port < 2);

	/* Set GPIO port register to enable PSC(port) signals */
	switch (port)
	{
	case 0:
		MCF_GPIO_PAR_PSC0 = (0
#ifdef HARDWARE_FLOW_CONTROL
			| MCF_GPIO_PAR_PSC0_PAR_CTS0_CTS
			| MCF_GPIO_PAR_PSC0_PAR_RTS0_RTS
#endif
			| MCF_GPIO_PAR_PSC0_PAR_TXD0
			| MCF_GPIO_PAR_PSC0_PAR_RXD0);
		break;
	case 1:
		MCF_GPIO_PAR_PSC1 = (0
#ifdef HARDWARE_FLOW_CONTROL
			| MCF_GPIO_PAR_PSC1_PAR_CTS1_CTS
			| MCF_GPIO_PAR_PSC1_PAR_RTS1_RTS
#endif
			| MCF_GPIO_PAR_PSC1_PAR_TXD1
			| MCF_GPIO_PAR_PSC1_PAR_RXD1);
		break;
	default:
		/* Only PSC0 and PSC1 can be used as the UART port on Verdi */
		return;
	}

	/* Put PSC in UART mode */
	MCF_PSC_SICR(port) = MCF_PSC_SICR_SIM_UART;

	/* Rx and Tx baud rate from timers */
	MCF_PSC_CSR(port) = (0
		| MCF_PSC_CSR_RCSEL_SYS_CLK 
		| MCF_PSC_CSR_TCSEL_SYS_CLK);

	/* Calculate baud settings */
	divider = (uint16)((MCF_CLK)/(baud * 32));
	MCF_PSC_CTUR(port) =  (uint8) ((divider >> 8) & 0xFF);
	MCF_PSC_CTLR(port) =  (uint8) (divider & 0xFF);

	/* Reset transmitter, receiver, mode register, and error conditions */
	MCF_PSC_CR(port) = MCF_PSC_CR_RESET_RX;
	MCF_PSC_CR(port) = MCF_PSC_CR_RESET_TX;
	MCF_PSC_CR(port) = MCF_PSC_CR_RESET_ERROR;
	MCF_PSC_CR(port) = MCF_PSC_CR_BKCHGINT;
	MCF_PSC_CR(port) = MCF_PSC_CR_RESET_MR;

	/* 8-bit data, no parity */
	MCF_PSC_MR(port) = (0
#ifdef UART_HARDWARE_FLOW_CONTROL
		| MCF_PSC_MR_RXRTS
#endif
		| MCF_PSC_MR_PM_NONE
		| MCF_PSC_MR_BC_8);

	/* No echo or loopback, 1 stop bit */
	MCF_PSC_MR(port) = (0
#ifdef UART_HARDWARE_FLOW_CONTROL
		| MCF_PSC_MR_TXCTS
#endif 
		| MCF_PSC_MR_CM_NORMAL
		| MCF_PSC_MR_SB_STOP_BITS_1);

	/* Mask all UART interrupts */
	MCF_PSC_IMR(port) = 0x0000;

	/* Enable RTS to send */
	MCF_PSC_OPSET(port) = MCF_PSC_OPSET_RTS;

	/* Setup FIFO Alarms */
	
	MCF_PSC_RFAR(port) = MCF_PSC_RFAR_ALARM(248);
	MCF_PSC_TFAR(port) = MCF_PSC_TFAR_ALARM(248);

	/* Enable receiver and transmitter */
	MCF_PSC_CR(port) =(0
		| MCF_PSC_CR_RX_ENABLED
		| MCF_PSC_CR_TX_ENABLED);
}
/********************************************************************/
char
psc_uart_in_char (int port)
{
	ASSERT(port >= 0 && port < 2);

	/* Wait until character has been received */
	while (!(MCF_PSC_SR(port) & MCF_PSC_SR_RXRDY))
		;
	return *((uint8 *) &MCF_PSC_RB(port));
}
/********************************************************************/
void
psc_uart_out_char (int port, char ch)
{
	ASSERT(port >= 0 && port < 2);

	/* Wait until space is available in the FIFO */
	while (!(MCF_PSC_SR(port) & MCF_PSC_SR_TXRDY))
		;
	*((uint8 *) &MCF_PSC_TB(port)) = ch;
}
/********************************************************************/
int
psc_uart_char_present (int port)
{
	ASSERT(port >= 0 && port < 2);

	return (MCF_PSC_SR(port) & MCF_PSC_SR_RXRDY);
}
/********************************************************************/
