/*
 * File:		io.c
 * Purpose:		Serial Input/Output routines
 *
 * Notes:       TERMINAL_PORT defined in <board>.h
 */

#include "plccore.h"
#include "psc_uart.h"
#include "io.h"

/********************************************************************/
char
in_char (void)
{
	return psc_uart_in_char(TERMINAL_PORT);
}
/********************************************************************/
void
out_char (char ch)
{
	psc_uart_out_char(TERMINAL_PORT, ch);
}
/********************************************************************/
int
char_present (void)
{
	return psc_uart_char_present(TERMINAL_PORT);
}
/********************************************************************/
