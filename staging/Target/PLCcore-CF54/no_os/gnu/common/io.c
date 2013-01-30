/*
 * File:		io.c
 * Purpose:		Serial Input/Output routines
 *
 * Notes:       TERMINAL_PORT defined in <board>.h
 */

#include "../include/common.h"
#include "../include/uartdrv.h"
#include "../include/io.h"

/********************************************************************/
char
in_char (void)
{
	return UartInChar(TERMINAL_PORT);
}
/********************************************************************/
void
out_char (char ch)
{
	UartOutChar(TERMINAL_PORT, ch);
}
/********************************************************************/
int
char_present (void)
{
	return UartCharPresent(TERMINAL_PORT);
}
/********************************************************************/
