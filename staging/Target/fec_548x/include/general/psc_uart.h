/*
 * File:		psc_uart.h
 * Purpose:		Simple UART routines 
 *
 * Notes:
 */

#ifndef _PSC_UART_H_
#define _PSC_UART_H_

/********************************************************************/

void psc_uart_init (int, int);
char psc_uart_in_char (int);
void psc_uart_out_char (int, char);
int  psc_uart_char_present (int);

/********************************************************************/

#endif /* _PSC_UART_H_ */
