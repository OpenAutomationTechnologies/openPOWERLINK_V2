/************************************************************************
*
*  FILE NAME: isr.c
*
*  PURPOSE: Interrupt service dispatcher implementation. 
*
*  AUTHOR: Oleg Boguslavsky
*
*************************************************************************/
#include "isr.h"
//#include "timer.h"

int isr_counter = 0;

/************************************************************************
* NAME: isr_lock
*
* DESCRIPTION: After execution of this routine all interrupts will be 
*              pended
*************************************************************************/

void isr_lock(void) 
{
     __asm__ __volatile__ ("move  #0x2700,%SR\n\t");
	isr_counter++;

}

/************************************************************************
* NAME: isr_unlock
*
* DESCRIPTION: Executes all pending interrupt handlers and 
*              enables hardware interrupts processing
*************************************************************************/

void isr_unlock(void) 
{
    isr_counter--;
	if(!isr_counter)
	{
       __asm__ __volatile__ ("move  #0x2000,%SR\n\t");
	}
}
