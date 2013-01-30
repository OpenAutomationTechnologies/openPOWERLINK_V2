/************************************************************************
*
*  FILE NAME: isr.h
*
*  PURPOSE: Interrupt service dispatcher function definitions, data structures, etc.
*
*  AUTHOR: Oleg Boguslavsky
*
*************************************************************************/

#ifndef _ISR_H_
#define	_ISR_H_

// queue of interrupt descriptors
struct irq_info
{
     struct irq_info * next;  // next irq in chain
     unsigned int irq;        // irq number
     void (*handler)(void);   // irq handler
     unsigned int pended;     // indicates interrupt pending
};

// Interrupt dispatcher functions
unsigned long isr_set_vector(unsigned int irq_number, void (*handler)(void));
void isr_reset_vector(unsigned int irq_number, void (*handler)(void));
void isr_generate_irq(unsigned int irq_number);

void isr_lock(void); 
void isr_unlock(void); 

#endif
