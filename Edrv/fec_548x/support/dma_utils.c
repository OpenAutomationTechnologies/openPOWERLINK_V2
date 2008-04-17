/*
 * File:	dma_utils.c
 * Purpose: General purpose utilities for the multi-channel DMA
 *
 * Notes:		
 */

#include "common.h"
#include "dma_utils.h"
#include "mcf548x/mcf548x_intc.h"
#include "mcf548x_dma.h"

#define BENCHMARK

/* 
 * This global keeps track of which initiators have been
 * used of the available assignments.  Initiators 0-15 are
 * hardwired.  Initiators 16-31 are multiplexed and controlled
 * via the Initiatior Mux Control Registe (IMCR).  The 
 * assigned requestor is stored with the associated initiator
 * number.  
 */
static int used_reqs[32] = 
{
    DMA_ALWAYS,  DMA_DSPI_RX, DMA_DSPI_TX, DMA_DREQ0,
    DMA_PSC0_RX, DMA_PSC0_TX, DMA_USBEP0,  DMA_USBEP1,
    DMA_USBEP2,  DMA_USBEP3,  DMA_PCI_TX,  DMA_PCI_RX,
    DMA_PSC1_RX, DMA_PSC1_TX, DMA_I2C_RX,  DMA_I2C_TX,
    0,           0,           0,           0,  
    0,           0,           0,           0,  
    0,           0,           0,           0,  
    0,           0,           0,           0
};

/*
 * This global keeps track of which channels have been assigned
 * to tasks.  This methology assumes that no single initiator
 * will be tied to more than one task/channel
 */
static char used_channel[16] = 
{
    -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1
};

unsigned int connected_channel[16] = 
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};
/********************************************************************/
/*
 * Enable all DMA interrupts
 *
 * Parameters:
 *  pri     Interrupt Priority
 *  lvl     Interrupt Level
 */
void
dma_irq_enable(uint8 lvl, uint8 pri)
{
    ASSERT(lvl > 0 && lvl < 8);
    ASSERT(pri < 8);

    /* Setup the DMA ICR (#48) */
    MCF_INTC_ICR48 = 0
        | MCF_INTC_ICRn_IP(pri)
        | MCF_INTC_ICRn_IL(lvl);

    /* Unmask all task interrupts */
    MCF_DMA_DIMR = 0;

    /* Clear the interrupt pending register */
    MCF_DMA_DIPR = 0;

    /* Unmask the DMA interrupt in the interrupt controller */
    MCF_INTC_IMRH &= ~MCF_INTC_IMRH_INT_MASK48;
}
/********************************************************************/
/*
 * Disable all DMA interrupts
 */
void
dma_irq_disable(void)
{
    /* Mask all task interrupts */
    MCF_DMA_DIMR = (uint32)~0;

    /* Clear any pending task interrupts */
    MCF_DMA_DIPR = (uint32)~0;

    /* Mask the DMA interrupt in the interrupt controller */
    MCF_INTC_IMRH |= MCF_INTC_IMRH_INT_MASK48;
}
/********************************************************************/
/*
 * Attempt to enable the provided Initiator in the Initiator
 * Mux Control Register
 *
 * Parameters:
 *  initiator   Initiator identifier
 *
 * Return Value:
 *  1   if unable to make the assignment
 *  0   successful
 */
int
dma_set_initiator(int initiator)
{
    switch (initiator)
    {
        /* These initiators are always active */
        case DMA_ALWAYS:
        case DMA_DSPI_RX:
        case DMA_DSPI_TX: 
        case DMA_DREQ0: 
        case DMA_PSC0_RX: 
        case DMA_PSC0_TX: 
        case DMA_USBEP0: 
        case DMA_USBEP1: 
        case DMA_USBEP2: 
        case DMA_USBEP3: 
        case DMA_PCI_TX: 
        case DMA_PCI_RX: 
        case DMA_PSC1_RX: 
        case DMA_PSC1_TX: 
        case DMA_I2C_RX: 
        case DMA_I2C_TX:
            break;
        case DMA_FEC0_RX:
            MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC16(3)) 
                            | MCF_DMA_IMCR_SRC16_FEC0RX;
            used_reqs[16] = DMA_FEC0_RX;
            break;
        case DMA_FEC0_TX:
            MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC17(3)) 
                            | MCF_DMA_IMCR_SRC17_FEC0TX;
            used_reqs[17] = DMA_FEC0_TX;
            break;
        case DMA_FEC1_RX:
            MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC20(3)) 
                            | MCF_DMA_IMCR_SRC20_FEC1RX;
            used_reqs[20] = DMA_FEC1_RX;
            break;
        case DMA_FEC1_TX:
            if (used_reqs[21] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC21(3)) 
                                | MCF_DMA_IMCR_SRC21_FEC1TX;
                used_reqs[21] = DMA_FEC1_TX;
            }
            else if (used_reqs[25] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC25(3)) 
                                | MCF_DMA_IMCR_SRC25_FEC1TX;
                used_reqs[25] = DMA_FEC1_TX;
            }
            else if (used_reqs[31] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC31(3)) 
                                | MCF_DMA_IMCR_SRC31_FEC1TX;
                used_reqs[31] = DMA_FEC1_TX;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_DREQ1:
            if (used_reqs[29] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC29(3)) 
                                | MCF_DMA_IMCR_SRC29_DREQ1;
                used_reqs[29] = DMA_DREQ1;
            }
            else if (used_reqs[21] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC21(3)) 
                                | MCF_DMA_IMCR_SRC21_DREQ1;
                used_reqs[21] = DMA_DREQ1;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM0:
            if (used_reqs[24] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC24(3)) 
                                | MCF_DMA_IMCR_SRC24_CTM0;
                used_reqs[24] = DMA_CTM0;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM1:
            if (used_reqs[25] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC25(3)) 
                                | MCF_DMA_IMCR_SRC25_CTM1;
                used_reqs[25] = DMA_CTM1;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM2:
            if (used_reqs[26] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC26(3)) 
                                | MCF_DMA_IMCR_SRC26_CTM2;
                used_reqs[26] = DMA_CTM2;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM3:
            if (used_reqs[27] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC27(3)) 
                                | MCF_DMA_IMCR_SRC27_CTM3;
                used_reqs[27] = DMA_CTM3;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM4:
            if (used_reqs[28] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC28(3)) 
                                | MCF_DMA_IMCR_SRC28_CTM4;
                used_reqs[28] = DMA_CTM4;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM5:
            if (used_reqs[29] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC29(3)) 
                                | MCF_DMA_IMCR_SRC29_CTM5;
                used_reqs[29] = DMA_CTM5;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM6:
            if (used_reqs[30] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC30(3)) 
                                | MCF_DMA_IMCR_SRC30_CTM6;
                used_reqs[30] = DMA_CTM6;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_CTM7:
            if (used_reqs[31] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC31(3)) 
                                | MCF_DMA_IMCR_SRC31_CTM7;
                used_reqs[31] = DMA_CTM7;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_USBEP4:
            if (used_reqs[26] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC26(3)) 
                                | MCF_DMA_IMCR_SRC26_USBEP4;
                used_reqs[26] = DMA_USBEP4;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_USBEP5:
            if (used_reqs[27] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC27(3)) 
                                | MCF_DMA_IMCR_SRC27_USBEP5;
                used_reqs[27] = DMA_USBEP5;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_USBEP6:
            if (used_reqs[28] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC28(3)) 
                                | MCF_DMA_IMCR_SRC28_USBEP6;
                used_reqs[28] = DMA_USBEP6;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_PSC2_RX:
            if (used_reqs[28] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC28(3)) 
                                | MCF_DMA_IMCR_SRC28_PSC2RX;
                used_reqs[28] = DMA_PSC2_RX;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_PSC2_TX:
            if (used_reqs[29] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC29(3)) 
                                | MCF_DMA_IMCR_SRC29_PSC2TX;
                used_reqs[29] = DMA_PSC2_TX;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_PSC3_RX:
            if (used_reqs[30] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC30(3)) 
                                | MCF_DMA_IMCR_SRC30_PSC3RX;
                used_reqs[30] = DMA_PSC3_RX;
            }
            else /* No empty slots */
                return 1;
            break;
        case DMA_PSC3_TX:
            if (used_reqs[31] == 0)
            {
                MCF_DMA_IMCR = (MCF_DMA_IMCR & ~MCF_DMA_IMCR_SRC31(3)) 
                                | MCF_DMA_IMCR_SRC31_PSC3TX;
                used_reqs[31] = DMA_PSC3_TX;
            }
            else /* No empty slots */
                return 1;
            break;
        default:
            return 1;
    }
    return 0;
}
/********************************************************************/
/*
 * Return the initiator number for the given requestor
 *
 * Parameters:
 *  requestor   Initiator/Requestor identifier
 *
 * Return Value:
 *  The initiator number (0-31) if initiator has been assigned
 *  0 (always initiator) otherwise
 */
unsigned int
dma_get_initiator(int requestor)
{
    uint32 i;

    for (i=0; i<sizeof(used_reqs); ++i)
    {
        if (used_reqs[i] == requestor)
            return i;
    }
    return 0;
}
/********************************************************************/
/*
 * Remove the given initiator from the active list
 *
 * Parameters:
 *  requestor   Initiator/Requestor identifier
 */
void
dma_remove_initiator(int requestor)
{
    uint32 i;

    for (i=0; i<sizeof(used_reqs); ++i)
    {
        if (used_reqs[i] == requestor)
        {
            used_reqs[i] = -1;
            break;
        }
    }
}

/********************************************************************/
/*
 * Attempt to find an available channel and mark is as used
 *
 * Parameters:
 *  requestor   Initiator/Requestor identifier
 *
 * Return Value:
 *  First available channel (from 6 to 15) or -1 if they are all occupied
 */
int
dma_set_channel(int requestor)
{
    uint32 i;

    for (i=0; i<sizeof(used_channel); ++i)
    {
        if (used_channel[i] == -1)
        {
            used_channel[i] = requestor;
            return i;
        }
    }

    /* All channels taken */
    return -1;
}

/********************************************************************/
/*
 * Return the channel being initiated by the given requestor
 *
 * Parameters:
 *  requestor   Initiator/Requestor identifier
 */
int
dma_get_channel(int requestor)
{
    uint32 i;

    for (i=0; i<sizeof(used_channel); ++i)
    {
        if (used_channel[i] == requestor)
            return i;
    }
    return -1;
}

/********************************************************************/
/*
 * Connects a channel with reference on your data 
 *
 * Parameters:
 *  channel   channel number
 *  reference addres of your data
  */
int
dma_connect(int channel, int address)
{
  if ((channel<16)&&(channel>=0))
    connected_channel[channel]=address;
  else
    return -1;
  return 0;  
}

/********************************************************************/
/*
 * Disconnects a channel with reference on your data 
 *
 * Parameters:
 *  channel   channel number
*/
int
dma_disconnect(int channel)
{
  if ((channel<16)&&(channel>=0))
    connected_channel[channel]=0;
  else
    return -1;
  return 0;  
}


/********************************************************************/
/*
 * Remove the channel being initiated by the given requestor from 
 * the active list
 *
 * Parameters:
 *  requestor   Initiator/Requestor identifier
 */
void
dma_remove_channel(int requestor)
{
    uint32 i;

    for (i=0; i<sizeof(used_channel); ++i)
    {
        if (used_channel[i] == requestor)
        {
            used_channel[i] = -1;
            break;
        }
    }
}
/********************************************************************/
/* 
 * This is the catch-all interrupt handler for the mult-channel DMA 
 */
volatile uint8 dma_iflag[16];
uint32 tx = 0;

void dma_interrupt_handler(void) {    
    __asm__ __volatile__ ("lea  %sp@(-24),%sp\n\t"   
		           "moveml  %d0-%d2/%a0-%a2,%sp@\n\t" 
			   "jsr    dma_interrupt_handler_real\n\t" 
		           "moveml  %sp@,%d0-%d2/%a0-%a2\n\t"
			   "lea  %sp@(24),%sp\n\t"
			   "unlk %fp\n\t"
			   "rte\n\t");
}

void dma_interrupt_handler_real (void) 
{


    uint32 i, interrupts;

#ifdef BENCHMARK
    // reset LED
    MCF_GPIO_PODR_PCIBG &= ~0x02;  // Level
#endif

    //  Determine which interrupt(s) triggered by AND'ing the
    // pending interrupts with those that aren't masked.
    interrupts = MCF_DMA_DIPR & ~MCF_DMA_DIMR;

    // Make sure we are here for a reason
    ASSERT(interrupts != 0);

    // Clear the interrupt in the pending register
    MCF_DMA_DIPR = interrupts;

    for (i=0; i<16; ++i, interrupts>>=1)
    {
        if (interrupts & 0x1)
	{
           if(connected_channel[i] != 0)
	        ((void (*)(void))(connected_channel[i]))();	   
	}
        
    }

#ifdef BENCHMARK
    // set LED
    MCF_GPIO_PODR_PCIBG |= 0x02;  // Level
#endif

}

void dma_remove_channel_by_number(int channel)
{
   if(channel < sizeof(used_channel) && channel >= 0)
      used_channel[channel] = -1;
}

/********************************************************************/
// to do: register interrupt handlers
//        call registered handlers from dma_interrupt_handler()
