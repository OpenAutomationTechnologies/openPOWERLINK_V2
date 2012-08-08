/**
********************************************************************************
\file       systemComponents.c

\brief      Module which contains of processor specific functions
            (nios ii version)

Provides all functions which are platform dependent for the application of the
directIO example.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include "systemComponents.h"

#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include "nios2.h"
#include <sys/alt_cache.h>

#ifdef NODE_SWITCH_SPI_BASE
#include "altera_avalon_spi_regs.h"
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief               init the processor peripheral

Flush the instruction and data cache and reset the leds.
*/
//------------------------------------------------------------------------------
void SysComp_initPeripheral(void)
{
    alt_icache_flush_all();
    alt_dcache_flush_all();

    SysComp_setPowerlinkStatus(0xff);
}


//------------------------------------------------------------------------------
/**
\brief               free the processor cache

Flush and disable the instruction and data cache.
*/
//------------------------------------------------------------------------------
void SysComp_freeProcessorCache(void)
{
    alt_icache_flush_all();
    alt_dcache_flush_all();
}

//------------------------------------------------------------------------------
/**
\brief               enable global interrupts

Dummy function on Nios II as interrupts are enabled by default
*/
//------------------------------------------------------------------------------
void SysComp_enableInterrupts(void)
{
    //no interrupt enable needed on nios2 (fallthrough)
}

//------------------------------------------------------------------------------
/**
\brief              read the node ID from the available peripheral

This function reads the node id from the given board peripheral. If the board
is not supporting node switches zero is returned.

\return             nodeId
\retval             [1-239]         the given node id
*/
//------------------------------------------------------------------------------
BYTE SysComp_getNodeId(void)
{
    BYTE nodeId = 0;

    /* read port configuration input pins */
#ifdef NODE_SWITCH_BASE
    nodeId = IORD_ALTERA_AVALON_PIO_DATA(NODE_SWITCH_BASE);
#endif

#ifdef NODE_SWITCH_SPI_BASE
    // read node-ID from hex switch on baseboard, which is connected via SPI shift register
    IOWR_ALTERA_AVALON_SPI_TXDATA(NODE_SWITCH_BASE, 0xFF);   // generate pulse for latching inputs
    while ((IORD_ALTERA_AVALON_SPI_STATUS(NODE_SWITCH_BASE) & ALTERA_AVALON_SPI_STATUS_RRDY_MSK) == 0)
        ;

    nodeId = IORD_ALTERA_AVALON_SPI_RXDATA(NODE_SWITCH_BASE);
#endif

    return nodeId;
}


//------------------------------------------------------------------------------
/**
\brief              set the powerlink led

This function sets the powerlink status or error led.

\param              bBitNum_p       powerlink status (1: state; 2: error)
*/
//------------------------------------------------------------------------------
void SysComp_setPowerlinkStatus(BYTE bBitNum_p)
{
    #ifdef STATUS_LEDS_BASE
        IOWR_ALTERA_AVALON_PIO_SET_BITS(STATUS_LEDS_BASE, bBitNum_p);
    #endif
}

//------------------------------------------------------------------------------
/**
\brief             reset the powerlink led

This function resets the powerlink status or error led.

\param             bBitNum_p       powerlink status (1: state; 2: error)
*/
//------------------------------------------------------------------------------
void SysComp_resetPowerlinkStatus(BYTE bBitNum_p)
{
    #ifdef STATUS_LEDS_BASE
    IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(STATUS_LEDS_BASE, bBitNum_p);
    #endif
}
