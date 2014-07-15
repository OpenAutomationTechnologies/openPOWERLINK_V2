/**
********************************************************************************
\file   hostiflib_microblaze.c

\brief  Host Interface Library Support File - For Microblaze target

This file provides specific function definition for Xilinx Microblaze CPU to
support host interface

\ingroup module_hostiflib
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
------------------------------------------------------------------------------*/
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_microblaze.h"

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

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Register ISR for hostif

\param  BaseAddress_p    Base address of Device
\param  InterruptId_p    Interrupt ID for which handler is to be registered
\param  Handler_p        Callback ISR routine
\param  CallBackRef_p    Reference argument to be passed with ISR.

\return 0                Success

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
int hostiflib_registerHandler(u32 baseAddress_p,
                              int interruptId_p,
                              XInterruptHandler handler_p,
                              void* callBackRef_p)
{
    XIntc_RegisterHandler(baseAddress_p, interruptId_p, handler_p, callBackRef_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read functions with Invalidation

\param  base_p      Base address to be read
\param  offset_p    Offset to be read

\return Read data of the given type
 */
//------------------------------------------------------------------------------
u32 mb_read32(u32 base_p, u32 offset_p)
{
    u32    address = (u32) base_p + offset_p;
    microblaze_invalidate_dcache_range(address, 4);
    return Xil_In32(address);
}

u16 mb_read16(u32 base_p, u32 offset_p)
{
    u32    address = (u32) base_p + offset_p;
    microblaze_invalidate_dcache_range(address, 2);
    return Xil_In16(address);
}

u8 mb_read8(u32 base_p, u32 offset_p)
{
    u32    address = (u32) base_p + offset_p;
    microblaze_invalidate_dcache_range(address, 1);
    return Xil_In8(address);
}

//------------------------------------------------------------------------------
/**
\brief  Write functions with flushing

\param  base_p      Base address to writer
\param  offset_p    Offset to write
\param  val_p       Value to write
 */
//------------------------------------------------------------------------------
void mb_write32(u32 base_p, u32 offset_p, u32 val_p)
{
    u32    address = (u32) base_p + offset_p;
    Xil_Out32(address, val_p);
    microblaze_flush_dcache_range(address, 4);
}

void mb_write16(u32 base_p, u32 offset_p, u16 val_p)
{
    u32    address = (u32) base_p + offset_p;
    Xil_Out16(address, val_p);
    microblaze_flush_dcache_range(address, 2);
}

void mb_write8(u32 base_p, u32 offset_p, u8 val_p)
{
    u32    address = (u32) base_p + offset_p;
    Xil_Out8(address, val_p);
    microblaze_flush_dcache_range(address, 1);
}
