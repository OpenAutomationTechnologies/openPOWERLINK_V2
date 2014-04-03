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
              www.kalycito.com
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

//------------------------------------------------------------------------------
/**
\brief  Register ISR for hostif

\param  BaseAddress_p    base address of Device
\param  InterruptId_p    interrupt ID for which handler is to be registered
\param  Handler_p        callback ISR routine
\param  CallBackRef_p    reference argument to be passed with ISR.

\return 1 Success

\ingroup module_hostiflib Process Queue
*/
//------------------------------------------------------------------------------
int hostiflib_RegisterHandler (u32 BaseAddress_p, int InterruptId_p,
            XInterruptHandler Handler_p, void *CallBackRef_p)
{
    XIntc_RegisterHandler(BaseAddress_p,InterruptId_p,Handler_p,CallBackRef_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  READ functions with Invalidation

\param  dwBase_p    base address to be READ/WRITE
\param  offset_p    Offset to be READ/WRITE

\ret    read data of the given type

 */
//------------------------------------------------------------------------------
u32 MB_READ32(u32 dwBase_p,u32 offset_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    microblaze_invalidate_dcache_range(Address, 4);
    return Xil_In32(Address);
}

u16 MB_READ16(u32 dwBase_p,u32 offset_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    microblaze_invalidate_dcache_range(Address, 2);
    return Xil_In16(Address);

}

u8 MB_READ8(u32 dwBase_p,u32 offset_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    microblaze_invalidate_dcache_range(Address, 1);
    return Xil_In8(Address);
}

//------------------------------------------------------------------------------
/**
\brief  WRITE functions with flushing

\param  dwBase_p    base address to WRITE
\param  offset_p    Offset to WRITE
\param  Val_p       Value to WRITE

\ret    void
 */
//------------------------------------------------------------------------------

void MB_WRITE32(u32 dwBase_p,u32 offset_p,u32 Val_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    Xil_Out32(Address,Val_p);
    microblaze_flush_dcache_range(Address, 4);
}

void MB_WRITE16(u32 dwBase_p,u32 offset_p,u16 Val_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    Xil_Out16(Address,Val_p);
    microblaze_flush_dcache_range(Address, 2);
}

void MB_WRITE8(u32 dwBase_p,u32 offset_p,u8 Val_p)
{
    u32 Address = (u32) dwBase_p + offset_p;
    Xil_Out8(Address,Val_p);
    microblaze_flush_dcache_range(Address, 1);

}

//EOF
