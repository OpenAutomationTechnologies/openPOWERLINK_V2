/**
********************************************************************************
\file   cachemem-xilinx.c

\brief  Xilinx data cache aware memory implementation

The file implements data cache aware memory access for Xilinx ARM (Zynq) and
Microblaze.
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
#include <xil_cache.h>
#include <xil_types.h>  // Use u8, u16 and u32 data types
#include <xil_io.h>
#include <xparameters.h>

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

// Define data cache invalidate and flush if data cache is available
#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define CACHEMEM_FLUSH(addr, len)       microblaze_flush_dcache_range(addr,len)
#define CACHEMEM_INVALIDATE(addr, len)  microblaze_invalidate_dcache_range(addr, len)
#else
// Empty macros, because there is no cache!
#define CACHEMEM_FLUSH(addr, len)
#define CACHEMEM_INVALIDATE(addr, len)
#endif

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
\brief  Read 32 bit data cache aware

This function reads 32 bit data at the given address + offset with prior data
cache invalidation.

\param  base_p      Base address to be read
\param  offset_p    Offset to be read

\return 32 bit read data of the given type
 */
//------------------------------------------------------------------------------
unsigned long cachemem_invAndRead32(void* base_p, unsigned int offset_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    CACHEMEM_INVALIDATE(address, sizeof(u32));

    return (unsigned long)Xil_In32(address);
}

//------------------------------------------------------------------------------
/**
\brief  Read 16 bit data cache aware

This function reads 16 bit data at the given address + offset with prior data
cache invalidation.

\param  base_p      Base address to be read
\param  offset_p    Offset to be read

\return 16 bit read data of the given type
 */
//------------------------------------------------------------------------------
unsigned short cachemem_invAndRead16(void* base_p, unsigned int offset_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    CACHEMEM_INVALIDATE(address, sizeof(u16));

    return (unsigned short)Xil_In16(address);
}

//------------------------------------------------------------------------------
/**
\brief  Read 8 bit data cache aware

This function reads 8 bit data at the given address + offset with prior data
cache invalidation.

\param  base_p      Base address to be read
\param  offset_p    Offset to be read

\return 8 bit read data of the given type
 */
//------------------------------------------------------------------------------
unsigned char cachemem_invAndRead8(void* base_p, unsigned int offset_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    CACHEMEM_INVALIDATE(address, sizeof(u8));

    return (unsigned char)Xil_In8(address);
}

//------------------------------------------------------------------------------
/**
\brief  Write 32 bit data cache aware

This function writes 32 bit data to the given address + offset with data cache
flush afterwards.

\param  base_p      Base address to writer
\param  offset_p    Offset to write
\param  val_p       32 bit value to write
 */
//------------------------------------------------------------------------------
void cachemem_writeAndFlush32(void* base_p, unsigned int offset_p, unsigned long val_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    Xil_Out32(address, (u32)val_p);

    CACHEMEM_FLUSH(address, sizeof(u32));
}

//------------------------------------------------------------------------------
/**
\brief  Write 16 bit data cache aware

This function writes 16 bit data to the given address + offset with data cache
flush afterwards.

\param  base_p      Base address to writer
\param  offset_p    Offset to write
\param  val_p       16 bit value to write
 */
//------------------------------------------------------------------------------
void cachemem_writeAndFlush16(void* base_p, unsigned int offset_p, unsigned short val_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    Xil_Out16(address, (u16)val_p);

    CACHEMEM_FLUSH(address, sizeof(u16));
}

//------------------------------------------------------------------------------
/**
\brief  Write 8 bit data cache aware

This function writes 8 bit data to the given address + offset with data cache
flush afterwards.

\param  base_p      Base address to writer
\param  offset_p    Offset to write
\param  val_p       8 bit value to write
 */
//------------------------------------------------------------------------------
void cachemem_writeAndFlush8(void* base_p, unsigned int offset_p, unsigned char val_p)
{
    unsigned int address = (unsigned int) base_p + offset_p;

    Xil_Out8(address, (u8)val_p);

    CACHEMEM_FLUSH(address, sizeof(u8));
}
