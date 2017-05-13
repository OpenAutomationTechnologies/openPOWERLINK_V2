/**
********************************************************************************
\file   mb_uart.c

\brief  Board specific Microblaze UART implementation

This file contains the Xilinx ZC702 board specific Microblaze UART
implementation. The Microblaze stdout is redirected to the PS UART component.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Kalycito Infotech Private Limited
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
#include <xil_io.h>
#include <mb_uart.h>

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
// base address of PS Uart1
#define UART_BASE    0xE0001000

// Write to memory location or register
#define WRITE_REG(base, offset, data) \
    * (unsigned int*)(base + offset) = ((unsigned int) data);

// Read from memory location or register
#define READ_REG(base, offset) \
    * (unsigned int*)(base + offset);

// Modified Ua
#define UART_ISTRANSMIT_FULL(base) \
    ((Xil_In32((base) + 0x2C) & 0x10) == 0x10)

#define UART_SEND_BYTE(base, data)            \
    while (UART_ISTRANSMIT_FULL((u32)base)) ; \
    WRITE_REG((u32)base, 0x30, (u8)data)

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
\brief Initialization rouinte for UART module


\param  char_p            Character too be sent

*/
//------------------------------------------------------------------------------
void uart_init(void)
{
    // Dummy initialization routine to enable indexing of this module
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Re-definition of standard BSP outbyte for Zynq

This will redirect prints from Microblaze to the common UART
device on Zynq platform which is not handled by the generated BSP.

\param  char_p            Character too be sent

*/
//------------------------------------------------------------------------------
void outbyte(char char_p)
{
    UART_SEND_BYTE(UART_BASE, char_p);
}

///\}
