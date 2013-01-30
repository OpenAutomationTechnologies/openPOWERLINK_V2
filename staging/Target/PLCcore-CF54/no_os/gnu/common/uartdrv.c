/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  driver for the psc uart

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                DevC++
                GNU C 3.3.2

  -------------------------------------------------------------------------

  Revision History:

  2005/08/04 -ct:   start of the implementation

****************************************************************************/

#include "..\include\common.h"

#include "..\include\uartdrv.h"

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------




/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <UART>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
void UartInit (int Port_p, uint32 dwBaudrate_p)
{
uint16 wCRV;


	// calculate counter reference value for the baudrate
	wCRV = (uint16)(((SYSTEM_CLOCK *1000000) / 32) / dwBaudrate_p);

	// set GPIO port register to enable PSC(port) signals
	switch (Port_p)
	{
	case 0:	MCF_GPIO_PAR_PSC0 = MCF_GPIO_PAR_PSC0_PAR_TXD0 |
			                    MCF_GPIO_PAR_PSC0_PAR_RXD0 ;

			break;

	case 1: MCF_GPIO_PAR_PSC1 = MCF_GPIO_PAR_PSC1_PAR_TXD1 |
								MCF_GPIO_PAR_PSC1_PAR_RXD1 ;
			break;

	case 2: MCF_GPIO_PAR_PSC2 = MCF_GPIO_PAR_PSC2_PAR_TXD2 |
								MCF_GPIO_PAR_PSC2_PAR_RXD2 ;
			break;

	case 3: MCF_GPIO_PAR_PSC3 = MCF_GPIO_PAR_PSC3_PAR_TXD3 |
								MCF_GPIO_PAR_PSC3_PAR_RXD3 ;
			break;

	default:
			return;
	}

	// Put PSCx in UART mode
	MCF_PSC_SICR(Port_p) = 0;

	// Rx and Tx baud rate from comm timer 0 to 3 */
	MCF_PSC_CSR(Port_p) = MCF_PSC_CSR_RCSEL(0xD) |
						   MCF_PSC_CSR_TCSEL(0xD) ;

    MCF_PSC_CTUR(Port_p) = (uint8)(wCRV >> 8);
    MCF_PSC_CTLR(Port_p) = (uint8)wCRV;

	// Reset transmitter, receiver, mode register, and error conditions
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_RESET_RX;
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_RESET_TX;
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_RESET_ERROR;
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_BKCHGINT;
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_RESET_MR;

	// 8-bit data, no parity
	MCF_PSC_MR(Port_p) = MCF_PSC_MR_PM_NONE |
	                      MCF_PSC_MR_BC_8    ;

	// normal mode, 1 stop bit
	MCF_PSC_MR(Port_p) = MCF_PSC_MR_CM_NORMAL |
	                      MCF_PSC_MR_SB_STOP_BITS_1;

	// Mask all UART interrupts
	MCF_PSC_IMR(Port_p) = 0x0000;

	/* Setup FIFO Alarms */
	MCF_PSC_RFAR(Port_p) = MCF_PSC_RFAR_ALARM(248);
	MCF_PSC_TFAR(Port_p) = MCF_PSC_TFAR_ALARM(248);

	// Enable receiver and transmitter
	MCF_PSC_CR(Port_p) = MCF_PSC_CR_RX_ENABLED |
	                      MCF_PSC_CR_TX_ENABLED ;
}

/********************************************************************/
char UartInChar (int Port_p)
{

	// Wait until character has been received
	while (!(MCF_PSC_SR(Port_p) & MCF_PSC_SR_RXRDY))
		;
	return *((uint8 *) &MCF_PSC_RB(Port_p));
}
/********************************************************************/
void UartOutChar (int Port_p, char cData_p)
{

	// Wait until space is available in the FIFO
	while (!(MCF_PSC_SR(Port_p) & MCF_PSC_SR_TXRDY))
		;
	*((uint8 *) &MCF_PSC_TB(Port_p)) = cData_p;
}
/********************************************************************/
int UartCharPresent (int Port_p)
{

	return (MCF_PSC_SR(Port_p) & MCF_PSC_SR_RXRDY);
}
/********************************************************************/
