/**
********************************************************************************
\file   lcd.c

\brief  Generic LCD interface

The generic LCD interface module enables to control any LCD.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.
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
#include <string.h>
#include <oplk/oplk.h>
#include "lcd.h"
#include "lcdl.h"


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
#define LCD_COLUMN  16  ///< Minimum line size needed

static const char aStrNmtState_l[10][LCD_COLUMN + 1] =
{
        "INVALID          ",
        "OFF              ",
        "INITIALISATION   ",
        "NOT ACTIVE       ",
        "BASIC ETHERNET   ",
        "PRE_OP1          ",
        "PRE_OP2          ",
        "READY_TO_OP      ",
        "OPERATIONAL      ",
        "STOPPED          "
};

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
\brief  Initialize the LCD

The function initializes the generic LCD instance.

*/
//------------------------------------------------------------------------------
void lcd_init(void)
{
    // Initialize low-level module
    lcdl_init();
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the LCD

The function exits the generic LCD instance.
*/
//------------------------------------------------------------------------------
void lcd_exit(void)
{
    // Shutdown low-level module
    lcdl_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Clear the LCD

The function clears the display.
*/
//------------------------------------------------------------------------------
void lcd_clear(void)
{
    lcdl_clear();
}

//------------------------------------------------------------------------------
/**
\brief  Print text to LCD

The function prints the provided text to the specified line in the LCD.

\param[in]      sText_p             Text to be printed
\param[in]      line_p              Line to print the text in
*/
//------------------------------------------------------------------------------
void lcd_printText(const char* sText_p,
                   UINT line_p)
{
    if (lcdl_changeToLine(line_p) != 0)
        return;

    lcdl_printText(sText_p);
}

//------------------------------------------------------------------------------
/**
\brief  Print NMT state to LCD

The function prints the NMT state to the second line of the display.

\param[in]      nmtState_p          NMT state to be written
*/
//------------------------------------------------------------------------------
void lcd_printNmtState(tNmtState nmtState_p)
{
    if (lcdl_changeToLine(2) != 0)
        return;

    switch (nmtState_p)
    {
        case kNmtGsOff:
            lcdl_printText(aStrNmtState_l[1]);
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
            lcdl_printText(aStrNmtState_l[2]);
            break;

        case kNmtCsNotActive:
        case kNmtMsNotActive:
            lcdl_printText(aStrNmtState_l[3]);
            break;

        case kNmtCsBasicEthernet:
        case kNmtMsBasicEthernet:
            lcdl_printText(aStrNmtState_l[4]);
            break;

        case kNmtCsPreOperational1:
        case kNmtMsPreOperational1:
            lcdl_printText(aStrNmtState_l[5]);
            break;

        case kNmtCsPreOperational2:
        case kNmtMsPreOperational2:
            lcdl_printText(aStrNmtState_l[6]);
            break;

        case kNmtCsReadyToOperate:
        case kNmtMsReadyToOperate:
            lcdl_printText(aStrNmtState_l[7]);
            break;

        case kNmtCsOperational:
        case kNmtMsOperational:
            lcdl_printText(aStrNmtState_l[8]);
            break;

        case kNmtCsStopped:
            lcdl_printText(aStrNmtState_l[9]);
            break;

        default:
            lcdl_printText(aStrNmtState_l[0]);
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Print node ID to LCD

The function prints the provided node ID to the first line of the display.
In addition to the printed node ID 'MN' (=0xF0) or 'CN' is added.

\param[in]      nodeId_p            node ID to be written
*/
//------------------------------------------------------------------------------
void lcd_printNodeId(UINT8 nodeId_p)
{
    char textNodeID[LCD_COLUMN+1];

    sprintf(textNodeID,
            "NodeID=0x%02X (%s)",
            nodeId_p,
            (nodeId_p == C_ADR_MN_DEF_NODE_ID) ? "MN" : "CN");

    if (lcdl_changeToLine(1) != 0)
        return;

    lcdl_printText(textNodeID);
}

//------------------------------------------------------------------------------
/**
\brief  Print error code to LCD

The function prints the provided error code to the second line of the display.

\param[in]      error_p             error code
*/
//------------------------------------------------------------------------------
void lcd_printError(tOplkError error_p)
{
    char textError[LCD_COLUMN+1];

    sprintf(textError, "ERROR=0x%04X", error_p);

    if (lcdl_changeToLine(2) != 0)
        return;

    lcdl_printText(textError);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
