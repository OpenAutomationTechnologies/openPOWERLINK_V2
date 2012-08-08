/**
********************************************************************************
\file       Cmp_Lcd.c

\brief      Non generic lcd functions for the TERASIC board

Application of the directIO example which starts the openPOWERLINK stack and
implements AppCbSync and AppCbEvent.

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
#include "Cmp_Lcd.h"
#include "lcd.h"
#include "system.h"


#ifdef LCD_BASE // LCD module present

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

static char aStrNmtState_l[10][17] = {"INVALID         ",
                                     "OFF             ",
                                     "INITIALISATION  ",
                                     "NOT ACTIVE      ",
                                     "BASIC ETHERNET  ",
                                     "PRE_OP1         ",
                                     "PRE_OP2         ",
                                     "READY_TO_OP     ",
                                     "OPERATIONAL     ",
                                     "STOPPED         " };

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief               Init the LCD display

Calls the generic init LCD function
*/
//------------------------------------------------------------------------------
void SysComp_LcdInit(void)
{
    LCD_Init();
}

//------------------------------------------------------------------------------
/**
\brief               Clear the LCD display

Calls the generic clear LCD function
*/
//------------------------------------------------------------------------------
void SysComp_LcdClear(void)
{
    LCD_Clear();
}

//------------------------------------------------------------------------------
/**
\brief               Write text to LCD display

Passes the test to the generic print function which pints the test on the LCD

\param               Text                  the text to be printed
*/
//------------------------------------------------------------------------------
void SysComp_LcdSetText(char* Text)
{
    LCD_Show_Text(Text);
}

//------------------------------------------------------------------------------
/**
\brief               Change the line to write to

Enables a switch from one line to the other

\param               LineNum_p            Line number (1 = line one, 2 = line two)
*/
//------------------------------------------------------------------------------
tEplKernel SysComp_LcdSetLine(int LineNum_p)
{
    tEplKernel retval = kEplSuccessful;

    switch(LineNum_p)
    {
        case 1:
            LCD_Line1();
        break;
        case 2:
            LCD_Line2();
        break;
        default:
            retval = kEplApiInvalidParam;
    }

    return retval;
}

//------------------------------------------------------------------------------
/**
\brief               Test the LCD display

Init the LCD display and write a test message to both lines of the display
*/
//------------------------------------------------------------------------------
void SysComp_LcdTest(void)
{
  char Text1[16] = "POWERLINK SLAVE";   ///< text for line 1
  char Text2[16] = "-- by B & R -- ";   ///< text for line 2
  //  Initial LCD
  LCD_Init();
  //  Show Text to LCD
  LCD_Show_Text(Text1);
  //  Change Line2
  LCD_Line2();
  //  Show Text to LCD
  LCD_Show_Text(Text2);
}

//------------------------------------------------------------------------------
/**
\brief               Write NMT state to LCD display

Write the current NMT state to line two of the LCD display

\param               NmtState_p                  IN: current state machine value
*/
//------------------------------------------------------------------------------
void SysComp_LcdPrintState(tEplNmtState NmtState_p)
{
    LCD_Line2();
    switch (NmtState_p)
    {
        case kEplNmtGsOff               : LCD_Show_Text(aStrNmtState_l[1]); break;
        case kEplNmtGsInitialising      : LCD_Show_Text(aStrNmtState_l[2]); break;
        case kEplNmtGsResetApplication  : LCD_Show_Text(aStrNmtState_l[2]); break;
        case kEplNmtGsResetCommunication: LCD_Show_Text(aStrNmtState_l[2]); break;
        case kEplNmtGsResetConfiguration: LCD_Show_Text(aStrNmtState_l[2]); break;
        case kEplNmtCsNotActive         : LCD_Show_Text(aStrNmtState_l[3]); break;
        case kEplNmtCsPreOperational1   : LCD_Show_Text(aStrNmtState_l[5]); break;
        case kEplNmtCsStopped           : LCD_Show_Text(aStrNmtState_l[9]); break;
        case kEplNmtCsPreOperational2   : LCD_Show_Text(aStrNmtState_l[6]); break;
        case kEplNmtCsReadyToOperate    : LCD_Show_Text(aStrNmtState_l[7]); break;
        case kEplNmtCsOperational       : LCD_Show_Text(aStrNmtState_l[8]); break;
        case kEplNmtCsBasicEthernet     : LCD_Show_Text(aStrNmtState_l[4]); break;
        default:
        LCD_Show_Text(aStrNmtState_l[0]);
        break;
    }

}

//------------------------------------------------------------------------------
/**
\brief               Print node info on LCD

Writes the node id to line one of the LCD display.

\param               nodeID                           the node ID which was read
*/
//------------------------------------------------------------------------------
void SysComp_LcdPrintNodeInfo (WORD wNodeId_p)
{
    char TextNodeID[17];

    sprintf(TextNodeID, "Node/ID:0x%02X", wNodeId_p);

    LCD_Clear();
    LCD_Show_Text(TextNodeID);

}

#endif // LCD_BASE
