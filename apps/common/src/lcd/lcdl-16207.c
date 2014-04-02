/**
********************************************************************************
\file       lcd-16207.c

\brief      LCD functions for Altera Avalon LCD IP-Core with HD44780

This implementation uses the Altera Avalon LCD 16207 IP-Core to handle the
display controller HD44780 - available e.g. on the Terasic DE2-115 board.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <unistd.h> // for usleep()
#include <string.h>

#include <system.h>
#include <altera_avalon_lcd_16207_regs.h>

#include "lcdl.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef LCD_BASE
#error "Rename the LCD component in QSYS/SOPC to 'lcd'!"
#endif

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
#define LCDL_COLUMN         16  ///< Column count
#define LCDL_LINE           2   ///< Line count

#define LCDL_WRCMD(data)    IOWR_ALTERA_AVALON_LCD_16207_COMMAND(LCD_BASE, data)
#define LCDL_RDCMD()        IORD_ALTERA_AVALON_LCD_16207_STATUS(LCD_BASE)
#define LCDL_WRDATA(data)   IOWR_ALTERA_AVALON_LCD_16207_DATA(LCD_BASE, data)
#define LCDL_RDDATA()       IORD_ALTERA_AVALON_LCD_16207_DATA(LCD_BASE)

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

This function writes a sequence of initialization parameters to the LCD.
*/
//------------------------------------------------------------------------------
int lcdl_init(void)
{
    LCDL_WRCMD(0x38);
    usleep(2000);
    LCDL_WRCMD(0x0C);
    usleep(2000);
    LCDL_WRCMD(0x01);
    usleep(2000);
    LCDL_WRCMD(0x06);
    usleep(2000);
    LCDL_WRCMD(0x80);
    usleep(2000);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Exit the LCD instance

This function exits the LCD instance.
*/
//------------------------------------------------------------------------------
void lcdl_exit(void)
{

}

//------------------------------------------------------------------------------
/**
\brief  Clear the LCD

This function clears all lines of the display.
*/
//------------------------------------------------------------------------------
void lcdl_clear(void)
{
    LCDL_WRCMD(0x01);
    usleep(2000);
}

//------------------------------------------------------------------------------
/**
\brief  Change to specified line

Changes to specified line of the LCD

\param  line_p      Specifies the line

\return The function returns 0 if the line is changed successfully, -1 otherwise.
*/
//------------------------------------------------------------------------------
int lcdl_changeToLine(unsigned int line_p)
{
    if (line_p > LCDL_LINE)
        return -1;

    switch (line_p)
    {
        case 1:
            LCDL_WRCMD(0x80);
            break;

        case 2:
            LCDL_WRCMD(0xC0);
            break;

        default:
            break;
    }

    usleep(2000);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Print text to the LCD

Writes text to the LCD currently selected.

\param  sText_p     The text to print
*/
//------------------------------------------------------------------------------
void lcdl_printText(const char* sText_p)
{
    int i;
    int length = strlen(sText_p);

    // Longer text is cut due to column limitation!
    for (i = 0; i < LCDL_COLUMN; i++)
    {
        // Write blank if provided text is shorter than column count.
        if (i < length)
            LCDL_WRDATA(sText_p[i]);
        else
            LCDL_WRDATA(' ');

        usleep(2000);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

