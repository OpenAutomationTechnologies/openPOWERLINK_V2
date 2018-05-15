/**
********************************************************************************
\file   console-linux.c

\brief  Console input/output implementation for Linux

This file contains the console input/output implementation for Linux.

\ingroup module_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get character from console

This function reads a character from the console input. It uses the
termios library.

\return The function returns the read character.

\ingroup module_console
*/
//------------------------------------------------------------------------------
int console_getch(void)
{
    struct termios  oldt;
    struct termios  newt;
    int             ch;

    // Save old attributes
    tcgetattr(STDIN_FILENO, &oldt);

    // Calculate and set new attributes
    // ECHO ..... echo input key on/off
    // ICANON ... line buffering on/off
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read input key
    ch = getchar();

    // Restore old attributes
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

//------------------------------------------------------------------------------
/**
\brief  Detecting a keystroke

The function checks the console for a keystroke.

\return The function returns 0 if no key has been pressed or 1 if a key has
        been pressed.

\ingroup module_console
*/
//------------------------------------------------------------------------------
int console_kbhit(void)
{
    struct termios  oldt;
    struct termios  newt;
    int             oldf;
    int             newf;
    int             ch;

    // Save current attributes and file status
    tcgetattr(STDIN_FILENO, &oldt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

    // Calculate new attributes and file status
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newf = oldf | O_NONBLOCK;

    // Set new attributes and file status
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, newf);

    // Read input
    ch = getchar();

    // Restore save attributes and file status
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    // Put read character back where it was taken
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
