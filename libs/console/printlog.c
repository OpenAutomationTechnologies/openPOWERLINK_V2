/**
********************************************************************************
\file   printlog.c

\brief  Log printing functions for console applications

\ingroup module_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronik GmbH
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
#include <stdarg.h>
#include <time.h>

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Print log entry

The function prints a log entry on the console. It prepends the output
with the current date and time.

\param  fmt         Format string
\param  ...         Arguments to print

\ingroup module_console
*/
//------------------------------------------------------------------------------
void console_printlog(char *fmt, ...)
{
    va_list             arglist;
    time_t              timeStamp;
    struct tm           *p_timeVal;
    char                timeStr[20];

    time(&timeStamp);
    p_timeVal = localtime(&timeStamp);
    strftime(timeStr, 20, "%Y/%m/%d %H:%M:%S", p_timeVal);

    fprintf (stderr, "%s - ", timeStr);
    va_start(arglist, fmt);
    vfprintf(stderr, fmt, arglist);
    va_end(arglist);
}

//------------------------------------------------------------------------------
/**
\brief  Print log entry

The function adds a string to a log entry on the console (no timestamp).

\param  fmt         Format string
\param  ...         Arguments to print

\ingroup module_console
*/
//------------------------------------------------------------------------------
void console_printlogadd(char *fmt, ...)
{
    va_list             arglist;

    va_start(arglist, fmt);
    vfprintf(stderr, fmt, arglist);
    va_end(arglist);
}

