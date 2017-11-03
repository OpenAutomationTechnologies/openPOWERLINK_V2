/**
********************************************************************************
\file   trace-windows.c

\brief  Trace function for the Windows operating system

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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

// Windows version must be at least Windows XP
#if (defined(_WIN32_WINNT) && (_WIN32_WINNT < 0x0501))
#undef _WIN32_WINNT
#endif
#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x0501
#endif

// Do not use extended Win32 API functions
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif

#if defined(_KERNEL_MODE)
#include <Wdm.h>
#else
#include <Windows.h>
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Print debug trace message

The function prints a debug trace message.

\param[in]      fmt                 Format string
\param[in]      ...                 Arguments to print
*/
//------------------------------------------------------------------------------
void trace(const char* fmt, ...)
{
    char    aBuffer[0x0400];
    va_list argptr;

    va_start(argptr, fmt);
#if (_MSC_VER >= 1400)
    vsprintf_s(aBuffer, sizeof(aBuffer), fmt, argptr);
#else
    vsprintf(aBuffer, fmt, argptr);
#endif
    va_end(argptr);

#if defined(_KERNEL_MODE)
    DbgPrint((LPSTR)&aBuffer);
#else
    OutputDebugString((LPSTR)&aBuffer);
#endif
}
