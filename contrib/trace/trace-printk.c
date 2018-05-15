/**
********************************************************************************
\file   trace-printk.c

\brief  Trace function using Linux kernel printk

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <linux/kernel.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36))
#include <linux/printk.h>
#endif

#include <trace/trace.h>

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

#ifndef NDEBUG
//------------------------------------------------------------------------------
/**
\brief  Print debug trace message

The function prints a debug trace message using standard printk

\param[in]      fmt                 Format string
\param[in]      ...                 Arguments to print
*/
//------------------------------------------------------------------------------
void trace(const char* fmt, ...)
{
    va_list argptr;

    va_start(argptr, fmt);
    vprintk(fmt, argptr);
    va_end(argptr);
}
#endif