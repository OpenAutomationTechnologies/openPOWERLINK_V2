/*******************************************************************************

  File:         ftraceDebug.h

  (c) Bernecker + Rainer Industrie Elektronik Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Author:       Josef Baumgartner

  Description:  Header for Linux ftrace debug functions

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of copyright holders nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/

#ifndef FTRACE_H_
#define FTRACE_H_

//---------------------------------------------------------------------------
// includes
//---------------------------------------------------------------------------
#if defined(FTRACE_DEBUG)
#include <stdarg.h>
#include <fcntl.h>
#endif

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#if defined(FTRACE_DEBUG)
#define FTRACE_OPEN()                   FtraceOpen()
#define FTRACE_CLOSE()                  FtraceClose()
#define FTRACE_ENABLE(X)                FtraceEnable(X)
#define FTRACE_MARKER(...)              FtraceWriteTraceMarker(__VA_ARGS__)
#else
#define FTRACE_OPEN()
#define FTRACE_CLOSE()
#define FTRACE_ENABLE(X)
#define FTRACE_MARKER(...)
#endif

#ifdef  FTRACE_DEBUG
//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------
extern int FtraceOpen(void);
extern int FtraceClose(void);
extern void FtraceEnable(int fEnable_p);
void FtraceWriteTraceMarker(char *fmt, ...);

#endif /* INCLUDE_FTRACE_DEBUG */

#endif /* FTRACE_H_ */

