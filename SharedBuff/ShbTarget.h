/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  target specific functions for use without any OS
                (for ShbIpc-NoOS)

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
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2005/12/05 -as:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _SHBTARGET_H_
#define _SHBTARGET_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
// =========================================================================
// macros for memory access (depends on target system)
// =========================================================================

// NOTE:
// The following macros are used to combine standard library definitions. Some
// applications needs to use one common library function (e.g. memcpy()). So
// you can set (or change) it here.

#if (TARGET_SYSTEM == _NO_OS_)

    #include <stdlib.h>
    #include <stdio.h>

    //29.11.2004 f.j. sonst ist memcpy und memset unbekannt
    #include <string.h>

//    #define EPL_MEMCPY(dst,src,siz)     memcpy((void*)(dst),(const void*)(src),(size_t)(siz));
//    #define EPL_MEMSET(dst,val,siz)     memset((void*)(dst),(int)(val),(size_t)(siz));

//    #define EPL_MALLOC(siz)             malloc((size_t)(siz))
//    #define EPL_FREE(ptr)               free((void *)ptr)

    #ifndef PRINTF0
        #define PRINTF                      TRACE
        #define PRINTF0(arg)                TRACE0(arg)
        #define PRINTF1(arg,p1)             TRACE1(arg,p1)
        #define PRINTF2(arg,p1,p2)          TRACE2(arg,p1,p2)
        #define PRINTF3(arg,p1,p2,p3)       TRACE3(arg,p1,p2,p3)
        #define PRINTF4(arg,p1,p2,p3,p4)    TRACE4(arg,p1,p2,p3,p4)
        //#define PRINTF                      printf
        //#define PRINTF0(arg)                PRINTF(arg)
        //#define PRINTF1(arg,p1)             PRINTF(arg,p1)
        //#define PRINTF2(arg,p1,p2)          PRINTF(arg,p1,p2)
        //#define PRINTF3(arg,p1,p2,p3)       PRINTF(arg,p1,p2,p3)
        //#define PRINTF4(arg,p1,p2,p3,p4)    PRINTF(arg,p1,p2,p3,p4)
    #endif
#endif


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

void  PUBLIC  ShbTgtTimerInit(void);
DWORD PUBLIC  ShbTgtGetTickCountMs(void);

void  PUBLIC  ShbTgtEnableGlobalInterrupt(BYTE fEnable_p);
BYTE  PUBLIC  ShbTgtIsInterruptContext(void);

#endif  // #ifndef _SHBTARGET_H_


