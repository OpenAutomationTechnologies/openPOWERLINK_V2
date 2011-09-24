/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for target api function

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

#ifndef _EPLTARGET_H_
#define _EPLTARGET_H_


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

#if (TARGET_SYSTEM == _WIN32_)

    #define _WIN32_WINDOWS 0x0401
    #define _WIN32_WINNT   0x0400

    #include <stdlib.h>
    #include <stdio.h>

    #include <string.h>

//    #define EPL_MEMCPY(dst,src,siz)     memcpy((void*)(dst),(const void*)(src),(size_t)(siz));
//    #define EPL_MEMSET(dst,val,siz)     memset((void*)(dst),(int)(val),(size_t)(siz));

//    #define EPL_MALLOC(siz)             malloc((size_t)(siz))
//    #define EPL_FREE(ptr)               free((void *)ptr)

    #ifndef PRINTF0
        void trace (const char* fmt, ...);
        #ifdef _CONSOLE // use standard printf in console applications
            #define PRINTF                      printf
        #else           // use trace for output in debug window in Windows applications
            #define PRINTF                      TRACE
        #endif
        #define PRINTF0(arg)                PRINTF(arg)
        #define PRINTF1(arg,p1)             PRINTF(arg,p1)
        #define PRINTF2(arg,p1,p2)          PRINTF(arg,p1,p2)
        #define PRINTF3(arg,p1,p2,p3)       PRINTF(arg,p1,p2,p3)
        #define PRINTF4(arg,p1,p2,p3,p4)    PRINTF(arg,p1,p2,p3,p4)
    #endif

    #ifdef ASSERTMSG
        #undef ASSERTMSG
    #endif

    #define ASSERTMSG(expr,string)  if (!(expr)) { \
                                        MessageBox (NULL, string, "Assertion failed", MB_OK | MB_ICONERROR); \
                                        exit (-1);}

#elif (TARGET_SYSTEM == _WINCE_)

    #ifndef PRINTF0
        void trace (const char* fmt, ...);
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

    #ifdef ASSERTMSG
        #undef ASSERTMSG
    #endif

    #define ASSERTMSG(expr,string)  if (!(expr)) { \
                                        MessageBox (NULL, string, L"Assertion failed", MB_OK | MB_ICONERROR); \
                                        exit (-1);}



#elif (TARGET_SYSTEM == _NO_OS_)

    #include <stdlib.h>
    #include <stdio.h>
    #include <string.h>

    #ifndef PRINTF0
        #define PRINTF                      printf
        #define PRINTF0(arg)                PRINTF(arg)
        #define PRINTF1(arg,p1)             PRINTF(arg,p1)
        #define PRINTF2(arg,p1,p2)          PRINTF(arg,p1,p2)
        #define PRINTF3(arg,p1,p2,p3)       PRINTF(arg,p1,p2,p3)
        #define PRINTF4(arg,p1,p2,p3,p4)    PRINTF(arg,p1,p2,p3,p4)
        //#define PRINTF                      TRACE
        //#define PRINTF0(arg)                TRACE0(arg)
        //#define PRINTF1(arg,p1)             TRACE1(arg,p1)
        //#define PRINTF2(arg,p1,p2)          TRACE2(arg,p1,p2)
        //#define PRINTF3(arg,p1,p2,p3)       TRACE3(arg,p1,p2,p3)
        //#define PRINTF4(arg,p1,p2,p3,p4)    TRACE4(arg,p1,p2,p3,p4)
    #endif

    #if EPL_USE_SHAREDBUFF != FALSE
        #define EplTgtGetTickCountMs        ShbTgtGetTickCountMs
        #define EplTgtEnableGlobalInterrupt ShbTgtEnableGlobalInterrupt
    #endif


#elif (TARGET_SYSTEM == _LINUX_)

    #ifndef __KERNEL__
        #include <stdlib.h>
        #include <stdio.h>
        #include <string.h>
    #else
//        #include <linux/config.h>
        #include <linux/module.h>
        #include <linux/kernel.h>
        #include <linux/init.h>
        #include <linux/errno.h>
        #include <linux/major.h>
        #include <linux/version.h>
        #include <linux/slab.h>
    #endif

//    #define EPL_MEMCPY(dst,src,siz)     memcpy((void*)(dst),(const void*)(src),(size_t)(siz));
//    #define EPL_MEMSET(dst,val,siz)     memset((void*)(dst),(int)(val),(size_t)(siz));

    #ifdef __KERNEL__
        #define EPL_MALLOC(siz)             kmalloc(siz, GFP_KERNEL)
        #define EPL_FREE(ptr)               kfree(ptr)
    #endif

    #ifndef PRINTF0
        #ifdef __KERNEL__
            #define PRINTF                      printk
        #else
            #define PRINTF                      printf
        #endif
        #define PRINTF0(arg)                PRINTF(arg)
        #define PRINTF1(arg,p1)             PRINTF(arg,p1)
        #define PRINTF2(arg,p1,p2)          PRINTF(arg,p1,p2)
        #define PRINTF3(arg,p1,p2,p3)       PRINTF(arg,p1,p2,p3)
        #define PRINTF4(arg,p1,p2,p3,p4)    PRINTF(arg,p1,p2,p3,p4)
    #endif

#elif (TARGET_SYSTEM == _VXWORKS_)
    #include <stdlib.h>
    #include <stdio.h>
    #include <string.h>

    #ifndef PRINTF0
        #define PRINTF                      printf
        #define PRINTF0(arg)                PRINTF(arg)
        #define PRINTF1(arg,p1)             PRINTF(arg,p1)
        #define PRINTF2(arg,p1,p2)          PRINTF(arg,p1,p2)
        #define PRINTF3(arg,p1,p2,p3)       PRINTF(arg,p1,p2,p3)
        #define PRINTF4(arg,p1,p2,p3,p4)    PRINTF(arg,p1,p2,p3,p4)
    #endif

#endif


#ifndef EPL_MEMCPY
    #define EPL_MEMCPY(dst,src,siz)     memcpy((dst),(src),(siz))
#endif
#ifndef EPL_MEMSET
    #define EPL_MEMSET(dst,val,siz)     memset((dst),(val),(siz))
#endif
#ifndef EPL_MEMCMP
    #define EPL_MEMCMP(src1,src2,siz)   memcmp((src1),(src2),(siz))
#endif
#ifndef EPL_MALLOC
    #define EPL_MALLOC(siz)             malloc(siz)
#endif
#ifndef EPL_FREE
    #define EPL_FREE(ptr)               free(ptr)
#endif


#define EPL_TGT_INTMASK_ETH     0x0001  // ethernet interrupt
#define EPL_TGT_INTMASK_DMA     0x0002  // DMA interrupt

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

// currently no Timer functions are needed by EPL stack
// so they are not implemented yet
//void  PUBLIC EplTgtTimerInit(void);
//void PUBLIC TgtGetNetTime(tEplNetTime * pNetTime_p);

DWORD PUBLIC EplTgtGetTickCountMs(void);
void PUBLIC EplTgtEnableGlobalInterrupt(BYTE fEnable_p);

unsigned long long PUBLIC EplTgtGetTimeStampNs(void);

// functions for ethernet driver
tEplKernel PUBLIC TgtInitEthIsr(void);
void PUBLIC TgtFreeEthIsr(void);
void PUBLIC TgtEnableEthInterrupt0(BYTE fEnable_p, unsigned int uiInterruptMask_p);
void PUBLIC TgtEnableEthInterrupt1(BYTE fEnable_p, unsigned int uiInterruptMask_p);

// functions for target specific time stamps
DWORD PUBLIC EplTgtTimeStampTimeDiffNs (tEplTgtTimeStamp* pTimeStampPredecessor_p,
                                        tEplTgtTimeStamp* pTimeStampSuccessor_p);
void  PUBLIC EplTgtTimeStampCopy       (tEplTgtTimeStamp* pTimeStampDest_p,
                                        tEplTgtTimeStamp* pTimeStampSrc_p);
void  PUBLIC EplTgtTimeStampFree       (tEplTgtTimeStamp* pTimeStamp_p);
tEplTgtTimeStamp* PUBLIC EplTgtTimeStampAlloc (void);

tEplKernel PUBLIC EplTgtInit(void);
tEplKernel PUBLIC EplTgtCleanup(void);

#endif  // #ifndef _EPLTARGET_H_


