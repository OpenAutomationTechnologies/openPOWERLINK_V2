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
    #define _WIN32_WINNT   0x0501

    #include <stdlib.h>
    #include <stdio.h>

    #include <string.h>
    #include <Windows.h>

//    #define EPL_MEMCPY(dst,src,siz)     memcpy((void*)(dst),(const void*)(src),(size_t)(siz));
//    #define EPL_MEMSET(dst,val,siz)     memset((void*)(dst),(int)(val),(size_t)(siz));

//    #define EPL_MALLOC(siz)             malloc((size_t)(siz))
//    #define EPL_FREE(ptr)               free((void *)ptr)

    void trace (const char* fmt, ...);
    #ifdef _CONSOLE // use standard printf in console applications
        #define PRINTF(...)                      printf(__VA_ARGS__)
    #else           // use trace for output in debug window in Windows applications
        #define PRINTF(...)                      TRACE(__VA_ARGS__)
    #endif

    #ifdef ASSERTMSG
        #undef ASSERTMSG
    #endif

    #define ASSERTMSG(expr,string)  if (!(expr)) { \
                                        MessageBox (NULL, string, "Assertion failed", MB_OK | MB_ICONERROR); \
                                        exit (-1);}

    #define OPLK_ATOMIC_T    ULONG
    #define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                oldval = InterlockedExchange(address, newval);

#elif (TARGET_SYSTEM == _WINCE_)

    void trace (const char* fmt, ...);
    #define PRINTF(...)                 TRACE(__VA_ARGS__)

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

    #ifndef NDEBUG
        #define PRINTF(...)                 printf(__VA_ARGS__)
    #else // !NDEBUG
        #define PRINTF(...)
    #endif // !NDEBUG

    #if (DEV_SYSTEM == _DEV_NIOS2_)
        /* NOTE:
         * Nios II does not support atomic instructions, hence, pseudo atomic
         * macro is applied with locking.
         */
        #include <alt_types.h>
        #include <io.h>
        #include <lock.h>

        #define OPLK_ATOMIC_T    alt_u8
        #define OPLK_LOCK_T      LOCK_T
        #define OPLK_ATOMIC_INIT(base) \
                                if(target_initLock(&base->lock) != 0) \
                                    return kEplNoResource
        #define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                                target_lock(); \
                                oldval = IORD(address, 0); \
                                IOWR(address, 0, newval); \
                                target_unlock()
    #endif

    #if (DEV_SYSTEM == _DEV_MICROBLAZE_BIG_ \
        || DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_)
        /* NOTE:
         * Pseudo atomic macro is applied with locking.
         */
        #include <xil_types.h>
        #include <xil_io.h>
        #include <lock.h>

        #define OPLK_ATOMIC_T    u8
        #define OPLK_LOCK_T      LOCK_T
        #define OPLK_ATOMIC_INIT(base) \
                                if(target_initLock(&base->lock) != 0) \
                                    return kEplNoResource
        #define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                                target_lock(); \
                                oldval = Xil_In8(address); \
                                Xil_Out8(address, newval); \
                                target_unlock()
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

    #ifdef __KERNEL__
        #define PRINTF(...)                 printk(__VA_ARGS__)
    #else
        #define PRINTF(...)                 printf(__VA_ARGS__)
    #endif

    #define OPLK_ATOMIC_T    UINT8
    #define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
        oldval = __sync_lock_test_and_set(address, newval);

#elif (TARGET_SYSTEM == _VXWORKS_)
    #include <stdlib.h>
    #include <stdio.h>
    #include <string.h>

    #define PRINTF(...)                      printf(__VA_ARGS__)

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

#ifndef OPLK_ATOMIC_INIT
    #define OPLK_ATOMIC_INIT(ignore)    ((void)0)
#endif

#ifndef TIME_STAMP_T
    #define TIME_STAMP_T                UINT32
#endif

#define EPL_TGT_INTMASK_ETH     0x0001  // ethernet interrupt
#define EPL_TGT_INTMASK_DMA     0x0002  // DMA interrupt

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

typedef struct {
    TIME_STAMP_T timeStamp;
} tEplTgtTimeStamp;

//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// currently no Timer functions are needed by EPL stack
// so they are not implemented yet
//void  PUBLIC EplTgtTimerInit(void);
//void PUBLIC TgtGetNetTime(tEplNetTime * pNetTime_p);

DWORD PUBLIC EplTgtGetTickCountMs(void);


unsigned long long PUBLIC EplTgtGetTimeStampNs(void);

// functions for ethernet driver
tEplKernel PUBLIC TgtInitEthIsr(void);
void PUBLIC TgtFreeEthIsr(void);
void PUBLIC TgtEnableEthInterrupt0(BYTE fEnable_p, unsigned int uiInterruptMask_p);
void PUBLIC TgtEnableEthInterrupt1(BYTE fEnable_p, unsigned int uiInterruptMask_p);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // #ifndef _EPLTARGET_H_


