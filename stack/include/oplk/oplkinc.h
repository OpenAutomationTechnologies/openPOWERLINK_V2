/**
********************************************************************************
\file   oplk/oplkinc.h

\brief  Standard include file

This is the standard include file that must be included by every openPOWERLINK
header file. It includes all necessary files for setting up the basic types
and definitions.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_oplk_oplkinc_H_
#define _INC_oplk_oplkinc_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/targetsystem.h>
#include <oplk/oplkdefs.h>

#include <oplkcfg.h>                 // Stack configuration file
#include <oplk/defaultcfg.h>

#include <oplk/errordefs.h>
#include <oplk/featureflags.h>
#include <oplk/version.h>
#include <oplk/debug.h>
#include <oplk/ftracedebug.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  Set default definitions if not already set in target specific section

#ifndef OPLK_MEMCPY
#define OPLK_MEMCPY(dst, src, siz)    memcpy((dst), (src), (siz))
#endif

#ifndef OPLK_MEMSET
#define OPLK_MEMSET(dst, val, siz)    memset((dst), (val), (siz))
#endif

#ifndef OPLK_MEMCMP
#define OPLK_MEMCMP(src1, src2, siz)  memcmp((src1), (src2), (siz))
#endif
#ifndef OPLK_MALLOC
#define OPLK_MALLOC(siz)              malloc(siz)
#endif

#ifndef OPLK_FREE
#define OPLK_FREE(ptr)                free(ptr)
#endif

#ifndef OPLK_ATOMIC_INIT
#define OPLK_ATOMIC_INIT(ignore)      ((void)0)
#endif

#ifndef TIME_STAMP_T
#define TIME_STAMP_T                  UINT32
#endif

//------------------------------------------------------------------------------
//  definition of TRACE

#ifndef NDEBUG
#define TRACE(...) trace(__VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

void trace(const char* fmt, ...);

#ifdef __cplusplus
}
#endif
#else

#define TRACE(...)

#endif

//------------------------------------------------------------------------------
//  definition of ASSERT
#ifndef ASSERT

#if !defined (__linux__) && !defined (__KERNEL__)
#include <assert.h>
#define ASSERT(p)    assert(p)
#else
#define ASSERT(p)
#endif

#endif

//------------------------------------------------------------------------------
// This macro doesn't print out C-file and line number of the failed assertion
// but a string, which exactly names the mistake.
#if !defined(ASSERTMSG) && !defined(NDEBUG)

#define ASSERTMSG(expr, string) \
    if (!(expr)) \
    { \
        PRINTF("Assertion failed: " string);\
        for ( ; ; );\
    }
#else

#define ASSERTMSG(expr, string)

#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief  IEEE 1588 conforming net time structure

The structure defines a IEEE 1588 conforming net time.
*/
typedef struct
{
    UINT32              sec;        ///< Seconds
    UINT32              nsec;       ///< Nanoseconds
} tNetTime;

/**
\brief Hardware parameter structure

The following structure specifies the hardware parameters of an openPOWERLINK
Ethernet controller.
*/
typedef struct
{
    UINT                devNum;     ///< Device number of the used Ethernet controller
    const char*         pDevName;   ///< Device name of the Ethernet controller (valid if non-null)
} tHwParam;

/**
\brief Timestamp structure

The following structure defines a timestamp value use to store target specific
timestamps.
*/
typedef struct
{
    TIME_STAMP_T        timeStamp;      ///< The timestamp.
} tTimestamp;


/**
\brief Time of day structure

The following structure defines a CANopen time-of-day format.
*/
#ifndef _TIME_OF_DAY_DEFINED_
typedef struct
{
    ULONG               msec;           ///< Milliseconds after midnight
    USHORT              days;           ///< Days since January the 1st, 1984
} tTimeOfDay;

#define _TIME_OF_DAY_DEFINED_
#endif

//------------------------------------------------------------------------------
// global macros
//------------------------------------------------------------------------------
#ifndef tabentries
#define tabentries(aVar_p)  (sizeof(aVar_p) / sizeof(*(aVar_p)))
#endif

#ifndef min
#define min(a, b)           (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b)           (((a) > (b)) ? (a) : (b))
#endif

/* macro for adding two timespec values */
#define TIMESPECADD(vvp, uvp)                                           \
        {                                                               \
                (vvp)->tv_sec += (uvp)->tv_sec;                         \
                (vvp)->tv_nsec += (uvp)->tv_nsec;                       \
                if ((vvp)->tv_nsec >= 1000000000)                       \
                {                                                       \
                        (vvp)->tv_sec++;                                \
                        (vvp)->tv_nsec -= 1000000000;                   \
                }                                                       \
        }

#endif /* _INC_oplk_oplkinc_H_ */

