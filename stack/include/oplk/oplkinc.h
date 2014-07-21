/**
********************************************************************************
\file   oplk/oplkinc.h

\brief  Standard include file for public headers.

This is the standard include file for all public openPOWERLINK header files.
It includes all necessary files for setting up the basic types and definitions.
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
#include <oplk/errordefs.h>
#include <oplk/version.h>
#include <oplk/debug.h>
#include <oplk/ftracedebug.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


#ifndef TARGET_FLUSH_DCACHE
#define TARGET_FLUSH_DCACHE(base, range)        ((void)0)
#endif

#ifndef TARGET_INVALIDATE_DCACHE
#define TARGET_INVALIDATE_DCACHE(base, range)   ((void)0)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief  IEEE 1588 conforming net time structure

The structure defines an IEEE 1588 conforming net time.
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

#endif /* _INC_oplk_oplkinc_H_ */
