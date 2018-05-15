/**
********************************************************************************
\file   common/timer.h

\brief  Generic definitions for timer modules

This file contains some generic definitions for timer modules.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_common_timer_H_
#define _INC_common_timer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

// type for timer handle
#if (TARGET_SYSTEM == _WIN32_)
typedef ULONG_PTR tTimerHdl;
#else /* (TARGET_SYSTEM == _WIN32_) */
typedef ULONG tTimerHdl;
#endif /* (TARGET_SYSTEM == _WIN32_) */

/**
\brief  Structure for timer event arguments

The structure defines a timer event argument. It provides information about
the timer to the sink the event is sent to.
*/
typedef struct
{
    // Use a union of tTimerHdl and 64 bit variable to avoid
    // structure size mismatch while sharing the tTimerEventArg
    // between heterogeneous processors.
    union
    {
        tTimerHdl           handle;     ///< Native handle of the expired timer
        UINT64              padding;    ///< 64 Bit placeholder
    } timerHdl;

    union
    {
        UINT32          value;          ///< Timer argument supplied as UINT32
        void*           pValue;         ///< Timer argument supplied as void*
    } argument;                         ///< The timer argument the timer was initialized with.
} tTimerEventArg;

#endif /* _INC_common_timer_H_ */
