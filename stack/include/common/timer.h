/**
********************************************************************************
\file   common/timer.h

\brief  Generic definitions for timer modules

This file contains some generic definitions for timer modules.
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

#ifndef _INC_common_timer_H_
#define _INC_common_timer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

// type for timer handle
#if (TARGET_SYSTEM == _WIN32_)
typedef ULONG_PTR tTimerHdl;
#else
typedef ULONG tTimerHdl;
#endif

/**
\brief  Structure for timer arguments

The structure defines the arguments for a timer. It is used to setup a timer.
*/
typedef struct
{
    tEventSink          eventSink;      ///< The sink to send the event when the timer expires
    union
    {
        UINT32          value;          ///< Timer argument supplied as UINT32
        void*           pValue;         ///< Timer argument supplied as void*
    } argument;                         ///< The timer argument to be sent in the timer event
} tTimerArg;


/**
\brief  Structure for timer event arguments

The structure defines a timer event argument. It provides information about
the timer to the sink the event is sent to.
*/
typedef struct
{
    tTimerHdl           timerHdl;       ///< Delivers the handle of the expired timer
    union
    {
        UINT32          value;          ///< Timer argument supplied as UINT32
        void*           pValue;         ///< Timer argument supplied as void*
    } argument;                         ///< The timer argument the timer was initialized with.
} tTimerEventArg;

/**
\brief Type for timer callback function pointers

This type defines a function pointer to a timer callback function.

\param pEventArg_p       Pointer to timer event argument

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tTimerkCallback)(tTimerEventArg* pEventArg_p);

#endif /* _INC_common_timer_H_ */
