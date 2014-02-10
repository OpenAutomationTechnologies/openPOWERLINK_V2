/**
********************************************************************************
\file   event.c

\brief  Source file for general functions used by event handler modules

This file contains general functions used by the user and kernel event
handler modules.

\ingroup module_event
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/event.h>
#include "event.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Finds the appropriate event handler for a specific sink

The function searches the event dispatch table for the appropriate event
handler for the specified sink. It returns the pointer to the event handler
and the corresponding event source.

\param  ppDispatchEntry_p   Pointer to dispatch entry pointer.
\param  sink_p              Event sink to search for.
\param  ppfnEventHandler_p  Pointer to store event handler function pointer.
\param  pEventSource_p      Pointer to store the corresponding event source.

\return The function returns a tOplkError error code.
\retval kErrorOk          If the event sink was found.
\retval kErrorEventUnknownSink    IF the event sink was not found.

\ingroup module_event
*/
//------------------------------------------------------------------------------
tOplkError event_getHandlerForSink(tEventDispatchEntry** ppDispatchEntry_p,
                                   tEventSink sink_p,
                                   tProcessEventCb* ppfnEventHandler_p,
                                   tEventSource* pEventSource_p)
{
    tOplkError              ret = kErrorEventUnknownSink;

    while ((*ppDispatchEntry_p)->sink != kEventSinkInvalid)
    {
        if (sink_p == (*ppDispatchEntry_p)->sink)
        {
            *pEventSource_p = (*ppDispatchEntry_p)->source;
            *ppfnEventHandler_p = (*ppDispatchEntry_p)->pfnEventHandler;
            (*ppDispatchEntry_p)++;
            ret = kErrorOk;
            break;
        }
        (*ppDispatchEntry_p)++;
    }


    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

