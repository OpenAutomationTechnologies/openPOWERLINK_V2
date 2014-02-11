/**
********************************************************************************
\file   eventu.c

\brief  Source file for user event module

This file contains the source code of the user event module. It provides the
interface for posting and receiving events to/from other user modules.

\ingroup module_eventu
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
#include <user/eventu.h>
#include <user/eventucal.h>
#include <user/nmtu.h>
#include <user/nmtmnu.h>
#include <user/sdoseq.h>
#include <user/dllucal.h>
#include <user/ledu.h>
#include <oplk/benchmark.h>
#include <oplk/oplk.h>

#include "common/event/event.h"


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

/**
\brief Event user instance type

The user event instance holds the Api process callback function pointer.
*/
typedef struct
{
    tProcessEventCb         pfnApiProcessEventCb;  ///< Callback for generic api events
} tEventuInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError callApiEventCb (tEvent* pEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuInstance              instance_l;

/**
\brief  Event dispatch table

The following table defines the event handlers to be used for the specific
event sinks.
*/
static tEventDispatchEntry eventDispatchTbl_l[] =
{
    { kEventSinkNmtu,        kEventSourceNmtu,        nmtu_processEvent },
#if defined (CONFIG_INCLUDE_NMT_MN)
    { kEventSinkNmtMnu,      kEventSourceNmtMnu,      nmtmnu_processEvent },
#endif
#if defined (CONFIG_INCLUDE_SDOC) || defined(CONFIG_INCLUDE_SDOS)
    { kEventSinkSdoAsySeq,   kEventSourceSdoAsySeq,   sdoseq_processEvent },
#endif
#if defined (CONFIG_INCLUDE_LEDU)
    { kEventSinkLedu,        kEventSourceLedu,        ledu_processEvent },
#else
    { kEventSinkLedu,        kEventSourceLedu,        NULL },
#endif
    { kEventSinkDlluCal,     kEventSourceDllu,        dllucal_process },
    { kEventSinkErru,        kEventSourceErru,        NULL },
    { kEventSinkApi,         kEventSourceEplApi,      callApiEventCb },
    { kEventSinkInvalid,     kEventSourceInvalid,     NULL }
};


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event module

The function initializes the user event module. It is also responsible to call
the init function of it's CAL module.

\param  pfnApiProcessEventCb_p  Function pointer to generic event callback function.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_init(tProcessEventCb pfnApiProcessEventCb_p)
{
    tOplkError ret = kErrorOk;

    instance_l.pfnApiProcessEventCb = pfnApiProcessEventCb_p;

    ret = eventucal_init();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup user event module

This function cleans up the user event module.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_exit(void)
{
    tOplkError ret = kErrorOk;

    ret = eventucal_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event handler

This function processes events posted to the user layer. It examines the
sink and forwards the events by calling the event process function of the
specific module

\param  pEvent_p                Received event.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_process (tEvent *pEvent_p)
{
    tOplkError              ret = kErrorOk;
    tEventSource            eventSource;
    tProcessEventCb         pfnEventHandler;
    tEventDispatchEntry*    pDispatchEntry;

    pDispatchEntry = &eventDispatchTbl_l[0];
    ret = event_getHandlerForSink(&pDispatchEntry, pEvent_p->eventSink,
                                  &pfnEventHandler, &eventSource);
    if (ret == kErrorEventUnknownSink)
    {
        // Unknown sink, provide error event to API layer
        eventu_postError(kEventSourceEventu, ret, sizeof(pEvent_p->eventSink),
                        &pEvent_p->eventSink);
    }
    else
    {
        if (pfnEventHandler != NULL)
        {
            ret = pfnEventHandler(pEvent_p);
            if ((ret != kErrorOk) && (ret != kErrorShutdown))
            {
                // forward error event to API layer
                eventu_postError(kEventSourceEventu, ret,  sizeof(eventSource),
                                &eventSource);
            }
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It calls the post function of the
CAL module which distributes the event to the suitable event queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_postEvent (tEvent *pEvent_p)
{
    tOplkError ret = kErrorOk;

    // split event post to user internal and user to kernel
    switch(pEvent_p->eventSink)
    {
        // kernel layer modules
        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            ret = eventucal_postKernelEvent(pEvent_p);
            break;

        // user layer modules
        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
        case kEventSinkLedu:
            ret = eventucal_postUserEvent(pEvent_p);
            break;

        default:
            ret = kErrorEventUnknownSink;
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post an error event

This function posts an error event to the API module.

\param  eventSource_p           Source that caused the error
\param  error_p                 Error code
\param  argSize_p               Size of error argument
\param  pArg_p                  Error argument

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_postError (tEventSource eventSource_p,  tOplkError error_p,
                             UINT argSize_p, void* pArg_p)
{
    tOplkError          ret;
    tEventError         eventError;
    tEvent              event;

    ret = kErrorOk;

    // create argument
    eventError.eventSource = eventSource_p;
    eventError.oplkError = error_p;
    argSize_p = (UINT) min ((size_t) argSize_p, sizeof (eventError.errorArg));
    OPLK_MEMCPY(&eventError.errorArg, pArg_p, argSize_p);

    // create event
    event.eventType = kEventTypeError;
    event.eventSink = kEventSinkApi;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArgSize = (memberoffs (tEventError, errorArg) + argSize_p);
    event.pEventArg = &eventError;

    ret = eventu_postEvent(&event);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief	API event callback wrapper

This function implements an API event handler wrapper. It determines if
an API event callback was registered and calls it.

\param  pEvent_p            Pointer to event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callApiEventCb (tEvent* pEvent_p)
{
    if (instance_l.pfnApiProcessEventCb != NULL)
    {
        return instance_l.pfnApiProcessEventCb(pEvent_p);
    }
    return kErrorEventPostError;
}
/// \}


