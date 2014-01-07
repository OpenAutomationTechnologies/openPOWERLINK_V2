/**
********************************************************************************
\file   eventk.c

\brief  Source file for kernel event module

This file contains the source code of the kernel event module. It provides the
interface for posting events to other kernel modules.

\ingroup module_eventk
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
#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <kernel/nmtk.h>
#include <kernel/dllk.h>
#include <kernel/dllkcal.h>
#include <kernel/errhndk.h>
#include <Benchmark.h>

#if defined(CONFIG_INCLUDE_PDO)
#include <kernel/pdok.h>
#include <kernel/pdokcal.h>
#endif

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel handleNmtEventinDll(tEplEvent* pEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/**
\brief  Event dispatch table

The following table defines the event handlers to be used for the specific
event sinks.
*/
static tEventDispatchEntry eventDispatchTbl_l[] =
{
    { kEplEventSinkNmtk,        kEplEventSourceNmtk,        nmtk_process },
    { kEplEventSinkNmtk,        kEplEventSourceDllk,        handleNmtEventinDll },
    { kEplEventSinkDllk,        kEplEventSourceDllk,        dllk_process },
    { kEplEventSinkDllkCal,     kEplEventSourceDllk,        dllkcal_process },
    { kEplEventSinkErrk,        kEplEventSourceErrk,        errhndk_process },
#if defined(CONFIG_INCLUDE_PDO)
    { kEplEventSinkPdokCal,     kEplEventSourcePdok,        pdokcal_process },
#endif
    { kEplEventSinkInvalid,     kEplEventSourceInvalid,     NULL }
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event module

The function initializes the kernel event module. It is also responsible to call
the init function of it's CAL module.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tEplKernel eventk_init (void)
{
    tEplKernel  ret = kEplSuccessful;

    ret = eventkcal_init();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event module

This function cleans up the kernel event module.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tEplKernel eventk_exit (void)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventkcal_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event handler

This function processes events posted to the kernel layer. It examines the
sink and forwards the events by calling the event process function of the
specific module.

\param  pEvent_p                Received event.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tEplKernel eventk_process (tEplEvent *pEvent_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplEventSource         eventSource;
    tEplProcessEventCb      pfnEventHandler;
    BOOL                    fStop = FALSE;
    BOOL                    fAlreadyHandled = FALSE;
    tEventDispatchEntry*    pDispatchEntry;

    pDispatchEntry = &eventDispatchTbl_l[0];
    while (!fStop)
    {
        ret = event_getHandlerForSink(&pDispatchEntry, pEvent_p->m_EventSink,
                                      &pfnEventHandler, &eventSource);
        if (ret == kEplEventUnknownSink)
        {
                if (!fAlreadyHandled)
                {
                    // Unknown sink, provide error event to API layer
                    eventk_postError(kEplEventSourceEventk, ret,
                                     sizeof(pEvent_p->m_EventSink),
                                     &pEvent_p->m_EventSink);
                }
                else
                {
                    ret = kEplSuccessful;
                }
                fStop = TRUE;
        }
        else
        {
            if (pfnEventHandler != NULL)
            {
                ret = pfnEventHandler(pEvent_p);
                if ((ret != kEplSuccessful) && (ret != kEplShutdown))
                {
                    // forward error event to API layer
                    eventk_postError(kEplEventSourceEventk, ret,
                                     sizeof(eventSource),
                                      &eventSource);
                }
            }
            fAlreadyHandled = TRUE;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It calls the post function of the
CAL module which distributes the event to the suitable event queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tEplKernel eventk_postEvent (tEplEvent *pEvent_p)
{
    tEplKernel ret = kEplSuccessful;

    switch(pEvent_p->m_EventSink)
    {
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
            ret = eventkcal_postUserEvent(pEvent_p);
            break;

        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
            ret = eventkcal_postKernelEvent(pEvent_p);
            break;

        default:
            ret = kEplEventUnknownSink;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post an error event

This function posts an error event to the API module.

\param  eventSource_p           Source that caused the error
\param  eplError_p              Error code
\param  argSize_p               Size of error argument
\param  pArg_p                  Error argument

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tEplKernel eventk_postError (tEplEventSource eventSource_p, tEplKernel eplError_p,
                             UINT argSize_p, void *pArg_p)
{
    tEplKernel          ret;
    tEplEventError      eventError;
    tEplEvent           eplEvent;

    ret = kEplSuccessful;

    // create argument
    eventError.m_EventSource = eventSource_p;
    eventError.m_EplError = eplError_p;
    argSize_p = (UINT) min ((size_t) argSize_p, sizeof (eventError.m_Arg));
    EPL_MEMCPY(&eventError.m_Arg, pArg_p, argSize_p);

    // create event
    eplEvent.m_EventType = kEplEventTypeError;
    eplEvent.m_EventSink = kEplEventSinkApi;
    EPL_MEMSET(&eplEvent.m_NetTime, 0x00, sizeof(eplEvent.m_NetTime));
    eplEvent.m_uiSize = (memberoffs (tEplEventError, m_Arg) + argSize_p);
    eplEvent.m_pArg = &eventError;

    ret = eventk_postEvent(&eplEvent);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Handle NMT event in DLL

This function dispatches NMT events which need also be handled by DLL to
the DLLk module.

\param  pEvent_p                Event to process.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
static tEplKernel handleNmtEventinDll(tEplEvent* pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;

    BENCHMARK_MOD_27_RESET(0);

    if ((pEvent_p->m_EventType == kEplEventTypeNmtEvent) &&
        (*((tNmtEvent*)pEvent_p->m_pArg) == kNmtEventDllCeSoa))
    {
        BENCHMARK_MOD_27_SET(0);
        // forward SoA event to DLLk module for cycle preprocessing
        ret = dllk_process(pEvent_p);

        BENCHMARK_MOD_27_RESET(0);
    }

    return ret;
}

/// \}
