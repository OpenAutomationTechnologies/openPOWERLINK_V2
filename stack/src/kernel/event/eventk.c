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
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <stddef.h>

#include <common/oplkinc.h>
#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <kernel/nmtk.h>
#include <kernel/dllk.h>
#include <kernel/dllkcal.h>
#include <kernel/errhndk.h>
#include <kernel/timesynck.h>
#include <oplk/benchmark.h>

#if defined(CONFIG_INCLUDE_PDO)
#include <kernel/pdokcal.h>
#endif

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
static tOplkError handleNmtEventInDll(const tEvent* pEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event module

The function initializes the kernel event module. It is also responsible to call
the init function of its CAL module.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tOplkError eventk_init(void)
{
    tOplkError  ret;

    ret = eventkcal_init();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up kernel event module

This function cleans up the kernel event module.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tOplkError eventk_exit(void)
{
    tOplkError  ret;

    ret = eventkcal_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event handler

This function processes events posted to the kernel layer. It examines the
sink and forwards the events by calling the event process function of the
specific module.

\param[in]      pEvent_p            Received event.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tOplkError eventk_process(const tEvent* pEvent_p)
{
    tOplkError      ret = kErrorOk;
    tEventSource    eventSource;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    switch (pEvent_p->eventSink)
    {
        // Note: case statements are sorted for best performance!
        case kEventSinkDllk:
            ret = dllk_process(pEvent_p);
            eventSource = kEventSourceDllk;
            break;

#if defined(CONFIG_INCLUDE_PDO)
        case kEventSinkPdokCal:
            ret = pdokcal_process(pEvent_p);
            eventSource = kEventSourcePdok;
            break;
#endif
        case kEventSinkDllkCal:
            ret = dllkcal_process(pEvent_p);
            eventSource = kEventSourceDllk;
            break;

        case kEventSinkNmtk:
            ret = nmtk_process(pEvent_p);
            eventSource = kEventSourceNmtk;
            if ((ret != kErrorOk) && (ret != kErrorShutdown))
            {
                // forward error event to API layer
                eventk_postError(kEventSourceEventk,
                                 ret,
                                 sizeof(eventSource),
                                 &eventSource);
            }
            ret = handleNmtEventInDll(pEvent_p);
            eventSource = kEventSourceDllk;
            break;

        case kEventSinkErrk:
            ret = errhndk_process(pEvent_p);
            eventSource = kEventSourceErrk;
            break;

        case kEventSinkTimesynck:
            ret = timesynck_process(pEvent_p);
            eventSource = kEventSourceTimesynck;
            break;

        default:
            // Unknown sink, provide error event to API layer
            ret = kErrorEventUnknownSink;
            eventk_postError(kEventSourceEventk,
                             ret,
                             sizeof(pEvent_p->eventSink),
                             &pEvent_p->eventSink);
            break;
    }

    if ((ret != kErrorOk) &&
        (ret != kErrorShutdown) &&
        (ret != kErrorEventUnknownSink))
    {
        // forward error event to API layer
        eventk_postError(kEventSourceEventk,
                         ret,
                         sizeof(eventSource),
                         &eventSource);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It calls the post function of the
CAL module which distributes the event to the suitable event queue.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tOplkError eventk_postEvent(const tEvent* pEvent_p)
{
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    switch (pEvent_p->eventSink)
    {
        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
            ret = eventkcal_postUserEvent(pEvent_p);
            break;

        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
        case kEventSinkTimesynck:
            ret = eventkcal_postKernelEvent(pEvent_p);
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

\param[in]      eventSource_p       Source that caused the error
\param[in]      oplkError_p         Error code
\param[in]      argSize_p           Size of error argument
\param[in]      pArg_p              Error argument

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventk
*/
//------------------------------------------------------------------------------
tOplkError eventk_postError(tEventSource eventSource_p,
                            tOplkError oplkError_p,
                            UINT argSize_p,
                            const void* pArg_p)
{
    tOplkError  ret;
    tEventError eventError;
    tEvent      oplkEvent;

    // Check parameter validity
    ASSERT(pArg_p != NULL);

    // create argument
    eventError.eventSource = eventSource_p;
    eventError.oplkError = oplkError_p;
    argSize_p = (UINT)min((size_t)argSize_p, sizeof(eventError.errorArg));
    OPLK_MEMCPY(&eventError.errorArg, pArg_p, argSize_p);

    // create event
    oplkEvent.eventType = kEventTypeError;
    oplkEvent.eventSink = kEventSinkApi;
    OPLK_MEMSET(&oplkEvent.netTime, 0x00, sizeof(oplkEvent.netTime));
    oplkEvent.eventArgSize = offsetof(tEventError, errorArg) + argSize_p;
    oplkEvent.eventArg.pEventArg = &eventError;

    ret = eventk_postEvent(&oplkEvent);

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

This function dispatches NMT events which need to be handled by DLL to the DLLk
module.

\param[in]      pEvent_p            Event to process.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError handleNmtEventInDll(const tEvent* pEvent_p)
{
    tOplkError  ret = kErrorOk;

    BENCHMARK_MOD_27_RESET(0);

    if ((pEvent_p->eventType == kEventTypeNmtEvent) &&
        (*((tNmtEvent*)pEvent_p->eventArg.pEventArg) == kNmtEventDllCeSoa))
    {
        BENCHMARK_MOD_27_SET(0);

        // forward SoA event to DLLk module for cycle preprocessing
        ret = dllk_process(pEvent_p);

        BENCHMARK_MOD_27_RESET(0);
    }

    return ret;
}

/// \}
