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
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <common/oplkinc.h>
#include <user/eventu.h>
#include <user/nmtu.h>
#include <user/dllucal.h>
#include <user/eventucal.h>

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <user/nmtmnu.h>
#endif

#if (defined(CONFIG_INCLUDE_SDOC) || defined(CONFIG_INCLUDE_SDOS))
#include <user/sdoseq.h>
#include <user/sdotest.h>
#endif

#include <oplk/debugstr.h>
#include <stddef.h>

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

The user event instance holds the API process callback function pointer.
*/
typedef struct
{
    tProcessEventCb         pfnApiProcessEventCb;  ///< Callback for generic API events
    BOOL                    fInitialized;          ///< Flag to determine status of eventu module
} tEventuInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError callApiEventCb(const tEvent* pEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuInstance      instance_l;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event module

The function initializes the user event module. It is also responsible to call
the init function of its CAL module.

\param[in]      pfnApiProcessEventCb_p  Function pointer to generic event callback function.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_init(tProcessEventCb pfnApiProcessEventCb_p)
{
    tOplkError  ret;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuInstance));

    instance_l.pfnApiProcessEventCb = pfnApiProcessEventCb_p;

    ret = eventucal_init();
    if (ret == kErrorOk)
        instance_l.fInitialized = TRUE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event module

This function cleans up the user event module.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_exit(void)
{
    tOplkError  ret;

    ret = eventucal_exit();
    instance_l.fInitialized = FALSE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event handler

This function processes events posted to the user layer. It examines the
sink and forwards the events by calling the event process function of the
specific module

\param[in]      pEvent_p            Received event.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_process(const tEvent* pEvent_p)
{
    tOplkError      ret = kErrorOk;
    tEventSource    eventSource = kEventSourceInvalid;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    if (!instance_l.fInitialized)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Eventu module is not initialized\n", __func__);
        return kErrorNoResource;
    }

    switch (pEvent_p->eventSink)
    {
        case kEventSinkDlluCal:
            ret = dllucal_process(pEvent_p);
            eventSource = kEventSourceDllu;
            break;

        case kEventSinkNmtu:
            ret = nmtu_processEvent(pEvent_p);
            eventSource = kEventSourceNmtu;
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kEventSinkNmtMnu:
            ret = nmtmnu_processEvent(pEvent_p);
            eventSource = kEventSourceNmtMnu;
            break;
#endif

#if (defined(CONFIG_INCLUDE_SDOC) || defined(CONFIG_INCLUDE_SDOS))
        case kEventSinkSdoAsySeq:
            ret = sdoseq_processEvent(pEvent_p);
            eventSource = kEventSourceSdoAsySeq;
            break;
#endif

        case kEventSinkErru:
            break;

        case kEventSinkApi:
            ret = callApiEventCb(pEvent_p);
            eventSource = kEventSourceOplkApi;
            break;

        case kEventSinkSdoTest:
            ret = sdotestcom_cbEvent(pEvent_p);
            eventSource = kEventSourceSdoTest;
            break;

        default:
            // Unknown sink, provide error event to API layer
            eventu_postError(kEventSourceEventu,
                             ret,
                             sizeof(pEvent_p->eventSink),
                             &pEvent_p->eventSink);
            ret = kErrorEventUnknownSink;
            break;
    }

    if ((ret != kErrorOk) && (ret != kErrorShutdown))
    {
        // forward error event to API layer
        eventu_postError(kEventSourceEventu,
                         ret,
                         sizeof(eventSource),
                         &eventSource);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It calls the post function of the
CAL module which distributes the event to the suitable event queue.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_postEvent(const tEvent* pEvent_p)
{
    tOplkError  ret = kErrorOk;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    if (!instance_l.fInitialized)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Eventu module is not initialized\n", __func__);
        return kErrorNoResource;
    }

    // Split event post to user internal and user to kernel
    switch (pEvent_p->eventSink)
    {
        // kernel layer modules
        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
        case kEventSinkTimesynck:
            DEBUG_LVL_EVENTU_TRACE("U2K type:%s(%d) sink:%s(%d) size:%d!\n",
                                   debugstr_getEventTypeStr(pEvent_p->eventType),
                                   pEvent_p->eventType,
                                   debugstr_getEventSinkStr(pEvent_p->eventSink),
                                   pEvent_p->eventSink,
                                   pEvent_p->eventArgSize);
            ret = eventucal_postKernelEvent(pEvent_p);
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("User to kernel event could not be posted!!\n");
            }
            break;

        // user layer modules
        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkSdoTest:
        case kEventSinkDlluCal:
        case kEventSinkErru:
            DEBUG_LVL_EVENTU_TRACE("UINT  type:%s(%d) sink:%s(%d) size:%d!\n",
                                   debugstr_getEventTypeStr(pEvent_p->eventType),
                                   pEvent_p->eventType,
                                   debugstr_getEventSinkStr(pEvent_p->eventSink),
                                   pEvent_p->eventSink,
                                   pEvent_p->eventArgSize);
            ret = eventucal_postUserEvent(pEvent_p);
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("User internal event could not be posted!!\n");
            }
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
\param[in]      error_p             Error code
\param[in]      argSize_p           Size of error argument
\param[in]      pArg_p              Error argument

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventu
*/
//------------------------------------------------------------------------------
tOplkError eventu_postError(tEventSource eventSource_p,
                            tOplkError error_p,
                            UINT argSize_p,
                            const void* pArg_p)
{
    tOplkError  ret;
    tEventError eventError;
    tEvent      event;

    // Check parameter validity
    ASSERT((argSize_p == 0) ||
           (pArg_p != NULL));

    // create argument
    eventError.eventSource = eventSource_p;
    eventError.oplkError = error_p;
    argSize_p = (UINT)min((size_t)argSize_p, sizeof(eventError.errorArg));
    OPLK_MEMCPY(&eventError.errorArg, pArg_p, argSize_p);

    // create event
    event.eventType = kEventTypeError;
    event.eventSink = kEventSinkApi;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArgSize = offsetof(tEventError, errorArg) + argSize_p;
    event.eventArg.pEventArg = &eventError;

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
\brief  API event callback wrapper

This function implements an API event handler wrapper. It determines if an API
event callback was registered and calls it.

\param[in]      pEvent_p            Pointer to event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callApiEventCb(const tEvent* pEvent_p)
{
    if (instance_l.pfnApiProcessEventCb != NULL)
        return instance_l.pfnApiProcessEventCb(pEvent_p);

    return kErrorEventPostError;
}

/// \}
