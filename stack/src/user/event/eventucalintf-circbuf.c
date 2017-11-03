/**
********************************************************************************
\file   eventucalintf-circbuf.c

\brief  User event CAL interface module using circular buffers

This file implements a user event CAL interface module which is using
circular buffers for communication.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <common/oplkinc.h>
#include <user/eventucalintf.h>
#include <user/eventu.h>
#include <common/circbuffer.h>

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
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCircBufInstance*       instance_l[kEventQueueNum];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError postEvent(tCircBufInstance* pCircBufInstance_p,
                            const tEvent* pEvent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize an event queue

The function initializes a circular buffer event queue. The queue to initialize
is specified by eventQueue_p.

\param[in]      eventQueue_p        Event queue to initialize.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_initQueueCircbuf(tEventQueue eventQueue_p)
{
    tCircBufError   circError;

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] != NULL)
        return kErrorNoResource;

    switch (eventQueue_p)
    {
        case kEventQueueUInt:
            circError = circbuf_alloc(CIRCBUF_USER_INTERNAL_QUEUE,
                                      CONFIG_EVENT_SIZE_CIRCBUF_USER_INTERNAL,
                                      &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_USER_INTERNAL_QUEUE circbuffer\n");
                return kErrorNoResource;
            }
            break;

        case kEventQueueU2K:
            circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
                return kErrorNoResource;
            }
            break;

        case kEventQueueK2U:
            circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
                return kErrorNoResource;
            }
            break;

        default:
            return kErrorInvalidInstanceParam;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up a event queue

The function cleans up a circular buffer event queue. The queue to cleanup is
specified by eventQueue_p.

\param[in]      eventQueue_p        Event queue to cleanup.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exitQueueCircbuf(tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorOk;

    switch (eventQueue_p)
    {
        case kEventQueueUInt:
            circbuf_free(instance_l[eventQueue_p]);
            break;

        case kEventQueueU2K:
            circbuf_disconnect(instance_l[eventQueue_p]);
            break;

        case kEventQueueK2U:
            circbuf_disconnect(instance_l[eventQueue_p]);
            break;

        default:
            return kErrorInvalidInstanceParam;
    }

    instance_l[eventQueue_p] = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post event using circular buffer

This function posts an event to the provided queue instance.

\param[in]      eventQueue_p        Event queue to which the event should be posted to.
\param[in]      pEvent_p            Pointer to event

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postEventCircbuf(tEventQueue eventQueue_p,
                                      const tEvent* pEvent_p)
{
    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    return postEvent(instance_l[eventQueue_p], pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Process event using circular buffers

This function reads a circular buffer event queue and processes the event
by calling the event handlers process function.

\param[in]      eventQueue_p        Event queue used for reading the event.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_processEventCircbuf(tEventQueue eventQueue_p)
{
    tEvent*             pEvent;
    tCircBufError       error;
    tOplkError          ret;
    size_t              readSize;
    tCircBufInstance*   pCircBufInstance;
    BYTE                aRxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    pCircBufInstance = instance_l[eventQueue_p];

    error = circbuf_readData(pCircBufInstance,
                             aRxBuffer,
                             sizeof(tEvent) + MAX_EVENT_ARG_SIZE,
                             &readSize);
    if (error != kCircBufOk)
    {
        if (error == kCircBufNoReadableData)
            return kErrorOk;

        eventu_postError(kEventSourceEventk,
                         kErrorEventReadError,
                         sizeof(tCircBufError),
                         &error);

        return kErrorGeneralError;
    }

    pEvent = (tEvent*)aRxBuffer;
    pEvent->eventArgSize = (UINT)(readSize - sizeof(tEvent));

    if (pEvent->eventArgSize > 0)
        pEvent->eventArg.pEventArg = &aRxBuffer[sizeof(tEvent)];
    else
        pEvent->eventArg.pEventArg = NULL;

    ret = eventu_process(pEvent);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get number of active events

This function returns the number of events which are currently available in
the circular buffer event queue.

\param[in]      eventQueue_p        Event queue to read the count from.

\return The function returns the number of active events.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
UINT eventucal_getEventCountCircbuf(tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return 0;

    if (instance_l[eventQueue_p] == NULL)
        return 0;

    return circbuf_getDataCount(instance_l[eventQueue_p]);
}

//------------------------------------------------------------------------------
/**
\brief  Setup event signaling for circular buffer event queue

This function sets up event signaling for the specified circular buffer event
queue.

\param[in]      eventQueue_p        Event queue to read the count from.
\param[in]      pfnSignalCb_p       Pointer to signaling callback function.

\return The function returns the number of active events.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_setSignalingCircbuf(tEventQueue eventQueue_p,
                                         VOIDFUNCPTR pfnSignalCb_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    circBuf_setSignaling(instance_l[eventQueue_p], pfnSignalCb_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Post event using circular buffer

This function posts an event to the provided queue instance.

\param[in]      pCircBufInstance_p  Pointer to circular buffer instance
\param[in]      pEvent_p            Pointer to event

\return tOplkError
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(tCircBufInstance* pCircBufInstance_p,
                            const tEvent* pEvent_p)
{
    tOplkError      ret = kErrorOk;
    tCircBufError   circError;

    DEBUG_LVL_EVENTU_TRACE("%s() Event:%d Sink:%d\n",
                           __func__,
                           pEvent_p->eventType,
                           pEvent_p->eventSink);

    if (pEvent_p->eventArgSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance_p,
                                      pEvent_p,
                                      sizeof(tEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance_p,
                                              pEvent_p,
                                              sizeof(tEvent),
                                              pEvent_p->eventArg.pEventArg,
                                              (ULONG)pEvent_p->eventArgSize);
    }

    if (circError != kCircBufOk)
        ret = kErrorEventPostError;

    return ret;
}

/// \}
