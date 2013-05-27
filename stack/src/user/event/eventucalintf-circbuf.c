/**
********************************************************************************
\file   eventucalintf-circbuf.c

\brief  User event CAL interface module using circular buffers

This file implements a user event CAL interface module which is using
circular buffers for communication.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <eventcal.h>
#include <user/eventucal.h>

#include <circbuffer.h>

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
static tEplKernel postEvent (tCircBufInstance* pCircBufInstance_p, tEplEvent *pEvent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize a event queue

The function initializes a circular buffer event queue. The queue to initialize
is specified by eventQueue_p.

\param  eventQueue_p            Event queue to initialize.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_initQueueCircbuf(tEventQueue eventQueue_p)
{
    tCircBufError           circError = kCircBufOk;

    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] != NULL)
        return kEplNoResource;

    switch(eventQueue_p)
    {
        case kEventQueueUInt:
            circError = circbuf_alloc(CIRCBUF_USER_INTERNAL_QUEUE, EVENT_SIZE_CIRCBUF_USER_INTERNAL,
                                      &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_USER_INTERNAL_QUEUE circbuffer\n");
                return kEplNoResource;
            }
            break;

        case kEventQueueU2K:
            circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
                return kEplNoResource;
            }

            break;

        case kEventQueueK2U:
            circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
                return kEplNoResource;
            }
            break;

        default:
            return kEplInvalidInstanceParam;
            break;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup a event queue

The function cleans up a circular buffer event queue. The queue to cleanup is
specified by eventQueue_p.

\param  eventQueue_p            Event queue to cleanup.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_exitQueueCircbuf (tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplSuccessful;

    switch(eventQueue_p)
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
            return kEplInvalidInstanceParam;
            break;
    }

    instance_l[eventQueue_p] = NULL;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post event using circular buffer

This function posts an event to the provided queue instance.

\param  pEventQueue_p           Pointer to event queue instance
\param  pEvent_p                Pointer to event

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_postEventCircbuf (tEventQueue eventQueue_p, tEplEvent *pEvent_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    return postEvent(instance_l[eventQueue_p], pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Process event using circular buffers

This function reads a circular buffer event queue and processes the event
by calling the event handlers process function.

\param  eventQueue_p            Event queue used for reading the event.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_processEventCircbuf(tEventQueue eventQueue_p)
{
    tEplEvent*          pEplEvent;
    tCircBufError       error;
    tEplKernel          ret = kEplSuccessful;
    size_t              readSize;
    tCircBufInstance*   pCircBufInstance;
    BYTE                aRxBuffer[sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE];

    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    pCircBufInstance = instance_l[eventQueue_p];

    error = circbuf_readData(pCircBufInstance, aRxBuffer,
                             sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE, &readSize);
    if(error != kCircBufOk)
    {
        if (error == kCircBufNoReadableData)
            return kEplSuccessful;

        eventu_postError(kEplEventSourceEventk, kEplEventReadError,
                         sizeof(tCircBufError), &error);

        return kEplGeneralError;
    }

    pEplEvent = (tEplEvent *) aRxBuffer;
    pEplEvent->m_uiSize = (readSize - sizeof(tEplEvent));

    if(pEplEvent->m_uiSize > 0)
        pEplEvent->m_pArg = &aRxBuffer[sizeof(tEplEvent)];
    else
        pEplEvent->m_pArg = NULL;

    ret = eventu_process(pEplEvent);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get number of active events

This function returns the number of events which are currently available in
the circular buffer event queue.

\param  eventQueue_p            Event queue to read the count from.


\return The function returns the number of active events.
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

\param  eventQueue_p            Event queue to read the count from.
\param  pfnSignalCb_p           Pointer to signaling callback function.

\return The function returns the number of active events.
*/
//------------------------------------------------------------------------------

tEplKernel eventucal_setSignalingCircbuf(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    circBuf_setSignaling(instance_l[eventQueue_p], pfnSignalCb_p);
    return kEplSuccessful;
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

\param  pCircBufInstance_p      Pointer to circular buffer instance
\param  pEvent_p                Pointer to event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel postEvent (tCircBufInstance* pCircBufInstance_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tCircBufError       circError;
    //TRACE("%s() Event:%d Sink:%d\n", __func__, pEvent_p->m_EventType, pEvent_p->m_EventSink);

    if (pEvent_p->m_uiSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance_p, pEvent_p, sizeof(tEplEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance_p, pEvent_p, sizeof(tEplEvent),
                                        pEvent_p->m_pArg, (ULONG)pEvent_p->m_uiSize);
    }
    if(circError != kCircBufOk)
    {
        ret = kEplEventPostError;
    }
    return ret;
}

/// \}

