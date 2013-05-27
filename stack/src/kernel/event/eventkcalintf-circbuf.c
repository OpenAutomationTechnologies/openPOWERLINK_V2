/**
********************************************************************************
\file   eventkcalintf-circbuf.c

\brief  Kernel event CAL interface module using circular buffers

This file implements a kernel event CAL interface module which is using
circular buffers for communication.

\ingroup module_eventkcal
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
#include <EplInc.h>
#include <Epl.h>

#include <eventcal.h>
#include <kernel/eventkcal.h>

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
static tCircBufInstance*        instance_l[kEventQueueNum];
static BYTE                     aRxBuffer_l[kEventQueueNum][sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
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
tEplKernel eventkcal_initQueueCircbuf(tEventQueue eventQueue_p)
{
    tCircBufError           circError = kCircBufOk;

    if (eventQueue_p > kEventQueueNum)
    {
        TRACE("%s() Error: invalid queue %d!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    if (instance_l[eventQueue_p] != NULL)
    {
        TRACE("%s() Error: instance of queue %d not NULL!\n", __func__, eventQueue_p);
        return kEplNoResource;
    }

    switch(eventQueue_p)
    {
        case kEventQueueKInt:
            circError = circbuf_alloc(CIRCBUF_KERNEL_INTERNAL_QUEUE, EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL,
                                      &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_USER_INTERNAL_QUEUE circbuffer\n");
                return kEplNoResource;
            }
            break;

        case kEventQueueU2K:
            circError = circbuf_alloc(CIRCBUF_USER_TO_KERNEL_QUEUE, EVENT_SIZE_CIRCBUF_USER_TO_KERNEL,
                                      &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
                return kEplNoResource;
            }

            break;

        case kEventQueueK2U:
            circError = circbuf_alloc(CIRCBUF_KERNEL_TO_USER_QUEUE, EVENT_SIZE_CIRCBUF_KERNEL_TO_USER,
                                      &instance_l[eventQueue_p]);
            if (circError != kCircBufOk)
            {
                TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
                return kEplNoResource;
            }
            break;

        case kEventQueueUInt:
            circError = circbuf_alloc(CIRCBUF_USER_INTERNAL_QUEUE, EVENT_SIZE_CIRCBUF_USER_INTERNAL,
                                      &instance_l[eventQueue_p]);
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
tEplKernel eventkcal_exitQueueCircbuf (tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplSuccessful;

    circbuf_free(instance_l[eventQueue_p]);

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
tEplKernel eventkcal_postEventCircbuf (tEventQueue eventQueue_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tCircBufError       circError;

    if (eventQueue_p > kEventQueueNum)
    {
        TRACE("%s() invalid queue %d!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    if (instance_l[eventQueue_p] == NULL)
    {
        TRACE("%s() instance %d = NULL!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    /*TRACE("%s() Event:%d Sink:%d\n", __func__, pEvent_p->m_EventType, pEvent_p->m_EventSink);*/
    if (pEvent_p->m_uiSize == 0)
    {
        circError = circbuf_writeData(instance_l[eventQueue_p], pEvent_p, sizeof(tEplEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(instance_l[eventQueue_p], pEvent_p, sizeof(tEplEvent),
                                        pEvent_p->m_pArg, (ULONG)pEvent_p->m_uiSize);
    }
    if(circError != kCircBufOk)
    {
        ret = kEplEventPostError;
    }
    return ret;
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
tEplKernel eventkcal_processEventCircbuf(tEventQueue eventQueue_p)
{
    tEplEvent*          pEplEvent;
    tCircBufError       error;
    tEplKernel          ret = kEplSuccessful;
    size_t              readSize;
    tCircBufInstance*   pCircBufInstance;

    //TRACE("%s()\n", __func__);

    if (eventQueue_p > kEventQueueNum)
    {
        TRACE("%s() invalid queue %d!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    if (instance_l[eventQueue_p] == NULL)
    {
        TRACE("%s() instance %d = NULL!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    pCircBufInstance = instance_l[eventQueue_p];

    error = circbuf_readData(pCircBufInstance, aRxBuffer_l[eventQueue_p],
                             sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE, &readSize);
    if(error != kCircBufOk)
    {
        if (error == kCircBufNoReadableData)
            return kEplSuccessful;

        eventk_postError(kEplEventSourceEventk, kEplEventReadError,
                         sizeof(tCircBufError), &error);

        return kEplGeneralError;
    }
    pEplEvent = (tEplEvent *) aRxBuffer_l[eventQueue_p];
    pEplEvent->m_uiSize = (readSize - sizeof(tEplEvent));

    if(pEplEvent->m_uiSize > 0)
        pEplEvent->m_pArg = &aRxBuffer_l[eventQueue_p][sizeof(tEplEvent)];
    else
        pEplEvent->m_pArg = NULL;

    /*TRACE("Process Kernel  type:%s(%d) sink:%s(%d) size:%d!\n",
           EplGetEventTypeStr(pEplEvent->m_EventType), pEplEvent->m_EventType,
           EplGetEventSinkStr(pEplEvent->m_EventSink), pEplEvent->m_EventSink,
           pEplEvent->m_uiSize);*/

    ret = eventk_process(pEplEvent);
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    Read event from circular buffers

This function reads a circular buffer event queue and stores the event data
at pDataBuffer_p.

\param  eventQueue_p            Event queue used for reading the event.
\param  pDataBuffer_p           Pointer to store event.
\param  pReadSize_p             Pointer to store length of event.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_getEventCircbuf(tEventQueue eventQueue_p, BYTE* pDataBuffer_p,
                                     size_t* pReadSize_p)
{
    tCircBufError       error;
    tCircBufInstance*   pCircBufInstance;

    if (eventQueue_p > kEventQueueNum)
    {
        TRACE("%s() invalid queue %d!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    if (instance_l[eventQueue_p] == NULL)
    {
        TRACE("%s() instance %d = NULL!\n", __func__, eventQueue_p);
        return kEplInvalidInstanceParam;
    }

    pCircBufInstance = instance_l[eventQueue_p];

    error = circbuf_readData(pCircBufInstance, pDataBuffer_p,
                             sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE, pReadSize_p);
    if(error != kCircBufOk)
    {
        if (error == kCircBufNoReadableData)
            return kEplSuccessful;

        eventk_postError(kEplEventSourceEventk, kEplEventReadError,
                         sizeof(tCircBufError), &error);

        return kEplGeneralError;
    }

    return kEplSuccessful;
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
UINT eventkcal_getEventCountCircbuf(tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
    {
        TRACE("%s() Invalid event queue %d!!!\n", __func__, eventQueue_p);
        return 0;
    }

    if (instance_l[eventQueue_p] == NULL)
    {
        TRACE("%s() No event queue instance %d!!!\n", __func__, eventQueue_p);
        return 0;
    }

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

tEplKernel eventkcal_setSignalingCircbuf(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p)
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

/// \}
