/**
********************************************************************************
\file   eventucalintf-hostif.c

\brief  User event CAL interface module using the host interface

This file implements a kernel event CAL interface module which is using
the host interface for communication.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/target.h>

#include <user/eventucal.h>
#include <user/eventucalintf.h>

#include <hostiflib.h>
#include <lfqueue.h>

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
static tQueueInstance     instance_l[kEventQueueNum];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

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

\param  eventQueue_p            Event queue to initialize.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_initQueueHostif(tEventQueue eventQueue_p)
{
    tHostifInstance         pHifInstance;
    tHostifReturn           hifRet;
    tHostifInstanceId       hifInstanceId;
    UINT8*                  pBufBase = NULL;
    UINT                    bufSize;
    tQueueConfig            lfqConfig;
    tQueueReturn            lfqRet;

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] != NULL)
        return kErrorNoResource;

    pHifInstance = hostif_getInstance(0);

    if (pHifInstance == NULL)
        return kErrorNoResource;

    switch (eventQueue_p)
    {
        case kEventQueueK2U:
            hifInstanceId = kHostifInstIdK2UQueue;
            lfqConfig.queueRole = kQueueConsumer;
            break;

        case kEventQueueU2K:
            hifInstanceId = kHostifInstIdU2KQueue;
            lfqConfig.queueRole = kQueueProducer;
            break;

        default:
            return kErrorInvalidInstanceParam;
    }

    hifRet = hostif_getBuf(pHifInstance, hifInstanceId, &pBufBase, &bufSize);

    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                              __func__, hifRet);
        return kErrorNoResource;
    }

    lfqConfig.fAllocHeap = FALSE; // malloc done in hostif
    lfqConfig.pBase = pBufBase;
    lfqConfig.span = (UINT16)bufSize;
    lfqConfig.pfnCriticalSection = target_enableGlobalInterrupt;

    lfqRet = lfq_create(&lfqConfig, &instance_l[eventQueue_p]);

    if (lfqRet != kQueueSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Queue create failed (%d)\n",
                              __func__, lfqRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up an event queue

The function cleans up a circular buffer event queue. The queue to cleanup is
specified by eventQueue_p.

\param  eventQueue_p            Event queue to cleanup.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exitQueueHostif(tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorOk;

    switch (eventQueue_p)
    {
        case kEventQueueU2K:
        case kEventQueueK2U:
            lfq_delete(instance_l[eventQueue_p]);
            break;

        default:
            return kErrorInvalidInstanceParam;
            break;
    }

    instance_l[eventQueue_p] = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post event using host interface

This function posts an event to the specified hostif queue.

\param  eventQueue_p            Event queue to post the event to
\param  pEvent_p                Pointer to the event

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postEventHostif(tEventQueue eventQueue_p, tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tQueueReturn        lfqRet;
    DWORD               aPostBuffer[(sizeof(tEvent) + MAX_EVENT_ARG_SIZE) / 4];
    BYTE*               pPostBuffer = (BYTE*)aPostBuffer;
    ULONG               dataSize;

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    // initialize data size to mandatory part
    dataSize = sizeof(tEvent);

    // copy event into post buffer
    OPLK_MEMCPY(pPostBuffer, pEvent_p, dataSize);

    // copy argument data if present
    if (pEvent_p->pEventArg != NULL)
    {
        OPLK_MEMCPY(&pPostBuffer[dataSize], pEvent_p->pEventArg, pEvent_p->eventArgSize);
        // add optional argument data size
        dataSize += pEvent_p->eventArgSize;
    }

    lfqRet = lfq_entryEnqueue(instance_l[eventQueue_p], pPostBuffer, dataSize);
    if (lfqRet != kQueueSuccessful)
    {
        ret = kErrorEventPostError;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Process event using circular buffers

This function reads a circular buffer event queue and processes the event by
calling the event handler's process function.

\param  eventQueue_p            Event queue used for reading the event.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other                   Error

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_processEventHostif(tEventQueue eventQueue_p)
{
    tOplkError          ret = kErrorOk;
    tQueueReturn        lfqRet;
    tEvent*             pEplEvent;
    WORD                dataSize = sizeof(tEvent) + MAX_EVENT_ARG_SIZE;
    DWORD               aRxBuffer[(sizeof(tEvent) + MAX_EVENT_ARG_SIZE) / 4];
    BYTE*               pRxBuffer = (BYTE*)aRxBuffer;

    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    lfqRet = lfq_entryDequeue(instance_l[eventQueue_p], pRxBuffer, &dataSize);
    if (lfqRet != kQueueSuccessful)
    {
        eventu_postError(kEventSourceEventk, kErrorEventReadError,
                         sizeof(lfqRet), &lfqRet);
        goto Exit;
    }

    pEplEvent = (tEvent*)pRxBuffer;
    pEplEvent->eventArgSize = (UINT)dataSize - sizeof(tEvent);
    if(pEplEvent->eventArgSize > 0)
        pEplEvent->pEventArg = &pRxBuffer[sizeof(tEvent)];
    else
        pEplEvent->pEventArg = NULL;

    ret = eventu_process(pEplEvent);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get number of active events

This function returns the number of events which are currently available in
the circular buffer event queue.

\param  eventQueue_p            Event queue to read the count from.

\return The function returns the number of active events.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
UINT eventucal_getEventCountHostif(tEventQueue eventQueue_p)
{
    UINT16              count = 0;

    if (eventQueue_p > kEventQueueNum)
        return 0;

    if (instance_l[eventQueue_p] == NULL)
        return 0;

    lfq_getEntryCount(instance_l[eventQueue_p], &count);

    return count;
}

//------------------------------------------------------------------------------
/**
\brief  Setup event signaling for circular buffer event queue

This function sets up event signaling for the specified circular buffer event
queue.

\param  eventQueue_p            Event queue to read the count from.
\param  pfnSignalCb_p           Pointer to signaling callback function.

\return The function returns the number of active events.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------

tOplkError eventucal_setSignalingHostif(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kErrorInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kErrorInvalidInstanceParam;

    //TODO jz Implement async/event signaling in host interface

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

