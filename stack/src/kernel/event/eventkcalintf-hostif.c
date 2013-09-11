/**
********************************************************************************
\file   eventkcalintf-hostif.c

\brief  Kernel event CAL interface module using the host interface

This file implements a kernel event CAL interface module which is using
the host interface for communication.

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

#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>

#include <hostiflib.h>

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
static tHostifQueueInstance     instance_l[kEventQueueNum];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void rxSignalHandlerCb(void* pArg_p) SECTION_EVENTKCAL_HOSTIF_RXHDL;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize a event queue

The function initializes a host interface event queue. The queue to initialize
is specified by eventQueue_p.

\param  eventQueue_p            Event queue to initialize.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_initQueueHostif(tEventQueue eventQueue_p)
{
    tHostifInstance         pHifInstance;
    tHostifReturn           hifRet;

    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] != NULL)
        return kEplNoResource;

    pHifInstance = hostif_getInstance(kHostifProcPcp);

    switch(eventQueue_p)
    {
        case kEventQueueK2U:
            hifRet = hostif_queueCreate(pHifInstance, kHostifInstIdK2UQueue,
                                        &instance_l[eventQueue_p]);
            if(hifRet != kHostifSuccessful)
            {
                EPL_DBGLVL_ERROR_TRACE("%s() couldn't create queue instance (%d)\n",
                        __func__, hifRet);

                return kEplNoResource;
            }
            break;

        case kEventQueueU2K:
            hifRet = hostif_queueCreate(pHifInstance, kHostifInstIdU2KQueue,
                                        &instance_l[eventQueue_p]);
            if(hifRet != kHostifSuccessful)
            {
                EPL_DBGLVL_ERROR_TRACE("%s() couldn't create queue instance (%d)\n",
                        __func__, hifRet);
                return kEplNoResource;
            }

            //Assign queue callback
            hifRet = hostif_queueCallback(instance_l[eventQueue_p],
                    rxSignalHandlerCb, (void*)eventQueue_p);
            if(hifRet != kHostifSuccessful)
            {
                EPL_DBGLVL_ERROR_TRACE("%s() couldn't assign queue callback (%d)\n",
                        __func__, hifRet);
                return kEplNoResource;
            }
            break;

        default:
            return kEplInvalidInstanceParam;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup a event queue

The function cleans up a host interface event queue. The queue to cleanup is
specified by eventQueue_p.

\param  eventQueue_p            Event queue to cleanup.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_exitQueueHostif (tEventQueue eventQueue_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplSuccessful;

    switch(eventQueue_p)
    {
        case kEventQueueU2K:
        case kEventQueueK2U:
            hostif_queueDelete(instance_l[eventQueue_p]);
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
\brief    Post event using host interface

This function posts an event to the specified host interface queue.

\param  eventQueue_p            Event queue to which the event should be posted.
\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postEventHostif (tEventQueue eventQueue_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tHostifReturn       hifRet;
    DWORD               aPostBuffer[(sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE)/4];
    BYTE*               pPostBuffer = (BYTE*)aPostBuffer;
    ULONG               dataSize;

    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    // initialize data size to mandatory part
    dataSize = sizeof(tEplEvent);

    // copy event into post buffer
    EPL_MEMCPY(pPostBuffer, pEvent_p, dataSize);

    // copy argument data if present
    if(pEvent_p->m_pArg != NULL)
    {
        EPL_MEMCPY(&pPostBuffer[dataSize], pEvent_p->m_pArg, pEvent_p->m_uiSize);
        // add optional argument data size
        dataSize += pEvent_p->m_uiSize;
    }

    hifRet = hostif_queueInsert(instance_l[eventQueue_p], pPostBuffer, dataSize);
    if(hifRet != kHostifSuccessful)
    {
        ret = kEplEventPostError;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Process event using host interface

This function reads a host interface event queue and processes the event
by calling the event handlers process function.

\param  eventQueue_p            Event queue used for reading the event.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_processEventHostif(tEventQueue eventQueue_p)
{
    tEplKernel          ret = kEplSuccessful;
    tHostifReturn       hifRet;
    tEplEvent*          pEplEvent;
    WORD                dataSize = sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE;
    DWORD               aRxBuffer[(sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE)/4];
    BYTE*               pRxBuffer = (BYTE*)aRxBuffer;

    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    hifRet = hostif_queueExtract(instance_l[eventQueue_p], pRxBuffer, &dataSize);
    if(hifRet != kHostifSuccessful)
    {
        eventk_postError(kEplEventSourceEventk, kEplEventReadError,
                         sizeof (hifRet), &hifRet);
        goto Exit;
    }

    pEplEvent = (tEplEvent *) pRxBuffer;
    pEplEvent->m_uiSize = (UINT)dataSize - sizeof(tEplEvent);
    if(pEplEvent->m_uiSize > 0)
        pEplEvent->m_pArg = &pRxBuffer[sizeof(tEplEvent)];
    else
        pEplEvent->m_pArg = NULL;

    ret = eventk_process(pEplEvent);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get number of active events

This function returns the number of events which are currently available in
the host interface event queue.

\param  eventQueue_p            Event queue to read the count from.

\return The function returns the number of active events.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
UINT eventkcal_getEventCountHostif(tEventQueue eventQueue_p)
{
    UINT16              count = 0;

    if (eventQueue_p > kEventQueueNum)
        return 0;

    if (instance_l[eventQueue_p] == NULL)
        return 0;

    hostif_queueGetEntryCount (instance_l[eventQueue_p], &count);

    return count;
}

//------------------------------------------------------------------------------
/**
\brief  Setup event signaling for host interface event queue

This function sets up event signaling for the specified host interface event
queue.

\param  eventQueue_p            Event queue to read the count from.
\param  pfnSignalCb_p           Pointer to signaling callback function.

\return The function returns the number of active events.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------

tEplKernel eventkcal_setSignalingHostif(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p)
{
    if (eventQueue_p > kEventQueueNum)
        return kEplInvalidInstanceParam;

    if (instance_l[eventQueue_p] == NULL)
        return kEplInvalidInstanceParam;

    //TODO jz Implement async/event signaling in host interface

    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Event handler callback function

This function implements the callback function which should be called when
receiving an event.

\param  pArg_p                  EventQueue this event was received.
*/
//------------------------------------------------------------------------------
static void rxSignalHandlerCb(void* pArg_p)
{
    eventkcal_processEventHostif((tEventQueue)pArg_p);
}

/// \}

