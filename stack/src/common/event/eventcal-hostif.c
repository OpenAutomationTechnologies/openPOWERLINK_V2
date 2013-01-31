/**
********************************************************************************
\file   eventcal-hostif.c

\brief  Host Interface implementation of event CAL module

This event queue implementation applies the host interface for event forwarding.

\ingroup module_eventcal
*******************************************************************************/

/*------------------------------------------------------------------------------
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
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EplInc.h>

#include <eventcal.h>

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
/**
\brief event host interface instance type

The EventHifInstance is used for every event queue using the host interface for
event posting. Every host interface queue instance has its own process and
error post call back. The RxBuffer is used to copy a received event out of the
host interface.
*/
typedef struct
{
    tEventQueue             eventQueue;                 ///< Event queue
    tEplProcessEventCb      pfnProcessEventCb;          ///< Event process callback
    tEplPostErrorEventCb    pfnPostErrorEventCb;        ///< Post error event callback
    tHostifQueueInstance    pQueueInstance;             ///< host interface queue
    BYTE                    abRxBuffer[sizeof(tEplEvent) +
                                     EPL_MAX_EVENT_ARG_SIZE]; ///< Event receive buffer
} tEventHifInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void rxSignalHandlerCb (void *pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    host interface event posting add instance

Add a queue instance that uses host interface posting.

\param  ppEventQueueInst_p      double pointer to event queue instance
\param  eventQueue_p            event queue to be add
\param  pfnProcessEventCb       callback to event process
\param  pfnPostErrorEventCb     callback to event error post

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalhostif_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                                   tEventQueue eventQueue_p,
                                   tEplProcessEventCb pfnProcessEventCb,
                                   tEplPostErrorEventCb pfnPostErrorEventCb)
{
    tEplKernel          ret = kEplSuccessful;
    tHostifReturn       hifRet;
    tEventHifInstance*  pInstance;
    tHostifInstance     pHifInstance;
    tHostifInstanceId   hifInstanceId;

    // get host interface instance
    //FIXME: Find other way to get the host interface instance (omit dependency)
#ifdef CONFIG_INCLUDE_DLLU
    pHifInstance = hostif_getInstance(kHostifProcHost);
#elif defined(CONFIG_INCLUDE_DLLK)
    pHifInstance = hostif_getInstance(kHostifProcPcp);
#else
    pHifInstance = NULL;
#endif

    if(pHifInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    pInstance = (tEventHifInstance*)EPL_MALLOC(sizeof(tEventHifInstance));
    if(pInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pInstance->eventQueue = eventQueue_p;
    pInstance->pfnProcessEventCb = pfnProcessEventCb;
    pInstance->pfnPostErrorEventCb = pfnPostErrorEventCb;

    //initialize host interface
    switch(pInstance->eventQueue)
    {
        case kEventQueueK2U:
            hifInstanceId = kHostifInstIdK2UQueue;
            break;

        case kEventQueueU2K:
            hifInstanceId = kHostifInstIdU2KQueue;
            break;

        default:
            ret = kEplInvalidInstanceParam;
            goto Exit;
    }

    hifRet = hostif_queueCreate(pHifInstance, hifInstanceId, &pInstance->pQueueInstance);

    if(hifRet != kHostifSuccessful)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //register host interface callback
    if(pInstance->pfnProcessEventCb != NULL)
    {
        hifRet = hostif_queueCallback(pInstance->pQueueInstance, rxSignalHandlerCb, (void*)pInstance);
        if(hifRet != kHostifSuccessful)
        {
            ret = kEplNoResource;
            goto Exit;
        }
    }

    *ppEventQueueInst_p = (tEventHifInstance*)pInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    host interface event posting delete instance

Delete host interface posting queue instance.

\param  pEventQueueInst_p       pointer to event queue instance

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalhostif_delInstance (tEventQueueInstPtr pEventQueueInst_p)
{
    tEplKernel          ret = kEplSuccessful;
    tHostifReturn       hifRet;
    tEventHifInstance*  pEventInstance = (tEventHifInstance*)pEventQueueInst_p;

    if (pEventInstance == NULL)
        return kEplSuccessful;

    //set host interface callback to NULL
    if(pEventInstance->pfnProcessEventCb != NULL)
    {
        hifRet = hostif_queueCallback(pEventInstance->pQueueInstance, NULL, NULL);
        if(hifRet != kHostifSuccessful)
        {
            TRACE ("%s() hostifQueueCallback() failed with %d\n",
                   __func__, hifRet);
        }
    }

    //free host interface
    hifRet = hostif_queueDelete(pEventInstance->pQueueInstance);
    if(hifRet != kHostifSuccessful)
    {
        TRACE ("%s() hostifQueueDelete() failed with %d\n", __func__, hifRet);
    }

    //finally free the event instance
    EPL_FREE(pEventInstance);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    host interface event posting

This function posts an event to the provided queue instance.

\param  pEventQueue_p           pointer to event queue instance
\param  pEvent_p                pointer to event

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalhostif_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEventHifInstance*  pEventInstance = (tEventHifInstance*)pEventQueue_p;
    tHostifReturn       hifRet;
    BYTE aPostBuffer[sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE];
    ULONG               dataSize;

    if(pEventInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    // initialize data size to mandatory part
    dataSize = sizeof(tEplEvent);

    // copy event into post buffer
    EPL_MEMCPY(aPostBuffer, pEvent_p, dataSize);

    // copy argument data if present
    if(pEvent_p->m_pArg != NULL)
    {
        EPL_MEMCPY(&aPostBuffer[dataSize], pEvent_p->m_pArg, pEvent_p->m_uiSize);

        // add optional argument data size
        dataSize += pEvent_p->m_uiSize;
    }

    hifRet = hostif_queueInsert(pEventInstance->pQueueInstance, aPostBuffer, dataSize);
    if(hifRet != kHostifSuccessful)
    {
        ret = kEplEventPostError;
        goto Exit;
    }

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    host interface new data RX callback

This function is called by a host interface instance that has new data. The
data is copied from the host interface to the queue instance RX buffer.

\param  pArg_p                  argument of receive function (EventHifInstance)

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static void rxSignalHandlerCb (void *pArg_p)
{
    tHostifReturn       hifRet;
    tEventHifInstance*  pEventInstance = (tEventHifInstance*)pArg_p;
    tEplEvent*          pEplEvent;
    BYTE*               pDataBuffer;
    WORD                dataSize = sizeof(pEventInstance->abRxBuffer);

    if (pArg_p == NULL)
        return;

    pDataBuffer = pEventInstance->abRxBuffer;

    // copy data from event queue
    hifRet = hostif_queueExtract(pEventInstance->pQueueInstance, pDataBuffer, &dataSize);
    if(hifRet != kHostifSuccessful)
    {
        //post error
        if(pEventInstance->pfnPostErrorEventCb != NULL)
        {
            pEventInstance->pfnPostErrorEventCb(
                    kEplEventSourceEventk, kEplEventReadError,
                    sizeof (hifRet), &hifRet);
        }
        //note: If callback is invalid, no error is posted!

        goto Exit;
    }

    // resolve the pointer to the event structure
    pEplEvent = (tEplEvent *) pDataBuffer;
    // set Datasize
    pEplEvent->m_uiSize = (UINT)dataSize - sizeof(tEplEvent);
    if(pEplEvent->m_uiSize > 0)
    {
        // set pointer to argument
        pEplEvent->m_pArg = &pDataBuffer[sizeof(tEplEvent)];
    }
    else
    {
        //set pointer to NULL
        pEplEvent->m_pArg = NULL;
    }

    pEventInstance->pfnProcessEventCb(pEplEvent);

Exit:
    return;
}
/// \}
