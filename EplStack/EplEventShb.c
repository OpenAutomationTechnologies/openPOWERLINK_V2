/**
********************************************************************************
\file   EplEventShb.c

\brief  source file for shared buffer event posting

This event queue implementation applies the shared buffer for event forwarding.
The shared buffer is available for different architectures (e.g. NoOS).

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
#include "EplEventShb.h"
#include "kernel/EplEventkCal.h"
#include "user/EplEventuCal.h"

#include "SharedBuff.h"

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
\brief event shared buffer instance type

The EventShbInstance is used for every event queue using the shared buffer for
event posting. Every shared buffer queue instance has its own process and
error post call back. The RxBuffer is used to copy a received event out of the
shared buffer.
*/
typedef struct
{
    tEplEventQueue          EventQueue;
        ///< event queue
    tEplProcessEventCb      pfnProcessEventCb_m;
        ///< event process callback
    tEplPostErrorEventCb    pfnPostErrorEventCb_p;
        ///< post error event callback
    tShbInstance            pShbInstance_m;
        ///< shared buffer instance
    BYTE                    abRxBuffer_m[sizeof(tEplEvent) +
                                     EPL_MAX_EVENT_ARG_SIZE];
        ///< event receive buffer
} tEplEventShbInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void  EplEventRxSignalHandlerCb (tShbInstance pShbRxInstance_p,
        unsigned long ulDataSize_p, void *pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting add instance

Add a queue instance that uses shared buffer posting.

\param  ppEventQueue_p          double pointer to event queue instance
\param  EventQueue_p            event queue to be add
\param  pfnProcessEventCb_p     callback to event process

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventShbAddInstance (tEplEventQueueInstance *ppEventQueue_p,
        tEplEventQueue EventQueue_p,
        tEplProcessEventCb pfnProcessEventCb_p,
        tEplPostErrorEventCb pfnPostErrorEventCb_p)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplEventShbInstance *pEventShbInstance;
    unsigned int fShbNewCreated;

    pEventShbInstance = (tEplEventShbInstance*)
            EPL_MALLOC(sizeof(tEplEventShbInstance));

    if(pEventShbInstance == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pEventShbInstance->EventQueue = EventQueue_p;
    pEventShbInstance->pfnProcessEventCb_m = pfnProcessEventCb_p;
    pEventShbInstance->pfnPostErrorEventCb_p = pfnPostErrorEventCb_p;

    //initialize shared buffer
    switch(pEventShbInstance->EventQueue)
    {
        case kEplEventQueueK2U :
            ShbError = ShbCirAllocBuffer(
                    EPL_EVENT_SIZE_SHB_KERNEL_TO_USER,
                    EPL_EVENT_NAME_SHB_KERNEL_TO_USER,
                    &pEventShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        case kEplEventQueueUInt :
            ShbError = ShbCirAllocBuffer(
                    EPL_EVENT_SIZE_SHB_USER_INTERNAL,
                    EPL_EVENT_NAME_SHB_USER_INTERNAL,
                    &pEventShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        case kEplEventQueueKInt :
            ShbError = ShbCirAllocBuffer(
                    EPL_EVENT_SIZE_SHB_KERNEL_INTERNAL,
                    EPL_EVENT_NAME_SHB_KERNEL_INTERNAL,
                    &pEventShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        case kEplEventQueueU2K :
            ShbError = ShbCirAllocBuffer(
                    EPL_EVENT_SIZE_SHB_USER_TO_KERNEL,
                    EPL_EVENT_NAME_SHB_USER_TO_KERNEL,
                    &pEventShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        default:
            Ret = kEplInvalidInstanceParam;
            goto Exit;
            break;
    }

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    //register shared buffer callback
    if(pfnProcessEventCb_p != NULL)
    {
        ShbError = ShbCirSetSignalHandlerNewData(
                pEventShbInstance->pShbInstance_m,
                EplEventRxSignalHandlerCb, (void*) pEventShbInstance,
                kShbPriorityNormal);

        if(ShbError != kShbOk)
        {
            Ret = kEplNoResource;
            goto Exit;
        }
    }

    *ppEventQueue_p = (tEplEventShbInstance*)pEventShbInstance;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting delete instance

Delete shared buffer posting queue instance.

\param  pEventQueue_p           pointer to event queue instance

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventShbDelInstance (tEplEventQueueInstance pEventQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplEventShbInstance *pEventShbInstance =
            (tEplEventShbInstance*)pEventQueue_p;

    //set shared buffer callback to NULL
    if(pEventShbInstance->pfnProcessEventCb_m != NULL)
    {
        ShbError = ShbCirSetSignalHandlerNewData(
                pEventShbInstance->pShbInstance_m,
                NULL, NULL,
                kShbPriorityNormal);

        if(ShbError != kShbOk)
        {
            Ret = kEplNoResource;
            goto Exit;
        }
    }

    //free shared buffer
    ShbError = ShbCirReleaseBuffer(pEventShbInstance->pShbInstance_m);

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    //finally free the event instance
    EPL_FREE(pEventShbInstance);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting

This function posts an event to the provided queue instance.

\param  pEventQueue_p           pointer to event queue instance
\param  pEventQueue_p           pointer to event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventShbPost (tEplEventQueueInstance pEventQueue_p,
        tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventShbInstance *pEventShbInstance =
            (tEplEventShbInstance*)pEventQueue_p;

    tShbError ShbError;
    tShbCirChunk ShbCirChunk;
    unsigned long ulDataSize;
    unsigned int fBufferCompleted;

    if(pEventShbInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    // 2006/08/03 d.k.: Event and argument are posted as separate
    // chunks to the event queue.
    ulDataSize = sizeof(tEplEvent) + ((pEvent_p->m_pArg != NULL) ?
            pEvent_p->m_uiSize : 0);

    ShbError = ShbCirAllocDataBlock(pEventShbInstance->pShbInstance_m,
                           &ShbCirChunk,
                           ulDataSize);
    if(ShbError != kShbOk)
    {
        Ret = kEplEventPostError;
        goto Exit;
    }
    ShbError = ShbCirWriteDataChunk(pEventShbInstance->pShbInstance_m,
                           &ShbCirChunk,
                           pEvent_p,
                           sizeof (tEplEvent),
                           &fBufferCompleted);
    if(ShbError != kShbOk)
    {
        Ret = kEplEventPostError;
        goto Exit;
    }

    if (fBufferCompleted == FALSE)
    {
        ShbError = ShbCirWriteDataChunk(pEventShbInstance->pShbInstance_m,
                               &ShbCirChunk,
                               pEvent_p->m_pArg,
                               (unsigned long) pEvent_p->m_uiSize,
                               &fBufferCompleted);

        if ((ShbError != kShbOk) || (fBufferCompleted == FALSE))
        {
            Ret = kEplEventPostError;
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    shared buffer new data RX callback

This function is called by a shared buffer instance that has new data. The
data is copied from the shared buffer to the queue instance RX buffer.

\param  pShbRxInstance_p        pointer to receiving shared buffer instance
\param  ulDataSize_p            size of received data
\param  pArg_p                  argument of receive function (EventShbInstance)

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static void EplEventRxSignalHandlerCb (tShbInstance pShbRxInstance_p,
        unsigned long ulDataSize_p, void *pArg_p)
{
    tShbError ShbError;
    tEplEventShbInstance *pEventShbInstance = (tEplEventShbInstance*)pArg_p;

    tEplEvent *pEplEvent;
    BYTE* pabDataBuffer = pEventShbInstance->abRxBuffer_m;

    // copy data from event queue
    ShbError = ShbCirReadDataBlock (pShbRxInstance_p,
                            pabDataBuffer,
                            sizeof(pEventShbInstance->abRxBuffer_m),
                            &ulDataSize_p);

    if(ShbError != kShbOk)
    {
        //post error
        if(pEventShbInstance->pfnPostErrorEventCb_p != NULL)
        {
            pEventShbInstance->pfnPostErrorEventCb_p(
                    kEplEventSourceEventk, kEplEventReadError,
                    sizeof (ShbError), &ShbError);
        }
        //note: If callback is invalid, no error is posted!

        goto Exit;
    }

    // resolve the pointer to the event structure
    pEplEvent = (tEplEvent *) pabDataBuffer;
    // set Datasize
    pEplEvent->m_uiSize = (ulDataSize_p - sizeof(tEplEvent));
    if(pEplEvent->m_uiSize > 0)
    {
        // set pointer to argument
        pEplEvent->m_pArg = &pabDataBuffer[sizeof(tEplEvent)];
    }
    else
    {
        //set pointer to NULL
        pEplEvent->m_pArg = NULL;
    }

    pEventShbInstance->pfnProcessEventCb_m(pEplEvent);

Exit:
    return;
}
