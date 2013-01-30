/**
********************************************************************************
\file   eventcal-shb.c

\brief  Shared buffer implementation of event CAL module

This event queue implementation applies the shared buffer for event forwarding.
The shared buffer is available for different architectures (e.g. NoOS).

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

#include <SharedBuff.h>

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
// name and size of event queues
#define EPL_EVENT_NAME_SHB_KERNEL_TO_USER   "ShbKernelToUser"
#ifndef EPL_EVENT_SIZE_SHB_KERNEL_TO_USER
#define EPL_EVENT_SIZE_SHB_KERNEL_TO_USER   32768   // 32 kByte
#endif

#define EPL_EVENT_NAME_SHB_USER_TO_KERNEL   "ShbUserToKernel"
#ifndef EPL_EVENT_SIZE_SHB_USER_TO_KERNEL
#define EPL_EVENT_SIZE_SHB_USER_TO_KERNEL   32768   // 32 kByte
#endif

#define EPL_EVENT_NAME_SHB_KERNEL_INTERNAL  "ShbKernelInternal"
#ifndef EPL_EVENT_SIZE_SHB_KERNEL_INTERNAL
#define EPL_EVENT_SIZE_SHB_KERNEL_INTERNAL  32768   // 32 kByte
#endif

#define EPL_EVENT_NAME_SHB_USER_INTERNAL    "ShbUserInternal"
#ifndef EPL_EVENT_SIZE_SHB_USER_INTERNAL
#define EPL_EVENT_SIZE_SHB_USER_INTERNAL    32768   // 32 kByte
#endif

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
    tEventQueue             eventQueue;                 ///< Event queue
    tEplProcessEventCb      pfnProcessEventCb;          ///< Event process callback
    tEplPostErrorEventCb    pfnPostErrorEventCb;        ///< Post error event callback
    tShbInstance            pShbInstance;               ///< Shared buffer instance
    BYTE                    abRxBuffer[sizeof(tEplEvent) +
                                     EPL_MAX_EVENT_ARG_SIZE]; ///< Event receive buffer
} tEventShbInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void  rxSignalHandlerCb (tShbInstance pShbRxInstance_p, ULONG dataSize_p, void *pArg_p);

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

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalshb_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                                   tEventQueue eventQueue_p,
                                   tEplProcessEventCb pfnProcessEventCb,
                                   tEplPostErrorEventCb pfnPostErrorEventCb)
{
    tEplKernel          ret = kEplSuccessful;
    tShbError           shbError;
    tEventShbInstance*  pInstance;
    UINT                fShbNewCreated;

    pInstance = (tEventShbInstance*)EPL_MALLOC(sizeof(tEventShbInstance));
    if(pInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pInstance->eventQueue = eventQueue_p;
    pInstance->pfnProcessEventCb = pfnProcessEventCb;
    pInstance->pfnPostErrorEventCb = pfnPostErrorEventCb;

    //initialize shared buffer
    switch(pInstance->eventQueue)
    {
        case kEventQueueK2U:
            shbError = ShbCirAllocBuffer(EPL_EVENT_SIZE_SHB_KERNEL_TO_USER,
                                         EPL_EVENT_NAME_SHB_KERNEL_TO_USER,
                                         &pInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        case kEventQueueUInt:
            shbError = ShbCirAllocBuffer(EPL_EVENT_SIZE_SHB_USER_INTERNAL,
                                         EPL_EVENT_NAME_SHB_USER_INTERNAL,
                                         &pInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        case kEventQueueKInt:
            shbError = ShbCirAllocBuffer(EPL_EVENT_SIZE_SHB_KERNEL_INTERNAL,
                                         EPL_EVENT_NAME_SHB_KERNEL_INTERNAL,
                                         &pInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        case kEventQueueU2K:
            shbError = ShbCirAllocBuffer(EPL_EVENT_SIZE_SHB_USER_TO_KERNEL,
                                         EPL_EVENT_NAME_SHB_USER_TO_KERNEL,
                                         &pInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        default:
            ret = kEplInvalidInstanceParam;
            goto Exit;
            break;
    }

    if(shbError != kShbOk)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //register shared buffer callback
    if(pInstance->pfnProcessEventCb != NULL)
    {
        shbError = ShbCirSetSignalHandlerNewData(pInstance->pShbInstance,
                                                 rxSignalHandlerCb, (void*) pInstance,
                                                 kShbPriorityNormal);
        if(shbError != kShbOk)
        {
            ret = kEplNoResource;
            goto Exit;
        }
    }

    *ppEventQueueInst_p = (tEventShbInstance*)pInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting delete instance

Delete shared buffer posting queue instance.

\param  pEventQueue_p           pointer to event queue instance

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalshb_delInstance (tEventQueueInstPtr pEventQueueInst_p)
{
    tEplKernel          ret = kEplSuccessful;
    tShbError           shbError;
    tEventShbInstance*  pEventShbInstance = (tEventShbInstance*)pEventQueueInst_p;

    if (pEventShbInstance == NULL)
        return kEplSuccessful;

    //set shared buffer callback to NULL
    if(pEventShbInstance->pfnProcessEventCb != NULL)
    {
        shbError = ShbCirSetSignalHandlerNewData(pEventShbInstance->pShbInstance,
                                                 NULL, NULL, kShbPriorityNormal);
        if(shbError != kShbOk)
        {
            TRACE ("%s() ShbCirSetSignalHandlerNewData() failed with %d\n",
                   __func__, shbError);
        }
    }

    //free shared buffer
    shbError = ShbCirReleaseBuffer(pEventShbInstance->pShbInstance);
    if(shbError != kShbOk)
    {
        TRACE ("%s() ShbCirReleaseBuffer() failed with %d\n", __func__, shbError);
    }

    //finally free the event instance
    EPL_FREE(pEventShbInstance);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting

This function posts an event to the provided queue instance.

\param  pEventQueue_p           pointer to event queue instance
\param  pEventQueue_p           pointer to event

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcalshb_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEventShbInstance*  pEventShbInstance = (tEventShbInstance*)pEventQueue_p;
    tShbError           shbError;
    tShbCirChunk        shbCirChunk;
    ULONG               dataSize;
    UINT                fBufferCompleted;

    if(pEventShbInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    // 2006/08/03 d.k.: Event and argument are posted as separate
    // chunks to the event queue.
    dataSize = sizeof(tEplEvent) + ((pEvent_p->m_pArg != NULL) ? pEvent_p->m_uiSize : 0);

    shbError = ShbCirAllocDataBlock(pEventShbInstance->pShbInstance, &shbCirChunk,
                                    dataSize);
    if(shbError != kShbOk)
    {
        ret = kEplEventPostError;
        goto Exit;
    }
    shbError = ShbCirWriteDataChunk(pEventShbInstance->pShbInstance, &shbCirChunk,
                                    pEvent_p, sizeof (tEplEvent), &fBufferCompleted);
    if(shbError != kShbOk)
    {
        ret = kEplEventPostError;
        goto Exit;
    }

    if (fBufferCompleted == FALSE)
    {
        shbError = ShbCirWriteDataChunk(pEventShbInstance->pShbInstance, &shbCirChunk,
                                        pEvent_p->m_pArg, (ULONG)pEvent_p->m_uiSize,
                                        &fBufferCompleted);
        if ((shbError != kShbOk) || (fBufferCompleted == FALSE))
        {
            ret = kEplEventPostError;
            goto Exit;
        }
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
\brief    shared buffer new data RX callback

This function is called by a shared buffer instance that has new data. The
data is copied from the shared buffer to the queue instance RX buffer.

\param  pShbRxInstance_p        pointer to receiving shared buffer instance
\param  ulDataSize_p            size of received data
\param  pArg_p                  argument of receive function (EventShbInstance)

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static void rxSignalHandlerCb (tShbInstance pShbRxInstance_p, ULONG dataSize_p,
                               void *pArg_p)
{
    tShbError           shbError;
    tEventShbInstance*  pEventShbInstance = (tEventShbInstance*)pArg_p;
    tEplEvent*          pEplEvent;
    BYTE*               pDataBuffer;

    if (pArg_p == NULL)
        return;

    pDataBuffer = pEventShbInstance->abRxBuffer;

    // copy data from event queue
    shbError = ShbCirReadDataBlock (pShbRxInstance_p, pDataBuffer,
                                    sizeof(pEventShbInstance->abRxBuffer),
                                    &dataSize_p);
    if(shbError != kShbOk)
    {
        //post error
        if(pEventShbInstance->pfnPostErrorEventCb != NULL)
        {
            pEventShbInstance->pfnPostErrorEventCb(
                    kEplEventSourceEventk, kEplEventReadError,
                    sizeof (shbError), &shbError);
        }
        //note: If callback is invalid, no error is posted!

        goto Exit;
    }

    // resolve the pointer to the event structure
    pEplEvent = (tEplEvent *) pDataBuffer;
    // set Datasize
    pEplEvent->m_uiSize = (dataSize_p - sizeof(tEplEvent));
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

    pEventShbInstance->pfnProcessEventCb(pEplEvent);

Exit:
    return;
}
/// \}
