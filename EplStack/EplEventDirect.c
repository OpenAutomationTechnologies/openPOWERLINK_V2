/**
********************************************************************************
\file   EplEventDirect.c

\brief  source file for direct event posting

This event queue implementation applies direct calls, hence, an event posted
is processed in the same context.

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
#include "EplEventDirect.h"
#include "kernel/EplEventkCal.h"
#include "user/EplEventuCal.h"

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
\brief event direct instance type

The EventDirectInstance is used for every event queue using the direct call
event posting.
Note: The member fProcessThreadSafe determines if the event posting
is performed thread safe.
*/
typedef struct
{
    tEplEventQueue      EventQueue; ///< event queue
    tEplProcessEventCb  pfnProcessEventCb_m; ///< event process callback
    BOOL                fProcessThreadSafe; ///< thread-safe event process
} tEplEventDirectInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    direct event posting add instance

Add a queue instance that uses direct event posting.

\param  pEventQueue_p           pointer to event queue instance
\param  EventQueue_p            event queue to be add
\param  fProcessThreadSafe_p    queue should post events thread-safe

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventDirectAddInstance (tEplEventQueueInstance *ppEventQueue_p,
        tEplEventQueue EventQueue_p, BOOL fProcessThreadSafe_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventDirectInstance *pEventDirectInstance;

    pEventDirectInstance = (tEplEventDirectInstance*)
            EPL_MALLOC(sizeof(tEplEventDirectInstance));

    if(pEventDirectInstance == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    pEventDirectInstance->EventQueue = EventQueue_p;
    pEventDirectInstance->fProcessThreadSafe = fProcessThreadSafe_p;

    //set callback depending on event queue
    switch(pEventDirectInstance->EventQueue)
    {
        case kEplEventQueueK2U :
        case kEplEventQueueUInt :
            pEventDirectInstance->pfnProcessEventCb_m = EplEventuCalRxHandler;
            break;
        case kEplEventQueueKInt :
        case kEplEventQueueU2K :
            pEventDirectInstance->pfnProcessEventCb_m = EplEventkCalRxHandler;
            break;
        default:
            Ret = kEplInvalidInstanceParam;
            goto Exit;
            break;
    }

    *ppEventQueue_p = (tEplEventQueueInstance*)pEventDirectInstance;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    direct event posting delete instance

Delete direct posting queue instance.

\param  pEventQueue_p           pointer to event queue instance

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventDirectDelInstance (tEplEventQueueInstance pEventQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventDirectInstance *pEventDirectInstance =
            (tEplEventDirectInstance*)pEventQueue_p;

    //set callback NULL
    pEventDirectInstance->pfnProcessEventCb_m = NULL;

    //finally free the event instance
    EPL_FREE(pEventDirectInstance);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    direct event posting

This function posts an event to the provided queue instance. If enabled
the posting is done thread safe.

\param  pEventQueue_p           pointer to event queue instance
\param  pEventQueue_p           pointer to event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplEventDirectPost (tEplEventQueueInstance pEventQueue_p,
        tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventDirectInstance *pEventDirectInstance =
            (tEplEventDirectInstance*)pEventQueue_p;

    if(pEventDirectInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pEventDirectInstance->pfnProcessEventCb_m == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pEventDirectInstance->fProcessThreadSafe)
    {
        EplTgtEnableGlobalInterrupt(FALSE);
    }

    Ret = pEventDirectInstance->pfnProcessEventCb_m(pEvent_p);

    if(pEventDirectInstance->fProcessThreadSafe)
        {
            EplTgtEnableGlobalInterrupt(TRUE);
        }

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
