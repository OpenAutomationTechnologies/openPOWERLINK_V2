/**
********************************************************************************
\file   eventcal-direct.c

\brief  Direct call implementation of event CAL module

This event queue implementation applies direct calls, hence, an event posted
is processed in the same context.

\ingroup module_eventcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved

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
#include <eventcal.h>

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
    tEventQueue         eventQueue;         ///< event queue
    tEplProcessEventCb  pfnProcessEventCb;  ///< event process callback
    BOOL                fProcessThreadSafe; ///< thread-safe event process
} tEventDirectInstance;

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
\brief    Add a direct call event CAL instance

The function adds a direct call event CAL instance.

\param  ppEventQueueInst_p      Pointer to store event queue instance
\param  eventQueue_p            Event queue to be add
\param  pfnProcessEventCb_p     Pointer to process callback function
\param  fProcessThreadSafe_p    Queue should post events thread-safe

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcaldirect_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p,
                              tEplProcessEventCb pfnProcessEventCb_p,
                              BOOL fProcessThreadSafe_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEventDirectInstance*   pInstance;

    pInstance = (tEventDirectInstance*) EPL_MALLOC(sizeof(tEventDirectInstance));
    if(pInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    pInstance->eventQueue = eventQueue_p;
    pInstance->pfnProcessEventCb = pfnProcessEventCb_p;
    pInstance->fProcessThreadSafe = fProcessThreadSafe_p;

    *ppEventQueueInst_p = (tEventQueueInstPtr*)pInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a direct call event CAL instance

The function deletes an direct call event CAL instance.

\param  pEventQueueInst_p           pointer to event queue instance

\return Returns always kEplSuccessful

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcaldirect_delInstance (tEventQueueInstPtr pEventQueueInst_p)
{
    tEventDirectInstance*   pInstance = (tEventDirectInstance*)pEventQueueInst_p;

    if (pInstance == NULL)
        return kEplSuccessful;

    //set callback NULL
    pInstance->pfnProcessEventCb = NULL;

    //finally free the event instance
    EPL_FREE(pInstance);

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post event

This function posts an event to the provided queue instance. If enabled
the posting is done thread safe.

\param  pEventQueue_p           Pointer to event queue instance
\param  pEvent_p                Pointer to event

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventcal
*/
//------------------------------------------------------------------------------
tEplKernel eventcaldirect_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEventDirectInstance *pInstance = (tEventDirectInstance*)pEventQueue_p;

    if(pInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pInstance->pfnProcessEventCb == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pInstance->fProcessThreadSafe)
    {
        EplTgtEnableGlobalInterrupt(FALSE);
    }

    ret = pInstance->pfnProcessEventCb(pEvent_p);

    if(pInstance->fProcessThreadSafe)
        {
            EplTgtEnableGlobalInterrupt(TRUE);
        }

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

