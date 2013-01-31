/**
********************************************************************************
\file   eventucal-hostif.c

\brief  source file for user event CAL shared buffer implementation

This file contains the source of the user event CAL shared buffer
implementation.

\ingroup module_eventucal
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
------------------------------------------------------------------------------*/


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EplInc.h>
#include <user/eventucal.h>

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
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p);
static tEplKernel delInstance (tEventQueueInstPtr pEventQueue_p);
static tEplKernel postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p);

/* define external function interface */
static tEventCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    postEvent
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return pointer to function interface

This function returns a pointer to the function interface structure which
is used to access the dllcal functions of the direct call implementation.

\return Returns a pointer to the local function interface

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tEventCalFuncIntf* eventucalhostif_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Add instance of shared buffer user event CAL

The function adsd a event queue CAL instance that uses shared buffer posting.

\param  ppEventQueueInst_p      Pointer to store event queue instance
\param  eventQueue_p            Event queue of this instance

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
static tEplKernel addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplProcessEventCb      pfnProcessEventCb;
    tEplPostErrorEventCb    pfnPostErrorEventCb;

    switch(eventQueue_p)
    {
        case kEventQueueK2U:
            pfnProcessEventCb = eventucal_postEvent;
            pfnPostErrorEventCb = eventu_postError;
            break;

        case kEventQueueU2K:
            pfnProcessEventCb = NULL;
            pfnPostErrorEventCb = NULL;
            break;

        default:
            ret = kEplInvalidInstanceParam;
            goto Exit;
            break;
    }

    eventcalhostif_addInstance (ppEventQueueInst_p, eventQueue_p,
                             pfnProcessEventCb, pfnPostErrorEventCb);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete instance of shared buffer user event CAL

The function deletes a shared buffer ueser event CAL instance.

\param  pEventQueueInst_p       Pointer to event queue instance

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance (tEventQueueInstPtr pEventQueueInst_p)
{
    return eventcalhostif_delInstance(pEventQueueInst_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post event using shared buffers

This function posts an event to the provided queue instance.

\param  pEventQueue_p           Pointer to event queue instance
\param  pEvent_p                Pointer to event

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
static tEplKernel postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p)
{
    return eventcalhostif_postEvent(pEventQueue_p, pEvent_p);
}
