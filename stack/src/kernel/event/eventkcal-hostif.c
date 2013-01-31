/**
********************************************************************************
\file   eventkcal-hostif.c

\brief  source file for shared buffer event posting

This event queue implementation applies the shared buffer for event forwarding.

The only public function provided is eventkcalshb_getInterface(). This
function returns a set of function pointers which is provided to use this CAL
implementation from eventkcal.c.

The real functionality of the shb implementation is separated in
the common part to be used by both user and kernel layer modules.

\ingroup module_eventkcal
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
#include <eventcal.h>
#include <kernel/eventkcal.h>

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

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEventCalFuncIntf* eventkcalhostif_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Add instance of shared buffer kernel event CAL

The function adds a event queue CAL instance that uses shared buffer posting.

\param  ppEventQueueInst_p      Pointer to store event queue instance reference
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
            pfnProcessEventCb = NULL;
            pfnPostErrorEventCb = NULL;
            break;

        case kEventQueueU2K:
            pfnProcessEventCb = eventkcal_postEvent;
            pfnPostErrorEventCb = eventk_postError;
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
\brief    Delete instance of shared buffer kernel event CAL

Delete shared buffer posting queue instance.

\param  pEventQueue_p           pointer to event queue instance

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance (tEventQueueInstPtr pEventQueue_p)
{
    return eventcalhostif_delInstance(pEventQueue_p);
}

//------------------------------------------------------------------------------
/**
\brief    shared buffer event posting

This function posts an event to the provided queue instance.

\param  pEventQueue_p           Pointer to event queue instance
\param  pEvent_p                Pointer to event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p)
{
    return eventcalhostif_postEvent(pEventQueue_p, pEvent_p);
}
/// \}
