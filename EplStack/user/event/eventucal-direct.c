/**
********************************************************************************
\file   eventucal-direct.c

\brief  source file for direct event posting

This event queue implementation applies direct calls, hence, an event posted
is processed in the same context.

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
#include <eventcal.h>
#include <user/eventucal.h>
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
static tEplKernel delInstance (tEventQueueInstPtr pEventQueueInst_p);
static tEplKernel postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p);

/* define external function interface */
static tEventCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    postEvent,
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
*/
//------------------------------------------------------------------------------
tEventCalFuncIntf* eventucaldirect_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief    Add instance of direct call event CAL in user layer

The function adds a new instance of the direct call event CAL interface at
the user layer.

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
    BOOL                    fProcessThreadSafe;
    tEplProcessEventCb      pfnProcessEventCb;

    //set parameters depending on event queue
    switch(eventQueue_p)
    {
        case kEventQueueUInt:
            pfnProcessEventCb = eventucal_rxHandler;
            fProcessThreadSafe = FALSE;
            break;

        case kEventQueueU2K:
            pfnProcessEventCb = eventkcal_rxHandler;
            fProcessThreadSafe = TRUE;
            break;

        case kEventQueueK2U:
            // we don't use this queue but calling init is ok,
            // therefore it is no error
            ret = kEplSuccessful;
            goto Exit;
            break;

        default:
            ret = kEplInvalidInstanceParam;
            goto Exit;
            break;
    }

    ret = eventcaldirect_addInstance(ppEventQueueInst_p, eventQueue_p,
                                     pfnProcessEventCb, fProcessThreadSafe);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delet event CAL instance

The function deletes an event CAL instance.

\param  pEventQueueInst_p           pointer to event queue instance

\return Returns always kEplSuccessful
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance (tEventQueueInstPtr pEventQueueInst_p)
{
    return eventcaldirect_delInstance(pEventQueueInst_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post event using direct calls

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
    return eventcaldirect_postEvent(pEventQueue_p, pEvent_p);
}
