/**
********************************************************************************
\file   eventkcal-nooshostif.c

\brief  Kernel event CAL module for non-OS platform using the host-interface

This file implements the kernel event handler CAL module for a non-OS
platform. It uses the host interface library for the kernel-to-user /
user-to-kernel event queues and direct calls for the kernel-internal queue.

\see eventkcalintf-hostif.c

\ingroup module_eventkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <common/oplkinc.h>
#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>
#include <kernel/eventk.h>
#include <common/target.h>

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
\brief Kernel event CAL instance type

The structure contains all necessary information needed by the kernel event
CAL module.
*/
typedef struct
{
    BOOL                    fInitialized;
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module
static UINT8                aRxBuffer_l[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static BOOL checkForwardEventToKint(const tEvent* pEvent_p);
static BOOL eventSinkIsKernel(tEventSink eventSink_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kErrorOk)
        goto Exit;

    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    eventkcal_exitQueueCircbuf(kEventQueueKInt);
    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);

    return kErrorNoResource;
}


//------------------------------------------------------------------------------
/**
\brief    Clean up kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_exit(void)
{
    if (instance_l.fInitialized != FALSE)
    {
        eventkcal_exitQueueCircbuf(kEventQueueKInt);
        eventkcal_exitQueueCircbuf(kEventQueueK2U);
        eventkcal_exitQueueCircbuf(kEventQueueU2K);
    }
    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to the kernel queue.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postKernelEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    if (target_getInterruptContextFlag() && checkForwardEventToKint(pEvent_p))
    {
        // CPU is in interrupt context and forward to kernel-internal queue
        ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);
    }
    else
    {
        // CPU is not in interrupt context and kernel-internal queue post is not
        // needed -> direct processing!
        ret = eventk_process(pEvent_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to the user queue.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postUserEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    if (target_getInterruptContextFlag())
    {
        // CPU is in interrupt context
        // -> Post to kernel-internal queue first
        ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);
    }
    else
    {
        // CPU is outside of any interrupt context (background loop)
        // -> Post to kernel-to-user queue
        ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel CAL module

This function will be called by the systems process function.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
void eventkcal_process(void)
{
    tOplkError  ret;
    tEvent*     pEvent;
    size_t      readSize = sizeof(aRxBuffer_l);

    if (instance_l.fInitialized == FALSE)
        return;

    if (eventkcal_getEventCountCircbuf(kEventQueueKInt) > 0)
    {
        ret = eventkcal_getEventCircbuf(kEventQueueKInt, aRxBuffer_l, &readSize);
        if (ret == kErrorOk)
        {
            pEvent = (tEvent*)aRxBuffer_l;
            pEvent->eventArgSize = (readSize - sizeof(tEvent));

            if (pEvent->eventArgSize > 0)
                pEvent->eventArg.pEventArg = &aRxBuffer_l[sizeof(tEvent)];
            else
                pEvent->eventArg.pEventArg = NULL;

            if (!eventSinkIsKernel(pEvent->eventSink))
            {
                // Events from the kernel-internal queue to the user layer
                // have to be posted to the kernel-to-user queue finally.
                ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent);
                if (ret != kErrorOk)
                {
                    tEventQueue eventQueue = kEventQueueK2U;
                    // Forward error to API
                    eventk_postError(kEventSourceEventk,
                                     ret,
                                     sizeof(eventQueue),
                                     &eventQueue);
                }
            }
            else
            {
                // Events from the kernel-internal queue to kernel event sinks
                // can be processed directly.
                ret = eventk_process(pEvent);
            }
        }
    }
    else
    {
        if (eventkcal_getEventCountCircbuf(kEventQueueU2K) > 0)
        {
            ret = eventkcal_processEventCircbuf(kEventQueueU2K);
        }
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Check forward event to kernel internal queue

This function checks if the given event shall be forwarded by kernel internal
queue.

\param[in]      pEvent_p            Pointer to event

\return The function returns a BOOL.
\retval TRUE                        The event shall be forwarded by kernel internal queue.
\retval FALSE                       The event shall be forwarded by direct call.
*/
//------------------------------------------------------------------------------
static BOOL checkForwardEventToKint(const tEvent* pEvent_p)
{
    switch (pEvent_p->eventType)
    {
        case kEventTypePdoRx:
        case kEventTypeTimesynckControl:
        case kEventTypeDllkCycleFinish:
        case kEventTypeSync:
            return FALSE;

        default:
            return TRUE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Check if the given event sink is in kernel layer

This function checks if the given event sink lives in the kernel layer.

\param[in]      eventSink_p         Event sink of interest

\return The function returns a BOOL.
\retval TRUE                        The event sink is in kernel layer.
\retval FALSE                       The event sink is NOT in kernel layer.
*/
//------------------------------------------------------------------------------
static BOOL eventSinkIsKernel(tEventSink eventSink_p)
{
    switch (eventSink_p)
    {
        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            return TRUE;

        default:
            return FALSE;
    }
}

/// \}
