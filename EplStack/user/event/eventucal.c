/**
********************************************************************************
\file   eventucal.c

\brief  Source file for user event CAL module

The user event CAL module builds the interface between the user event
module and the different event queue implementations.

The user event CAL module produces events in the user-to-kernel (U2K) and
user-internal (UInt) queue. It consumes events from the user-internal (UInt)
and the kernel-to-user (K2U) queue.

For each queue a different implementation could be used. The event queue
instances of the used queues and the function interface are stored in the
CALs instance variable.

Which queue implementation is used is configured at compile time by the
following macros:
\li EPL_EVENT_K2U_QUEUE
\li EPL_EVENT_UINT_QUEUE
\li EPL_EVENT_U2K_QUEUE

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <user/eventu.h>
#include <user/eventucal.h>
#include <Benchmark.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Macros for function pointer interface
#define ADD_U2K_INSTANCE    instance_l.pUserToKernelFuncs->pfnAddInstance
#define DEL_U2K_INSTANCE    instance_l.pUserToKernelFuncs->pfnDelInstance
#define POST_U2K_EVENT      instance_l.pUserToKernelFuncs->pfnPostEvent
#define GET_U2K_QUEUE_TYPE  instance_l.pUserToKernelFuncs->pfnGetQueueType

#define ADD_UINT_INSTANCE   instance_l.pUserInternalFuncs->pfnAddInstance
#define DEL_UINT_INSTANCE   instance_l.pUserInternalFuncs->pfnDelInstance
#define POST_UINT_EVENT     instance_l.pUserInternalFuncs->pfnPostEvent
#define GET_UINT_QUEUE_TYPE instance_l.pUserInternalFuncs->pfnGetQueueType

#define ADD_K2U_INSTANCE    instance_l.pKernelToUserFuncs->pfnAddInstance
#define DEL_K2U_INSTANCE    instance_l.pKernelToUserFuncs->pfnDelInstance
#define GET_K2U_QUEUE_TYPE  instance_l.pKernelToUserFuncs->pfnGetQueueType

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
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    tEventQueueInstPtr      pU2KInstance;       ///< Pointer to event queue instance of user-to-kernel queue
    tEventQueueInstPtr      pK2UInstance;       ///< Pointer to event queue instance of kernel-to-user queue
    tEventQueueInstPtr      pUIntInstance;      ///< Pointer to event queue instance of user-internal queue
    tEventCalFuncIntf*      pUserToKernelFuncs; ///< Pointer to function interface for user-to-kernel queue
    tEventCalFuncIntf*      pUserInternalFuncs; ///< Pointer to function interface for user-internal queue
    tEventCalFuncIntf*      pKernelToUserFuncs; ///< Pointer to function interface for kernel-to-user queue
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event CAL module

The function initializes the user event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_init(void)
{
    tEplKernel          ret = kEplSuccessful;

    EPL_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    /* get function interface of the different queues */
    instance_l.pUserToKernelFuncs = GET_EVENTU_U2K_INTERFACE();
    instance_l.pUserInternalFuncs = GET_EVENTU_UINT_INTERFACE();
    instance_l.pKernelToUserFuncs = GET_EVENTU_K2U_INTERFACE();

    ret = ADD_U2K_INSTANCE(&instance_l.pU2KInstance, kEventQueueU2K);
    if (ret != kEplSuccessful)
        goto Exit;

    ret = ADD_UINT_INSTANCE(&instance_l.pUIntInstance, kEventQueueUInt);
    if (ret != kEplSuccessful)
        goto Exit;

    ret = ADD_K2U_INSTANCE(&instance_l.pK2UInstance, kEventQueueK2U);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup user event CAL module

The function cleans up the user event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_exit (void)
{
    DEL_U2K_INSTANCE(instance_l.pU2KInstance);

    DEL_UINT_INSTANCE(instance_l.pUIntInstance);

    DEL_K2U_INSTANCE(instance_l.pK2UInstance);

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_postEvent(tEplEvent *pEvent_p)
{
    tEplKernel ret = kEplSuccessful;

    BENCHMARK_MOD_28_SET(3);

    // split event post to user internal and user to kernel
    switch(pEvent_p->m_EventSink)
    {
        // kernel layer modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
            ret = POST_U2K_EVENT(instance_l.pU2KInstance, pEvent_p);
            break;

        // user layer modules
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
            ret = POST_UINT_EVENT(instance_l.pUIntInstance, pEvent_p);
            break;

        default:
            ret = kEplEventUnknownSink;
            break;

    }

    BENCHMARK_MOD_28_RESET(3);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event CAL receive handler

This is the event receive function for events posted to the user layer.

\param  pEvent_p                Received event to be processed.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventucal_rxHandler(tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;

    BENCHMARK_MOD_28_SET(5);

    Ret = eventu_process(pEvent_p);

    BENCHMARK_MOD_28_RESET(5);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
