/**
********************************************************************************
\file   eventkcal.c

\brief  Source file for kernel event CAL module

The kernel event CAL module builds the interface between the kernel event
module and the different event queue implementations.

The kernel event CAL module produces events in the kernel-to-user (K2U) and
kernel-internal (KInt) queue. It consumes events from the kernel-internal (KInt)
and the user-to-kernel (U2K) queue.

For each queue a different implementation could be used. The event queue
instances of the used queues and the function interface are stored in the
CALs instance variable.

Which queue implementation is used is configured at compile time by the
following macros:
\li EPL_EVENT_K2U_QUEUE
\li EPL_EVENT_KINT_QUEUE
\li EPL_EVENT_U2K_QUEUE
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
#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
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

#define ADD_KINT_INSTANCE   instance_l.pKernelInternalFuncs->pfnAddInstance
#define DEL_KINT_INSTANCE   instance_l.pKernelInternalFuncs->pfnDelInstance
#define POST_KINT_EVENT     instance_l.pKernelInternalFuncs->pfnPostEvent

#define ADD_K2U_INSTANCE    instance_l.pKernelToUserFuncs->pfnAddInstance
#define DEL_K2U_INSTANCE    instance_l.pKernelToUserFuncs->pfnDelInstance
#define POST_K2U_EVENT      instance_l.pKernelToUserFuncs->pfnPostEvent

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
    tEventQueueInstPtr      pK2UInstance;           ///< Pointer to event queue instance of kernel to user queue
    tEventQueueInstPtr      pU2KInstance;           ///< Pointer to event queue instance of user to kernel queue
    tEventQueueInstPtr      pKIntInstance;          ///< Pointer to event queue instance of kernel internal queue
    tEventCalFuncIntf*      pUserToKernelFuncs;     ///< Pointer to function interface for user-to-kernel queue
    tEventCalFuncIntf*      pKernelInternalFuncs;   ///< Pointer to function interface for kernel-internal queue
    tEventCalFuncIntf*      pKernelToUserFuncs;     ///< Pointer to function interface for kernel-to-user queue
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_init (void)
{
    tEplKernel      ret = kEplSuccessful;

    EPL_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    /* get function interface of the different queues */
    instance_l.pUserToKernelFuncs = GET_EVENTK_U2K_INTERFACE();
    instance_l.pKernelInternalFuncs = GET_EVENTK_KINT_INTERFACE();
    instance_l.pKernelToUserFuncs = GET_EVENTK_K2U_INTERFACE();

    ret = ADD_K2U_INSTANCE(&instance_l.pK2UInstance, kEventQueueK2U);
    if(ret  != kEplSuccessful)
    {
        goto Exit;
    }

    ret = ADD_KINT_INSTANCE(&instance_l.pKIntInstance, kEventQueueKInt);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    ret = ADD_U2K_INSTANCE(&instance_l.pU2KInstance, kEventQueueU2K);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_exit (void)
{
    DEL_K2U_INSTANCE(instance_l.pK2UInstance);
    instance_l.pK2UInstance = NULL;

    DEL_KINT_INSTANCE(instance_l.pKIntInstance);
    instance_l.pKIntInstance = NULL;

    DEL_U2K_INSTANCE(instance_l.pU2KInstance);
    instance_l.pU2KInstance = NULL;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    BENCHMARK_MOD_27_SET(2);

    // split event post to user internal and user to kernel
    switch(pEvent_p->m_EventSink)
    {
        // user layer modules
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
            ret = POST_K2U_EVENT(instance_l.pK2UInstance, pEvent_p);
            break;

        // kernel layer modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
            ret = POST_KINT_EVENT(instance_l.pKIntInstance, pEvent_p);
            break;

        default:
            ret = kEplEventUnknownSink;
            break;
    }

    BENCHMARK_MOD_27_RESET(2);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event CAL receive handler

This is the event receive function for events posted to the kernel layer.

\param  pEvent_p                Received event to be processed.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_rxHandler (tEplEvent *pEvent_p)
{
    tEplKernel ret = kEplSuccessful;

    BENCHMARK_MOD_27_SET(4);

    ret = eventk_process(pEvent_p);

    BENCHMARK_MOD_27_RESET(4);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

