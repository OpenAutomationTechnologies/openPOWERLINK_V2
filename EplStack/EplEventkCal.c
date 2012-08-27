/**
********************************************************************************
\file   EplEventkCal.c

\brief  source file for Epl-Kernelspace-Event-Cal-Module

The kernel event CAL builds the interface between the kernel event and the
event queue implementations. It determines the event forwarding (e.g. kernel
event posted in user layer is forwarded by user-to-kernel queue)
Note that the defines EPL_EVENT_K2U_QUEUE, EPL_EVENT_KINT_QUEUE and
EPL_EVENT_U2K_QUEUE determine the kernel event implementation!

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
#include "kernel/EplEventk.h"
#include "kernel/EplEventkCal.h"
#include "Benchmark.h"

#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_DIRECT || \
    EPL_EVENT_KINT_QUEUE == EPL_QUEUE_DIRECT || \
    EPL_EVENT_U2K_QUEUE == EPL_QUEUE_DIRECT
#include "EplEventDirect.h"
#endif
#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_SHB || \
    EPL_EVENT_KINT_QUEUE == EPL_QUEUE_SHB || \
    EPL_EVENT_U2K_QUEUE == EPL_QUEUE_SHB
#include "EplEventShb.h"
#endif
#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_HOSTINTERFACE || \
    EPL_EVENT_KINT_QUEUE == EPL_QUEUE_HOSTINTERFACE || \
    EPL_EVENT_U2K_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif

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
\brief Event kernel instance type

The EventkCal produces events in the kernel-to-user (K2U) and kernel-internal
(KInt) queue. It consumes events from the kernel-internal and the
user-to-kernel (U2K) queue.
The event queue instances are stored in the cal and are used when calling
the event queue implementations (e.g. DIRECT or SHB).
*/
typedef struct _EplEventkCalInstance
{
    tEplEventQueueInstance  EventQueueK2U_m;
    ///< event queue instance of kernel to user queue
    tEplEventQueueInstance  EventQueueU2K_m;
    ///< event queue instance of user to kernel queue
    tEplEventQueueInstance  EventQueueKInt_m;
    ///< event queue instance of kernel internal queue
} tEplEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplEventkCalInstance EplEventkCalInstance_g; ///< Event kernel instance

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Kernel cal event add instance

Add kernel event cal.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkCalAddInstance (void)
{
    tEplKernel Ret = kEplSuccessful;

    EPL_MEMSET(&EplEventkCalInstance_g, 0, sizeof(tEplEventkCalInstance));

    //initialize kernel to user event queue
#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplEventDirectAddInstance(&EplEventkCalInstance_g.EventQueueK2U_m,
            kEplEventQueueK2U, FALSE);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_SHB
    Ret = EplEventShbAddInstance(&EplEventkCalInstance_g.EventQueueK2U_m,
            kEplEventQueueK2U, NULL, NULL);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //initialize kernel internal event queue
#if EPL_EVENT_KINT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplEventDirectAddInstance(&EplEventkCalInstance_g.EventQueueKInt_m,
            kEplEventQueueKInt, TRUE);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_SHB
    Ret = EplEventShbAddInstance(&EplEventkCalInstance_g.EventQueueKInt_m,
            kEplEventQueueKInt, EplEventkCalRxHandler, EplEventkPostError);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //initialize user to kernel queue
#if EPL_EVENT_U2K_QUEUE == EPL_QUEUE_DIRECT
    //no add instance necessary for direct calls!
#elif EPL_EVENT_U2K_QUEUE == EPL_QUEUE_SHB
    //kernel events from user layer are posted to kernel internal queue
    Ret = EplEventShbAddInstance(&EplEventkCalInstance_g.EventQueueU2K_m,
            kEplEventQueueU2K, EplEventkCalPost, EplEventkPostError);
#elif EPL_EVENT_U2K_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel cal event delete instance

Delete kernel event cal.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkCalDelInstance (void)
{
    tEplKernel Ret = kEplSuccessful;

#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplEventDirectDelInstance(EplEventkCalInstance_g.EventQueueK2U_m);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_SHB
    Ret = EplEventShbDelInstance(EplEventkCalInstance_g.EventQueueK2U_m);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_EVENT_KINT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplEventDirectDelInstance(EplEventkCalInstance_g.EventQueueKInt_m);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_SHB
    Ret = EplEventShbDelInstance(EplEventkCalInstance_g.EventQueueKInt_m);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif

#if EPL_EVENT_U2K_QUEUE == EPL_QUEUE_DIRECT
    //no del instance necessary for direct calls!
#elif EPL_EVENT_U2K_QUEUE == EPL_QUEUE_SHB
    Ret = EplEventShbDelInstance(EplEventkCalInstance_g.EventQueueU2K_m);
#elif EPL_EVENT_U2K_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel cal event post

This function determines the event's sink and posts it to the corresponding
event queue.

\param  pEvent_p                event posted by kernel

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkCalPost (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;

    BENCHMARK_MOD_27_SET(2);

    // split event post to user internal and user to kernel
    switch(pEvent_p->m_EventSink)
    {
        // userspace modules
        case kEplEventSinkNmtMnu:
        case kEplEventSinkNmtu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        case kEplEventSinkLedu:
        {
#if EPL_EVENT_K2U_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplEventDirectPost(EplEventkCalInstance_g.EventQueueK2U_m,
                    pEvent_p);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_SHB
            Ret = EplEventShbPost(EplEventkCalInstance_g.EventQueueK2U_m,
                    pEvent_p);
#elif EPL_EVENT_K2U_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
            break;
        }

        // kernelspace modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
        {
#if EPL_EVENT_KINT_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplEventDirectPost(EplEventkCalInstance_g.EventQueueKInt_m,
                    pEvent_p);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_SHB
            Ret = EplEventShbPost(EplEventkCalInstance_g.EventQueueKInt_m,
                    pEvent_p);
#elif EPL_EVENT_KINT_QUEUE == EPL_QUEUE_HOSTINTERFACE
#error
#endif
            break;
        }

        default:
        {
            Ret = kEplEventUnknownSink;
        }


    }// end of switch(pEvent_p->m_EventSink)

    BENCHMARK_MOD_27_RESET(2);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel cal event receive handler

This is the event receive function for events posted to the kernel layer.

\param  pEvent_p                event posted by user

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkCalRxHandler (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;

    BENCHMARK_MOD_27_SET(4);

    Ret = EplEventkProcess(pEvent_p);

    BENCHMARK_MOD_27_RESET(4);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
