/**
********************************************************************************
\file   eventk.c

\brief  source file for Epl-Kernelspace-Event-Module

This is the highest abstraction of the kernel event module.

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
#include "kernel/eventk.h"
#include "kernel/eventkcal.h"
#include "kernel/EplNmtk.h"
#include "kernel/EplDllk.h"
#include "kernel/dllkcal.h"
#include "kernel/EplErrorHandlerk.h"
#include "Benchmark.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
#include "kernel/EplPdok.h"
#include "kernel/EplPdokCal.h"
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
\brief    Kernel event initialization

Initialize the kernel event module.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkInit (void)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventkAddInstance();

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event add instance

Add kernel event module.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkAddInstance (void)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventkCalAddInstance();

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event delete instance

Delete kernel event module.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkDelInstance (void)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventkCalDelInstance();

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel thread that dispatches kernel events

Process events posted to the kernel layer from the EventkCal.

\param  pEvent_p                kernel event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkProcess (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventSource EventSource;

    // check m_EventSink
    switch(pEvent_p->m_EventSink)
    {
        // NMT-Kernel-Modul
        case kEplEventSinkNmtk:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)
            Ret = EplNmtkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            BENCHMARK_MOD_27_RESET(0);
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            if ((pEvent_p->m_EventType == kEplEventTypeNmtEvent)
                && (*((tEplNmtEvent*)pEvent_p->m_pArg) == kEplNmtEventDllCeSoa))
            {

                BENCHMARK_MOD_27_SET(0);

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
                // forward SoA event to DLLk module for cycle preprocessing
                Ret = EplDllkProcess(pEvent_p);
                if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
                {
                    EventSource = kEplEventSourceDllk;

                    // Error event for API layer
                    EplEventkPostError(kEplEventSourceEventk,
                                    Ret,
                                    sizeof(EventSource),
                                    &EventSource);
                }
#endif
                BENCHMARK_MOD_27_RESET(0);

            }
#endif
            break;
        }

        // events for Dllk module
        case kEplEventSinkDllk:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            Ret = EplDllkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

        // events for DllkCal module
        case kEplEventSinkDllkCal:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            Ret = dllkcal_process(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
        // events for PDO CAL module
        case kEplEventSinkPdokCal:
        {
            Ret = EplPdokCalProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourcePdok;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
        // events for Error handler module
        case kEplEventSinkErrk:
        {
            // only call error handler if DLL is present
            Ret = EplErrorHandlerkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceErrk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

        // unknown sink
        default:
        {
            Ret = kEplEventUnknownSink;

            // Error event for API layer
            EplEventkPostError(kEplEventSourceEventk,
                            Ret,
                            sizeof(pEvent_p->m_EventSink),
                            &pEvent_p->m_EventSink);
        }

    } // end of switch(pEvent_p->m_EventSink)

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Kernel event post

This function posts an event to the EventkCal.

\param  pEvent_p                event posted by kernel

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkPost (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventkCalPost(pEvent_p);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    post error by kernel

This function posts an error to the Api.

\param  EventSource_p           source that posts the error
\param  EplError_p              error code
\param  uiArgSize_p             size of error argument
\param  pArg_p                  error argument

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventkPostError (tEplEventSource EventSource_p,
        tEplKernel EplError_p,
        unsigned int uiArgSize_p,
        void *pArg_p)
{
    tEplKernel Ret;
    tEplEventError EventError;
    tEplEvent EplEvent;

    Ret = kEplSuccessful;

    // create argument
    EventError.m_EventSource = EventSource_p;
    EventError.m_EplError = EplError_p;
    uiArgSize_p = (unsigned int) min ((size_t) uiArgSize_p, sizeof (EventError.m_Arg));
    EPL_MEMCPY(&EventError.m_Arg, pArg_p, uiArgSize_p);

    // create event
    EplEvent.m_EventType = kEplEventTypeError;
    EplEvent.m_EventSink = kEplEventSinkApi;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(EplEvent.m_NetTime));
    EplEvent.m_uiSize = (memberoffs (tEplEventError, m_Arg) + uiArgSize_p);
    EplEvent.m_pArg = &EventError;

    // post errorevent
    Ret = EplEventkPost(&EplEvent);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
