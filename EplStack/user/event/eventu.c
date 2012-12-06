/**
********************************************************************************
\file   eventu.c

\brief  source file for Epl-Userspace-Event-Module

This is the highest abstraction of the user event module.

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
#include "user/eventu.h"
#include "user/eventucal.h"
#include "user/EplNmtu.h"
#include "user/EplNmtMnu.h"
#include "user/EplSdoAsySequ.h"
#include "user/dllucal.h"
#include "user/EplLedu.h"
#include "Benchmark.h"


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
\brief Event user instance type

The user event instance holds the Api process callback function pointer.
*/
typedef struct _tEplEventuInstance
{
    tEplProcessEventCb  m_pfnApiProcessEventCb;
                                        ///< callback for generic api events
} tEplEventuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplEventuInstance EplEventuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    User event initialization

Initialize the user event module.

\param  pfnApiProcessEventCb_p  function pointer to generic event callback

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuInit (tEplProcessEventCb pfnApiProcessEventCb_p)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventuAddInstance(pfnApiProcessEventCb_p);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event add instance

Add user event module.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuAddInstance (
        tEplProcessEventCb pfnApiProcessEventCb_p)
{
    tEplKernel Ret = kEplSuccessful;

    // init instance variables
    EplEventuInstance_g.m_pfnApiProcessEventCb = pfnApiProcessEventCb_p;

    Ret = EplEventuCalAddInstance();

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event delete instance

Delete user event module.

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuDelInstance (void)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventuCalDelInstance();

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    User thread that dispatches user events

Process events posted to the user layer from the EventuCal.

\param  pEvent_p                user event

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuProcess (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplEventSource EventSource;

    // check m_EventSink
    switch(pEvent_p->m_EventSink)
    {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
        // NMT-User-Module
        case kEplEventSinkNmtu:
        {
            Ret = EplNmtuProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // NMT-MN-User-Module
        case kEplEventSinkNmtMnu:
        {
            Ret = EplNmtMnuProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtMnu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if ((((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)   \
     || (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0))
        // events for asynchronous SDO Sequence Layer
        case kEplEventSinkSdoAsySeq:
        {
            Ret = EplSdoAsySeqProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceSdoAsySeq;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

        // LED user part module
        case kEplEventSinkLedu:
        {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
            Ret = EplLeduProcessEvent(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceLedu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

        // event for EPL api
        case kEplEventSinkApi:
        {
            if (EplEventuInstance_g.m_pfnApiProcessEventCb != NULL)
            {
                Ret = EplEventuInstance_g.m_pfnApiProcessEventCb(pEvent_p);
                if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
                {
                    EventSource = kEplEventSourceEplApi;

                    // Error event for API layer
                    EplEventuPostError(kEplEventSourceEventu,
                                    Ret,
                                    sizeof(EventSource),
                                    &EventSource);
                }
            }
            break;

        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLU)) != 0)
        case kEplEventSinkDlluCal:
        {
            Ret = dllucal_process(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllu;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;

        }
#endif

        case kEplEventSinkErru:
        {
            /*
            Ret = EplErruProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceErru;

                // Error event for API layer
                EplEventuPostError(kEplEventSourceEventu,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            */
            break;

        }

        // unknown sink
        default:
        {
            Ret = kEplEventUnknownSink;

            // Error event for API layer
            EplEventuPostError(kEplEventSourceEventu,
                            Ret,
                            sizeof(pEvent_p->m_EventSink),
                            &pEvent_p->m_EventSink);
        }

    } // end of switch(pEvent_p->m_EventSink)

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    User event process

This function posts an event to the EventuCal.

\param  pEvent_p                event posted by user

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC EplEventuPost (tEplEvent *pEvent_p)
{
    tEplKernel Ret = kEplSuccessful;

    Ret = EplEventuCalPost(pEvent_p);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    post error by user

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
tEplKernel PUBLIC EplEventuPostError (tEplEventSource EventSource_p,
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
    Ret = EplEventuPost(&EplEvent);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
