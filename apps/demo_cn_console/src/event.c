/**
********************************************************************************
\file   event.c

\brief  CN application event handler

This file contains a demo CN application event handler.

\ingroup module_demo
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#include "event.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <console/console.h>
#include <eventlog/eventlog.h>

#include <stdio.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static BOOL*    pfGsOff_l;

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
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p);
static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                           void* pUserArg_p);
static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                        void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param[in]      pfGsOff_p           Pointer to GsOff flag (determines that stack is down)

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void initEvents(BOOL* pfGsOff_p)
{
    pfGsOff_l = pfGsOff_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the application's stack event handler.

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Pointer to union which describes the event in detail
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType eventType_p,
                         const tOplkApiEventArg* pEventArg_p,
                         void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    switch (eventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(&pEventArg_p->nmtStateChange, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(&pEventArg_p->internalError, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(&pEventArg_p->pdoChange, pUserArg_p);
            break;

        default:
            break;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process state change events

The function processes state change events.

\param[in]      pNmtStateChange_p   Pointer to the state change event structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
        return kErrorGeneralError;

    eventlog_printStateEvent(pNmtStateChange_p);

    switch (pNmtStateChange_p->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            ret = kErrorShutdown;

            printf("Stack received kNmtGsOff!\n");

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetConfiguration:
        case kNmtGsResetCommunication:
        case kNmtCsNotActive:               // Implement
        case kNmtCsPreOperational1:         // handling of
        case kNmtCsStopped:                 // different
        case kNmtCsPreOperational2:         // states here
        case kNmtCsReadyToOperate:
        case kNmtCsOperational:
        case kNmtCsBasicEthernet:           // no break;

        default:
            printf("Stack entered state: %s\n",
                   debugstr_getNmtStateStr(pNmtStateChange_p->newNmtState));
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param[in]      pInternalError_p    Pointer to the internal error structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                           void* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printErrorEvent(pInternalError_p);

    return kErrorOk;
}


//------------------------------------------------------------------------------
/**
\brief  Process PDO change events

The function processes PDO change events.

\param[in]      pPdoChange_p        Pointer to the PDO change event structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                        void* pUserArg_p)
{
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    size_t      varLen;

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printPdoEvent(pPdoChange_p);

    for (subIndex = 1; subIndex <= pPdoChange_p->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange_p->mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            eventlog_printMessage(kEventlogLevelError,
                                  kEventlogCategoryObjectDictionary,
                                  "Reading 0x%X/%d failed with %s(0x%X)",
                                  pPdoChange_p->mappParamIndex,
                                  subIndex,
                                  debugstr_getRetValStr(ret),
                                  ret);
            continue;
        }
        eventlog_printPdoMap(pPdoChange_p->mappParamIndex, subIndex, mappObject);
    }

    return kErrorOk;
}

/// \}
