/**
********************************************************************************
\file   event.c

\brief  MN Application event handler

This file contains a demo MN application event handler.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <console/console.h>
#include "event.h"

#include <eventlog/eventlog.h>

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
static tOplkError processStateChangeEvent(tOplkApiEventType eventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

static tOplkError processErrorWarningEvent(tOplkApiEventType eventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p);

static tOplkError processHistoryEvent(tOplkApiEventType eventType_p,
                                      tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p);

static tOplkError processNodeEvent(tOplkApiEventType eventType_p,
                                   tOplkApiEventArg* pEventArg_p,
                                   void* pUserArg_p);

static tOplkError processPdoChangeEvent(tOplkApiEventType eventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);

static tOplkError processCfmProgressEvent(tOplkApiEventType eventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

static tOplkError processCfmResultEvent(tOplkApiEventType eventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param  pCycle_p                Pointer to cycle time.
\param  pfGsOff_p               Pointer to GsOff flag (determines that stack is down)

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------

void initEvents(BOOL* pfGsOff_p)
{
    pfGsOff_l = pfGsOff_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the application stack event handler.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType eventType_p,
                         tOplkApiEventArg* pEventArg_p,
                         void* pUserArg_p)
{
    tOplkError          ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    // check if NMT_GS_OFF is reached
    switch (eventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventHistoryEntry:
            ret = processHistoryEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventNode:
            ret = processNodeEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(eventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(eventType_p, pEventArg_p, pUserArg_p);
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

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(tOplkApiEventType eventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tOplkError                  ret = kErrorOk;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->nmtStateChange;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        console_printlog("Application event module is not initialized!\n");
        return kErrorGeneralError;
    }

    eventlog_printStateEvent(pNmtStateChange);

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
           // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process() and main()
            ret = kErrorShutdown;

            printf("Stack received kNmtGsOff!\n");

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsResetCommunication:
            break;

        case kNmtGsResetConfiguration:
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:        // Implement
        case kNmtMsNotActive:               // handling of
        case kNmtMsPreOperational1:         // different
        case kNmtMsPreOperational2:         // states here
        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
        case kNmtMsBasicEthernet:           // no break

        default:
            printf("Stack entered state: %s\n", debugstr_getNmtStateStr(pNmtStateChange->newNmtState));
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processErrorWarningEvent(tOplkApiEventType eventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    tEventError*            pInternalError = &pEventArg_p->internalError;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printErrorEvent(pInternalError);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process history events

The function processes history events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processHistoryEvent(tOplkApiEventType eventType_p,
                                      tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p)
{
    tErrHistoryEntry*    pHistoryEntry = &pEventArg_p->errorHistoryEntry;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printHistoryEvent(pHistoryEntry);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process node events

The function processes node events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNodeEvent(tOplkApiEventType eventType_p,
                                   tOplkApiEventArg* pEventArg_p,
                                   void* pUserArg_p)
{
    tOplkApiEventNode*   pNode = &pEventArg_p->nodeEvent;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printNodeEvent(pNode);

    // check additional argument
    switch (pNode->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventNmtState:
            printf("Node %d entered state %s\n", pNode->nodeId, debugstr_getNmtStateStr(pNode->nmtState));
            break;

        case kNmtNodeEventError:
            break;

        case kNmtNodeEventFound:
            printf("Stack found node %d\n", pNode->nodeId);
            break;

        case kNmtNodeEventAmniReceived:
            break;

        default:
            break;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process PDO change events

The function processes PDO change events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processPdoChangeEvent(tOplkApiEventType eventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p)
{
    tOplkApiEventPdoChange*     pPdoChange = &pEventArg_p->pdoChange;
    UINT                        subIndex;
    UINT64                      mappObject;
    tOplkError                  ret;
    UINT                        varLen;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printPdoEvent(pPdoChange);

    for (subIndex = 1; subIndex <= pPdoChange->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange->mappParamIndex, subIndex, &mappObject, &varLen);
        if (ret != kErrorOk)
        {
            eventlog_printMessage(kEventlogLevelError, kEventlogCategoryObjectDictionary,
                                  "Reading 0x%X/%d failed with %s(0x%X)",
                                  pPdoChange->mappParamIndex, subIndex, debugstr_getRetValStr(ret), ret);
            continue;
        }
        eventlog_printPdoMap(pPdoChange->mappParamIndex, subIndex, mappObject);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM progress events

The function processes CFM progress events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmProgressEvent(tOplkApiEventType eventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tCfmEventCnProgress*     pCfmProgress = &pEventArg_p->cfmProgress;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmProgressEvent(pCfmProgress);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM result events

The function processes CFM result events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmResultEvent(tOplkApiEventType eventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p)
{
    tOplkApiEventCfmResult*       pCfmResult = &pEventArg_p->cfmResult;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmResultEvent(pCfmResult->nodeId, pCfmResult->nodeCommand);

    switch (pCfmResult->nodeCommand)
    {
        case kNmtNodeCommandConfOk:
            break;

        case kNmtNodeCommandConfErr:
            break;

        case kNmtNodeCommandConfReset:
            break;

        case kNmtNodeCommandConfRestored:
            break;

        default:
            break;
    }
    return kErrorOk;
}

/// \}
