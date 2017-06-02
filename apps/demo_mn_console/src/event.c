/**
********************************************************************************
\file   event.c

\brief  MN Application event handler

This file contains a demo MN application event handler.

\ingroup module_demo_mn_console
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
\brief Event instance
*/
typedef struct
{
    tEventConfig config; ///< Configuration provided in initEvents()
} tEventInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p);
static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                           void* pUserArg_p);
static tOplkError processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                      void* pUserArg_p);
static tOplkError processNodeEvent(const tOplkApiEventNode* pNode_p,
                                   void* pUserArg_p);
static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                        void* pUserArg_p);
static tOplkError processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                          void* pUserArg_p);
static tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                        void* pUserArg_p);
static tOplkError processFirmwareManagerEvents(tOplkApiEventType eventType_p,
                                               const tOplkApiEventArg* pEventArg_p,
                                               void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param[in]      pfGsOff_p           Pointer to GsOff flag (determines if the stack is down)

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------

void initEvents(const tEventConfig* config)
{
    memcpy(&instance_l, config, sizeof(tEventConfig));
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the application's stack event handler.

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Pointer to union which describes the event in detail
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType eventType_p,
                         const tOplkApiEventArg* pEventArg_p,
                         void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    // check if NMT_GS_OFF is reached
    switch (eventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(&pEventArg_p->nmtStateChange, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(&pEventArg_p->internalError, pUserArg_p);
            break;

        case kOplkApiEventHistoryEntry:
            ret = processHistoryEvent(&pEventArg_p->errorHistoryEntry, pUserArg_p);
            break;

        case kOplkApiEventNode:
            ret = processNodeEvent(&pEventArg_p->nodeEvent, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(&pEventArg_p->pdoChange, pUserArg_p);
            break;

        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(&pEventArg_p->cfmProgress, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(&pEventArg_p->cfmResult, pUserArg_p);
            break;

        default:
            break;
    }

    if (ret == kErrorOk)
    {
        ret = processFirmwareManagerEvents(eventType_p, pEventArg_p, pUserArg_p);
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

\param[in]      pNmtStateChange_p   Pointer to the state change structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    if (instance_l.config.pfGsOff == NULL)
    {
        console_printlog("Application event module is not initialized!\n");
        return kErrorGeneralError;
    }

    eventlog_printStateEvent(pNmtStateChange_p);

    switch (pNmtStateChange_p->newNmtState)
    {
        case kNmtGsOff:
           // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process() and main()
            ret = kErrorShutdown;

            printf("Stack received kNmtGsOff!\n");

            // signal that stack is off
            *instance_l.config.pfGsOff = TRUE;
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

\param[in]      pInternalError_p    Pointer to the error structure
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
\brief  Process history events

The function processes history events.

\param[in]      pHistoryEntry_p     Pointer to the history entry
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                      void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printHistoryEvent(pHistoryEntry_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process node events

The function processes node events.

\param[in]      pNode_p             Pointer to the node event
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNodeEvent(const tOplkApiEventNode* pNode_p,
                                   void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printNodeEvent(pNode_p);

    // check additional argument
    switch (pNode_p->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventNmtState:
            printf("Node %d entered state %s\n",
                   pNode_p->nodeId,
                   debugstr_getNmtStateStr(pNode_p->nmtState));
            break;

        case kNmtNodeEventError:
            break;

        case kNmtNodeEventFound:
            printf("Stack found node %d\n",
                   pNode_p->nodeId);
            break;

        case kNmtNodeEventAmniReceived:
            break;

        case kNmtNodeEventUpdateSw:
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

        eventlog_printPdoMap(pPdoChange_p->mappParamIndex,
                             subIndex,
                             mappObject);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM progress events

The function processes CFM progress events.

\param[in]      pCfmProgress_p      Pointer to the CFM progress information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                          void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmProgressEvent(pCfmProgress_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM result events

The function processes CFM result events.

\param[in]      pCfmResult_p        Pointer to the CFM result information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                        void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmResultEvent(pCfmResult_p->nodeId, pCfmResult_p->nodeCommand);

    switch (pCfmResult_p->nodeCommand)
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

static tOplkError processFirmwareManagerEvents(tOplkApiEventType eventType_p,
                                               const tOplkApiEventArg* pEventArg_p,
                                               void* pUserArg_p)
{
    tOplkError ret = kErrorOk;
    BOOL fCallFirmwareManager = FALSE;
    tOplkApiEventNode* pEventNode;

    switch (eventType_p)
    {
        case kOplkApiEventUserDef:
            fCallFirmwareManager = TRUE;
            break;

        case kOplkApiEventSdo:
            fCallFirmwareManager = TRUE;
            break;

        case kOplkApiEventNode:
            pEventNode = (tOplkApiEventNode*)&pEventArg_p->nodeEvent;
            fCallFirmwareManager = ((pEventNode->nodeEvent == kNmtNodeEventUpdateSw) ||
                    (pEventNode->nodeEvent == kNmtNodeEventConfDone));
            break;

        default:
            break;
    }

    if (fCallFirmwareManager && (instance_l.config.pfnFirmwareManagerCallback != NULL))
    {
        ret = instance_l.config.pfnFirmwareManagerCallback(eventType_p, pEventArg_p, pUserArg_p);
    }

    return ret;
}

/// \}
