/**
********************************************************************************
\file   event.c

\brief  MN Application event handler

This file contains a demo MN application event handler.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <stdio.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include "event.h"
#include <gpio/gpio.h>
#include <lcd/lcd.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static BOOL*        pfGsOff_l = NULL;
static tEventCb     pfnEventCb_l = NULL;
static tOplkError   errorEvent_l = kErrorOk;

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
static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p);

static tOplkError processHistoryEvent(tOplkApiEventType EventType_p,
                                      tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p);

static tOplkError processNodeEvent(tOplkApiEventType EventType_p,
                                   tOplkApiEventArg* pEventArg_p,
                                   void* pUserArg_p);

#ifdef CONFIG_INCLUDE_CFM
static tOplkError processCfmProgressEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

static tOplkError processCfmResultEvent(tOplkApiEventType EventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);
#else
static tOplkError setDefaultNodeAssignment(void);
static tOplkError processSdoEvent(tOplkApiEventType EventType_p,
                                  tOplkApiEventArg* pEventArg_p,
                                  void* pUserArg_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param  pfGsOff_p               Pointer to GsOff flag (determines that stack is down)
\param  pfnEventCb_p            User event callback

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
void initEvents(BOOL* pfGsOff_p, tEventCb pfnEventCb_p)
{
    pfGsOff_l = pfGsOff_p;
    pfnEventCb_l = pfnEventCb_p;
    errorEvent_l = kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the applications stack event handler.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType EventType_p, tOplkApiEventArg* pEventArg_p, void* pUserArg_p)
{
    tOplkError          ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    // check if NMT_GS_OFF is reached
    switch (EventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventHistoryEntry:
            ret = processHistoryEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventNode:
            ret = processNodeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

#ifdef CONFIG_INCLUDE_CFM
        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;
#else
        // Configuration Manager is not available,
        // so process SDO events
        case kOplkApiEventSdo:
            ret = processSdoEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;
#endif

        case kOplkApiEventLed:
            /* POWERLINK S/E LED needs to be changed */
            switch (pEventArg_p->ledEvent.ledType)
            {
                case kLedTypeStatus:
                    gpio_setStatusLed(pEventArg_p->ledEvent.fOn);
                    break;

                case kLedTypeError:
                    gpio_setErrorLed(pEventArg_p->ledEvent.fOn);
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    // call user event call back
    if ((ret == kErrorOk) && (pfnEventCb_l != NULL))
        ret = pfnEventCb_l(EventType_p, pEventArg_p, pUserArg_p);

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

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tOplkError                  ret = kErrorOk;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->nmtStateChange;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        PRINTF("Applications event module isn't initialized!\n");
        return kErrorGeneralError;
    }

    lcd_printNmtState(pNmtStateChange->newNmtState);

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
           // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process() and main()
            ret = kErrorShutdown;

            // In off state print error code
            if (errorEvent_l != kErrorOk)
                lcd_printError(errorEvent_l);

            // Reset error code
            errorEvent_l = kErrorOk;

            PRINTF("StateChangeEvent:kNmtGsOff originating event = 0x%X (%s)\n",
                   pNmtStateChange->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsResetCommunication:
#ifndef CONFIG_INCLUDE_CFM
            ret = setDefaultNodeAssignment();
#endif
            PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
            break;

        case kNmtGsResetConfiguration:
            PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
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
            PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    tEventError*            pInternalError = &pEventArg_p->internalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    errorEvent_l = pInternalError->oplkError;

    PRINTF("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
           debugstr_getEventSourceStr(pInternalError->eventSource),
           pInternalError->eventSource,
           debugstr_getRetValStr(pInternalError->oplkError),
           pInternalError->oplkError);

    // check additional argument
    switch (pInternalError->eventSource)
    {
        case kEventSourceEventk:
        case kEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            PRINTF(" OrgSource = %s %02X\n",
                   debugstr_getEventSourceStr(pInternalError->errorArg.eventSource),
                   pInternalError->errorArg.eventSource);
            break;

        case kEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            PRINTF(" val = %X\n", pInternalError->errorArg.uintArg);
            break;

        default:
            PRINTF("\n");
            break;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process history events

The function processes history events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processHistoryEvent(tOplkApiEventType EventType_p,
                                      tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p)
{
    tErrHistoryEntry*    pHistoryEntry = &pEventArg_p->errorHistoryEntry;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);
    UNUSED_PARAMETER(pHistoryEntry); // Avoid warning if debug is disabled

    PRINTF("HistoryEntry: Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
           pHistoryEntry->entryType, pHistoryEntry->errorCode,
           (WORD)pHistoryEntry->aAddInfo[0], (WORD)pHistoryEntry->aAddInfo[1],
           (WORD)pHistoryEntry->aAddInfo[2], (WORD)pHistoryEntry->aAddInfo[3],
           (WORD)pHistoryEntry->aAddInfo[4], (WORD)pHistoryEntry->aAddInfo[5],
           (WORD)pHistoryEntry->aAddInfo[6], (WORD)pHistoryEntry->aAddInfo[7]);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process node events

The function processes node events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNodeEvent(tOplkApiEventType EventType_p,
                                   tOplkApiEventArg* pEventArg_p,
                                   void* pUserArg_p)
{
    tOplkApiEventNode*   pNode = &pEventArg_p->nodeEvent;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    // check additional argument
    switch (pNode->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            PRINTF("NodeEvent: (Node=%u, CheckConf)\n", pNode->nodeId);
            break;

        case kNmtNodeEventUpdateConf:
            PRINTF("NodeEvent: (Node=%u, UpdateConf)\n", pNode->nodeId);
            break;

        case kNmtNodeEventNmtState:
            PRINTF("NodeEvent: (Node=%u, NmtState=%s)\n",
                   pNode->nodeId,
                   debugstr_getNmtStateStr(pNode->nmtState));
            break;

        case kNmtNodeEventError:
            PRINTF("NodeEvent: (Node=%u): Error=%s (0x%.4X)\n",
                   pNode->nodeId,
                   debugstr_getEmergErrCodeStr(pNode->errorCode),
                   pNode->errorCode);
            break;

        case kNmtNodeEventFound:
            PRINTF("NodeEvent: (Node=%u, Found)\n", pNode->nodeId);
            break;

        default:
            break;
    }
    return kErrorOk;
}

#ifdef CONFIG_INCLUDE_CFM
//------------------------------------------------------------------------------
/**
\brief  Process CFM progress events

The function processes CFM progress events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmProgressEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tCfmEventCnProgress*     pCfmProgress = &pEventArg_p->cfmProgress;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    PRINTF("CFM Progress: (Node=%u, CFM-Progress: Object 0x%X/%u, ",
           pCfmProgress->nodeId,
           pCfmProgress->objectIndex,
           pCfmProgress->objectSubIndex);

    PRINTF("%lu/%lu Bytes", (ULONG)pCfmProgress->bytesDownloaded,
                            (ULONG)pCfmProgress->totalNumberOfBytes);

    if ((pCfmProgress->sdoAbortCode != 0) ||
        (pCfmProgress->error != kErrorOk))
    {
        PRINTF(" -> SDO Abort=0x%lX, Error=0x%X)\n",
               (ULONG) pCfmProgress->sdoAbortCode,
               pCfmProgress->error);
    }
    else
    {
        PRINTF(")\n");
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM result events

The function processes CFM result events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmResultEvent(tOplkApiEventType EventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p)
{
    tOplkApiEventCfmResult*       pCfmResult = &pEventArg_p->cfmResult;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    switch (pCfmResult->nodeCommand)
    {
        case kNmtNodeCommandConfOk:
            PRINTF("CFM Result: (Node=%d, ConfOk)\n", pCfmResult->nodeId);
            break;

        case kNmtNodeCommandConfErr:
            PRINTF("CFM Result: (Node=%d, ConfErr)\n", pCfmResult->nodeId);
            break;

        case kNmtNodeCommandConfReset:
            PRINTF("CFM Result: (Node=%d, ConfReset)\n", pCfmResult->nodeId);
            break;

        case kNmtNodeCommandConfRestored:
            PRINTF("CFM Result: (Node=%d, ConfRestored)\n", pCfmResult->nodeId);
            break;

        default:
            PRINTF("CFM Result: (Node=%d, CfmResult=0x%X)\n", pCfmResult->nodeId,
                                                              pCfmResult->nodeCommand);
            break;
    }
    return kErrorOk;
}

#else

//------------------------------------------------------------------------------
/**
\brief  Process SDO events

The function processes SDO events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSdoEvent(tOplkApiEventType EventType_p,
                                  tOplkApiEventArg* pEventArg_p,
                                  void* pUserArg_p)
{
    tSdoComFinished*    pSdo = &pEventArg_p->sdoInfo;
    tOplkError          ret = kErrorOk;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    // SDO transfer finished
    if ((ret = oplk_freeSdoChannel(pSdo->sdoAccessType)) != kErrorOk)
    {
        return ret;
    }

    if (pSdo->sdoComConState == kSdoComTransferFinished)
    {   // continue boot-up of CN with NMT command Reset Configuration
        ret = oplk_triggerMnStateChange(pSdo->nodeId, kNmtNodeCommandConfReset);
    }
    else
    {   // indicate configuration error CN
        ret = oplk_triggerMnStateChange(pSdo->nodeId, kNmtNodeCommandConfErr);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set default node assignment

Set default node assignment in object dictionary if configuration manager is
not available.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setDefaultNodeAssignment(void)
{
    tOplkError  ret = kErrorOk;
    DWORD       nodeAssignment;

    nodeAssignment = (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS);    // 0x00000003L
    ret = oplk_writeLocalObject(0x1F81, 0x01, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x02, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x03, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x04, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x05, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x06, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x07, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x08, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x20, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0xFE, &nodeAssignment, sizeof(nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x6E, &nodeAssignment, sizeof(nodeAssignment));

    nodeAssignment = (NMT_NODEASSIGN_MN_PRES | NMT_NODEASSIGN_NODE_EXISTS);    // 0x00010001L
    ret = oplk_writeLocalObject(0x1F81, 0xF0, &nodeAssignment, sizeof(nodeAssignment));
    return ret;
}
#endif

/// \}
