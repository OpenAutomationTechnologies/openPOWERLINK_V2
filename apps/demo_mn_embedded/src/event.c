/**
********************************************************************************
\file   event.c

\brief  MN Application event handler

This file contains a demo MN application event handler.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
static BOOL*        pfGsOff_l = NULL;
static tEventCb     pfnEventCb_l = NULL;
static tOplkError   errorEvent_l = kErrorOk;

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
static tOplkError processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                          void* pUserArg_p);
static tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                        void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param[in]      pfGsOff_p           Pointer to GsOff flag (determines if the stack is down)
\param[in]      pfnEventCb_p        User event callback

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
void initEvents(BOOL* pfGsOff_p,
                tEventCb pfnEventCb_p)
{
    pfGsOff_l = pfGsOff_p;
    pfnEventCb_l = pfnEventCb_p;
    errorEvent_l = kErrorOk;
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

        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(&pEventArg_p->cfmProgress, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(&pEventArg_p->cfmResult, pUserArg_p);
            break;

        default:
            break;
    }

    // call user event call back
    if ((ret == kErrorOk) &&
        (pfnEventCb_l != NULL))
        ret = pfnEventCb_l(eventType_p, pEventArg_p, pUserArg_p);

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

    if (pfGsOff_l == NULL)
    {
        PRINTF("Application event module is not initialized!\n");
        return kErrorGeneralError;
    }

    lcd_printNmtState(pNmtStateChange_p->newNmtState);

    switch (pNmtStateChange_p->newNmtState)
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
                   pNmtStateChange_p->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange_p->nmtEvent));

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsResetCommunication:
            PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange_p->newNmtState,
                   pNmtStateChange_p->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange_p->nmtEvent));
            break;

        case kNmtGsResetConfiguration:
            PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange_p->newNmtState,
                   pNmtStateChange_p->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange_p->nmtEvent));
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
                   pNmtStateChange_p->newNmtState,
                   pNmtStateChange_p->nmtEvent,
                   debugstr_getNmtEventStr(pNmtStateChange_p->nmtEvent));
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

    errorEvent_l = pInternalError_p->oplkError;

    PRINTF("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
           debugstr_getEventSourceStr(pInternalError_p->eventSource),
           pInternalError_p->eventSource,
           debugstr_getRetValStr(pInternalError_p->oplkError),
           pInternalError_p->oplkError);

    // check additional argument
    switch (pInternalError_p->eventSource)
    {
        case kEventSourceEventk:
        case kEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            PRINTF(" OrgSource = %s %02X\n",
                   debugstr_getEventSourceStr(pInternalError_p->errorArg.eventSource),
                   pInternalError_p->errorArg.eventSource);
            break;

        case kEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the UINT argument contains the DLL state and the NMT event
            PRINTF(" val = %X\n", pInternalError_p->errorArg.uintArg);
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

\param[in]      pHistoryEntry_p     Pointer to the history entry
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                      void* pUserArg_p)
{
    UNUSED_PARAMETER(pHistoryEntry_p); // Avoid warning if debug is disabled
    UNUSED_PARAMETER(pUserArg_p);

    PRINTF("HistoryEntry: Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
           pHistoryEntry_p->entryType,
           pHistoryEntry_p->errorCode,
           (WORD)pHistoryEntry_p->aAddInfo[0],
           (WORD)pHistoryEntry_p->aAddInfo[1],
           (WORD)pHistoryEntry_p->aAddInfo[2],
           (WORD)pHistoryEntry_p->aAddInfo[3],
           (WORD)pHistoryEntry_p->aAddInfo[4],
           (WORD)pHistoryEntry_p->aAddInfo[5],
           (WORD)pHistoryEntry_p->aAddInfo[6],
           (WORD)pHistoryEntry_p->aAddInfo[7]);

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

    // check additional argument
    switch (pNode_p->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            PRINTF("NodeEvent: (Node=%u, CheckConf)\n", pNode_p->nodeId);
            break;

        case kNmtNodeEventUpdateConf:
            PRINTF("NodeEvent: (Node=%u, UpdateConf)\n", pNode_p->nodeId);
            break;

        case kNmtNodeEventNmtState:
            PRINTF("NodeEvent: (Node=%u, NmtState=%s)\n",
                   pNode_p->nodeId,
                   debugstr_getNmtStateStr(pNode_p->nmtState));
            break;

        case kNmtNodeEventError:
            PRINTF("NodeEvent: (Node=%u): Error=%s (0x%.4X)\n",
                   pNode_p->nodeId,
                   debugstr_getEmergErrCodeStr(pNode_p->errorCode),
                   pNode_p->errorCode);
            break;

        case kNmtNodeEventFound:
            PRINTF("NodeEvent: (Node=%u, Found)\n", pNode_p->nodeId);
            break;

        default:
            break;
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

    PRINTF("CFM Progress: (Node=%u, CFM-Progress: Object 0x%X/%u, ",
           pCfmProgress_p->nodeId,
           pCfmProgress_p->objectIndex,
           pCfmProgress_p->objectSubIndex);

    PRINTF("%lu/%lu Bytes",
           (ULONG)pCfmProgress_p->bytesDownloaded,
           (ULONG)pCfmProgress_p->totalNumberOfBytes);

    if ((pCfmProgress_p->sdoAbortCode != 0) ||
        (pCfmProgress_p->error != kErrorOk))
    {
        PRINTF(" -> SDO Abort=0x%lX, Error=0x%X)\n",
               (ULONG)pCfmProgress_p->sdoAbortCode,
               pCfmProgress_p->error);
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

\param[in]      pCfmResult_p        Pointer to the CFM result information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                        void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    switch (pCfmResult_p->nodeCommand)
    {
        case kNmtNodeCommandConfOk:
            PRINTF("CFM Result: (Node=%d, ConfOk)\n",
                   pCfmResult_p->nodeId);
            break;

        case kNmtNodeCommandConfErr:
            PRINTF("CFM Result: (Node=%d, ConfErr)\n",
                   pCfmResult_p->nodeId);
            break;

        case kNmtNodeCommandConfReset:
            PRINTF("CFM Result: (Node=%d, ConfReset)\n",
                   pCfmResult_p->nodeId);
            break;

        case kNmtNodeCommandConfRestored:
            PRINTF("CFM Result: (Node=%d, ConfRestored)\n",
                   pCfmResult_p->nodeId);
            break;

        default:
            PRINTF("CFM Result: (Node=%d, CfmResult=0x%X)\n",
                   pCfmResult_p->nodeId,
                   pCfmResult_p->nodeCommand);
            break;
    }

    return kErrorOk;
}

/// \}
