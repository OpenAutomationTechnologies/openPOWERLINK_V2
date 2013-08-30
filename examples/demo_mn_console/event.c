/**
********************************************************************************
\file   event.c

\brief  MN Application event handler

This file contains a demo MN application event handler.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <Epl.h>
#include <console/console.h>

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
static tEplKernel processStateChangeEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p);

static tEplKernel processErrorWarningEvent(tEplApiEventType EventType_p,
                                           tEplApiEventArg* pEventArg_p,
                                           void GENERIC* pUserArg_p);

static tEplKernel processHistoryEvent(tEplApiEventType EventType_p,
                                      tEplApiEventArg* pEventArg_p,
                                      void GENERIC* pUserArg_p);

static tEplKernel processNodeEvent(tEplApiEventType EventType_p,
                                   tEplApiEventArg* pEventArg_p,
                                   void GENERIC* pUserArg_p);

#ifdef CONFIG_INCLUDE_CFM
static tEplKernel processCfmProgressEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p);

static tEplKernel processCfmResultEvent(tEplApiEventType EventType_p,
                                        tEplApiEventArg* pEventArg_p,
                                        void GENERIC* pUserArg_p);
#else
static tEplKernel setDefaultNodeAssignment(void);
static tEplKernel processSdoEvent(tEplApiEventType EventType_p,
                                  tEplApiEventArg* pEventArg_p,
                                  void GENERIC* pUserArg_p);
#endif

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

void initEvents (BOOL* pfGsOff_p)
{
    pfGsOff_l = pfGsOff_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the applications stack event handler.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC processEvents(tEplApiEventType EventType_p,
                                tEplApiEventArg* pEventArg_p,
                                void GENERIC* pUserArg_p)
{
    tEplKernel          ret = kEplSuccessful;

    UNUSED_PARAMETER(pUserArg_p);

    // check if NMT_GS_OFF is reached
    switch (EventType_p)
    {
        case kEplApiEventNmtStateChange:
            ret = processStateChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventCriticalError:
        case kEplApiEventWarning:
            ret = processErrorWarningEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventHistoryEntry:
            ret = processHistoryEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventNode:
            ret = processNodeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

#ifdef CONFIG_INCLUDE_CFM
        case kEplApiEventCfmProgress:
            ret = processCfmProgressEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventCfmResult:
            ret = processCfmResultEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;
#else
        // Configuration Manager is not available,
        // so process SDO events
        case kEplApiEventSdo:
            ret = processSdoEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;
#endif

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

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateChangeEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->m_NmtStateChange;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        console_printlog("Applications event module isn't initialized!\n");
        return kEplGeneralError;
    }

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
           // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical EPL stack error
            // -> also shut down oplk_process() and main()
            ret = kEplShutdown;

            console_printlog("StateChangeEvent:kNmtGsOff originating event = 0x%X (%s)\n",
                     pNmtStateChange->nmtEvent,
                     EplGetNmtEventStr(pNmtStateChange->nmtEvent));

            // signal that stack is off
            *pfGsOff_l = FALSE;
            break;

        case kNmtGsResetCommunication:
#ifndef CONFIG_INCLUDE_CFM
            ret = setDefaultNodeAssignment();
#endif
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   EplGetNmtEventStr(pNmtStateChange->nmtEvent));
            break;

        case kNmtGsResetConfiguration:
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   EplGetNmtEventStr(pNmtStateChange->nmtEvent));
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
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                   pNmtStateChange->newNmtState,
                   pNmtStateChange->nmtEvent,
                   EplGetNmtEventStr(pNmtStateChange->nmtEvent));
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processErrorWarningEvent(tEplApiEventType EventType_p,
                                           tEplApiEventArg* pEventArg_p,
                                           void GENERIC* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    tEplEventError*         pInternalError = &pEventArg_p->m_InternalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("Err/Warn: Source = %s (%02X) EplError = %s (0x%03X)\n",
                EplGetEventSourceStr(pInternalError->m_EventSource),
                pInternalError->m_EventSource,
                EplGetEplKernelStr(pInternalError->m_EplError),
                pInternalError->m_EplError);

    FTRACE_MARKER("Err/Warn: Source = %s (%02X) EplError = %s (0x%03X)\n",
                EplGetEventSourceStr(pInternalError->m_EventSource),
                pInternalError->m_EventSource,
                EplGetEplKernelStr(pInternalError->m_EplError),
                pInternalError->m_EplError);

    // check additional argument
    switch (pInternalError->m_EventSource)
    {
        case kEplEventSourceEventk:
        case kEplEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            console_printlog(" OrgSource = %s %02X\n",
                     EplGetEventSourceStr(pInternalError->m_Arg.m_EventSource),
                     pInternalError->m_Arg.m_EventSource);

            FTRACE_MARKER(" OrgSource = %s %02X\n",
                     EplGetEventSourceStr(pInternalError->m_Arg.m_EventSource),
                     pInternalError->m_Arg.m_EventSource);
            break;

        case kEplEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            console_printlog(" val = %X\n", pInternalError->m_Arg.m_dwArg);
            FTRACE_MARKER(" val = %X\n", pInternalError->m_Arg.m_dwArg);
            break;

        default:
            console_printlog("\n");
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process history events

The function processes history events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processHistoryEvent(tEplApiEventType EventType_p,
                                      tEplApiEventArg* pEventArg_p,
                                      void GENERIC* pUserArg_p)
{
    tEplErrHistoryEntry*    pHistoryEntry = &pEventArg_p->m_ErrHistoryEntry;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("HistoryEntry: Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
             pHistoryEntry->m_wEntryType, pHistoryEntry->m_wErrorCode,
            (WORD)pHistoryEntry->m_abAddInfo[0], (WORD)pHistoryEntry->m_abAddInfo[1],
            (WORD)pHistoryEntry->m_abAddInfo[2], (WORD)pHistoryEntry->m_abAddInfo[3],
            (WORD)pHistoryEntry->m_abAddInfo[4], (WORD)pHistoryEntry->m_abAddInfo[5],
            (WORD)pHistoryEntry->m_abAddInfo[6], (WORD)pHistoryEntry->m_abAddInfo[7]);

    FTRACE_MARKER("HistoryEntry: Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
            pHistoryEntry->m_wEntryType, pHistoryEntry->m_wErrorCode,
            (WORD)pHistoryEntry->m_abAddInfo[0], (WORD)pHistoryEntry->m_abAddInfo[1],
            (WORD)pHistoryEntry->m_abAddInfo[2], (WORD)pHistoryEntry->m_abAddInfo[3],
            (WORD)pHistoryEntry->m_abAddInfo[4], (WORD)pHistoryEntry->m_abAddInfo[5],
            (WORD)pHistoryEntry->m_abAddInfo[6], (WORD)pHistoryEntry->m_abAddInfo[7]);

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process node events

The function processes node events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processNodeEvent(tEplApiEventType EventType_p,
                                   tEplApiEventArg* pEventArg_p,
                                   void GENERIC* pUserArg_p)
{
    tEplApiEventNode*   pNode = &pEventArg_p->m_Node;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    // check additional argument
    switch (pNode->m_NodeEvent)
    {
        case kNmtNodeEventCheckConf:
            console_printlog("NodeEvent: (Node=%u, CheckConf)\n", pNode->m_uiNodeId);
            break;

        case kNmtNodeEventUpdateConf:
            console_printlog("NodeEvent: (Node=%u, UpdateConf)\n", pNode->m_uiNodeId);
            break;

        case kNmtNodeEventNmtState:
            console_printlog("NodeEvent: (Node=%u, NmtState=%s)\n",
                     pNode->m_uiNodeId,
                     EplGetNmtStateStr(pNode->m_NmtState));
            break;

        case kNmtNodeEventError:
            console_printlog("NodeEvent: (Node=%u): Error=%s (0x%.4X)\n",
                    pNode->m_uiNodeId,
                    EplGetEmergErrCodeStr(pNode->m_wErrorCode),
                    pNode->m_wErrorCode);
            break;

        case kNmtNodeEventFound:
            console_printlog("NodeEvent: (Node=%u, Found)\n", pNode->m_uiNodeId);
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

#ifdef CONFIG_INCLUDE_CFM
//------------------------------------------------------------------------------
/**
\brief  Process CFM progress events

The function processes CFM progress events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processCfmProgressEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p)
{
    tCfmEventCnProgress*     pCfmProgress = &pEventArg_p->m_CfmProgress;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("CFM Progress: (Node=%u, CFM-Progress: Object 0x%X/%u, ",
                                                 pCfmProgress->nodeId,
                                                 pCfmProgress->objectIndex,
                                                 pCfmProgress->objectSubIndex);

    console_printlogadd("%lu/%lu Bytes", (ULONG)pCfmProgress->bytesDownloaded,
                            (ULONG)pCfmProgress->totalNumberOfBytes);

    if ((pCfmProgress->sdoAbortCode != 0)
        || (pCfmProgress->error != kEplSuccessful))
    {
        console_printlogadd(" -> SDO Abort=0x%lX, Error=0x%X)\n",
               (ULONG) pCfmProgress->sdoAbortCode,
               pCfmProgress->error);
    }
    else
    {
        console_printlogadd(")\n");
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM result events

The function processes CFM result events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processCfmResultEvent(tEplApiEventType EventType_p,
                                        tEplApiEventArg* pEventArg_p,
                                        void GENERIC* pUserArg_p)
{
    tEplApiEventCfmResult*       pCfmResult = &pEventArg_p->m_CfmResult;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    switch (pCfmResult->m_NodeCommand)
    {
        case kNmtNodeCommandConfOk:
            console_printlog("CFM Result: (Node=%d, ConfOk)\n", pCfmResult->m_uiNodeId);
            break;

        case kNmtNodeCommandConfErr:
            console_printlog("CFM Result: (Node=%d, ConfErr)\n", pCfmResult->m_uiNodeId);
            break;

        case kNmtNodeCommandConfReset:
            console_printlog("CFM Result: (Node=%d, ConfReset)\n", pCfmResult->m_uiNodeId);
            break;

        case kNmtNodeCommandConfRestored:
            console_printlog("CFM Result: (Node=%d, ConfRestored)\n", pCfmResult->m_uiNodeId);
            break;

        default:
            console_printlog("CFM Result: (Node=%d, CfmResult=0x%X)\n", pCfmResult->m_uiNodeId,
                                                                pCfmResult->m_NodeCommand);
            break;
    }
    return kEplSuccessful;
}

#else

//------------------------------------------------------------------------------
/**
\brief  Process SDO events

The function processes SDO events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processSdoEvent(tEplApiEventType EventType_p,
                                  tEplApiEventArg* pEventArg_p,
                                  void GENERIC* pUserArg_p)
{
    tEplSdoComFinished*       pSdo = &pEventArg_p->m_Sdo;
    tEplKernel                ret = kEplSuccessful;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    // SDO transfer finished
    if ((ret = oplk_freeSdoChannel(pSdo->m_SdoAccessType)) != kEplSuccessful)
    {
        return ret;
    }

    if (pSdo->m_SdoComConState == kEplSdoComTransferFinished)
    {   // continue boot-up of CN with NMT command Reset Configuration
        ret = oplk_triggerMnStateChange(pSdo->m_uiNodeId, kNmtNodeCommandConfReset);
    }
    else
    {   // indicate configuration error CN
        ret = oplk_triggerMnStateChange(pSdo->m_uiNodeId, kNmtNodeCommandConfErr);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set default node assignment

Set default node assignment in object dictionary if configuration manager is
not available.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel setDefaultNodeAssignment(void)
{
    tEplKernel  ret = kEplSuccessful;
    DWORD       nodeAssignment;

    nodeAssignment = (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS);    // 0x00000003L
    ret = oplk_writeLocalObject(0x1F81, 0x01, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x02, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x03, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x04, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x05, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x06, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x07, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x08, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x20, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0xFE, &nodeAssignment, sizeof (nodeAssignment));
    ret = oplk_writeLocalObject(0x1F81, 0x6E, &nodeAssignment, sizeof (nodeAssignment));

    nodeAssignment = (EPL_NODEASSIGN_MN_PRES | EPL_NODEASSIGN_NODE_EXISTS);    // 0x00010001L
    ret = oplk_writeLocalObject(0x1F81, 0xF0, &nodeAssignment, sizeof (nodeAssignment));
    return ret;
}
#endif

///\}




