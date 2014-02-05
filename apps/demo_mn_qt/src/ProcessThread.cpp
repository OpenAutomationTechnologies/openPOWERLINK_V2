/**
********************************************************************************
\file   ProcessThread.cpp

\brief  Implementation of ProcessThread class

This file contains the implementation of the ProcessThread class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <QWidget>
#include <QThread>
#include <QString>
#include <QMutex>
#include <QWaitCondition>
#include <QDateTime>

#include <MainWindow.h>
#include <ProcessThread.h>
#include <console/console.h>

#include <oplk/debugstr.h>

//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------
ProcessThread    *pProcessThread_g;

#if !defined(CONFIG_INCLUDE_CFM)
// Configuration Manager is not available,
// so store local CycleLen for configuration of remote CNs
static DWORD        cycleLen_g;
#endif

//============================================================================//
//            S T A T I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Event callback function

The function processes state change events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------

tOplkError ProcessThread::appCbEvent(tEplApiEventType EventType_p,
                                        tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p)
{
    return pProcessThread_g->processEvent(EventType_p, pEventArg_p, pUserArg_p);
}

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a ProcessThread object

\param  pMainWindow_p       Pointer to main window object
*/
//------------------------------------------------------------------------------
ProcessThread::ProcessThread(MainWindow *pMainWindow_p)
{
    pProcessThread_g = this;
    pMainWindow = pMainWindow_p;

    status = -1;
}

//------------------------------------------------------------------------------
/**
\brief  thread starting point

run() implements the starting point for the event thread.
*/
//------------------------------------------------------------------------------
void ProcessThread::run()
{
    tOplkError          EplRet;

    // start process function
    EplRet = oplk_process();
}

//------------------------------------------------------------------------------
/**
\brief  Signal POWERLINK status

The function signals the POWERLINK status

\param  status_p       POWERLINK status
*/
//------------------------------------------------------------------------------
void ProcessThread::sigEplStatus(int status_p)
{
    if(status_p != status)
    {
        emit eplStatusChanged(status_p);
        status = status_p;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Signal a log message

The function signals a log message entry to the text edit of the main window.

\param  log_p           Log entry to print.
*/
//------------------------------------------------------------------------------
void ProcessThread::sigPrintLog(QString log_p)
{
    QString str;

    str.append(QDateTime::currentDateTime().toString("yyyy/MM/dd-hh:mm:ss.zzz"));
    str.append(" - ");
    str.append(log_p);

    emit printLog(str);
}

//------------------------------------------------------------------------------
/**
\brief  Signal POWERLINK NMT state

sigNmtState() signals the POWERLINK NMT state

\param  State_p       POWERLINK NMT state
*/
//------------------------------------------------------------------------------
void ProcessThread::sigNmtState(tNmtState State_p)
{
    QString strState;

    switch(State_p)
    {
        case kNmtGsOff:
            strState = "Off"; break;
        case kNmtGsInitialising:
            strState = "Initializing"; break;
        case kNmtGsResetApplication:
            strState = "Reset Application"; break;
        case kNmtGsResetCommunication:
            strState = "Reset Communication"; break;
        case kNmtGsResetConfiguration:
            strState = "Reset Configuration"; break;
        case kNmtCsNotActive:
            strState = "CN Not Active"; break;
        case kNmtCsPreOperational1:
            strState = "CN Pre-Operational 1"; break;
        case kNmtCsPreOperational2:
            strState = "CN Pre-Operational 2"; break;
        case kNmtCsReadyToOperate:
            strState = "CN ReadyToOperate"; break;
        case kNmtCsOperational:
            strState = "CN Operational"; break;
        case kNmtCsBasicEthernet:
            strState = "CN Basic Ethernet"; break;
        case kNmtMsNotActive:
            strState = "MN Not Active"; break;
        case kNmtMsPreOperational1:
            strState = "MN Pre-Operational 1"; break;
        case kNmtMsPreOperational2:
            strState = "MN Pre-Operational 2"; break;
        case kNmtMsReadyToOperate:
            strState = "MN ReadyToOperate"; break;
        case kNmtMsOperational:
            strState = "MN Operational"; break;
        case kNmtMsBasicEthernet:
            strState = "MN Basic Ethernet"; break;
        default:
            strState = "??? (0x";
            strState += QString::number(State_p, 16);
            strState += ")";
            break;
    }

    emit nmtStateChanged(strState);
}

//------------------------------------------------------------------------------
/**
\brief  wait for NMT state off

waitForNmtStateOff() waits until the NMT state NMT_STATE_OFF is reached
*/
//------------------------------------------------------------------------------
void ProcessThread::waitForNmtStateOff()
{
    Mutex.lock();
    if (status > 0)
    {
        NmtStateOff.wait(&Mutex);
    }
    Mutex.unlock();
}

//------------------------------------------------------------------------------
/**
\brief  doing cleanup in NMT_STATE_OFF

reachedNmtStateOff() cleanes up some staff after the NMT state NMT_STATE_OFF
is reached.
*/
//------------------------------------------------------------------------------
void ProcessThread::reachedNmtStateOff()
{
    emit allNodesRemoved();
    Mutex.lock();
    NmtStateOff.wakeAll();
    Mutex.unlock();
}

//------------------------------------------------------------------------------
/**
\brief  returns pointer to callback

The function returns the address of the event callback function.

\return Returns the address of event callback function
*/
//------------------------------------------------------------------------------
tEplApiCbEvent ProcessThread::getEventCbFunc(void)
{
    return appCbEvent;
}

//------------------------------------------------------------------------------
/**
\brief  POWERLINK event callback function

AppCbEvent() implements the openPOWERLINKs event callback function.

\param  EventType_p         Type of event
\param  pEventArg_p         Argument of event
\param  pUserArg_p          User argument
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processEvent(tEplApiEventType EventType_p,
                      tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

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

        case kEplApiEventCfmProgress:
            ret = processCfmProgressEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kEplApiEventCfmResult:
            ret = processCfmResultEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

#if !defined(CONFIG_INCLUDE_CFM)
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
//            P R I V A T E    M E M B E R    F U N C T I O N S               //
//============================================================================//

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
tOplkError ProcessThread::processStateChangeEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p)
{
    tOplkError                  ret = kErrorOk;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->m_NmtStateChange;
#if !defined(CONFIG_INCLUDE_CFM)
    UINT                        varLen;
#endif
    const char                  *string;
    QString                     str;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    sigNmtState(pNmtStateChange->newNmtState);
    string = debugstr_getNmtEventStr(pNmtStateChange->nmtEvent);

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
            pProcessThread_g->sigEplStatus(0);

            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical EPL stack error
            // -> also shut down EplApiProcess()
            ret = kErrorShutdown;
            // and unblock DataInDataOutThread
            oplk_freeProcessImage(); //jba do we need it here?

            sigPrintLog(QString("StateChangeEvent(0x%1) originating event = 0x%2 (%3)")
                     .arg(pNmtStateChange->newNmtState, 0, 16, QLatin1Char('0'))
                     .arg(pNmtStateChange->nmtEvent, 0, 16, QLatin1Char('0'))
                     .arg(debugstr_getNmtEventStr(pNmtStateChange->nmtEvent)));
            reachedNmtStateOff();
            break;

        case kNmtGsResetCommunication:
#if !defined(CONFIG_INCLUDE_CFM)
            ret = setDefaultNodeAssignment();
#endif
            pProcessThread_g->sigEplStatus(1);
            sigPrintLog(QString("StateChangeEvent(0x%1) originating event = 0x%2 (%3)")
                     .arg(pNmtStateChange->newNmtState, 4, 16, QLatin1Char('0'))
                     .arg(pNmtStateChange->nmtEvent, 4, 16, QLatin1Char('0'))
                     .arg(debugstr_getNmtEventStr(pNmtStateChange->nmtEvent)));
            break;

        case kNmtGsResetConfiguration:
#if !defined(CONFIG_INCLUDE_CFM)
        // Configuration Manager is not available,
        // so fetch object 0x1006 NMT_CycleLen_U32 from local OD
        // (in little endian byte order)
        // for configuration of remote CN
            varLen = sizeof(UINT32);
            EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00, &cycleLen_g,
                                      &varLen, kSdoTypeAsnd, NULL);
            if (ret != kErrorOk)
            {   // local OD access failed
                break;
            }
#endif
            sigEplStatus(1);
            sigPrintLog(QString("StateChangeEvent(0x%1) originating event = 0x%2 (%3)")
                     .arg(pNmtStateChange->newNmtState, 4, 16, QLatin1Char('0'))
                     .arg(pNmtStateChange->nmtEvent, 4, 16, QLatin1Char('0'))
                     .arg(debugstr_getNmtEventStr(pNmtStateChange->nmtEvent)));
            break;

        case kNmtCsNotActive:
        case kNmtMsNotActive:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtCsPreOperational1:
        case kNmtMsPreOperational1:
        case kNmtCsPreOperational2:
        case kNmtMsPreOperational2:
        case kNmtCsReadyToOperate:
        case kNmtMsReadyToOperate:
        case kNmtCsBasicEthernet:
        case kNmtMsBasicEthernet:
            sigPrintLog(QString("StateChangeEvent(0x%1) originating event = 0x%2 (%3)")
                     .arg(pNmtStateChange->newNmtState, 4, 16, QLatin1Char('0'))
                     .arg(pNmtStateChange->nmtEvent, 4, 16, QLatin1Char('0'))
                     .arg(debugstr_getNmtEventStr(pNmtStateChange->nmtEvent)));
            sigEplStatus(1);
            break;

        case kNmtCsOperational:
        case kNmtMsOperational:
            sigPrintLog(QString("StateChangeEvent(0x%1) originating event = 0x%2 (%3)")
                     .arg(pNmtStateChange->newNmtState, 4, 16, QLatin1Char('0'))
                     .arg(pNmtStateChange->nmtEvent, 4, 16, QLatin1Char('0'))
                     .arg(debugstr_getNmtEventStr(pNmtStateChange->nmtEvent)));
            sigEplStatus(2);
            break;


        default:
            sigEplStatus(-1);
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
tOplkError ProcessThread::processErrorWarningEvent(tEplApiEventType EventType_p,
                                           tEplApiEventArg* pEventArg_p,
                                           void GENERIC* pUserArg_p)
{
    tEplEventError*         pInternalError = &pEventArg_p->m_InternalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    sigPrintLog(QString("Err/Warn: Source = %1 (0x%2) EplError = %3 (0x%4)")
            .arg(debugstr_getEventSourceStr(pInternalError->m_EventSource))
            .arg(pInternalError->m_EventSource, 2, 16, QLatin1Char('0'))
            .arg(debugstr_getRetValStr(pInternalError->m_EplError))
            .arg(pInternalError->m_EplError, 3, 16, QLatin1Char('0')));

    switch (pInternalError->m_EventSource)
    {
        case kEplEventSourceEventk:
        case kEplEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            sigPrintLog(QString(" OrgSource = %1 %2")
                 .arg(debugstr_getEventSourceStr(pInternalError->m_Arg.m_EventSource))
                 .arg(pInternalError->m_Arg.m_EventSource, 2, 16, QLatin1Char('0')));
            break;

        case kEplEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            sigPrintLog(QString(" val = %1").arg(pInternalError->m_Arg.m_dwArg, 0, 16));
            break;

        default:
            //sigPrintLog(QString("\n"));
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
tOplkError ProcessThread::processHistoryEvent(tEplApiEventType EventType_p,
                                      tEplApiEventArg* pEventArg_p,
                                      void GENERIC* pUserArg_p)
{
    tErrHistoryEntry*    pHistoryEntry = &pEventArg_p->m_ErrHistoryEntry;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    sigPrintLog(QString("HistoryEntry: Type=0x%1 Code=0x%2 (0x%3 %4 %5 %6 %7 %8 %9 %10)")
            .arg(pHistoryEntry->m_wEntryType, 4, 16, QLatin1Char('0'))
            .arg(pHistoryEntry->m_wErrorCode, 4, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[0], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[1], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[2], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[3], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[4], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[5], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[6], 2, 16, QLatin1Char('0'))
            .arg((WORD)pHistoryEntry->m_abAddInfo[7], 2, 16, QLatin1Char('0')));

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
tOplkError ProcessThread::processNodeEvent(tEplApiEventType EventType_p,
                                   tEplApiEventArg* pEventArg_p,
                                   void GENERIC* pUserArg_p)
{
    tEplApiEventNode*   pNode = &pEventArg_p->m_Node;
    tOplkError          EplRet = kErrorOk;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);
    // printf("AppCbEvent(Node): NodeId=%u Event=0x%02X\n",
    //       pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_NodeEvent);
    // check additional argument
    switch (pEventArg_p->m_Node.m_NodeEvent)
    {
        case kNmtNodeEventCheckConf:
#if !defined(CONFIG_INCLUDE_CFM)
            // Configuration Manager is not available,
            // so configure CycleLen (object 0x1006) on CN
            tSdoComConHdl SdoComConHdl;

            // update object 0x1006 on CN
            EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId,
                                       0x1006, 0x00, &cycleLen_g, 4,
                                       kSdoTypeAsnd, NULL);
            if (EplRet == kErrorApiTaskDeferred)
            {   // SDO transfer started
                EplRet = kErrorReject;
            }
            else if (EplRet == kErrorOk)
            {   // local OD access (should not occur)
                printf("AppCbEvent(Node) write to local OD\n");
            }
            else
            {   // error occured

                EplRet = EplApiFreeSdoChannel(SdoComConHdl);
                SdoComConHdl = 0;

                EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId,
                                           0x1006, 0x00, &cycleLen_g, 4,
                                           kSdoTypeAsnd, NULL);
                if (EplRet == kErrorApiTaskDeferred)
                {   // SDO transfer started
                    EplRet = kErrorReject;
                }
                else
                {
                    printf("AppCbEvent(Node): EplApiWriteObject() returned 0x%03X", EplRet);
                }
            }
#endif

            sigPrintLog(QString("Node Event: (Node=%2, CheckConf)")
                        .arg(pEventArg_p->m_Node.m_uiNodeId, 0, 10));
            break;

        case kNmtNodeEventUpdateConf:
            sigPrintLog(QString("Node Event: (Node=%1, UpdateConf)")
                        .arg(pEventArg_p->m_Node.m_uiNodeId, 0, 10));
            break;

        case kNmtNodeEventFound:
            pProcessThread_g->sigNodeAppeared(pEventArg_p->m_Node.m_uiNodeId);
            break;

        case kNmtNodeEventNmtState:
            switch (pEventArg_p->m_Node.m_NmtState)
            {
                case kNmtGsOff:
                case kNmtGsInitialising:
                case kNmtGsResetApplication:
                case kNmtGsResetCommunication:
                case kNmtGsResetConfiguration:
                case kNmtCsNotActive:
                    pProcessThread_g->sigNodeDisappeared(pEventArg_p->m_Node.m_uiNodeId);
                    break;

                case kNmtCsPreOperational1:
                case kNmtCsPreOperational2:
                case kNmtCsReadyToOperate:
                    pProcessThread_g->sigNodeAppeared(pEventArg_p->m_Node.m_uiNodeId);
                    pProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 1);
                    break;

                case kNmtCsOperational:
                    pProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 2);
                    break;

                case kNmtCsBasicEthernet:
                case kNmtCsStopped:
                default:
                    pProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, -1);
                    break;
            }
            break;

        case kNmtNodeEventError:
            pProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, -1);
            sigPrintLog(QString("AppCbEvent (Node=%1): Error = %2 (0x%3)")
                    .arg(pEventArg_p->m_Node.m_uiNodeId, 0, 10)
                    .arg(debugstr_getEmergErrCodeStr(pEventArg_p->m_Node.m_wErrorCode))
                    .arg(pEventArg_p->m_Node.m_wErrorCode, 4, 16, QLatin1Char('0')));
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

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processCfmProgressEvent(tEplApiEventType EventType_p,
                                          tEplApiEventArg* pEventArg_p,
                                          void GENERIC* pUserArg_p)
{
    tCfmEventCnProgress*     pCfmProgress = &pEventArg_p->m_CfmProgress;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    sigPrintLog(QString("CFM Progress: (Node=%1, CFM-Progress: Object 0x%2/%3,  %4/%5 Bytes")
             .arg(pCfmProgress->nodeId, 0, 10)
             .arg(pCfmProgress->objectIndex, 4, 16, QLatin1Char('0'))
             .arg(pCfmProgress->objectSubIndex, 0, 10)
             .arg((ULONG)pCfmProgress->bytesDownloaded, 0, 10)
             .arg((ULONG)pCfmProgress->totalNumberOfBytes, 0, 10));

    if ((pCfmProgress->sdoAbortCode != 0)
        || (pCfmProgress->error != kErrorOk))
    {
        sigPrintLog(QString("             -> SDO Abort=0x%1, Error=0x%2)")
                 .arg((ULONG) pCfmProgress->sdoAbortCode, 0, 16 , QLatin1Char('0'))
                 .arg(pCfmProgress->error, 0, 16, QLatin1Char('0')));
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
tOplkError ProcessThread::processCfmResultEvent(tEplApiEventType EventType_p,
                                        tEplApiEventArg* pEventArg_p,
                                        void GENERIC* pUserArg_p)
{
    tEplApiEventCfmResult*       pCfmResult = &pEventArg_p->m_CfmResult;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    switch (pCfmResult->m_NodeCommand)
    {
        case kNmtNodeCommandConfOk:
            sigPrintLog(QString("CFM Result: (Node=%1, ConfOk)").arg(pCfmResult->m_uiNodeId, 0, 10));
            break;

        case kNmtNodeCommandConfErr:
            sigPrintLog(QString("CFM Result: (Node=%1, ConfErr)").arg(pCfmResult->m_uiNodeId, 0, 10));
            break;

        case kNmtNodeCommandConfReset:
            sigPrintLog(QString("CFM Result: (Node=%1, ConfReset)").arg(pCfmResult->m_uiNodeId, 0, 10));
            break;

        case kNmtNodeCommandConfRestored:
            sigPrintLog(QString("CFM Result: (Node=%1, ConfRestored)").arg(pCfmResult->m_uiNodeId, 0, 10));
            break;

        default:
            sigPrintLog(QString("CFM Result: (Node=%d, CfmResult=0x%X)")
                    .arg(pCfmResult->m_uiNodeId, 0, 10)
                    .arg(pCfmResult->m_NodeCommand, 4, 16, QLatin1Char('0')));
            break;
    }
    return kErrorOk;
}

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
tOplkError ProcessThread::processSdoEvent(tEplApiEventType EventType_p,
                                  tEplApiEventArg* pEventArg_p,
                                  void GENERIC* pUserArg_p)
{
    tSdoComFinished*          pSdo = &pEventArg_p->m_Sdo;
    tOplkError                ret = kErrorOk;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    // SDO transfer finished
    if ((ret = oplk_freeSdoChannel(pSdo->sdoAccessType)) != kErrorOk)
    {
        return ret;
    }

    if (pSdo->sdoComConState == kEplSdoComTransferFinished)
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
tOplkError ProcessThread::setDefaultNodeAssignment(void)
{
    tOplkError  ret = kErrorOk;
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



