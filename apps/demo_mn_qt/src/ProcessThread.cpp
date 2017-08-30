/**
********************************************************************************
\file   ProcessThread.cpp

\brief  Implementation of ProcessThread class

This file contains the implementation of the ProcessThread class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

Q_DECLARE_METATYPE(tSdoComFinished)

//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------
ProcessThread* pProcessThread_g;

//============================================================================//
//            S T A T I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Event callback function

The function processes state change events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------

tOplkError ProcessThread::appCbEvent(tOplkApiEventType eventType_p,
                                     tOplkApiEventArg* pEventArg_p, void* pUserArg_p)
{
    return pProcessThread_g->processEvent(eventType_p, pEventArg_p, pUserArg_p);
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
ProcessThread::ProcessThread(MainWindow* pMainWindow_p)
{
    qRegisterMetaType<tSdoComFinished>();
    pProcessThread_g = this;
    pMainWindow = pMainWindow_p;
    pEventLog = new EventLog();

    QObject::connect(pEventLog, SIGNAL(printLog(const QString&)),
                     pMainWindow, SLOT(printlog(const QString&)));

    status = -1;
    currentNmtState = kNmtGsOff;
    fMnActive = false;
}

//------------------------------------------------------------------------------
/**
\brief  Destructor

Destructs a ProcessThread object
*/
//------------------------------------------------------------------------------
ProcessThread::~ProcessThread()
{
    delete pEventLog;
}

//------------------------------------------------------------------------------
/**
\brief  thread starting point

run() implements the starting point for the event thread.
*/
//------------------------------------------------------------------------------
void ProcessThread::run()
{
    tOplkError          ret;

    // start process function
    ret = oplk_process();
}

//------------------------------------------------------------------------------
/**
\brief  Signal POWERLINK status

The function signals the POWERLINK status

\param  status_p       POWERLINK status
*/
//------------------------------------------------------------------------------
void ProcessThread::sigOplkStatus(int status_p)
{
    if (status_p != status)
    {
        emit oplkStatusChanged(status_p);
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

    switch (State_p)
    {
        case kNmtGsOff:
            strState = "Off";
            break;

        case kNmtGsInitialising:
            strState = "Initializing";
            break;

        case kNmtGsResetApplication:
            strState = "Reset Application";
            break;

        case kNmtGsResetCommunication:
            strState = "Reset Communication";
            break;

        case kNmtGsResetConfiguration:
            strState = "Reset Configuration";
            break;

        case kNmtCsNotActive:
            strState = "CN Not Active";
            break;

        case kNmtCsPreOperational1:
            strState = "CN Pre-Operational 1";
            break;

        case kNmtCsPreOperational2:
            strState = "CN Pre-Operational 2";
            break;

        case kNmtCsReadyToOperate:
            strState = "CN ReadyToOperate";
            break;

        case kNmtCsOperational:
            strState = "CN Operational";
            break;

        case kNmtCsBasicEthernet:
            strState = "CN Basic Ethernet";
            break;

        case kNmtMsNotActive:
            strState = "MN Not Active";
            break;

        case kNmtMsPreOperational1:
            strState = "MN Pre-Operational 1";
            break;

        case kNmtMsPreOperational2:
            strState = "MN Pre-Operational 2";
            break;

        case kNmtMsReadyToOperate:
            strState = "MN ReadyToOperate";
            break;

        case kNmtMsOperational:
            strState = "MN Operational";
            break;

        case kNmtMsBasicEthernet:
            strState = "MN Basic Ethernet";
            break;

        case kNmtRmsNotActive:
            strState = "RMN Not Active";
            break;

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
\brief  Signal active MN state

The function signals if it is in an active MN state.

\param  fMnActive_p       If true it is in an active MN state otherwise not.
*/
//------------------------------------------------------------------------------
void ProcessThread::sigMnActive(bool fMnActive_p)
{
    if (fMnActive_p != fMnActive)
    {
        emit isMnActive(fMnActive_p);
        fMnActive = fMnActive_p;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Wait for NMT state off

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
\brief  Do cleanup in NMT_STATE_OFF

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
\brief  Returns pointer to callback

The function returns the address of the event callback function.

\return Returns the address of event callback function
*/
//------------------------------------------------------------------------------
tOplkApiCbEvent ProcessThread::getEventCbFunc(void)
{
    return appCbEvent;
}

//------------------------------------------------------------------------------
/**
\brief  POWERLINK event callback function

AppCbEvent() implements the openPOWERLINKs event callback function.

\param  eventType_p         Type of event
\param  pEventArg_p         Argument of event
\param  pUserArg_p          User argument
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processEvent(tOplkApiEventType eventType_p,
                                       tOplkApiEventArg* pEventArg_p,
                                       void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

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

        case kOplkApiEventSdo:
            emit sdoFinished(pEventArg_p->sdoInfo);
            break;

        case kOplkApiEventUserDef:
            emit userDefEvent(pEventArg_p->pUserArg);

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

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processStateChangeEvent(tOplkApiEventType eventType_p,
                                                  tOplkApiEventArg* pEventArg_p,
                                                  void* pUserArg_p)
{
    tOplkError                  ret = kErrorOk;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->nmtStateChange;
    QString                     str;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);


    currentNmtState = pNmtStateChange->newNmtState;
    sigNmtState(pNmtStateChange->newNmtState);

    pEventLog->printEvent(pNmtStateChange);

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
            pProcessThread_g->sigOplkStatus(0);

            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process()
            ret = kErrorShutdown;
            // and unblock DataInDataOutThread
            oplk_freeProcessImage(); //jba do we need it here?

            reachedNmtStateOff();
            sigMnActive(false);
            break;

        case kNmtGsResetCommunication:
            pProcessThread_g->sigOplkStatus(1);
            sigMnActive(false);
            break;

        case kNmtGsResetConfiguration:
            sigOplkStatus(1);
            sigMnActive(false);
            break;

        case kNmtCsNotActive:
        case kNmtMsNotActive:
        case kNmtRmsNotActive:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtCsPreOperational1:
        case kNmtMsPreOperational1:
        case kNmtCsPreOperational2:
        case kNmtMsPreOperational2:
        case kNmtCsReadyToOperate:
        case kNmtCsBasicEthernet:
        case kNmtMsBasicEthernet:
            sigOplkStatus(1);
            sigMnActive(false);
            break;

        case kNmtCsOperational:
            sigOplkStatus(2);
            sigMnActive(false);
            break;

        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
            sigOplkStatus(2);
            sigMnActive(true);
            break;


        default:
            sigOplkStatus(-1);
            sigMnActive(false);
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
tOplkError ProcessThread::processErrorWarningEvent(tOplkApiEventType eventType_p,
                                                   tOplkApiEventArg* pEventArg_p,
                                                   void* pUserArg_p)
{
    tEventError*            pInternalError = &pEventArg_p->internalError;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    pEventLog->printEvent(pInternalError);

    return kErrorOk;
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
tOplkError ProcessThread::processPdoChangeEvent(tOplkApiEventType eventType_p,
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


    pEventLog->printEvent(pPdoChange);

    for (subIndex = 1; subIndex <= pPdoChange->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange->mappParamIndex, subIndex, &mappObject, &varLen);
        if (ret != kErrorOk)
        {
            pEventLog->printMessage(kEventlogLevelError, kEventlogCategoryObjectDictionary,
                                    "Reading 0x%X/%d failed with %s(0x%X)",
                                    pPdoChange->mappParamIndex, subIndex, debugstr_getRetValStr(ret), ret);
            continue;
        }
        pEventLog->printPdoMap(pPdoChange->mappParamIndex, subIndex, mappObject);
    }
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
tOplkError ProcessThread::processHistoryEvent(tOplkApiEventType eventType_p,
                                              tOplkApiEventArg* pEventArg_p,
                                              void* pUserArg_p)
{
    tErrHistoryEntry*    pHistoryEntry = &pEventArg_p->errorHistoryEntry;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    pEventLog->printEvent(pHistoryEntry);

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
tOplkError ProcessThread::processNodeEvent(tOplkApiEventType eventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p)
{
    tOplkApiEventNode*   pNode = &pEventArg_p->nodeEvent;
    tOplkError           ret = kErrorOk;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);
    // printf("AppCbEvent(Node): NodeId=%u Event=0x%02X\n",
    //        pEventArg_p->nodeEvent.nodeId, pEventArg_p->nodeEvent.nodeEvent);

    pEventLog->printEvent(pNode);

    // check additional argument
    switch (pEventArg_p->nodeEvent.nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventFound:
            pProcessThread_g->sigNodeAppeared(pEventArg_p->nodeEvent.nodeId);
            break;

        case kNmtNodeEventNmtState:
            switch (pEventArg_p->nodeEvent.nmtState)
            {
                case kNmtGsOff:
                case kNmtGsInitialising:
                case kNmtGsResetApplication:
                case kNmtGsResetCommunication:
                case kNmtGsResetConfiguration:
                case kNmtCsNotActive:
                    pProcessThread_g->sigNodeDisappeared(pEventArg_p->nodeEvent.nodeId);
                    break;

                case kNmtCsPreOperational1:
                case kNmtCsPreOperational2:
                case kNmtCsReadyToOperate:
                    pProcessThread_g->sigNodeAppeared(pEventArg_p->nodeEvent.nodeId);
                    pProcessThread_g->sigNodeStatus(pEventArg_p->nodeEvent.nodeId, 1);
                    break;

                case kNmtCsOperational:
                    pProcessThread_g->sigNodeAppeared(pEventArg_p->nodeEvent.nodeId);
                    pProcessThread_g->sigNodeStatus(pEventArg_p->nodeEvent.nodeId, 2);
                    break;

                case kNmtCsBasicEthernet:
                case kNmtCsStopped:
                default:
                    pProcessThread_g->sigNodeStatus(pEventArg_p->nodeEvent.nodeId, -1);
                    break;
            }
            break;

        case kNmtNodeEventError:
            pProcessThread_g->sigNodeStatus(pEventArg_p->nodeEvent.nodeId, -1);
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
\brief  Process CFM progress events

The function processes CFM progress events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processCfmProgressEvent(tOplkApiEventType eventType_p,
                                                  tOplkApiEventArg* pEventArg_p,
                                                  void* pUserArg_p)
{
    tCfmEventCnProgress*     pCfmProgress = &pEventArg_p->cfmProgress;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    pEventLog->printEvent(pCfmProgress);

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
tOplkError ProcessThread::processCfmResultEvent(tOplkApiEventType eventType_p,
                                                tOplkApiEventArg* pEventArg_p,
                                                void* pUserArg_p)
{
    tOplkApiEventCfmResult*       pCfmResult = &pEventArg_p->cfmResult;

    UNUSED_PARAMETER(eventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    pEventLog->printEvent(pCfmResult->nodeId, pCfmResult->nodeCommand);

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

//------------------------------------------------------------------------------
/**
\brief  Process SDO events

The function processes SDO events.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processSdoEvent(tOplkApiEventType eventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tSdoComFinished*          pSdo = &pEventArg_p->sdoInfo;
    tOplkError                ret = kErrorOk;

    UNUSED_PARAMETER(eventType_p);
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
tOplkError ProcessThread::setDefaultNodeAssignment(void)
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

