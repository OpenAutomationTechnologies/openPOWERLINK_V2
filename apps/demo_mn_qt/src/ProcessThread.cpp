/**
********************************************************************************
\file   ProcessThread.cpp

\brief  Implementation of ProcessThread class

This file contains the implementation of the ProcessThread class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <QtGui>
#include <ProcessThread.h>
#include <MainWindow.h>
#include <EventLog.h>

#include <QWidget>
#include <QDateTime>

#include <oplk/debugstr.h>

Q_DECLARE_METATYPE(tSdoComFinished)
Q_DECLARE_METATYPE(tNmtState)

//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------
ProcessThread*  pProcessThread_g;

//============================================================================//
//            S T A T I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Event callback function

The function processes state change events.

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Pointer to union which describes the event in detail
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::appCbEvent(tOplkApiEventType eventType_p,
                                     const tOplkApiEventArg* pEventArg_p,
                                     void* pUserArg_p)
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

\param[in,out]  pMainWindow_p       Pointer to main window object
*/
//------------------------------------------------------------------------------
ProcessThread::ProcessThread(MainWindow* pMainWindow_p)
{
    qRegisterMetaType<tSdoComFinished>();
    qRegisterMetaType<tNmtState>("tNmtState");

    pProcessThread_g = this;

    this->pMainWindow = pMainWindow_p;
    this->pEventLog = new EventLog();

    QObject::connect(pEventLog,
                     SIGNAL(printLog(const QString&)),
                     pMainWindow,
                     SLOT(printLogMessage(const QString&)));

    this->status = -1;
    this->currentNmtState = kNmtGsOff;
    this->fMnActive = false;
}

//------------------------------------------------------------------------------
/**
\brief  Destructor

Destructs a ProcessThread object
*/
//------------------------------------------------------------------------------
ProcessThread::~ProcessThread()
{
    delete this->pEventLog;
}

//------------------------------------------------------------------------------
/**
\brief  thread starting point

run() implements the starting point for the event thread.
*/
//------------------------------------------------------------------------------
void ProcessThread::run()
{
    tOplkError  ret;

    // start process function
    ret = oplk_process();
}

//------------------------------------------------------------------------------
/**
\brief  Signal POWERLINK status

The function signals the POWERLINK status

\param[in]      status_p            POWERLINK status
*/
//------------------------------------------------------------------------------
void ProcessThread::sigNmtStateChanged(tNmtState status_p)
{
    if (status_p != this->status)
    {
        emit nmtStateChanged(status_p);
        this->status = status_p;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Signal a log message

The function signals a log message entry to the text edit of the main window.

\param[in]      rLog_p              Log entry to print.
*/
//------------------------------------------------------------------------------
void ProcessThread::sigPrintLog(const QString& rLog_p)
{
    QString str;

    str.append(QDateTime::currentDateTime().toString("yyyy/MM/dd-hh:mm:ss.zzz"));
    str.append(" - ");
    str.append(rLog_p);

    emit printLog(str);
}

//------------------------------------------------------------------------------
/**
\brief  Signal active MN state

The function signals if it is in an active MN state.

\param[in]      fMnActive_p         If true it is in an active MN state otherwise not.
*/
//------------------------------------------------------------------------------
void ProcessThread::sigMnActive(bool fMnActive_p)
{
    if (fMnActive_p != this->fMnActive)
    {
        emit isMnActive(fMnActive_p);
        this->fMnActive = fMnActive_p;
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
    this->mutex.lock();

    if (this->status > 0)
        this->nmtStateOff.wait(&this->mutex);

    this->mutex.unlock();
}

//------------------------------------------------------------------------------
/**
\brief  Do cleanup in NMT_STATE_OFF

reachedNmtStateOff() cleans up some staff after the NMT state NMT_STATE_OFF
is reached.
*/
//------------------------------------------------------------------------------
void ProcessThread::reachedNmtStateOff()
{
    emit allNodesRemoved();

    this->mutex.lock();
    this->nmtStateOff.wakeAll();
    this->mutex.unlock();
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

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Argument of event
\param[in]      pUserArg_p          User argument
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processEvent(tOplkApiEventType eventType_p,
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

\param  pNmtStateChange_p   Pointer to the state change structure
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                                  void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;
    QString     str;

    UNUSED_PARAMETER(pUserArg_p);

    this->currentNmtState = pNmtStateChange_p->newNmtState;
    this->sigNmtStateChanged(pNmtStateChange_p->newNmtState);

    this->pEventLog->printEvent(pNmtStateChange_p);

    switch (pNmtStateChange_p->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process()
            ret = kErrorShutdown;
            // and unblock DataInDataOutThread
            oplk_freeProcessImage(); //jba do we need it here?

            this->reachedNmtStateOff();
            this->sigMnActive(false);
            break;

        case kNmtGsResetCommunication:
            this->sigMnActive(false);
            break;

        case kNmtGsResetConfiguration:
            this->sigMnActive(false);
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
            this->sigMnActive(false);
            break;

        case kNmtCsOperational:
            this->sigMnActive(false);
            break;

        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
            this->sigMnActive(true);
            break;


        default:
            this->sigMnActive(false);
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
tOplkError ProcessThread::processErrorWarningEvent(const tEventError* pInternalError_p,
                                                   void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pInternalError_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param[in]      pPdoChange_p        Pointer to the PDO change information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                                void* pUserArg_p)
{
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    UINT        varLen;

    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pPdoChange_p);

    for (subIndex = 1; subIndex <= pPdoChange_p->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange_p->mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            this->pEventLog->printMessage(kEventlogLevelError,
                                          kEventlogCategoryObjectDictionary,
                                          "Reading 0x%X/%d failed with %s(0x%X)",
                                          pPdoChange_p->mappParamIndex,
                                          subIndex,
                                          debugstr_getRetValStr(ret),
                                          ret);
            continue;
        }
        this->pEventLog->printPdoMap(pPdoChange_p->mappParamIndex, subIndex, mappObject);
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
tOplkError ProcessThread::processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                              void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pHistoryEntry_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process node events

The function processes node events.

\param[in]      pNode_p             Pointer to the node event structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processNodeEvent(const tOplkApiEventNode* pNode_p,
                                           void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pNode_p);

    // check additional argument
    switch (pNode_p->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventFound:
            pProcessThread_g->sigNodeAppeared(pNode_p->nodeId);
            break;

        case kNmtNodeEventNmtState:
            switch (pNode_p->nmtState)
            {
                case kNmtGsOff:
                case kNmtGsInitialising:
                case kNmtGsResetApplication:
                case kNmtGsResetCommunication:
                case kNmtGsResetConfiguration:
                case kNmtCsNotActive:
                    pProcessThread_g->sigNodeDisappeared(pNode_p->nodeId);
                    break;

                case kNmtCsPreOperational1:
                case kNmtCsPreOperational2:
                case kNmtCsReadyToOperate:
                    pProcessThread_g->sigNodeAppeared(pNode_p->nodeId);
                    pProcessThread_g->sigNodeStatus(pNode_p->nodeId, pNode_p->nmtState);
                    break;

                case kNmtCsOperational:
                    pProcessThread_g->sigNodeAppeared(pNode_p->nodeId);
                    pProcessThread_g->sigNodeStatus(pNode_p->nodeId, pNode_p->nmtState);
                    break;

                case kNmtCsBasicEthernet:
                case kNmtCsStopped:
                default:
                    pProcessThread_g->sigNodeStatus(pNode_p->nodeId, pNode_p->nmtState);
                    break;
            }
            break;

        case kNmtNodeEventError:
            pProcessThread_g->sigNodeStatus(pNode_p->nodeId, -1);
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

\param[in]      pCfmProgress_p      Pointer to the CFM progress information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                                  void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pCfmProgress_p);

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
tOplkError ProcessThread::processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                                void* pUserArg_p)
{
    UNUSED_PARAMETER(pUserArg_p);

    this->pEventLog->printEvent(pCfmResult_p->nodeId,
                                pCfmResult_p->nodeCommand);

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

//------------------------------------------------------------------------------
/**
\brief  Process SDO events

The function processes SDO events.

\param[in]      pSdo_p              Pointer to SDO event information
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError ProcessThread::processSdoEvent(const tSdoComFinished* pSdo_p,
                                          void* pUserArg_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    // SDO transfer finished
    ret = oplk_freeSdoChannel(pSdo_p->sdoAccessType);
    if (ret != kErrorOk)
        return ret;

    if (pSdo_p->sdoComConState == kSdoComTransferFinished)
    {   // continue boot-up of CN with NMT command Reset Configuration
        ret = oplk_triggerMnStateChange(pSdo_p->nodeId, kNmtNodeCommandConfReset);
    }
    else
    {   // indicate configuration error CN
        ret = oplk_triggerMnStateChange(pSdo_p->nodeId, kNmtNodeCommandConfErr);
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
    tOplkError  ret;
    UINT32      nodeAssignment;

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
