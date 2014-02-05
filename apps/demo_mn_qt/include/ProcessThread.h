/**
********************************************************************************
\file   ProcessThread.h

\brief  Header file for POWERLINK process thread

The file contains the definitions for the POWERLINK process thread.
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

#ifndef _INC_ProcessThread_H_
#define _INC_ProcessThread_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QWidget;
class QString;
class MainWindow;

//------------------------------------------------------------------------------
/**
\brief  ProcessThread class

The class implements the thread used to control the POWERLINK
network (NMT).
*/
//------------------------------------------------------------------------------
class ProcessThread : public QThread
{
    Q_OBJECT

public:
    ProcessThread(MainWindow *pMainWindow_p);

    void            run();
    void            sigEplStatus(int status_p);
    void            sigNmtState(tNmtState State_p);
    void            sigPrintLog(QString log_p);
    void            sigNodeAppeared(int nodeId_p) { emit nodeAppeared(nodeId_p); };
    void            sigNodeDisappeared(int nodeId_p) { emit nodeDisappeared(nodeId_p); };
    void            sigNodeStatus(int nodeId_p, int status_p) { emit nodeStatusChanged(nodeId_p, status_p); };

    tEplApiCbEvent getEventCbFunc(void);

    void            waitForNmtStateOff();
    void            reachedNmtStateOff();

signals:
    void            eplStatusChanged(int status_p);
    void            nmtStateChanged(const QString &strState_p);
    void            nodeAppeared(int nodeId_p);
    void            nodeDisappeared(int nodeId_p);
    void            allNodesRemoved();
    void            nodeStatusChanged(int iNodeId_p, int iStatus_p);
    void            printLog(const QString &strState_p);

private:
    static tOplkError appCbEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);

    tOplkError      processEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processStateChangeEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processErrorWarningEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processSdoEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processHistoryEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processNodeEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processCfmProgressEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      processCfmResultEvent(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);
    tOplkError      setDefaultNodeAssignment(void);

    QMutex          Mutex;
    QWaitCondition  NmtStateOff;
    MainWindow*     pMainWindow;

    int             status;
};

#endif /* _INC_ProcessThread_H_ */
