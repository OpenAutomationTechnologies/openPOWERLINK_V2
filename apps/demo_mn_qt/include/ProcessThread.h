/**
********************************************************************************
\file   ProcessThread.h

\brief  Header file for POWERLINK process thread

The file contains the definitions for the POWERLINK process thread.
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
#ifndef _INC_demo_ProcessThread_H_
#define _INC_demo_ProcessThread_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <QThread>
#include <QString>
#include <QMutex>
#include <QWaitCondition>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class QWidget;
class MainWindow;
class EventLog;

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
    ProcessThread(EventLog* pEventLog_p);

    void              run();
    void              sigNmtStateChanged(tNmtState status_p);
    void              sigMnActive(bool fMnActive_p);
    void              sigNodeStatus(int nodeId_p, tNmtState status_p) { emit nodeStatusChanged(nodeId_p, status_p); };

    tOplkApiCbEvent   getEventCbFunc(void);

    void              waitForNmtStateOff();
    void              reachedNmtStateOff();

signals:
    void              nmtStateChanged(tNmtState status_p);
    void              isMnActive(bool fMnActive_p);
    void              nodeStatusChanged(int nodeId_p, tNmtState status_p);
    void              userDefEvent(void* pUserArg_p);
    void              sdoFinished(tSdoComFinished sdoInfo_p);

private:
    static tOplkError appCbEvent(tOplkApiEventType eventType_p,
                                 const tOplkApiEventArg* pEventArg_p,
                                 void* pUserArg_p);

    tOplkError processEvent(tOplkApiEventType eventType_p,
                            const tOplkApiEventArg* pEventArg_p,
                            void* pUserArg_p);
    tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                       void* pUserArg_p);
    tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                        void* pUserArg_p);
    tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                     void* pUserArg_p);
    tOplkError processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                   void* pUserArg_p);
    tOplkError processNodeEvent(const tOplkApiEventNode* pNode_p,
                                void* pUserArg_p);
    tOplkError processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                       void* pUserArg_p);
    tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                     void* pUserArg_p);
    tOplkError processSdoEvent(const tSdoComFinished* pSdo_p,
                               void* pUserArg_p);
    tOplkError setDefaultNodeAssignment(void);

    QMutex            mutex;
    QWaitCondition    nmtStateOff;
    EventLog*         pEventLog;

    int               status;
    tNmtState         currentNmtState;
    bool              fMnActive;
};

#endif /* _INC_demo_ProcessThread_H_ */
