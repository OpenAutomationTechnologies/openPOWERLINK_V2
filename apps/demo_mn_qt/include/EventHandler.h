/**
********************************************************************************
\file   EventHandler.h

\brief  Header file for the openPOWERLINK event handler

The file contains the definitions for the openPOWERLINK event handler.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_demo_EventHandler_H_
#define _INC_demo_EventHandler_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QObject>
#include <QString>
#include <QMutex>
#include <QWaitCondition>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------
class MainWindow;
class EventLog;

//------------------------------------------------------------------------------
/**
\brief  EventHandler class

The class implements the event handler of the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
class EventHandler : public QObject
{
    Q_OBJECT

public:
    static tOplkError appCbEvent(tOplkApiEventType eventType_p,
                                 const tOplkApiEventArg* pEventArg_p,
                                 void* pUserArg_p);
    EventHandler(EventLog* pEventLog_p);
    void awaitNmtGsOff();

signals:
    void nmtStateChanged(tNmtState status_p);
    void nodeStatusChanged(unsigned int nodeId_p, tNmtState status_p);
    void isMnActive(bool fMnActive_p);

    void userDefEvent(void* pUserArg_p);
    void sdoFinished(tSdoComFinished sdoInfo_p);

protected:
    virtual tOplkError userDefinedEvent(void* pUserArg_p);
    virtual tOplkError nmtStateChangeEvent(const tEventNmtStateChange& nmtStateChange_p);
    virtual tOplkError criticalErrorEvent(const tEventError& internalError_p);
    virtual tOplkError warningEvent(const tEventError& internalError_p);
    virtual tOplkError historyEntryEvent(const tErrHistoryEntry& historyEntry_p);
    virtual tOplkError nodeEvent(const tOplkApiEventNode& nodeEvent_p);
    virtual tOplkError bootEvent(const tOplkApiEventBoot& bootEvent_p);
    virtual tOplkError sdoCommandFinishedEvent(const tSdoComFinished& sdoEvent_p);
    virtual tOplkError obdAccessEvent(const tObdCbParam& obdEvent_p);
    virtual tOplkError cfmProgressEvent(const tCfmEventCnProgress& cfmProgress_p);
    virtual tOplkError cfmResultEvent(const tOplkApiEventCfmResult& cfmResult_p);
    virtual tOplkError asndReceivedEvent(const tOplkApiEventRcvAsnd& receivedAsnd_p);
    virtual tOplkError pdoChangeEvent(const tOplkApiEventPdoChange& pdoChange_p);
    virtual tOplkError presReceivedEvent(const tOplkApiEventReceivedPres& receivedPres_p);
    virtual tOplkError nonPlkReceivedEvent(const tOplkApiEventReceivedNonPlk& receivedEth_p);
    virtual tOplkError defaultGwChangedEvent(const tOplkApiEventDefaultGwChange& defaultGw_p);
    virtual tOplkError sdoComReceived(const tOplkApiEventReceivedSdoCom& receivedSdoCom_p);
    virtual tOplkError sdoSeqReceived(const tOplkApiEventReceivedSdoSeq& receivedSdoSeq_p);

private:
    void sigMnActive(bool fMnActive_p);

    EventLog*       pEventLog;
    bool            fMnActive;
    QMutex          mutex;
    QWaitCondition  nmtGsOff;
};

#endif /* _INC_demo_EventHandler_H_ */
