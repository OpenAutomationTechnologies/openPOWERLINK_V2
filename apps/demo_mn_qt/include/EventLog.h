/**
********************************************************************************
\file   EventLog.h

\brief  Header file for event logger class

This file contains the definitions of the event logger class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_EventLog_H_
#define _INC_EventLog_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QObject>
#include <QString>

#include <oplk/oplk.h>
#include <eventlog/eventlogstring.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


const   UINT32      EVENTLOG_MAX_LENGTH = 256;

//------------------------------------------------------------------------------
// class definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  EventLog class

The class implements an event logger.
*/
//------------------------------------------------------------------------------
class EventLog : public QObject
{
    Q_OBJECT

public:
    EventLog();
    void printEvent(tOplkApiEventNode* pNodeEvent_p);
    void printEvent(tErrHistoryEntry* pHistory_p);
    void printEvent(tEventError* pError_p);
    void printEvent(tEventNmtStateChange* pNmtStateChange_p);
    void printEvent(tOplkApiEventPdoChange* pPdoChange_p);
    void printEvent(tCfmEventCnProgress* pCfmProgress_p);
    void printEvent(UINT nodeId_p, tNmtNodeCommand nodeCommand_p);
    void printMessage(tEventlogLevel level_p, tEventlogCategory category_p, const char* fmt_p, ...);
    void printPdoMap(UINT16 mapObject_p, UINT8 subIndex_p, UINT64 mapping_p);

signals:
    void printLog(const QString &strState_p);

private:
    UINT32              filterLevel;            ///< The level filter. It contains a bitmask with all
                                                ///< levels to be printed.
    UINT32              filterCategory;         ///< The category filter. It contains a bitmask with all
                                                ///< categories to be printed.
    tEventlogFormat     logFormat;
};

#endif /* _INC_EventLog_H_ */
