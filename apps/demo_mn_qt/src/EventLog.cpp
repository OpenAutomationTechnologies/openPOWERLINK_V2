/**
********************************************************************************
\file   EventLog.cpp

\brief  Implementation of the event logger class

This file implements the data event logger class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <EventLog.h>

#include <cstdarg>

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
const size_t EventLog::EVENTLOG_MAX_LENGTH = 256;

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Default Constructor

The function implements the default constructor of the EventLog class.

\param[in]      filterLevel_p       The filter level of the log messages.
\param[in]      filterCategory_p    The filter category of the log messages.
\param[in]      logFormat_p         The format of the logging.
*/
//------------------------------------------------------------------------------
EventLog::EventLog(tEventlogLevel filterLevel_p,
                   tEventlogCategory filterCategory_p,
                   tEventlogFormat logFormat_p) :
    filterLevel(filterLevel_p),
    filterCategory(filterCategory_p),
    logFormat(logFormat_p)
{
}

//------------------------------------------------------------------------------
/**
\brief  Print a node event

The function is used to log openPOWERLINK node events.

\param[in]      nodeEvent_p         The node event information to be logged.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tOplkApiEventNode& nodeEvent_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryNodeEvent))))
        return;

    eventlog_createNodeEventString(&nodeEvent_p,
                                   this->logFormat,
                                   cstring,
                                   EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a history event

The function is used to log openPOWERLINK history events.

\param[in]      history_p           The information about the history event to log.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tErrHistoryEntry& history_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryHistoryEvent))))
        return;

    eventlog_createHistoryEventString(&history_p,
                                      this->logFormat,
                                      cstring,
                                      EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print an error event

The function is used to log openPOWERLINK error/warning events.

\param[in]      error_p             The information about the error event to log.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tEventError& error_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryErrorEvent))))
        return;

    eventlog_createErrorEventString(&error_p,
                                    this->logFormat,
                                    cstring,
                                    EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a state change event

The function is used to log openPOWERLINK state change events.

\param[in]      stateChangeEvent_p  The state change event information to be logged.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tEventNmtStateChange& nmtStateChange_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryStateChangeEvent))))
        return;

    eventlog_createStateEventString(&nmtStateChange_p,
                                    this->logFormat,
                                    cstring,
                                    EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a PDO change event

The function is used to log openPOWERLINK PDO change events.

\param[in]      pdoChange_p        The information about the received event.

*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tOplkApiEventPdoChange& pdoChange_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryPdoEvent))))
        return;

    eventlog_createPdoEventString(&pdoChange_p,
                                  this->logFormat,
                                  cstring,
                                  EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a CFM progress event

The function is used to log openPOWERLINK CFM progress events.

\param[in]      cfmProgress_p       The information about the received event.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(const tCfmEventCnProgress& cfmProgress_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryCfmProgressEvent))))
        return;

    eventlog_createCfmProgressEventString(&cfmProgress_p,
                                          this->logFormat,
                                          cstring,
                                          EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a CFM result event

The function is used to log openPOWERLINK CFM result events.

\param[in]      nodeId_p            The node ID of the node specified in the received
                                    event.
\param[in]      nodeCommand_p       The nodeCommand of the received CFM result event.
*/
//------------------------------------------------------------------------------
void EventLog::printEvent(UINT nodeId_p,
                          tNmtNodeCommand nodeCommand_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelEvent)) &&
          (this->filterCategory & (1 << kEventlogCategoryCfmResultEvent))))
        return;

    eventlog_createCfmResultEventString(nodeId_p,
                                        nodeCommand_p,
                                        this->logFormat,
                                        cstring,
                                        EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}

//------------------------------------------------------------------------------
/**
\brief  Print a generic log message

The function can be used to print a generic log message. You can specify a
printf style format string for printing.

\param[in]      level_p             The log level to be used for the output.
\param[in]      category_p          The log category to be used for the output.
\param[in]      fmt_p               The printf style format string which specifies the
                                    message.
\param[in]      ...                 Required arguments according to the format string.
*/
//------------------------------------------------------------------------------
void EventLog::printMessage(tEventlogLevel level_p,
                            tEventlogCategory category_p,
                            const char* fmt_p,
                            ...)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];
    va_list arglist;

    if (!((this->filterLevel & (1 << level_p)) &&
          (this->filterCategory & (1 << category_p))))
        return;

    va_start(arglist, fmt_p);
    eventlog_createMessageString(cstring,
                                 EventLog::EVENTLOG_MAX_LENGTH,
                                 level_p,
                                 category_p,
                                 fmt_p,
                                 arglist);
    va_end(arglist);

    emit printLog(QString::fromLatin1(cstring, EventLog::EVENTLOG_MAX_LENGTH));
}

//------------------------------------------------------------------------------
/**
\brief  Log a PDO mapping information

The function is used to log openPOWERLINK PDO mappings.

\param[in]      mapObject_p         The object index of the mapping object.
\param[in]      subIndex_p          The sub-index of the mapping object.
\param[in]      mapping_p           The 64bit mapping information.
*/
//------------------------------------------------------------------------------
void EventLog::printPdoMap(UINT16 mapObject_p,
                           UINT8 subIndex_p,
                           UINT64 mapping_p)
{
    char    cstring[EventLog::EVENTLOG_MAX_LENGTH];

    if (!((this->filterLevel & (1 << kEventlogLevelInfo)) &&
          (this->filterCategory & (1 << kEventlogCategoryPdoMap))))
        return;

    eventlog_createPdoMapString(mapObject_p,
                                subIndex_p, mapping_p,
                                this->logFormat,
                                cstring,
                                EventLog::EVENTLOG_MAX_LENGTH);

    emit printLog(QString::fromLatin1(cstring));
}
