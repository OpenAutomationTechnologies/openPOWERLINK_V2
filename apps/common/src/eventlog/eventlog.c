/**
********************************************************************************
\file   eventlog.c

\brief  openPOWERLIK application event logger

This module implements an event logger for openPOWERLINK applications. It can
print system events in a defined format. It allows filtering of events via
categories and levels.

The module is intended to be used by openPOWERLINK C applications to print
uniform log output. The module uses the eventlog string module to generate the
log message strings.

\ingroup module_app_eventlog
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#include <oplk/debugstr.h>
#include "eventlog.h"

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
#define EVENTLOG_MAX_LENGTH   256               ///< Maximum log message length

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
* Instance of event log module
*
* This structure contains all valid information of an event log instance.
*/
typedef struct
{
    tEventlogFormat     format;                 ///< The format of the log output
                                                ///< (kEventlogFormatReadable or kEventlogFormatParsable)
    tEventlogOutputCb   pfnOutput;              ///< Function pointer to the output function
    UINT32              filterLevel;            ///< The level filter. It contains a bitmask with all
                                                ///< levels to be printed.
    UINT32              filterCategory;         ///< The category filter. It contains a bitmask with all
                                                ///< categories to be printed.
} tEventlogInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tEventlogInstance eventlogInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void printMessage(char* message_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize event logger

The function initializes the event logger. It sets up the logging filter and
the logging format. A callback function that is used by the logger for
printing log messages must be provided.

\param  format_p            Specifies the logging format.
\param  filterLevel_p       Sets the filter for the logging level.
\param  filterCategory_p    Sets the filter for the logging category.
\param  pfnOutput_p         A callback function to be provided to the logger
                            for printing log messages.
\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_init(tEventlogFormat format_p, UINT32 filterLevel_p,
                   UINT32 filterCategory_p, tEventlogOutputCb pfnOutput_p)
{
    eventlogInstance_l.format = format_p;
    eventlogInstance_l.pfnOutput = pfnOutput_p;
    eventlogInstance_l.filterLevel = filterLevel_p;
    eventlogInstance_l.filterCategory = filterCategory_p;
}

//------------------------------------------------------------------------------
/**
\brief  Print a generic log message

The function can be used to print a generic log message. You can specify a
printf style format string for printing.

\param  level_p             The log level to be used for the output.
\param  category_p          The log category to be used for the output.
\param  fmt_p               The printf style format string which specifies the
                            message.
\param  ...                 Required arguments according to the format string.
\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printMessage(tEventlogLevel level_p, tEventlogCategory category_p,
                           char* fmt_p, ...)
{
    va_list             arglist;
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << level_p)) &&
          (eventlogInstance_l.filterCategory & (1 << category_p))))
        return;

    va_start(arglist, fmt_p);
    eventlog_createMessageString(logMsg, EVENTLOG_MAX_LENGTH, level_p, category_p,
                                 fmt_p, arglist);
    va_end(arglist);

    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Print a node event

The function is used to log openPOWERLINK node events.

\param  pNodeEvent_p        The node event information to be logged.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printNodeEvent(tOplkApiEventNode* pNodeEvent_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryNodeEvent))))
        return;

    eventlog_createNodeEventString(pNodeEvent_p, eventlogInstance_l.format, logMsg,
                                   EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Print a state change event

The function is used to log openPOWERLINK state change events.

\param  pStateChangeEvent_p     The state change event information to be logged.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printStateEvent(tEventNmtStateChange* pStateChangeEvent_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryStateChangeEvent))))
        return;

    eventlog_createStateEventString(pStateChangeEvent_p, eventlogInstance_l.format,
                                    logMsg, EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Print a CFM result event

The function is used to log openPOWERLINK CFM result events.

\param  nodeId_p            The node ID of the node specified in the received
                            event.
\param  nodeCommand_p       The nodeCommand of the received CFM result event.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printCfmResultEvent(UINT8 nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryCfmResultEvent))))
        return;

    eventlog_createCfmResultEventString(nodeId_p, nodeCommand_p, eventlogInstance_l.format,
                                        logMsg, EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Print a CFM progress event

The function is used to log openPOWERLINK CFM progress events.

\param  pProgress_p         The information about the received event.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printCfmProgressEvent(tCfmEventCnProgress* pProgress_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryCfmProgressEvent))))
        return;

    eventlog_createCfmProgressEventString(pProgress_p, eventlogInstance_l.format,
                                          logMsg, EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Print a PDO change event

The function is used to log openPOWERLINK PDO change events.

\param  pPdoChange_p            The information about the received event.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printPdoEvent(tOplkApiEventPdoChange* pPdoChange_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryPdoEvent))))
        return;

    eventlog_createPdoEventString(pPdoChange_p, eventlogInstance_l.format, logMsg,
                                  EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Log a PDO mapping information

The function is used to log openPOWERLINK PDO mappings.

\param  mapObject_p     The object index of the mapping object.
\param  subIndex_p      The sub-index of the mapping object.
\param  mapping_p       The 64bit mapping information.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printPdoMap(UINT16 mapObject_p, UINT8 subIndex_p, UINT64 mapping_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelInfo)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryPdoMap))))
        return;

    eventlog_createPdoMapString(mapObject_p, subIndex_p, mapping_p,
                              eventlogInstance_l.format, logMsg, EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Log a history event

The function is used to log openPOWERLINK history events.

\param  pHistory_p      The information about the history event to log.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printHistoryEvent(tErrHistoryEntry* pHistory_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryHistoryEvent))))
        return;

    eventlog_createHistoryEventString(pHistory_p, eventlogInstance_l.format, logMsg,
                                      EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//------------------------------------------------------------------------------
/**
\brief  Log an error event

The function is used to log openPOWERLINK error/warning events.

\param  pError_p      The information about the error event to log.

\ingroup module_app_eventlog
*/
//------------------------------------------------------------------------------
void eventlog_printErrorEvent(tEventError* pError_p)
{
    char                logMsg[EVENTLOG_MAX_LENGTH];

    if (!((eventlogInstance_l.filterLevel & (1 << kEventlogLevelEvent)) &&
          (eventlogInstance_l.filterCategory & (1 << kEventlogCategoryErrorEvent))))
        return;

    eventlog_createErrorEventString(pError_p, eventlogInstance_l.format, logMsg,
                                    EVENTLOG_MAX_LENGTH);
    printMessage(logMsg);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Print event log message

The function prints the eventlog message by calling the registered printing
callback function.

\param  message_p       Event log message to be printed.
*/
//------------------------------------------------------------------------------
static void printMessage(char* message_p)
{
    if (eventlogInstance_l.pfnOutput != NULL)
    {
        eventlogInstance_l.pfnOutput(message_p);
        eventlogInstance_l.pfnOutput("\n");
    }
}

///\}
