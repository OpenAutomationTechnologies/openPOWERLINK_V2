/**
********************************************************************************
\file   eventlogstring.c

\brief  openPOWERLIK application event logger - string functions

This module implements the string functions for the event logger. It creates
the output strings for the miscellaneous events which can be printed by the
event logger. The strings can be created according to the specified output
format. Two output formats are available. kEventlogFormatParsable specifies
a format that's intended to be parsed by evaluation and test tools.
kEventlogFormatReadable is intended to be read by humans.

\ingroup module_app_eventlogstring
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "eventlogstring.h"
#include <oplk/debugstr.h>
#include <stdio.h>
#include <time.h>

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
/**
* String values for log levels
*
* The variable contains an array with all log level strings.
*/
static const char* const sEventlogLevel_l[] =
{
    "FATAL   ",
    "ERROR   ",
    "WARNING ",
    "INFO    ",
    "DEBUG   ",
    "EVENT   ",
};

/**
* String values for log categories
*
* The variable contains an array with all log category strings.
*/
static const char* const sEventlogCategory_l[] =
{
    // Generic
    "GENERIC       ",
    "CONTROL       ",
    "APP           ",
    "PDOMAP        ",
    "OBDICT        ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    //Events
    "NODE          ",
    "CFM_RESULT    ",
    "CFM_PROGRESS  ",
    "STATE_CHANGE  ",
    "PDO_CHANGE    ",
    "HISTORY       ",
    "ERROR         ",
    "SDO           ",
    "USER          ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
};

/**
* String values for node events
*
* The variable contains an array with all node event strings.
*/
static const char* const sLogEventNodeEvent_l[] =
{
    "NmtNodeEventFound",
    "NmtNodeEventUpdateSw",
    "NmtNodeEventCheckConf",
    "NmtNodeEventUpdateConf",
    "NmtNodeEventVerifyConf",
    "NmtNodeEventReadyToStart",
    "NmtNodeEventNmtState",
    "NmtNodeEventError",
    "NmtNodeEventAmniReceived",
    "NmtNodeEventConfDone"
};

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static size_t createTimeString(char* string_p,
                               size_t strlen_p);
static size_t createLogLevelString(char* string_p,
                                   size_t strlen_p,
                                   tEventlogLevel logLevel_p);
static size_t createLogCategoryString(char* string_p,
                                      size_t strlen_p,
                                      tEventlogCategory logCategory_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a generic log message

The function can be used to print a generic log message. You can specify a
printf style format string for printing.

\param[out]     message_p           Pointer to the message which should be printed.
\param[in]      messageSize_p       Size of the message string to be printed.
\param[in]      level_p             The log level to be used for the output.
\param[in]      category_p          The log category to be used for the output.
\param[in]      fmt_p               The printf style format string which specifies the
                                    message.
\param[in]      arg_p               Required arguments according to the format string.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createMessageString(char* message_p,
                                  size_t messageSize_p,
                                  tEventlogLevel level_p,
                                  tEventlogCategory category_p,
                                  const char* fmt_p,
                                  va_list arg_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, level_p);
    len += createLogCategoryString(message_p + len, messageSize_p - len, category_p);

    len += vsnprintf(message_p + len, messageSize_p - len, fmt_p, arg_p);
}

//------------------------------------------------------------------------------
/**
\brief  Create a node event string

The function is used to log openPOWERLINK node events.

\param[in]      pNodeEvent_p        The node event information to be logged.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createNodeEventString(const tOplkApiEventNode* pNodeEvent_p,
                                    tEventlogFormat format_p,
                                    char* message_p,
                                    size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryNodeEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "%03d ",
                            pNodeEvent_p->nodeId);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "EVENT:0x%02X ",
                            pNodeEvent_p->nodeEvent);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "STATE:0x%02X ",
                            pNodeEvent_p->nmtState);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "ERROR:0x%04X MANDATORY:%d",
                            pNodeEvent_p->errorCode,
                            pNodeEvent_p->fMandatory);
            break;

        case kEventlogFormatReadable:
            len = snprintf(message_p + len,
                           messageSize_p,
                           "Node=%3u, %s State:%s",
                           pNodeEvent_p->nodeId,
                           sLogEventNodeEvent_l[pNodeEvent_p->nodeEvent],
                           debugstr_getNmtStateStr(pNodeEvent_p->nmtState));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Print a state change event

The function is used to log openPOWERLINK state change events.

\param[in]      pStateChangeEvent_p The state change event information to be logged.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createStateEventString(const tEventNmtStateChange* pStateChangeEvent_p,
                                     tEventlogFormat format_p,
                                     char* message_p,
                                     size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryStateChangeEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "OLDSTATE:0x%03X NEWSTATE:0x%03X EVENT:0x%02X",
                            pStateChangeEvent_p->oldNmtState,
                            pStateChangeEvent_p->newNmtState,
                            pStateChangeEvent_p->nmtEvent);
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "%s->%s Originating event:%s",
                            debugstr_getNmtStateStr(pStateChangeEvent_p->oldNmtState),
                            debugstr_getNmtStateStr(pStateChangeEvent_p->newNmtState),
                            debugstr_getNmtEventStr(pStateChangeEvent_p->nmtEvent));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create a CFM result event string

The function is used to log openPOWERLINK CFM result events.

\param[in]      nodeId_p            The node ID of the node specified in the received
                                    event.
\param[in]      nodeCommand_p       The nodeCommand of the received CFM result event.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createCfmResultEventString(UINT8 nodeId_p,
                                         tNmtNodeCommand nodeCommand_p,
                                         tEventlogFormat format_p,
                                         char* message_p,
                                         size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryCfmResultEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len, messageSize_p - len, "%03d ", nodeId_p);
            len += snprintf(message_p + len, messageSize_p - len, "0x%02X ", nodeCommand_p);
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "Node=%3d, %s", nodeId_p,
                            debugstr_getNmtNodeCommandTypeStr(nodeCommand_p));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create a CFM progress event

The function is used to log openPOWERLINK CFM progress events.

\param[in]      pProgress_p         The information about the CFM progress event.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createCfmProgressEventString(const tCfmEventCnProgress* pProgress_p,
                                           tEventlogFormat format_p,
                                           char* message_p,
                                           size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryCfmProgressEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "%03d ",
                            pProgress_p->nodeId);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "OBJECT:0x%04X/%03d ",
                            pProgress_p->objectIndex,
                            pProgress_p->objectSubIndex);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "SIZE:%05zu DOWNLOADED:%04zu ",
                            pProgress_p->totalNumberOfBytes,
                            pProgress_p->bytesDownloaded);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "ABORTCODE:0x%08X ERROR:0x%04X ",
                            pProgress_p->sdoAbortCode,
                            pProgress_p->error);
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "Node=%3u, Object 0x%X/%03u, ",
                            pProgress_p->nodeId,
                            pProgress_p->objectIndex,
                            pProgress_p->objectSubIndex);

            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "%4zu/%4zu Bytes",
                            pProgress_p->bytesDownloaded,
                            pProgress_p->totalNumberOfBytes);

            if ((pProgress_p->sdoAbortCode != 0) ||
                (pProgress_p->error != kErrorOk))
            {
                len += snprintf(message_p + len,
                                messageSize_p - len,
                                " -> SDO Abort=%s(0x%08X), Error=0x%04X",
                                debugstr_getAbortCodeStr(pProgress_p->sdoAbortCode),
                                pProgress_p->sdoAbortCode,
                                pProgress_p->error);
            }
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create a PDO change event string

The function creates a PDO change event string.

\param[in]      pPdoChange_p        The information about the PDO change event.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createPdoEventString(const tOplkApiEventPdoChange* pPdoChange_p,
                                   tEventlogFormat format_p,
                                   char* message_p,
                                   size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryPdoEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            (pPdoChange_p->fTx ? "TPDO " : "RPDO "));
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "MAPOBJ:0x%04x NODE:%03d NUM:%d ",
                            pPdoChange_p->mappParamIndex,
                            pPdoChange_p->nodeId,
                            pPdoChange_p->mappObjectCount);
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            (pPdoChange_p->fActivated ? "ACTIVATE" : "DELETE"));
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "%sPDO = 0x%X to node %d with %d objects %s",
                            (pPdoChange_p->fTx ? "T" : "R"),
                            pPdoChange_p->mappParamIndex,
                            pPdoChange_p->nodeId,
                            pPdoChange_p->mappObjectCount,
                            (pPdoChange_p->fActivated ? "activated" : "deleted"));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create a PDO mapping information string

The function creates a PDO mapping information string.

\param[in]      mapObject_p         The object index of the mapping object.
\param[in]      subIndex_p          The sub-index of the mapping object.
\param[in]      mapping_p           The 64 bit mapping information.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createPdoMapString(UINT16 mapObject_p,
                                 UINT8 subIndex_p,
                                 UINT64 mapping_p,
                                 tEventlogFormat format_p,
                                 char* message_p,
                                 size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelInfo);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryPdoMap);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "MAPOBJ:0x%04x/%d OBJECT:0x%04x/%d OFFSET:%d LEN:%d",
                            mapObject_p,
                            subIndex_p,
                            (UINT32)(mapping_p & 0x000000000000FFFFULL),
                            (UINT32)((mapping_p & 0x0000000000FF0000ULL) >> 16),
                            (UINT16)((mapping_p & 0x0000FFFF00000000ULL) >> 32),
                            (UINT16)((mapping_p & 0xFFFF000000000000ULL) >> 48));
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "  0x%04x/%d Mapped object 0x%X/%3d", mapObject_p, subIndex_p,
                            (UINT32)(mapping_p & 0x000000000000FFFFULL),
                            (UINT32)((mapping_p & 0x0000000000FF0000ULL) >> 16));
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create a history event string

The function creates a history event string.

\param[in]      pHistory_p          The information about the history event.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createHistoryEventString(const tErrHistoryEntry* pHistory_p,
                                       tEventlogFormat format_p,
                                       char* message_p,
                                       size_t messageSize_p)
{
    size_t  len;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryHistoryEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "TYPE:0x%04X CODE:0x%04X INFO:0x%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                            pHistory_p->entryType,
                            pHistory_p->errorCode,
                            (UINT16)pHistory_p->aAddInfo[0],
                            (UINT16)pHistory_p->aAddInfo[1],
                            (UINT16)pHistory_p->aAddInfo[2],
                            (UINT16)pHistory_p->aAddInfo[3],
                            (UINT16)pHistory_p->aAddInfo[4],
                            (UINT16)pHistory_p->aAddInfo[5],
                            (UINT16)pHistory_p->aAddInfo[6],
                            (UINT16)pHistory_p->aAddInfo[7]);
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "HistoryEntry: Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)",
                            pHistory_p->entryType,
                            pHistory_p->errorCode,
                            (UINT16)pHistory_p->aAddInfo[0],
                            (UINT16)pHistory_p->aAddInfo[1],
                            (UINT16)pHistory_p->aAddInfo[2],
                            (UINT16)pHistory_p->aAddInfo[3],
                            (UINT16)pHistory_p->aAddInfo[4],
                            (UINT16)pHistory_p->aAddInfo[5],
                            (UINT16)pHistory_p->aAddInfo[6],
                            (UINT16)pHistory_p->aAddInfo[7]);
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Create an error event string

The function creates a string for openPOWERLINK error/warning events.

\param[in]      pError_p            The information about the error event.
\param[in]      format_p            The format of the created string (kEventlogFormatParsable
                                    or kEventlogFormatReadable).
\param[out]     message_p           Buffer to store the created message string.
\param[in]      messageSize_p       Size of the message buffer.

\ingroup module_app_eventlogstring
*/
//------------------------------------------------------------------------------
void eventlog_createErrorEventString(const tEventError* pError_p,
                                     tEventlogFormat format_p,
                                     char* message_p,
                                     size_t messageSize_p)
{
    size_t  len;
    UINT    i;

    len =  createTimeString(message_p, messageSize_p);
    len += createLogLevelString(message_p + len, messageSize_p - len, kEventlogLevelEvent);
    len += createLogCategoryString(message_p + len, messageSize_p - len, kEventlogCategoryHistoryEvent);

    switch (format_p)
    {
        case kEventlogFormatParsable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "SOURCE:0x%02X ERROR:0x%03X ARG",
                            pError_p->eventSource,
                            pError_p->oplkError);

            for (i = 0; i < sizeof(pError_p->errorArg); ++i)
            {
                len += snprintf(message_p + len,
                                messageSize_p - len,
                                ":0x%02X",
                                *((UINT8*)&pError_p->errorArg + i));
            }
            break;

        case kEventlogFormatReadable:
            len += snprintf(message_p + len,
                            messageSize_p - len,
                            "Source = %s (0x%02X) OplkError = %s (0x%03X) ",
                            debugstr_getEventSourceStr(pError_p->eventSource),
                            pError_p->eventSource,
                            debugstr_getRetValStr(pError_p->oplkError),
                            pError_p->oplkError);

            // check additional argument
            switch (pError_p->eventSource)
            {
                case kEventSourceEventk:
                case kEventSourceEventu:
                    // error occurred within event processing
                    // either in kernel or in user part
                    len += snprintf(message_p + len,
                                    messageSize_p - len,
                                    "OrgSource = %s 0x%02X",
                                    debugstr_getEventSourceStr(pError_p->errorArg.eventSource),
                                    pError_p->errorArg.eventSource);
                    break;

                case kEventSourceDllk:
                    // error occurred within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    len += snprintf(message_p + len,
                                    messageSize_p - len,
                                    "Val = 0x%X",
                                    pError_p->errorArg.uintArg);
                    break;
            }
            break;
    }


}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief  Create timestamp string

The function creates the timestamp string.

\param[out]     string_p            String buffer to print to.
\param[in]      strlen_p            Length of string buffer.

\return The function returns the number of characters printed to the log string.
*/
//------------------------------------------------------------------------------
static size_t createTimeString(char* string_p,
                               size_t strlen_p)
{
    time_t      timeStamp;
    struct tm*  p_timeVal;
    size_t      len;

    time(&timeStamp);
    p_timeVal = localtime(&timeStamp);
    len = strftime(string_p, strlen_p, "%Y/%m/%d-%H:%M:%S ", p_timeVal);

    return len;
}

//------------------------------------------------------------------------------
/**
\brief  Create log level string

The function creates the log level string.

\param[out]     string_p            String buffer to print to.
\param[in]      strlen_p            Length of string buffer.
\param[in]      logLevel_p          The log level to print.

\return The function returns the number of characters printed to the log string.
*/
//------------------------------------------------------------------------------
static size_t createLogLevelString(char* string_p,
                                   size_t strlen_p,
                                   tEventlogLevel logLevel_p)
{
    size_t  len;

    len = snprintf(string_p,
                   strlen_p,
                   "%s ",
                   sEventlogLevel_l[logLevel_p]);

    return len;
}

//------------------------------------------------------------------------------
/**
\brief  Create log category string

The function creates the log category string.

\param[out]     string_p            String buffer to print to.
\param[in]      strlen_p            Length of string buffer.
\param[in]      logCategory_p       The log category to print.

\return The function returns the number of characters printed to the log string.
*/
//------------------------------------------------------------------------------
static size_t createLogCategoryString(char* string_p,
                                      size_t strlen_p,
                                      tEventlogCategory logCategory_p)
{
    size_t  len;

    len = snprintf(string_p,
                   strlen_p,
                   "%s ",
                   sEventlogCategory_l[logCategory_p]);

    return len;
}

/// \}
