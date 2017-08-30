/**
********************************************************************************
\file   eventlogstring.h

\brief  Definitions for eventlog string functions

This file contains the definitions for the string functions of the eventlog
module.
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

#ifndef _INC_eventlogstring_H_
#define _INC_eventlogstring_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdarg.h>

#include <oplk/oplk.h>
#include <oplk/nmt.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// unfortunately older windows compilers don't support snprintf and vsnprintf
#if (TARGET_SYSTEM == _WIN32_)
#if _MSC_VER < 1900
#define snprintf _snprintf
#define vsnprintf _vsnprintf
#endif
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* Enumeration for eventlog levels
*
* This enumeration specifies the valid eventlog levels. The eventlog levels
* are used to filter the eventlog output.
*/
typedef enum
{
    kEventlogLevelFatal = 0,    ///< Fatal errors
    kEventlogLevelError,        ///< Errors
    kEventlogLevelWarning,      ///< Warnings
    kEventlogLevelInfo,         ///< Information
    kEventlogLevelDebug,        ///< Debug information
    kEventlogLevelEvent,        ///< Events
} eEventlogLevel;
typedef UINT32 tEventlogLevel;

/**
* Enumeration for eventlog categories
*
* The eventlog is separated in different output categories. This enumeration
* lists the valid categories. The categories are used to filter the eventlog
* output.
*/
typedef enum
{
    kEventlogCategoryGeneric = 0,               ///< Generic information
    kEventlogCategoryControl,                   ///< Control information
    kEventlogCategoryApplication,               ///< Information of the Application
    kEventlogCategoryPdoMap,                    ///< PDO mapping information
    kEventlogCategoryObjectDictionary,          ///< Object dictionary information

    kEventlogCategoryNodeEvent = 15,            ///< Node events
    kEventlogCategoryCfmResultEvent,            ///< CfmResult events
    kEventlogCategoryCfmProgressEvent,          ///< CfmProgress events
    kEventlogCategoryStateChangeEvent,          ///< StateChange events
    kEventlogCategoryPdoEvent,                  ///< Pdo events
    kEventlogCategoryHistoryEvent,              ///< History events
    kEventlogCategoryErrorEvent,                ///< Error events
    kEventlogCategorySdoEvent,                  ///< Sdo events
    kEventlogCategoryUserEvent,                 ///< User event
} eEventlogCategory;
typedef UINT32 tEventlogCategory;

/**
* Enumeration for CFM events
*
* This enumeration lists the valid CFM events.
*/
typedef enum
{
    kLogEventCfmProgress = 0,                   ///< CFM progress event
    kLogEventCfmResult                          ///< CFM result event
} eEventlogEventCfmType;
typedef UINT32 tEventlogEventCfmType;

/**
* Enumeration for eventlog output format
*
* This enumeration lists the valid eventlog output formats.
*/
typedef enum
{
    kEventlogFormatParsable,        ///< Parsable format. Intended for further processing.
    kEventlogFormatReadable         ///< Readable format. Optimized for easy reading by humans.
} eEventlogFormat;
typedef UINT32 tEventlogFormat;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

void eventlog_createMessageString(char* message_p, size_t messageSize_p, tEventlogLevel level_p, tEventlogCategory category_p, const char* fmt_p, va_list arg_p);
void eventlog_createNodeEventString(tOplkApiEventNode* pNodeEvent_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createStateEventString(tEventNmtStateChange* pStateChangeEvent_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createCfmResultEventString(UINT8 nodeId_p, tNmtNodeCommand nodeCommand_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createCfmProgressEventString(tCfmEventCnProgress* pProgress_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createPdoEventString(tOplkApiEventPdoChange* pPdoChange_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createHistoryEventString(tErrHistoryEntry* pHistory_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createErrorEventString(tEventError* pError_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);
void eventlog_createPdoMapString(UINT16 mapObject_p, UINT8 subIndex_p, UINT64 mapping_p, tEventlogFormat format_p, char* message_p, size_t messageSize_p);


#ifdef __cplusplus
}
#endif

#endif /* _INC_eventlogstring_H_ */
