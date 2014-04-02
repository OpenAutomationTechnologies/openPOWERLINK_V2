/**
********************************************************************************
\file   event.c

\brief  CN application event handler

This file contains a demo CN application event handler.

\ingroup module_demo
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <console/console.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static BOOL*    pfGsOff_l;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p);

static tOplkError processPdoChangeEvent(tOplkApiEventType EventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param  pfGsOff_p               Pointer to GsOff flag (determines that stack is down)

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void initEvents(BOOL* pfGsOff_p)
{
    pfGsOff_l = pfGsOff_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the application's stack event handler.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType EventType_p,
                         tOplkApiEventArg* pEventArg_p,
                         void* pUserArg_p)
{
    tOplkError          ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    switch (EventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        default:
            break;
    }
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process state change events

The function processes state change events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                          tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p)
{
    tOplkError                  ret = kErrorOk;
    tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->nmtStateChange;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        return kErrorGeneralError;
    }

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            ret = kErrorShutdown;

            console_printlog("StateChangeEvent:kNmtGsOff originating event = 0x%X (%s)\n",
                             pNmtStateChange->nmtEvent,
                             debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));

            // signal that stack is off
            *pfGsOff_l = FALSE;
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetConfiguration:
        case kNmtGsResetCommunication:
        case kNmtCsNotActive:               // Implement
        case kNmtCsPreOperational1:         // handling of
        case kNmtCsStopped:                 // different
        case kNmtCsPreOperational2:         // states here
        case kNmtCsReadyToOperate:
        case kNmtCsOperational:
        case kNmtCsBasicEthernet:           // no break;

        default:
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                             pNmtStateChange->newNmtState,
                             pNmtStateChange->nmtEvent,
                             debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                           tOplkApiEventArg* pEventArg_p,
                                           void* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    tEventError*            pInternalError = &pEventArg_p->internalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
                     debugstr_getEventSourceStr(pInternalError->eventSource),
                     pInternalError->eventSource,
                     debugstr_getRetValStr(pInternalError->oplkError),
                     pInternalError->oplkError);

    FTRACE_MARKER("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
                  debugstr_getEventSourceStr(pInternalError->eventSource),
                  pInternalError->eventSource,
                  debugstr_getRetValStr(pInternalError->oplkError),
                  pInternalError->oplkError);

    // check additional argument
    switch (pInternalError->eventSource)
    {
        case kEventSourceEventk:
        case kEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            console_printlog(" OrgSource = %s %02X\n",
                             debugstr_getEventSourceStr(pInternalError->errorArg.eventSource),
                             pInternalError->errorArg.eventSource);

            FTRACE_MARKER(" OrgSource = %s %02X\n",
                          debugstr_getEventSourceStr(pInternalError->errorArg.eventSource),
                          pInternalError->errorArg.eventSource);
            break;

        case kEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            console_printlog(" val = %X\n", pInternalError->errorArg.uintArg);
            FTRACE_MARKER(" val = %X\n", pInternalError->errorArg.uintArg);
            break;

        default:
            console_printlog("\n");
            break;
    }
    return kErrorOk;
}


//------------------------------------------------------------------------------
/**
\brief  Process PDO change events

The function processes PDO change events.

\param  EventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processPdoChangeEvent(tOplkApiEventType EventType_p,
                                        tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p)
{
    tOplkApiEventPdoChange*     pPdoChange = &pEventArg_p->pdoChange;
    UINT                        subIndex;
    UINT64                      mappObject;
    tOplkError                  ret;
    UINT                        varLen;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("PDO change event: (%sPDO = 0x%X to node 0x%X with %d objects %s)\n",
                     (pPdoChange->fTx ? "T" : "R"), pPdoChange->mappParamIndex,
                     pPdoChange->nodeId, pPdoChange->mappObjectCount,
                     (pPdoChange->fActivated ? "activated" : "deleted"));

    for (subIndex = 1; subIndex <= pPdoChange->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange->mappParamIndex, subIndex, &mappObject, &varLen);
        if (ret != kErrorOk)
        {
            console_printlog("  Reading 0x%X/%d failed with 0x%X\n",
                             pPdoChange->mappParamIndex, subIndex, ret);
            continue;
        }
        console_printlog("  %d. mapped object 0x%X/%d\n", subIndex, mappObject & 0x00FFFFULL,
                         (mappObject & 0xFF0000ULL) >> 16);
    }
    return kErrorOk;
}

///\}

