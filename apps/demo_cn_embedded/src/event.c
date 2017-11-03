/**
********************************************************************************
\file   event.c

\brief  CN application event handler

This file contains a demo CN application event handler.

\ingroup module_demo
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include "event.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <gpio/gpio.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventCb pfnEventCb_l = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p);
static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                           void* pUserArg_p);
static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                        void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Initialize applications event module

The function initializes the applications event module

\param[in]      pfnEventCb_p        User event callback

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
void initEvents(tEventCb pfnEventCb_p)
{
    pfnEventCb_l = pfnEventCb_p;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK events

The function implements the application's stack event handler.

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Pointer to union which describes the event in detail
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
tOplkError processEvents(tOplkApiEventType eventType_p,
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

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(&pEventArg_p->pdoChange, pUserArg_p);
            break;

        default:
            break;
    }

    // call user event call back
    if ((ret == kErrorOk) &&
        (pfnEventCb_l != NULL))
        ret = pfnEventCb_l(eventType_p, pEventArg_p, pUserArg_p);

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

\param[in]      pNmtStateChange_p   Pointer to the state change event structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                          void* pUserArg_p)
{
    UNUSED_PARAMETER(pNmtStateChange_p);    // Avoid warning if debug is disabled
    UNUSED_PARAMETER(pUserArg_p);

    PRINTF("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
           pNmtStateChange_p->newNmtState,
           pNmtStateChange_p->nmtEvent,
           debugstr_getNmtEventStr(pNmtStateChange_p->nmtEvent));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process error and warning events

The function processes error and warning events.

\param[in]      pInternalError_p    Pointer to the internal error structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p,
                                           void* pUserArg_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    UNUSED_PARAMETER(pUserArg_p);

    PRINTF("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
           debugstr_getEventSourceStr(pInternalError_p->eventSource),
           pInternalError_p->eventSource,
           debugstr_getRetValStr(pInternalError_p->oplkError),
           pInternalError_p->oplkError);

    PRINTF("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
           debugstr_getEventSourceStr(pInternalError_p->eventSource),
           pInternalError_p->eventSource,
           debugstr_getRetValStr(pInternalError_p->oplkError),
           pInternalError_p->oplkError);

    // check additional argument
    switch (pInternalError_p->eventSource)
    {
        case kEventSourceEventk:
        case kEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            PRINTF(" OrgSource = %s %02X\n",
                   debugstr_getEventSourceStr(pInternalError_p->errorArg.eventSource),
                   pInternalError_p->errorArg.eventSource);

            PRINTF(" OrgSource = %s %02X\n",
                   debugstr_getEventSourceStr(pInternalError_p->errorArg.eventSource),
                   pInternalError_p->errorArg.eventSource);
            break;

        case kEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the UINT argument contains the DLL state and the NMT event
            PRINTF(" val = %X\n", pInternalError_p->errorArg.uintArg);
            PRINTF(" val = %X\n", pInternalError_p->errorArg.uintArg);
            break;

        default:
            PRINTF("\n");
            break;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process PDO change events

The function processes PDO change events.

\param[in]      pPdoChange_p        Pointer to the PDO change event structure
\param[in]      pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                        void* pUserArg_p)
{
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    UINT        varLen;

    UNUSED_PARAMETER(pUserArg_p);

    PRINTF("PDO change event: (%sPDO = 0x%X to node 0x%X with %d objects %s)\n",
           (pPdoChange_p->fTx ? "T" : "R"),
           pPdoChange_p->mappParamIndex,
           pPdoChange_p->nodeId,
           pPdoChange_p->mappObjectCount,
           (pPdoChange_p->fActivated ? "activated" : "deleted"));

    for (subIndex = 1; subIndex <= pPdoChange_p->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange_p->mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            PRINTF("  Reading 0x%X/%d failed with 0x%X\n\"%s\"\n",
                   pPdoChange_p->mappParamIndex,
                   subIndex,
                   ret,
                   debugstr_getRetValStr(ret));
            continue;
        }
        PRINTF("  %d. mapped object 0x%llX/", subIndex, mappObject & 0x00FFFFULL);
        PRINTF("%lld\n", (mappObject & 0xFF0000ULL) >> 16);
    }

    return kErrorOk;
}

/// \}
