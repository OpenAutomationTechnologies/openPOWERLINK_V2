/**
********************************************************************************
\file   EventHandler.cpp

\brief  Implementation of the openPOWERLINK event handler class

This file contains the implementation of the openPOWERLINK event handler class.
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EventHandler.h>
#include <MainWindow.h>
#include <EventLog.h>

#include <QDateTime>

#include <oplk/debugstr.h>

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
Q_DECLARE_METATYPE(tSdoComFinished)
Q_DECLARE_METATYPE(tNmtState)

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Event callback function

The function processes events of the openPOWERLINK stack.
It is provided to the openPOWERLINK stack as its event callback,
while an object pointer ("this") is provided as user specific argument.

\param[in]      eventType_p         Type of event
\param[in]      pEventArg_p         Pointer to union describing the event in detail
\param[in]      pUserArg_p          User specific argument (contains a pointer to
                                    an instance of this class)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::appCbEvent(tOplkApiEventType eventType_p,
                                    const tOplkApiEventArg* pEventArg_p,
                                    void* pUserArg_p)
{
    // We know that the userArg contains our "this" pointer
    EventHandler* const instance = static_cast<EventHandler* const>(pUserArg_p);
    tOplkError          ret;

    // check if NMT_GS_OFF is reached
    switch (eventType_p)
    {
        case kOplkApiEventUserDef:
            ret = instance->userDefinedEvent(pEventArg_p->pUserArg);
            break;

        case kOplkApiEventNmtStateChange:
            ret = instance->nmtStateChangeEvent(pEventArg_p->nmtStateChange);
            break;

        case kOplkApiEventCriticalError:
            ret = instance->criticalErrorEvent(pEventArg_p->internalError);
            break;

        case kOplkApiEventWarning:
            ret = instance->warningEvent(pEventArg_p->internalError);
            break;

        case kOplkApiEventHistoryEntry:
            ret = instance->historyEntryEvent(pEventArg_p->errorHistoryEntry);
            break;

        case kOplkApiEventNode:
            ret = instance->nodeEvent(pEventArg_p->nodeEvent);
            break;

        case kOplkApiEventBoot:
            ret = instance->bootEvent(pEventArg_p->bootEvent);
            break;

        case kOplkApiEventSdo:
            ret = instance->sdoCommandFinishedEvent(pEventArg_p->sdoInfo);
            break;

        case kOplkApiEventObdAccess:
            ret = instance->obdAccessEvent(pEventArg_p->obdCbParam);
            break;

        case kOplkApiEventCfmProgress:
            ret = instance->cfmProgressEvent(pEventArg_p->cfmProgress);
            break;

        case kOplkApiEventCfmResult:
            ret = instance->cfmResultEvent(pEventArg_p->cfmResult);
            break;

        case kOplkApiEventReceivedAsnd:
            ret = instance->asndReceivedEvent(pEventArg_p->receivedAsnd);
            break;

        case kOplkApiEventPdoChange:
            ret = instance->pdoChangeEvent(pEventArg_p->pdoChange);
            break;

        case kOplkApiEventReceivedPres:
            ret = instance->presReceivedEvent(pEventArg_p->receivedPres);
            break;

        case kOplkApiEventReceivedNonPlk:
            ret = instance->nonPlkReceivedEvent(pEventArg_p->receivedEth);
            break;

        case kOplkApiEventDefaultGwChange:
            ret = instance->defaultGwChangedEvent(pEventArg_p->defaultGwChange);
            break;

        case kOplkApiEventReceivedSdoCom:
            ret = instance->sdoComReceived(pEventArg_p->receivedSdoCom);
            break;

        case kOplkApiEventReceivedSdoSeq:
            ret = instance->sdoSeqReceived(pEventArg_p->receivedSdoSeq);
            break;

        default:
            // Ignore the unknown event
            qDebug("Unknown event: 0x%x", eventType_p);
            ret = kErrorOk;
            break;
    }

    return ret;
}

//============================================================================//
//            P U B L I C    M E M B E R    F U N C T I O N S                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs an EventHandler object

\param[in,out]  pEventLog_p         Pointer to event logger object
*/
//------------------------------------------------------------------------------
EventHandler::EventHandler(EventLog* pEventLog_p) :
    fMnActive(false),
    pEventLog(pEventLog_p)
{
    qRegisterMetaType<tSdoComFinished>("tSdoComFinished");
    qRegisterMetaType<tNmtState>("tNmtState");
}

//------------------------------------------------------------------------------
/**
\brief  Await reaching the NMT_GS_OFF state

Blocks the current thread until NMT_GS_OFF state is reached
*/
//------------------------------------------------------------------------------
void EventHandler::awaitNmtGsOff()
{
    this->mutex.lock();
    this->nmtGsOff.wait(&this->mutex);
    this->mutex.unlock();
}

//============================================================================//
//          P R O T E C T E D    M E M B E R    F U N C T I O N S             //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Process a user event.

\param[in]      pUserArg_p          User-specific arguments

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::userDefinedEvent(void* pUserArg_p)
{
    emit userDefEvent(pUserArg_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process the NMT state change events of the local node.

\param[in]      nmtStateChange_p    Details of the NMT state changes

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::nmtStateChangeEvent(const tEventNmtStateChange& nmtStateChange_p)
{
    tOplkError  ret;

    // Print the event
    this->pEventLog->printEvent(nmtStateChange_p);

    // Signal new NMT state
    emit nmtStateChanged(nmtStateChange_p.newNmtState);

    // Handle the new NMT state
    switch (nmtStateChange_p.newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process()
            this->mutex.lock();
            this->nmtGsOff.wakeAll();
            this->mutex.unlock();

            this->sigMnActive(false);
            ret = kErrorShutdown;
            break;

        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
            this->sigMnActive(true);
            ret = kErrorOk;
            break;

        default:
            this->sigMnActive(false);
            ret = kErrorOk;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process critical error events of the stack.

\param[in]      internalError_p     Details of the critical error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::criticalErrorEvent(const tEventError& internalError_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine
    this->pEventLog->printEvent(internalError_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process warning events of the stack.

\param[in]      internalError_p     Details of the warning event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::warningEvent(const tEventError& internalError_p)
{
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine
    this->pEventLog->printEvent(internalError_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process history events generated by the stack.

\param[in]      historyEntry_p      Details of the history event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::historyEntryEvent(const tErrHistoryEntry& historyEntry_p)
{
    this->pEventLog->printEvent(historyEntry_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process events generated by the stack with respect to the remote nodes.

\param[in]      nodeEvent_p         Details of the node event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::nodeEvent(const tOplkApiEventNode& nodeEvent_p)
{
    tOplkError  ret = kErrorOk;

    // Print the event
    this->pEventLog->printEvent(nodeEvent_p);

    // Check the occured event
    switch (nodeEvent_p.nodeEvent)
    {
        case kNmtNodeEventCheckConf:
        case kNmtNodeEventUpdateConf:
        case kNmtNodeEventFound:
            break;

        case kNmtNodeEventNmtState:
            emit nodeStatusChanged(nodeEvent_p.nodeId, nodeEvent_p.nmtState);
            break;

        case kNmtNodeEventError:
            emit nodeStatusChanged(nodeEvent_p.nodeId, kNmtStateInvalid);
            break;

        case kNmtNodeEventAmniReceived:
        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process boot events generated by the stack.

\param[in]      bootEvent_p         Details of the boot event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::bootEvent(const tOplkApiEventBoot& bootEvent_p)
{
    UNUSED_PARAMETER(bootEvent_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process the events generated during the SDO transfer

\param[in]      sdoEvent_p          Details of the SDO events occurred.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::sdoCommandFinishedEvent(const tSdoComFinished& sdoEvent_p)
{
    emit sdoFinished(sdoEvent_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process an object dictionary access event

\param[in]      obdEvent_p          Details of the OD access.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::obdAccessEvent(const tObdCbParam& obdEvent_p)
{
    UNUSED_PARAMETER(obdEvent_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM progress events

\param[in]      cfmProgress_p       Details of the CFM progress events.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::cfmProgressEvent(const tCfmEventCnProgress& cfmProgress_p)
{
    this->pEventLog->printEvent(cfmProgress_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process CFM result events

\param[in]      cfmResult_p         Result of the CFM event occurred.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::cfmResultEvent(const tOplkApiEventCfmResult& cfmResult_p)
{
    this->pEventLog->printEvent(cfmResult_p.nodeId,
                                cfmResult_p.nodeCommand);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process ASnd received event

\param[in]      receivedAsnd_p      Information about the received ASnd.

\note    This event will only occur, if the ASnd forwarding has
         been enabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::asndReceivedEvent(const tOplkApiEventRcvAsnd& receivedAsnd_p)
{
    UNUSED_PARAMETER(receivedAsnd_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process the PDO change events.

\param[in]      pdoChange_p         Details of the PDO change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::pdoChangeEvent(const tOplkApiEventPdoChange& pdoChange_p)
{
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    UINT        varLen;

    this->pEventLog->printEvent(pdoChange_p);

    for (subIndex = 1; subIndex <= pdoChange_p.mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pdoChange_p.mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            this->pEventLog->printMessage(kEventlogLevelError,
                                          kEventlogCategoryObjectDictionary,
                                          "Reading 0x%X/%d failed with %s(0x%X)",
                                          pdoChange_p.mappParamIndex,
                                          subIndex,
                                          debugstr_getRetValStr(ret),
                                          ret);
            continue;
        }

        this->pEventLog->printPdoMap(pdoChange_p.mappParamIndex,
                                     subIndex,
                                     mappObject);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process PRes received event

\param[in]      receivedPres_p      Information about the received PRes.

\note    This event will only occur if the PRes forwarding has been enabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::presReceivedEvent(const tOplkApiEventReceivedPres& receivedPres_p)
{
    UNUSED_PARAMETER(receivedPres_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process a non-POWERLINK frame received event

\param[in]      receivedEth_p       Information about the received non-POWERLINK frame.

\note    This event will only occur if Virtual Ethernet is enabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::nonPlkReceivedEvent(const tOplkApiEventReceivedNonPlk& receivedEth_p)
{
    UNUSED_PARAMETER(receivedEth_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process a default gateway changed event

\param[in]      defaultGw_p         Information about the new default gateway

\note    This event will only occur if Virtual Ethernet is enabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::defaultGwChangedEvent(const tOplkApiEventDefaultGwChange& defaultGw_p)
{
    UNUSED_PARAMETER(defaultGw_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO command layer frame received event

\param[in]      receivedSdoCom_p    Information about the received SDO
                                    command layer frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::sdoComReceived(const tOplkApiEventReceivedSdoCom& receivedSdoCom_p)
{
    UNUSED_PARAMETER(receivedSdoCom_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO sequence layer frame received event

\param[in]      receivedSdoSeq_p    Information about the received SDO
                                    sequence layer frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError EventHandler::sdoSeqReceived(const tOplkApiEventReceivedSdoSeq& receivedSdoSeq_p)
{
    UNUSED_PARAMETER(receivedSdoSeq_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E    M E M B E R    F U N C T I O N S               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Signal active MN state

The function signals if it is in an active MN state.

\param[in]      fMnActive_p         If true it is in an active MN state otherwise not.
*/
//------------------------------------------------------------------------------
void EventHandler::sigMnActive(bool fMnActive_p)
{
    if (fMnActive_p != this->fMnActive)
    {
        emit isMnActive(fMnActive_p);
        this->fMnActive = fMnActive_p;
    }
}
