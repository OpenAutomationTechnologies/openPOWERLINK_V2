/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Qt based demoapplication: connection to stack (event callback function)

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2008/04/11 m.u.:   start of implementation

****************************************************************************/

#include <QWidget>
#include <QThread>
#include <QString>
#include <QMutex>
#include <QWaitCondition>

#include <EplApi.h>


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define SDO_NODEID  0xF0

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------

EplProcessThread    *pEplProcessThread_g;

//---------------------------------------------------------------------------
// Event callback function
//---------------------------------------------------------------------------

tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           /*pUserArg_p*/)
{
tEplKernel  EplRet = kEplSuccessful;
const char* pszNmtState = NULL;

    switch (EventType_p)
    {
        case kEplApiEventNmtStateChange:
        {
            pEplProcessThread_g->sigNmtState(
                    pEventArg_p->m_NmtStateChange.m_NewNmtState);

            switch (pEventArg_p->m_NmtStateChange.m_NewNmtState)
            {
                case kEplNmtGsOff:
                {
                    pszNmtState = "Off";
                    pEplProcessThread_g->sigEplStatus(0);

                    // NMT state machine was shut down,
                    // because of user signal (CTRL-C) or critical EPL stack error
                    // -> also shut down EplApiProcess()
                    EplRet = kEplShutdown;

                    printf("AppCbEvent(kEplNmtGsOff) originating event = 0x%X\n",
                           pEventArg_p->m_NmtStateChange.m_NmtEvent);

                    pEplProcessThread_g->reachedNmtStateOff();
                    break;
                }

                case kEplNmtGsResetCommunication:
                {

                    pszNmtState = "ResetCommunication";
                    pEplProcessThread_g->sigEplStatus(1);

                    // continue
                    break;
                }

                case kEplNmtGsResetConfiguration:
                {

                    pszNmtState = "ResetConfiguration";
                    pEplProcessThread_g->sigEplStatus(1);

                    break;
                }

                case kEplNmtCsPreOperational1:
                case kEplNmtMsPreOperational1:
                {
                    pszNmtState = "PreOp1";
                    pEplProcessThread_g->sigEplStatus(1);

                    break;
                }

                case kEplNmtCsPreOperational2:
                case kEplNmtMsPreOperational2:
                {
                    pszNmtState = "PreOp2";
                    pEplProcessThread_g->sigEplStatus(1);

                    break;
                }

                case kEplNmtCsReadyToOperate:
                case kEplNmtMsReadyToOperate:
                {
                    pszNmtState = "ReadyToOp";
                    pEplProcessThread_g->sigEplStatus(1);

                    break;
                }

                case kEplNmtGsInitialising:
                {
                    pszNmtState = "Init";
                    pEplProcessThread_g->sigEplStatus(1);
                    break;
                }

                case kEplNmtGsResetApplication:
                {
                    pszNmtState = "ResetApp";
                    pEplProcessThread_g->sigEplStatus(1);
                    break;
                }

                case kEplNmtCsNotActive:
                case kEplNmtMsNotActive:
                {
                    pszNmtState = "NotActive";
                    pEplProcessThread_g->sigEplStatus(1);
                    break;
                }

                case kEplNmtCsOperational:
                case kEplNmtMsOperational:
                {
                    pszNmtState = "Operational";
                    pEplProcessThread_g->sigEplStatus(2);
                    break;
                }

                case kEplNmtCsBasicEthernet:
                case kEplNmtMsBasicEthernet:
                {
                    pszNmtState = "BasicEthernet";
                    pEplProcessThread_g->sigEplStatus(1);
                    break;
                }

                default:
                {
                    pszNmtState = "Others";
                    pEplProcessThread_g->sigEplStatus(-1);
                }
            }
            printf("AppCbEvent(NMT) event 0x%X -> %s (0x%X)\n",
                   pEventArg_p->m_NmtStateChange.m_NmtEvent,
                   pszNmtState,
                   pEventArg_p->m_NmtStateChange.m_NewNmtState);

            break;
        }

        case kEplApiEventCriticalError:
        case kEplApiEventWarning:
        {
            printf("AppCbEvent(Err/Warn): Source=%02X EplError=0x%03X",
                   pEventArg_p->m_InternalError.m_EventSource,
                   pEventArg_p->m_InternalError.m_EplError);
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {
                    printf(" OrgSource=%02X\n",
                           pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }
#if 0
                case ...
                {
                    printf(" Index=0x%04X, Subindex=0x%X",
                           pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiIndex,
                           pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiSubIndex);
                    break;
                }
#endif
                default:
                {
                    printf("\n");
                    break;
                }
            }
            break;
        }

        case kEplApiEventHistoryEntry:
        {   // new history entry

            PRINTF("%s(HistoryEntry): Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X)\n",
                    __func__,
                    pEventArg_p->m_ErrHistoryEntry.m_wEntryType,
                    pEventArg_p->m_ErrHistoryEntry.m_wErrorCode,
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[0],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[1],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[2],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[3],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[4],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[5],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[6]);
            break;
        }

        case kEplApiEventNode:
        {
            // printf("AppCbEvent(Node): NodeId=%u Event=0x%02X\n",
            //       pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_NodeEvent);
            // check additional argument
            switch (pEventArg_p->m_Node.m_NodeEvent)
            {
                case kEplNmtNodeEventCheckConf:
                {
                    PRINTF2("%s(Node=0x%X, CheckConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventUpdateConf:
                {
                    PRINTF2("%s(Node=0x%X, UpdateConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventFound:
                {

                    pEplProcessThread_g->sigNodeAppeared(pEventArg_p->m_Node.m_uiNodeId);

                    break;
                }

                case kEplNmtNodeEventNmtState:
                {
                    switch (pEventArg_p->m_Node.m_NmtState)
                    {
                        case kEplNmtGsOff:
                        case kEplNmtGsInitialising:
                        case kEplNmtGsResetApplication:
                        case kEplNmtGsResetCommunication:
                        case kEplNmtGsResetConfiguration:
                        case kEplNmtCsNotActive:
                        {
//                            pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 0);
                            pEplProcessThread_g->sigNodeDisappeared(pEventArg_p->m_Node.m_uiNodeId);
                            break;
                        }
                        case kEplNmtCsPreOperational1:
                        case kEplNmtCsPreOperational2:
                        case kEplNmtCsReadyToOperate:
                        {
                            pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 1);
                            break;
                        }
                        case kEplNmtCsOperational:
                        {
                            pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 2);
                            break;
                        }
                        case kEplNmtCsBasicEthernet:
                        case kEplNmtCsStopped:
                        default:
                        {
                            pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, -1);
                            break;
                        }
                    }
                    break;
                }

                case kEplNmtNodeEventError:
                {
                    pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, -1);
                    printf("AppCbEvent(Node 0x%X): ErrorCode: 0x%04hX\n",
                            pEventArg_p->m_Node.m_uiNodeId,
                            pEventArg_p->m_Node.m_wErrorCode);
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        case kEplApiEventCfmProgress:
        {
            PRINTF4("%s(Node=0x%X, CFM-Progress: Object 0x%X/%u, ", __func__, pEventArg_p->m_CfmProgress.m_uiNodeId, pEventArg_p->m_CfmProgress.m_uiObjectIndex, pEventArg_p->m_CfmProgress.m_uiObjectSubIndex);
            PRINTF2("%lu/%lu Bytes", (ULONG) pEventArg_p->m_CfmProgress.m_dwBytesDownloaded, (ULONG) pEventArg_p->m_CfmProgress.m_dwTotalNumberOfBytes);
            if ((pEventArg_p->m_CfmProgress.m_dwSdoAbortCode != 0)
                || (pEventArg_p->m_CfmProgress.m_EplError != kEplSuccessful))
            {
                PRINTF2(" -> SDO Abort=0x%lX, Error=0x%X)\n", pEventArg_p->m_CfmProgress.m_dwSdoAbortCode, pEventArg_p->m_CfmProgress.m_EplError);
            }
            else
            {
                PRINTF0(")\n");
            }
            break;
        }

        case kEplApiEventCfmResult:
        {
            switch (pEventArg_p->m_CfmResult.m_NodeCommand)
            {
                case kEplNmtNodeCommandConfOk:
                {
                    PRINTF2("%s(Node=0x%X, ConfOk)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfErr:
                {
                    PRINTF2("%s(Node=0x%X, ConfErr)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfReset:
                {
                    PRINTF2("%s(Node=0x%X, ConfReset)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfRestored:
                {
                    PRINTF2("%s(Node=0x%X, ConfRestored)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                default:
                {
                    PRINTF3("%s(Node=0x%X, CfmResult=0x%X)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId, pEventArg_p->m_CfmResult.m_NodeCommand);
                    break;
                }
            }
            break;
        }
#endif

        default:
            break;
    }

    return EplRet;
}


//=========================================================================//
//                                                                         //
//          E p l P r o c e s s T h r e a d                                //
//                                                                         //
//=========================================================================//

EplProcessThread::EplProcessThread()
{
    pEplProcessThread_g = this;
    iEplStatus = -1;
}


void EplProcessThread::run()
{
tEplKernel          EplRet;

    // start process function
    EplRet = EplApiProcess();

}


void EplProcessThread::sigEplStatus(int iStatus_p)
{
    if(iStatus_p != iEplStatus)
    {
        emit eplStatusChanged(iStatus_p);
        iEplStatus = iStatus_p;
    }
}


void EplProcessThread::sigNmtState(tEplNmtState State_p)
{
    QString strState;

    switch(State_p)
    {
        case kEplNmtGsOff:
            strState = "Off"; break;
        case kEplNmtGsInitialising:
            strState = "Initializing"; break;
        case kEplNmtGsResetApplication:
            strState = "Reset Application"; break;
        case kEplNmtGsResetCommunication:
            strState = "Reset Communication"; break;
        case kEplNmtGsResetConfiguration:
            strState = "Reset Configuration"; break;
        case kEplNmtCsNotActive:
            strState = "CN Not Active"; break;
        case kEplNmtCsPreOperational1:
            strState = "CN Pre-Operational 1"; break;
        case kEplNmtCsPreOperational2:
            strState = "CN Pre-Operational 2"; break;
        case kEplNmtCsReadyToOperate:
            strState = "CN ReadyToOperate"; break;
        case kEplNmtCsOperational:
            strState = "CN Operational"; break;
        case kEplNmtCsBasicEthernet:
            strState = "CN Basic Ethernet"; break;
        case kEplNmtMsNotActive:
            strState = "MN Not Active"; break;
        case kEplNmtMsPreOperational1:
            strState = "MN Pre-Operational 1"; break;
        case kEplNmtMsPreOperational2:
            strState = "MN Pre-Operational 2"; break;
        case kEplNmtMsReadyToOperate:
            strState = "MN ReadyToOperate"; break;
        case kEplNmtMsOperational:
            strState = "MN Operational"; break;
        case kEplNmtMsBasicEthernet:
            strState = "MN Basic Ethernet"; break;
        default:
            strState = "??? (0x";
            strState += QString::number(State_p, 16);
            strState += ")";
    }

    emit nmtStateChanged(strState);
}

void EplProcessThread::waitForNmtStateOff()
{
    Mutex.lock();
    if (iEplStatus > 0)
    {
        NmtStateOff.wait(&Mutex);
    }
    Mutex.unlock();
}

void EplProcessThread::reachedNmtStateOff()
{
    emit allNodesRemoved();
    Mutex.lock();
    NmtStateOff.wakeAll();
    Mutex.unlock();
}

tEplApiCbEvent EplProcessThread::getEventCbFunc()
{
    return AppCbEvent;
}


