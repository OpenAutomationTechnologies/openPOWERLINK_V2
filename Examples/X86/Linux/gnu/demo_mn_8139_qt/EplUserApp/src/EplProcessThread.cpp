/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.de

  Project:      EPL Messe Demo

  Description:  demoapplication for EPL

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

static DWORD        dw_le_CycleLen_g;

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
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                DWORD   dwBuffer;
#endif

                    pszNmtState = "ResetCommunication";
                    pEplProcessThread_g->sigEplStatus(1);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                    // configure OD for MN in state ResetComm after reseting the OD
                    // TODO: setup your own network configuration here
                    dwBuffer = (EPL_NODEASSIGN_NODE_IS_CN |
                                EPL_NODEASSIGN_NODE_EXISTS);    // 0x00000003L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x01, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x02, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x03, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x04, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x05, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x06, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x07, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x08, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x20, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xFE, &dwBuffer, 4);
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwBuffer, 4);

//                    dwBuffer |= EPL_NODEASSIGN_MANDATORY_CN;    // 0x0000000BL
//                    EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwBuffer, 4);
                    dwBuffer = (EPL_NODEASSIGN_MN_PRES |
                                EPL_NODEASSIGN_NODE_EXISTS);    // 0x00010001L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xF0, &dwBuffer, 4);
#endif

                    // continue
                    break;
                }

                case kEplNmtGsResetConfiguration:
                {
                unsigned int uiSize;

                    pszNmtState = "ResetConfiguration";
                    pEplProcessThread_g->sigEplStatus(1);

                    // fetch object 0x1006 NMT_CycleLen_U32 from local OD
                    // (in little endian byte order)
                    // for configuration of remote CN
                    uiSize = 4;
                    EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00, &dw_le_CycleLen_g,
                                              &uiSize, kEplSdoTypeAsnd, NULL);
                    if (EplRet != kEplSuccessful)
                    {   // local OD access failed
                        break;
                    }

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

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplApiEventNode:
        {
            // printf("AppCbEvent(Node): NodeId=%u Event=0x%02X\n",
            //       pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_NodeEvent);
            // check additional argument
            switch (pEventArg_p->m_Node.m_NodeEvent)
            {
                case kEplNmtNodeEventFound:
                {

                    pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 0);
                    pEplProcessThread_g->sigNodeAppeared(pEventArg_p->m_Node.m_uiNodeId);

                    break;
                }

                case kEplNmtNodeEventCheckConf:
                {
                tEplSdoComConHdl SdoComConHdl;

                    pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 1);

                    // update object 0x1006 on CN
                    EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId,
                                               0x1006, 0x00, &dw_le_CycleLen_g, 4,
                                               kEplSdoTypeAsnd, NULL);
                    if (EplRet == kEplApiTaskDeferred)
                    {   // SDO transfer started
                        EplRet = kEplReject;
                    }
                    else if (EplRet == kEplSuccessful)
                    {   // local OD access (should not occur)
                        PRINTF0("AppCbEvent(Node) write to local OD\n");
                    }
                    else
                    {   // error occured

                        EplRet = EplApiFreeSdoChannel(SdoComConHdl);
                        SdoComConHdl = 0;

                        EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId,
                                                   0x1006, 0x00, &dw_le_CycleLen_g, 4,
                                                   kEplSdoTypeAsnd, NULL);
                        if (EplRet == kEplApiTaskDeferred)
                        {   // SDO transfer started
                            EplRet = kEplReject;
                        }
                        else
                        {
                            PRINTF1("AppCbEvent(Node): EplApiWriteObject() returned 0x%03X\n", EplRet);
                        }
                    }

                    break;
                }

                case kEplNmtNodeEventNmtState:
                {
                    switch (pEventArg_p->m_Node.m_NmtState)
                    {
                        case kEplNmtGsOff:
                        {
                            pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, 0);
                            break;
                        }
                        case kEplNmtGsInitialising:
                        case kEplNmtGsResetApplication:
                        case kEplNmtGsResetCommunication:
                        case kEplNmtGsResetConfiguration:
                        case kEplNmtCsNotActive:
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
                    pEplProcessThread_g->sigNodeDisappeared(pEventArg_p->m_Node.m_uiNodeId);
                    PRINTF2("AppCbEvent(Node 0x%X): ErrorCode: 0x%04hX\n",
                            pEventArg_p->m_Node.m_uiNodeId,
                            pEventArg_p->m_Node.m_wErrorCode);
                    break;
                }

                default:
                {
                    pEplProcessThread_g->sigNodeStatus(pEventArg_p->m_Node.m_uiNodeId, -1);
                    pEplProcessThread_g->sigNodeAppeared(pEventArg_p->m_Node.m_uiNodeId);
                }
            }
            break;
        }

        case kEplApiEventSdo:
        {   // SDO transfer finished
            EplRet = EplApiFreeSdoChannel(pEventArg_p->m_Sdo.m_SdoComConHdl);
            if (EplRet != kEplSuccessful)
            {
                break;
            }
            if (pEventArg_p->m_Sdo.m_SdoComConState == kEplSdoComTransferFinished)
            {   // continue boot-up of CN with NMT command Reset Configuration
                EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId,
                                                    kEplNmtNodeCommandConfReset);
            }
            else
            {   // indicate configuration error CN
                EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId,
                                                    kEplNmtNodeCommandConfErr);
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

    // kill sync process
    //kill(PidSync, SIGINT);

    // wait for finish of sync process
    //waitpid(PidSync, NULL, 0);

    // ShutdownEpl:
    //EplRet = EplApiShutdown();

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


