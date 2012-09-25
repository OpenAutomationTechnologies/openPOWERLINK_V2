/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for NMT user part module

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
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/06/09 k.t.:   start of the implementation

****************************************************************************/


#include "EplInc.h"
#include "user/EplNmtu.h"
#include "user/EplObdu.h"
#include "user/EplTimeru.h"
#include "user/EplDlluCal.h"

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tEplNmtState                    m_LocalNmtState;
    tEplNmtuStateChangeCallback     m_pfnNmtChangeCb;
    tEplTimerHdl                    m_TimerHdl;

} tEplNmtuInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEplNmtuInstance EplNmtuInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

#if EPL_NMT_MAX_NODE_ID > 0

static tEplKernel EplNmtuConfigureDll(void);

#endif


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplNmtuInit
//
// Description: init first instance of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuInit()
{
tEplKernel Ret;

    Ret = EplNmtuAddInstance();

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuAddInstance
//
// Description: init other instances of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuAddInstance()
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    EplNmtuInstance_g.m_pfnNmtChangeCb = NULL;

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuDelInstance
//
// Description: delete instance
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuDelInstance()
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    EplNmtuInstance_g.m_pfnNmtChangeCb = NULL;

    // delete timer
    Ret = EplTimeruDeleteTimer(&EplNmtuInstance_g.m_TimerHdl);

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuNmtEvent
//
// Description: sends the NMT-Event to the NMT-State-Maschine
//
//
//
// Parameters:  NmtEvent_p  = NMT-Event to send
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuNmtEvent(tEplNmtEvent NmtEvent_p)
{
tEplKernel  Ret;
tEplEvent   Event;

    Event.m_EventSink = kEplEventSinkNmtk;
    Event.m_NetTime.m_dwNanoSec = 0;
    Event.m_NetTime.m_dwSec = 0;
    Event.m_EventType = kEplEventTypeNmtEvent;
    Event.m_pArg = &NmtEvent_p;
    Event.m_uiSize = sizeof(NmtEvent_p);

    Ret = EplEventuPost(&Event);


    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuGetNmtState
//
// Description: returns the actuell NMT-State
//
//
//
// Parameters:
//
//
// Returns:     tEplNmtState  = NMT-State
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplNmtState PUBLIC EplNmtuGetNmtState()
{
    return EplNmtuInstance_g.m_LocalNmtState;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuProcessEvent
//
// Description: processes events from event queue
//
//
//
// Parameters:  pEplEvent_p =   pointer to event
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuProcessEvent(
            tEplEvent* pEplEvent_p)
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // process event
    switch(pEplEvent_p->m_EventType)
    {
        // state change of NMT-Module
        case kEplEventTypeNmtStateChange:
        {
        tEplEventNmtStateChange* pNmtStateChange;

            // delete timer
            Ret = EplTimeruDeleteTimer(&EplNmtuInstance_g.m_TimerHdl);

            pNmtStateChange = (tEplEventNmtStateChange*)pEplEvent_p->m_pArg;

            EplNmtuInstance_g.m_LocalNmtState = pNmtStateChange->m_NewNmtState;

            // call cb-functions to inform higher layer
            if(EplNmtuInstance_g.m_pfnNmtChangeCb != NULL)
            {
                Ret = EplNmtuInstance_g.m_pfnNmtChangeCb(*pNmtStateChange);
            }

            if (Ret == kEplSuccessful)
            {   // everything is OK, so switch to next state if necessary
                switch (pNmtStateChange->m_NewNmtState)
                {
                    // EPL stack is not running
                    case kEplNmtGsOff:
                        break;

                    // first init of the hardware
                    case kEplNmtGsInitialising:
                    {
                        Ret = EplNmtuNmtEvent(kEplNmtEventEnterResetApp);
                        break;
                    }

                    // init of the manufacturer-specific profile area and the
                    // standardised device profile area
                    case kEplNmtGsResetApplication:
                    {
                        Ret = EplNmtuNmtEvent(kEplNmtEventEnterResetCom);
                        break;
                    }

                    // init of the communication profile area
                    case kEplNmtGsResetCommunication:
                    {
                        Ret = EplNmtuNmtEvent(kEplNmtEventEnterResetConfig);
                        break;
                    }

                    // build the configuration with infos from OD
                    case kEplNmtGsResetConfiguration:
                    {
                    unsigned int uiNodeId;

#if EPL_NMT_MAX_NODE_ID > 0
                        // configure the DLL (PReq/PRes payload limits and PRes timeout)
                        Ret = EplNmtuConfigureDll();
                        if (Ret != kEplSuccessful)
                        {
                            break;
                        }
#endif // EPL_NMT_MAX_NODE_ID > 0

                        // get node ID from OD
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) != 0) || (EPL_OBD_USE_KERNEL != FALSE)
                        uiNodeId = EplObduGetNodeId(EPL_MCO_PTR_INSTANCE_PTR);
#else
                        uiNodeId = 0;
#endif
                        //check node ID if not should be master or slave
                        if (uiNodeId == EPL_C_ADR_MN_DEF_NODE_ID)
                        {   // node shall be MN
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                            Ret = EplNmtuNmtEvent(kEplNmtEventEnterMsNotActive);
#else
                            TRACE("EplNmtuProcess(): no MN functionality implemented\n");
#endif
                        }
                        else
                        {   // node shall be CN
                            Ret = EplNmtuNmtEvent(kEplNmtEventEnterCsNotActive);
                        }
                        break;
                    }

                    //-----------------------------------------------------------
                    // CN part of the state machine

                    // node listens for EPL-Frames and check timeout
                    case kEplNmtCsNotActive:
                    {
                    DWORD           dwBasicEthernetTimeout;
                    tEplObdSize     ObdSize;
                    tEplTimerArg    TimerArg;

                        // create timer to switch automatically to BasicEthernet if no MN available in network

                        // read NMT_CNBasicEthernetTimeout_U32 from OD
                        ObdSize = sizeof(dwBasicEthernetTimeout);
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) != 0) || (EPL_OBD_USE_KERNEL != FALSE)
                        Ret = EplObduReadEntry(EPL_MCO_PTR_INSTANCE_PTR_
                                                0x1F99,
                                                0x00,
                                                &dwBasicEthernetTimeout,
                                                &ObdSize);
#else
                        Ret = kEplObdIndexNotExist;
#endif
                        if (Ret != kEplSuccessful)
                        {
                            break;
                        }
                        if (dwBasicEthernetTimeout != 0)
                        {   // BasicEthernet is enabled
                            // convert us into ms
                            dwBasicEthernetTimeout = dwBasicEthernetTimeout / 1000;
                            if (dwBasicEthernetTimeout == 0)
                            {   // timer was below one ms
                                // set one ms
                                dwBasicEthernetTimeout = 1;
                            }
                            TimerArg.m_EventSink = kEplEventSinkNmtk;
                            TimerArg.m_Arg.m_dwVal = (DWORD) kEplNmtEventTimerBasicEthernet;
                            Ret = EplTimeruModifyTimerMs(&EplNmtuInstance_g.m_TimerHdl, (unsigned long) dwBasicEthernetTimeout, TimerArg);
                            // potential error is forwarded to event queue which generates error event
                        }
                        break;
                    }

                    // node processes only async frames
                    case kEplNmtCsPreOperational1:
                    {
                        break;
                    }

                    // node processes isochronous and asynchronous frames
                    case kEplNmtCsPreOperational2:
                    {
                        Ret = EplNmtuNmtEvent(kEplNmtEventEnterReadyToOperate);
                        break;
                    }

                    // node should be configured and application is ready
                    case kEplNmtCsReadyToOperate:
                    {
                        break;
                    }

                    // normal work state
                    case kEplNmtCsOperational:
                    {
                        break;
                    }

                    // node stopped by MN
                    // -> only process asynchronous frames
                    case kEplNmtCsStopped:
                    {
                        break;
                    }

                    // no EPL cycle
                    // -> normal ethernet communication
                    case kEplNmtCsBasicEthernet:
                    {
                        break;
                    }

                    //-----------------------------------------------------------
                    // MN part of the state machine

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                    // node listens for EPL-Frames and check timeout
                    case kEplNmtMsNotActive:
                    {
                    DWORD           dwBasicEthernetTimeout;
                    tEplObdSize     ObdSize;
                    tEplTimerArg    TimerArg;

                        // create timer to switch automatically to BasicEthernet/PreOp1 if no other MN active in network

                        // check NMT_StartUp_U32.Bit13
                        // read NMT_StartUp_U32 from OD
                        ObdSize = sizeof(dwBasicEthernetTimeout);
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) != 0) || (EPL_OBD_USE_KERNEL != FALSE)
                        Ret = EplObduReadEntry(EPL_MCO_PTR_INSTANCE_PTR_
                                                0x1F80,
                                                0x00,
                                                &dwBasicEthernetTimeout,
                                                &ObdSize);
#else
                        Ret = kEplObdIndexNotExist;
#endif
                        if(Ret != kEplSuccessful)
                        {
                            break;
                        }

                        if((dwBasicEthernetTimeout & EPL_NMTST_BASICETHERNET) == 0)
                        {   // NMT_StartUp_U32.Bit13 == 0
                            // new state PreOperational1
                            TimerArg.m_Arg.m_dwVal = (DWORD) kEplNmtEventTimerMsPreOp1;
                        }
                        else
                        {   // NMT_StartUp_U32.Bit13 == 1
                            // new state BasicEthernet
                            TimerArg.m_Arg.m_dwVal = (DWORD) kEplNmtEventTimerBasicEthernet;
                        }

                        // read NMT_BootTime_REC.MNWaitNotAct_U32 from OD
                        ObdSize = sizeof(dwBasicEthernetTimeout);
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) != 0) || (EPL_OBD_USE_KERNEL != FALSE)
                        Ret = EplObduReadEntry(EPL_MCO_PTR_INSTANCE_PTR_
                                                0x1F89,
                                                0x01,
                                                &dwBasicEthernetTimeout,
                                                &ObdSize);
#else
                        Ret = kEplObdIndexNotExist;
#endif
                        if(Ret != kEplSuccessful)
                        {
                            break;
                        }
                        // convert us into ms
                        dwBasicEthernetTimeout = dwBasicEthernetTimeout / 1000;
                        if (dwBasicEthernetTimeout == 0)
                        {   // timer was below one ms
                            // set one ms
                            dwBasicEthernetTimeout = 1;
                        }
                        TimerArg.m_EventSink = kEplEventSinkNmtk;
                        Ret = EplTimeruModifyTimerMs(&EplNmtuInstance_g.m_TimerHdl, (unsigned long) dwBasicEthernetTimeout, TimerArg);
                        // potential error is forwarded to event queue which generates error event
                        break;
                    }

                    // node processes only async frames
                    case kEplNmtMsPreOperational1:
                    {
                    DWORD           dwBasicEthernetTimeout = 0;
                    tEplObdSize     ObdSize;
                    tEplTimerArg    TimerArg;

                        // create timer to switch automatically to PreOp2 if MN identified all mandatory CNs

                        // read NMT_BootTime_REC.MNWaitPreOp1_U32 from OD
                        ObdSize = sizeof(dwBasicEthernetTimeout);
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) != 0) || (EPL_OBD_USE_KERNEL != FALSE)
                        Ret = EplObduReadEntry(EPL_MCO_PTR_INSTANCE_PTR_
                                                0x1F89,
                                                0x03,
                                                &dwBasicEthernetTimeout,
                                                &ObdSize);
                        if(Ret != kEplSuccessful)
                        {
                            // ignore error, because this timeout is optional
                            dwBasicEthernetTimeout = 0;
                        }
#endif
                        if (dwBasicEthernetTimeout == 0)
                        {   // delay is deactivated
                            // immediately post timer event
                            Ret = EplNmtuNmtEvent(kEplNmtEventTimerMsPreOp2);
                            break;
                        }
                        // convert us into ms
                        dwBasicEthernetTimeout = dwBasicEthernetTimeout / 1000;
                        if (dwBasicEthernetTimeout == 0)
                        {   // timer was below one ms
                            // set one ms
                            dwBasicEthernetTimeout = 1;
                        }
                        TimerArg.m_EventSink = kEplEventSinkNmtk;
                        TimerArg.m_Arg.m_dwVal = (DWORD) kEplNmtEventTimerMsPreOp2;
                        Ret = EplTimeruModifyTimerMs(&EplNmtuInstance_g.m_TimerHdl, (unsigned long) dwBasicEthernetTimeout, TimerArg);
                        // potential error is forwarded to event queue which generates error event
                        break;
                    }

                    // node processes isochronous and asynchronous frames
                    case kEplNmtMsPreOperational2:
                    {
                        break;
                    }

                    // node should be configured and application is ready
                    case kEplNmtMsReadyToOperate:
                    {
                        break;
                    }

                    // normal work state
                    case kEplNmtMsOperational:
                    {
                        break;
                    }

                    // no EPL cycle
                    // -> normal ethernet communication
                    case kEplNmtMsBasicEthernet:
                    {
                        break;
                    }
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

                    default:
                    {
                        Ret = kEplNmtInvalidState;
                        TRACE("EplNmtuProcess(): unhandled NMT state 0x%X\n", pNmtStateChange->m_NewNmtState);
                    }
                }
            }
            else if (Ret == kEplReject)
            {   // application wants to change NMT state itself
                // it's OK
                Ret = kEplSuccessful;
            }

            EPL_DBGLVL_NMTU_TRACE("EplNmtuProcessEvent(): NMT-State-Maschine announce change of NMT State\n");
            break;
        }

        default:
        {
            Ret = kEplNmtInvalidEvent;
            break;
        }

    }

//Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtuRegisterStateChangeCb
//
// Description: register Callback-function go get informed about a
//              NMT-Change-State-Event
//
//
//
// Parameters:  pfnEplNmtStateChangeCb_p = functionpointer
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtuRegisterStateChangeCb(
            tEplNmtuStateChangeCallback pfnEplNmtStateChangeCb_p)
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    // save callback-function in modul global var
    EplNmtuInstance_g.m_pfnNmtChangeCb = pfnEplNmtStateChangeCb_p;

    return Ret;

}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#if EPL_NMT_MAX_NODE_ID > 0

//---------------------------------------------------------------------------
//
// Function:    EplNmtuConfigureDll
//
// Description: configures PReq/PRes payload limits and PRes timeouts in DLL
//              for each active node in object 0x1F81.
//
// Parameters:  void
//
// Returns:     tEplKernel      = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtuConfigureDll(void)
{
tEplKernel      Ret = kEplSuccessful;
DWORD           dwNodeCfg;
tEplObdSize     ObdSize;
tEplDllNodeInfo DllNodeInfo;
unsigned int    uiIndex;
BYTE            bCount;

    // read number of nodes from object 0x1F81/0
    ObdSize = sizeof (bCount);
    Ret = EplObduReadEntry(0x1F81, 0, &bCount, &ObdSize);
    if ((Ret == kEplObdIndexNotExist) || (Ret == kEplObdSubindexNotExist))
    {
        Ret = kEplSuccessful;
        goto Exit;
    }
    else if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    for (uiIndex = 1; uiIndex <= bCount; uiIndex++)
    {
        ObdSize = sizeof (dwNodeCfg);
        Ret = EplObduReadEntry(0x1F81, uiIndex, &dwNodeCfg, &ObdSize);
        if (Ret == kEplObdSubindexNotExist)
        {   // not all subindexes of object 0x1F81 have to exist
            continue;
        }
        else if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        if ((dwNodeCfg & (EPL_NODEASSIGN_NODE_EXISTS | EPL_NODEASSIGN_ASYNCONLY_NODE)) == EPL_NODEASSIGN_NODE_EXISTS)
        {   // node exists and runs in isochronous phase
            DllNodeInfo.m_uiNodeId = uiIndex;

            ObdSize = sizeof (DllNodeInfo.m_wPresPayloadLimit);
            Ret = EplObduReadEntry(0x1F8D, uiIndex, &DllNodeInfo.m_wPresPayloadLimit, &ObdSize);
            if ((Ret == kEplObdIndexNotExist) || (Ret == kEplObdSubindexNotExist))
            {
                DllNodeInfo.m_wPresPayloadLimit = 0;
            }
            else if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            if ((dwNodeCfg & (EPL_NODEASSIGN_NODE_IS_CN
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
                                                        | EPL_NODEASSIGN_PRES_CHAINING
#endif
                    )) == EPL_NODEASSIGN_NODE_IS_CN)
            {   // node is CN
                ObdSize = sizeof (DllNodeInfo.m_wPreqPayloadLimit);
                Ret = EplObduReadEntry(0x1F8B, uiIndex, &DllNodeInfo.m_wPreqPayloadLimit, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }

                ObdSize = sizeof (DllNodeInfo.m_dwPresTimeoutNs);
                Ret = EplObduReadEntry(0x1F92, uiIndex, &DllNodeInfo.m_dwPresTimeoutNs, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
            else
            {
                DllNodeInfo.m_dwPresTimeoutNs = 0;
                DllNodeInfo.m_wPreqPayloadLimit = 0;
            }
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

            Ret = EplDlluCalConfigNode(&DllNodeInfo);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }

Exit:
    return Ret;
}

#endif // EPL_NMT_MAX_NODE_ID > 0

#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)


// EOF

