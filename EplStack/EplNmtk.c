/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for NMT kernel part module

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

#include "kernel/EplNmtk.h"
#include "EplTimer.h"           // for tEplTimerEventArg

#include "kernel/EplDllk.h"     // for EplDllkProcess()

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)
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

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif
#define EPL_NMTK_DBG_POST_TRACE_VALUE(NmtEvent_p, OldNmtState_p, NewNmtState_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkNmtk << 28) | (NmtEvent_p << 16) \
                             | ((OldNmtState_p & 0xFF) << 8) \
                             | (NewNmtState_p & 0xFF))


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------
// struct for instance table
INSTANCE_TYPE_BEGIN

    EPL_MCO_DECL_INSTANCE_MEMBER ()

    STATIC  volatile    tEplNmtState    INST_FAR    m_NmtState;
    STATIC  volatile    BOOL            INST_FAR    m_fEnableReadyToOperate;
    STATIC  volatile    BOOL            INST_FAR    m_fAppReadyToOperate;
    STATIC  volatile    BOOL            INST_FAR    m_fTimerMsPreOp2;
    STATIC  volatile    BOOL            INST_FAR    m_fAllMandatoryCNIdent;
    STATIC  volatile    BOOL            INST_FAR    m_fFrozen;

INSTANCE_TYPE_END
//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
// This macro replace the unspecific pointer to an instance through
// the modul specific type for the local instance table. This macro
// must defined in each modul.
//#define tEplPtrInstance             tEplInstanceInfo MEM*

EPL_MCO_DECL_INSTANCE_VAR ()

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
EPL_MCO_DEFINE_INSTANCE_FCT ()


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <NMT_Kernel-Module>                                 */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: This module realize the NMT-State-Machine of the EPL-Stack
//
//
/***************************************************************************/
//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//
//---------------------------------------------------------------------------
//
// Function:        EplNmtkInit
//
// Description: initializes the first instance
//
//
//
// Parameters:  EPL_MCO_DECL_PTR_INSTANCE_PTR = Instance pointer
//              uiNodeId_p = Node Id of the lokal node
//
//
// Returns:     tEplKernel  =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtkInit(EPL_MCO_DECL_PTR_INSTANCE_PTR)
{
tEplKernel Ret;

    Ret = EplNmtkAddInstance (EPL_MCO_PTR_INSTANCE_PTR);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:        EplNmtkAddInstance
//
// Description: adds a new instance
//
//
//
// Parameters:  EPL_MCO_DECL_PTR_INSTANCE_PTR = Instance pointer
//
//
// Returns:     tEplKernel  =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtkAddInstance(EPL_MCO_DECL_PTR_INSTANCE_PTR)
{
EPL_MCO_DECL_INSTANCE_PTR_LOCAL
tEplKernel              Ret;
//tEplEvent               Event;
//tEplEventNmtStateChange NmtStateChange;

     // check if pointer to instance pointer valid
    // get free instance and set the globale instance pointer
    // set also the instance addr to parameterlist
    EPL_MCO_CHECK_PTR_INSTANCE_PTR ();
    EPL_MCO_GET_FREE_INSTANCE_PTR ();
    EPL_MCO_SET_PTR_INSTANCE_PTR ();

    // sign instance as used
    EPL_MCO_WRITE_INSTANCE_STATE (kStateUsed);


    Ret = kEplSuccessful;

    // initialize intern vaiables
    // 2006/07/31 d.k.: set NMT-State to kEplNmtGsOff
    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
    // set NMT-State to kEplNmtGsInitialising
    //EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;

    // set flags to FALSE
    EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
    EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
    EPL_MCO_GLB_VAR(m_fTimerMsPreOp2) = FALSE;
    EPL_MCO_GLB_VAR(m_fAllMandatoryCNIdent) = FALSE;
    EPL_MCO_GLB_VAR(m_fFrozen) = FALSE;

//    EPL_MCO_GLB_VAR(m_TimerHdl) = 0;

    // inform higher layer about state change
    // 2006/07/31 d.k.: The EPL API layer/application has to start NMT state
    //                  machine via NmtEventSwReset after initialisation of
    //                  all modules has been completed. DLL has to be initialised
    //                  after NMTk because NMT state shall not be uninitialised
    //                  at that time.
/*    NmtStateChange.m_NewNmtState = EPL_MCO_GLB_VAR(m_NmtState);
    NmtStateChange.m_NmtEvent = kEplNmtEventNoEvent;
    Event.m_EventSink = kEplEventSinkNmtu;
    Event.m_EventType = kEplEventTypeNmtStateChange;
    EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
    Event.m_pArg = &NmtStateChange;
    Event.m_uiSize = sizeof(NmtStateChange);
    Ret = EplEventkPost(&Event);
*/
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:        EplNmtkDelInstance
//
// Description: delete instance
//
//
//
// Parameters:  EPL_MCO_DECL_PTR_INSTANCE_PTR = Instance pointer
//
//
// Returns:     tEplKernel  =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
#if (EPL_USE_DELETEINST_FUNC != FALSE)
EPLDLLEXPORT tEplKernel PUBLIC EplNmtkDelInstance(EPL_MCO_DECL_PTR_INSTANCE_PTR)
{
tEplKernel              Ret = kEplSuccessful;
    // check for all API function if instance is valid
    EPL_MCO_CHECK_INSTANCE_STATE ();

    // set NMT-State to kEplNmtGsOff
    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;

    // sign instance as unused
    EPL_MCO_WRITE_INSTANCE_STATE (kStateUnused);

    // delete timer
//    Ret = EplTimerkDeleteTimer(&EPL_MCO_GLB_VAR(m_TimerHdl));

    return Ret;
}
#endif // (EPL_USE_DELETEINST_FUNC != FALSE)


//---------------------------------------------------------------------------
//
// Function:        EplNmtkProcess
//
// Description: main process function
//              -> process NMT-State-Maschine und read NMT-Events from Queue
//
//
//
// Parameters:  EPL_MCO_DECL_PTR_INSTANCE_PTR_ = Instance pointer
//              pEvent_p    =   Epl-Event with NMT-event to process
//
//
// Returns:     tEplKernel  =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplKernel PUBLIC EplNmtkProcess(EPL_MCO_DECL_PTR_INSTANCE_PTR_
                                              tEplEvent* pEvent_p)
{
tEplKernel              Ret;
tEplNmtState            OldNmtState;
tEplNmtEvent            NmtEvent;
tEplEvent               Event;
tEplEventNmtStateChange NmtStateChange;

    // check for all API function if instance is valid
    EPL_MCO_CHECK_INSTANCE_STATE ();

    Ret = kEplSuccessful;

    switch(pEvent_p->m_EventType)
    {
        case kEplEventTypeNmtEvent:
        {
            NmtEvent = *((tEplNmtEvent*)pEvent_p->m_pArg);
            break;
        }

        case kEplEventTypeTimer:
        {
            NmtEvent = (tEplNmtEvent)((tEplTimerEventArg*)pEvent_p->m_pArg)->m_Arg.m_dwVal;
            break;
        }
        default:
        {
            Ret = kEplNmtInvalidEvent;
            goto Exit;
        }
    }

    // save NMT-State
    // needed for later comparison to
    // inform higher layer about state change
    OldNmtState = EPL_MCO_GLB_VAR(m_NmtState);

    // NMT-State-Maschine
    switch(EPL_MCO_GLB_VAR(m_NmtState))
    {
        //-----------------------------------------------------------
        // general part of the statemaschine

        // first init of the hardware
        case kEplNmtGsOff:
        {
            // leave this state only if higher layer says so
            if (NmtEvent == kEplNmtEventSwReset)
            {   // NMT_GT8, NMT_GT1
                // new state kEplNmtGsInitialising
                EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
            }
            break;
        }

        // first init of the hardware
        case kEplNmtGsInitialising:
        {
            // leave this state only if higher layer says so

            // check events
            switch(NmtEvent)
            {
                // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // new state kEplNmtGsResetApplication
                case kEplNmtEventEnterResetApp:
                {   // NMT_GT10
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

        // init of the manufacturer-specific profile area and the
        // standardised device profile area
        case kEplNmtGsResetApplication:
        {
            // check events
            switch(NmtEvent)
            {
                // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // leave this state only if higher layer
                // say so
                case kEplNmtEventEnterResetCom:
                {   // NMT_GT11
                    // new state kEplNmtGsResetCommunication
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

        // init of the communication profile area
        case kEplNmtGsResetCommunication:
        {
            // check events
            switch(NmtEvent)
            {
                // 2006/07/31 d.k.: react also on NMT reset commands in ResetComm state
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // leave this state only if higher layer
                // say so
                case kEplNmtEventEnterResetConfig:
                {   // NMT_GT12
                    // new state kEplNmtGsResetCommunication
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

        // build the configuration with infos from OD
        case kEplNmtGsResetConfiguration:
        {
            // reset flags
            EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
            EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
            EPL_MCO_GLB_VAR(m_fFrozen) = FALSE;

            // check events
            switch(NmtEvent)
            {
                // 2006/07/31 d.k.: react also on NMT reset commands in ResetConf state
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                case kEplNmtEventResetCom:
                {   // NMT_GT5
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // leave this state only if higher layer says so
                case kEplNmtEventEnterCsNotActive:
                {   // Node should be CN (NMT_CT1)
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsNotActive;
                    break;

                }

                case kEplNmtEventEnterMsNotActive:
                {   // Node should be MN (NMT_MT1)
                    #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0)
                        // no MN functionality
                        // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
                        EPL_MCO_GLB_VAR(m_fFrozen) = TRUE;
                    #else

                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsNotActive;
                    #endif
                    break;

                }

                default:
                {
                    break;
                }
            }
            break;
        }

        //-----------------------------------------------------------
        // CN part of the statemaschine

        // node liste for EPL-Frames and check timeout
        case kEplNmtCsNotActive:
        {

            // check events
            switch(NmtEvent)
            {
                // 2006/07/31 d.k.: react also on NMT reset commands in NotActive state
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command Reset Configuration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // see if SoA or SoC received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // NMT_CT2
                    // new state PRE_OPERATIONAL1
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                // timeout for SoA and Soc
                case kEplNmtEventTimerBasicEthernet:
                {   // NMT_CT3
                    // new state BASIC_ETHERNET
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsBasicEthernet;
                    break;
                }

                default:
                {
                    break;
                }
            }// end of switch(NmtEvent)

            break;
        }

        // node processes only async frames
        case kEplNmtCsPreOperational1:
        {

            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command Reset Configuration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // check if SoC received
                case kEplNmtEventDllCeSoc:
                {   // NMT_CT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational2;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)

            break;
        }

        // node processes isochronous and asynchronous frames
        case kEplNmtCsPreOperational2:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command Reset Configuration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // NMT Command StopNode
                case kEplNmtEventStopNode:
                {   // NMT_CT8
                    // reset flags
                    EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
                    EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsStopped;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_CT11
                    // reset flags
                    EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
                    EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                // check if application is ready to operate
                case kEplNmtEventEnterReadyToOperate:
                {
                    // check if command NMTEnableReadyToOperate from MN was received
                    if(EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) == TRUE)
                    {   // reset flags
                        EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
                        EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
                        // change state (NMT_CT6)
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsReadyToOperate;
                    }
                    else
                    {   // set Flag (NMT_CT5)
                        EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = TRUE;
                    }
                    break;
                }

                // NMT Commando EnableReadyToOperate
                case kEplNmtEventEnableReadyToOperate:
                {
                    // check if application is ready
                    if(EPL_MCO_GLB_VAR(m_fAppReadyToOperate) == TRUE)
                    {   // reset flags
                        EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = FALSE;
                        EPL_MCO_GLB_VAR(m_fAppReadyToOperate) = FALSE;
                        // change state (NMT_CT6)
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsReadyToOperate;
                    }
                    else
                    {   // set Flag (NMT_CT5)
                        EPL_MCO_GLB_VAR(m_fEnableReadyToOperate) = TRUE;
                    }
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        // node should be configured und application is ready
        case kEplNmtCsReadyToOperate:
        {
             // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // NMT Command StopNode
                case kEplNmtEventStopNode:
                {   // NMT_CT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsStopped;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_CT11
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                // NMT Command StartNode
                case kEplNmtEventStartNode:
                {   // NMT_CT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsOperational;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        // normal work state
        case kEplNmtCsOperational:
        {

             // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // NMT Command StopNode
                case kEplNmtEventStopNode:
                {   // NMT_CT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsStopped;
                    break;
                }

                // NMT Command EnterPreOperational2
                case kEplNmtEventEnterPreOperational2:
                {   // NMT_CT9
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational2;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_CT11
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        // node stopped by MN
        // -> only process asynchronous frames
        case kEplNmtCsStopped:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // NMT Command EnterPreOperational2
                case kEplNmtEventEnterPreOperational2:
                {   // NMT_CT10
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational2;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_CT11
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        // no epl cycle
        // -> normal ethernet communication
        case kEplNmtCsBasicEthernet:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // error occured
                // d.k.: how does this error occur? on CRC errors
/*                case kEplNmtEventNmtCycleError:
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }
*/
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCePreq:
                case kEplNmtEventDllCePres:
                case kEplNmtEventDllCeSoa:
                {   // NMT_CT12
                    // EPL frame on net -> stop any communication
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)

            break;
        }

        //-----------------------------------------------------------
        // MN part of the statemaschine

        // MN listen to network
        // -> if no EPL traffic go to next state
        case kEplNmtMsNotActive:
        {
            #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0)
                // no MN functionality
                // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
                EPL_MCO_GLB_VAR(m_fFrozen) = TRUE;
            #else

                // check events
                switch(NmtEvent)
                {
                    // NMT Command SwitchOff
                    case kEplNmtEventCriticalError:
                    case kEplNmtEventSwitchOff:
                    {   // NMT_GT3
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                        break;
                    }

                    // NMT Command SwReset
                    case kEplNmtEventSwReset:
                    {   // NMT_GT8
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                        break;
                    }

                    // NMT Command ResetNode
                    case kEplNmtEventResetNode:
                    {   // NMT_GT4
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                        break;
                    }

                    // NMT Command ResetCommunication
                    // or internal Communication error
                    case kEplNmtEventResetCom:          // NMT_GT5
                    case kEplNmtEventInternComError:    // NMT_GT6
                    {
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                        break;
                    }

                    // NMT Command ResetConfiguration
                    case kEplNmtEventResetConfig:
                    {   // NMT_GT7
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                        break;
                    }

                    // EPL frames received
                    case kEplNmtEventDllCeSoc:
                    case kEplNmtEventDllCeSoa:
                    {   // other MN in network
                        // $$$ d.k.: generate error history entry
                        EPL_MCO_GLB_VAR(m_fFrozen) = TRUE;
                        break;
                    }

                    // timeout event
                    case kEplNmtEventTimerBasicEthernet:
                    {   // NMT_MT7
                        if (EPL_MCO_GLB_VAR(m_fFrozen) == FALSE)
                        {   // new state BasicEthernet
                            EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsBasicEthernet;
                        }
                        break;
                    }

                    // timeout event
                    case kEplNmtEventTimerMsPreOp1:
                    {   // NMT_MT2
                        if (EPL_MCO_GLB_VAR(m_fFrozen) == FALSE)
                        {   // new state PreOp1
                            EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational1;
                            EPL_MCO_GLB_VAR(m_fTimerMsPreOp2) = FALSE;
                            EPL_MCO_GLB_VAR(m_fAllMandatoryCNIdent) = FALSE;

                        }
                        break;
                    }

                    default:
                    {
                        break;
                    }

                }// end of switch(NmtEvent)

            #endif // ((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0)

            break;
        }
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        // MN process reduces epl cycle
        case kEplNmtMsPreOperational1:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // EPL frames received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // other MN in network
                    // $$$ d.k.: generate error history entry
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // error occured
                // d.k. MSPreOp1->CSPreOp1: nonsense -> keep state
                /*
                case kEplNmtEventNmtCycleError:
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }
                */

                case kEplNmtEventAllMandatoryCNIdent:
                {   // all mandatory CN identified
                    if (EPL_MCO_GLB_VAR(m_fTimerMsPreOp2) != FALSE)
                    {   // NMT_MT3
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational2;
                    }
                    else
                    {
                        EPL_MCO_GLB_VAR(m_fAllMandatoryCNIdent) = TRUE;
                    }
                    break;
                }

                case kEplNmtEventTimerMsPreOp2:
                {   // residence time for PreOp1 is elapsed
                    if (EPL_MCO_GLB_VAR(m_fAllMandatoryCNIdent) != FALSE)
                    {   // NMT_MT3
                        EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational2;
                    }
                    else
                    {
                        EPL_MCO_GLB_VAR(m_fTimerMsPreOp2) = TRUE;
                    }
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        // MN process full epl cycle
        case kEplNmtMsPreOperational2:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // EPL frames received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // other MN in network
                    // $$$ d.k.: generate error history entry
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_MT6
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational1;
                    break;
                }

                case kEplNmtEventEnterReadyToOperate:
                {   // NMT_MT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsReadyToOperate;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)

            break;
        }

        // all madatory nodes ready to operate
        // -> MN process full epl cycle
        case kEplNmtMsReadyToOperate:
        {

            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // EPL frames received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // other MN in network
                    // $$$ d.k.: generate error history entry
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_MT6
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational1;
                    break;
                }

                case kEplNmtEventEnterMsOperational:
                {   // NMT_MT5
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsOperational;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)

            break;
        }

        // normal eplcycle processing
        case kEplNmtMsOperational:
        {
            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // EPL frames received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // other MN in network
                    // $$$ d.k.: generate error history entry
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // error occured
                case kEplNmtEventNmtCycleError:
                {   // NMT_MT6
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtMsPreOperational1;
                    break;
                }

                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }

        //  normal ethernet traffic
        case kEplNmtMsBasicEthernet:
        {

            // check events
            switch(NmtEvent)
            {
                // NMT Command SwitchOff
                case kEplNmtEventCriticalError:
                case kEplNmtEventSwitchOff:
                {   // NMT_GT3
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsOff;
                    break;
                }

                // NMT Command SwReset
                case kEplNmtEventSwReset:
                {   // NMT_GT8
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsInitialising;
                    break;
                }

                // NMT Command ResetNode
                case kEplNmtEventResetNode:
                {   // NMT_GT4
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
                    break;
                }

                // NMT Command ResetCommunication
                // or internal Communication error
                case kEplNmtEventResetCom:          // NMT_GT5
                case kEplNmtEventInternComError:    // NMT_GT6
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // NMT Command ResetConfiguration
                case kEplNmtEventResetConfig:
                {   // NMT_GT7
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetConfiguration;
                    break;
                }

                // EPL frames received
                case kEplNmtEventDllCeSoc:
                case kEplNmtEventDllCeSoa:
                {   // other MN in network
                    // $$$ d.k.: generate error history entry
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetCommunication;
                    break;
                }

                // error occured
                // d.k. BE->PreOp1 on cycle error? No
/*                case kEplNmtEventNmtCycleError:
                {
                    EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtCsPreOperational1;
                    break;
                }
*/
                default:
                {
                    break;
                }

            }// end of switch(NmtEvent)
            break;
        }
#endif //#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

        default:
        {
            //DEBUG_EPL_DBGLVL_NMTK_TRACE0(EPL_DBGLVL_NMT ,"Error in EplNmtProcess: Unknown NMT-State");
            //EPL_MCO_GLB_VAR(m_NmtState) = kEplNmtGsResetApplication;
            Ret = kEplNmtInvalidState;
            goto Exit;
        }

    }// end of switch(NmtEvent)

    // inform higher layer about State-Change if needed
    if (OldNmtState != EPL_MCO_GLB_VAR(m_NmtState))
    {
        EPL_NMTK_DBG_POST_TRACE_VALUE(NmtEvent, OldNmtState, EPL_MCO_GLB_VAR(m_NmtState));

        EPL_DBGLVL_NMTK_TRACE2("EplNmtkProcess(NMT-Event = 0x%04X): New NMT-State = 0x%03X\n", NmtEvent, NmtStateChange.m_NewNmtState);

        NmtStateChange.m_NewNmtState = EPL_MCO_GLB_VAR(m_NmtState);
        NmtStateChange.m_OldNmtState = OldNmtState;
        NmtStateChange.m_NmtEvent = NmtEvent;
        Event.m_EventType = kEplEventTypeNmtStateChange;
        EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
        Event.m_pArg = &NmtStateChange;
        Event.m_uiSize = sizeof(NmtStateChange);

        // inform DLLk module about state change
        Event.m_EventSink = kEplEventSinkDllk;
        // d.k.: directly call DLLk process function, because
        //       1. execution of process function is still synchonized and serialized,
        //       2. it is the same as without event queues (i.e. well tested),
        //       3. DLLk will get those necessary events even if event queue is full
        //       4. event queue is very inefficient
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
        Ret = EplDllkProcess(&Event);
#else
        Ret = EplEventkPost(&Event);
#endif
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // inform higher layer about state change
        Event.m_EventSink = kEplEventSinkNmtu;
        Ret = EplEventkPost(&Event);
    }

Exit:

    return Ret;
}

/*
//---------------------------------------------------------------------------
//
// Function:    EplNmtkGetNmtState
//
// Description: return the actuell NMT-State and the bits
//              to for MN- or CN-mode
//
//
//
// Parameters:  EPL_MCO_DECL_PTR_INSTANCE_PTR_ = Instancepointer
//
//
// Returns:     tEplNmtState = NMT-State
//
//
// State:
//
//---------------------------------------------------------------------------
EPLDLLEXPORT tEplNmtState PUBLIC EplNmtkGetNmtState(EPL_MCO_DECL_PTR_INSTANCE_PTR)
{
tEplNmtState NmtState;

    NmtState = EPL_MCO_GLB_VAR(m_NmtState);

    return NmtState;

}
*/

//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//
EPL_MCO_DECL_INSTANCE_FCT ()
//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------


#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)

// EOF

