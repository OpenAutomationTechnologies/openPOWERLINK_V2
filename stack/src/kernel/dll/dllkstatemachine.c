/**
********************************************************************************
\file   dllkstatemachine.c

\brief  Implementation of DLL state machine

This file contains the implementation of the DLL state machine. It is part of
the DLL kernel module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
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
#include <common/oplkinc.h>
#include "dllkstatemachine.h"
#include "dllkframe.h"

#include <kernel/dllk.h>
#include <kernel/errhndk.h>

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
#include <kernel/hrestimer.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <kernel/edrvcyclic.h>
#endif

#include "dllk-internal.h"

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
typedef tOplkError (*tDllkStateFunc)(tNmtState nmtState_p,
                                     tNmtEvent nmtEvent_p,
                                     tEventDllError* pDllEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError processNmtMsPreop1(tNmtState nmtState_p,
                                     tNmtEvent nmtEvent_p,
                                     tEventDllError* pDllEvent_p);
static tOplkError processNmtMsFullCycle(tNmtState nmtState_p,
                                        tNmtEvent nmtEvent_p,
                                        tEventDllError* pDllEvent_p);
#endif

static tOplkError processCsFullCycleDllWaitPreq(tNmtState nmtState_p,
                                                tNmtEvent nmtEvent_p,
                                                tEventDllError* pDllEvent_p);
static tOplkError processCsFullCycleDllWaitSoc(tNmtState nmtState_p,
                                               tNmtEvent nmtEvent_p,
                                               tEventDllError* pDllEvent_p);
static tOplkError processCsFullCycleDllWaitSoa(tNmtState nmtState_p,
                                               tNmtEvent nmtEvent_p,
                                               tEventDllError* pDllEvent_p);
static tOplkError processCsFullCycleDllGsInit(tNmtState nmtState_p,
                                              tNmtEvent nmtEvent_p,
                                              tEventDllError* pDllEvent_p);

static tOplkError processCsStoppedDllWaitPreq(tNmtState nmtState_p,
                                              tNmtEvent nmtEvent_p,
                                              tEventDllError* pDllEvent_p);
static tOplkError processCsStoppedDllWaitSoc(tNmtState nmtState_p,
                                             tNmtEvent nmtEvent_p,
                                             tEventDllError* pDllEvent_p);
static tOplkError processCsStoppedDllWaitSoa(tNmtState nmtState_p,
                                             tNmtEvent nmtEvent_p,
                                             tEventDllError* pDllEvent_p);
static tOplkError processCsStoppedDllGsInit(tNmtState nmtState_p,
                                            tNmtEvent nmtEvent_p,
                                            tEventDllError* pDllEvent_p);

static BOOL triggerLossOfSocEvent(void);
static BOOL triggerLossOfSocEventOnFrameTimeout(void);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDllkStateFunc    pfnProcessCsFullCycle_l[] =
{
    processCsFullCycleDllGsInit,
    processCsFullCycleDllWaitPreq,
    processCsFullCycleDllWaitSoc,
    processCsFullCycleDllWaitSoa,
};

static tDllkStateFunc    pfnProcessCsStopped_l[] =
{
    processCsStoppedDllGsInit,
    processCsStoppedDllWaitPreq,
    processCsStoppedDllWaitSoc,
    processCsStoppedDllWaitSoa,
};


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Change DLL state

The function implements the main function of the DLL state machine. It
changes the DLL state depending on the NMT state and the received event.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkstatemachine_changeState(tNmtEvent nmtEvent_p,
                                        tNmtState nmtState_p)
{
    tOplkError      ret = kErrorOk;
    tEventDllError  dllEvent;

    dllEvent.dllErrorEvents = 0;
    dllEvent.nodeId = 0;
    dllEvent.nmtState = nmtState_p;
    dllEvent.oplkError = kErrorOk;

    switch (nmtState_p)
    {
        case kNmtCsPreOperational2:
        case kNmtCsReadyToOperate:
        case kNmtCsOperational:
            if (dllkInstance_g.dllState < kDllMsNonCyclic)     // ensure that only CS states are handled
                ret = pfnProcessCsFullCycle_l[dllkInstance_g.dllState](nmtState_p,
                                                                       nmtEvent_p,
                                                                       &dllEvent);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kNmtMsPreOperational2:
        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
            ret = processNmtMsFullCycle(nmtState_p, nmtEvent_p, &dllEvent);
            if (ret != kErrorOk)
                return ret;
            break;
#endif

        case kNmtGsOff:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
        case kNmtCsBasicEthernet:
            // enter DLL_GS_INIT
            dllkInstance_g.dllState = kDllGsInit;
            break;

        case kNmtCsNotActive:
        case kNmtCsPreOperational1:
        case kNmtRmsNotActive:
            if (nmtEvent_p == kNmtEventDllCeSoc)
                dllkInstance_g.dllState = kDllCsWaitPreq;       // SoC received - enter DLL_CS_WAIT_PREQ
            else
                dllkInstance_g.dllState = kDllGsInit;           // enter DLL_GS_INIT
            break;

        case kNmtCsStopped:
            if (dllkInstance_g.dllState < kDllMsNonCyclic)     // ensure that only CS states are handled
                ret = pfnProcessCsStopped_l[dllkInstance_g.dllState](nmtState_p,
                                                                     nmtEvent_p,
                                                                     &dllEvent);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kNmtMsNotActive:
        case kNmtMsBasicEthernet:
            break;

        case kNmtMsPreOperational1:
            ret = processNmtMsPreop1(nmtState_p, nmtEvent_p, &dllEvent);
            if (ret != kErrorOk)
                return ret;
            break;
#endif

        default:
            break;
    }

    if (dllEvent.dllErrorEvents != 0)
    {   // error event set -> post it to error handler
        ret = errhndk_postError(&dllEvent);
    }

    return ret;
}

//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes in NMT state MsPreoperational1

The function handles DLL state changes in NMT state MsPreoperational1

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtMsPreop1(tNmtState nmtState_p,
                                     tNmtEvent nmtEvent_p,
                                     tEventDllError* pDllEvent_p)
{
    tOplkError  ret = kErrorOk;
    tDllState   dummyDllState;

    UNUSED_PARAMETER(pDllEvent_p);

    if (dllkInstance_g.dllState != kDllMsNonCyclic)
    {   // stop cycle timer
        ret = edrvcyclic_stopCycle(FALSE);
        if (ret != kErrorOk)
            return ret;

        dllkInstance_g.dllState = kDllMsNonCyclic;
        // stop further processing, because it will be restarted by NMT MN module
        return ret;
    }

    switch (nmtEvent_p)
    {
        case kNmtEventDllMeSocTrig:
        case kNmtEventDllCeAsnd:
            // because of reduced POWERLINK cycle SoA shall be triggered, not SoC
            ret = dllkframe_asyncFrameNotReceived(dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq],
                                                  dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq]);
            if (ret != kErrorOk)
                return ret;

            // $$$ d.k. only continue with sending of the SoA, if the received ASnd was the requested one
            //          or the transmission of the previous SoA has already finished.
            //          If we receive multiple ASnd after a SoA, we will get kErrorInvalidOperation in SendSoa()
            //          otherwise.

            // go ahead and send SoA
            ret = dllkframe_mnSendSoa(nmtState_p,
                                      &dummyDllState,
                                      (dllkInstance_g.cycleCount >= C_DLL_PREOP1_START_CYCLES));
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s() send SoA failed failed with 0x%X\n", __func__, ret);
            }

            // increment cycle counter to detect if C_DLL_PREOP1_START_CYCLES empty cycles are elapsed
            dllkInstance_g.cycleCount++;
            ret = kErrorOk;
            // reprogramming of timer will be done in cbFrameTransmitted()
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL states changes in MN full cycle states

The function handles DLL state changes in the MN full cycle states
MsPreOperational2, MsReadyToOperate and MsOperational.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtMsFullCycle(tNmtState nmtState_p,
                                        tNmtEvent nmtEvent_p,
                                        tEventDllError* pDllEvent_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(nmtState_p);

    switch (nmtEvent_p)
    {
        case kNmtEventDllMeSocTrig:
            // update cycle counter
            if (dllkInstance_g.dllConfigParam.multipleCycleCnt > 0)
            {   // multiplexed cycle active
                dllkInstance_g.cycleCount = (dllkInstance_g.cycleCount + 1) %
                                                dllkInstance_g.dllConfigParam.multipleCycleCnt;
                // $$$ check multiplexed cycle restart
                //     -> toggle MC flag
                //     -> change node linked list
            }

            switch (dllkInstance_g.dllState)
            {
                case kDllMsNonCyclic:
                    // start continuous cycle timer
                    // ASndTimeout is checked on next SoC Tx callback function
                    ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle);
                    if (ret != kErrorOk)
                        return ret;

                    ret = edrvcyclic_startCycle(TRUE);
                    if (ret != kErrorOk)
                        return ret;

                    // initialize cycle counter
                    dllkInstance_g.cycleCount = 0;

                    dllkInstance_g.dllState = kDllMsWaitSocTrig;
                    // initialize SoAReq number for ProcessSync (cycle preparation)
                    dllkInstance_g.syncLastSoaReq = dllkInstance_g.curLastSoaReq;

                    // forward dummy SoA event to DLLk, ErrorHandler and PDO module
                    // to trigger preparation of first cycle
                    ret = dllk_postEvent(kEventTypeSync);
                    if (ret != kErrorOk)
                        return ret;
                    break;

                default:
                    // wrong DLL state / cycle time exceeded
                    pDllEvent_p->dllErrorEvents |= DLL_ERR_MN_CYCTIMEEXCEED;
                    dllkInstance_g.dllState = kDllMsWaitSocTrig;
                    break;
            }
            break;

        case kNmtEventDllMeAsndTimeout:
            // SoC has been sent, update the prescaleCycleCount
            if (dllkInstance_g.dllConfigParam.prescaler > 0)
            {
                if (++dllkInstance_g.prescaleCycleCount == dllkInstance_g.dllConfigParam.prescaler)
                {
                    dllkInstance_g.prescaleCycleCount = 0;
                    dllkInstance_g.mnFlag1 ^= PLK_FRAME_FLAG1_PS;
                }
            }
            // SoC has been sent, so ASnd should have been received
            // report if SoA was correctly answered
            ret = dllkframe_asyncFrameNotReceived(dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq],
                                                  dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq]);
            // switch SoAReq buffer
            dllkInstance_g.curLastSoaReq++;
            if (dllkInstance_g.curLastSoaReq >= DLLK_SOAREQ_COUNT)
                dllkInstance_g.curLastSoaReq = 0;
            break;

        default:
            break;
    }

    return ret;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS full cycle in DllWaitPreq

The function handles DLL state changes in full cycle NMT states and DLL
state WaitPreq.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsFullCycleDllWaitPreq(tNmtState nmtState_p,
                                                tNmtEvent nmtEvent_p,
                                                tEventDllError* pDllEvent_p)
{
    switch (nmtEvent_p)
    {
        case kNmtEventDllCePreq:                // DLL_CT2
            // enter DLL_CS_WAIT_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_RECVD_PREQ;
            dllkInstance_g.dllState = kDllCsWaitSoa;
            break;

        case kNmtEventDllCeFrameTimeout:        // DLL_CT8
            if (nmtState_p == kNmtCsPreOperational2)
            {   // ignore frame timeout in PreOp2,
                // because the previously configured cycle len
                // may be wrong.
                // 2008/10/15 d.k. If it would not be ignored,
                // we would go cyclically to PreOp1 and on next
                // SoC back to PreOp2.
                break;
            }

            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;

            // enter DLL_CS_WAIT_SOC
            dllkInstance_g.dllState = kDllCsWaitSoc;
            break;

#if defined(CONFIG_INCLUDE_MASND)
            case kNmtEventDllCeAInv:
                // check if multiplexed and PReq should have been received in this cycle
                // and if >= NMT_CS_READY_TO_OPERATE
                if ((dllkInstance_g.cycleCount == 0) &&
                    (nmtState_p >= kNmtCsReadyToOperate))
                {
                    pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_PREQ | DLL_ERR_CN_LOSS_SOA;
                }

                // enter DLL_CS_WAIT_SOC
                dllkInstance_g.dllState = kDllCsWaitSoc;
                break;
#endif

        case kNmtEventDllCeSoa:
            // check if multiplexed and PReq should have been received in this cycle
            // and if >= NMT_CS_READY_TO_OPERATE
            if ((dllkInstance_g.cycleCount == 0) && (nmtState_p >= kNmtCsReadyToOperate))
            {   // report DLL_CEV_LOSS_OF_PREQ
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_PREQ;
            }
            // enter DLL_CS_WAIT_SOC
            dllkInstance_g.dllState = kDllCsWaitSoc;
            break;

        case kNmtEventDllCeSoc:                 // DLL_CT7
        case kNmtEventDllCeAsnd:
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            //fall through

        case kNmtEventDllCePres:
        default:
            // remain in this state
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS full cycle in DllWaitSoC

The function handles DLL state changes in full cycle NMT states and DLL
state WaitSoC.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsFullCycleDllWaitSoc(tNmtState nmtState_p,
                                               tNmtEvent nmtEvent_p,
                                               tEventDllError* pDllEvent_p)
{
    switch (nmtEvent_p)
    {
        case kNmtEventDllCeSoc:             // DLL_CT1
            // start of cycle and isochronous phase
            // enter DLL_CS_WAIT_PREQ
            dllkInstance_g.dllState = kDllCsWaitPreq;

            // Valid Soc arrived -> Reset report flags!
            dllkInstance_g.lossSocStatus.fLossReported = FALSE;
            dllkInstance_g.lossSocStatus.fTimeoutOccurred = FALSE;
            break;

        case kNmtEventDllCeFrameTimeout:    // DLL_CT4
            if (nmtState_p == kNmtCsPreOperational2)
            {   // ignore frame timeout in PreOp2,
                // because the previously configured cycle len
                // may be wrong.
                // 2008/10/15 d.k. If it would not be ignored,
                // we would go cyclically to PreOp1 and on next
                // SoC back to PreOp2.
                break;
            }

            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            break;

        case kNmtEventDllCePres:
        case kNmtEventDllCePreq:
            // enter DLL_CS_WAIT_SOA
            dllkInstance_g.dllState = kDllCsWaitSoa;
            if (triggerLossOfSocEvent())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;
            break;

        case kNmtEventDllCeSoa:
            if (nmtState_p == kNmtCsPreOperational2)
            {   // logical loss of SoC is counted always in PreOp2
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;
            }
            else
            {   // in other states, avoid double counting with SoC timeouts
                if (triggerLossOfSocEvent())
                    pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;
            }
        case kNmtEventDllCeAsnd:
        default:
            // remain in this state
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS full cycle in DllWaitSoA

The function handles DLL state changes in full cycle NMT states and DLL
state WaitSoA.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsFullCycleDllWaitSoa(tNmtState nmtState_p,
                                               tNmtEvent nmtEvent_p,
                                               tEventDllError* pDllEvent_p)
{
    switch (nmtEvent_p)
    {
        case kNmtEventDllCeFrameTimeout:
            // DLL_CT3
            if (nmtState_p == kNmtCsPreOperational2)
            {   // ignore frame timeout in PreOp2,
                // because the previously configured cycle len
                // may be wrong.
                // 2008/10/15 d.k. If it would not be ignored,
                // we would go cyclically to PreOp1 and on next
                // SoC back to PreOp2.
                break;
            }

            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;


            dllkInstance_g.dllState = kDllCsWaitSoc;
            break;

        case kNmtEventDllCePreq:
            if (triggerLossOfSocEvent())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;

        case kNmtEventDllCeSoa:
            // enter DLL_CS_WAIT_SOC
            dllkInstance_g.dllState = kDllCsWaitSoc;
            // prepare new cycle -> Reset report flags!
            dllkInstance_g.lossSocStatus.fLossReported = FALSE;
            dllkInstance_g.lossSocStatus.fTimeoutOccurred = FALSE;
            break;

        case kNmtEventDllCeSoc:             // DLL_CT9
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            // enter DLL_CS_WAIT_PREQ
            dllkInstance_g.dllState = kDllCsWaitPreq;
            break;

        case kNmtEventDllCeAsnd:            // DLL_CT10
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;

        case kNmtEventDllCePres:
        default:
            // remain in this state
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS full cycle in DllGsInit

The function handles DLL state changes in full cycle NMT states and DLL
state GsInit.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsFullCycleDllGsInit(tNmtState nmtState_p,
                                              tNmtEvent nmtEvent_p,
                                              tEventDllError* pDllEvent_p)
{
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nmtEvent_p);
    UNUSED_PARAMETER(pDllEvent_p);

    // enter DLL_CS_WAIT_PREQ
    dllkInstance_g.dllState = kDllCsWaitPreq;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS stopped in DllWaitPReq

The function handles DLL state changes in CS stopped NMT state and DLL
state WaitPReq.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsStoppedDllWaitPreq(tNmtState nmtState_p,
                                              tNmtEvent nmtEvent_p,
                                              tEventDllError* pDllEvent_p)
{
    UNUSED_PARAMETER(nmtState_p);

    switch (nmtEvent_p)
    {
        case kNmtEventDllCePreq:            // DLL_CT2
            // enter DLL_CS_WAIT_SOA
            dllkInstance_g.dllState = kDllCsWaitSoa;
            break;

        case kNmtEventDllCeFrameTimeout:    // DLL_CT8
            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            //fall through

        case kNmtEventDllCeSoa:
            // NMT_CS_STOPPED active - it is Ok if no PReq was received
            // enter DLL_CS_WAIT_SOC
            dllkInstance_g.dllState = kDllCsWaitSoc;
            break;

        case kNmtEventDllCeSoc:             // DLL_CT7
        case kNmtEventDllCeAsnd:
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            //fall through

        case kNmtEventDllCePres:
        default:
            // remain in this state
            break;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS stopped in DllWaitSoC

The function handles DLL state changes in CS stopped NMT state and DLL
state WaitSoC.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsStoppedDllWaitSoc(tNmtState nmtState_p,
                                             tNmtEvent nmtEvent_p,
                                             tEventDllError* pDllEvent_p)
{
    UNUSED_PARAMETER(nmtState_p);

    switch (nmtEvent_p)
    {
        case kNmtEventDllCeSoc:             // DLL_CT1
            // start of cycle and isochronous phase - enter DLL_CS_WAIT_SOA
            dllkInstance_g.dllState = kDllCsWaitSoa;
            break;

        case kNmtEventDllCePreq:            // DLL_CT4
        case kNmtEventDllCeSoa:
            if (triggerLossOfSocEvent())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            break;

        case kNmtEventDllCeFrameTimeout:
            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            //fall through
#if defined(CONFIG_INCLUDE_MASND)
        case kNmtEventDllCeAInv:
#endif
        case kNmtEventDllCeAsnd:
        default:
            // remain in this state
            break;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS stopped in DllWaitSoA

The function handles DLL state changes in CS stopped NMT state and DLL
state WaitSoA.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsStoppedDllWaitSoa(tNmtState nmtState_p,
                                             tNmtEvent nmtEvent_p,
                                             tEventDllError* pDllEvent_p)
{
    UNUSED_PARAMETER(nmtState_p);

    switch (nmtEvent_p)
    {
        case kNmtEventDllCeFrameTimeout:            // DLL_CT3
            if (triggerLossOfSocEventOnFrameTimeout())
                pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;

            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            // fall through

#if defined(CONFIG_INCLUDE_MASND)
        case kNmtEventDllCeAInv:
            //  report DLL_ERR_CN_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
#endif

        case kNmtEventDllCeSoa:
            // enter DLL_CS_WAIT_SOC
            dllkInstance_g.dllState = kDllCsWaitSoc;
            break;

        case kNmtEventDllCeSoc:                     // DLL_CT9
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            // remain in DLL_CS_WAIT_SOA
            break;

        case kNmtEventDllCeAsnd:                    // DLL_CT10
            // report DLL_CEV_LOSS_SOA
            pDllEvent_p->dllErrorEvents |= DLL_ERR_CN_LOSS_SOA;
            // fall through

        case kNmtEventDllCePreq:
            // NMT_CS_STOPPED active and we do not expect any PReq
            // so just ignore it

        case kNmtEventDllCePres:
        default:
            // remain in this state
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Handle DLL state changes for CS stopped in GsInit

The function handles DLL state changes in CS stopped NMT state and DLL
state GsInit.

\param[in]      nmtEvent_p          Event to handle.
\param[in]      nmtState_p          Current NMT state.
\param[in,out]  pDllEvent_p         DLL error event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCsStoppedDllGsInit(tNmtState nmtState_p,
                                            tNmtEvent nmtEvent_p,
                                            tEventDllError* pDllEvent_p)
{
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nmtEvent_p);
    UNUSED_PARAMETER(pDllEvent_p);

    // enter DLL_CS_WAIT_PREQ
    dllkInstance_g.dllState = kDllCsWaitSoa;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Check if the loss of SoC event shall be triggered

The loss of SoC event shall only be reported once per cycle to the error
handler. This functions checks if the event was already reported in this
cycle.

\return The return indicates if a loss of SoC event shall be triggered
\retval TRUE       Trigger error event
\retval FALSE      Error event already reported in this cycle

*/
//------------------------------------------------------------------------------
static BOOL triggerLossOfSocEvent(void)
{
    BOOL fTriggerEvent = FALSE;

    if (!dllkInstance_g.lossSocStatus.fLossReported)
    {
        dllkInstance_g.lossSocStatus.fLossReported = TRUE;
        fTriggerEvent = TRUE;
    }

    return fTriggerEvent;
}

//------------------------------------------------------------------------------
/**
\brief  Check if the loss of SoC event shall be triggered on a frame timeout

The loss of SoC event shall only be reported once per cycle to the error
handler. This functions checks if the event was already reported in this
cycle in case of a frame timeout. Contrary to the logical checks for
loss of SoC detection the frame timeout always occurs for a lost SoC.
Therefore the timeout event is used to detect multiple loss of SoC after each
other.

\return The return indicates if a loss of SoC event shall be triggered
\retval TRUE       Trigger error event
\retval FALSE      Error event already reported in this cycle

*/
//------------------------------------------------------------------------------
static BOOL triggerLossOfSocEventOnFrameTimeout(void)
{
    BOOL fTriggerEvent = FALSE;

    if (!dllkInstance_g.lossSocStatus.fLossReported)
    {
        dllkInstance_g.lossSocStatus.fLossReported = TRUE;
        dllkInstance_g.lossSocStatus.fTimeoutOccurred = TRUE;

        // Loss of Soc will be reported and can be marked as such!
        fTriggerEvent = TRUE;
    }
    else
    {
        if (dllkInstance_g.lossSocStatus.fTimeoutOccurred)
        {
            // Loss and timeout already reported -> Second SoC lost
            fTriggerEvent = TRUE;
        }

        dllkInstance_g.lossSocStatus.fTimeoutOccurred = TRUE;
    }

    return fTriggerEvent;
}

/// \}
