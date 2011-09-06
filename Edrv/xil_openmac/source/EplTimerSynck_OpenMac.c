/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Implementation of synchronization timer module

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

                $RCSfile: EplTimerSynck_OpenMac.c,v $

                $Author: Michael.Ulbricht $

                $Revision: 1.8 $  $Date: 2010/06/30 09:58:08 $

                $State: Exp $

                Build Environment:
                    GNU

  -------------------------------------------------------------------------

  Revision History:

****************************************************************************/

#include "EplInc.h"
#include "kernel/EplTimerSynck.h"
#include "EplTgtTimeStamp_openMac.h"
#include "Benchmark.h"

#include "omethlib.h"
#ifdef __NIOS2__
#include "system.h"
#include <sys/alt_irq.h>
#include <alt_types.h>
#include <io.h>
#endif
#ifdef __MICROBLAZE__
#include "xparameters.h"
#include "IRQ_Microblaze.h"
#endif


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

#ifndef EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC
#define EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC      FALSE
#endif

#ifdef __NIOS2__
#define EPL_TIMER_SYNC_BASE		POWERLINK_0_MAC_CMP_BASE //from system.h
#define EPL_TIMER_SYNC_IRQ		POWERLINK_0_MAC_CMP_IRQ
#endif
#ifdef __MICROBLAZE__
#define EPL_TIMER_SYNC_BASE		(XPAR_XIL_OPENMAC_0_MEM0_BASEADDR + 0x3000) //from xparameters.h
#define EPL_TIMER_SYNC_IRQ		XPAR_XPS_INTC_0_XIL_OPENMAC_0_CMP_IR_N_INTR
#endif
/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplTimerSynck                                       */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: timer sync module implementation for openMAC
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define TIMER_HDL_SYNC          0
#define TIMER_HDL_LOSSOFSYNC    1
#define TIMER_HDL_INVALID       0xFF
#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
#define TIMER_HDL_LOSSOFSYNC2   2
#define TIMER_COUNT             3
#else
#define TIMER_COUNT             2
#endif

#define TIMEDIFF_COUNT_SHIFT    3
#define TIMEDIFF_COUNT          (1 << TIMEDIFF_COUNT_SHIFT)

#define PROPORTIONAL_FRACTION_SHIFT 3
#define PROPORTIONAL_FRACTION       (1 << PROPORTIONAL_FRACTION_SHIFT)


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    DWORD   m_dwAbsoluteTime;
    BOOL    m_fEnabled;

} tEplTimerSynckTimerInfo;

typedef struct
{
//    DWORD                       m_dwAdvanceShiftUs;
//    DWORD                       m_dwCycleLenUs;
    tEplTimerSynckCbSync        m_pfnCbSync;
    DWORD                       m_dwLossOfSyncToleranceNs;
    tEplTimerSynckCbLossOfSync  m_pfnCbLossOfSync;
    DWORD                       m_dwLossOfSyncTimeout;
#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    DWORD                       m_dwLossOfSyncTolerance2Ns;
    tEplTimerSynckCbLossOfSync  m_pfnCbLossOfSync2;
    DWORD                       m_dwLossOfSyncTimeout2;
#endif

    // EplTimerSynckCtrl specific
    BOOL                        m_fRunning;
    DWORD                       m_adwActualTimeDiff[TIMEDIFF_COUNT];
    unsigned int                m_uiActualTimeDiffNextIndex;
    DWORD                       m_dwMeanTimeDiff;
    DWORD                       m_dwConfiguredTimeDiff;
    DWORD                       m_dwAdvanceShift;
    DWORD                       m_dwRejectThreshold;
    DWORD                       m_dwTargetSyncTime;
    DWORD                       m_dwPreviousSyncTime;

    // EplTimerSynckDrv specific
    tEplTimerSynckTimerInfo     m_aTimerInfo[TIMER_COUNT];
    unsigned int                m_uiActiveTimerHdl;

} tEplTimerSynckInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerSynckInstance   EplTimerSynckInstance_l;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplTimerSynckCtrlDoSyncAdjustment (DWORD dwTimeStamp_p);
static void  EplTimerSynckCtrlSetConfiguredTimeDiff (DWORD dwConfiguredTimeDiff_p);
static void  EplTimerSynckCtrlUpdateLossOfSyncTolerance (void);
static DWORD EplTimerSynckCtrlGetNextAbsoluteTime   (unsigned int uiTimerHdl_p, DWORD dwCurrentTime_p);

static inline void  EplTimerSynckDrvCompareInterruptEnable  (void);
static inline void  EplTimerSynckDrvCompareInterruptDisable (void);
static inline void  EplTimerSynckDrvSetCompareValue         (DWORD dwVal);
static inline DWORD EplTimerSynckDrvGetTimeValue            (void);

#ifdef __NIOS2__
static void EplTimerSynckDrvInterruptHandler (void* pArg_p, alt_u32 dwInt_p);
#endif

#ifdef __MICROBLAZE__
static void EplTimerSynckDrvInterruptHandler (void* pArg_p);
#endif

static tEplKernel EplTimerSynckDrvModifyTimerAbs(unsigned int uiTimerHdl_p,
                                                 DWORD        dwAbsoluteTime_p);

static tEplKernel EplTimerSynckDrvModifyTimerRel(unsigned int uiTimerHdl_p,
                                                 int          iTimeAdjustment_p,
                                                 DWORD*       pdwAbsoluteTime_p,
                                                 BOOL*        pfAbsoluteTimeAlreadySet_p);

static tEplKernel EplTimerSynckDrvDeleteTimer(unsigned int uiTimerHdl_p);


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckAddInstance()
//
// Description: initializes the synchronization timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckAddInstance (void)
{
tEplKernel      Ret = kEplSuccessful;


    EPL_MEMSET(&EplTimerSynckInstance_l, 0, sizeof (EplTimerSynckInstance_l));

    EplTimerSynckDrvCompareInterruptDisable();

    EplTimerSynckDrvSetCompareValue( 0 );

#ifdef __NIOS2__
    if (alt_irq_register(EPL_TIMER_SYNC_IRQ, NULL, EplTimerSynckDrvInterruptHandler))
    {
        Ret = kEplNoResource;
    }
#endif

#ifdef __MICROBLAZE__
	{
    	XIntc *Intc = getIntc(); //get Interrupt Controller Instance
    	XStatus Status = 0;
		
		PRINTF0("Connect Timer Synck to Interrupt Controller:\n");
		Status = XIntc_Connect(	Intc, 
								EPL_TIMER_SYNC_IRQ, 
								(XInterruptHandler)EplTimerSynckDrvInterruptHandler, 
								NULL);
		if(Status != XST_SUCCESS)
		{
			Ret = kEplNoResource;
			goto Exit;
		}
		PRINTF0("done!\n");
	}
Exit:
#endif

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckDelInstance()
//
// Description: shuts down the synchronization timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckDelInstance (void)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckDrvCompareInterruptDisable();
    EplTimerSynckDrvSetCompareValue( 0 );
#ifdef __NIOS2__
    alt_irq_register(EPL_TIMER_SYNC_IRQ, NULL, NULL);
#endif

#ifdef __MICROBLAZE__
	{
    	XIntc *Intc = getIntc(); //get Interrupt Controller Instance

		XIntc_Disconnect(Intc, EPL_TIMER_SYNC_IRQ); 
	}
#endif

    EPL_MEMSET(&EplTimerSynckInstance_l, 0, sizeof (EplTimerSynckInstance_l));

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckRegSyncHandler()
//
// Description: registers handler for synchronized periodic call back
//
// Parameters:  pfnTimerSynckCbSync_p   = pointer to callback function,
//                                        which will be called in interrupt context.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckRegSyncHandler (tEplTimerSynckCbSync pfnTimerSynckCbSync_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_pfnCbSync = pfnTimerSynckCbSync_p;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckRegLossOfSyncHandler()
//
// Description: registers handler for loss of sync
//
// Parameters:  pfnTimerSynckCbSync_p   = pointer to callback function
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckRegLossOfSyncHandler (tEplTimerSynckCbLossOfSync pfnTimerSynckCbLossOfSync_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_pfnCbLossOfSync = pfnTimerSynckCbLossOfSync_p;

    return Ret;

}


#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)

//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckRegLossOfSyncHandler2()
//
// Description: registers handler for second loss of sync (needed for PResFallBackTimeout)
//
// Parameters:  pfnTimerSynckCbSync2_p  = pointer to callback function
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckRegLossOfSyncHandler2 (tEplTimerSynckCbLossOfSync pfnTimerSynckCbLossOfSync2_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_pfnCbLossOfSync2 = pfnTimerSynckCbLossOfSync2_p;

    return Ret;

}

#endif


//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckSetSyncShiftUs()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckSetSyncShiftUs (DWORD dwAdvanceShiftUs_p)
{
tEplKernel      Ret = kEplSuccessful;


//    EplTimerSynckInstance_l.m_dwAdvanceShiftUs = dwAdvanceShiftUs_p;
    EplTimerSynckInstance_l.m_dwAdvanceShift = OMETH_US_2_TICKS(dwAdvanceShiftUs_p);

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckSetCycleLenUs()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckSetCycleLenUs (DWORD dwCycleLenUs_p)
{
tEplKernel      Ret = kEplSuccessful;

    EplTimerSynckCtrlSetConfiguredTimeDiff(OMETH_US_2_TICKS(dwCycleLenUs_p));

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckSetLossOfSyncToleranceNs()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckSetLossOfSyncToleranceNs (DWORD dwLossOfSyncToleranceNs_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_dwLossOfSyncToleranceNs = dwLossOfSyncToleranceNs_p;

    EplTimerSynckCtrlUpdateLossOfSyncTolerance();

    return Ret;

}


#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)

//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckSetLossOfSyncTolerance2Ns()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckSetLossOfSyncTolerance2Ns (DWORD dwLossOfSyncTolerance2Ns_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_dwLossOfSyncTolerance2Ns = dwLossOfSyncTolerance2Ns_p;

    if (dwLossOfSyncTolerance2Ns_p > 0)
    {
        EplTimerSynckInstance_l.m_dwLossOfSyncTimeout2 = EplTimerSynckInstance_l.m_dwConfiguredTimeDiff
                + OMETH_NS_2_TICKS(EplTimerSynckInstance_l.m_dwLossOfSyncTolerance2Ns);
    }
    else
    {
        EplTimerSynckInstance_l.m_dwLossOfSyncTimeout2 = 0;
    }

    return Ret;

}

#endif


//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckTriggerAtTimeStamp()
//
// Description:
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckTriggerAtTimeStamp(tEplTgtTimeStamp* pTimeStamp_p)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = EplTimerSynckDrvModifyTimerAbs(TIMER_HDL_LOSSOFSYNC,
                                      (pTimeStamp_p->m_dwTimeStamp + EplTimerSynckInstance_l.m_dwLossOfSyncTimeout));
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    if (EplTimerSynckInstance_l.m_dwLossOfSyncTimeout2 > 0)
    {
        Ret = EplTimerSynckDrvModifyTimerAbs(TIMER_HDL_LOSSOFSYNC2,
                                          (pTimeStamp_p->m_dwTimeStamp + EplTimerSynckInstance_l.m_dwLossOfSyncTimeout2));
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

    Ret = EplTimerSynckCtrlDoSyncAdjustment(pTimeStamp_p->m_dwTimeStamp);

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckStopSync()
//
// Description:
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckStopSync (void)
{
tEplKernel      Ret = kEplSuccessful;

    EplTimerSynckInstance_l.m_fRunning = FALSE;

    Ret = EplTimerSynckDrvDeleteTimer(TIMER_HDL_SYNC);
    Ret = EplTimerSynckDrvDeleteTimer(TIMER_HDL_LOSSOFSYNC);
#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    Ret = EplTimerSynckDrvDeleteTimer(TIMER_HDL_LOSSOFSYNC2);
#endif

    return Ret;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          S U B C L A S S  EplTimerSynckCtrl                             */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: closed loop control implementation
//
/***************************************************************************/

static void  EplTimerSynckCtrlAddActualTimeDiff     (DWORD dwActualTimeDiff_p);
static void  EplTimerSynckCtrlCalcMeanTimeDiff      (void);
static void  EplTimerSynckCtrlUpdateRejectThreshold (void);


static tEplKernel EplTimerSynckCtrlDoSyncAdjustment (DWORD dwTimeStamp_p)
{
tEplKernel      Ret = kEplSuccessful;
DWORD           dwActualTimeDiff;
int             iDeviation;
BOOL            fCurrentSyncModified = FALSE;

    dwTimeStamp_p -= EplTimerSynckInstance_l.m_dwAdvanceShift;

    if (EplTimerSynckInstance_l.m_fRunning != FALSE)
    {
        dwActualTimeDiff = dwTimeStamp_p - EplTimerSynckInstance_l.m_dwPreviousSyncTime;

        EplTimerSynckCtrlAddActualTimeDiff(dwActualTimeDiff);

        iDeviation = dwTimeStamp_p - EplTimerSynckInstance_l.m_dwTargetSyncTime;

        iDeviation = iDeviation >> PROPORTIONAL_FRACTION_SHIFT;

        Ret = EplTimerSynckDrvModifyTimerRel(TIMER_HDL_SYNC, iDeviation, &EplTimerSynckInstance_l.m_dwTargetSyncTime, &fCurrentSyncModified);

        if (fCurrentSyncModified != FALSE)
        {   // set target to next sync
            EplTimerSynckInstance_l.m_dwTargetSyncTime += EplTimerSynckInstance_l.m_dwMeanTimeDiff;
        }
    }
    else
    {   // first trigger
        EplTimerSynckInstance_l.m_dwTargetSyncTime = dwTimeStamp_p + EplTimerSynckInstance_l.m_dwMeanTimeDiff;
        EplTimerSynckInstance_l.m_fRunning = TRUE;

        Ret = EplTimerSynckDrvModifyTimerAbs(TIMER_HDL_SYNC, EplTimerSynckInstance_l.m_dwTargetSyncTime);
    }

    EplTimerSynckInstance_l.m_dwPreviousSyncTime = dwTimeStamp_p;

    return Ret;
}


static void EplTimerSynckCtrlAddActualTimeDiff (DWORD dwActualTimeDiff_p)
{

    // always add small TimeDiff values
    // reject TimeDiff values which are too large
    if (dwActualTimeDiff_p < EplTimerSynckInstance_l.m_dwRejectThreshold)
    {
        EplTimerSynckInstance_l.m_adwActualTimeDiff[EplTimerSynckInstance_l.m_uiActualTimeDiffNextIndex]
            = dwActualTimeDiff_p;
        EplTimerSynckInstance_l.m_uiActualTimeDiffNextIndex++;
        EplTimerSynckInstance_l.m_uiActualTimeDiffNextIndex &= (TIMEDIFF_COUNT - 1);

        EplTimerSynckCtrlCalcMeanTimeDiff();
    }
    else
    {   // adjust target sync time, because of Loss of Sync
        for (; dwActualTimeDiff_p >= EplTimerSynckInstance_l.m_dwRejectThreshold;
             dwActualTimeDiff_p -= EplTimerSynckInstance_l.m_dwMeanTimeDiff,
             EplTimerSynckInstance_l.m_dwTargetSyncTime += EplTimerSynckInstance_l.m_dwMeanTimeDiff)
        {
        }
    }

}



static void EplTimerSynckCtrlCalcMeanTimeDiff (void)
{
int     nIdx;
DWORD   dwTimeDiffSum;

    dwTimeDiffSum = 0;

    for (nIdx=0; nIdx < TIMEDIFF_COUNT; nIdx++)
    {
        dwTimeDiffSum += EplTimerSynckInstance_l.m_adwActualTimeDiff[nIdx];
    }

    EplTimerSynckInstance_l.m_dwMeanTimeDiff = dwTimeDiffSum >> TIMEDIFF_COUNT_SHIFT;

}



static void EplTimerSynckCtrlSetConfiguredTimeDiff (DWORD dwConfiguredTimeDiff_p)
{
int     nIdx;

    EplTimerSynckInstance_l.m_dwConfiguredTimeDiff = dwConfiguredTimeDiff_p;

    for (nIdx=0; nIdx < TIMEDIFF_COUNT; nIdx++)
    {
        EplTimerSynckInstance_l.m_adwActualTimeDiff[nIdx] = dwConfiguredTimeDiff_p;
    }

    EplTimerSynckInstance_l.m_dwMeanTimeDiff = dwConfiguredTimeDiff_p;

    EplTimerSynckCtrlUpdateRejectThreshold();

}



static void EplTimerSynckCtrlUpdateLossOfSyncTolerance (void)
{
    EplTimerSynckCtrlUpdateRejectThreshold();
}



static void EplTimerSynckCtrlUpdateRejectThreshold (void)
{
DWORD   dwLossOfSyncTolerance;
DWORD   dwMaxRejectThreshold;

    dwLossOfSyncTolerance = OMETH_NS_2_TICKS(EplTimerSynckInstance_l.m_dwLossOfSyncToleranceNs);
    dwMaxRejectThreshold  = EplTimerSynckInstance_l.m_dwConfiguredTimeDiff >> 1;  // half of cycle length

    EplTimerSynckInstance_l.m_dwRejectThreshold = EplTimerSynckInstance_l.m_dwConfiguredTimeDiff;

    if (dwLossOfSyncTolerance > dwMaxRejectThreshold)
    {
        EplTimerSynckInstance_l.m_dwRejectThreshold += dwMaxRejectThreshold;
    }
    else
    {
        EplTimerSynckInstance_l.m_dwRejectThreshold += dwLossOfSyncTolerance;
    }

    EplTimerSynckInstance_l.m_dwLossOfSyncTimeout = EplTimerSynckInstance_l.m_dwConfiguredTimeDiff + dwLossOfSyncTolerance;
}



static DWORD EplTimerSynckCtrlGetNextAbsoluteTime (unsigned int uiTimerHdl_p, DWORD dwCurrentTime_p)
{
DWORD   dwNextAbsoluteTime;

    switch (uiTimerHdl_p)
    {
        case TIMER_HDL_SYNC:
        {
            dwNextAbsoluteTime = dwCurrentTime_p + EplTimerSynckInstance_l.m_dwMeanTimeDiff;
            break;
        }

        case TIMER_HDL_LOSSOFSYNC:
        {
            dwNextAbsoluteTime = dwCurrentTime_p + EplTimerSynckInstance_l.m_dwConfiguredTimeDiff;
            break;
        }

        default:
        {
            dwNextAbsoluteTime = 0;
            break;
        }
    }

    return dwNextAbsoluteTime;

}




/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          S U B C L A S S  EplTimerSynckDrv                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: timer driver implementation
//
/***************************************************************************/

#define TIMER_DRV_MIN_TIME_DIFF     500

static inline unsigned int EplTimerSynckDrvFindShortestTimer(void);

static void EplTimerSynckDrvConfigureShortestTimer(void);



static tEplKernel EplTimerSynckDrvModifyTimerAbs(unsigned int uiTimerHdl_p,
                                              DWORD        dwAbsoluteTime_p)
{
tEplKernel                  Ret = kEplSuccessful;
tEplTimerSynckTimerInfo*    pTimerInfo;

    if (uiTimerHdl_p >= TIMER_COUNT)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiTimerHdl_p];
    pTimerInfo->m_dwAbsoluteTime = dwAbsoluteTime_p;
    pTimerInfo->m_fEnabled = TRUE;

    EplTimerSynckDrvConfigureShortestTimer();

Exit:
    return Ret;

}


static tEplKernel EplTimerSynckDrvModifyTimerRel(unsigned int uiTimerHdl_p,
                                                 int          iTimeAdjustment_p,
                                                 DWORD*       pdwAbsoluteTime_p,
                                                 BOOL*        pfAbsoluteTimeAlreadySet_p)
{
tEplKernel                  Ret = kEplSuccessful;
tEplTimerSynckTimerInfo*    pTimerInfo;

    if (uiTimerHdl_p >= TIMER_COUNT)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiTimerHdl_p];
    if (pTimerInfo->m_dwAbsoluteTime == *pdwAbsoluteTime_p)
    {
        *pfAbsoluteTimeAlreadySet_p = TRUE;
    }
    else
    {
        *pfAbsoluteTimeAlreadySet_p = FALSE;
    }

    pTimerInfo->m_dwAbsoluteTime += iTimeAdjustment_p;

    *pdwAbsoluteTime_p = pTimerInfo->m_dwAbsoluteTime;
    pTimerInfo->m_fEnabled = TRUE;

    EplTimerSynckDrvConfigureShortestTimer();

Exit:
    return Ret;

}



static tEplKernel EplTimerSynckDrvDeleteTimer(unsigned int uiTimerHdl_p)
{
tEplKernel                  Ret = kEplSuccessful;
tEplTimerSynckTimerInfo*    pTimerInfo;

    if (uiTimerHdl_p >= TIMER_COUNT)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiTimerHdl_p];
    pTimerInfo->m_fEnabled = FALSE;

    EplTimerSynckDrvConfigureShortestTimer();

Exit:
    return Ret;

}


static inline unsigned int EplTimerSynckDrvFindShortestTimer(void)
{
unsigned int                uiTargetTimerHdl;
unsigned int                uiCurrentTimerHdl;
tEplTimerSynckTimerInfo*    pTimerInfo;
DWORD                       dwAbsoluteTime = 0;

    uiTargetTimerHdl = TIMER_HDL_INVALID;

    for (pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[0],
         uiCurrentTimerHdl = 0;
         uiCurrentTimerHdl < TIMER_COUNT;
         pTimerInfo++, uiCurrentTimerHdl++)
    {
        if (pTimerInfo->m_fEnabled != FALSE)
        {
            if ((uiTargetTimerHdl == TIMER_HDL_INVALID)
                || ((long)(pTimerInfo->m_dwAbsoluteTime - dwAbsoluteTime) < 0))
            {
                dwAbsoluteTime = pTimerInfo->m_dwAbsoluteTime;
                uiTargetTimerHdl = (unsigned int)(pTimerInfo - &EplTimerSynckInstance_l.m_aTimerInfo[0]);
            }
        }
    }

    return uiTargetTimerHdl;
}


static void EplTimerSynckDrvConfigureShortestTimer(void)
{
unsigned int                uiNextTimerHdl;
tEplTimerSynckTimerInfo*    pTimerInfo;
DWORD                       dwTargetAbsoluteTime;
DWORD                       dwCurrentTime;

    EplTimerSynckDrvCompareInterruptDisable();

    uiNextTimerHdl = EplTimerSynckDrvFindShortestTimer();
    if (uiNextTimerHdl != TIMER_HDL_INVALID)
    {
        pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiNextTimerHdl];

        EplTimerSynckInstance_l.m_uiActiveTimerHdl = uiNextTimerHdl;
        dwTargetAbsoluteTime = pTimerInfo->m_dwAbsoluteTime;

        dwCurrentTime = EplTimerSynckDrvGetTimeValue();
        if ((long)(dwTargetAbsoluteTime - dwCurrentTime) < TIMER_DRV_MIN_TIME_DIFF)
        {
            dwTargetAbsoluteTime = dwCurrentTime + TIMER_DRV_MIN_TIME_DIFF;
        }

        EplTimerSynckDrvSetCompareValue(dwTargetAbsoluteTime);

        // enable timer
        EplTimerSynckDrvCompareInterruptEnable();
    }
    else
    {
        EplTimerSynckDrvSetCompareValue(0);
        EplTimerSynckInstance_l.m_uiActiveTimerHdl = TIMER_HDL_INVALID;
    }

}


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define TIMERCMP_REG_OFF_CTRL       4
#define TIMERCMP_REG_OFF_CMP_VAL    0
#define TIMERCMP_REG_OFF_STATUS     4
#define TIMERCMP_REG_OFF_TIME_VAL   0

//#if defined(__NIOS2__) || defined(__MICROBLAZE__)
static inline void EplTimerSynckDrvCompareInterruptDisable (void)
{
	#if defined(__NIOS2__)
	    IOWR_32DIRECT( EPL_TIMER_SYNC_BASE, TIMERCMP_REG_OFF_CTRL, 0 );
	#elif defined(__MICROBLAZE__)
	    DWORD *p = (DWORD*)EPL_TIMER_SYNC_BASE;
    	*(p+TIMERCMP_REG_OFF_CTRL/4) = 0;
	#endif
}

static inline void EplTimerSynckDrvCompareInterruptEnable (void)
{
	#if defined(__NIOS2__)
	    IOWR_32DIRECT( EPL_TIMER_SYNC_BASE, TIMERCMP_REG_OFF_CTRL, 1 );
	#elif defined(__MICROBLAZE__)
	    DWORD *p = (DWORD*)EPL_TIMER_SYNC_BASE;
	    *(p+TIMERCMP_REG_OFF_CTRL/4) = 1;
	#endif
}

static inline void EplTimerSynckDrvSetCompareValue (DWORD dwVal)
{

	#if defined(__NIOS2__)
	    IOWR_32DIRECT( EPL_TIMER_SYNC_BASE, TIMERCMP_REG_OFF_CMP_VAL, dwVal );
	#elif defined(__MICROBLAZE__)
	    DWORD *p = (DWORD*)EPL_TIMER_SYNC_BASE;

    	*(p+TIMERCMP_REG_OFF_CMP_VAL/4) = dwVal;

	#endif

}

static inline DWORD EplTimerSynckDrvGetTimeValue (void)
{
	#if defined(__NIOS2__)
	    return IORD_32DIRECT( EPL_TIMER_SYNC_BASE, TIMERCMP_REG_OFF_TIME_VAL );
	#elif defined(__MICROBLAZE__)
	    DWORD *p = (DWORD*)EPL_TIMER_SYNC_BASE;
    	return *(p+TIMERCMP_REG_OFF_TIME_VAL/4);
	#endif
}

#if defined(__NIOS2__) || defined(__MICROBLAZE__)
#if defined(__NIOS2__)
static void EplTimerSynckDrvInterruptHandler (void* pArg_p, alt_u32 dwInt_p)
#elif defined(__MICROBLAZE__)
static void EplTimerSynckDrvInterruptHandler (void* pArg_p)
#endif
{
unsigned int                uiTimerHdl;
tEplTimerSynckTimerInfo*    pTimerInfo;
unsigned int                uiNextTimerHdl;

    BENCHMARK_MOD_24_SET(4);

    uiTimerHdl = EplTimerSynckInstance_l.m_uiActiveTimerHdl;
    if (uiTimerHdl < TIMER_COUNT)
    {
        pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiTimerHdl];
        pTimerInfo->m_dwAbsoluteTime = EplTimerSynckCtrlGetNextAbsoluteTime(uiTimerHdl, pTimerInfo->m_dwAbsoluteTime);

        // execute the sync if it will elapse in a very short moment
        // to give the sync event the highest priority.
        uiNextTimerHdl = EplTimerSynckDrvFindShortestTimer();
        if ((uiNextTimerHdl != uiTimerHdl)
            && (uiNextTimerHdl == TIMER_HDL_SYNC))
        {
            pTimerInfo = &EplTimerSynckInstance_l.m_aTimerInfo[uiTimerHdl];

            if ((pTimerInfo->m_fEnabled != FALSE)
                && ((long)(pTimerInfo->m_dwAbsoluteTime - EplTimerSynckDrvGetTimeValue()) < TIMER_DRV_MIN_TIME_DIFF))
            {
                pTimerInfo->m_dwAbsoluteTime = EplTimerSynckCtrlGetNextAbsoluteTime(uiNextTimerHdl, pTimerInfo->m_dwAbsoluteTime);

                if (EplTimerSynckInstance_l.m_pfnCbSync != NULL)
                {
                    EplTimerSynckInstance_l.m_pfnCbSync();
                }
            }
        }

        switch (uiTimerHdl)
        {
            case TIMER_HDL_SYNC:
            {
                if (EplTimerSynckInstance_l.m_pfnCbSync != NULL)
                {
                    EplTimerSynckInstance_l.m_pfnCbSync();
                }
                break;
            }

            case TIMER_HDL_LOSSOFSYNC:
            {
                if (EplTimerSynckInstance_l.m_pfnCbLossOfSync != NULL)
                {
                    EplTimerSynckInstance_l.m_pfnCbLossOfSync();
                }
                break;
            }

#if (EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
            case TIMER_HDL_LOSSOFSYNC2:
            {
                if (EplTimerSynckInstance_l.m_pfnCbLossOfSync2 != NULL)
                {
                    EplTimerSynckInstance_l.m_pfnCbLossOfSync2();
                }
                break;
            }
#endif

            default:
            {
                break;
            }
        }
    }

    EplTimerSynckDrvConfigureShortestTimer();

    BENCHMARK_MOD_24_RESET(4);
    return;

}
#endif

