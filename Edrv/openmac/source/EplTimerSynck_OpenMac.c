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

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GNU

  -------------------------------------------------------------------------

  Revision History:

****************************************************************************/

#include "EplInc.h"
#include "EplTimerSynck.h"
#include "EplTgtTimeStamp_openMac.h"



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



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplTimerSynck                                             */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
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
#define TIMER_COUNT             2

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

} tEplTimerSynckTimerEntry;

typedef struct
{
    DWORD                       m_dwAdvanceShiftUs;
    DWORD                       m_dwCycleLenUs;
    DWORD                       m_dwLossOfSyncToleranceUs;
    tEplTimerSynckCbSync        m_pfnCbSync;
    tEplTimerSynckCbLossOfSync  m_pfnCbLossOfSync;
    DWORD                       m_dwLossOfSyncTimeout;

    // EplTimerSynckCtrl specific
    DWORD                       m_aActualTimeDiff[TIMEDIFF_COUNT];
    unsigned int                m_uiActualTimeDiffIndex;
    DWORD                       m_dwMeanTimeDiff;
    DWORD                       m_dwTargetTimeDiff;
    DWORD                       m_dwAdvanceShift;

    // EplTimerSynckDrv specific
    tEplTimerSynckTimerEntry    m_aTimerEntry[TIMER_COUNT];
    unsigned int                m_uiActiveTimerHdl;

} tEplTimerSynckInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimeSynckInstance    EplTimeSynckInstance_l;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static inline void  EplTimerSynckCompareInterruptDisable (void);
static inline void  EplTimerSynckCompareInterruptEnable  (void);
static inline DWORD EplTimerSynckGetTimeValue            (void);
static inline void  EplTimerSynckSetCompareValue         (DWORD dwVal);

static void EplTimerSynckInterruptHandler (void* pArg_p, alt_u32 dwInt_p);


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

    EplTimerSynckCompareInterruptDisable();
    EplTimerSynckSetCompareValue( 0 );

    if (alt_irq_register(OPENMAC_0_TIMERCMP_IRQ, NULL, EplTimerSynckInterruptHandler))
    {
        Ret = kEplNoResource;
    }

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


    EplTimerSynckCompareInterruptDisable();
    EplTimerSynckSetCompareValue( 0 );

    alt_irq_register(OPENMAC_0_TIMERCMP_IRQ, NULL, NULL);

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


    EplTimerSynckInstance_l.m_dwAdvanceShiftUs = dwAdvanceShiftUs_p;

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


    EplTimerSynckInstance_l.m_dwCycleLenUs = dwCycleLenUs_p;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerSynckSetLossOfSyncToleranceUs()
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

tEplKernel PUBLIC EplTimerSynckSetLossOfSyncToleranceUs (DWORD dwLossOfSyncToleranceUs_p)
{
tEplKernel      Ret = kEplSuccessful;


    EplTimerSynckInstance_l.m_dwLossOfSyncToleranceUs = dwLossOfSyncToleranceUs_p;

    return Ret;

}



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


    // modify LossOfSync timer

    EplTimerSynckCtrlDoSyncAdjustment(pTimeStamp_p->m_dwTimeStamp);

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



    return Ret;
}
    


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


static void EplTimerSynckCtrlDoSyncAdjustment (DWORD dwTimeStamp_p)
{
    
    
}



static void EplTimerSynckCtrlCalcMeanTimeDiff (void)
{
}



static void EplTimerSynckCtrlSetTargetTimeDiff (DWORD dwTargetTimeDiff_p)
{
}



static DWORD EplTimerSynckCtrlGetPeriod (unsigned int uiTimerHdl_p)
{

    switch ()

}



static tEplKernel EplTimerSynckDrvModifyTimer(unsigned int uiTimerHdl_p,
                                           DWORD        dwAbsoluteTime_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
DWORD                       dwTimeNs;
DWORD                       dwTimeSteps;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (fContinuously_p != FALSE)
    {
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        uiIndex = 0;
        if (EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback != NULL)
        {   // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }
    }
    else
    {
        uiIndex = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
    }

    // modify slice timer
    pTimerInfo = &EplTimerHighReskInstance_l.m_TimerInfo;
    EplTimerHighReskCompareInterruptDisable();

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    // => unused in this implementation, as the timer can always be stopped
    pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                        | ((uiIndex + 1) << TIMERHDL_SHIFT);

    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

    // calculate counter
    if (ullTimeNs_p > 0xFFFFFFFF)
    {   // time is too large, so decrease it to the maximum time
        dwTimeNs = 0xFFFFFFFF;
    }
    else
    {
        dwTimeNs = (DWORD) ullTimeNs_p;
    }

    if (dwTimeNs < 10000)
    {   // time is too less, so increase it to the minimum time
        dwTimeNs = 10000;
    }

    dwTimeSteps = OMETH_NS_2_TICKS(dwTimeNs);

    EplTimerHighReskSetCompareValue( EplTimerHighReskGetTimeValue() + dwTimeSteps);

    // enable timer
    EplTimerHighReskCompareInterruptEnable();

Exit:
    return Ret;

}



static tEplKernel EplTimerSynckDrvDeleteTimer(unsigned int uiTimerHdl_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &EplTimerHighReskInstance_l.m_TimerInfo;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_pfnCallback = NULL;

    *pTimerHdl_p = 0;

    EplTimerHighReskCompareInterruptDisable();
    EplTimerHighReskSetCompareValue( 0 );

Exit:
    return Ret;

}



static inline void EplTimerSynckDrvCompareInterruptDisable (void)
{
    IOWR_32DIRECT( OPENMAC_0_TIMERCMP_BASE, TIMERCMP_REG_OFF_CTRL, 0 );
}

static inline void EplTimerSynckDrvCompareInterruptEnable (void)
{
    IOWR_32DIRECT( OPENMAC_0_TIMERCMP_BASE, TIMERCMP_REG_OFF_CTRL, 1 );
}

static inline void EplTimerSynckDrvSetCompareValue (DWORD dwVal)
{
    IOWR_32DIRECT( OPENMAC_0_TIMERCMP_BASE, TIMERCMP_REG_OFF_CMP_VAL, dwVal );
}

static inline DWORD EplTimerSynckDrvGetTimeValue (void)
{
    return IORD_32DIRECT( OPENMAC_0_TIMERCMP_BASE, TIMERCMP_REG_OFF_TIME_VAL );
}



static void EplTimerSynckDrvInterruptHandler (void* pArg_p, alt_u32 dwInt_p)
{

    BENCHMARK_MOD_24_SET(4);

    EplTimerHighReskSetCompareValue(0);
    EplTimerHighReskCompareInterruptDisable();

    if (EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback != NULL)
    {
        EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback(&EplTimerHighReskInstance_l.m_TimerInfo.m_EventArg);
    }

    BENCHMARK_MOD_24_RESET(4);
    return;

}

