/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Part of Ethernet driver to implement time-triggered transmission.
                It is necessary to implement Managing Nodes.

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

  2010/03/18 d.k.:   start of implementation

****************************************************************************/

#include "EplInc.h"
#include "edrv.h"
#include "kernel/EplTimerHighResk.h"


#if EPL_TIMER_USE_HIGHRES == FALSE
#error "EdrvCyclic needs EPL_TIMER_USE_HIGHRES = TRUE"
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

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
#ifndef EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US
#define EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US         50
#endif

#ifndef EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US
#define EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US             150
#endif
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tEdrvTxBuffer**     m_paTxBufferList;
//    unsigned int        m_uiCurTxBufferCount;
    unsigned int        m_uiMaxTxBufferCount;
    unsigned int        m_uiCurTxBufferList;
    unsigned int        m_uiCurTxBufferEntry;
    DWORD               m_dwCycleLenUs;
    tEplTimerHdl        m_TimerHdlCycle;
    tEplTimerHdl        m_TimerHdlSlot;
    tEdrvCyclicCbSync   m_pfnCbSync;
    tEdrvCyclicCbError  m_pfnCbError;

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    unsigned int        m_uiSampleNo;
    unsigned long long  m_ullStartCycleTimeStamp;
    unsigned long long  m_ullLastSlotTimeStamp;

    tEdrvCyclicDiagnostics m_Diag;
#endif

} tEdrvCyclicInstance;



//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


static tEplKernel PUBLIC EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p);

static tEplKernel PUBLIC EdrvCyclicCbTimerSlot(tEplTimerEventArg* pEventArg_p);

static tEplKernel EdrvCyclicProcessTxBufferList(void);



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEdrvCyclicInstance EdrvCyclicInstance_l;




/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <EdrvCyclic>                                        */
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

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------




//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicInit
//
// Description: initialize EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicInit()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvCyclicInstance_l, 0, sizeof (EdrvCyclicInstance_l));

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    EdrvCyclicInstance_l.m_Diag.m_dwCycleTimeMin        = 0xFFFFFFFF;
    EdrvCyclicInstance_l.m_Diag.m_dwUsedCycleTimeMin    = 0xFFFFFFFF;
    EdrvCyclicInstance_l.m_Diag.m_dwSpareCycleTimeMin   = 0xFFFFFFFF;
#endif

//Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicShutdown
//
// Description: Shutdown EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicShutdown(void)
{
    if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
    {
        EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
        EdrvCyclicInstance_l.m_paTxBufferList = NULL;
        EdrvCyclicInstance_l.m_uiMaxTxBufferCount = 0;
    }

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetMaxTxBufferListSize
//
// Description: Sets the maximum number of TxBuffer list entries.
//
// Parameters:  uiMaxListSize_p = maximum number of TxBuffer list entries.
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicSetMaxTxBufferListSize(unsigned int uiMaxListSize_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (EdrvCyclicInstance_l.m_uiMaxTxBufferCount != uiMaxListSize_p)
    {
        EdrvCyclicInstance_l.m_uiMaxTxBufferCount = uiMaxListSize_p;
        if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
        {
            EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
            EdrvCyclicInstance_l.m_paTxBufferList = NULL;
        }

        EdrvCyclicInstance_l.m_paTxBufferList = EPL_MALLOC(sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * uiMaxListSize_p * 2);
        if (EdrvCyclicInstance_l.m_paTxBufferList == NULL)
        {
            Ret = kEplEdrvNoFreeBufEntry;
        }

        EdrvCyclicInstance_l.m_uiCurTxBufferList = 0;

        EPL_MEMSET(EdrvCyclicInstance_l.m_paTxBufferList, 0, sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * uiMaxListSize_p * 2);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetNextTxBufferList
//
// Description: Sets the next TxBuffer list.
//
// Parameters:  apTxBuffer_p
//              uiTxBufferCount_p
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicSetNextTxBufferList(tEdrvTxBuffer** apTxBuffer_p, unsigned int uiTxBufferCount_p)
{
tEplKernel  Ret = kEplSuccessful;
unsigned int    uiNextTxBufferList;

    uiNextTxBufferList = EdrvCyclicInstance_l.m_uiCurTxBufferList ^ EdrvCyclicInstance_l.m_uiMaxTxBufferCount;

    // check if next list is free
    if (EdrvCyclicInstance_l.m_paTxBufferList[uiNextTxBufferList] != NULL)
    {
        Ret = kEplEdrvNextTxListNotEmpty;
        goto Exit;
    }

    if ((uiTxBufferCount_p == 0)
        || (uiTxBufferCount_p > EdrvCyclicInstance_l.m_uiMaxTxBufferCount))
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    // check if last entry in list equals a NULL pointer
    if (apTxBuffer_p[uiTxBufferCount_p - 1] != NULL)
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    EPL_MEMCPY(&EdrvCyclicInstance_l.m_paTxBufferList[uiNextTxBufferList], apTxBuffer_p, sizeof (*apTxBuffer_p) * uiTxBufferCount_p);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetCycleLenUs()
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

tEplKernel EdrvCyclicSetCycleLenUs (DWORD dwCycleLenUs_p)
{
tEplKernel      Ret = kEplSuccessful;

    EdrvCyclicInstance_l.m_dwCycleLenUs = dwCycleLenUs_p;

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicStartCycle()
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

tEplKernel EdrvCyclicStartCycle (void)
{
tEplKernel      Ret = kEplSuccessful;

    if (EdrvCyclicInstance_l.m_dwCycleLenUs == 0)
    {
        Ret = kEplEdrvInvalidCycleLen;
        goto Exit;
    }

    // clear Tx buffer list
    EdrvCyclicInstance_l.m_uiCurTxBufferList = 0;
    EdrvCyclicInstance_l.m_uiCurTxBufferEntry = 0;
    EPL_MEMSET(EdrvCyclicInstance_l.m_paTxBufferList, 0,
        sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * EdrvCyclicInstance_l.m_uiMaxTxBufferCount * 2);

    Ret = EplTimerHighReskModifyTimerNs(&EdrvCyclicInstance_l.m_TimerHdlCycle,
        EdrvCyclicInstance_l.m_dwCycleLenUs * 1000ULL,
        EdrvCyclicCbTimerCycle,
        0L,
        TRUE);

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    EdrvCyclicInstance_l.m_ullLastSlotTimeStamp = 0;
#endif

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicStopCycle()
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

tEplKernel EdrvCyclicStopCycle (void)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = EplTimerHighReskDeleteTimer(&EdrvCyclicInstance_l.m_TimerHdlCycle);
    Ret = EplTimerHighReskDeleteTimer(&EdrvCyclicInstance_l.m_TimerHdlSlot);

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    EdrvCyclicInstance_l.m_ullStartCycleTimeStamp = 0;
#endif

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicRegSyncHandler()
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

tEplKernel EdrvCyclicRegSyncHandler (tEdrvCyclicCbSync pfnCbSync_p)
{
tEplKernel      Ret = kEplSuccessful;


    EdrvCyclicInstance_l.m_pfnCbSync = pfnCbSync_p;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicRegErrorHandler()
//
// Description: registers handler for error events
//
// Parameters:  pfnTimerSynckCbSync_p   = pointer to callback function,
//                                        which will be called in interrupt context.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicRegErrorHandler (tEdrvCyclicCbError pfnCbError_p)
{
tEplKernel      Ret = kEplSuccessful;


    EdrvCyclicInstance_l.m_pfnCbError = pfnCbError_p;

    return Ret;

}


#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicGetDiagnostics()
//
// Description: Returns diagnostic information
//
// Parameters:  ppDiagnostics_p     = OUT Pointer to pointer to diagnostic info
//
// Returns:     tEplKernel          = Error code
//
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicGetDiagnostics(tEdrvCyclicDiagnostics** ppDiagnostics_p)
{
    *ppDiagnostics_p = &EdrvCyclicInstance_l.m_Diag;

    return kEplSuccessful;
}
#endif



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicCbTimerCycle()
//
// Description: called by timer module. It starts the next cycle.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
DWORD           dwCycleTime;
DWORD           dwUsedCycleTime;
DWORD           dwSpareCycleTime;
unsigned long long ullStartNewCycleTimeStamp;
#endif

    if (pEventArg_p->m_TimerHdl != EdrvCyclicInstance_l.m_TimerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    ullStartNewCycleTimeStamp = EplTgtGetTimeStampNs();
#endif

    if (EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry] != NULL)
    {
        Ret = kEplEdrvTxListNotFinishedYet;
        goto Exit;
    }

    EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferList] = NULL;

    // enter new cycle -> switch Tx buffer list
    EdrvCyclicInstance_l.m_uiCurTxBufferList ^= EdrvCyclicInstance_l.m_uiMaxTxBufferCount;
    EdrvCyclicInstance_l.m_uiCurTxBufferEntry = EdrvCyclicInstance_l.m_uiCurTxBufferList;

    if (EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry] == NULL)
    {
        Ret = kEplEdrvCurTxListEmpty;
        goto Exit;
    }

    Ret = EdrvCyclicProcessTxBufferList();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (EdrvCyclicInstance_l.m_pfnCbSync != NULL)
    {
        Ret = EdrvCyclicInstance_l.m_pfnCbSync();
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    if (EdrvCyclicInstance_l.m_ullStartCycleTimeStamp != 0)
    {
        // calculate time diffs of previous cycle
        dwCycleTime      = (DWORD) (ullStartNewCycleTimeStamp - EdrvCyclicInstance_l.m_ullStartCycleTimeStamp);
        if (EdrvCyclicInstance_l.m_Diag.m_dwCycleTimeMin > dwCycleTime)
        {
            EdrvCyclicInstance_l.m_Diag.m_dwCycleTimeMin = dwCycleTime;
        }
        if (EdrvCyclicInstance_l.m_Diag.m_dwCycleTimeMax < dwCycleTime)
        {
            EdrvCyclicInstance_l.m_Diag.m_dwCycleTimeMax = dwCycleTime;
        }

        if (EdrvCyclicInstance_l.m_ullLastSlotTimeStamp != 0)
        {
            dwUsedCycleTime  = (DWORD) (EdrvCyclicInstance_l.m_ullLastSlotTimeStamp - EdrvCyclicInstance_l.m_ullStartCycleTimeStamp);
            dwSpareCycleTime = (DWORD) (ullStartNewCycleTimeStamp - EdrvCyclicInstance_l.m_ullLastSlotTimeStamp);

            if (EdrvCyclicInstance_l.m_Diag.m_dwUsedCycleTimeMin > dwUsedCycleTime)
            {
                EdrvCyclicInstance_l.m_Diag.m_dwUsedCycleTimeMin = dwUsedCycleTime;
            }
            if (EdrvCyclicInstance_l.m_Diag.m_dwUsedCycleTimeMax < dwUsedCycleTime)
            {
                EdrvCyclicInstance_l.m_Diag.m_dwUsedCycleTimeMax = dwUsedCycleTime;
            }
            if (EdrvCyclicInstance_l.m_Diag.m_dwSpareCycleTimeMin > dwSpareCycleTime)
            {
                EdrvCyclicInstance_l.m_Diag.m_dwSpareCycleTimeMin = dwSpareCycleTime;
            }
            if (EdrvCyclicInstance_l.m_Diag.m_dwSpareCycleTimeMax < dwSpareCycleTime)
            {
                EdrvCyclicInstance_l.m_Diag.m_dwSpareCycleTimeMax = dwSpareCycleTime;
            }
        }
        else
        {
            dwUsedCycleTime = 0;
            dwSpareCycleTime = dwCycleTime;
        }

        EdrvCyclicInstance_l.m_Diag.m_ullCycleTimeMeanSum      += dwCycleTime;
        EdrvCyclicInstance_l.m_Diag.m_ullUsedCycleTimeMeanSum  += dwUsedCycleTime;
        EdrvCyclicInstance_l.m_Diag.m_ullSpareCycleTimeMeanSum += dwSpareCycleTime;
        EdrvCyclicInstance_l.m_Diag.m_ullCycleCount++;

        // sample previous cycle if deviations exceed threshold
        if (    (EdrvCyclicInstance_l.m_Diag.m_uiSampleNum == 0) /* sample first cycle for start time */
                || (abs(dwCycleTime - EdrvCyclicInstance_l.m_dwCycleLenUs * 1000) > EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US * 1000)
                || (dwSpareCycleTime < EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US * 1000))
        {
        unsigned int uiSampleNo = EdrvCyclicInstance_l.m_uiSampleNo;

            EdrvCyclicInstance_l.m_Diag.m_aullSampleTimeStamp[uiSampleNo] = EdrvCyclicInstance_l.m_ullStartCycleTimeStamp;
            EdrvCyclicInstance_l.m_Diag.m_adwCycleTime[uiSampleNo]       = dwCycleTime;
            EdrvCyclicInstance_l.m_Diag.m_adwUsedCycleTime[uiSampleNo]   = dwUsedCycleTime;
            EdrvCyclicInstance_l.m_Diag.m_adwSpareCycleTime[uiSampleNo]  = dwSpareCycleTime;

            EdrvCyclicInstance_l.m_Diag.m_uiSampleNum++;
            if (EdrvCyclicInstance_l.m_Diag.m_uiSampleBufferedNum != EDRV_CYCLIC_SAMPLE_NUM)
            {
                EdrvCyclicInstance_l.m_Diag.m_uiSampleBufferedNum++;
            }

            EdrvCyclicInstance_l.m_uiSampleNo++;
            if (EdrvCyclicInstance_l.m_uiSampleNo == EDRV_CYCLIC_SAMPLE_NUM)
            {
                EdrvCyclicInstance_l.m_uiSampleNo = 1;
            }
        }
    }

    EdrvCyclicInstance_l.m_ullStartCycleTimeStamp = ullStartNewCycleTimeStamp;
    EdrvCyclicInstance_l.m_ullLastSlotTimeStamp = 0;
#endif

Exit:
    if (Ret != kEplSuccessful)
    {
        if (EdrvCyclicInstance_l.m_pfnCbError != NULL)
        {
            Ret = EdrvCyclicInstance_l.m_pfnCbError(Ret, NULL);
        }
    }
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicCbTimerSlot()
//
// Description: called by timer module. It triggers the transmission of the next frame.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EdrvCyclicCbTimerSlot(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer*  pTxBuffer = NULL;

    if (pEventArg_p->m_TimerHdl != EdrvCyclicInstance_l.m_TimerHdlSlot)
    {   // zombie callback
        // just exit
        goto Exit;
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    EdrvCyclicInstance_l.m_ullLastSlotTimeStamp = EplTgtGetTimeStampNs();
#endif

    pTxBuffer = EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry];
    Ret = EdrvSendTxMsg(pTxBuffer);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    EdrvCyclicInstance_l.m_uiCurTxBufferEntry++;

    Ret = EdrvCyclicProcessTxBufferList();

Exit:
    if (Ret != kEplSuccessful)
    {
        if (EdrvCyclicInstance_l.m_pfnCbError != NULL)
        {
            Ret = EdrvCyclicInstance_l.m_pfnCbError(Ret, pTxBuffer);
        }
    }
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicProcessTxBufferList()
//
// Description: processes the Tx buffer list.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EdrvCyclicProcessTxBufferList(void)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer*  pTxBuffer;

    while ((pTxBuffer = EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry]) != NULL)
    {
        if (pTxBuffer->m_dwTimeOffsetNs == 0)
        {
            Ret = EdrvSendTxMsg(pTxBuffer);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {
            Ret = EplTimerHighReskModifyTimerNs(&EdrvCyclicInstance_l.m_TimerHdlSlot,
                pTxBuffer->m_dwTimeOffsetNs,
                EdrvCyclicCbTimerSlot,
                0L,
                FALSE);

            break;
        }

        EdrvCyclicInstance_l.m_uiCurTxBufferEntry++;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
        if (EdrvCyclicInstance_l.m_pfnCbError != NULL)
        {
            Ret = EdrvCyclicInstance_l.m_pfnCbError(Ret, pTxBuffer);
        }
    }
    return Ret;
}

