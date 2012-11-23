/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for kernel DLL Communication Abstraction Layer module

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

  2006/06/15 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "kernel/EplDllkCal.h"
#include "kernel/EplDllk.h"
#include "kernel/EplEventk.h"
#include "EplDllCal.h"

#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT || \
    EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
#include "EplDllCalDirect.h"
#endif
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB || \
    EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB || \
    (EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB && \
            EPL_DLL_PRES_CHAINING_MN != FALSE)
#include "EplDllCalShb.h"
#endif
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE || \
    EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE || \
    (EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE && \
            EPL_DLL_PRES_CHAINING_MN != FALSE)
#error "Host interface not yet supported!"
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
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

#if (EPL_DLL_PRES_CHAINING_MN != FALSE) \
    && (EPL_DLLCAL_TX_SYNC_QUEUE != EPL_QUEUE_SHB) \
    && (EPL_DLLCAL_TX_SYNC_QUEUE != EPL_QUEUE_HOSTINTERFACE)
#error "DLLCal module does not support direct calls with PRC MN"
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplDllkCal                                          */
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

#if EPL_DLL_PRES_CHAINING_MN != FALSE
#define EPL_DLLKCAL_MAX_QUEUES  6   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq, SyncReq
#else
#define EPL_DLLKCAL_MAX_QUEUES  5   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq
#endif

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
      tEplDllCalQueueInstance     DllCalQueueTxNmt_m;
          ///< Dll Cal Queue instance for NMT priority
      tEplDllCalQueueInstance     DllCalQueueTxGen_m;
          ///< Dll Cal Queue instance for Generic priority
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0) \
    && (EPL_DLL_PRES_CHAINING_MN != FALSE)
    tEplDllCalQueueInstance     DllCalQueueTxSync_m;
        ///< Dll Cal Queue instance for Sync Request
#endif

    tEplDllkCalStatistics   m_Statistics;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // IdentRequest queue with CN node IDs
    unsigned int    m_auiQueueIdentReq[EPL_D_NMT_MaxCNNumber_U8 + 1];   // 1 entry is reserved to distinguish between full and empty
    volatile unsigned int    m_uiWriteIdentReq;
    volatile unsigned int    m_uiReadIdentReq;

    // StatusRequest queue with CN node IDs
    unsigned int    m_auiQueueStatusReq[EPL_D_NMT_MaxCNNumber_U8 + 1];  // 1 entry is reserved to distinguish between full and empty
    volatile unsigned int    m_uiWriteStatusReq;
    volatile unsigned int    m_uiReadStatusReq;

    unsigned int    m_auiQueueCnRequests[254 * 2];
        // first 254 entries represent the generic requests of the corresponding node
        // second 254 entries represent the NMT requests of the corresponding node
    unsigned int    m_uiNextQueueCnRequest;
    unsigned int    m_uiNextRequestQueue;

#endif

} tEplDllkCalInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

// if no dynamic memory allocation shall be used
// define structures statically
static tEplDllkCalInstance     EplDllkCalInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAddInstance()
//
// Description: add and initialize new instance of DLL CAL module
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAddInstance(void)
{
tEplKernel      Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplDllkCalInstance_g, 0, sizeof (EplDllkCalInstance_g));

#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectAddInstance(&EplDllkCalInstance_g.DllCalQueueTxNmt_m,
            kEplDllCalQueueTxNmt);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbAddInstance(&EplDllkCalInstance_g.DllCalQueueTxNmt_m,
            kEplDllCalQueueTxNmt);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectAddInstance(&EplDllkCalInstance_g.DllCalQueueTxGen_m,
            kEplDllCalQueueTxGen);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbAddInstance(&EplDllkCalInstance_g.DllCalQueueTxGen_m,
            kEplDllCalQueueTxGen);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
#if EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbAddInstance(&EplDllkCalInstance_g.DllCalQueueTxSync_m,
            kEplDllCalQueueTxSync);
#elif EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalDelInstance()
//
// Description: deletes instance of DLL CAL module
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalDelInstance(void)
{
tEplKernel      Ret = kEplSuccessful;

#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectDelInstance(EplDllkCalInstance_g.DllCalQueueTxNmt_m);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbDelInstance(EplDllkCalInstance_g.DllCalQueueTxNmt_m);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectDelInstance(EplDllkCalInstance_g.DllCalQueueTxGen_m);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbDelInstance(EplDllkCalInstance_g.DllCalQueueTxGen_m);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
#if EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbDelInstance(EplDllkCalInstance_g.DllCalQueueTxSync_m);
#elif EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // reset instance structure
    EPL_MEMSET(&EplDllkCalInstance_g, 0, sizeof (EplDllkCalInstance_g));

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalProcess
//
// Description: process the passed configuration
//
// Parameters:  pEvent_p                = event containing configuration options
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalProcess(tEplEvent * pEvent_p)
{
tEplKernel      Ret = kEplSuccessful;

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypeDllkServFilter:
        {
        tEplDllCalAsndServiceIdFilter*  pServFilter;

            pServFilter = (tEplDllCalAsndServiceIdFilter*) pEvent_p->m_pArg;
            Ret = EplDllkSetAsndServiceIdFilter(pServFilter->m_ServiceId, pServFilter->m_Filter);
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplEventTypeDllkIssueReq:
        {
        tEplDllCalIssueRequest*  pIssueReq;

            pIssueReq = (tEplDllCalIssueRequest*) pEvent_p->m_pArg;
            Ret = EplDllkCalIssueRequest(pIssueReq->m_Service, pIssueReq->m_uiNodeId, pIssueReq->m_bSoaFlag1);
            break;
        }
#endif

#if EPL_NMT_MAX_NODE_ID > 0
        case kEplEventTypeDllkConfigNode:
        {
        tEplDllNodeInfo*    pNodeInfo;

            pNodeInfo = (tEplDllNodeInfo*) pEvent_p->m_pArg;
            Ret = EplDllkConfigNode(pNodeInfo);
            break;
        }

        case kEplEventTypeDllkAddNode:
        {
        tEplDllNodeOpParam*    pNodeOpParam;

            pNodeOpParam = (tEplDllNodeOpParam*) pEvent_p->m_pArg;
            Ret = EplDllkAddNode(pNodeOpParam);
            break;
        }

        case kEplEventTypeDllkDelNode:
        {
        tEplDllNodeOpParam*    pNodeOpParam;

            pNodeOpParam = (tEplDllNodeOpParam*) pEvent_p->m_pArg;
            Ret = EplDllkDeleteNode(pNodeOpParam);
            break;
        }
#endif // EPL_NMT_MAX_NODE_ID > 0

        case kEplEventTypeDllkIdentity:
        {
        tEplDllIdentParam*  pIdentParam;

            pIdentParam = (tEplDllIdentParam*) pEvent_p->m_pArg;
            if (pIdentParam->m_uiSizeOfStruct > pEvent_p->m_uiSize)
            {
                pIdentParam->m_uiSizeOfStruct = pEvent_p->m_uiSize;
            }
            Ret = EplDllkSetIdentity(pIdentParam);
            break;
        }

        case kEplEventTypeDllkConfig:
        {
        tEplDllConfigParam* pConfigParam;

            pConfigParam = (tEplDllConfigParam*) pEvent_p->m_pArg;
            if (pConfigParam->m_uiSizeOfStruct > pEvent_p->m_uiSize)
            {
                pConfigParam->m_uiSizeOfStruct = pEvent_p->m_uiSize;
            }
            Ret = EplDllkConfig(pConfigParam);
            break;
        }

        default:
        {
            Ret = kEplInvalidEvent;
            break;
        }
    }

//Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncGetTxCount()
//
// Description: returns count of Tx frames of FIFO with highest priority
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncGetTxCount(tEplDllAsyncReqPriority * pPriority_p, unsigned int * puiCount_p)
{
tEplKernel  Ret = kEplSuccessful;
unsigned long ulFrameCount;

#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectGetDataBlockCount(
            EplDllkCalInstance_g.DllCalQueueTxNmt_m, &ulFrameCount);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxNmt_m,
            &ulFrameCount);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (ulFrameCount > EplDllkCalInstance_g.m_Statistics.m_ulMaxTxFrameCountNmt)
    {
        EplDllkCalInstance_g.m_Statistics.m_ulMaxTxFrameCountNmt = ulFrameCount;
    }

    if (ulFrameCount != 0)
    {   // NMT requests are in queue
        *pPriority_p = kEplDllAsyncReqPrioNmt;
        *puiCount_p = (unsigned int) ulFrameCount;
        goto Exit;
    }

#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectGetDataBlockCount(
            EplDllkCalInstance_g.DllCalQueueTxGen_m, &ulFrameCount);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxGen_m,
            &ulFrameCount);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (ulFrameCount > EplDllkCalInstance_g.m_Statistics.m_ulMaxTxFrameCountGen)
    {
        EplDllkCalInstance_g.m_Statistics.m_ulMaxTxFrameCountGen = ulFrameCount;
    }

    *pPriority_p = kEplDllAsyncReqPrioGeneric;
    *puiCount_p = (unsigned int) ulFrameCount;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncGetTxFrame()
//
// Description: returns Tx frames from FIFO with specified priority
//
// Parameters:  pFrame_p                = IN: pointer to buffer
//              puiFrameSize_p          = IN: max size of buffer
//                                        OUT: actual size of frame
//              Priority_p              = IN: priority
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncGetTxFrame(void * pFrame_p, unsigned int * puiFrameSize_p, tEplDllAsyncReqPriority Priority_p)
{
tEplKernel      Ret = kEplSuccessful;


    switch (Priority_p)
    {
        case kEplDllAsyncReqPrioNmt:    // NMT request priority
#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplDllCalDirectGetDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxNmt_m,
                    (BYTE*) pFrame_p, puiFrameSize_p);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
            Ret = EplDllCalShbGetDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxNmt_m,
                    (BYTE*) pFrame_p, puiFrameSize_p);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
            break;
        default:    // generic priority
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplDllCalDirectGetDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxGen_m,
                    (BYTE*) pFrame_p, puiFrameSize_p);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
            Ret = EplDllCalShbGetDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxGen_m,
                    (BYTE*) pFrame_p, puiFrameSize_p);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
            break;
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncFrameReceived()
//
// Description: passes ASnd frame to receive FIFO.
//              It will be called only for frames with registered AsndServiceIds.
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncFrameReceived(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel  Ret = kEplSuccessful;
tEplEvent   Event;

    Event.m_EventSink = kEplEventSinkDlluCal;
    Event.m_EventType = kEplEventTypeAsndRx;
    Event.m_pArg = pFrameInfo_p->m_pFrame;
    Event.m_uiSize = pFrameInfo_p->m_uiFrameSize;

    Ret = EplEventkPost(&Event);
    if (Ret != kEplSuccessful)
    {
        EplDllkCalInstance_g.m_Statistics.m_ulCurRxFrameCount++;
    }
    else
    {
        EplDllkCalInstance_g.m_Statistics.m_ulMaxRxFrameCount++;
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncSend()
//
// Description: puts the given frame into the transmit FIFO with the specified
//              priority.
//
// Parameters:  pFrameInfo_p            = frame info structure
//              Priority_p              = priority
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncSend(tEplFrameInfo * pFrameInfo_p, tEplDllAsyncReqPriority Priority_p)
{
tEplKernel Ret = kEplSuccessful;
tEplEvent Event;

    switch (Priority_p)
    {
        case kEplDllAsyncReqPrioNmt:    // NMT request priority
#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplDllCalDirectInsertDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxNmt_m,
                    (BYTE*)pFrameInfo_p->m_pFrame,
                    &(pFrameInfo_p->m_uiFrameSize));
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
            Ret = EplDllCalShbInsertDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxNmt_m,
                    (BYTE*)pFrameInfo_p->m_pFrame,
                    &(pFrameInfo_p->m_uiFrameSize));
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
            break;

        default:    // generic priority
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
            Ret = EplDllCalDirectInsertDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxGen_m,
                    (BYTE*)pFrameInfo_p->m_pFrame,
                    &(pFrameInfo_p->m_uiFrameSize));
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
            Ret = EplDllCalShbInsertDataBlock(
                    EplDllkCalInstance_g.DllCalQueueTxGen_m,
                    (BYTE*)pFrameInfo_p->m_pFrame,
                    &(pFrameInfo_p->m_uiFrameSize));
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif
            break;
    }

    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // post event to DLL
    Event.m_EventSink = kEplEventSinkDllk;
    Event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
    Event.m_pArg = &Priority_p;
    Event.m_uiSize = sizeof(Priority_p);
    Ret = EplEventkPost(&Event);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncClearBuffer()
//
// Description: clears the transmit buffer
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncClearBuffer(void)
{
tEplKernel  Ret = kEplSuccessful;

    //Ret is ignored
#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectResetDataBlockQueue(
            EplDllkCalInstance_g.DllCalQueueTxNmt_m);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbResetDataBlockQueue(
            EplDllkCalInstance_g.DllCalQueueTxNmt_m, 1000);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    //Ret is ignored
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectResetDataBlockQueue(
            EplDllkCalInstance_g.DllCalQueueTxGen_m);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbResetDataBlockQueue(
            EplDllkCalInstance_g.DllCalQueueTxGen_m, 1000);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncClearQueues()
//
// Description: clears the transmit buffer
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
tEplKernel EplDllkCalAsyncClearQueues(void)
{
tEplKernel  Ret = kEplSuccessful;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    //Ret is ignored
#if EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbResetDataBlockQueue(
            EplDllkCalInstance_g.DllCalQueueTxSync_m, 1000);
#elif EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

#endif

    // clear MN asynchronous queues
    EplDllkCalInstance_g.m_uiNextQueueCnRequest = 0;
    EplDllkCalInstance_g.m_uiNextRequestQueue = 0;
    EplDllkCalInstance_g.m_uiReadIdentReq = 0;
    EplDllkCalInstance_g.m_uiWriteIdentReq = 0;
    EplDllkCalInstance_g.m_uiReadStatusReq = 0;
    EplDllkCalInstance_g.m_uiWriteStatusReq = 0;

    return Ret;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalGetStatistics()
//
// Description: returns statistics of the asynchronous queues.
//
// Parameters:  ppStatistics            = statistics structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalGetStatistics(tEplDllkCalStatistics ** ppStatistics)
{
tEplKernel  Ret = kEplSuccessful;

    //Ret is ignored
#if EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxNmt_m,
            &EplDllkCalInstance_g.m_Statistics.m_ulCurTxFrameCountNmt);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxNmt_m,
            &EplDllkCalInstance_g.m_Statistics.m_ulCurTxFrameCountNmt);
#elif EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    //Ret is ignored
#if EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_DIRECT
    Ret = EplDllCalDirectGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxGen_m,
            &EplDllkCalInstance_g.m_Statistics.m_ulCurTxFrameCountGen);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_SHB
    Ret = EplDllCalShbGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxGen_m,
            &EplDllkCalInstance_g.m_Statistics.m_ulCurTxFrameCountGen);
#elif EPL_DLLCAL_TX_GEN_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

    *ppStatistics = &EplDllkCalInstance_g.m_Statistics;
    return Ret;
}


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalIssueRequest()
//
// Description: issues a StatusRequest or a IdentRequest to the specified node.
//
// Parameters:  Service_p               = request service ID
//              uiNodeId_p              = node ID
//              bSoaFlag1_p             = flag1 for this node (transmit in SoA and PReq)
//                                        If 0xFF this flag is ignored.
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalIssueRequest(tEplDllReqServiceId Service_p, unsigned int uiNodeId_p, BYTE bSoaFlag1_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (bSoaFlag1_p != 0xFF)
    {
        Ret = EplDllkSetFlag1OfNode(uiNodeId_p, bSoaFlag1_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    // add node to appropriate request queue
    switch (Service_p)
    {
        case kEplDllReqServiceIdent:
        {
            if (((EplDllkCalInstance_g.m_uiWriteIdentReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueIdentReq))
                == EplDllkCalInstance_g.m_uiReadIdentReq)
            {   // queue is full
                Ret = kEplDllAsyncTxBufferFull;
                goto Exit;
            }
            EplDllkCalInstance_g.m_auiQueueIdentReq[EplDllkCalInstance_g.m_uiWriteIdentReq] = uiNodeId_p;
            EplDllkCalInstance_g.m_uiWriteIdentReq =
                (EplDllkCalInstance_g.m_uiWriteIdentReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueIdentReq);
            break;
        }

        case kEplDllReqServiceStatus:
        {
            if (((EplDllkCalInstance_g.m_uiWriteStatusReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueStatusReq))
                == EplDllkCalInstance_g.m_uiReadStatusReq)
            {   // queue is full
                Ret = kEplDllAsyncTxBufferFull;
                goto Exit;
            }
            EplDllkCalInstance_g.m_auiQueueStatusReq[EplDllkCalInstance_g.m_uiWriteStatusReq] = uiNodeId_p;
            EplDllkCalInstance_g.m_uiWriteStatusReq =
                (EplDllkCalInstance_g.m_uiWriteStatusReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueStatusReq);
            break;
        }

        default:
        {
            Ret = kEplDllInvalidParam;
            goto Exit;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncGetSoaRequest()
//
// Description: returns next request for SoA. This function is called by DLLk module.
//
// Parameters:  pReqServiceId_p         = pointer to request service ID
//                                        IN: available request for MN NMT or generic request queue (Flag2.PR)
//                                            or kEplDllReqServiceNo if queues are empty
//                                        OUT: next request
//              puiNodeId_p             = OUT: pointer to node ID of next request
//                                             = EPL_C_ADR_INVALID, if request is self addressed
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncGetSoaRequest(tEplDllReqServiceId* pReqServiceId_p, unsigned int* puiNodeId_p, tEplSoaPayload* pSoaPayload_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiCount;

#if EPL_DLL_PRES_CHAINING_MN == FALSE
    UNUSED_PARAMETER(pSoaPayload_p);
#endif

    for (uiCount = EPL_DLLKCAL_MAX_QUEUES; uiCount > 0; uiCount--)
    {
        switch (EplDllkCalInstance_g.m_uiNextRequestQueue)
        {
            case 0:
            {   // CnGenReq
                for (;EplDllkCalInstance_g.m_uiNextQueueCnRequest < (tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2);
                    EplDllkCalInstance_g.m_uiNextQueueCnRequest++)
                {
                    if (EplDllkCalInstance_g.m_auiQueueCnRequests[EplDllkCalInstance_g.m_uiNextQueueCnRequest] > 0)
                    {   // non empty queue found
                        // remove one request from queue
                        EplDllkCalInstance_g.m_auiQueueCnRequests[EplDllkCalInstance_g.m_uiNextQueueCnRequest]--;
                        *puiNodeId_p = EplDllkCalInstance_g.m_uiNextQueueCnRequest + 1;
                        *pReqServiceId_p = kEplDllReqServiceUnspecified;
                        EplDllkCalInstance_g.m_uiNextQueueCnRequest++;
                        if (EplDllkCalInstance_g.m_uiNextQueueCnRequest >= (tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2))
                        {   // last node reached
                            // continue with CnNmtReq queue at next SoA
                            EplDllkCalInstance_g.m_uiNextRequestQueue = 1;
                        }
                        goto Exit;
                    }
                }
                // all CnGenReq queues are empty -> continue with CnNmtReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 1;
                break;
            }

            case 1:
            {   // CnNmtReq
                for (;EplDllkCalInstance_g.m_uiNextQueueCnRequest < tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests);
                    EplDllkCalInstance_g.m_uiNextQueueCnRequest++)
                {
                    if (EplDllkCalInstance_g.m_auiQueueCnRequests[EplDllkCalInstance_g.m_uiNextQueueCnRequest] > 0)
                    {   // non empty queue found
                        // remove one request from queue
                        EplDllkCalInstance_g.m_auiQueueCnRequests[EplDllkCalInstance_g.m_uiNextQueueCnRequest]--;
                        *puiNodeId_p = EplDllkCalInstance_g.m_uiNextQueueCnRequest + 1 - (tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2);
                        *pReqServiceId_p = kEplDllReqServiceNmtRequest;
                        EplDllkCalInstance_g.m_uiNextQueueCnRequest++;
                        if (EplDllkCalInstance_g.m_uiNextQueueCnRequest > tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests))
                        {   // last node reached
                            // restart CnGenReq queue
                            EplDllkCalInstance_g.m_uiNextQueueCnRequest = 0;
                            // continue with MnGenReq queue at next SoA
                            EplDllkCalInstance_g.m_uiNextRequestQueue = 2;
                        }
                        goto Exit;
                    }
                }
                // restart CnGenReq queue
                EplDllkCalInstance_g.m_uiNextQueueCnRequest = 0;
                // all CnNmtReq queues are empty -> continue with MnGenReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 2;
                break;
            }

            case 2:
            {   // MnNmtReq and MnGenReq
                // next queue will be MnIdentReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 3;
                if (*pReqServiceId_p != kEplDllReqServiceNo)
                {
                    *puiNodeId_p = EPL_C_ADR_INVALID;   // DLLk must exchange this with the actual node ID
                    goto Exit;
                }
                break;
            }

            case 3:
            {   // MnIdentReq
                // next queue will be MnStatusReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 4;
                if (EplDllkCalInstance_g.m_uiReadIdentReq != EplDllkCalInstance_g.m_uiWriteIdentReq)
                {   // queue is not empty
                    *puiNodeId_p = EplDllkCalInstance_g.m_auiQueueIdentReq[EplDllkCalInstance_g.m_uiReadIdentReq];
                    EplDllkCalInstance_g.m_uiReadIdentReq =
                        (EplDllkCalInstance_g.m_uiReadIdentReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueIdentReq);
                    *pReqServiceId_p = kEplDllReqServiceIdent;
                    goto Exit;
                }
                break;
            }

            case 4:
            {   // MnStatusReq
#if EPL_DLL_PRES_CHAINING_MN != FALSE
                // next queue will be MnSyncReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 5;
#else
                // next queue will be CnGenReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 0;
#endif
                if (EplDllkCalInstance_g.m_uiReadStatusReq != EplDllkCalInstance_g.m_uiWriteStatusReq)
                {   // queue is not empty
                    *puiNodeId_p = EplDllkCalInstance_g.m_auiQueueStatusReq[EplDllkCalInstance_g.m_uiReadStatusReq];
                    EplDllkCalInstance_g.m_uiReadStatusReq =
                        (EplDllkCalInstance_g.m_uiReadStatusReq + 1) % tabentries (EplDllkCalInstance_g.m_auiQueueStatusReq);
                    *pReqServiceId_p = kEplDllReqServiceStatus;
                    goto Exit;
                }
                break;
            }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
            case 5:
            {   // MnSyncReq
            unsigned long       ulSyncReqCount = 0;
            unsigned int        uiSyncReqSize = 0;
            tEplDllSyncRequest  SyncRequest;

                // next queue will be CnGenReq queue
                EplDllkCalInstance_g.m_uiNextRequestQueue = 0;
#if EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB
                Ret = EplDllCalShbGetDataBlockCount(EplDllkCalInstance_g.DllCalQueueTxSync_m, &ulSyncReqCount);
#elif EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

                if(Ret != kEplSuccessful)
                {
                    goto Exit;
                }
                if (ulSyncReqCount > 0)
                {
                    uiSyncReqSize = sizeof(SyncRequest);
#if EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_SHB
                    Ret = EplDllCalShbGetDataBlock(EplDllkCalInstance_g.DllCalQueueTxSync_m, (BYTE *)&SyncRequest, &uiSyncReqSize);
#elif EPL_DLLCAL_TX_SYNC_QUEUE == EPL_QUEUE_HOSTINTERFACE

#endif

                    if(Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    if (uiSyncReqSize > memberoffs(tEplDllSyncRequest, m_dwSyncControl))
                    {
                        AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwSyncControl, SyncRequest.m_dwSyncControl);
                        if ((SyncRequest.m_dwSyncControl & EPL_SYNC_PRES_MODE_SET) != 0)
                        {
                        tEplDllNodeOpParam  NodeOpParam;

                            NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                            NodeOpParam.m_uiNodeId = SyncRequest.m_uiNodeId;

                            Ret = EplDllkAddNode(&NodeOpParam);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
                        if ((SyncRequest.m_dwSyncControl & EPL_SYNC_PRES_MODE_RESET) != 0)
                        {
                        tEplDllNodeOpParam  NodeOpParam;

                            NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                            NodeOpParam.m_uiNodeId = SyncRequest.m_uiNodeId;

                            Ret = EplDllkDeleteNode(&NodeOpParam);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
                    }
                    if (uiSyncReqSize > memberoffs(tEplDllSyncRequest, m_dwPResTimeFirst))
                    {
                        AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwPResTimeFirst, SyncRequest.m_dwPResTimeFirst);
                    }
                    if (uiSyncReqSize > memberoffs(tEplDllSyncRequest, m_dwPResFallBackTimeout))
                    {
                        AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwPResFallBackTimeout, SyncRequest.m_dwPResFallBackTimeout);
                    }

                    if ((SyncRequest.m_dwSyncControl & EPL_SYNC_DEST_MAC_ADDRESS_VALID) != 0)
                    {
                        Ret = EplDllkGetCnMacAddress(SyncRequest.m_uiNodeId, &pSoaPayload_p->m_SyncRequest.m_be_abDestMacAddress[0]);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }

                    *puiNodeId_p = SyncRequest.m_uiNodeId;
                    *pReqServiceId_p = kEplDllReqServiceSync;
                    goto Exit;
                }
                break;
            }
#endif
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCalAsyncSetPendingRequests()
//
// Description: sets the pending asynchronous frame requests of the specified node.
//              This will add the node to the asynchronous request scheduler.
//
// Parameters:  uiNodeId_p              = node ID
//              AsyncReqPrio_p          = asynchronous request priority
//              uiCount_p               = count of asynchronous frames
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkCalAsyncSetPendingRequests(unsigned int uiNodeId_p, tEplDllAsyncReqPriority AsyncReqPrio_p, unsigned int uiCount_p)
{
tEplKernel  Ret = kEplSuccessful;

    // add node to appropriate request queue
    switch (AsyncReqPrio_p)
    {
        case kEplDllAsyncReqPrioNmt:
        {
            uiNodeId_p--;
            if (uiNodeId_p >= (tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2))
            {
                Ret = kEplDllInvalidParam;
                goto Exit;
            }
            uiNodeId_p += tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2;
            EplDllkCalInstance_g.m_auiQueueCnRequests[uiNodeId_p] = uiCount_p;
            break;
        }

        default:
        {
            uiNodeId_p--;
            if (uiNodeId_p >= (tabentries (EplDllkCalInstance_g.m_auiQueueCnRequests) / 2))
            {
                Ret = kEplDllInvalidParam;
                goto Exit;
            }
            EplDllkCalInstance_g.m_auiQueueCnRequests[uiNodeId_p] = uiCount_p;
            break;
        }
    }

Exit:
    return Ret;
}
#endif //(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

// EOF

