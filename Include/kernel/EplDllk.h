/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for kernelspace DLL module

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

  2006/06/08 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_DLLK_H_
#define _EPL_DLLK_H_

#include "EplDll.h"
#include "EplEvent.h"

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

typedef tEplKernel (* tEplDllkCbAsync) (tEplFrameInfo * pFrameInfo_p);

typedef struct
{
    BYTE                m_be_abLocalMac[6];
    tEplHwParam         m_HwParam;

} tEplDllkInitParam;

// forward declaration
struct _tEdrvTxBuffer;

struct _tEplDllkNodeInfo
{
    unsigned int                m_uiNodeId;
    WORD                        m_wPresPayloadLimit;    // object 0x1F8D: NMT_PResPayloadLimitList_AU16
    BYTE                        m_bPresFilterFlags;
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    BYTE                        m_be_abMacAddr[6];
    BYTE                        m_bSoaFlag1;
    BOOL                        m_fSoftDelete;          // delete node after error and ignore error
    WORD                        m_wPreqPayloadLimit;    // object 0x1F8B: NMT_MNPReqPayloadLimitList_AU16
    tEplNmtState                m_NmtState;
    unsigned long               m_ulDllErrorEvents;
    DWORD                       m_dwPresTimeoutNs;        // object 0x1F92: NMT_MNCNPResTimeout_AU32
    struct _tEdrvTxBuffer*      m_pPreqTxBuffer;
    struct _tEplDllkNodeInfo*   m_pNextNodeInfo;
#endif

};

typedef struct _tEplDllkNodeInfo tEplDllkNodeInfo;

typedef struct
{
    DWORD   m_dwSyncControl;
    DWORD   m_dwPResTimeFirstNs;
    DWORD   m_dwPResTimeSecondNs;
    DWORD   m_dwSyncMNDelayFirstNs;
    DWORD   m_dwSyncMNDelaySecondNs;

} tEplDllkPrcCycleTiming;


// callback function for frame processing
typedef tEplKernel (* tEplDllkCbProcessRpdo) (tEplFrameInfo * pFrameInfo_p);

typedef tEplKernel (* tEplDllkCbProcessTpdo) (tEplFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p);


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

tEplKernel EplDllkAddInstance(tEplDllkInitParam * pInitParam_p);

tEplKernel EplDllkDelInstance(void);

// called before NMT_GS_COMMUNICATING will be entered to configure fixed parameters
tEplKernel EplDllkConfig(tEplDllConfigParam * pDllConfigParam_p);

// set identity of local node (may be at any time, e.g. in case of hostname change)
tEplKernel EplDllkSetIdentity(tEplDllIdentParam * pDllIdentParam_p);

// process internal events and do work that cannot be done in interrupt-context
tEplKernel EplDllkProcess(tEplEvent * pEvent_p);

// registers handler for non-EPL frames (used by Virtual Ethernet driver)
tEplKernel EplDllkRegAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p);

// deregisters handler for non-EPL frames
tEplKernel EplDllkDeregAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p);

// register C_DLL_MULTICAST_ASND in ethernet driver if any AsndServiceId is registered
tEplKernel EplDllkSetAsndServiceIdFilter(tEplDllAsndServiceId ServiceId_p, tEplDllAsndFilter Filter_p);

// registers handler for RPDOs frames
tEplKernel EplDllkRegRpdoHandler(tEplDllkCbProcessRpdo pfnDllkCbProcessRpdo_p);

// registers handler for TPDOs frames
tEplKernel EplDllkRegTpdoHandler(tEplDllkCbProcessTpdo pfnDllkCbProcessTpdo_p);

// registers handler for Sync event
tEplSyncCb EplDllkRegSyncHandler(tEplSyncCb pfnCbSync_p);

// Releases the rx buffer for the specified rx frame in Edrv
tEplKernel EplDllkReleaseRxFrame(tEplFrame* pFrame_p, unsigned int uiFrameSize_p);


#if EPL_NMT_MAX_NODE_ID > 0

tEplKernel EplDllkConfigNode(tEplDllNodeInfo* pNodeInfo_p);

tEplKernel EplDllkAddNode(tEplDllNodeOpParam* pNodeOpParam_p);

tEplKernel EplDllkDeleteNode(tEplDllNodeOpParam* pNodeOpParam_p);

#endif // EPL_NMT_MAX_NODE_ID > 0


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

tEplKernel EplDllkSetFlag1OfNode(unsigned int uiNodeId_p, BYTE bSoaFlag1_p);

tEplKernel EplDllkGetCurrentCnNodeIdList(BYTE** ppbCnNodeIdList_p);


#if EPL_DLL_PRES_CHAINING_MN != FALSE
tEplKernel EplDllkGetCnMacAddress(unsigned int uiNodeId_p, BYTE* pb_be_CnMacAddress_p);
#endif

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

#endif  // #ifndef _EPL_DLLK_H_


