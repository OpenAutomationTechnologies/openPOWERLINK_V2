/**
********************************************************************************
\file   dllkfilter.c

\brief  Filter functions for Edrv

This file contains the functions to setup the edrv filter structures.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/ami.h>
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

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
\brief  Setup ASnd filter

The function sets up an ASnd filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.

*/
//------------------------------------------------------------------------------
void dllk_setupAsndFilter(tEdrvFilter* pFilter_p)
{
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeAsnd);
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoC filter

The function sets up an SoC filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.

*/
//------------------------------------------------------------------------------
void dllk_setupSocFilter(tEdrvFilter* pFilter_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOC);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA filter

The function sets up an SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaFilter(tEdrvFilter* pFilter_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/IdentReq filter

The function sets up an IdentReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaIdentReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both Asnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceIdent);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/StatusReq filter

The function sets up an StatusReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaStatusReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both Asnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceStatus);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/NmtReq filter

The function sets up an NmtReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaNmtReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both Asnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceNmtRequest);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}


#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Setup SoA/SyncReq filter

The function sets up an SyncReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaSyncReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both Asnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceSync);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Setup Unspecific SoA filter

The function sets up an Unspecific SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
void dllk_setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both Asnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceUnspecified);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup PRes filter

The function sets up a PRes filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  fEnable_p       Flag determines if filter is enabled or disabled.

*/
//------------------------------------------------------------------------------
void dllk_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_PRES);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypePres);
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
    pFilter_p->fEnable = fEnable_p;
}

//------------------------------------------------------------------------------
/**
\brief  Setup PReq filter

The function sets up an PReq filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.
\param  pMacAdrs_p      Pointer to mac address of node.

*/
//------------------------------------------------------------------------------
void dllk_setupPreqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer* pBuffer_p, UINT8* pMacAdrs_p)
{
    OPLK_MEMCPY(&pFilter_p->aFilterValue[0], pMacAdrs_p, 6);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypePreq);
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[15], (UINT8)nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[15], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[16], C_ADR_MN_DEF_NODE_ID);
    ami_setUint8Be(&pFilter_p->aFilterMask[16], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

#if NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief  Add PRes filter for the specified node

This function adds a PRes filter for the specified node.

\param  pIntNodeInfo_p      Pointer to internal node info structure.
\param  nodeOpType_p        Type of PRes filter.
\param  fUpdateEdrv_p       Flag determines if Edrv Filter should be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllk_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType nodeOpType_p,
                              BOOL fUpdateEdrv_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           presFilterFlags = 0;

    switch (nodeOpType_p)
    {
        case kDllNodeOpTypeFilterPdo:
            presFilterFlags = DLLK_FILTER_FLAG_PDO;
            break;

        case kDllNodeOpTypeFilterHeartbeat:
            presFilterFlags = DLLK_FILTER_FLAG_HB;
            break;

        default:
            ret = kErrorDllInvalidParam;
            goto Exit;
    }

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if CONFIG_DLL_PRES_FILTER_COUNT < 0
            dllkInstance_g.usedPresFilterCount++;
            if (dllkInstance_g.usedPresFilterCount == 1)
            {
                // enable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = TRUE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                       DLLK_FILTER_PRES, EDRV_FILTER_CHANGE_STATE);
                if (ret != kErrorOk)
                    goto Exit;
            }

#else
            UINT handle;

            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (ami_getUint8Le(&dllkInstance_g.aFilter[handle].aFilterValue[16]) == C_ADR_INVALID)
                {
                    ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterValue[16],
                                   pIntNodeInfo_p->nodeId);
                    dllkInstance_g.aFilter[handle].fEnable = TRUE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           handle, (EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_VALUE));
                    if (ret != kErrorOk)
                        goto Exit;
                    break;
                }
            }
#endif
        }
    }
    pIntNodeInfo_p->presFilterFlags |= presFilterFlags;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete PRes filter for the specified node

This function deletes a PRes filter for the specified node.

\param  pIntNodeInfo_p      Pointer to internal node info structure.
\param  nodeOpType_p        Type of PRes filter.
\param  fUpdateEdrv_p       Flag determines if Edrv Filter should be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllk_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType nodeOpType_p,
                                 BOOL fUpdateEdrv_p)
{
    tOplkError      ret = kErrorOk;
    BYTE            bPresFilterFlags = 0;

    switch (nodeOpType_p)
    {
        case kDllNodeOpTypeFilterPdo:
            bPresFilterFlags = DLLK_FILTER_FLAG_PDO;
            break;

        case kDllNodeOpTypeFilterHeartbeat:
            bPresFilterFlags = DLLK_FILTER_FLAG_HB;
            break;

        default:
            ret = kErrorDllInvalidParam;
            goto Exit;
    }

    pIntNodeInfo_p->presFilterFlags &= ~bPresFilterFlags;

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if CONFIG_DLL_PRES_FILTER_COUNT < 0
            if (dllkInstance_g.usedPresFilterCount > 0)
                dllkInstance_g.usedPresFilterCount--;

            if (dllkInstance_g.usedPresFilterCount == 0)
            {
                // disable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = FALSE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                       DLLK_FILTER_PRES, EDRV_FILTER_CHANGE_STATE);
                if (ret != kErrorOk)
                    goto Exit;
            }

#else
            UINT handle;

            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (ami_getUint8Le(&dllkInstance_g.aFilter[handle].aFilterValue[16]) ==
                                                pIntNodeInfo_p->nodeId)
                {
                    ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterValue[16], C_ADR_INVALID);
                    dllkInstance_g.aFilter[handle].fEnable = FALSE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           handle, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kErrorOk)
                        goto Exit;
                    break;
                }
            }
#endif
        }
    }
Exit:
    return ret;
}
#endif

//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//

///\}

