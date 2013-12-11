/**
********************************************************************************
\file   dllkfilter.c

\brief  Filter functions for Edrv

This file contains the functions to setup the edrv filter structures.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupAsndFilter(tEdrvFilter* pFilter_p)
{
    AmiSetQword48ToBe(&pFilter_p->aFilterValue[0], EPL_C_DLL_MULTICAST_ASND);
    AmiSetQword48ToBe(&pFilter_p->aFilterMask[0],  EPL_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoC filter

The function sets up an SoC filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSocFilter(tEdrvFilter* pFilter_p)
{
    AmiSetQword48ToBe(&pFilter_p->aFilterValue[0], EPL_C_DLL_MULTICAST_SOC);
    AmiSetQword48ToBe(&pFilter_p->aFilterMask[0], EPL_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA filter

The function sets up an SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaFilter(tEdrvFilter* pFilter_p)
{
    AmiSetQword48ToBe(&pFilter_p->aFilterValue[0], EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&pFilter_p->aFilterMask[0], EPL_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/IdentReq filter

The function sets up an IdentReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaIdentReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypeSoa);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[20],   kDllReqServiceIdent);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[20],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[21],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[21],    0xFF);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaStatusReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypeSoa);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[20],   kDllReqServiceStatus);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[20],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[21],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[21],    0xFF);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaNmtReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypeSoa);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[20],   kDllReqServiceNmtRequest);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[20],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[21],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[21],    0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}


#if EPL_DLL_PRES_CHAINING_CN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Setup SoA/SyncReq filter

The function sets up an SyncReq SoA filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  nodeId_p        Node ID for which to set the filter.
\param  pBuffer_p       Pointer to TX buffer.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaSyncReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypeSoa);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[20],   kDllReqServiceSync);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[20],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[21],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[21],    0xFF);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypeSoa);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[20],   kDllReqServiceUnspecified);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[20],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[21],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[21],    0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup PRes filter

The function sets up a PRes filter in the Edrv filter structure.

\param  pFilter_p       Pointer to Edrv filte structure.
\param  fEnable_p       Flag determines if filter is enabled or disabled.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p)
{
    AmiSetQword48ToBe   (&pFilter_p->aFilterValue[0],    EPL_C_DLL_MULTICAST_PRES);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypePres);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_setupPreqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p, UINT8* pMacAdrs_p)
{
    EPL_MEMCPY(&pFilter_p->aFilterValue[0], pMacAdrs_p, 6);
    AmiSetQword48ToBe   (&pFilter_p->aFilterMask[0],     EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe      (&pFilter_p->aFilterValue[12],   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe      (&pFilter_p->aFilterMask[12],    0xFFFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[14],   kEplMsgTypePreq);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[14],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[15],   (UINT8)nodeId_p);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[15],    0xFF);
    AmiSetByteToBe      (&pFilter_p->aFilterValue[16],   EPL_C_ADR_MN_DEF_NODE_ID);
    AmiSetByteToBe      (&pFilter_p->aFilterMask[16],    0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

#if EPL_NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief  Add PRes filter for the specified node

This function adds a PRes filter for the specified node.

\param  pIntNodeInfo_p      Pointer to internal node info structure.
\param  nodeOpType_p        Type of PRes filter.
\param  fUpdateEdrv_p       Flag determines if Edrv Filter should be updated.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType nodeOpType_p,
                              BOOL fUpdateEdrv_p)
{
    tEplKernel      ret = kEplSuccessful;
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
            ret = kEplDllInvalidParam;
            goto Exit;
    }

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if EPL_DLL_PRES_FILTER_COUNT < 0
            dllkInstance_g.usedPresFilterCount++;
            if (dllkInstance_g.usedPresFilterCount == 1)
            {
                // enable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = TRUE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                       DLLK_FILTER_PRES, EDRV_FILTER_CHANGE_STATE);
                if (ret != kEplSuccessful)
                    goto Exit;
            }

#else
            UINT        handle;
            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (AmiGetByteFromLe(&dllkInstance_g.aFilter[handle].aFilterValue[16]) == EPL_C_ADR_INVALID)
                {
                    AmiSetByteToBe(&dllkInstance_g.aFilter[handle].aFilterValue[16],
                                   pIntNodeInfo_p->nodeId);
                    dllkInstance_g.aFilter[handle].fEnable = TRUE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           handle, (EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_VALUE));
                    if (ret != kEplSuccessful)
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

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType nodeOpType_p,
                                 BOOL fUpdateEdrv_p)
{
    tEplKernel      ret = kEplSuccessful;
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
            ret = kEplDllInvalidParam;
            goto Exit;
    }

    pIntNodeInfo_p->presFilterFlags &= ~bPresFilterFlags;

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if EPL_DLL_PRES_FILTER_COUNT < 0
            if (dllkInstance_g.usedPresFilterCount > 0)
                dllkInstance_g.usedPresFilterCount--;

            if (dllkInstance_g.usedPresFilterCount == 0)
            {
                // disable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = FALSE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                       DLLK_FILTER_PRES, EDRV_FILTER_CHANGE_STATE);
                if (ret != kEplSuccessful)
                    goto Exit;
            }

#else
            UINT        handle;

            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (AmiGetByteFromLe(&dllkInstance_g.aFilter[handle].aFilterValue[16]) ==
                                                pIntNodeInfo_p->nodeId)
                {
                    AmiSetByteToBe(&dllkInstance_g.aFilter[handle].aFilterValue[16], EPL_C_ADR_INVALID);
                    dllkInstance_g.aFilter[handle].fEnable = FALSE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           handle, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kEplSuccessful)
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







