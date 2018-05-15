/**
********************************************************************************
\file   dllkfilter.c

\brief  Filter functions for Edrv

This file contains the functions to setup the edrv filter structures.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <kernel/dllkfilter.h>

#include "dllk-internal.h"

#include <common/ami.h>
#include <oplk/frame.h>
#include <oplk/dll.h>

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
static void setupAsndFilter(tEdrvFilter* pFilter_p);
static void setupSocFilter(tEdrvFilter* pFilter_p);
static void setupSoaFilter(tEdrvFilter* pFilter_p);
static void setupSoaIdentReqFilter(tEdrvFilter* pFilter_p,
                                   UINT8 nodeId_p,
                                   tEdrvTxBuffer* pBuffer_p);
static void setupSoaStatusReqFilter(tEdrvFilter* pFilter_p,
                                    UINT8 nodeId_p,
                                    tEdrvTxBuffer* pBuffer_p);
static void setupSoaNmtReqFilter(tEdrvFilter* pFilter_p,
                                 UINT8 nodeId_p,
                                 tEdrvTxBuffer* pBuffer_p);
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
static void setupSoaSyncReqFilter(tEdrvFilter* pFilter_p,
                                  UINT8 nodeId_p,
                                  tEdrvTxBuffer* pBuffer_p);
#endif
static void setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p,
                                    UINT8 nodeId_p,
                                    tEdrvTxBuffer* pBuffer_p);
#if defined(CONFIG_INCLUDE_VETH)
static void setupVethUnicast(tEdrvFilter* pFilter_p,
                             const UINT8* pMacAdrs_p,
                             BOOL fEnable_p);
static void setupVethBroadcast(tEdrvFilter* pFilter_p, BOOL fEnable_p);
#endif

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
\brief  Setup all POWERLINK filters

The function sets up all filters in the Edrv filter structure.

*/
//------------------------------------------------------------------------------
void dllkfilter_setupFilters(void)
{
    OPLK_MEMSET(dllkInstance_g.aFilter, 0, sizeof(dllkInstance_g.aFilter));
    setupAsndFilter(&dllkInstance_g.aFilter[DLLK_FILTER_ASND]);
    setupSocFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOC]);
    setupSoaFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA]);
    setupSoaIdentReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_IDREQ],
                           dllkInstance_g.dllConfigParam.nodeId,
                           &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES]);
    setupSoaStatusReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_STATREQ],
                            dllkInstance_g.dllConfigParam.nodeId,
                            &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES]);
    setupSoaNmtReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_NMTREQ],
                         dllkInstance_g.dllConfigParam.nodeId,
                         &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ]);
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    setupSoaSyncReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_SYNCREQ],
                          dllkInstance_g.dllConfigParam.nodeId,
                          &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
#endif
    setupSoaUnspecReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_NONPLK],
                            dllkInstance_g.dllConfigParam.nodeId,
                            &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK]);
#if defined(CONFIG_INCLUDE_VETH)
    setupVethUnicast(&dllkInstance_g.aFilter[DLLK_FILTER_VETH_UNICAST],
                     edrv_getMacAddr(),
                     TRUE);
    setupVethBroadcast(&dllkInstance_g.aFilter[DLLK_FILTER_VETH_BROADCAST], TRUE);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Setup PRes filter

The function sets up a PRes filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      fEnable_p           Flag determines if filter is enabled or disabled.

*/
//------------------------------------------------------------------------------
void dllkfilter_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p)
{
    // Check parameter validity
    ASSERT(pFilter_p != NULL);

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

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.
\param[in]      pMacAdrs_p          Pointer to mac address of node.

*/
//------------------------------------------------------------------------------
void dllkfilter_setupPreqFilter(tEdrvFilter* pFilter_p,
                                UINT8 nodeId_p,
                                tEdrvTxBuffer* pBuffer_p,
                                const UINT8* pMacAdrs_p)
{
    // Check parameter validity
    ASSERT(pFilter_p != NULL);
    ASSERT(pBuffer_p != NULL);
    ASSERT(pMacAdrs_p != NULL);

    OPLK_MEMCPY(&pFilter_p->aFilterValue[0], pMacAdrs_p, 6);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypePreq);
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[15], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[15], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[16], C_ADR_MN_DEF_NODE_ID);
    ami_setUint8Be(&pFilter_p->aFilterMask[16], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//


//------------------------------------------------------------------------------
/**
\brief  Setup ASnd filter

The function sets up an ASnd filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.

*/
//------------------------------------------------------------------------------
static void setupAsndFilter(tEdrvFilter* pFilter_p)
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

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.

*/
//------------------------------------------------------------------------------
static void setupSocFilter(tEdrvFilter* pFilter_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOC);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA filter

The function sets up an SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.

*/
//------------------------------------------------------------------------------
static void setupSoaFilter(tEdrvFilter* pFilter_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    pFilter_p->fEnable = TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/IdentReq filter

The function sets up an IdentReq SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
static void setupSoaIdentReqFilter(tEdrvFilter* pFilter_p,
                                   UINT8 nodeId_p,
                                   tEdrvTxBuffer* pBuffer_p)
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
    ami_setUint8Be(&pFilter_p->aFilterValue[21], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/StatusReq filter

The function sets up an StatusReq SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
static void setupSoaStatusReqFilter(tEdrvFilter* pFilter_p,
                                    UINT8 nodeId_p,
                                    tEdrvTxBuffer* pBuffer_p)
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
    ami_setUint8Be(&pFilter_p->aFilterValue[21], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Setup SoA/NmtReq filter

The function sets up an NmtReq SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
static void setupSoaNmtReqFilter(tEdrvFilter* pFilter_p,
                                 UINT8 nodeId_p,
                                 tEdrvTxBuffer* pBuffer_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MULTICAST_SOA);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);
    ami_setUint16Be(&pFilter_p->aFilterValue[12], C_DLL_ETHERTYPE_EPL);
    ami_setUint16Be(&pFilter_p->aFilterMask[12], 0xFFFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[14], kMsgTypeSoa);
#if defined(CONFIG_INCLUDE_MASND)
    // Ignore bit4 of message type to react on both ASnd and AInv frames
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xF7);
#else
    ami_setUint8Be(&pFilter_p->aFilterMask[14], 0xFF);
#endif
    ami_setUint8Be(&pFilter_p->aFilterValue[20], kDllReqServiceNmtRequest);
    ami_setUint8Be(&pFilter_p->aFilterMask[20], 0xFF);
    ami_setUint8Be(&pFilter_p->aFilterValue[21], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}


#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Setup SoA/SyncReq filter

The function sets up an SyncReq SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
static void setupSoaSyncReqFilter(tEdrvFilter* pFilter_p,
                                  UINT8 nodeId_p,
                                  tEdrvTxBuffer* pBuffer_p)
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
    ami_setUint8Be(&pFilter_p->aFilterValue[21], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Setup Unspecific SoA filter

The function sets up an Unspecific SoA filter in the Edrv filter structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      nodeId_p            Node ID for which to set the filter.
\param[in]      pBuffer_p           Pointer to TX buffer.

*/
//------------------------------------------------------------------------------
static void setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p,
                                    UINT8 nodeId_p,
                                    tEdrvTxBuffer* pBuffer_p)
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
    ami_setUint8Be(&pFilter_p->aFilterValue[21], nodeId_p);
    ami_setUint8Be(&pFilter_p->aFilterMask[21], 0xFF);
    pFilter_p->pTxBuffer = pBuffer_p;
    pFilter_p->fEnable = FALSE;
}

#if defined(CONFIG_INCLUDE_VETH)
//------------------------------------------------------------------------------
/**
\brief  Setup virtual Ethernet unicast filter

The function sets up an virtual Ethernet unicast filter in the Edrv filter
structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      pMacAdrs_p          Pointer to MAC address of node.
\param[in]      fEnable_p           Flag determines if filter is enabled or disabled.

*/
//------------------------------------------------------------------------------
static void setupVethUnicast(tEdrvFilter* pFilter_p,
                             const UINT8* pMacAdrs_p,
                             BOOL fEnable_p)
{
    OPLK_MEMCPY(&pFilter_p->aFilterValue[0], pMacAdrs_p, 6);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);

    pFilter_p->pTxBuffer = NULL;
    pFilter_p->fEnable = fEnable_p;
}

//------------------------------------------------------------------------------
/**
\brief  Setup virtual Ethernet broadcast filter

The function sets up an virtual Ethernet broadcast filter in the Edrv filter
structure.

\param[in,out]  pFilter_p           Pointer to Edrv filter structure.
\param[in]      fEnable_p           Flag determines if filter is enabled or disabled.

*/
//------------------------------------------------------------------------------
static void setupVethBroadcast(tEdrvFilter* pFilter_p, BOOL fEnable_p)
{
    ami_setUint48Be(&pFilter_p->aFilterValue[0], C_DLL_MACADDR_MASK);
    ami_setUint48Be(&pFilter_p->aFilterMask[0], C_DLL_MACADDR_MASK);

    pFilter_p->pTxBuffer = NULL;
    pFilter_p->fEnable = fEnable_p;
}
#endif

/// \}
