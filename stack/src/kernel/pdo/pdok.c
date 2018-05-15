/**
********************************************************************************
\file   pdok.c

\brief  Implementation of kernel PDO module

This file contains the main implementation of the kernel PDO module.

\ingroup module_pdok
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <kernel/pdok.h>
#include <kernel/pdokcal.h>
#include <kernel/pdoklut.h>
#include <kernel/dllk.h>
#include <common/ami.h>
#include <oplk/debugstr.h>

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

/**
\brief Kernel PDO module instance

The following structure defines the instance variable of the kernel PDO module.
*/

typedef struct
{
    tPdoChannelSetup        pdoChannels;                            ///< PDO channel setup
    BOOL                    fRunning;                               ///< Flag determines if PDO engine is running
    tPdoklutEntry           aTxPdoLut[D_PDO_TPDOChannels_U16];      ///< TX PDO lookup table used for fast search of PDO channels
    tPdoklutEntry           aRxPdoLut[D_PDO_RPDOChannels_U16];      ///< RX PDO lookup table used for fast search of PDO channels
} tPdokInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdokInstance  pdokInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError cbProcessTpdo(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p) SECTION_PDOK_PROCESS_TPDO_CB;
static tOplkError copyTxPdo(tPlkFrame* pFrame_p, UINT frameSize_p, BOOL fReadyFlag_p);
static void       disablePdoChannels(tPdoChannel* pPdoChannel, UINT channelCnt);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO kernel module

The function initializes the PDO kernel module.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_init(void)
{
    tOplkError  ret;

    OPLK_MEMSET(&pdokInstance_g, 0, sizeof(pdokInstance_g));

    ret = pdokcal_init();
    if (ret != kErrorOk)
        return ret;

    dllk_regTpdoHandler(cbProcessTpdo);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO kernel module

The function cleans up the PDO kernel module.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_exit(void)
{
    pdokInstance_g.fRunning = FALSE;
    dllk_regTpdoHandler(NULL);
    pdok_deAllocChannelMem();
    pdokcal_cleanupPdoMem();
    pdokcal_exit();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  De-allocate memory for PDOs

This function de-allocates memory for PDOs.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_deAllocChannelMem(void)
{
    tOplkError      ret = kErrorOk;

#if (NMT_MAX_NODE_ID > 0)
    tDllNodeOpParam nodeOpParam;

    nodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
    nodeOpParam.nodeId = C_ADR_BROADCAST;
    ret = dllk_deleteNode(&nodeOpParam);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_PDO_TRACE("%s() dllk_deleteNode failed (%s)\n",
                            __func__,
                            debugstr_getRetValStr(ret));
        return ret;
    }
#endif // NMT_MAX_NODE_ID > 0

    // de-allocate mem for RX PDO channels
    if (pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount != 0)
    {
        pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount = 0;
        if (pdokInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            OPLK_FREE(pdokInstance_g.pdoChannels.pRxPdoChannel);
            pdokInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }
    }

    // de-allocate mem for TX PDO channels
    if (pdokInstance_g.pdoChannels.allocation.txPdoChannelCount != 0)
    {
        pdokInstance_g.pdoChannels.allocation.txPdoChannelCount = 0;
        if (pdokInstance_g.pdoChannels.pTxPdoChannel != NULL)
        {
            OPLK_FREE(pdokInstance_g.pdoChannels.pTxPdoChannel);
            pdokInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels

This function allocates memory for the PDO channels.

\param[in]      pAllocationParam_p  Pointer to allocation parameters.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_allocChannelMem(const tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError      ret = kErrorOk;

#if (NMT_MAX_NODE_ID > 0)
    tDllNodeOpParam nodeOpParam;
#endif

    // Check parameter validity
    ASSERT(pAllocationParam_p != NULL);

#if (NMT_MAX_NODE_ID > 0)
    nodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
    nodeOpParam.nodeId = C_ADR_BROADCAST;
    ret = dllk_deleteNode(&nodeOpParam);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif // NMT_MAX_NODE_ID > 0

    pdoklut_clear(pdokInstance_g.aRxPdoLut, D_PDO_RPDOChannels_U16);

    if (pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount != pAllocationParam_p->rxPdoChannelCount)
    {   // allocation should be changed
        pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount = pAllocationParam_p->rxPdoChannelCount;
        if (pdokInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            OPLK_FREE(pdokInstance_g.pdoChannels.pRxPdoChannel);
            pdokInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }

        if (pAllocationParam_p->rxPdoChannelCount > 0)
        {
            pdokInstance_g.pdoChannels.pRxPdoChannel =
                (tPdoChannel*)OPLK_MALLOC(sizeof(*pdokInstance_g.pdoChannels.pRxPdoChannel) *
                                              pAllocationParam_p->rxPdoChannelCount);

            if (pdokInstance_g.pdoChannels.pRxPdoChannel == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }
        }
    }

    disablePdoChannels(pdokInstance_g.pdoChannels.pRxPdoChannel,
                       pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount);

    pdoklut_clear(pdokInstance_g.aTxPdoLut, D_PDO_TPDOChannels_U16);

    if (pdokInstance_g.pdoChannels.allocation.txPdoChannelCount != pAllocationParam_p->txPdoChannelCount)
    {   // allocation should be changed
        pdokInstance_g.pdoChannels.allocation.txPdoChannelCount = pAllocationParam_p->txPdoChannelCount;
        if (pdokInstance_g.pdoChannels.pTxPdoChannel != NULL)
        {
            OPLK_FREE(pdokInstance_g.pdoChannels.pTxPdoChannel);
            pdokInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }

        if (pAllocationParam_p->txPdoChannelCount > 0)
        {
            pdokInstance_g.pdoChannels.pTxPdoChannel =
                (tPdoChannel*)OPLK_MALLOC(sizeof(*pdokInstance_g.pdoChannels.pTxPdoChannel) *
                                              pAllocationParam_p->txPdoChannelCount);

            if (pdokInstance_g.pdoChannels.pTxPdoChannel == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }
        }
    }

    disablePdoChannels(pdokInstance_g.pdoChannels.pTxPdoChannel,
                       pdokInstance_g.pdoChannels.allocation.txPdoChannelCount);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configures the specified PDO channel

\param[in]      pChannelConf_p      PDO channel configuration

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_configureChannel(const tPdoChannelConf* pChannelConf_p)
{
    tOplkError      ret = kErrorOk;
    tPdoChannel*    pDestPdoChannel;

    // Check parameter validity
    ASSERT(pChannelConf_p != NULL);

    if (pChannelConf_p->fTx == FALSE)
    {   // RPDO
#if (NMT_MAX_NODE_ID > 0)
        tDllNodeOpParam nodeOpParam;
        nodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
#endif

        if (pChannelConf_p->channelId >= pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount)
        {
            ret = kErrorPdoNotExist;
            goto Exit;
        }

        pDestPdoChannel = &pdokInstance_g.pdoChannels.pRxPdoChannel[pChannelConf_p->channelId];

        // copy channel configuration to local structure
        OPLK_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel, sizeof(pChannelConf_p->pdoChannel));

        // Store channel ID for fast access
        pdoklut_addChannel(pdokInstance_g.aRxPdoLut, pDestPdoChannel, pChannelConf_p->channelId);

#if (NMT_MAX_NODE_ID > 0)
        if ((pDestPdoChannel->nodeId != PDO_INVALID_NODE_ID) &&
            (pDestPdoChannel->nodeId != PDO_PREQ_NODE_ID))
        {   // disable old PRes filter in DLL
            nodeOpParam.nodeId = pDestPdoChannel->nodeId;
            ret = dllk_deleteNode(&nodeOpParam);
            if (ret != kErrorOk)
                goto Exit;

            // enable new PRes filter in DLL
            nodeOpParam.nodeId = pDestPdoChannel->nodeId;
            ret = dllk_addNode(&nodeOpParam);
            if (ret != kErrorOk)
                goto Exit;
        }
#endif // NMT_MAX_NODE_ID > 0
    }
    else
    {   // TPDO
        if (pChannelConf_p->channelId >= pdokInstance_g.pdoChannels.allocation.txPdoChannelCount)
        {
            ret = kErrorPdoNotExist;
            goto Exit;
        }

        pDestPdoChannel = &pdokInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId];

        // copy channel to local structure
        OPLK_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel, sizeof(pChannelConf_p->pdoChannel));

        // Store channel ID for fast access
        pdoklut_addChannel(pdokInstance_g.aTxPdoLut, pDestPdoChannel, pChannelConf_p->channelId);
    }

    pdokInstance_g.fRunning = FALSE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process a RxPDO

The function processes a received RxPDO.

\param[in]      pFrame_p            Pointer to frame to be decoded
\param[in]      frameSize_p         Size of frame to be encoded

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_processRxPdo(const tPlkFrame* pFrame_p, UINT frameSize_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           frameData;
    UINT            nodeId;
    tMsgType        msgType;
    tPdoChannel*    pPdoChannel;
    UINT8           channelId;
    UINT8           index;
    UINT16          pdoPayloadSize;

    // Check parameter validity
    ASSERT(pFrame_p != NULL);

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC == FALSE)
    UNUSED_PARAMETER(frameSize_p);
#endif

    // check if received RPDO is valid
    frameData = ami_getUint8Le(&pFrame_p->data.pres.flag1);
    if ((frameData & PLK_FRAME_FLAG1_RD) == 0)
    {   // RPDO invalid
        goto Exit;
    }

    // retrieve POWERLINK message type
    msgType = (tMsgType)ami_getUint8Le(&pFrame_p->messageType);
    if (msgType == kMsgTypePreq)
    {   // RPDO is PReq frame
        nodeId = PDO_PREQ_NODE_ID;  // 0x00
    }
    else
    {   // RPDO is PRes frame
        // retrieve node ID
        nodeId = ami_getUint8Le(&pFrame_p->srcNodeId);
    }

    if (pdokInstance_g.fRunning)
    {
        // Get PDO channel reference
        index = 0;
        while ((channelId = pdoklut_getChannel(pdokInstance_g.aRxPdoLut, index, nodeId)) != PDOKLUT_INVALID_CHANNEL)
        {
            index++;
            pPdoChannel = &pdokInstance_g.pdoChannels.pRxPdoChannel[channelId];

            // retrieve PDO version from frame
            frameData = ami_getUint8Le(&pFrame_p->data.pres.pdoVersion);
            if ((pPdoChannel->mappingVersion & PLK_VERSION_MAIN) != (frameData & PLK_VERSION_MAIN))
            {   // PDO versions do not match
                // $$$ raise PDO error E_PDO_MAP_VERS
                // terminate processing of this RPDO
                goto Exit;
            }

            // valid RPDO found
            pdoPayloadSize = ami_getUint16Le(&pFrame_p->data.pres.sizeLe);
            if (pPdoChannel->nextChannelOffset > pdoPayloadSize)
            {   // RPDO is too short
                // $$$ raise PDO error E_PDO_SHORT_RX, set Ret
                goto Exit;
            }

            /*
            TRACE("%s() Channel:%d Node:%d MapObjectCnt:%d PdoSize:%d\n",
                  __func__,
                  channelId,
                  nodeId,
                  pPdoChannel->mappObjectCount,
                  pPdoChannel->pdoSize);
            */

            pdokcal_writeRxPdo(channelId,
                               &pFrame_p->data.pres.aPayload[0] + pPdoChannel->offset,
                               pPdoChannel->nextChannelOffset - pPdoChannel->offset);
        }
    }

Exit:
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE)
    dllk_releaseRxFrame((tPlkFrame*)pFrame_p, frameSize_p);
    // $$$ return value?
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  setup PDO buffers

The function sets up the memory used to store PDO frames.

\param[in]      rxPdoMemSize_p      Size of RX PDO buffers.
\param[in]      txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdok
*/
//------------------------------------------------------------------------------
tOplkError pdok_setupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p)
{
    tOplkError  ret;

    ret = pdokcal_initPdoMem(&pdokInstance_g.pdoChannels,
                             rxPdoMemSize_p,
                             txPdoMemSize_p);
    if (ret != kErrorOk)
        return ret;

    pdokInstance_g.fRunning = TRUE;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  TPDO callback function

This function is called by the DLL if a PRes or a PReq need to be encoded. It
is called in NMT_CS_PRE_OPERATIONAL_2, NMT_CS_READY_TO_OPERATE and
NMT_CS_OPERATIONAL.

\param[in,out]  pFrameInfo_p        Pointer to frame info structure
\param[in]      fReadyFlag_p        State of RD flag which shall be set in TPDO

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError cbProcessTpdo(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p)
{
    tOplkError  ret;

    ret = copyTxPdo(pFrameInfo_p->frame.pBuffer, pFrameInfo_p->frameSize, fReadyFlag_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Disable PDO channels

The function disables all PDO channels of a given direction (RX/TX)

\param[in,out]  pPdoChannel_p       Pointer to first PDO channel
\param[in]      channelCnt_p        Number of PDO channels
*/
//------------------------------------------------------------------------------
static void disablePdoChannels(tPdoChannel* pPdoChannel_p, UINT channelCnt_p)
{
    UINT    index;

    // disable all PDOs
    for (index = 0; index < channelCnt_p; index++)
    {
        pPdoChannel_p[index].nodeId = PDO_INVALID_NODE_ID;
        //pPdoChannel_p[index].pVar = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Copy TX PDO

This function copies a PDO into the specified frame.

\param[in,out]  pFrame_p            Pointer to frame.
\param[in]      frameSize_p         Size of frame.
\param[in]      fReadyFlag_p        State of RD flag which shall be set in TPDO.
//
\return The function returns a tOplkError error code.
**/
//---------------------------------------------------------------------------
static tOplkError copyTxPdo(tPlkFrame* pFrame_p, UINT frameSize_p, BOOL fReadyFlag_p)
{
    tOplkError          ret = kErrorOk;
    UINT8               flag1;
    UINT                nodeId;
    tMsgType            msgType;
    tPdoChannel*        pPdoChannel;
    UINT8               channelId;
    UINT16              pdoSize;
    UINT                index;

    // set TPDO invalid, so that only fully processed TPDOs are sent as valid
    flag1 = ami_getUint8Le(&pFrame_p->data.pres.flag1);
    ami_setUint8Le(&pFrame_p->data.pres.flag1, (flag1 & ~PLK_FRAME_FLAG1_RD));

    // retrieve POWERLINK message type
    msgType = (tMsgType)ami_getUint8Le(&pFrame_p->messageType);
    if (msgType == kMsgTypePres)
    {   // TPDO is PRes frame
        nodeId = PDO_PRES_NODE_ID;  // 0x00
    }
    else
    {   // TPDO is PReq frame
        // retrieve node ID
        nodeId = ami_getUint8Le(&pFrame_p->dstNodeId);
    }

    if (pdokInstance_g.fRunning)
    {
        pdoSize = 0;

        // Get PDO channel reference
        index = 0;
        while ((channelId = pdoklut_getChannel(pdokInstance_g.aTxPdoLut, index, nodeId)) != PDOKLUT_INVALID_CHANNEL)
        {
            index++;
            pPdoChannel = &pdokInstance_g.pdoChannels.pTxPdoChannel[channelId];

            // TRACE("%s() Channel:%d Node:%d MapObjectCnt:%d PdoSize:%d\n",
            //      __func__,
            //      channelId,
            //      nodeId,
            //      pPdoChannel->mappObjectCount,
            //      pPdoChannel->pdoSize);

            if ((UINT32)(pPdoChannel->nextChannelOffset + 24) <= frameSize_p)
            {
                // set PDO version in frame
                ami_setUint8Le(&pFrame_p->data.pres.pdoVersion, pPdoChannel->mappingVersion);

                pdokcal_readTxPdo(channelId, &pFrame_p->data.pres.aPayload[0] + pPdoChannel->offset,
                                  pPdoChannel->nextChannelOffset - pPdoChannel->offset);

                // set PDO size in frame
                pdoSize = pPdoChannel->nextChannelOffset;
            }
            else
            {   // TPDO is too short or invalid
                // $$$ raise PDO error, set ret
                pdoSize = 0;
                break;
            }
        }

    }
    else
    {
        // set PDO size in frame to zero, because no TPDO mapped
        pdoSize = 0;
    }

    // set PDO size in frame
    ami_setUint16Le(&pFrame_p->data.pres.sizeLe, pdoSize);

    if (fReadyFlag_p != FALSE)
    {
        // set TPDO valid
        ami_setUint8Le(&pFrame_p->data.pres.flag1, (flag1 | PLK_FRAME_FLAG1_RD));
    }

    return ret;
}

/// \}
