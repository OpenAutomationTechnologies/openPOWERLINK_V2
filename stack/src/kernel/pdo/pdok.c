/**
********************************************************************************
\file   pdok.c

\brief  Implementation of kernel PDO module

This file contains the main implementation of the kernel PDO module.

\ingroup module_pdok
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/obd.h>
#include <oplk/ami.h>
#include <kernel/pdok.h>
#include <kernel/pdokcal.h>
#include "kernel/eventk.h"
#include <kernel/dllk.h>
#include <oplk/benchmark.h>

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
    tPdoChannelSetup        pdoChannels;        ///< PDO channel setup
    BOOL                    fRunning;           ///< Flag determines if PDO engine is running
}tPdokInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdokInstance  pdokInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError cbProcessTpdo(tFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p) SECTION_PDOK_PROCESS_TPDO_CB;
static tOplkError copyTxPdo(tEplFrame* pFrame_p, UINT frameSize_p, BOOL fReadyFlag_p);
static void disablePdoChannels(tPdoChannel *pPdoChannel, UINT channelCnt);

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
    tOplkError      ret = kEplSuccessful;

    EPL_MEMSET(&pdokInstance_g, 0, sizeof(pdokInstance_g));

    if ((ret = pdokcal_init()) != kEplSuccessful)
    {
        return ret;
    }

    dllk_regTpdoHandler(cbProcessTpdo);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO kernel module

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
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Deallocate memory for PDOs

This function deallocates memory for PDOs.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_deAllocChannelMem(void)
{
    tOplkError      Ret = kEplSuccessful;

#if EPL_NMT_MAX_NODE_ID > 0
    tDllNodeOpParam     NodeOpParam;

    NodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
    NodeOpParam.nodeId = EPL_C_ADR_BROADCAST;
    Ret = dllk_deleteNode(&NodeOpParam);
    if (Ret != kEplSuccessful)
    {
        DEBUG_LVL_PDO_TRACE("%s() EplDllkDeleteNode failed (%s)\n",
                             __func__, EplGetOplkErrorStr(Ret));
        return Ret;
    }
#endif // EPL_NMT_MAX_NODE_ID > 0


    // deallocate mem for RX PDO channels
    if (pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount != 0)
    {
        pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount = 0;
        if (pdokInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            EPL_FREE(pdokInstance_g.pdoChannels.pRxPdoChannel);
            pdokInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }
    }
    // deallocate mem for TX PDO channels
    if (pdokInstance_g.pdoChannels.allocation.txPdoChannelCount != 0)
    {
        pdokInstance_g.pdoChannels.allocation.txPdoChannelCount = 0;
        if (pdokInstance_g.pdoChannels.pTxPdoChannel != NULL)
        {
            EPL_FREE(pdokInstance_g.pdoChannels.pTxPdoChannel);
            pdokInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels

This function allocates memory for the PDO channels

\param  pAllocationParam_p      Pointer to allocation parameters.

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_allocChannelMem(tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError      ret = kEplSuccessful;

#if EPL_NMT_MAX_NODE_ID > 0
    tDllNodeOpParam     nodeOpParam;

    nodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
    nodeOpParam.nodeId = EPL_C_ADR_BROADCAST;
    ret = dllk_deleteNode(&nodeOpParam);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif // EPL_NMT_MAX_NODE_ID > 0

    if (pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount != pAllocationParam_p->rxPdoChannelCount)
    {   // allocation should be changed
        pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount =  pAllocationParam_p->rxPdoChannelCount;
        if (pdokInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            EPL_FREE(pdokInstance_g.pdoChannels.pRxPdoChannel);
            pdokInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }

        if (pAllocationParam_p->rxPdoChannelCount > 0)
        {
            pdokInstance_g.pdoChannels.pRxPdoChannel =
                            EPL_MALLOC(sizeof (*pdokInstance_g.pdoChannels.pRxPdoChannel) *
                                       pAllocationParam_p->rxPdoChannelCount);

            if (pdokInstance_g.pdoChannels.pRxPdoChannel == NULL)
            {
                ret = kEplPdoInitError;
                goto Exit;
            }
        }
    }

    disablePdoChannels(pdokInstance_g.pdoChannels.pRxPdoChannel,
                       pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount);

    if (pdokInstance_g.pdoChannels.allocation.txPdoChannelCount != pAllocationParam_p->txPdoChannelCount)
    {   // allocation should be changed

        pdokInstance_g.pdoChannels.allocation.txPdoChannelCount = pAllocationParam_p->txPdoChannelCount;
        if (pdokInstance_g.pdoChannels.pTxPdoChannel != NULL)
        {
            EPL_FREE(pdokInstance_g.pdoChannels.pTxPdoChannel);
            pdokInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }

        if (pAllocationParam_p->txPdoChannelCount > 0)
        {
            pdokInstance_g.pdoChannels.pTxPdoChannel =
                    EPL_MALLOC(sizeof (*pdokInstance_g.pdoChannels.pTxPdoChannel) *
                               pAllocationParam_p->txPdoChannelCount);

            if (pdokInstance_g.pdoChannels.pTxPdoChannel == NULL)
            {
                ret = kEplPdoInitError;
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

\param  pChannelConf_p          PDO channel configuration

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_configureChannel(tPdoChannelConf* pChannelConf_p)
{
    tOplkError          Ret = kEplSuccessful;
    tPdoChannel*        pDestPdoChannel;

    if (pChannelConf_p->fTx == FALSE)
    {   // RPDO
#if EPL_NMT_MAX_NODE_ID > 0
        tDllNodeOpParam     NodeOpParam;
        NodeOpParam.opNodeType = kDllNodeOpTypeFilterPdo;
#endif

        if (pChannelConf_p->channelId >= pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount)
        {
            Ret = kEplPdoNotExist;
            goto Exit;
        }

        pDestPdoChannel = &pdokInstance_g.pdoChannels.pRxPdoChannel[pChannelConf_p->channelId];

#if EPL_NMT_MAX_NODE_ID > 0
        if ((pDestPdoChannel->nodeId != PDO_INVALID_NODE_ID)
            && (pDestPdoChannel->nodeId != PDO_PREQ_NODE_ID))
        {   // disable old PRes filter in DLL
            NodeOpParam.nodeId = pDestPdoChannel->nodeId;
            Ret = dllk_deleteNode(&NodeOpParam);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
#endif // EPL_NMT_MAX_NODE_ID > 0

        // copy channel configuration to local structure
        EPL_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel,
                   sizeof (pChannelConf_p->pdoChannel));

#if EPL_NMT_MAX_NODE_ID > 0
        if ((pDestPdoChannel->nodeId != PDO_INVALID_NODE_ID)
            && (pDestPdoChannel->nodeId != PDO_PREQ_NODE_ID))
        {   // enable new PRes filter in DLL
            NodeOpParam.nodeId = pDestPdoChannel->nodeId;
            Ret = dllk_addNode(&NodeOpParam);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
#endif // EPL_NMT_MAX_NODE_ID > 0

    }
    else
    {   // TPDO
        if (pChannelConf_p->channelId >= pdokInstance_g.pdoChannels.allocation.txPdoChannelCount)
        {
            Ret = kEplPdoNotExist;
            goto Exit;
        }

        pDestPdoChannel = &pdokInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId];

        // copy channel to local structure
        EPL_MEMCPY(&pdokInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId],
                   &pChannelConf_p->pdoChannel, sizeof (pChannelConf_p->pdoChannel));
    }

    pdokInstance_g.fRunning = FALSE;
Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process a RxPDO

The function processes a received RxPDO.

\param  pFrame_p                Pointer to frame to be decoded
\param  frameSize_p             Size of frame to be encoded

\return The function returns a tOplkError error code.

\ingroup module_pdok
**/
//------------------------------------------------------------------------------
tOplkError pdok_processRxPdo(tEplFrame* pFrame_p, UINT frameSize_p)
{
    tOplkError          ret = kEplSuccessful;
    BYTE                frameData;
    UINT                nodeId;
    tEplMsgType         msgType;
    tPdoChannel*        pPdoChannel;
    UINT                channelId;

    // check if received RPDO is valid
    frameData = ami_getUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bFlag1);
    if ((frameData & EPL_FRAME_FLAG1_RD) == 0)
    {   // RPDO invalid
        goto Exit;
    }

    // retrieve EPL message type
    msgType = ami_getUint8Le(&pFrame_p->m_le_bMessageType);
    if (msgType == kEplMsgTypePreq)
    {   // RPDO is PReq frame
        nodeId = PDO_PREQ_NODE_ID;  // 0x00
    }
    else
    {   // RPDO is PRes frame
        // retrieve node ID
        nodeId = ami_getUint8Le(&pFrame_p->m_le_bSrcNodeId);
    }

    if (pdokInstance_g.fRunning)
    {
        // search for appropriate valid RPDO
        for (channelId = 0, pPdoChannel = &pdokInstance_g.pdoChannels.pRxPdoChannel[0];
             channelId < pdokInstance_g.pdoChannels.allocation.rxPdoChannelCount;
             channelId++, pPdoChannel++)
        {
            if (pPdoChannel->nodeId != nodeId)
            {
                continue;
            }

            // retrieve PDO version from frame
            frameData = ami_getUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bPdoVersion);
            if ((pPdoChannel->mappingVersion & EPL_VERSION_MAIN) != (frameData & EPL_VERSION_MAIN))
            {   // PDO versions do not match
                // $$$ raise PDO error
                // termiate processing of this RPDO
                goto Exit;
            }

            // valid RPDO found

            if ((unsigned int)(pPdoChannel->pdoSize + EPL_FRAME_OFFSET_PDO_PAYLOAD) > frameSize_p)
            {   // RPDO is too short
                // $$$ raise PDO error, set Ret
                goto Exit;
            }

            /*
            TRACE ("%s() Channel:%d Node:%d MapObjectCnt:%d PdoSize:%d\n",
                   __func__, channelId, nodeId, pPdoChannel->mappObjectCount,
                   pPdoChannel->pdoSize);
            */

            pdokcal_writeRxPdo(channelId,
                              &pFrame_p->m_Data.m_Pres.m_le_abPayload[0],
                              pPdoChannel->pdoSize);

            // processing finished successfully
            break;
        }
    }

Exit:
#if DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS != FALSE
    dllk_releaseRxFrame(pFrame_p, frameSize_p);
    // $$$ return value?
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  setup PDO buffers

The function sets up the memory used to store PDO frames.

\param  rxPdoMemSize_p      Size of RX PDO buffers.
\param  txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdok
*/
//------------------------------------------------------------------------------
tOplkError pdok_setupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p)
{
    tOplkError              ret;

    ret = pdokcal_initPdoMem(&pdokInstance_g.pdoChannels, rxPdoMemSize_p,
                             txPdoMemSize_p);
    if (ret != kEplSuccessful)
        return ret;

    pdokInstance_g.fRunning = TRUE;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event.

\return The function returns a tOplkError error code.

\ingroup module_pdok
*/
//------------------------------------------------------------------------------
tOplkError pdok_sendSyncEvent(void)
{
    pdokcal_sendSyncEvent();
    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  TPDO callback function

This function is called by DLL if PRes or PReq need to be encoded. It is called
in NMT_CS_PRE_OPERATIONAL_2, NMT_CS_READY_TO_OPERATE and NMT_CS_OPERATIONAL.

\param  pFrameInfo_p                Pointer to frame info structure
\param  fReadyFlag_p                State of RD flag which shall be set in TPDO

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError cbProcessTpdo(tFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p)
{
    tOplkError      Ret = kEplSuccessful;
    Ret = copyTxPdo(pFrameInfo_p->pFrame, pFrameInfo_p->frameSize, fReadyFlag_p);
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  disable PDO channels

The function disables all PDO channels of a direction (RX/TX)

\param  pPdoChannel_p           Pointer to first PDO channel
\param  channelCnt_p            Number of PDO channels
*/
//------------------------------------------------------------------------------
static void disablePdoChannels(tPdoChannel *pPdoChannel_p, UINT channelCnt_p)
{
    UINT        index;

    // disable all TPDOs
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

\param  pFrame_p                Pointer to frame.
\param  frameSize_p             Size of frame.
\param  fReadyFlag_p
//
\return The function returns a tOplkError error code.
**/
//---------------------------------------------------------------------------
static tOplkError copyTxPdo(tEplFrame* pFrame_p, UINT frameSize_p, BOOL fReadyFlag_p)
{
    tOplkError          ret = kEplSuccessful;
    BYTE                flag1;
    UINT                nodeId;
    tEplMsgType         msgType;
    tPdoChannel*        pPdoChannel;
    UINT                channelId;

    // set TPDO invalid, so that only fully processed TPDOs are sent as valid
    flag1 = ami_getUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bFlag1);
    ami_setUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bFlag1, (flag1 & ~EPL_FRAME_FLAG1_RD));

    // retrieve EPL message type
    msgType = ami_getUint8Le(&pFrame_p->m_le_bMessageType);
    if (msgType == kEplMsgTypePres)
    {   // TPDO is PRes frame
        nodeId = PDO_PRES_NODE_ID;  // 0x00
    }
    else
    {   // TPDO is PReq frame
        // retrieve node ID
        nodeId = ami_getUint8Le(&pFrame_p->m_le_bDstNodeId);
    }

    if (pdokInstance_g.fRunning)
    {
        // search for appropriate valid TPDO
        for (channelId = 0, pPdoChannel = &pdokInstance_g.pdoChannels.pTxPdoChannel[0];
             channelId < pdokInstance_g.pdoChannels.allocation.txPdoChannelCount;
             channelId++, pPdoChannel++)
        {
            if (pPdoChannel->nodeId != nodeId)
            {
                continue;
            }

            // valid TPDO found
            if ((unsigned int)(pPdoChannel->pdoSize + 24) > frameSize_p)
            {   // TPDO is too short
                // $$$ raise PDO error, set ret
                break;
            }

            /*
            TRACE ("%s() Channel:%d Node:%d MapObjectCnt:%d PdoSize:%d\n",
                __func__, channelId, nodeId, pPdoChannel->mappObjectCount, pPdoChannel->pdoSize);
            */

            // set PDO version in frame
            ami_setUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bPdoVersion, pPdoChannel->mappingVersion);

            pdokcal_readTxPdo(channelId, &pFrame_p->m_Data.m_Pres.m_le_abPayload[0],
                              pPdoChannel->pdoSize);

            // set PDO size in frame
            ami_setUint16Le(&pFrame_p->m_Data.m_Pres.m_le_wSize, pPdoChannel->pdoSize);

            if (fReadyFlag_p != FALSE)
            {
                // set TPDO valid
                ami_setUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bFlag1, (flag1 | EPL_FRAME_FLAG1_RD));
            }

            // processing finished successfully
            goto Exit;
        }
    }

    // set PDO size in frame to zero, because no TPDO mapped
    ami_setUint16Le(&pFrame_p->m_Data.m_Pres.m_le_wSize, 0);

    if (fReadyFlag_p != FALSE)
    {
        // set TPDO valid even if TPDO size is 0
        ami_setUint8Le(&pFrame_p->m_Data.m_Pres.m_le_bFlag1, (flag1 | EPL_FRAME_FLAG1_RD));
    }

Exit:
    return ret;
}

///\}


