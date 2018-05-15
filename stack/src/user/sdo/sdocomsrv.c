/**
********************************************************************************
\file   sdocomsrv.c

\brief  Implementation of SDO Command Layer server

This file contains the standard command layer implementation of the SDO server.

\ingroup module_sdocom_std
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <user/sdocomint.h>
#include <user/sdocomsrv.h>
#include <common/ami.h>
#include <stddef.h>

#if defined(CONFIG_INCLUDE_SDOS)
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
static tOplkError initWriteByIndex(tSdoComCon* pSdoComCon_p,
                                   const tAsySdoCom* pSdoCom_p);
static tOplkError initReadByIndex(tSdoComCon* pSdoComCon_p,
                                  const tAsySdoCom* pSdoCom_p);
static tOplkError initWriteMultiByIndex(tSdoComCon* pSdoComCon_p,
                                        const tAsySdoCom* pSdoCom_p);
static tOplkError initReadMultiByIndex(tSdoComCon* pSdoComCon_p,
                                       const tAsySdoCom* pSdoCom_p);
static tOplkError finishInitWriteByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError finishInitReadByIndex(tSdoComCon* pSdoComCon_p,
                                        tPlkFrame* pPlkFrame_p,
                                        size_t sdoCmdDataSize_p);
static tOplkError finishInitWriteMultiByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError finishInitReadMultiByIndex(tSdoComCon* pSdoComCon_p,
                                             tPlkFrame* pPlkFrame_p,
                                             size_t sdoCmdDataSize_p);
static tOplkError finishWriteByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError finishReadByIndex(tSdoComCon* pSdoComCon_p,
                                    tPlkFrame* pPlkFrame_p,
                                    size_t sdoCmdDataSize_p);
static tOplkError processResponseWriteByIndex(tSdoComCon* pSdoComCon_p,
                                              const tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processResponseReadByIndex(tSdoComCon* pSdoComCon_p);
static void       updateTransferAfterTx(tSdoComCon* pSdoComCon_p);
static tOplkError obdSearchConnection(tSdoComConHdl sdoObdConHdl_p,
                                      tSdoComCon** pSdoComCon_p);
static tOplkError saveObdConnectionHdl(tSdoComCon* pSdoComCon_p,
                                       tSdoObdAccType sdoAccessType_p);
static void       fillCmdFrameDataSegmInit(tAsySdoCom* pCommandFrame_p,
                                           const void* pSrcData_p,
                                           size_t size_p);
static void       setCmdFrameHdrSegmTtlSize(tAsySdoCom* pCommandFrame_p,
                                            size_t sizeTotal_p);
static tOplkError sendAckResponseFrame(const tSdoComCon* pSdoComCon_p);
static tOplkError sendResponseFrame(tSdoComCon* pSdoComCon_p,
                                    const tPlkFrame* pPlkFrame_p,
                                    size_t sdoCmdDataSize_p);
static tOplkError abortTransfer(tSdoComCon* pSdoComCon_p,
                                UINT32 abortCode_p);
static BOOL       abortReadByIndexIfSizeInvalid(tSdoComCon* pSdoComCon_p,
                                                size_t sdoCmdDataSize_p);
static tOplkError abortMultiTransfer(tSdoComCon* pSdoComCon_p,
                                     const tPlkFrame* pFrame_p,
                                     size_t sdoCmdDataSize_p);
static tOplkError obdFinishCb(tSdoObdConHdl* pObdHdl_p);
static void       setCmdFrameWriteMultiRespAbort(tAsySdoComWriteMultResp* pSubHdr_p,
                                                 UINT16 index_p,
                                                 UINT8 subIndex_p,
                                                 UINT32 abortCode_p);
static tOplkError setCmdFrameReadMultiRespAbort(tAsySdoComMultWriteReqReadResp* pSubHdr_p,
                                                tAsySdoComMultWriteReqReadResp* pLastSubHdr_p,
                                                tAsySdoComMultWriteReqReadResp** ppNextSubHdr_p,
                                                tSdoComCon* pSdoComCon_p,
                                                ptrdiff_t cmdOffset_p,
                                                size_t cmdBufSize_p);
static UINT32     getSdoErrorCode(tOplkError errorCode_p);
static void       assignSdoErrorCode(tOplkError errorCode_p,
                                     UINT32* pSetSdoError_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO server connection

The function initializes the SDO server connection

\param[in,out]  pSdoComCon_p        Command layer connection control handle
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomsrv_initCon(tSdoComCon* pSdoComCon_p,
                             const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError  ret = kErrorOk;

    // check if init of a transfer and no SDO abort
    if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_RESPONSE) == 0)
    {   // SDO request
        if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_ABORT) == 0)
        {   // no SDO abort, save transaction id
            pSdoComCon_p->transactionId = ami_getUint8Le(&pRecvdCmdLayer_p->transactionId);

            switch (pRecvdCmdLayer_p->commandId)
            {
                case kSdoServiceNIL:
                    // simply acknowledge NIL command on sequence layer
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, NULL);
                    break;

                case kSdoServiceReadByIndex:
                    // read by index, search entry and start transfer
                    ret = initReadByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                case kSdoServiceWriteByIndex:
                    // search entry an start write
                    ret = initWriteByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                case kSdoServiceWriteMultiByIndex:
                    // search first entry an start write
                    ret = initWriteMultiByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                case kSdoServiceReadMultiByIndex:
                    // search first entry an start write
                    ret = initReadMultiByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                default:
                    //  unsupported command -> send abort
                    ret = abortTransfer(pSdoComCon_p, SDO_AC_UNKNOWN_COMMAND_SPECIFIER);
                    break;
            }
        }
    }
    else
    {   // this command layer handle is not responsible
        // (wrong direction or wrong transaction ID)
        return kErrorSdoComNotResponsible;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateServerSegmTrans

The function processes the SDO command handler state: kSdoComStateServerSegmTrans

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomsrv_processStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p,
                                                 tSdoComConEvent sdoComConEvent_p,
                                                 const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError      ret = kErrorOk;
    tSdoComCon*     pSdoComCon;
    UINT8           flag;

    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send next frame
        // these events are used for requesting new data from the OD
        // and transferring it to the sequence layer.
        // Practically, kSdoComConEventAckReceived happens only if the sequence
        // layer Tx history buffer is full, and an ack was received.
        case kSdoComConEventFrameSent:
            updateTransferAfterTx(pSdoComCon);
            // no break - fall through is intended
        case kSdoComConEventAckReceived:
            // check if it is a read
            if ((pSdoComCon->sdoServiceType == kSdoServiceReadByIndex) &&
                (pSdoComCon->transferSize != 0))
            {
                // ignore info about last sent segment
                ret = processResponseReadByIndex(pSdoComCon);
            }
            break;

        // process next command layer frame
        // Note: content is only valid for WriteByIndex due to sequence layer
        //       forwarding mechanism
        case kSdoComConEventRec:
            // check if the frame is an SDO response and has the right transaction ID
            flag = ami_getUint8Le(&pRecvdCmdLayer_p->flags);

            if (((flag & SDO_CMDL_FLAG_RESPONSE) == 0) &&
                (ami_getUint8Le(&pRecvdCmdLayer_p->transactionId) == pSdoComCon->transactionId))
            {
                // check if it is an abort
                if ((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    pSdoComCon->transferSize = 0;
                    pSdoComCon->transferredBytes = 0;
                    pSdoComCon->sdoComState = kSdoComStateIdle;
                    pSdoComCon->lastAbortCode = 0;
                    // higher layer will not be informed
                    return ret;
                }

                if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
                    ret = processResponseWriteByIndex(pSdoComCon, pRecvdCmdLayer_p);
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                ret = kErrorSdoComNotResponsible;
            }
            break;

        // connection closed
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventConClosed:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
            break;

        default:
            break;
    }

    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize a WriteByIndex command

The function initializes a WriteByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pSdoCom_p           Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initWriteByIndex(tSdoComCon* pSdoComCon_p,
                                   const tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret = kErrorOk;
    UINT16          index;
    UINT8           subindex;
    size_t          segmPayloadSize = 0;
    const void*     pSrcData;
    tSdoObdConHdl   obdHdl;

    // An init of a write -> variable part of header possible

    // check if expedited or segmented transfer
    if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMINIT)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        index = ami_getUint16Le(&pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE]);
        subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE + 2]);
        pSrcData = &pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE];
        pSdoComCon_p->transferSize = ami_getUint32Le(&pSdoCom_p->aCommandData[0]);
        pSdoComCon_p->transferSize -= (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
        segmPayloadSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        segmPayloadSize -= (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
        // prepare next state here, due to the possibility of a pending transfer
        // and immediately following segments
        pSdoComCon_p->sdoComState = kSdoComStateServerSegmTrans;
    }
    else if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
        subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);
        pSrcData = &pSdoCom_p->aCommandData[SDO_CMDL_HDR_WRITEBYINDEX_SIZE];
        pSdoComCon_p->transferSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        pSdoComCon_p->transferSize -= SDO_CMDL_HDR_WRITEBYINDEX_SIZE;
        segmPayloadSize = pSdoComCon_p->transferSize;
        // next state assigned later, since no following segments are expected
    }
    else
        return ret; // just ignore any other transfer type

    // save service
    pSdoComCon_p->targetIndex = index;
    pSdoComCon_p->targetSubIndex = subindex;
    pSdoComCon_p->sdoServiceType = kSdoServiceWriteByIndex;
    pSdoComCon_p->transferredBytes = 0;
    pSdoComCon_p->pendingTransferSize = segmPayloadSize;

    // forward command data to OD
    obdHdl.index = index;
    obdHdl.subIndex = subindex;
    obdHdl.pSrcData = (void*)pSrcData;
    obdHdl.totalPendSize = (UINT)pSdoComCon_p->transferSize;
    obdHdl.dataSize = (UINT)segmPayloadSize;
    obdHdl.dataOffset = 0;              // first segment
    ret = saveObdConnectionHdl(pSdoComCon_p, kSdoComConObdInitWriteByIndex);
    if (ret != kErrorOk)
        return ret;

    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_g.pfnProcessObdWrite(&obdHdl, obdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
        goto Abort;

    ret = finishInitWriteByIndex(pSdoComCon_p);
    if (ret != kErrorOk)
        goto Abort;

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a ReadByIndex command

The function initializes a ReadByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pSdoCom_p           Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initReadByIndex(tSdoComCon* pSdoComCon_p,
                                  const tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret = kErrorOk;
    UINT16          index;
    UINT8           subindex;
    tSdoObdConHdl   obdHdl;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame = (tPlkFrame*)&aFrame[0];
    size_t          maxReadBuffSize = SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE;

    OPLK_MEMSET(pFrame, 0x00, sizeof(aFrame));

    // get requested object of received read command
    index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
    subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);

    // save service
    pSdoComCon_p->targetIndex = index;
    pSdoComCon_p->targetSubIndex = subindex;
    pSdoComCon_p->sdoServiceType = kSdoServiceReadByIndex;
    pSdoComCon_p->transferredBytes = 0;
    pSdoComCon_p->pendingTransferSize = maxReadBuffSize;
    ret = saveObdConnectionHdl(pSdoComCon_p, kSdoComConObdInitReadByIndex);
    if (ret != kErrorOk)
        return ret;

    // request command data from OD
    OPLK_MEMSET(&obdHdl, 0x00, sizeof(obdHdl));
    obdHdl.index = index;
    obdHdl.subIndex = subindex;
    // shift command payload buffer artificially to the right, and relocate header later if necessary
    obdHdl.pDstData = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[SDO_CMDL_HDR_VAR_SIZE];
    obdHdl.dataSize = (UINT)maxReadBuffSize;
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_g.pfnProcessObdRead(&obdHdl, obdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    // save service - init read by index
    pSdoComCon_p->transferSize = obdHdl.totalPendSize; // from function call
    if (pSdoComCon_p->transferSize <= maxReadBuffSize)
    {   // expedited transfer -> relocate frame header

        // SDO command payload in buffer set to variable header location,
        // therefore relocate frame header to align it to the payload
        pFrame = (tPlkFrame*)((UINT8*)pFrame + SDO_CMDL_HDR_VAR_SIZE);
    }

    if (pSdoComCon_p->lastAbortCode != 0)
        goto Abort;

    ret = finishInitReadByIndex(pSdoComCon_p,
                                pFrame,
                                obdHdl.dataSize);
    if (ret != kErrorOk)
        goto Abort;

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a WriteMultiParam command

The function initializes a Write Multiple Parameter by Index SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pSdoCom_p           Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initWriteMultiByIndex(tSdoComCon* pSdoComCon_p,
                                        const tAsySdoCom* pSdoCom_p)
{
    tOplkError                      ret = kErrorOk;
    UINT16                          index;
    UINT8                           subindex;
    void*                           pSrcData;
    tAsySdoComMultWriteReqReadResp* pCurSubHdr;
    tAsySdoComMultWriteReqReadResp* pNextSubHdr;
    tSdoObdConHdl                   obdHdl;
    BOOL                            fLastDataSetReached = FALSE;

    // allocate and initialize command layer response frame
    UINT8                       aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*                  pRespFrame = (tPlkFrame*)&aFrame[0];
    tAsySdoComWriteMultResp*    pRespSubFrame = (tAsySdoComWriteMultResp*)
                                                &pRespFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[0];
    size_t                      respCmdDataSize = 0;

    OPLK_MEMSET(pRespFrame, 0x00, sizeof(aFrame));

    // An init of a write -> variable part of header possible

    // check if expedited or segmented transfer
    if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMINIT)
    {
        // segmented access is not allowed
        return kErrorSdoComInvalidServiceType;
    }
    else if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        // for this command type: command layer non-fixed part size
        pSdoComCon_p->reqSegmSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        // init first subheader
        pCurSubHdr = (tAsySdoComMultWriteReqReadResp*)&pSdoCom_p->aCommandData[0];

        // next state assigned later, since no following segments are expected
    }
    else
        return ret; // just ignore any other transfer type

    pSdoComCon_p->sdoServiceType = kSdoServiceWriteMultiByIndex;
    pSdoComCon_p->transferredBytes = 0;

    while (!fLastDataSetReached)
    {
        index = ami_getUint16Le(&pCurSubHdr->index);
        subindex = ami_getUint8Le(&pCurSubHdr->subIndex);
        pSrcData = &pCurSubHdr->aCommandData[0];
        pNextSubHdr = (tAsySdoComMultWriteReqReadResp*)((UINT8*)pSdoCom_p + pCurSubHdr->byteOffsetNext);

        // save service
        pSdoComCon_p->targetIndex = index;
        pSdoComCon_p->targetSubIndex = subindex;

        if (pCurSubHdr->byteOffsetNext != 0)
        {
            pSdoComCon_p->pendingTransferSize = (UINT8*)pNextSubHdr - (UINT8*)pCurSubHdr - \
                                                 SDO_CMDL_HDR_WRITEMULTBYINDEX_SIZE - \
                                                 (pCurSubHdr->info & SDO_CMDL_FLAG_PADSIZE_MASK);
        }
        else
        {
            // it is the last data set thus another calculation method is used
            pSdoComCon_p->pendingTransferSize = (UINT8*)&pSdoCom_p->aCommandData[0] + pSdoComCon_p->reqSegmSize - \
                                                ((UINT8*)pCurSubHdr + SDO_CMDL_HDR_WRITEMULTBYINDEX_SIZE) - \
                                                (pCurSubHdr->info & SDO_CMDL_FLAG_PADSIZE_MASK);
        }

        // forward command data to OD
        obdHdl.index = index;
        obdHdl.subIndex = subindex;
        obdHdl.pSrcData = pSrcData;
        obdHdl.totalPendSize = (UINT)pSdoComCon_p->pendingTransferSize; // only current data set size
        obdHdl.dataSize = (UINT)pSdoComCon_p->pendingTransferSize;
        obdHdl.dataOffset = 0;                                          // first segment

        // deferred answer not supported for this transfer type -> callback is NULL
        ret = sdoComInstance_g.pfnProcessObdWrite(&obdHdl, NULL);
        pSdoComCon_p->lastAbortCode = 0;
        assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);

        if (pSdoComCon_p->lastAbortCode != 0)
        {
            // prepare sub-abort response
            setCmdFrameWriteMultiRespAbort(pRespSubFrame,
                                           index,
                                           subindex,
                                           pSdoComCon_p->lastAbortCode);
            pSdoComCon_p->lastAbortCode = 0;  // reset abort code
            respCmdDataSize += sizeof(tAsySdoComWriteMultResp);
            if ((respCmdDataSize + sizeof(tAsySdoComWriteMultResp)) > SDO_CMD_SEGM_TX_MAX_SIZE)
            {   // too many entries for next abort
                // send abort now and ignore following objects
                pSdoComCon_p->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
                ret = abortMultiTransfer(pSdoComCon_p,
                                         pRespFrame,
                                         respCmdDataSize);
                return ret;
            }

            // set pointer to next sub-frame
            pRespSubFrame++;
        }

        if (pCurSubHdr->byteOffsetNext != 0)
        {   // increment to next object
            pCurSubHdr = pNextSubHdr;

            if ((UINT8*)pNextSubHdr > ((UINT8*)&pSdoCom_p->aCommandData[0] + pSdoComCon_p->reqSegmSize))
            {
                // frame overrun, bad request frame!
                abortTransfer(pSdoComCon_p, SDO_AC_INVALID_BLOCK_SIZE);
                return ret;
            }
        }
        else
        {
            // this was the last object -> exit loop
            fLastDataSetReached = TRUE;
        }
    }

    if (respCmdDataSize > 0)
    {
        // at least one abort code was prepared
        ret = abortMultiTransfer(pSdoComCon_p,
                                 pRespFrame,
                                 respCmdDataSize);
        if (ret != kErrorOk)
        {
            abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
            return ret;
        }
    }
    else
    {
        // finish successfully
        ret = finishInitWriteMultiByIndex(pSdoComCon_p);
        if (ret != kErrorOk)
        {
            abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
            return ret;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a ReadMultiParam command

The function initializes a Read Multiple Parameter by Index SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pSdoCom_p           Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initReadMultiByIndex(tSdoComCon* pSdoComCon_p,
                                       const tAsySdoCom* pSdoCom_p)
{
    tOplkError                  ret = kErrorOk;
    tAsySdoComReadMultReq*      pReqSubHdr;
    UINT32                      reqCmdOffset = 0;

    // allocate and initialize command layer response frame
    UINT8                           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*                      pRespFrame = (tPlkFrame*)&aFrame[0];
    tAsySdoComMultWriteReqReadResp* pRespSubFrame;
    tAsySdoComMultWriteReqReadResp* pRespLastSubFrame = NULL;
    tAsySdoComMultWriteReqReadResp* pNextRespSubFrame = NULL;
    ptrdiff_t                       respCmdSubFrameOffset = 0;
    size_t                          respCmdDataBufSize = 0;

    OPLK_MEMSET(pRespFrame, 0x00, sizeof(aFrame));

    pRespSubFrame = (tAsySdoComMultWriteReqReadResp*)
                    &pRespFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[0];

    // check if expedited or segmented transfer
    if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMINIT)
    {
        // segmentation is not supported
        return kErrorSdoComInvalidServiceType;
    }
    else if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        // for this command type: command layer non-fixed part size
        pSdoComCon_p->reqSegmSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        // init first subheader
        pReqSubHdr = (tAsySdoComReadMultReq*)&pSdoCom_p->aCommandData[0];

        // next state assigned later, since no following segments are expected
    }
    else
        return ret; // just ignore any other transfer type

    pSdoComCon_p->sdoServiceType = kSdoServiceReadMultiByIndex;
    pSdoComCon_p->transferredBytes = 0;
    pSdoComCon_p->respSegmSize = SDO_CMDL_HDR_FIXED_SIZE;

    // loop over segment size and process all object requests
    while ((reqCmdOffset < pSdoComCon_p->reqSegmSize) &&
           (reqCmdOffset < (C_DLL_MAX_ASYNC_MTU - 58)))
    {
        respCmdSubFrameOffset = (UINT8*)pRespSubFrame - (UINT8*)&pRespFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[0];
        respCmdDataBufSize = SDO_MAX_TX_FRAME_SIZE - respCmdSubFrameOffset;

        if (respCmdSubFrameOffset <= SDO_CMD_SEGM_TX_MAX_SIZE)
        {
            // get requested object of received read command
            pSdoComCon_p->targetIndex = ami_getUint16Le(&pReqSubHdr->index);
            pSdoComCon_p->targetSubIndex = ami_getUint8Le(&pReqSubHdr->subIndex);

            ret = setCmdFrameReadMultiRespAbort(pRespSubFrame,
                                                pRespLastSubFrame,
                                                &pNextRespSubFrame,
                                                pSdoComCon_p,
                                                respCmdSubFrameOffset,
                                                respCmdDataBufSize);
            if (ret != kErrorOk)
            {
                // too many entries for next data or abort
                // send abort now and ignore following objects
                ret = abortMultiTransfer(pSdoComCon_p,
                                         pRespFrame,
                                         pSdoComCon_p->respSegmSize);
                return ret;
            }
        }
        else
        {
            // too many entries for next data or abort
            // send abort now and ignore following objects
            ret = abortMultiTransfer(pSdoComCon_p,
                                     pRespFrame,
                                     pSdoComCon_p->respSegmSize);
            return ret;
        }

        // increment for next iteration
        // Request frame
        pReqSubHdr++;
        reqCmdOffset += sizeof(tAsySdoComReadMultReq);
        // Response frame
        pRespLastSubFrame = pRespSubFrame;
        pRespSubFrame = pNextRespSubFrame;
    }

    ret = finishInitReadMultiByIndex(pSdoComCon_p,
                                     pRespFrame,
                                     pSdoComCon_p->respSegmSize);
    if (ret != kErrorOk)
    {
        abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        return ret;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial WriteByIndex command

The function completes an initial WriteByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishInitWriteByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError  ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {
        // expedited transfer finished, send command acknowledge
        ret = sendAckResponseFrame(pSdoComCon_p);
        pSdoComCon_p->transferSize = 0;

        // prepare next state
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }
    else
    {
        // segmented transfer started
        // Note: State change to kSdoComStateServerSegmTrans already done earlier
        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, FALSE);

        // send acknowledge without any command layer data  (sequence layer ack)
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, NULL);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial ReadByIndex command

The function completes an initial ReadByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in,out]  pPlkFrame_p         Pointer to PLK frame with SDO command layer data and
                                    size sdoCmdDataSize_p (optional).
                                    If not used, little endian command layer data
                                    will be copied from pSdoComCon_p->pData.
\param[in]      sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishInitReadByIndex(tSdoComCon* pSdoComCon_p,
                                        tPlkFrame* pPlkFrame_p,
                                        size_t sdoCmdDataSize_p)
{
    tOplkError  ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    // decide if segmented or expedited transfer

    // NOTE: maximum Tx buffer size was stored in pSdoComCon_p->pendingTransferSize
    // It is known by this module, that the send function also provides
    // the same maximum size for initial segmented read transfers, therefore
    // this decision can be taken here.
    if (pSdoComCon_p->transferSize > pSdoComCon_p->pendingTransferSize)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        pSdoComCon_p->sdoComState = kSdoComStateServerSegmTrans;
    }
    else
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
    }

    if (abortReadByIndexIfSizeInvalid(pSdoComCon_p, sdoCmdDataSize_p))
    {
        ret = kErrorObdValueLengthError;
        goto Exit;
    }

    ret = sendResponseFrame(pSdoComCon_p,
                            pPlkFrame_p,
                            sdoCmdDataSize_p);
    if ((ret != kErrorOk) &&
        (ret != kErrorSdoSeqConnectionBusy))
    {
        ret = abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        goto Exit;
    }

    // check next state
    if (pSdoComCon_p->transferSize == 0)
    {
        // already finished -> stay idle
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial WriteMultiParam command

The function completes an initial Write Multiple Parameters by Index SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishInitWriteMultiByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError  ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {
        // expedited transfer finished, send command acknowledge
        ret = sendAckResponseFrame(pSdoComCon_p);
        sdocomint_updateHdlTransfSize(pSdoComCon_p, 0, TRUE);

        // prepare next state
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }
    else
    {
        // segmented transfer not allowed
        return kErrorSdoComInvalidServiceType;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial ReadMultiParam command

The function completes an initial Read Multiple Parameters by Index SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pPlkFrame_p         Pointer to PLK frame with SDO command layer data and
                                    size sdoCmdDataSize_p (optional).
                                    If not used, little endian command layer data
                                    will be copied from pSdoComCon_p->pData.
\param[in]      sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishInitReadMultiByIndex(tSdoComCon* pSdoComCon_p,
                                             tPlkFrame* pPlkFrame_p,
                                             size_t sdoCmdDataSize_p)
{
    // this type of transfer has to fit in a single frame
    pSdoComCon_p->sdoTransferType = kSdoTransExpedited;

    // same as abort: passing on a prepared multi-response frame
    return abortMultiTransfer(pSdoComCon_p,
                              pPlkFrame_p,
                              sdoCmdDataSize_p);
}

//------------------------------------------------------------------------------
/**
\brief  Finish a segmented WriteByIndex command

The function completes a non-initial segmented WriteByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishWriteByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError  ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (pSdoComCon_p->transferSize != 0)
    {
        // not yet the last segment
        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, FALSE);

        // send acknowledge without any Command layer data (sequence layer ack)
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
    }
    else
    {
        // transfer finished
        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, TRUE);

        if (pSdoComCon_p->lastAbortCode == 0)
        {
            // send empty response indicating completed transfer
            sendResponseFrame(pSdoComCon_p, NULL, 0);
            // if all send -> back to idle
            if (pSdoComCon_p->transferSize == 0)
            {   // back to idle
                pSdoComCon_p->sdoComState = kSdoComStateIdle;
                pSdoComCon_p->lastAbortCode = 0;
            }
            else
            {
                ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
                pSdoComCon_p->lastAbortCode = 0;
            }
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish a segmented ReadByIndex command

The function completes a non-initial segmented ReadByIndex SDO command.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in,out]  pPlkFrame_p         Pointer to PLK frame with SDO command layer data and
                                    size sdoCmdDataSize_p (optional).
                                    If not used, little endian command layer data
                                    will be copied from pSdoComCon_p->pData.
\param[in]      sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError finishReadByIndex(tSdoComCon* pSdoComCon_p,
                                    tPlkFrame* pPlkFrame_p,
                                    size_t sdoCmdDataSize_p)
{
    tOplkError  ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (abortReadByIndexIfSizeInvalid(pSdoComCon_p, sdoCmdDataSize_p))
        goto Exit;

    ret = sendResponseFrame(pSdoComCon_p,
                            pPlkFrame_p,
                            sdoCmdDataSize_p);
    if ((ret != kErrorOk) &&
        (ret != kErrorSdoSeqConnectionBusy))
    {
        ret = abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        goto Exit;
    }

    // if all send -> back to idle
    if (pSdoComCon_p->transferSize == 0)
    {   // back to idle
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes the WriteByIndex response to a received frame

The function processes a WriteByIndex command layer response to a received
frame.

\param[in,out]  pSdoComCon_p        Command layer connection control handle
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processResponseWriteByIndex(tSdoComCon* pSdoComCon_p,
                                              const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError      ret = kErrorOk;
    size_t          size;
    tSdoObdConHdl   obdHdl;

    size = ami_getUint16Le(&pRecvdCmdLayer_p->segmentSizeLe);
    if (size > pSdoComCon_p->transferSize)
    {
        pSdoComCon_p->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        return ret;
    }

    // save size for handle update
    pSdoComCon_p->pendingTransferSize = size;

    // forward command data to OD
    obdHdl.index = pSdoComCon_p->targetIndex;
    obdHdl.subIndex = pSdoComCon_p->targetSubIndex;
    obdHdl.pSrcData = (void*)&(pRecvdCmdLayer_p->aCommandData[0]);
    obdHdl.totalPendSize = (UINT)pSdoComCon_p->transferSize;
    obdHdl.dataSize = (UINT)size;
    obdHdl.dataOffset = (UINT)pSdoComCon_p->transferredBytes;
    // check end of transfer before forwarding to object dictionary
    if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMCOMPL)
    {
        // transfer finished
        pSdoComCon_p->transferSize = 0;
    }

    ret = saveObdConnectionHdl(pSdoComCon_p, kSdoComConObdWriteByIndex);
    if (ret != kErrorOk)
        return ret;

    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_g.pfnProcessObdWrite(&obdHdl, obdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {
        // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
        goto Abort;

    ret = finishWriteByIndex(pSdoComCon_p);
    if (ret != kErrorOk)
        goto Abort;

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes the ReadByIndex response to a received frame

The function processes a ReadByIndex command layer response to a received frame.

\param[in,out]  pSdoComCon_p        Command layer connection control handle

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processResponseReadByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame = (tPlkFrame*)&aFrame[0];
    size_t          maxReadBuffSize = SDO_CMD_SEGM_TX_MAX_SIZE;
    tSdoObdConHdl   obdHdl;

    OPLK_MEMSET(pFrame, 0x00, sizeof(aFrame));

    // send next frame
    // request command data from OD
    ret = saveObdConnectionHdl(pSdoComCon_p, kSdoComConObdReadByIndex);
    if (ret != kErrorOk)
        return ret;

    OPLK_MEMSET(&obdHdl, 0x00, sizeof(obdHdl));
    obdHdl.index = pSdoComCon_p->targetIndex;
    obdHdl.subIndex = pSdoComCon_p->targetSubIndex;
    obdHdl.pDstData = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[0];
    obdHdl.dataSize = (UINT)maxReadBuffSize;
    obdHdl.dataOffset = (UINT)pSdoComCon_p->transferredBytes;
    obdHdl.totalPendSize = (UINT)pSdoComCon_p->transferSize;
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_g.pfnProcessObdRead(&obdHdl, obdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {
        // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
        goto Abort;

    ret = finishReadByIndex(pSdoComCon_p,
                            pFrame,
                            obdHdl.dataSize);
    if (ret != kErrorOk)
        goto Abort;

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update the SDO command layer connection structure size fields after Tx

The function updates the SDO command layer connection structure size related
members. It has to be called after a successful send (or storage in the history buffer)
of command layer data sent by the server. For certain transfer types
pSdoComCon_p->pendingTxBytes which was set before the actual transmission
happened, is used to update the connection structure.

\param[in,out]  pSdoComCon_p    Pointer to SDO command layer connection structure

*/
//------------------------------------------------------------------------------
static void updateTransferAfterTx(tSdoComCon* pSdoComCon_p)
{
    if ((pSdoComCon_p->transferSize > 0) &&
        (pSdoComCon_p->sdoServiceType == kSdoServiceReadByIndex))
    {
        switch (pSdoComCon_p->sdoTransferType)
        {
            case kSdoTransExpedited:
                sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->transferSize, TRUE);
                break;

            case kSdoTransSegmented:
                if ((pSdoComCon_p->transferredBytes > 0) &&
                    (pSdoComCon_p->transferSize <= SDO_CMD_SEGM_TX_MAX_SIZE))
                {
                    // last segment
                    sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTxBytes, TRUE);
                }
                else
                {
                    // first and middle segments
                    sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTxBytes, FALSE);
                    pSdoComCon_p->pendingTxBytes = 0;
                }
                break;

            default:
                break;
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Search SDO connection handle by OD handle number

The function searches for SDO connection control structures by using a handle
number previously provided to an external (object dictionary) module.

\param[in]      sdoObdConHdl_p      Previously provided connection handle number
\param[out]     ppSdoComCon_p       Pointer to found SDO control structure,
                                    NULL if not found (call by reference).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError obdSearchConnection(tSdoComConHdl sdoObdConHdl_p,
                                      tSdoComCon** ppSdoComCon_p)
{
    tSdoComCon*     pSdoComCon;
    tSdoComConHdl   hdlCount;

    // get pointer to first element of the array
    pSdoComCon = &sdoComInstance_g.sdoComCon[0];
    hdlCount = 0;

    // get pointer to control structure of connection
    while (hdlCount < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoObdConHdl == sdoObdConHdl_p)
        {   // matching command layer handle found
            if (pSdoComCon->sdoSeqConHdl == 0)
                return kErrorSdoComInvalidHandle;

            *ppSdoComCon_p = pSdoComCon;
            return kErrorOk;
        }
        pSdoComCon++;
        hdlCount++;
    }

    *ppSdoComCon_p = NULL;
    return kErrorSdoComInvalidHandle;
}

//------------------------------------------------------------------------------
/**
\brief  Generate and save a new connection handle for OD access

The function generates a new connection number and stores it to the command
layer control structure. If this was successful, pSdoComCon_p->sdoObdConHdl
can be forwarded as handle to the OD. The OD has to provide this handle
together with a delayed answer.

\param[in,out]  pSdoComCon_p        pointer SDO control structure
\param[in]      sdoAccessType_p     SDO server object dictionary access type

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError saveObdConnectionHdl(tSdoComCon* pSdoComCon_p,
                                       tSdoObdAccType sdoAccessType_p)
{
    if (pSdoComCon_p->sdoObdConHdl == 0)
    {
        // set new handle
        if (++sdoComInstance_g.sdoObdConCounter == 0)
            ++sdoComInstance_g.sdoObdConCounter;

        pSdoComCon_p->sdoObdConHdl = sdoComInstance_g.sdoObdConCounter;
        pSdoComCon_p->sdoObdAccType = sdoAccessType_p;
        return kErrorOk;
    }
    else
    {
        // OD connection is currently in use for this command layer connection
        return kErrorSdoComHandleBusy;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Copy data to an initial segmented SDO command layer frame

The function copies data to the payload section of an SDO command layer frame
for initial segmented transfer frames.

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      pSrcData_p          Pointer to data source start
\param[in]      size_p              Size of data to copy
*/
//------------------------------------------------------------------------------
static void fillCmdFrameDataSegmInit(tAsySdoCom* pCommandFrame_p,
                                     const void* pSrcData_p,
                                     size_t size_p)
{
    OPLK_MEMCPY(&pCommandFrame_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE],
                pSrcData_p,
                size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set the total transfer size for an initial segmented transfer

The function sets the total transfer size for an initial segmented transfer in
the variable header of a command layer frame.

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      sizeTotal_p         Total transfer size
*/
//------------------------------------------------------------------------------
static void setCmdFrameHdrSegmTtlSize(tAsySdoCom* pCommandFrame_p,
                                      size_t sizeTotal_p)
{
    // init data size in variable header, which includes itself
    ami_setUint32Le(&pCommandFrame_p->aCommandData[0], (UINT32)sizeTotal_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send an empty SDO command layer ack frame from SDO server

The function creates and sends a command layer acknowledge frame from an
 SDO server. This frame is without command layer data.

\param[in]      pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendAckResponseFrame(const tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    size_t          sizeOfCmdFrame;

    pFrame = (tPlkFrame*)&aFrame[0];
    sdocomint_initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    sdocomint_overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_RESPONSE);
    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE;
    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an SDO command layer response frame from an SDO server

The function creates and sends a command layer frame from an SDO server.
It can either copy data directly from pSdoComCon_p->data into an empty frame,
or if provided, only set the command layer header around existing command
layer payload in pPlkFrame_p.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in,out]  pPlkFrame_p         Pointer to PLK frame with SDO command layer data.
                                    If not used (NULL), little endian command layer data
                                    will be copied from pSdoComCon_p->pData.
\param[in]      sdoCmdDataSize_p    Size of command layer data (without any header)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendResponseFrame(tSdoComCon* pSdoComCon_p,
                                    const tPlkFrame* pPlkFrame_p,
                                    size_t sdoCmdDataSize_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    size_t          sizeOfCmdFrame = 0;

    if (pPlkFrame_p == NULL)
    {   // only pointer to frame with little endian command layer data is provided
        // by caller -> this function will do the copy operation, and provide the buffer
        pFrame = (tPlkFrame*)&aFrame[0];
        sdocomint_initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);
    }
    else
    {   // frame with command layer data is provided by caller
        pFrame = (tPlkFrame*)pPlkFrame_p;
        // build generic part of command frame
        pCommandFrame = (tAsySdoCom*)&pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
        ami_setUint8Le(&pCommandFrame->commandId, pSdoComCon_p->sdoServiceType);
        ami_setUint8Le(&pCommandFrame->transactionId, pSdoComCon_p->transactionId);
    }

    sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_RESPONSE);

    // setup command layer
    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {
        // Expedited transfer
        // copy data into frame, if frame not provided by caller
        if (pPlkFrame_p == NULL)
            sdocomint_fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sdoCmdDataSize_p);

        sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sdoCmdDataSize_p);
        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sdoCmdDataSize_p;
    }
    else if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
    {
        // segmented transfer
        // distinguish between init, segment and complete
        if (pSdoComCon_p->transferredBytes == 0)
        {   // init
            if (sdoCmdDataSize_p > SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE)
            {
                ret = kErrorSdoSeqFrameSizeError;
                goto Exit;
            }

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
                fillCmdFrameDataSegmInit(pCommandFrame, pSdoComCon_p->pData, sdoCmdDataSize_p);

            sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMINIT);
            sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sdoCmdDataSize_p + SDO_CMDL_HDR_VAR_SIZE);
            setCmdFrameHdrSegmTtlSize(pCommandFrame, pSdoComCon_p->transferSize + SDO_CMDL_HDR_VAR_SIZE);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMDL_HDR_VAR_SIZE + sdoCmdDataSize_p;
            pSdoComCon_p->pendingTxBytes = sdoCmdDataSize_p;
        }
        else if ((pSdoComCon_p->transferredBytes > 0) && (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE))
        {
            // segment
            if (sdoCmdDataSize_p > SDO_CMD_SEGM_TX_MAX_SIZE)
            {
                ret = kErrorSdoSeqFrameSizeError;
                goto Exit;
            }

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
                sdocomint_fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sdoCmdDataSize_p);

            sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMENTED);
            sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sdoCmdDataSize_p);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sdoCmdDataSize_p;
            pSdoComCon_p->pendingTxBytes = sdoCmdDataSize_p;
        }
        else
        {
            // complete
            // block sending empty frames from other transfer types than kSdoServiceWriteByIndex
            if ((pSdoComCon_p->transferSize == 0) && (pSdoComCon_p->sdoServiceType != kSdoServiceWriteByIndex))
                return ret;

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
                sdocomint_fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sdoCmdDataSize_p);

            if (pSdoComCon_p->sdoServiceType == kSdoServiceReadByIndex)
                sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMCOMPL);

            sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sdoCmdDataSize_p);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sdoCmdDataSize_p;
            pSdoComCon_p->pendingTxBytes = sdoCmdDataSize_p;
        }
    }

    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort an SDO transfer

The function sends an abort message, and causes a state change back to the
initial SDO server state.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      abortCode_p         Abort code to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError abortTransfer(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    size_t          sizeOfCmdFrame;
    size_t          sizeOfCmdData;

    pFrame = (tPlkFrame*)&aFrame[0];
    sdocomint_initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    sizeOfCmdData = sizeof(abortCode_p);
    // copy abort code to frame
    ami_setUint32Le(&pCommandFrame->aCommandData[0], abortCode_p);
    sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_RESPONSE | SDO_CMDL_FLAG_ABORT);
    sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

    sdocomint_updateHdlTransfSize(pSdoComCon_p, 0, TRUE);
    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
    DEBUG_LVL_SDO_TRACE("ERROR: SDO Aborted!\n");

    pSdoComCon_p->sdoComState = kSdoComStateIdle;
    // invalidate connection to OD, so a delayed response from OD recognizes
    // that there is no connection present
    pSdoComCon_p->sdoObdConHdl = 0;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort an SDO Read by Index SDO transfer if size is invalid

The function checks the due command data response size for integrity depending
on the transfer type and aborts the transfer if it is invalid.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      sdoCmdDataSize_p    Size of command data to be checked

\return The function returns TRUE if transfer was aborted and FALSE if not.
*/
//------------------------------------------------------------------------------
static BOOL abortReadByIndexIfSizeInvalid(tSdoComCon* pSdoComCon_p,
                                          size_t sdoCmdDataSize_p)
{
    if ((pSdoComCon_p->sdoTransferType == kSdoTransExpedited) ||
        ((pSdoComCon_p->sdoTransferType == kSdoTransSegmented) &&
         ((pSdoComCon_p->transferredBytes > 0) &&
          (pSdoComCon_p->transferSize <= SDO_CMD_SEGM_TX_MAX_SIZE))))
    {   // expedited or last segment
        if (sdoCmdDataSize_p != pSdoComCon_p->transferSize)
        {
            abortTransfer(pSdoComCon_p, getSdoErrorCode(kErrorSdoSeqFrameSizeError));
            return TRUE;
        }
    }
    else if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
    {
        if (sdoCmdDataSize_p > pSdoComCon_p->transferSize)
        {
            abortTransfer(pSdoComCon_p, SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH);
            return TRUE;
        }
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Abort a WriteMultiParam command

The function sends an abort message, and causes a state change back to the
initial SDO server state.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      pFrame_p            Pointer to Eth frame containing an SDO command layer frame
\param[in]      sdoCmdDataSize_p    Size of command layer data (without any header)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError abortMultiTransfer(tSdoComCon* pSdoComCon_p,
                                     const tPlkFrame* pFrame_p,
                                     size_t sdoCmdDataSize_p)
{
    tOplkError      ret = kErrorOk;

    if (pSdoComCon_p->respSegmSize > SDO_CMD_SEGM_TX_MAX_SIZE)
    {
        // too many entries
        pSdoComCon_p->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = abortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
    }
    else
    {
        ret = sendResponseFrame(pSdoComCon_p,
                                pFrame_p,
                                sdoCmdDataSize_p);
        if (ret != kErrorOk)
        {
            ret = abortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        }
    }

    sdocomint_updateHdlTransfSize(pSdoComCon_p, 0, TRUE);
    pSdoComCon_p->sdoComState = kSdoComStateIdle;
    pSdoComCon_p->lastAbortCode = 0;

    // invalidate connection to OD, so a delayed response from OD recognizes
    // that there is no connection present
    pSdoComCon_p->sdoObdConHdl = 0;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Object dictionary finishes an SDO server access

The function completes an SDO server access and will be called by the object
dictionary.

\parblock
\param[in,out]  pObdHdl_p           Connection handle to SDO command layer. Used members:
                                    \li tSdoObdConHdl::sdoHdl
                                    \li tSdoObdConHdl::plkError

                                    Only used for read access (status saved in local instance):
                                    \li tSdoObdConHdl::pSrcData
                                    \li tSdoObdConHdl::dataSize      Data size to be copied

                                    Only used for initial read or write access
                                    (status saved in local instance):
                                    \li tSdoObdConHdl::totalPendSize object size
\endparblock

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError obdFinishCb(tSdoObdConHdl* pObdHdl_p)
{
    tOplkError      ret = kErrorOk;
    tSdoComCon*     pSdoComCon = NULL;

    ret = obdSearchConnection(pObdHdl_p->sdoHdl, &pSdoComCon);
    if (ret != kErrorOk)
    {
        // overwrite potentially existing error
        pObdHdl_p->plkError = ret;
        if (pSdoComCon == NULL)
        {   // no handle found, exit immediately
            goto Exit;
        }
    }

    if (pObdHdl_p->plkError != kErrorOk)
    {
        assignSdoErrorCode(pObdHdl_p->plkError, &pSdoComCon->lastAbortCode);
        abortTransfer(pSdoComCon, pSdoComCon->lastAbortCode);
        ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
        OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
        goto Exit;
    }

    switch (pSdoComCon->sdoObdAccType)
    {
        case kSdoComConObdInitReadByIndex:
            if (pObdHdl_p->pSrcData == NULL)
            {
                ret = abortTransfer(pSdoComCon, SDO_AC_DATA_NOT_TRANSF_OR_STORED);
                goto Exit;
            }

            // start of command layer data (in little endian format)
            // to be copied by SDO server
            pSdoComCon->pData = pObdHdl_p->pSrcData;
            // save service - init read by index
            pSdoComCon->transferSize = pObdHdl_p->totalPendSize;
            ret = finishInitReadByIndex(pSdoComCon, NULL, pObdHdl_p->dataSize);
            break;

        case kSdoComConObdReadByIndex:
            if (pObdHdl_p->pSrcData == NULL)
            {
                ret = abortTransfer(pSdoComCon, SDO_AC_DATA_NOT_TRANSF_OR_STORED);
                goto Exit;
            }

            // start of command layer data to be copied by SDO server
            pSdoComCon->pData = pObdHdl_p->pSrcData;
            ret = finishReadByIndex(pSdoComCon, NULL, pObdHdl_p->dataSize);
            break;

        case kSdoComConObdInitWriteByIndex:
            // save service - init write by index
            pSdoComCon->transferSize = pObdHdl_p->totalPendSize;
            ret = finishInitWriteByIndex(pSdoComCon);
            break;

        case kSdoComConObdWriteByIndex:
            ret = finishWriteByIndex(pSdoComCon);
            break;

        default:
            ret = kErrorSdoComInvalidHandle;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set abort code in a WriteMultiResp sub-header

The function sets the abort code and abort flag in a sub-header of a
Write Multiple Parameter by Index response frame.

\param[in,out]  pSubHdr_p   Pointer to start of sub-header of a WriteMultiResp
\param[in]      index_p     Index of faulty access
\param[in]      subIndex_p  Subindex of faulty access
\param[in]      abortCode_p SDO abort code
*/
//------------------------------------------------------------------------------
static void setCmdFrameWriteMultiRespAbort(tAsySdoComWriteMultResp* pSubHdr_p,
                                           UINT16 index_p,
                                           UINT8 subIndex_p,
                                           UINT32 abortCode_p)
{
    ami_setUint16Le(&pSubHdr_p->index, index_p);
    ami_setUint8Le(&pSubHdr_p->subIndex, subIndex_p);
    ami_setUint32Le(&pSubHdr_p->subAbortCode, abortCode_p);
    pSubHdr_p->abortFlag = 0x80;
    DEBUG_LVL_SDO_TRACE("ERROR: SDO Aborted!\n");
}

//------------------------------------------------------------------------------
/**
\brief  Set data or abort code in a ReadMultiResp sub-header

The function sets command layer data or an abort code plus abort flag in a sub-header of a
Read Multiple Parameter by Index response frame.

\param[in,out]  pSubHdr_p       Pointer to start of sub-header of a ReadMultiResp
\param[in,out]  pLastSubHdr_p   Pointer to start of previous sub-header of a ReadMultiResp
                                If pLastSubHdr_p = NULL, it is the first sub-response.

\param[out]     ppNextSubHdr_p  sub-header for next object response
\param[in,out]  pSdoComCon_p    Pointer to SDO command layer connection structure.
                                in : respSegmSize, targetIndex, targetSubIndex, lastAbortCode
                                out: respSegmSize (updated)
\param[in]      cmdOffset_p     Offset counting from start of command layer payload
\param[in]      cmdBufSize_p    Size of payload buffer for object data storage

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setCmdFrameReadMultiRespAbort(tAsySdoComMultWriteReqReadResp* pSubHdr_p,
                                                tAsySdoComMultWriteReqReadResp* pLastSubHdr_p,
                                                tAsySdoComMultWriteReqReadResp** ppNextSubHdr_p,
                                                tSdoComCon* pSdoComCon_p,
                                                ptrdiff_t cmdOffset_p,
                                                size_t cmdBufSize_p)
{
    tOplkError      ret = kErrorOk;
    size_t          subPayloadSize;
    size_t          subPayloadBufSize = cmdBufSize_p - 8;   //TODO: Avoid the magic number 8 in this function!
    size_t          nextAlignedOffset = 0;                  // offset from old to next payload start
    tSdoObdConHdl   obdHdl;

    // request command data from OD
    OPLK_MEMSET(&obdHdl, 0x00, sizeof(obdHdl));
    obdHdl.index = pSdoComCon_p->targetIndex;
    obdHdl.subIndex = pSdoComCon_p->targetSubIndex;
    obdHdl.pDstData = &pSubHdr_p->aCommandData[0];
    obdHdl.dataSize = (UINT)subPayloadBufSize;

    // deferred answer not supported for this transfer type -> callback is NULL
    ret = sdoComInstance_g.pfnProcessObdRead(&obdHdl, NULL);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);

    if ((pSdoComCon_p->lastAbortCode == 0) &&
        ((obdHdl.totalPendSize > subPayloadBufSize) ||
        (obdHdl.totalPendSize != obdHdl.dataSize)))
    {   // response does not fit into a single frame (expedited transfer)
        pSdoComCon_p->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        DEBUG_LVL_SDO_TRACE("ERROR: SDO Aborted!\n");

        // instead of data we send abort
        subPayloadSize = sizeof(pSdoComCon_p->lastAbortCode);
        if (subPayloadSize > subPayloadBufSize)
        {
            // not even the abort code fits into the sub-access
            // => too many entries in buffer
            DEBUG_LVL_ERROR_TRACE("%s() Subpayload size exceeds buffer!\n", __func__);
            return kErrorNoResource; //TODO: Use other error code for that!
        }
        else
        {
            // overwrite any previously prepared data with abort code
            ami_setUint32Le(&pSubHdr_p->aCommandData[0], pSdoComCon_p->lastAbortCode);
            pSubHdr_p->info = 0x80; // set MSB abort flag, no padding
            // reset abort code for following sub-accesses
            pSdoComCon_p->lastAbortCode = 0;
        }
    }
    else
    {
        subPayloadSize = obdHdl.dataSize;
        // 2 LSB padding byte info, no abort
        pSubHdr_p->info = (~subPayloadSize + 1U) & SDO_CMDL_FLAG_PADSIZE_MASK;
    }

    ami_setUint16Le(&pSubHdr_p->index, pSdoComCon_p->targetIndex);
    ami_setUint8Le(&pSubHdr_p->subIndex, pSdoComCon_p->targetSubIndex);
    // -> per default no next sub-response
    ami_setUint32Le(&pSubHdr_p->byteOffsetNext, 0);

    if (pLastSubHdr_p != NULL)
    {   // this is the next sub-response, assign offset so it can be found in frame
        ami_setUint32Le(&pLastSubHdr_p->byteOffsetNext, (UINT32)(cmdOffset_p + 8));
    }

    pSdoComCon_p->respSegmSize = cmdOffset_p + 8 + subPayloadSize;

    nextAlignedOffset = (8 + subPayloadSize + 3) & ~3;
    *ppNextSubHdr_p = (tAsySdoComMultWriteReqReadResp*)((UINT8*)pSubHdr_p + nextAlignedOffset);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get SDO error code from openPOWERLINK stack error

The function translates an eOplkError code to an SDO error code.

\param[in]      errorCode_p         Error code of type eOplkError

\return The function returns an SDO error code.
*/
//------------------------------------------------------------------------------
static UINT32 getSdoErrorCode(tOplkError errorCode_p)
{
    UINT32 sdoErrorCode = 0;

    switch (errorCode_p)
    {
        case kErrorOk:
        case kErrorReject:
            sdoErrorCode = 0;
            break;

        case kErrorObdIndexNotExist:
        case kErrorObdVarEntryNotExist:
            sdoErrorCode = SDO_AC_OBJECT_NOT_EXIST;
            break;

        case kErrorObdSubindexNotExist:
            sdoErrorCode = SDO_AC_SUB_INDEX_NOT_EXIST;
            break;

        case kErrorObdReadViolation:
            sdoErrorCode = SDO_AC_READ_TO_WRITE_ONLY_OBJ;
            break;

        case kErrorObdWriteViolation:
            sdoErrorCode = SDO_AC_WRITE_TO_READ_ONLY_OBJ;
            break;

        case kErrorObdAccessViolation:
            sdoErrorCode = SDO_AC_UNSUPPORTED_ACCESS;
            break;

        case kErrorObdValueTooLow:
            sdoErrorCode = SDO_AC_VALUE_RANGE_TOO_LOW;
            break;

        case kErrorObdValueTooHigh:
            sdoErrorCode = SDO_AC_VALUE_RANGE_TOO_HIGH;
            break;

        case kErrorObdValueLengthError:
            sdoErrorCode = SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH;
            break;

        case kErrorObdOutOfMemory:
            sdoErrorCode = SDO_AC_OUT_OF_MEMORY;
            break;

        case kErrorObdNoConfigData:
            sdoErrorCode = SDO_AC_CONFIG_DATA_EMPTY;
            break;

        default:
            sdoErrorCode = SDO_AC_GENERAL_ERROR;
            break;
    }

    return sdoErrorCode;
}

//------------------------------------------------------------------------------
/**
\brief  Sets an SDO error code to the provided address

The function sets an SDO error code to the provided address in case of an error.
If there is no error, no value is written. The SDO error code is translated
from an eOplkError (\ref tOplkError) error code.

\param[in]      errorCode_p         Error code of type eOplkError
\param[out]     pSetSdoError_p      SDO error code is written to this address
*/
//------------------------------------------------------------------------------
static void assignSdoErrorCode(tOplkError errorCode_p, UINT32* pSetSdoError_p)
{
    if ((errorCode_p != kErrorOk) && (errorCode_p != kErrorReject))
    {
        if (pSetSdoError_p != NULL)
            *pSetSdoError_p = getSdoErrorCode(errorCode_p);
    }
}

/// \}

#endif // defined(CONFIG_INCLUDE_SDOS)
