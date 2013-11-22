/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for SDO Command Layer module

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

  2006/06/26 k.t.:   start of the implementation

****************************************************************************/

#include <user/sdocom.h>

#if !defined(CONFIG_INCLUDE_SDOS) && !defined(CONFIG_INCLUDE_SDOC)
#error 'ERROR: At least SDO Server or SDO Client should be activate!'
#endif

#if defined(CONFIG_INCLUDE_SDOS) && !defined(CONFIG_INCLUDE_OBD)
#error 'ERROR: SDO Server needs OBD module!'
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

#ifndef MAX_SDO_COM_CON
#define MAX_SDO_COM_CON         5
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

/**
\brief  SDO command layer events

This enumeration defines all valid events which will be processed by the
SDO command layer.
*/
typedef enum
{
    kSdoComConEventSendFirst        = 0x00, ///< First frame to send
    kSdoComConEventRec              = 0x01, ///< Frame received
    kSdoComConEventConEstablished   = 0x02, ///< Connection established
    kSdoComConEventConClosed        = 0x03, ///< Connection closed
    kSdoComConEventAckReceived      = 0x04, ///< Acknowledge received by lower layer -> continue sending
    kSdoComConEventFrameSended      = 0x05, ///< Lower has send a frame
    kSdoComConEventInitError        = 0x06, ///< Error during initialization of the connection
    kSdoComConEventTimeout          = 0x07, ///< Timeout in lower layer
    kSdoComConEventTransferAbort    = 0x08, ///< Transfer abort by lower layer
#if defined(CONFIG_INCLUDE_SDOC)
    kSdoComConEventInitCon          = 0x09, ///< Init connection (only client)
    kSdoComConEventAbort            = 0x0A, ///< Abort SDO transfer (only client)
#endif
} tSdoComConEvent;


/**
\brief  SDO command layer message types

This enumeration defines all valid message types handled by the SDO command layer.
*/
typedef enum
{
    kSdoComSendTypeReq              = 0x00,  ///< Send a request
    kSdoComSendTypeAckRes           = 0x01,  ///< Send a response without data
    kSdoComSendTypeRes              = 0x02,  ///< Send response with data
    kSdoComSendTypeAbort            = 0x03   ///< Send abort
} tSdoComSendType;

/**
\brief  SDO command layer states

This enumeration defines all valid states of the SDO command layer state
machine.
*/
typedef enum
{
    kSdoComStateIdle                = 0x00, ///< Idle state
#if defined(CONFIG_INCLUDE_SDOS)
    kSdoComStateServerSegmTrans     = 0x01, ///< Client: Send following frames
#endif
#if defined(CONFIG_INCLUDE_SDOC)
    kSdoComStateClientWaitInit      = 0x10, ///< Server: Wait for init connection on lower layer
    kSdoComStateClientConnected     = 0x11, ///< Server: Connection established
    kSdoComStateClientSegmTrans     = 0x12  ///< Server: Send following frames
#endif
} tSdoComState;

/**
\brief  SDO command layer connection control structure

This structure defines the connection control structure of the command layer.
*/
typedef struct
{
    tSdoSeqConHdl       sdoSeqConHdl;       // if != 0 -> entry used
    tSdoComState        sdoComState;
    UINT8               transactionId;
    UINT                nodeId;             ///< NodeId of the target -> needed to reinit connection after timeout
    tSdoTransType       sdoTransferType;    ///< Transfer Type: Auto, Expedited, Segmented
    tSdoServiceType     sdoServiceType;     ///< Service Type: WriteByIndex, ReadByIndex
    tSdoType            sdoProtocolType;    ///< Protocol Type: Auto, Udp, ASnd
    UINT8*              pData;              ///< Pointer to data
    UINT                transferSize;       ///< Number of bytes to transfer
    UINT                transferredBytes;   ///< Number of bytes already transferred
    tSdoFinishedCb      pfnTransferFinished;///< Callback function to be called in the end of the SDO transfer
    void*               pUserArg;           ///< User definable argument pointer
    UINT32              lastAbortCode;      ///< Last abort code
#if defined(CONFIG_INCLUDE_SDOC)
    UINT                targetIndex;        ///< Object Index to access
    UINT                targetSubIndex;     ///< Object subindex to access
#endif
} tSdoComCon;

/**
\brief  SDO command layer instance structure

This structure describes a SDO command layer instance
*/
typedef struct
{
    tSdoComCon          sdoComCon[MAX_SDO_COM_CON]; ///< Array to store command layer connections
#if defined(WIN32) || defined(_WIN32)
    LPCRITICAL_SECTION  pCriticalSection;
    CRITICAL_SECTION    criticalSection;
#endif
} tSdoComInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
static tSdoComInstance sdoComInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static tEplKernel receiveCb (tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoCom* pSdoCom_p, UINT dataSize_p);
static tEplKernel conStateChangeCb (tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoConState sdoConnectionState_p);
static tEplKernel searchConnection(tSdoSeqConHdl sdoSeqConHdl_p, tSdoComConEvent sdoComConEvent_p, tAsySdoCom* pSdoCom_p);
static tEplKernel processState(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent SdoComConEvent_p,
                               tAsySdoCom* pSdoCom_p);
static tEplKernel processStateIdle(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                            tAsySdoCom* pRecvdCmdLayer_p);
static tEplKernel processStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p);
static tEplKernel processStateClientWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                             tAsySdoCom* pRecvdCmdLayer_p);
static tEplKernel processStateClientConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p);
static tEplKernel processStateClientSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                                 tAsySdoCom* pRecvdCmdLayer_p);
static tEplKernel transferFinished(tSdoComConHdl sdoComConHdl_p, tSdoComCon* pSdoComCon_p,
                                   tSdoComConState sdoComConState_p);

#if defined (CONFIG_INCLUDE_SDOS)
static tEplKernel serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
static tEplKernel serverSendFrame(tSdoComCon* pSdoComCon_p, UINT index_p,
                                  UINT subIndex_p, tSdoComSendType sendType_p);
static tEplKernel serverInitWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
#endif

#if defined(CONFIG_INCLUDE_SDOC)
static tEplKernel clientSend(tSdoComCon* pSdoComCon_p);
static tEplKernel clientProcessFrame(tSdoComConHdl sdoComConHdl_p, tAsySdoCom* pSdoCom_p);
static tEplKernel clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p);
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <SDO Command Layer>                                 */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: SDO Command layer Modul
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    sdocom_init
//
// Description: Init first instance of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_init(void)
{
    return sdocom_addInstance();
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_addInstance
//
// Description: Init additional instance of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_addInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    EPL_MEMSET(&sdoComInstance_l, 0x00, sizeof(sdoComInstance_l));

    ret = sdoseq_addInstance(receiveCb, conStateChangeCb);
    if(ret != kEplSuccessful)
        return ret;

#if defined(WIN32) || defined(_WIN32)
    sdoComInstance_l.pCriticalSection = &sdoComInstance_l.criticalSection;
    InitializeCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_delInstance
//
// Description: delete instance of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_delInstance(void)
{
    tEplKernel  ret = kEplSuccessful;

#if defined(WIN32) || defined(_WIN32)
    DeleteCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    ret = sdoseq_delInstance();
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOC)
//---------------------------------------------------------------------------
//
// Function:    sdocom_defineConnection
//
// Description: function defines a SDO connection to another node
//              -> init lower layer and returns a handle for the connection.
//              Two client connections to the same node via the same protocol
//              are not allowed. If this function detects such a situation
//              it will return kEplSdoComHandleExists and the handle of
//              the existing connection in pSdoComConHdl_p.
//              Using of existing server connections is possible.
//
// Parameters:  pSdoComConHdl_p     = pointer to the buffer of the handle
//              targetNodeId_p      = NodeId of the targetnode
//              protType_p          = type of protocol to use for connection
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_defineConnection(tSdoComConHdl* pSdoComConHdl_p, UINT targetNodeId_p,
                                   tSdoType protType_p)
{
    tEplKernel      ret;
    UINT            count;
    UINT            freeHdl;
    tSdoComCon*     pSdoComCon;

    if((targetNodeId_p == EPL_C_ADR_INVALID) || (targetNodeId_p >= EPL_C_ADR_BROADCAST))
        ret = kEplInvalidNodeId;

    // search free control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[0];
    count = 0;
    freeHdl = MAX_SDO_COM_CON;
    while (count < MAX_SDO_COM_CON)
    {
        if (pSdoComCon->sdoSeqConHdl == 0)
        {   // free entry
            freeHdl = count;
        }
        else if ((pSdoComCon->nodeId == targetNodeId_p) && (pSdoComCon->sdoProtocolType == protType_p))
        {   // existing client connection with same node ID and same protocol type
            *pSdoComConHdl_p = count;
            return kEplSdoComHandleExists;
        }
        count++;
        pSdoComCon++;
    }

    if (freeHdl == MAX_SDO_COM_CON)
    {
        return kEplSdoComNoFreeHandle;
    }

    pSdoComCon = &sdoComInstance_l.sdoComCon[freeHdl];

    *pSdoComConHdl_p = freeHdl;                 // save handle for application

    pSdoComCon->sdoProtocolType = protType_p;
    pSdoComCon->nodeId = targetNodeId_p;
    pSdoComCon->transactionId = 0;

    switch(protType_p)
    {
        case kSdoTypeUdp:
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeUdp);
            if(ret != kEplSuccessful)
                return ret;
            break;

        case kSdoTypeAsnd:
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeAsnd);
            if(ret != kEplSuccessful)
                return ret;
            break;

        case kSdoTypePdo:       // SDO over PDO -> not supported
        default:
            return kEplSdoComUnsupportedProt;
            break;
    }

    ret = processState(freeHdl, kSdoComConEventInitCon, NULL);
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_initTransferByIndex
//
// Description: function init SDO Transfer for a defined connection
//
//
//
// Parameters:  SdoComTransParam_p    = Structure with parameters for connection
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_initTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    tEplKernel      ret;
    tSdoComCon*     pSdoComCon;

    if ((pSdoComTransParam_p->subindex >= 0xFF) || (pSdoComTransParam_p->index == 0) ||
        (pSdoComTransParam_p->index > 0xFFFF) || (pSdoComTransParam_p->pData == NULL) ||
        (pSdoComTransParam_p->dataSize == 0))
        return kEplSdoComInvalidParam;

    if(pSdoComTransParam_p->sdoComConHdl >= MAX_SDO_COM_CON)
        return kEplSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[pSdoComTransParam_p->sdoComConHdl];

    if(pSdoComCon->sdoSeqConHdl == 0)
        return kEplSdoComInvalidHandle;

    // check if command layer is idle
    if ((pSdoComCon->transferredBytes + pSdoComCon->transferSize) > 0)
        return kEplSdoComHandleBusy;

    // callback function for end of transfer
    pSdoComCon->pfnTransferFinished = pSdoComTransParam_p->pfnSdoFinishedCb;
    pSdoComCon->pUserArg = pSdoComTransParam_p->pUserArg;

    if (pSdoComTransParam_p->sdoAccessType == kSdoAccessTypeRead)
    {
        pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
    }
    else
    {
        pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;

    }

    pSdoComCon->pData = pSdoComTransParam_p->pData;             // save pointer to data
    pSdoComCon->transferSize = pSdoComTransParam_p->dataSize;   // maximal bytes to transfer
    pSdoComCon->transferredBytes = 0;                           // bytes already transfered

    pSdoComCon->lastAbortCode = 0;
    pSdoComCon->sdoTransferType = kSdoTransAuto;

    pSdoComCon->targetIndex = pSdoComTransParam_p->index;
    pSdoComCon->targetSubIndex = pSdoComTransParam_p->subindex;

    ret = processState(pSdoComTransParam_p->sdoComConHdl, kSdoComConEventSendFirst, NULL);

    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_undefineConnection
//
// Description: function undefines a SDO connection
//
//
//
// Parameters:  sdoComConHdl_p    = handle for the connection
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_undefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComCon*         pSdoComCon;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
        return kEplSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // $$$ d.k. abort a running transfer before closing the sequence layer
    if(((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) != SDO_SEQ_INVALID_HDL) &&
        (pSdoComCon->sdoSeqConHdl != 0))
    {
        // close connection in lower layer
        switch(pSdoComCon->sdoProtocolType)
        {
            case kSdoTypeAsnd:
            case kSdoTypeUdp:
                ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                break;

            case kSdoTypePdo:
            case kSdoTypeAuto:
            default:
                return kEplSdoComUnsupportedProt;
                break;
        }
    }

    EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_getState
//
// Description: function returns the state of the connection
//
//
//
// Parameters:  sdoComConHdl_p    = handle for the connection
//              pSdoComFinished_p = pointer to structure for sdo state
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_getState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComCon*         pSdoComCon;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
        return kEplSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if(pSdoComCon->sdoSeqConHdl == 0)
        return kEplSdoComInvalidHandle;

    pSdoComFinished_p->pUserArg = pSdoComCon->pUserArg;
    pSdoComFinished_p->nodeId = pSdoComCon->nodeId;
    pSdoComFinished_p->targetIndex = pSdoComCon->targetIndex;
    pSdoComFinished_p->targetSubIndex = pSdoComCon->targetSubIndex;
    pSdoComFinished_p->transferredBytes = pSdoComCon->transferredBytes;
    pSdoComFinished_p->abortCode = pSdoComCon->lastAbortCode;
    pSdoComFinished_p->sdoComConHdl = sdoComConHdl_p;
    if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
    {
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeWrite;
    }
    else
    {
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeRead;
    }

    if(pSdoComCon->lastAbortCode != 0)
    {   // sdo abort
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferRxAborted;
        // delete abort code
        pSdoComCon->lastAbortCode = 0;
    }
    else if((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK)== SDO_SEQ_INVALID_HDL)
    {   // check state
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferLowerLayerAbort;
    }
    else if(pSdoComCon->sdoComState == kSdoComStateClientWaitInit)
    {   // finished
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferNotActive;
    }
    else if(pSdoComCon->transferSize == 0)
    {   // finished
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferFinished;
    }
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_getNodeId
//
// Description: returns the remote node-ID which corresponds to the specified handle
//
// Parameters:  sdoComConHdl_p    = handle for the connection
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
UINT sdocom_getNodeId(tSdoComConHdl sdoComConHdl_p)
{
    UINT            nodeId = EPL_C_ADR_INVALID;
    tSdoComCon*     pSdoComCon;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
        return nodeId;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    if(pSdoComCon->sdoSeqConHdl == 0)
        return nodeId;

    nodeId = pSdoComCon->nodeId;
    return nodeId;
}

//---------------------------------------------------------------------------
//
// Function:    sdocom_abortTransfer
//
// Description: function abort a sdo transfer
//
//
//
// Parameters:  sdoComConHdl_p    = handle for the connection
//              abortCode_p     = abort code
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdocom_abortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tEplKernel      ret;
    tSdoComCon*     pSdoComCon;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
        return kEplSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    if(pSdoComCon->sdoSeqConHdl == 0)
        return kEplSdoComInvalidHandle;

    pSdoComCon->pData = (UINT8*)&abortCode_p;
    ret = processState(sdoComConHdl_p, kSdoComConEventAbort, (tAsySdoCom*)NULL);
    return ret;
}

#endif

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:        receiveCb
//
// Description:     callback function for SDO Sequence Layer
//                  -> indicates new data
//
//
//
// Parameters:      sdoSeqConHdl_p = Handle for connection
//                  pSdoCom_p   = pointer to data
//                  dataSize_p   = size of data ($$$ not used yet, but it should)
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel receiveCb (tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoCom* pSdoCom_p, UINT dataSize_p)
{
    tEplKernel       ret;

    UNUSED_PARAMETER(dataSize_p);

    ret = searchConnection(sdoSeqConHdl_p, kSdoComConEventRec, pSdoCom_p);
    EPL_DBGLVL_SDO_TRACE("receiveCb SdoSeqConHdl: 0x%X, First Byte of pSdoCom_p: 0x%02X, dataSize_p: 0x%04X\n",
                         sdoSeqConHdl_p, (WORD)pSdoCom_p->m_le_abCommandData[0], dataSize_p);
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:        conStateChangeCb
//
// Description:     callback function called by SDO Sequence Layer to inform
//                  command layer about state change of connection
//
//
//
// Parameters:      sdoSeqConHdl_p      = Handle of the connection
//                  sdoConnectionState_p    = Event of the connection
//
//
// Returns:         tEplKernel  = Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel conStateChangeCb (tSdoSeqConHdl sdoSeqConHdl_p,
                                    tAsySdoConState sdoConnectionState_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComConEvent     sdoComConEvent = kSdoComConEventSendFirst;

    switch(sdoConnectionState_p)
    {
        case kAsySdoConStateConnected:
            EPL_DBGLVL_SDO_TRACE("Connection established\n");
            sdoComConEvent = kSdoComConEventConEstablished;
            // start transmission if needed
            break;

        case kAsySdoConStateInitError:
            EPL_DBGLVL_SDO_TRACE("Error during initialisation\n");
            sdoComConEvent = kSdoComConEventInitError;
            // inform app about error and close sequence layer handle
            break;

        case kAsySdoConStateConClosed:
            EPL_DBGLVL_SDO_TRACE("Connection closed\n");
            sdoComConEvent = kSdoComConEventConClosed;
            // close sequence layer handle
            break;

        case kAsySdoConStateAckReceived:
            EPL_DBGLVL_SDO_TRACE("Acknowledge received\n");
            sdoComConEvent = kSdoComConEventAckReceived;
            // continue transmission
            break;

        case kAsySdoConStateFrameSended:
            EPL_DBGLVL_SDO_TRACE("One Frame sent\n");
            sdoComConEvent = kSdoComConEventFrameSended;
            // to continue transmission
            break;

        case kAsySdoConStateTimeout:
            EPL_DBGLVL_SDO_TRACE("Timeout\n");
            sdoComConEvent = kSdoComConEventTimeout;
            // close sequence layer handle
            break;

        case kAsySdoConStateTransferAbort:
            EPL_DBGLVL_SDO_TRACE("Transfer aborted\n");
            sdoComConEvent = kSdoComConEventTransferAbort;
            // inform higher layer if necessary,
            // but do not close sequence layer handle
            break;
    }

    ret = searchConnection(sdoSeqConHdl_p, sdoComConEvent, (tAsySdoCom*)NULL);
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:        searchConnection
//
// Description:     search a Sdo Sequence Layer connection handle in the
//                  control structure of the Command Layer
//
// Parameters:      sdoSeqConHdl_p     = Handle to search
//                  sdoComConEvent_p = event to process
//                  pSdoCom_p     = pointer to received frame
//
// Returns:         tEplKernel
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel searchConnection(tSdoSeqConHdl sdoSeqConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                   tAsySdoCom* pSdoCom_p)
{
    tEplKernel          ret;
    tSdoComCon*         pSdoComCon;
    tSdoComConHdl       hdlCount;
    tSdoComConHdl       hdlFree;

    ret = kEplSdoComNotResponsible;

    // get pointer to first element of the array
    pSdoComCon = &sdoComInstance_l.sdoComCon[0];
    hdlCount = 0;
    hdlFree = 0xFFFF;
    while (hdlCount < MAX_SDO_COM_CON)
    {
        if (pSdoComCon->sdoSeqConHdl == sdoSeqConHdl_p)
        {   // matching command layer handle found
            ret = processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
        else if ((pSdoComCon->sdoSeqConHdl == 0) &&(hdlFree == 0xFFFF))
        {
            hdlFree = hdlCount;
        }
        pSdoComCon++;
        hdlCount++;
    }

    if (ret == kEplSdoComNotResponsible)
    {   // no responsible command layer handle found
        if (hdlFree == 0xFFFF)
        {   // no free handle delete connection immediately
            // 2008/04/14 m.u./d.k. This connection actually does not exist.
            //                      pSdoComCon is invalid.
            // ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            ret = kEplSdoComNoFreeHandle;
        }
        else
        {   // create new handle
            hdlCount = hdlFree;
            pSdoComCon = &sdoComInstance_l.sdoComCon[hdlCount];
            pSdoComCon->sdoSeqConHdl = sdoSeqConHdl_p;
            ret = processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateIdle

The function processes the SDO command handler state: kSdoComStateIdle

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateIdle(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                   tAsySdoCom* pRecvdCmdLayer_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComCon*         pSdoComCon;

#if defined(CONFIG_INCLUDE_SDOS)
    UINT32              abortCode;
#endif

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch(sdoComConEvent_p)
    {
#if defined(CONFIG_INCLUDE_SDOC)
        case kSdoComConEventInitCon: // init con for client
            // call of the init function already processed in sdocom_defineConnection()
            // only change state to kSdoComStateClientWaitInit
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            break;
#endif

        case kSdoComConEventRec: // int con for server
#if defined(CONFIG_INCLUDE_SDOS)
            // check if init of an transfer and no SDO abort
            if ((pRecvdCmdLayer_p->m_le_bFlags & SDO_CMDL_FLAG_RESPONSE) == 0)
            {   // SDO request
                if ((pRecvdCmdLayer_p->m_le_bFlags & SDO_CMDL_FLAG_ABORT) == 0)
                {   // no SDO abort, save tansaction id
                    pSdoComCon->transactionId = AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bTransactionId);

                    switch(pRecvdCmdLayer_p->m_le_bCommandId)
                    {
                        case kSdoServiceNIL:
                            // simply acknowlegde NIL command on sequence layer
                            ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                            break;

                        case kSdoServiceReadByIndex:
                            // read by index, search entry an start transfer
                            serverInitReadByIndex(pSdoComCon, pRecvdCmdLayer_p);
                            // check next state
                            if(pSdoComCon->transferSize == 0)
                            {   // ready -> stay idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                            else
                            {   // segmented transfer
                                pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                            }
                            break;

                        case kSdoServiceWriteByIndex:
                            // search entry an start write
                            serverInitWriteByIndex(pSdoComCon, pRecvdCmdLayer_p);
                            // check next state
                            if(pSdoComCon->transferSize == 0)
                            {   // already -> stay idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                            else
                            {   // segmented transfer
                                pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                            }
                            break;

                        default:
                            //  unsupported command -> send abort
                            abortCode = EPL_SDOAC_UNKNOWN_COMMAND_SPECIFIER;
                            pSdoComCon->pData = (UINT8*)&abortCode;
                            ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                            break;
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                return kEplSdoComNotResponsible;
            }
#endif
            break;

        // connection closed
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventConClosed:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
            break;

        default:
            break;
    }
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOS)

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateServerSegmTrans

The function processes the SDO command handler state: kSdoComStateServerSegmTrans

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT                size;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch(sdoComConEvent_p)
    {
        // send next frame
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameSended:
            // check if it is a read
            if(pSdoComCon->sdoServiceType == kSdoServiceReadByIndex)
            {
                // send next frame
                serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeRes);
                // if all send -> back to idle
                if(pSdoComCon->transferSize == 0)
                {   // back to idle
                    pSdoComCon->sdoComState = kSdoComStateIdle;
                    pSdoComCon->lastAbortCode = 0;
                }
            }
            break;

        // process next frame
        case kSdoComConEventRec:
            // check if the frame is a SDO response and has the right transaction ID
            flag = AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bFlags);

            if (((flag & SDO_CMDL_FLAG_RESPONSE) == 0) &&
                (AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bTransactionId) == pSdoComCon->transactionId))
            {
                // check if it is a abort
                if ((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    pSdoComCon->transferSize = 0;
                    pSdoComCon->transferredBytes = 0;
                    pSdoComCon->sdoComState = kSdoComStateIdle;
                    pSdoComCon->lastAbortCode = 0;
                    // d.k.: do not execute anything further on this command
                    break;
                }

                // check if it is a write
                if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
                {
                    size = AmiGetWordFromLe(&pRecvdCmdLayer_p->m_le_wSegmentSize);
                    if (size > pSdoComCon->transferSize)
                    {
                        pSdoComCon->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_TOO_HIGH;
                        ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                        return ret;
                    }
                    if (pSdoComCon->lastAbortCode == 0)
                    {
                        EPL_MEMCPY(pSdoComCon->pData, &pRecvdCmdLayer_p->m_le_abCommandData[0], size);
                        (pSdoComCon->pData) += size;
                    }
                    pSdoComCon->transferredBytes += size;
                    pSdoComCon->transferSize -= size;

                    // check end of transfer
                    if((pRecvdCmdLayer_p->m_le_bFlags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMCOMPL)
                    {   // transfer ready
                        pSdoComCon->transferSize = 0;

                        if(pSdoComCon->lastAbortCode == 0)
                        {
                            // send response
                            serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeRes);
                            // if all send -> back to idle
                            if(pSdoComCon->transferSize == 0)
                            {   // back to idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                        }
                        else
                        {   // send abort
                            pSdoComCon->pData = (UINT8*)&pSdoComCon->lastAbortCode;
                            ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                            pSdoComCon->lastAbortCode = 0;
                        }
                    }
                    else
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                ret = kEplSdoComNotResponsible;
            }
            break;

        // connection closed
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventConClosed:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
            break;

        default:
            break;
    }
    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Process state processStateClientWaitInit

The function processes the SDO command handler state: processStateClientWaitInit

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateClientWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                             tAsySdoCom* pRecvdCmdLayer_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComCon*         pSdoComCon;

    UNUSED_PARAMETER(pRecvdCmdLayer_p);

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // if connection handle is invalid reinit connection
    // d.k.: this will be done only on new events (i.e. InitTransfer)
    if((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
    {
        // check kind of connection to reinit
        switch(pSdoComCon->sdoProtocolType)
        {
            case kSdoTypeUdp:
                ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeUdp);
                if(ret != kEplSuccessful)
                    return ret;
                break;

            case kSdoTypeAsnd:
                ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeAsnd);
                if(ret != kEplSuccessful)
                    return ret;
                break;

            case kSdoTypePdo:   // Pdo -> not supported
            default:
                ret = kEplSdoComUnsupportedProt;
                return ret;
                break;
        }
        // d.k.: reset transaction ID, because new sequence layer connection was initialized
        // $$$ d.k. is this really necessary?
        //pSdoComCon->transactionId = 0;
    }

    switch(sdoComConEvent_p)
    {
        case kSdoComConEventConEstablished:
            // send first frame if needed
            if ((pSdoComCon->transferSize > 0) && (pSdoComCon->targetIndex != 0))
            {   // start SDO transfer
                // check if segemted transfer
                if (pSdoComCon->sdoTransferType == kSdoTransSegmented)
                {
                    pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                }
                else
                {
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                }

                ret = clientSend(pSdoComCon);
                if (ret != kEplSuccessful)
                    return ret;
            }
            else
            {
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
            }
            break;

        case kSdoComConEventSendFirst:
            // infos for transfer already saved by function sdocom_initTransferByIndex
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
            break;

        case kSdoComConEventConClosed:
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventTransferAbort:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            if (sdoComConEvent_p == kSdoComConEventTimeout)
            {
                pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
            }
            else
            {
                pSdoComCon->lastAbortCode = 0;
            }
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            // d.k.: do not clean control structure
            break;

        default:
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateClientConnected

The function processes the SDO command handler state: kSdoComStateClientConnected

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateClientConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p)
{

    tEplKernel          ret = kEplSuccessful;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch(sdoComConEvent_p)
    {
        // send a frame
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameSended:
            ret = clientSend(pSdoComCon);
            if(ret != kEplSuccessful)
                return ret;

            // check if read transfer finished
            if((pSdoComCon->transferSize == 0) && (pSdoComCon->transferredBytes != 0) &&
               (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->lastAbortCode = 0;
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);
                return ret;
            }

            if(pSdoComCon->sdoTransferType == kSdoTransSegmented)
            {
                pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                return ret;
            }
            break;

        case kSdoComConEventRec:
            // check if the frame is a SDO response and has the right transaction ID
            flag = AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bFlags);
            if (((flag & SDO_CMDL_FLAG_RESPONSE) != 0) &&
                 (AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bTransactionId) == pSdoComCon->transactionId))
            {
                if((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    // send acknowledge without any Command layer data
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->lastAbortCode = AmiGetDwordFromLe(&pRecvdCmdLayer_p->m_le_abCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if(pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);
                        return ret;
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                ret = kEplSdoComNotResponsible;
                return ret;
            }
            break;

        // connection closed event go back to kSdoComStateClientWaitInit
        case kSdoComConEventConClosed:
            // connection closed by communication partner
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon,*((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            break;

        default:
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateClientSegmTrans

The function processes the SDO command handler state: kSdoComStateClientSegmTrans

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStateClientSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                                 tAsySdoCom* pRecvdCmdLayer_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch(sdoComConEvent_p)
    {
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameSended:
            ret = clientSend(pSdoComCon);
            if(ret != kEplSuccessful)
                return ret;

            // check if read transfer finished
            if((pSdoComCon->transferSize == 0) && (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
                pSdoComCon->lastAbortCode = 0;
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);
                return ret;
            }
            break;

        case kSdoComConEventRec:
            // check if the frame is a response
            flag = AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bFlags);
            if (((flag & SDO_CMDL_FLAG_RESPONSE) != 0) &&
                (AmiGetByteFromLe(&pRecvdCmdLayer_p->m_le_bTransactionId) == pSdoComCon->transactionId))
            {
                if((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    // send acknowledge without any Command layer data
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                    pSdoComCon->lastAbortCode = AmiGetDwordFromLe(&pRecvdCmdLayer_p->m_le_abCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if(pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tEplFrame*)NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->sdoComState = kSdoComStateClientConnected;
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);
                    }
                }
            }
            break;

        // connection closed event go back to kSdoComStateClientWaitInit
        case kSdoComConEventConClosed:
            // connection closed by communication partner
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon,*((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->sdoComState = kSdoComStateClientConnected;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);
            break;

        default:
            break;
    }
    return ret;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Process SDO command layer state machine

The function processes the SDO command handler state machine. Depending
on the state the command layer event is processed.

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processState(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                               tAsySdoCom* pRecvdCmdLayer_p)
{
    tEplKernel          ret = kEplSuccessful;
    tSdoComCon*         pSdoComCon;

#if defined(WIN32) || defined(_WIN32)
    EnterCriticalSection(sdoComInstance_l.pCriticalSection);
    EPL_DBGLVL_SDO_TRACE("\n\tEnterCiticalSection processState\n\n");
#endif

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // process state maschine
    switch(pSdoComCon->sdoComState)
    {
        // idle state
        case kSdoComStateIdle:
            ret = processStateIdle(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

#if defined(CONFIG_INCLUDE_SDOS)
        //----------------------------------------------------------------------
        // SDO Server part
        // segmented transfer
        case kSdoComStateServerSegmTrans:
            ret = processStateServerSegmTrans(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif

#if defined(CONFIG_INCLUDE_SDOC)
        //----------------------------------------------------------------------
        // SDO Client part
        // wait for finish of establishing connection
        case kSdoComStateClientWaitInit:
            ret = processStateClientWaitInit(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        case kSdoComStateClientConnected:
            ret = processStateClientConnected(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        // process segmented transfer
        case kSdoComStateClientSegmTrans:
            ret = processStateClientSegmTransfer(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif
    }

#if defined(WIN32) || defined(_WIN32)
    EPL_DBGLVL_SDO_TRACE("\n\tLeaveCriticalSection processState\n\n");
    LeaveCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOS)
//---------------------------------------------------------------------------
//
// Function:        serverInitReadByIndex
//
// Description:    function start the processing of an read by index command
//
//
//
// Parameters:      pSdoComCon     = pointer to control structure of connection
//                  pSdoCom_p     = pointer to received frame
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p)
{
    tEplKernel      ret;
    UINT            index;
    UINT            subindex;
    tObdSize        entrySize;
    tObdAccess      accessType;
    UINT32          abortCode;

    abortCode = 0;

    // An init of a read could not be a segmented transfer -> no variable part of header

    index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
    subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[2]);

    ret = obd_getAccessType(index, subindex, &accessType);
    if(ret == kEplObdSubindexNotExist)
    {   // subentry doesn't exist
        abortCode = EPL_SDOAC_SUB_INDEX_NOT_EXIST;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }
    else if(ret != kEplSuccessful)
    {   // entry doesn't exist
        abortCode = EPL_SDOAC_OBJECT_NOT_EXIST;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }

    // access type must be read or const
    if(((accessType & kObdAccRead) == 0) && ((accessType & kObdAccConst) == 0))
    {
        if((accessType & kObdAccWrite) != 0)
        {
            abortCode = EPL_SDOAC_READ_TO_WRITE_ONLY_OBJ;
        }
        else
        {
            abortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
        }
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }

    pSdoComCon_p->sdoServiceType = kSdoServiceReadByIndex;

    // get size of object to see if segmented or expedited transfer
    entrySize = obd_getDataSize(index, subindex);
    if(entrySize > SDO_MAX_SEGMENT_SIZE)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        pSdoComCon_p->pData = obd_getObjectDataPtr(index, subindex);
    }
    else
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
    }

    pSdoComCon_p->transferSize = entrySize;
    pSdoComCon_p->transferredBytes = 0;

    ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeRes);
    if(ret != kEplSuccessful)
    {
        abortCode = EPL_SDOAC_GENERAL_ERROR;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:        serverSendFrame();
//
// Description:    function creats and send a frame for server
//
//
//
// Parameters:      pSdoComCon_p     = pointer to control structure of connection
//                  index_p        = index to send if expedited transfer else 0
//                  subIndex_p     = subindex to send if expedited transfer else 0
//                  sendType_p       = to of frame to send
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel serverSendFrame(tSdoComCon* pSdoComCon_p, UINT index_p,
                                  UINT subIndex_p, tSdoComSendType sendType_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tEplFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;
    UINT8           flag;

    pFrame = (tEplFrame*)&aFrame[0];
    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame - get pointer to command layer part of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe(&pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe(&pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    sizeOfFrame = 8;        // set size to header size

    switch(sendType_p)
    {

        case kSdoComSendTypeReq:    // request frame to send
            // nothing to do for server -> error
            return kEplSdoComInvalidSendType;
            break;

        case kSdoComSendTypeAckRes: // response without data to send
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  SDO_CMDL_FLAG_RESPONSE);
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            break;

        case kSdoComSendTypeRes:    // response frame to send
            flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
            flag |= SDO_CMDL_FLAG_RESPONSE;
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

            if(pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
            {   // Expedited transfer
                // copy data in frame
                ret = obd_readEntryToLe(index_p, subIndex_p,
                                        &pCommandFrame->m_le_abCommandData[0],
                                        (tObdSize*)&pSdoComCon_p->transferSize);
                if(ret != kEplSuccessful)
                    return ret;

                AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);
                sizeOfFrame += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferSize = 0;

                sizeOfFrame += pSdoComCon_p->transferSize;
                ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            }
            else if(pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
            {   // segmented transfer
                // distinguish between init, segment and complete
                if(pSdoComCon_p->transferredBytes == 0)
                {   // init
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= SDO_CMDL_FLAG_SEGMINIT;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);
                    // init data size in variable header, which includes itself
                    AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], pSdoComCon_p->transferSize + 4);
                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[4],pSdoComCon_p->pData, (SDO_MAX_SEGMENT_SIZE-4));

                    pSdoComCon_p->transferSize -= (SDO_MAX_SEGMENT_SIZE-4);
                    pSdoComCon_p->transferredBytes += (SDO_MAX_SEGMENT_SIZE-4);
                    pSdoComCon_p->pData +=(SDO_MAX_SEGMENT_SIZE-4);

                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, SDO_MAX_SEGMENT_SIZE);

                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
                else if((pSdoComCon_p->transferredBytes > 0) &&(pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE))
                {   // segment
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= SDO_CMDL_FLAG_SEGMENTED;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[0],pSdoComCon_p->pData, SDO_MAX_SEGMENT_SIZE);
                    pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                    pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                    pSdoComCon_p->pData +=SDO_MAX_SEGMENT_SIZE;
                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize,SDO_MAX_SEGMENT_SIZE);

                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
                else
                {
                    if((pSdoComCon_p->transferSize == 0) && (pSdoComCon_p->sdoServiceType != kSdoServiceWriteByIndex))
                        return ret;

                    // complete
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= SDO_CMDL_FLAG_SEGMCOMPL;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);
                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[0],pSdoComCon_p->pData, pSdoComCon_p->transferSize);
                    pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                    pSdoComCon_p->pData +=pSdoComCon_p->transferSize;
                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);

                    sizeOfFrame += pSdoComCon_p->transferSize;
                    pSdoComCon_p->transferSize = 0;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
            }
            break;

        case kSdoComSendTypeAbort:
            flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
            flag |= (SDO_CMDL_FLAG_RESPONSE | SDO_CMDL_FLAG_ABORT);
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

            // copy abort code to frame
            AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], *((UINT32*)pSdoComCon_p->pData));
            AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, sizeof(UINT32));
            pSdoComCon_p->transferredBytes = sizeof(UINT32);
            pSdoComCon_p->transferSize = 0;

            sizeOfFrame += sizeof(UINT32);
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            DEBUG_LVL_25_TRACE("ERROR: SDO Aborted!\n");
            break;
    }
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:        serverInitWriteByIndex
//
// Description:    function start the processing of an write by index command
//
//
//
// Parameters:      pSdoComCon_p     = pointer to control structure of connection
//                  pSdoCom_p     = pointer to received frame
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel serverInitWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            index;
    UINT            subindex;
    UINT            bytesToTransfer;
    tObdSize        entrySize;
    tObdAccess      accessType;
    UINT8*          pSrcData;

    // An init of a write -> variable part of header possible

    // check if expedited or segmented transfer
    if ((pSdoCom_p->m_le_bFlags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMINIT)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[4]);
        subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[6]);
        pSrcData = &pSdoCom_p->m_le_abCommandData[8];
        pSdoComCon_p->transferSize = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);

        pSdoComCon_p->transferSize -= 8;
    }
    else if ((pSdoCom_p->m_le_bFlags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
        subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[2]);
        pSrcData = &pSdoCom_p->m_le_abCommandData[4];
        pSdoComCon_p->transferSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
        // subtract header
        pSdoComCon_p->transferSize -= 4;    // jba define for header (all occurances) // subtract header
    }
    else
    {
        return ret;                 // just ignore any other transfer type
    }

    ret = obd_getAccessType(index, subindex, &accessType);
    if (ret == kEplObdSubindexNotExist)
    {
        pSdoComCon_p->lastAbortCode = EPL_SDOAC_SUB_INDEX_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }
    else if(ret != kEplSuccessful)
    {   // entry doesn't exist
        pSdoComCon_p->lastAbortCode = EPL_SDOAC_OBJECT_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // compare access type,  must be read
    if((accessType & kObdAccWrite) == 0)
    {
        if((accessType & kObdAccRead) != 0)
        {
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_WRITE_TO_READ_ONLY_OBJ;
        }
        else
        {
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
        }

        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // save service
    pSdoComCon_p->sdoServiceType = kSdoServiceWriteByIndex;
    pSdoComCon_p->transferredBytes = 0;

    // write data to OD
    if(pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {   // expedited transfer, size checking is done by obd_writeEntryFromLe()

        ret = obd_writeEntryFromLe(index, subindex, pSrcData, pSdoComCon_p->transferSize);
        switch (ret)
        {
            case kEplSuccessful:
                break;

            case kEplObdAccessViolation:
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
                // send abort
                goto Abort;
                break;

            case kEplObdValueLengthError:
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_NOT_MATCH;
                // send abort
                goto Abort;
                break;

            case kEplObdValueTooHigh:
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_VALUE_RANGE_TOO_HIGH;
                // send abort
                goto Abort;
                break;

            case kEplObdValueTooLow:
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_VALUE_RANGE_TOO_LOW;
                // send abort
                goto Abort;
                break;

            default:
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
                // send abort
                goto Abort;
                break;
        }

        // send command acknowledge
        ret = serverSendFrame(pSdoComCon_p, 0, 0, kSdoComSendTypeAckRes);

        pSdoComCon_p->transferSize = 0;
        return ret;
    }
    else
    {
        // get size of the object to check if it fits
        // because we directly write to the destination memory
        // d.k. no one calls the user OD callback function

        entrySize = obd_getDataSize(index, subindex);
        if(entrySize < pSdoComCon_p->transferSize)
        {   // parameter too big
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_TOO_HIGH;
            // send abort
            // d.k. This is wrong: k.t. not needed send abort on end of write
            /*pSdoComCon_p->pData = (UINT8*)&abortCode;
            ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
            goto Abort;
        }

        bytesToTransfer = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
        // eleminate header (Command header (8) + variable part (4) + Command header (4))
        bytesToTransfer -= 16;
        // get pointer to object entry
        pSdoComCon_p->pData = obd_getObjectDataPtr(index, subindex);
        if(pSdoComCon_p->pData == NULL)
        {
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
            // send abort
            // d.k. This is wrong: k.t. not needed send abort on end of write
/*            pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
            ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
            goto Abort;
        }

        EPL_MEMCPY(pSdoComCon_p->pData, pSrcData, bytesToTransfer);
        pSdoComCon_p->transferredBytes = bytesToTransfer;
        pSdoComCon_p->transferSize -= bytesToTransfer;
        (/*(UINT8*)*/pSdoComCon_p->pData) += bytesToTransfer;

        // send acknowledge without any Command layer data
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tEplFrame*)NULL);
        return ret;
    }

Abort:
    if(pSdoComCon_p->lastAbortCode != 0)
    {
        // send abort
        pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);

        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
        pSdoComCon_p->transferSize = 0;
    }

    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_SDOC)
//---------------------------------------------------------------------------
//
// Function:        clientSend
//
// Description:    function starts an sdo transfer an send all further frames
//
//
//
// Parameters:      pSdoComCon_p     = pointer to control structure of connection
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel clientSend(tSdoComCon* pSdoComCon_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tEplFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;
    UINT8           flags;
    UINT8*          pPayload;

    pFrame = (tEplFrame*)&aFrame[0];

    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe( &pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe( &pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    // set size constant part of header
    sizeOfFrame = 8;        // jba add define!

    // check if first frame to send -> command header needed
    if (pSdoComCon_p->transferSize > 0)
    {
        if (pSdoComCon_p->transferredBytes == 0)
        {   // start SDO transfer
            // check if segmented or expedited transfer
            // only for write commands
            switch(pSdoComCon_p->sdoServiceType)
            {
                case kSdoServiceReadByIndex:
                    // first frame of read access always expedited
                    pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                    pPayload = &pCommandFrame->m_le_abCommandData[0];
                    AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, 4);

                    // create command header
                    AmiSetWordToLe(pPayload, (WORD)pSdoComCon_p->targetIndex);
                    pPayload += 2;
                    AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                    sizeOfFrame += 4;
                    pSdoComCon_p->transferredBytes = 1;
                    break;

                case kSdoServiceWriteByIndex:
                    if(pSdoComCon_p->transferSize > (SDO_MAX_SEGMENT_SIZE - 4))
                    {   // segmented transfer
                        // -> variable part of header needed
                        // save that transfer is segmented
                        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
                        // fill variable part of header
                        // set data size which includes the header
                        AmiSetDwordToLe( &pCommandFrame->m_le_abCommandData[0], pSdoComCon_p->transferSize + 8);
                        // set pointer to real payload
                        pPayload = &pCommandFrame->m_le_abCommandData[4];
                        // fill rest of header
                        AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, SDO_MAX_SEGMENT_SIZE);
                        flags = SDO_CMDL_FLAG_SEGMINIT;
                        AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                        // create command header
                        AmiSetWordToLe(pPayload, (WORD) pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        // on byte for reserved
                        pPayload += 2;

                        sizeOfFrame += SDO_MAX_SEGMENT_SIZE;

                        EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  (SDO_MAX_SEGMENT_SIZE - 8));
                        pSdoComCon_p->pData += (SDO_MAX_SEGMENT_SIZE - 8);
                        pSdoComCon_p->transferSize -= (SDO_MAX_SEGMENT_SIZE - 8);
                        pSdoComCon_p->transferredBytes = (SDO_MAX_SEGMENT_SIZE - 8);

                    }
                    else
                    {   // expedited transfer
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        pPayload = &pCommandFrame->m_le_abCommandData[0];

                        AmiSetWordToLe(pPayload, (WORD) pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        // + 2 -> one byte for subindex and one byte reserved
                        pPayload += 2;
                        EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                        sizeOfFrame += (4 + pSdoComCon_p->transferSize);
                        AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, (WORD) (4 + pSdoComCon_p->transferSize));

                        pSdoComCon_p->transferredBytes = pSdoComCon_p->transferSize;
                        pSdoComCon_p->transferSize = 0;
                    }
                    break;

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    return kEplSdoComInvalidServiceType;
            }
        }
        else
        {   // continue SDO transfer
            switch(pSdoComCon_p->sdoServiceType)
            {
                // for expedited read is nothing to do -> server sends data
                case kSdoServiceWriteByIndex:
                    // send next frame
                    if(pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
                    {
                        if(pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE)
                        {   // next segment
                            pPayload = &pCommandFrame->m_le_abCommandData[0];
                            AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, SDO_MAX_SEGMENT_SIZE);
                            flags = SDO_CMDL_FLAG_SEGMENTED;
                            AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                            EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  SDO_MAX_SEGMENT_SIZE);
                            pSdoComCon_p->pData += SDO_MAX_SEGMENT_SIZE;
                            pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                            pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                            sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                        }
                        else
                        {   // end of transfer
                            pPayload = &pCommandFrame->m_le_abCommandData[0];
                            AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);
                            flags = SDO_CMDL_FLAG_SEGMCOMPL;
                            AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                            EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                            pSdoComCon_p->pData += pSdoComCon_p->transferSize;
                            sizeOfFrame += pSdoComCon_p->transferSize;
                            pSdoComCon_p->transferSize = 0;
                            pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                        }
                    }
                    else
                    {
                        return ret;
                    }
                    break;

                default:
                    return ret;
                    break;
            }
        }
    }
    else
    {
        return ret;
    }

    // call send function of lower layer
    switch(pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            break;

        default:
            return kEplSdoComUnsupportedProt;
    }

    return ret;
}

//---------------------------------------------------------------------------
//
// Function:        clientProcessFrame
//
// Description:    function process a received frame
//
//
//
// Parameters:      sdoComConHdl_p      = connection handle
//                  pSdoCom_p     = pointer to frame to process
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel clientProcessFrame(tSdoComConHdl sdoComConHdl_p, tAsySdoCom* pSdoCom_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT8               flags;
    UINT8               transactionId;
    UINT8               command;
    UINT                segmentSize;
    UINT                dataSize;
    ULONG               transferSize;
    tSdoComCon*         pSdoComCon;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    transactionId = AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId);
    if(pSdoComCon->transactionId != transactionId)
    {
        // if running transfer
        if((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize !=0))
        {
            pSdoComCon->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
            clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
        }
    }
    else
    {   // check if correct command
        command = AmiGetByteFromLe(&pSdoCom_p->m_le_bCommandId);
        if(pSdoComCon->sdoServiceType != command)
        {
            // incorrect command
            // if running transfer
            if((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize !=0))
            {
                pSdoComCon->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
            }
        }
        else
        {   // switch on command
            switch(pSdoComCon->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    // check if confirmation from server
                    // nothing more to do
                    break;

                case kSdoServiceReadByIndex:
                    flags = AmiGetByteFromLe(&pSdoCom_p->m_le_bFlags);
                    flags &= SDO_CMDL_FLAG_SEGM_MASK;
                    switch (flags)
                    {
                        case SDO_CMDL_FLAG_EXPEDITED:
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // buffer provided by the application is too small -> copy only a part
                                dataSize = pSdoComCon->transferSize;
                            }
                            else
                            {   // buffer fits
                                dataSize = segmentSize;
                            }

                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], dataSize);
                            pSdoComCon->transferSize = 0;
                            pSdoComCon->transferredBytes = dataSize;
                            break;


                        case SDO_CMDL_FLAG_SEGMINIT:
                            // get total size of transfer including the header
                            transferSize = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
                            // subtract size of variable header from data size
                            transferSize -= 4;
                            if (transferSize <= pSdoComCon->transferSize)
                            {   // buffer fits
                                pSdoComCon->transferSize = (UINT)transferSize;
                            }
                            else
                            {   // buffer too small -> send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_TOO_HIGH;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                return ret;
                            }

                            // get segment size
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            // subtract size of variable header from segment size
                            segmentSize -= 4;
                            // copy data
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[4], segmentSize);

                            // correct counter an pointer
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes = segmentSize;
                            pSdoComCon->transferSize -= segmentSize;

                            break;

                        case SDO_CMDL_FLAG_SEGMENTED:
                            // get segment size
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // segment too large -> send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_INVALID_BLOCK_SIZE;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                return ret;
                            }
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], segmentSize);
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize -= segmentSize;
                            break;

                        case SDO_CMDL_FLAG_SEGMCOMPL:
                            // get segment size
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            // check if data to copy fit to buffer
                            if(segmentSize > pSdoComCon->transferSize)
                            {   // segment too large -> send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_INVALID_BLOCK_SIZE;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                return ret;
                            }
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], segmentSize);
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize  = 0;
                            break;
                    }
                    break;

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    // $$$ d.k. What should we do?
                    break;
            }
        }
    }

    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    clientSendAbort
//
// Description: function send a abort message
//
//
//
// Parameters:  pSdoComCon_p     = pointer to control structure of connection
//              abortCode_p    = Sdo abort code
//
// Returns:     tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tEplFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;

    pFrame = (tEplFrame*)&aFrame[0];

    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe( &pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe( &pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    sizeOfFrame = 8;
    pCommandFrame->m_le_bFlags |= SDO_CMDL_FLAG_ABORT;

    AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], abortCode_p);
    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, sizeof(UINT32));

    pSdoComCon_p->transferredBytes = sizeof(UINT32);
    pSdoComCon_p->transferSize = 0;

    sizeOfFrame += sizeof(UINT32);

    pSdoComCon_p->lastAbortCode = abortCode_p;

    // call send function of lower layer
    switch(pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            if (ret == kEplSdoSeqConnectionBusy)
            {
                DEBUG_LVL_25_TRACE("%s tried to send abort 0x%lX while connection is already closed\n",
                    __func__, (ULONG)abortCode_p);
                ret = kEplSuccessful;
            }
            break;

        default:
            ret = kEplSdoComUnsupportedProt;
            break;
    }
    return ret;
}
#endif

//---------------------------------------------------------------------------
//
// Function:    transferFinished
//
// Description: calls callback function of application if available
//              and clears entry in control structure
//
// Parameters:  pSdoComCon_p     = pointer to control structure of connection
//              SdoComConState_p = state of SDO transfer
//
// Returns:     tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel transferFinished(tSdoComConHdl sdoComConHdl_p, tSdoComCon* pSdoComCon_p,
                                   tSdoComConState sdoComConState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tSdoFinishedCb  pfnTransferFinished;
    tSdoComFinished sdoComFinished;

    if(pSdoComCon_p->pfnTransferFinished != NULL)
    {

        sdoComFinished.pUserArg = pSdoComCon_p->pUserArg;
        sdoComFinished.nodeId = pSdoComCon_p->nodeId;
        sdoComFinished.targetIndex = pSdoComCon_p->targetIndex;
        sdoComFinished.targetSubIndex = pSdoComCon_p->targetSubIndex;
        sdoComFinished.transferredBytes = pSdoComCon_p->transferredBytes;
        sdoComFinished.abortCode = pSdoComCon_p->lastAbortCode;
        sdoComFinished.sdoComConHdl = sdoComConHdl_p;
        sdoComFinished.sdoComConState = sdoComConState_p;
        if (pSdoComCon_p->sdoServiceType == kSdoServiceWriteByIndex)
        {
            sdoComFinished.sdoAccessType = kSdoAccessTypeWrite;
        }
        else
        {
            sdoComFinished.sdoAccessType = kSdoAccessTypeRead;
        }

        // reset transfer state so this handle is not busy anymore
        pSdoComCon_p->transferredBytes = 0;
        pSdoComCon_p->transferSize = 0;

        pfnTransferFinished = pSdoComCon_p->pfnTransferFinished;
        // delete function pointer to inform application only once for each transfer
        pSdoComCon_p->pfnTransferFinished = NULL;

        // call application's callback function
        pfnTransferFinished(&sdoComFinished);
    }
    return ret;
}

