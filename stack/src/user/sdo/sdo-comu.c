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

#if ((((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) == 0) &&\
     (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) == 0)   )

    #error 'ERROR: At least SDO Server or SDO Client should be activate!'

#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)

#if !defined(CONFIG_INCLUDE_OBD)
#error 'ERROR: SDO Server needs OBDu module!'
#endif

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
    UINT                timeout;            ///< Timeout for this connection
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
tEplKernel PUBLIC receiveCb (tSdoSeqConHdl    sdoSeqConHdl_p,
                                    tAsySdoCom*      pSdoCom_p,
                                    UINT        dataSize_p);


tEplKernel PUBLIC conStateChangeCb (tSdoSeqConHdl    sdoSeqConHdl_p,
                                    tAsySdoConState  sdoConnectionState_p);

static tEplKernel searchConnection(tSdoSeqConHdl    sdoSeqConHdl_p,
                                         tSdoComConEvent sdoComConEvent_p,
                                         tAsySdoCom*     pSdoCom_p);

static tEplKernel processState(tSdoComConHdl   sdoComConHdl_p,
                                         tSdoComConEvent sdoComConEvent_p,
                                         tAsySdoCom*     pSdoCom_p);

static tEplKernel transferFinished(tSdoComConHdl   sdoComConHdl_p,
                                            tSdoComCon*     pSdoComCon_p,
                                            tSdoComConState SdoComConState_p);

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
static tEplKernel serverInitReadByIndex(tSdoComCon*     pSdoComCon_p,
                                         tAsySdoCom*     pSdoCom_p);

static tEplKernel serverSendFrame(tSdoComCon*     pSdoComCon_p,
                                           UINT       index_p,
                                           UINT       subIndex_p,
                                           tSdoComSendType sendType_p);

static tEplKernel serverInitWriteByIndex(tSdoComCon*     pSdoComCon_p,
                                         tAsySdoCom*     pSdoCom_p);
#endif


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)

static tEplKernel clientSend(tSdoComCon* pSdoComCon_p);

static tEplKernel clientProcessFrame(tSdoComConHdl   sdoComConHdl_p,
                                              tAsySdoCom*     pSdoCom_p);

static tEplKernel clientSendAbort(tSdoComCon* pSdoComCon_p,
                                           UINT32          abortCode_p);
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
tEplKernel  ret;


    ret = sdocom_addInstance();

return ret;

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
tEplKernel ret;

    ret = kEplSuccessful;

    // init control structure
    EPL_MEMSET(&sdoComInstance_l, 0x00, sizeof(sdoComInstance_l));

    // init instance of lower layer
    ret = sdoseq_addInstance(receiveCb, conStateChangeCb);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

#if defined(WIN32) || defined(_WIN32)
    // create critical section for process function
    sdoComInstance_l.pCriticalSection = &sdoComInstance_l.criticalSection;
    InitializeCriticalSection(sdoComInstance_l.pCriticalSection);
#endif

Exit:
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
tEplKernel  ret;

    ret = kEplSuccessful;


#if defined(WIN32) || defined(_WIN32)
    // delete critical section for process function
    DeleteCriticalSection(sdoComInstance_l.pCriticalSection);
#endif

    ret = sdoseq_delInstance();
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }


Exit:
    return ret;
}

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
tEplKernel sdocom_defineConnection(tSdoComConHdl* pSdoComConHdl_p, UINT targetNodeId_p,
                                   tSdoType protType_p)
{
tEplKernel      ret;
UINT            count;
UINT            freeHdl;
tSdoComCon*     pSdoComCon;

    // check Parameter
    ASSERT(pSdoComConHdl_p != NULL);

    // check NodeId
    if((targetNodeId_p == EPL_C_ADR_INVALID)
        ||(targetNodeId_p >= EPL_C_ADR_BROADCAST))
    {
        ret = kEplInvalidNodeId;

    }

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
        else if ((pSdoComCon->nodeId == targetNodeId_p)
            && (pSdoComCon->sdoProtocolType == protType_p))
        {   // existing client connection with same node ID and same protocol type
            *pSdoComConHdl_p = count;
            ret = kEplSdoComHandleExists;
            goto Exit;
        }
        count++;
        pSdoComCon++;
    }

    if (freeHdl == MAX_SDO_COM_CON)
    {
        ret = kEplSdoComNoFreeHandle;
        goto Exit;
    }

    pSdoComCon = &sdoComInstance_l.sdoComCon[freeHdl];
    // save handle for application
    *pSdoComConHdl_p = freeHdl;
    // save parameters
    pSdoComCon->sdoProtocolType = protType_p;
    pSdoComCon->nodeId = targetNodeId_p;

    // set Transaction Id
    pSdoComCon->transactionId = 0;

    // check protocol
    switch(protType_p)
    {
        // udp
        case kSdoTypeUdp:
        {
            // call connection int function of lower layer
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl,
                          pSdoComCon->nodeId,
                          kSdoTypeUdp);
            if(ret != kEplSuccessful)
            {
                goto Exit;
            }
            break;
        }

        // Asend
        case kSdoTypeAsnd:
        {
            // call connection int function of lower layer
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl,
                          pSdoComCon->nodeId,
                          kSdoTypeAsnd);
            if(ret != kEplSuccessful)
            {
                goto Exit;
            }
            break;
        }

        // Pdo -> not supported
        case kSdoTypePdo:
        default:
        {
            ret = kEplSdoComUnsupportedProt;
            goto Exit;
        }
    }// end of switch(m_ProtType_p)

    // call process function
    ret = processState(freeHdl,
                                    kSdoComConEventInitCon,
                                    NULL);

Exit:
    return ret;
}
#endif
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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
tEplKernel sdocom_initTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p)
{
tEplKernel      ret;
tSdoComCon*     pSdoComCon;

    // check parameter
    if ((pSdoComTransParam_p->subindex >= 0xFF)
        || (pSdoComTransParam_p->index == 0)
        || (pSdoComTransParam_p->index > 0xFFFF)
        || (pSdoComTransParam_p->pData == NULL)
        || (pSdoComTransParam_p->dataSize == 0))
    {
        ret = kEplSdoComInvalidParam;
        goto Exit;
    }

    if(pSdoComTransParam_p->sdoComConHdl >= MAX_SDO_COM_CON)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[pSdoComTransParam_p->sdoComConHdl];

    // check if handle ok
    if(pSdoComCon->sdoSeqConHdl == 0)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // check if command layer is idle
    if ((pSdoComCon->transferredBytes + pSdoComCon->transferSize) > 0)
    {   // handle is not idle
        ret = kEplSdoComHandleBusy;
        goto Exit;
    }

    // save parameter
    // callback function for end of transfer
    pSdoComCon->pfnTransferFinished = pSdoComTransParam_p->pfnSdoFinishedCb;
    pSdoComCon->pUserArg = pSdoComTransParam_p->pUserArg;

    // set type of SDO command
    if (pSdoComTransParam_p->sdoAccessType == kSdoAccessTypeRead)
    {
        pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
    }
    else
    {
        pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;

    }
    // save pointer to data
    pSdoComCon->pData = pSdoComTransParam_p->pData;
    // maximal bytes to transfer
    pSdoComCon->transferSize = pSdoComTransParam_p->dataSize;
    // bytes already transfered
    pSdoComCon->transferredBytes = 0;

    // reset parts of control structure
    pSdoComCon->lastAbortCode = 0;
    pSdoComCon->sdoTransferType = kSdoTransAuto;
    // save timeout
    //pSdoComCon->timeout = SdoComTransParam_p.timeout;

    // save index and subindex
    pSdoComCon->targetIndex = pSdoComTransParam_p->index;
    pSdoComCon->targetSubIndex = pSdoComTransParam_p->subindex;

    // call process function
    ret = processState(pSdoComTransParam_p->sdoComConHdl,
                                    kSdoComConEventSendFirst,    // event to start transfer
                                    NULL);

Exit:
    return ret;

}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
tEplKernel sdocom_undefineConnection(tSdoComConHdl sdoComConHdl_p)
{
tEplKernel          ret;
tSdoComCon*         pSdoComCon;

    ret = kEplSuccessful;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // $$$ d.k. abort a running transfer before closing the sequence layer

    if(((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK)  != SDO_SEQ_INVALID_HDL)
        && (pSdoComCon->sdoSeqConHdl != 0))
    {
        // close connection in lower layer
        switch(pSdoComCon->sdoProtocolType)
        {
            case kSdoTypeAsnd:
            case kSdoTypeUdp:
            {
                ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                break;
            }

            case kSdoTypePdo:
            case kSdoTypeAuto:
            default:
            {
                ret = kEplSdoComUnsupportedProt;
                goto Exit;
            }

        }// end of switch(pSdoComCon->sdoProtocolType)
    }


    // clean control structure
    EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
Exit:
    return ret;
}
#endif
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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
tEplKernel sdocom_getState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p)
{
tEplKernel          ret;
tSdoComCon*         pSdoComCon;

    ret = kEplSuccessful;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if(pSdoComCon->sdoSeqConHdl == 0)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

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
    {
        // finished
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferNotActive;
    }
    else if(pSdoComCon->transferSize == 0)
    {   // finished
        pSdoComFinished_p->sdoComConState = kEplSdoComTransferFinished;
    }

Exit:
    return ret;

}
#endif


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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
UINT sdocom_getNodeId(tSdoComConHdl sdoComConHdl_p)
{
UINT            nodeId = EPL_C_ADR_INVALID;
tSdoComCon*     pSdoComCon;

    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
    {
        goto Exit;
    }

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if(pSdoComCon->sdoSeqConHdl == 0)
    {
        goto Exit;
    }

    nodeId = pSdoComCon->nodeId;

Exit:
    return nodeId;
}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
tEplKernel sdocom_abortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
tEplKernel      ret;
tSdoComCon*     pSdoComCon;


    if(sdoComConHdl_p >= MAX_SDO_COM_CON)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if(pSdoComCon->sdoSeqConHdl == 0)
    {
        ret = kEplSdoComInvalidHandle;
        goto Exit;
    }

    // save pointer to abort code
    pSdoComCon->pData = (UINT8*)&abortCode_p;

    ret = processState(sdoComConHdl_p,
                                kSdoComConEventAbort,
                                (tAsySdoCom*)NULL);

Exit:
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
tEplKernel PUBLIC receiveCb (tSdoSeqConHdl    sdoSeqConHdl_p,
                                    tAsySdoCom*      pSdoCom_p,
                                    UINT        dataSize_p)
{
tEplKernel       ret;

    UNUSED_PARAMETER(dataSize_p);

    // search connection internally
    ret = searchConnection(sdoSeqConHdl_p,
                                   kSdoComConEventRec,
                                   pSdoCom_p);

    EPL_DBGLVL_SDO_TRACE("receiveCb SdoSeqConHdl: 0x%X, First Byte of pSdoCom_p: 0x%02X, dataSize_p: 0x%04X\n", sdoSeqConHdl_p, (WORD)pSdoCom_p->m_le_abCommandData[0], dataSize_p);

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
tEplKernel PUBLIC conStateChangeCb (tSdoSeqConHdl    sdoSeqConHdl_p,
                                    tAsySdoConState  sdoConnectionState_p)
{
tEplKernel          ret;
tSdoComConEvent     sdoComConEvent = kSdoComConEventSendFirst;

    ret = kEplSuccessful;

    // check state
    switch(sdoConnectionState_p)
    {
        case kAsySdoConStateConnected:
        {
            EPL_DBGLVL_SDO_TRACE("Connection established\n");
            sdoComConEvent = kSdoComConEventConEstablished;
            // start transmission if needed
            break;
        }

        case kAsySdoConStateInitError:
        {
            EPL_DBGLVL_SDO_TRACE("Error during initialisation\n");
            sdoComConEvent = kSdoComConEventInitError;
            // inform app about error and close sequence layer handle
            break;
        }

        case kAsySdoConStateConClosed:
        {
            EPL_DBGLVL_SDO_TRACE("Connection closed\n");
            sdoComConEvent = kSdoComConEventConClosed;
            // close sequence layer handle
            break;
        }

        case kAsySdoConStateAckReceived:
        {
            EPL_DBGLVL_SDO_TRACE("Acknowledge received\n");
            sdoComConEvent = kSdoComConEventAckReceived;
            // continue transmission
            break;
        }

        case kAsySdoConStateFrameSended:
        {
            EPL_DBGLVL_SDO_TRACE("One Frame sent\n");
            sdoComConEvent = kSdoComConEventFrameSended;
            // to continue transmission
            break;

        }

        case kAsySdoConStateTimeout:
        {
            EPL_DBGLVL_SDO_TRACE("Timeout\n");
            sdoComConEvent = kSdoComConEventTimeout;
            // close sequence layer handle
            break;

        }

        case kAsySdoConStateTransferAbort:
        {
            EPL_DBGLVL_SDO_TRACE("Transfer aborted\n");
            sdoComConEvent = kSdoComConEventTransferAbort;
            // inform higher layer if necessary,
            // but do not close sequence layer handle
            break;
        }

    }// end of switch(sdoConnectionState_p)

    ret = searchConnection(sdoSeqConHdl_p,
                                   sdoComConEvent,
                                   (tAsySdoCom*)NULL);

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
static tEplKernel searchConnection(tSdoSeqConHdl    sdoSeqConHdl_p,
                                         tSdoComConEvent sdoComConEvent_p,
                                         tAsySdoCom*     pSdoCom_p)
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
            ret = processState(hdlCount,
                                    sdoComConEvent_p,
                                    pSdoCom_p);
        }
        else if ((pSdoComCon->sdoSeqConHdl == 0)
            &&(hdlFree == 0xFFFF))
        {
            hdlFree = hdlCount;
        }

        pSdoComCon++;
        hdlCount++;
    }

    if (ret == kEplSdoComNotResponsible)
    {   // no responsible command layer handle found
        if (hdlFree == 0xFFFF)
        {   // no free handle
            // delete connection immediately
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
            ret = processState(hdlCount,
                                    sdoComConEvent_p,
                                    pSdoCom_p);
        }
    }

    return ret;

}

//---------------------------------------------------------------------------
//
// Function:        processState
//
// Description:     search a Sdo Sequence Layer connection handle in the
//                  control structure of the Command Layer
//
//
//
// Parameters:      sdoComConHdl_p     = index of control structure of connection
//                  sdoComConEvent_p = event to process
//                  pSdoCom_p     = pointer to received frame
//
// Returns:         tEplKernel  =  errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel processState(tSdoComConHdl   sdoComConHdl_p,
                                         tSdoComConEvent sdoComConEvent_p,
                                         tAsySdoCom*     pSdoCom_p)
{
tEplKernel          ret;
tSdoComCon*         pSdoComCon;
UINT8               flag;

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
UINT32               abortCode;
UINT        uiSize;
#endif

#if defined(WIN32) || defined(_WIN32)
    // enter  critical section for process function
    EnterCriticalSection(sdoComInstance_l.pCriticalSection);
    EPL_DBGLVL_SDO_TRACE("\n\tEnterCiticalSection processState\n\n");
#endif

    ret = kEplSuccessful;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // process state maschine
    switch(pSdoComCon->sdoComState)
    {
        // idle state
        case kSdoComStateIdle:
        {
            // check events
            switch(sdoComConEvent_p)
            {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
                // init con for client
                case kSdoComConEventInitCon:
                {

                    // call of the init function already
                    // processed in sdocom_defineConnection()
                    // only change state to kSdoComStateClientWaitInit
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    break;
                }
#endif


                // int con for server
                case kSdoComConEventRec:
                {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
                    // check if init of an transfer and no SDO abort
                    if ((pSdoCom_p->m_le_bFlags & 0x80) == 0)
                    {   // SDO request
                        if ((pSdoCom_p->m_le_bFlags & 0x40) == 0)
                        {   // no SDO abort
                            // save tansaction id
                            pSdoComCon->transactionId = AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId);
                            // check command
                            switch(pSdoCom_p->m_le_bCommandId)
                            {
                                case kSdoServiceNIL:
                                {   // simply acknowlegde NIL command on sequence layer

                                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                            0,
                                                                            (tEplFrame*)NULL);

                                    break;
                                }

                                case kSdoServiceReadByIndex:
                                {   // read by index

                                    // search entry an start transfer
                                    serverInitReadByIndex(pSdoComCon,
                                                                    pSdoCom_p);
                                    // check next state
                                    if(pSdoComCon->transferSize == 0)
                                    {   // ready -> stay idle
                                        pSdoComCon->sdoComState = kSdoComStateIdle;
                                        // reset abort code
                                        pSdoComCon->lastAbortCode = 0;
                                    }
                                    else
                                    {   // segmented transfer
                                        pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                                    }

                                    break;
                                }

                                case kSdoServiceWriteByIndex:
                                {

                                    // search entry an start write
                                    serverInitWriteByIndex(pSdoComCon,
                                                                    pSdoCom_p);
                                    // check next state
                                    if(pSdoComCon->transferSize == 0)
                                    {   // already -> stay idle
                                        pSdoComCon->sdoComState = kSdoComStateIdle;
                                        // reset abort code
                                        pSdoComCon->lastAbortCode = 0;
                                    }
                                    else
                                    {   // segmented transfer
                                        pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                                    }

                                    break;
                                }

                                default:
                                {
                                    //  unsupported command
                                    //       -> abort senden
                                    abortCode = EPL_SDOAC_UNKNOWN_COMMAND_SPECIFIER;
                                    // send abort
                                    pSdoComCon->pData = (UINT8*)&abortCode;
                                    ret = serverSendFrame(pSdoComCon,
                                                                0,
                                                                0,
                                                                kSdoComSendTypeAbort);

                                    break;
                                }


                            }// end of switch(pSdoCom_p->m_le_bCommandId)
                        }
                    }
                    else
                    {   // this command layer handle is not responsible
                        // (wrong direction or wrong transaction ID)
                        ret = kEplSdoComNotResponsible;
                        goto Exit;
                    }
#endif // end of #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)

                    break;
                }

                // connection closed
                case kSdoComConEventInitError:
                case kSdoComConEventTimeout:
                case kSdoComConEventConClosed:
                {
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    // clean control structure
                    EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
                    break;
                }

                default:
                    // d.k. do nothing
                    break;
            }// end of switch(sdoComConEvent_p)
            break;
        }

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
        //----------------------------------------------------------------------
        // SDO Server part
        // segmented transfer
        case kSdoComStateServerSegmTrans:
        {
            // check events
            switch(sdoComConEvent_p)
            {
                // send next frame
                case kSdoComConEventAckReceived:
                case kSdoComConEventFrameSended:
                {
                    // check if it is a read
                    if(pSdoComCon->sdoServiceType == kSdoServiceReadByIndex)
                    {
                        // send next frame
                        serverSendFrame(pSdoComCon,
                                                            0,
                                                            0,
                                                            kSdoComSendTypeRes);
                        // if all send -> back to idle
                        if(pSdoComCon->transferSize == 0)
                        {   // back to idle
                            pSdoComCon->sdoComState = kSdoComStateIdle;
                            // reset abort code
                            pSdoComCon->lastAbortCode = 0;
                        }

                    }
                    break;
                }

                // process next frame
                case kSdoComConEventRec:
                {
                    // check if the frame is a SDO response and has the right transaction ID
                    flag = AmiGetByteFromLe(&pSdoCom_p->m_le_bFlags);

                    if (((flag & 0x80) == 0)
                        && (AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId) == pSdoComCon->transactionId))
                    {
                        // check if it is a abort
                        if ((flag & 0x40) != 0)
                        {   // SDO abort
                            // clear control structure
                            pSdoComCon->transferSize = 0;
                            pSdoComCon->transferredBytes = 0;
                            // change state
                            pSdoComCon->sdoComState = kSdoComStateIdle;
                            // reset abort code
                            pSdoComCon->lastAbortCode = 0;
                            // d.k.: do not execute anything further on this command
                            break;
                        }

                        // check if it is a write
                        if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
                        {
                            // write data to OD
                            uiSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            if (uiSize > pSdoComCon->transferSize)
                            {
                                pSdoComCon->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_TOO_HIGH;
                                // send abort
                                ret = serverSendFrame(pSdoComCon,
                                                            0,
                                                            0,
                                                            kSdoComSendTypeAbort);
                                goto Exit;
                            }
                            if (pSdoComCon->lastAbortCode == 0)
                            {
                                EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0],uiSize);
                                (pSdoComCon->pData) += uiSize;
                            }
                            // update counter
                            pSdoComCon->transferredBytes += uiSize;
                            pSdoComCon->transferSize -= uiSize;

                            // check end of transfer
                            if((pSdoCom_p->m_le_bFlags & 0x30) == 0x30)
                            {   // transfer ready
                                pSdoComCon->transferSize = 0;

                                if(pSdoComCon->lastAbortCode == 0)
                                {
                                    // send response
                                    // send next frame
                                    serverSendFrame(pSdoComCon,
                                                                        0,
                                                                        0,
                                                                        kSdoComSendTypeRes);
                                    // if all send -> back to idle
                                    if(pSdoComCon->transferSize == 0)
                                    {   // back to idle
                                        pSdoComCon->sdoComState = kSdoComStateIdle;
                                        // reset abort code
                                        pSdoComCon->lastAbortCode = 0;
                                    }
                                }
                                else
                                {   // send dabort code
                                    // send abort
                                    pSdoComCon->pData = (UINT8*)&pSdoComCon->lastAbortCode;
                                    ret = serverSendFrame(pSdoComCon,
                                                                0,
                                                                0,
                                                                kSdoComSendTypeAbort);

                                    // reset abort code
                                    pSdoComCon->lastAbortCode = 0;

                                }
                            }
                            else
                            {
                                // send acknowledge without any Command layer data
                                ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                        0,
                                                                        (tEplFrame*)NULL);
                            }
                        }
                    }
                    else
                    {   // this command layer handle is not responsible
                        // (wrong direction or wrong transaction ID)
                        ret = kEplSdoComNotResponsible;
                        goto Exit;
                    }
                    break;
                }

                // connection closed
                case kSdoComConEventInitError:
                case kSdoComConEventTimeout:
                case kSdoComConEventConClosed:
                {
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    // clean control structure
                    EPL_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
                    break;
                }

                default:
                    // d.k. do nothing
                    break;
            }// end of switch(sdoComConEvent_p)

            break;
        }
#endif // endif of #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
        //----------------------------------------------------------------------
        // SDO Client part
        // wait for finish of establishing connection
        case kSdoComStateClientWaitInit:
        {

            // if connection handle is invalid reinit connection
            // d.k.: this will be done only on new events (i.e. InitTransfer)
            if((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
            {
                // check kind of connection to reinit
                // check protocol
                switch(pSdoComCon->sdoProtocolType)
                {
                    // udp
                    case kSdoTypeUdp:
                    {
                        // call connection int function of lower layer
                        ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl,
                                    pSdoComCon->nodeId,
                                    kSdoTypeUdp);
                        if(ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        break;
                    }

                    // Asend -> not supported
                    case kSdoTypeAsnd:
                    {
                        // call connection int function of lower layer
                        ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl,
                                    pSdoComCon->nodeId,
                                    kSdoTypeAsnd);
                        if(ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        break;
                    }

                    // Pdo -> not supported
                    case kSdoTypePdo:
                    default:
                    {
                        ret = kEplSdoComUnsupportedProt;
                        goto Exit;
                    }
                }// end of switch(m_ProtType_p)
                // d.k.: reset transaction ID, because new sequence layer connection was initialized
                // $$$ d.k. is this really necessary?
                //pSdoComCon->transactionId = 0;
            }

            // check events
            switch(sdoComConEvent_p)
            {
                // connection established
                case kSdoComConEventConEstablished:
                {
                    // send first frame if needed
                    if ((pSdoComCon->transferSize > 0)
                        && (pSdoComCon->targetIndex != 0))
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
                        {
                            goto Exit;
                        }

                    }
                    else
                    {
                        // goto state kSdoComStateClientConnected
                        pSdoComCon->sdoComState = kSdoComStateClientConnected;
                    }
                    goto Exit;
                }

                case kSdoComConEventSendFirst:
                {
                    // infos for transfer already saved by function sdocom_initTransferByIndex
                    break;
                }

                // abort to send from higher layer
                case kSdoComConEventAbort:
                {
                    // call callback of application
                    pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);

                    break;
                }

                case kSdoComConEventConClosed:
                case kSdoComConEventInitError:
                case kSdoComConEventTimeout:
                case kSdoComConEventTransferAbort:
                {
                    // close sequence layer handle
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
                    // call callback function
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
                }

                default:
                    // d.k. do nothing
                    break;

            } // end of  switch(sdoComConEvent_p)
            break;
        }

        // connected
        case kSdoComStateClientConnected:
        {
            // check events
            switch(sdoComConEvent_p)
            {
                // send a frame
                case kSdoComConEventSendFirst:
                case kSdoComConEventAckReceived:
                case kSdoComConEventFrameSended:
                {
                    ret = clientSend(pSdoComCon);
                    if(ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    // check if read transfer finished
                    if((pSdoComCon->transferSize == 0)
                        && (pSdoComCon->transferredBytes != 0)
                        && (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
                    {
                        // inc transaction id
                        pSdoComCon->transactionId++;
                        // call callback of application
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);

                        goto Exit;
                    }

                    // check if segemted transfer
                    if(pSdoComCon->sdoTransferType == kSdoTransSegmented)
                    {
                        pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                        goto Exit;
                    }
                    break;
                }

                // frame received
                case kSdoComConEventRec:
                {
                    // check if the frame is a SDO response and has the right transaction ID
                    flag = AmiGetByteFromLe(&pSdoCom_p->m_le_bFlags);
                    if (((flag & 0x80) != 0) && (AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId) == pSdoComCon->transactionId))
                    {
                        // check if abort or not
                        if((flag & 0x40) != 0)
                        {
                            // send acknowledge without any Command layer data
                            ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                    0,
                                                                    (tEplFrame*)NULL);
                            // inc transaction id
                            pSdoComCon->transactionId++;
                            // save abort code
                            pSdoComCon->lastAbortCode = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
                            // call callback of application
                            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferRxAborted);

                            goto Exit;
                        }
                        else
                        {   // normal frame received
                            // check frame
                            ret = clientProcessFrame(sdoComConHdl_p, pSdoCom_p);

                            // check if transfer ready
                            if(pSdoComCon->transferSize == 0)
                            {
                                // send acknowledge without any Command layer data
                                ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                        0,
                                                                        (tEplFrame*)NULL);
                                // inc transaction id
                                pSdoComCon->transactionId++;
                                // call callback of application
                                pSdoComCon->lastAbortCode = 0;
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);

                                goto Exit;
                            }

                        }
                    }
                    else
                    {   // this command layer handle is not responsible
                        // (wrong direction or wrong transaction ID)
                        ret = kEplSdoComNotResponsible;
                        goto Exit;
                    }
                    break;
                }

                // connection closed event go back to kSdoComStateClientWaitInit
                case kSdoComConEventConClosed:
                {   // connection closed by communication partner
                    // close sequence layer handle
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    // set handle to invalid and enter kSdoComStateClientWaitInit
                    pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;

                    // call callback of application
                    pSdoComCon->lastAbortCode = 0;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);

                    break;
                }

                // abort to send from higher layer
                case kSdoComConEventAbort:
                {
                    clientSendAbort(pSdoComCon,*((UINT32*)pSdoComCon->pData));

                    // inc transaction id
                    pSdoComCon->transactionId++;
                    // call callback of application
                    pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);

                    break;
                }

                case kSdoComConEventInitError:
                case kSdoComConEventTimeout:
                {
                    // close sequence layer handle
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    // call callback of application
                    pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);

                    break;
                }

                case kSdoComConEventTransferAbort:
                {
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    // call callback of application
                    pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);

                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            } // end of switch(sdoComConEvent_p)

            break;
        }

        // process segmented transfer
        case kSdoComStateClientSegmTrans:
        {
            // check events
            switch(sdoComConEvent_p)
            {
                // sned a frame
                case kSdoComConEventSendFirst:
                case kSdoComConEventAckReceived:
                case kSdoComConEventFrameSended:
                {
                    ret = clientSend(pSdoComCon);
                    if(ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    // check if read transfer finished
                    if((pSdoComCon->transferSize == 0)
                        && (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
                    {
                        // inc transaction id
                        pSdoComCon->transactionId++;
                        // change state
                        pSdoComCon->sdoComState = kSdoComStateClientConnected;
                        // call callback of application
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);

                        goto Exit;
                    }

                    break;
                }

                // frame received
                case kSdoComConEventRec:
                {
                    // check if the frame is a response
                    flag = AmiGetByteFromLe(&pSdoCom_p->m_le_bFlags);
                    if (((flag & 0x80) != 0) && (AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId) == pSdoComCon->transactionId))
                    {
                        // check if abort or not
                        if((flag & 0x40) != 0)
                        {
                            // send acknowledge without any Command layer data
                            ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                    0,
                                                                    (tEplFrame*)NULL);
                            // inc transaction id
                            pSdoComCon->transactionId++;
                            // change state
                            pSdoComCon->sdoComState = kSdoComStateClientConnected;
                            // save abort code
                            pSdoComCon->lastAbortCode = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
                            // call callback of application
                            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferRxAborted);

                            goto Exit;
                        }
                        else
                        {   // normal frame received
                            // check frame
                            ret = clientProcessFrame(sdoComConHdl_p, pSdoCom_p);

                            // check if transfer ready
                            if(pSdoComCon->transferSize == 0)
                            {
                                // send acknowledge without any Command layer data
                                ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl,
                                                                        0,
                                                                        (tEplFrame*)NULL);
                                // inc transaction id
                                pSdoComCon->transactionId++;
                                // change state
                                pSdoComCon->sdoComState = kSdoComStateClientConnected;
                                // call callback of application
                                pSdoComCon->lastAbortCode = 0;
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);

                            }

                        }
                    }
                    break;
                }

                // connection closed event go back to kSdoComStateClientWaitInit
                case kSdoComConEventConClosed:
                {   // connection closed by communication partner
                    // close sequence layer handle
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    // set handle to invalid and enter kSdoComStateClientWaitInit
                    pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    // inc transaction id
                    pSdoComCon->transactionId++;
                    // call callback of application
                    pSdoComCon->lastAbortCode = 0;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferFinished);

                    break;
                }

                // abort to send from higher layer
                case kSdoComConEventAbort:
                {
                    clientSendAbort(pSdoComCon,*((UINT32*)pSdoComCon->pData));

                    // inc transaction id
                    pSdoComCon->transactionId++;
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                    // call callback of application
                    pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);

                    break;
                }

                case kSdoComConEventInitError:
                case kSdoComConEventTimeout:
                {
                    // close sequence layer handle
                    ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                    pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    // call callback of application
                    pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);

                    break;
                }

                case kSdoComConEventTransferAbort:
                {
                    // change state
                    pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
                    // call callback of application
                    pSdoComCon->lastAbortCode = EPL_SDOAC_TIME_OUT;
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferLowerLayerAbort);

                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            } // end of switch(sdoComConEvent_p)

            break;
        }
#endif // endo of #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)

    }// end of switch(pSdoComCon->sdoComState)



#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
Exit:
#endif

#if defined(WIN32) || defined(_WIN32)
    // leave critical section for process function
    EPL_DBGLVL_SDO_TRACE("\n\tLeaveCriticalSection processState\n\n");
    LeaveCriticalSection(sdoComInstance_l.pCriticalSection);

#endif

    return ret;

}


//---------------------------------------------------------------------------
//
// Function:        serverInitReadByIndex
//
// Description:    function start the processing of an read by index command
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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
static tEplKernel serverInitReadByIndex(tSdoComCon*     pSdoComCon_p,
                                         tAsySdoCom*     pSdoCom_p)
{
tEplKernel      ret;
UINT            index;
UINT            subindex;
tObdSize        entrySize;
tObdAccess      accessType;
UINT32          abortCode;

    abortCode = 0;

    // a init of a read could not be a segmented transfer
    // -> no variable part of header

    // get index and subindex
    index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
    subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[2]);

    // check accesstype of entry
    // existens of entry
    ret = obd_getAccessType(index, subindex, &accessType);
    if(ret == kEplObdSubindexNotExist)
    {   // subentry doesn't exist
        abortCode = EPL_SDOAC_SUB_INDEX_NOT_EXIST;
        // send abort
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);
        goto Exit;
    }
    else if(ret != kEplSuccessful)
    {   // entry doesn't exist
        abortCode = EPL_SDOAC_OBJECT_NOT_EXIST;
        // send abort
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);
        goto Exit;
    }

    // compare accesstype must be read or const
    if(((accessType & kObdAccRead) == 0)
        && ((accessType & kObdAccConst) == 0))
    {

        if((accessType & kObdAccWrite) != 0)
        {
            // entry read a write only object
            abortCode = EPL_SDOAC_READ_TO_WRITE_ONLY_OBJ;
        }
        else
        {
            abortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
        }
        // send abort
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);
        goto Exit;
    }

    // save service
    pSdoComCon_p->sdoServiceType = kSdoServiceReadByIndex;

    // get size of object to see iof segmented or expedited transfer
    entrySize = obd_getDataSize(index, subindex);
    if(entrySize > SDO_MAX_SEGMENT_SIZE)
    {   // segmented transfer
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        // get pointer to object-entry data
        pSdoComCon_p->pData = obd_getObjectDataPtr(index, subindex);
    }
    else
    {   // expedited transfer
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
    }

    pSdoComCon_p->transferSize = entrySize;
    pSdoComCon_p->transferredBytes = 0;

    ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeRes);
    if(ret != kEplSuccessful)
    {
        // error -> abort
        abortCode = EPL_SDOAC_GENERAL_ERROR;
        // send abort
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);
        goto Exit;
    }

Exit:
    return ret;
}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
static tEplKernel serverSendFrame(tSdoComCon*     pSdoComCon_p,
                                           UINT       index_p,
                                           UINT       subIndex_p,
                                           tSdoComSendType sendType_p)
{
tEplKernel      ret;
UINT8           aFrame[SDO_MAX_FRAME_SIZE];
tEplFrame*      pFrame;
tAsySdoCom*     pCommandFrame;
UINT            sizeOfFrame;
UINT8           flag;

    ret = kEplSuccessful;

    pFrame = (tEplFrame*)&aFrame[0];

    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    // get pointer to command layerpart of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe(&pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe(&pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    // set size to header size
    sizeOfFrame = 8;

    // check SendType
    switch(sendType_p)
    {
        // requestframe to send
        case kSdoComSendTypeReq:
        {
            // nothing to do for server
            //-> error
            ret = kEplSdoComInvalidSendType;
            break;
        }

        // response without data to send
        case kSdoComSendTypeAckRes:
        {
            // set response flag
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  0x80);

            // send frame
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                            sizeOfFrame,
                                            pFrame);

            break;
        }

        // responsframe to send
        case kSdoComSendTypeRes:
        {
            // set response flag
            flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
            flag |= 0x80;
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

            // check type of resonse
            if(pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
            {   // Expedited transfer
                // copy data in frame
                ret = obd_readEntryToLe(index_p,
                                        subIndex_p,
                                        &pCommandFrame->m_le_abCommandData[0],
                                        (tObdSize*)&pSdoComCon_p->transferSize);
                if(ret != kEplSuccessful)
                {
                    goto Exit;
                }

                // set size of frame
                AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);

                // correct byte-counter
                sizeOfFrame += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferSize = 0;


                // send frame
                sizeOfFrame += pSdoComCon_p->transferSize;
                ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                            sizeOfFrame,
                                            pFrame);
            }
            else if(pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
            {   // segmented transfer
                // distinguish between init, segment and complete
                if(pSdoComCon_p->transferredBytes == 0)
                {   // init
                    // set init flag
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= 0x10;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);
                    // init data size in variable header, which includes itself
                    AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], pSdoComCon_p->transferSize + 4);
                    // copy data in frame
                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[4],pSdoComCon_p->pData, (SDO_MAX_SEGMENT_SIZE-4));

                    // correct byte-counter
                    pSdoComCon_p->transferSize -= (SDO_MAX_SEGMENT_SIZE-4);
                    pSdoComCon_p->transferredBytes += (SDO_MAX_SEGMENT_SIZE-4);
                    // move data pointer
                    pSdoComCon_p->pData +=(SDO_MAX_SEGMENT_SIZE-4);

                    // set segment size
                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, SDO_MAX_SEGMENT_SIZE);

                    // send frame
                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                                sizeOfFrame,
                                                pFrame);

                }
                else if((pSdoComCon_p->transferredBytes > 0)
                    &&(pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE))
                {   // segment
                    // set segment flag
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= 0x20;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

                    // copy data in frame
                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[0],pSdoComCon_p->pData, SDO_MAX_SEGMENT_SIZE);

                    // correct byte-counter
                    pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                    pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                    // move data pointer
                    pSdoComCon_p->pData +=SDO_MAX_SEGMENT_SIZE;

                    // set segment size
                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize,SDO_MAX_SEGMENT_SIZE);

                    // send frame
                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                                sizeOfFrame,
                                                pFrame);
                }
                else
                {
                    if((pSdoComCon_p->transferSize == 0)
                        && (pSdoComCon_p->sdoServiceType != kSdoServiceWriteByIndex))
                    {
                        goto Exit;
                    }
                    // complete
                    // set segment complete flag
                    flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
                    flag |= 0x30;
                    AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

                    // copy data in frame
                    EPL_MEMCPY(&pCommandFrame->m_le_abCommandData[0],pSdoComCon_p->pData, pSdoComCon_p->transferSize);

                    // correct byte-counter
                    pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;


                    // move data pointer
                    pSdoComCon_p->pData +=pSdoComCon_p->transferSize;

                    // set segment size
                    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);

                    // send frame
                    sizeOfFrame += pSdoComCon_p->transferSize;
                    pSdoComCon_p->transferSize = 0;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                                sizeOfFrame,
                                                pFrame);
                }

            }
            break;
        }
        // abort to send
        case kSdoComSendTypeAbort:
        {
            // set response and abort flag
            flag = AmiGetByteFromLe( &pCommandFrame->m_le_bFlags);
            flag |= 0xC0;
            AmiSetByteToLe(&pCommandFrame->m_le_bFlags,  flag);

            // copy abortcode to frame
            AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], *((UINT32*)pSdoComCon_p->pData));

            // set size of segment
            AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, sizeof(UINT32));

            // update counter
            pSdoComCon_p->transferredBytes = sizeof(UINT32);
            pSdoComCon_p->transferSize = 0;

            // calc framesize
            sizeOfFrame += sizeof(UINT32);
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                                sizeOfFrame,
                                                pFrame);
            DEBUG_LVL_25_TRACE("ERROR: SDO Aborted!\n");
            break;
        }
    } // end of switch(sendType_p)

Exit:
    return ret;
}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOS)) != 0)
static tEplKernel serverInitWriteByIndex(tSdoComCon*     pSdoComCon_p,
                                         tAsySdoCom*     pSdoCom_p)
{
tEplKernel      ret = kEplSuccessful;
UINT            index;
UINT            subindex;
UINT            bytesToTransfer;
tObdSize        entrySize;
tObdAccess      accessType;
UINT8*          pSrcData;

    // a init of a write
    // -> variable part of header possible

    // check if expedited or segmented transfer
    if ((pSdoCom_p->m_le_bFlags & 0x30) == 0x10)
    {   // initiate segmented transfer
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        // get index and subindex
        index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[4]);
        subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[6]);
        // get source-pointer for copy
        pSrcData = &pSdoCom_p->m_le_abCommandData[8];
        // save size
        pSdoComCon_p->transferSize = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
        // subtract header
        pSdoComCon_p->transferSize -= 8;

    }
    else if ((pSdoCom_p->m_le_bFlags & 0x30) == 0x00)
    {   // expedited transfer
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        // get index and subindex
        index = AmiGetWordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
        subindex = AmiGetByteFromLe(&pSdoCom_p->m_le_abCommandData[2]);
        // get source-pointer for copy
        pSrcData = &pSdoCom_p->m_le_abCommandData[4];
        // save size
        pSdoComCon_p->transferSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
        // subtract header
        pSdoComCon_p->transferSize -= 4;

    }
    else
    {
        // just ignore any other transfer type
        goto Exit;
    }

    // check accesstype of entry
    // existens of entry
    ret = obd_getAccessType(index, subindex, &accessType);
    if (ret == kEplObdSubindexNotExist)
    {   // subentry doesn't exist
        pSdoComCon_p->lastAbortCode = EPL_SDOAC_SUB_INDEX_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);*/
        goto Abort;
    }
    else if(ret != kEplSuccessful)
    {   // entry doesn't exist
        pSdoComCon_p->lastAbortCode = EPL_SDOAC_OBJECT_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // compare accesstype must be read
    if((accessType & kObdAccWrite) == 0)
    {

        if((accessType & kObdAccRead) != 0)
        {
            // entry write a read only object
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_WRITE_TO_READ_ONLY_OBJ;
        }
        else
        {
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
        }
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // save service
    pSdoComCon_p->sdoServiceType = kSdoServiceWriteByIndex;

    pSdoComCon_p->transferredBytes = 0;

    // write data to OD
    if(pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {   // expedited transfer
        // size checking is done by obd_writeEntryFromLe()

        ret = obd_writeEntryFromLe(index,
                                    subindex,
                                    pSrcData,
                                    pSdoComCon_p->transferSize);
        switch (ret)
        {
            case kEplSuccessful:
            {
                break;
            }

            case kEplObdAccessViolation:
            {
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_UNSUPPORTED_ACCESS;
                // send abort
                goto Abort;
            }

            case kEplObdValueLengthError:
            {
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_NOT_MATCH;
                // send abort
                goto Abort;
            }

            case kEplObdValueTooHigh:
            {
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_VALUE_RANGE_TOO_HIGH;
                // send abort
                goto Abort;
            }

            case kEplObdValueTooLow:
            {
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_VALUE_RANGE_TOO_LOW;
                // send abort
                goto Abort;
            }

            default:
            {
                pSdoComCon_p->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
                // send abort
                goto Abort;
            }
        }
        // send command acknowledge
        ret = serverSendFrame(pSdoComCon_p,
                                        0,
                                        0,
                                        kSdoComSendTypeAckRes);

        pSdoComCon_p->transferSize = 0;
        goto Exit;
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
            ret = serverSendFrame(pSdoComCon_p,
                                        index,
                                        subindex,
                                        kSdoComSendTypeAbort);*/
            goto Abort;
        }

        bytesToTransfer = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
        // eleminate header (Command header (8) + variable part (4) + Command header (4))
        bytesToTransfer -= 16;
        // get pointer to object entry
        pSdoComCon_p->pData = obd_getObjectDataPtr(index,
                                                        subindex);
        if(pSdoComCon_p->pData == NULL)
        {
            pSdoComCon_p->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
            // send abort
            // d.k. This is wrong: k.t. not needed send abort on end of write
/*            pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
            ret = serverSendFrame(pSdoComCon_p,
                                        index,
                                        subindex,
                                        kSdoComSendTypeAbort);*/
            goto Abort;
        }

        // copy data
        EPL_MEMCPY(pSdoComCon_p->pData, pSrcData, bytesToTransfer);

        // update internal counter
        pSdoComCon_p->transferredBytes = bytesToTransfer;
        pSdoComCon_p->transferSize -= bytesToTransfer;

        // update target pointer
        (/*(UINT8*)*/pSdoComCon_p->pData) += bytesToTransfer;

        // send acknowledge without any Command layer data
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                                0,
                                                (tEplFrame*)NULL);
        goto Exit;
    }

Abort:
    if(pSdoComCon_p->lastAbortCode != 0)
    {
        // send abort
        pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p,
                                    index,
                                    subindex,
                                    kSdoComSendTypeAbort);

        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
        pSdoComCon_p->transferSize = 0;
        goto Exit;
    }

Exit:
    return ret;
}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
static tEplKernel clientSend(tSdoComCon* pSdoComCon_p)
{
tEplKernel      ret;
UINT8           aFrame[SDO_MAX_FRAME_SIZE];
tEplFrame*      pFrame;
tAsySdoCom*     pCommandFrame;
UINT            sizeOfFrame;
UINT8           flags;
UINT8*          pPayload;

    ret = kEplSuccessful;

    pFrame = (tEplFrame*)&aFrame[0];

    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    // get pointer to command layerpart of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe( &pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe( &pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    // set size constant part of header
    sizeOfFrame = 8;

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
                {   // first frame of read access always expedited
                    pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                    pPayload = &pCommandFrame->m_le_abCommandData[0];
                    // fill rest of header
                    AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, 4);

                    // create command header
                    AmiSetWordToLe(pPayload, (WORD)pSdoComCon_p->targetIndex);
                    pPayload += 2;
                    AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                    // calc size
                    sizeOfFrame += 4;

                    // set pSdoComCon_p->transferredBytes to one
                    pSdoComCon_p->transferredBytes = 1;
                    break;
                }

                case kSdoServiceWriteByIndex:
                {
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
                        flags = 0x10;
                        AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                        // create command header
                        AmiSetWordToLe(pPayload, (WORD) pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        // on byte for reserved
                        pPayload += 2;
                        // calc size
                        sizeOfFrame += SDO_MAX_SEGMENT_SIZE;

                        // copy payload
                        EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  (SDO_MAX_SEGMENT_SIZE - 8));
                        pSdoComCon_p->pData += (SDO_MAX_SEGMENT_SIZE - 8);
                        // correct intern counter
                        pSdoComCon_p->transferSize -= (SDO_MAX_SEGMENT_SIZE - 8);
                        pSdoComCon_p->transferredBytes = (SDO_MAX_SEGMENT_SIZE - 8);

                    }
                    else
                    {   // expedited trandsfer
                        // save that transfer is expedited
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        pPayload = &pCommandFrame->m_le_abCommandData[0];

                        // create command header
                        AmiSetWordToLe(pPayload, (WORD) pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        AmiSetByteToLe(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        // + 2 -> one byte for subindex and one byte reserved
                        pPayload += 2;
                        // copy data
                        EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                        // calc size
                        sizeOfFrame += (4 + pSdoComCon_p->transferSize);
                        // fill rest of header
                        AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, (WORD) (4 + pSdoComCon_p->transferSize));

                        pSdoComCon_p->transferredBytes = pSdoComCon_p->transferSize;
                        pSdoComCon_p->transferSize = 0;
                    }
                    break;
                }

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    ret = kEplSdoComInvalidServiceType;
                    goto Exit;
            } // end of switch(pSdoComCon_p->sdoServiceType)
        }
        else // (pSdoComCon_p->transferredBytes > 0)
        {   // continue SDO transfer
            switch(pSdoComCon_p->sdoServiceType)
            {
                // for expedited read is nothing to do
                // -> server sends data

                case kSdoServiceWriteByIndex:
                {   // send next frame
                    if(pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
                    {
                        if(pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE)
                        {   // next segment
                            pPayload = &pCommandFrame->m_le_abCommandData[0];
                            // fill rest of header
                            AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, SDO_MAX_SEGMENT_SIZE);
                            flags = 0x20;
                            AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                            // copy data
                            EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  SDO_MAX_SEGMENT_SIZE);
                            pSdoComCon_p->pData += SDO_MAX_SEGMENT_SIZE;
                            // correct intern counter
                            pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                            pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                            // calc size
                            sizeOfFrame += SDO_MAX_SEGMENT_SIZE;


                        }
                        else
                        {   // end of transfer
                            pPayload = &pCommandFrame->m_le_abCommandData[0];
                            // fill rest of header
                            AmiSetWordToLe( &pCommandFrame->m_le_wSegmentSize, (WORD) pSdoComCon_p->transferSize);
                            flags = 0x30;
                            AmiSetByteToLe( &pCommandFrame->m_le_bFlags, flags);
                            // copy data
                            EPL_MEMCPY( pPayload,pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                            pSdoComCon_p->pData += pSdoComCon_p->transferSize;
                            // calc size
                            sizeOfFrame += pSdoComCon_p->transferSize;
                            // correct intern counter
                            pSdoComCon_p->transferSize = 0;
                            pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;

                        }
                    }
                    else
                    {
                        goto Exit;
                    }
                    break;
                }
                default:
                {
                    goto Exit;
                }
            } // end of switch(pSdoComCon_p->sdoServiceType)
        }
    }
    else
    {
        goto Exit;
    }


    // call send function of lower layer
    switch(pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
        {
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                        sizeOfFrame,
                                        pFrame);
            break;
        }

        default:
        {
            ret = kEplSdoComUnsupportedProt;
        }
    } // end of switch(pSdoComCon_p->sdoProtocolType)


Exit:
    return ret;

}
#endif
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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
static tEplKernel clientProcessFrame(tSdoComConHdl   sdoComConHdl_p,
                                              tAsySdoCom*     pSdoCom_p)
{
tEplKernel          ret;
UINT8               flags;
UINT8               transactionId;
UINT8               command;
UINT                segmentSize;
UINT                dataSize;
ULONG               transferSize;
tSdoComCon*         pSdoComCon;


    ret = kEplSuccessful;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if transaction Id fit
    transactionId = AmiGetByteFromLe(&pSdoCom_p->m_le_bTransactionId);
    if(pSdoComCon->transactionId != transactionId)
    {
        // incorrect transaction id

        // if running transfer
        if((pSdoComCon->transferredBytes != 0)
            && (pSdoComCon->transferSize !=0))
        {
            pSdoComCon->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
            // -> send abort
            clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
            // call callback of application
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
            if((pSdoComCon->transferredBytes != 0)
                && (pSdoComCon->transferSize !=0))
            {
                pSdoComCon->lastAbortCode = EPL_SDOAC_GENERAL_ERROR;
                // -> send abort
                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                // call callback of application
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
            }

        }
        else
        {   // switch on command
            switch(pSdoComCon->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                {   // check if confirmation from server
                    // nothing more to do
                    break;
                }

                case kSdoServiceReadByIndex:
                {   // check if it is an segmented or an expedited transfer
                    flags = AmiGetByteFromLe(&pSdoCom_p->m_le_bFlags);
                    // mask uninteressting bits
                    flags &= 0x30;
                    switch (flags)
                    {
                        // expedited transfer
                        case 0x00:
                        {
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // buffer provided by the application is too small
                                // copy only a part
                                dataSize = pSdoComCon->transferSize;
                            }
                            else
                            {   // buffer fits
                                dataSize = segmentSize;
                            }

                            // copy data
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], dataSize);

                            // correct counter
                            pSdoComCon->transferSize = 0;
                            pSdoComCon->transferredBytes = dataSize;
                            break;
                        }

                        // start of a segmented transfer
                        case 0x10:
                        {   // get total size of transfer including the header
                            transferSize = AmiGetDwordFromLe(&pSdoCom_p->m_le_abCommandData[0]);
                            // subtract size of variable header from data size
                            transferSize -= 4;
                            if (transferSize <= pSdoComCon->transferSize)
                            {   // buffer fits
                                pSdoComCon->transferSize = (UINT)transferSize;
                            }
                            else
                            {   // buffer too small
                                // send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_DATA_TYPE_LENGTH_TOO_HIGH;
                                // -> send abort
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                // call callback of application
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                goto Exit;
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
                        }

                        // segment
                        case 0x20:
                        {
                            // get segment size
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // segment too large
                                // send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_INVALID_BLOCK_SIZE;
                                // -> send abort
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                // call callback of application
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                goto Exit;
                            }
                            // copy data
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], segmentSize);

                            // correct counter an pointer
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize -= segmentSize;
                            break;
                        }

                        // last segment
                        case 0x30:
                        {
                            // get segment size
                            // check size of buffer
                            segmentSize = AmiGetWordFromLe(&pSdoCom_p->m_le_wSegmentSize);
                            // check if data to copy fit to buffer
                            if(segmentSize > pSdoComCon->transferSize)
                            {   // segment too large
                                // send abort
                                pSdoComCon->lastAbortCode = EPL_SDOAC_INVALID_BLOCK_SIZE;
                                // -> send abort
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                // call callback of application
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kEplSdoComTransferTxAborted);
                                goto Exit;
                            }
                            // copy data
                            EPL_MEMCPY(pSdoComCon->pData, &pSdoCom_p->m_le_abCommandData[0], segmentSize);

                            // correct counter an pointer
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize  = 0;

                            break;
                        }
                    }// end of switch(bBuffer & 0x30)

                    break;
                }

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    // $$$ d.k. What should we do?
                    break;
            }// end of switch(pSdoComCon->sdoServiceType)
        }
    }

Exit:
    return ret;
}
#endif

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
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
static tEplKernel clientSendAbort(tSdoComCon* pSdoComCon_p,
                                           UINT32          abortCode_p)
{
tEplKernel      ret;
UINT8           aFrame[SDO_MAX_FRAME_SIZE];
tEplFrame*      pFrame;
tAsySdoCom*     pCommandFrame;
UINT            sizeOfFrame;

    ret = kEplSuccessful;

    pFrame = (tEplFrame*)&aFrame[0];

    EPL_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    // get pointer to command layerpart of frame
    pCommandFrame = &pFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abSdoSeqPayload;
    AmiSetByteToLe( &pCommandFrame->m_le_bCommandId, pSdoComCon_p->sdoServiceType);
    AmiSetByteToLe( &pCommandFrame->m_le_bTransactionId, pSdoComCon_p->transactionId);

    sizeOfFrame = 8;

    // set response and abort flag
    pCommandFrame->m_le_bFlags |= 0x40;

    // copy abortcode to frame
    AmiSetDwordToLe(&pCommandFrame->m_le_abCommandData[0], abortCode_p);

    // set size of segment
    AmiSetWordToLe(&pCommandFrame->m_le_wSegmentSize, sizeof(UINT32));

    // update counter
    pSdoComCon_p->transferredBytes = sizeof(UINT32);
    pSdoComCon_p->transferSize = 0;

    // calc framesize
    sizeOfFrame += sizeof(UINT32);

    // save abort code
    pSdoComCon_p->lastAbortCode = abortCode_p;

    // call send function of lower layer
    switch(pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
        {
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl,
                                        sizeOfFrame,
                                        pFrame);
            if (ret == kEplSdoSeqConnectionBusy)
            {
                DEBUG_LVL_25_TRACE("%s tried to send abort 0x%lX while connection is already closed\n",
                    __func__, (unsigned long) abortCode_p);
                ret = kEplSuccessful;
            }
            break;
        }

        default:
        {
            ret = kEplSdoComUnsupportedProt;
        }
    } // end of switch(pSdoComCon_p->sdoProtocolType)


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
static tEplKernel transferFinished(tSdoComConHdl   sdoComConHdl_p,
                                            tSdoComCon*     pSdoComCon_p,
                                            tSdoComConState SdoComConState_p)
{
tEplKernel      ret;

    ret = kEplSuccessful;

    if(pSdoComCon_p->pfnTransferFinished != NULL)
    {
    tSdoFinishedCb   pfnTransferFinished;
    tSdoComFinished  SdoComFinished;

        SdoComFinished.pUserArg = pSdoComCon_p->pUserArg;
        SdoComFinished.nodeId = pSdoComCon_p->nodeId;
        SdoComFinished.targetIndex = pSdoComCon_p->targetIndex;
        SdoComFinished.targetSubIndex = pSdoComCon_p->targetSubIndex;
        SdoComFinished.transferredBytes = pSdoComCon_p->transferredBytes;
        SdoComFinished.abortCode = pSdoComCon_p->lastAbortCode;
        SdoComFinished.sdoComConHdl = sdoComConHdl_p;
        SdoComFinished.sdoComConState = SdoComConState_p;
        if (pSdoComCon_p->sdoServiceType == kSdoServiceWriteByIndex)
        {
            SdoComFinished.sdoAccessType = kSdoAccessTypeWrite;
        }
        else
        {
            SdoComFinished.sdoAccessType = kSdoAccessTypeRead;
        }

        // reset transfer state so this handle is not busy anymore
        pSdoComCon_p->transferredBytes = 0;
        pSdoComCon_p->transferSize = 0;

        pfnTransferFinished = pSdoComCon_p->pfnTransferFinished;
        // delete function pointer to inform application only once for each transfer
        pSdoComCon_p->pfnTransferFinished = NULL;

        // call application's callback function
        pfnTransferFinished(&SdoComFinished);

    }

    return ret;
}

// EOF

