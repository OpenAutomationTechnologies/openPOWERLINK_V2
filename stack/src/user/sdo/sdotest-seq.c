/**
********************************************************************************
\file   sdotest-seq.c

\brief  SDO sequence layer test functions

This file contains the implementation of the SDO Test Sequence Layer.

\ingroup module_sdotest_seq
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/sdotest.h>

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

/**
\brief  SDO sequence layer function pointer struct

This structure defines the connection control structure of the sequence layer.
*/
typedef struct
{
    tOplkError (*pfnInit)(tSequLayerReceiveCb     receiveCb_p);    ///< Init function pointer
    tOplkError (*pfnExit)(void);                                   ///< Exit function pointer
    tOplkError (*pfnInitCon)(tSdoConHdl*          sdoConHandle_p,
                             UINT                 targetNodeId_p); ///< Init Connection function pointer
    tOplkError (*pfnSendData)(tSdoConHdl          sdoConHandle_p,  ///< Send Data function pointer
                              tPlkFrame*          pSrcData_p,
                              UINT32              dwDataSize_p);   ///< Send Data function pointer
    tOplkError (*pfnDelCon)(tSdoConHdl            sdoConHandle_p); ///< Delete Connection function pointer
} tSdoTestSeqFunc;

/**
\brief  SDO sequence layer states

This enumeration defines all valid states for the SDO command layer state
machine.
*/
typedef enum
{
    kOplkTestSdoSequConIdle,                 ///< Lower layer connection is unused
    kOplkTestSdoSequConActive,               ///< Lower layer connection is used
} eSdoTestSeqConState;

/**
\brief SDO sequence layer state data type

Data type for the enumerator \ref eSdoTestSeqConState.
*/
typedef UINT32 tSdoTestSeqConState;

/**
\brief  SDO sequence layer connection control structure

This structure is used by the SDO sequence layer to save connection information.
*/
typedef struct
{
    UINT                        nodeId;       ///< Target node id
    tSdoType                    sdoType;      ///< Type of SDO transfer (Asnd/UDP)
    tSdoConHdl                  sdoConHandle; ///< Lower layer connection handle
    tSdoTestSeqConState         state;        ///< State of this test connection
    tSdoTestSeqFunc*            pFuncTable;   ///< Current function table (depends on SDO type)
} tSdoTestSeqCon;

/**
\brief  SDO sequence layer instance structure

This structure defines a SDO sequence layer instance.
*/
typedef struct
{
    tSdoTestSeqCon              seqCon;       ///< SDO sequence layer test connection
    sdoApiCbSeqTest             apiCb;        ///< API callback
} tSdoTestSeq;

/**
\brief  SDO sequence layer udp functions

This array sets the function pointers to the udp functions.
*/
#if defined(CONFIG_INCLUDE_SDO_UDP)
tSdoTestSeqFunc sdoTestSeqUdpFuncs =
{
    sdoudp_init,
    sdoudp_exit,
    sdoudp_initCon,
    sdoudp_sendData,
    sdoudp_delConnection
};
#endif

/**
\brief  SDO sequence layer asnd functions

This array sets the function pointers to the asnd functions.
*/
#if defined(CONFIG_INCLUDE_SDO_ASND)
tSdoTestSeqFunc sdoTestSeqAsndFuncs =
{
    sdoasnd_init,
    sdoasnd_exit,
    sdoasnd_initCon,
    sdoasnd_sendData,
    sdoasnd_deleteCon
};
#endif

static tSdoTestSeq sdoTestSeqInst;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
tOplkError sdotestseq_conCb(tSdoConHdl sdoConHdl_p, tAsySdoSeq* pSdoSeqData_p,
                            UINT dataSize_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the SDO sequence layer

This function initializes the SDO sequence layer test.

\param  sdoSequCbApi_p         pointer to the Sequence layer callback function

\return The function returns a tOplkError error code.

\ingroup module_sdotest_seq
*/
//------------------------------------------------------------------------------
tOplkError sdotestseq_init(sdoApiCbSeqTest sdoSequCbApi_p)
{
    tOplkError  ret;

    ret = kErrorOk;

    // Check parameters
    if (sdoSequCbApi_p == NULL)
    {
        return kErrorInvalidOperation;
    }

    // Init module
    OPLK_MEMSET(&sdoTestSeqInst, 0, sizeof(sdoTestSeqInst));

    sdoTestSeqInst.apiCb = sdoSequCbApi_p;

    // Init connection handling
    sdoTestSeqInst.seqCon.state = kOplkTestSdoSequConIdle;

    // Init sub modules
#if defined (CONFIG_INCLUDE_SDO_ASND)
    ret = sdoasnd_init(sdotestseq_conCb);
    if (ret != kErrorOk)
    {
        sdoasnd_exit();
        return ret;
    }
#endif

#if defined (CONFIG_INCLUDE_SDO_UDP)
    ret = sdoudp_init(sdotestseq_conCb);
    if (ret != kErrorOk)
    {
        sdoudp_exit();
        return ret;
    }
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down the SDO sequence layer

This function shuts down the SDO sequence layer.

\return The function returns a tOplkError error code.

\ingroup module_sdotest_seq
*/
//------------------------------------------------------------------------------
tOplkError sdotestseq_exit(void)
{
    tSdoTestSeqCon* pCon;
#if defined (CONFIG_INCLUDE_SDO_ASND)
    tOplkError retAsnd;
#endif
#if defined (CONFIG_INCLUDE_SDO_UDP)
    tOplkError retUdp;
#endif

    // Shutdown sub modules
#if defined (CONFIG_INCLUDE_SDO_ASND)
    retAsnd = sdoasnd_exit();
#endif

#if defined (CONFIG_INCLUDE_SDO_UDP)
    retUdp = sdoudp_exit();
#endif

    // Shutdown connection handling
    pCon = &sdoTestSeqInst.seqCon;

    if (pCon->state != kOplkTestSdoSequConIdle)
    {
        switch (pCon->sdoType)
        {
            case kSdoTypeUdp:

#if defined (CONFIG_INCLUDE_SDO_UDP)
                sdoudp_delConnection(pCon->sdoConHandle);
#endif
                break;

            case kSdoTypeAsnd:

#if defined (CONFIG_INCLUDE_SDO_ASND)
                sdoasnd_deleteCon(pCon->sdoConHandle);
#endif
                break;

            default:
            case kSdoTypeAuto:
            case kSdoTypePdo:

                // Wrong SDO type -->  ignore
                break;
        }
    }
    pCon->state = kOplkTestSdoSequConIdle;

    // Check return values, return first error
#if defined (CONFIG_INCLUDE_SDO_ASND)
    if (retAsnd != kErrorOk)
    {
        return retAsnd;
    }
#endif

#if defined (CONFIG_INCLUDE_SDO_UDP)
    if (retUdp != kErrorOk)
    {
        return retUdp;
    }
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  SDO sequence layer send frame

This function sends an SDO sequence layer frame.

\param  nodeId_p         Node ID of target node
\param  sdoType_p        Type of SDO lower layer (Asnd/Udp)
\param  pSdoSeq_p        SDO sequence layer frame
\param  sdoSize_p        Size of SDO sequence layer frame

\return The function returns a tOplkError error code.

\ingroup module_sdotest_seq
*/
//------------------------------------------------------------------------------
tOplkError sdotestseq_sendFrame(UINT nodeId_p, tSdoType sdoType_p, tAsySdoSeq* pSdoSeq_p,
                                size_t sdoSize_p)
{
    tOplkError           ret = kErrorOk;
    tSdoTestSeqCon*      pCon;
    size_t               FrameSize;
    tPlkFrame*           pFrame;
    tAsySdoSeq*          pSequDst;

    // Check parameters
    FrameSize = sdoSize_p + PLK_FRAME_OFFSET_SDO_SEQU;
    if (FrameSize > C_DLL_MAX_ASYNC_MTU)
    {
        return kErrorInvalidOperation;
    }

    // Try to get a valid lower layer connection
    pCon = &sdoTestSeqInst.seqCon;
    if (pCon->state == kOplkTestSdoSequConIdle)
    {
        // We need a new connection
        switch (sdoType_p)
        {
            case kSdoTypeUdp:

#if defined (CONFIG_INCLUDE_SDO_UDP)
                ret = sdoudp_initCon(&pCon->sdoConHandle, nodeId_p);
#else
                ret = kErrorSdoSeqUnsupportedProt;
#endif
                if (ret != kErrorOk)
                {
                    return ret;
                }

#if defined (CONFIG_INCLUDE_SDO_UDP)
                pCon->pFuncTable = &sdoTestSeqUdpFuncs;
#endif
                break;

            case kSdoTypeAsnd:

#if defined (CONFIG_INCLUDE_SDO_ASND)
                ret = sdoasnd_initCon(&pCon->sdoConHandle, nodeId_p);
#else
                ret = kErrorSdoSeqUnsupportedProt;
#endif
                if (ret != kErrorOk)
                {
                    return ret;
                }

#if defined (CONFIG_INCLUDE_SDO_ASND)
                pCon->pFuncTable = &sdoTestSeqAsndFuncs;
#endif
                break;

            default:
            case kSdoTypeAuto:
            case kSdoTypePdo:

                // Current implementation only supports Asnd and UDP
                return kErrorSdoSeqUnsupportedProt;
        }

        // Save parameters
        pCon->state   = kOplkTestSdoSequConActive;
        pCon->sdoType = sdoType_p;
        pCon->nodeId  = nodeId_p;
    }
    else
    {
        // Connection exists, check parameters
        if ((nodeId_p != pCon->nodeId) ||
            (sdoType_p != pCon->sdoType))
        {
            return kErrorInvalidOperation;
        }
    }

    // Get frame buffer
    pFrame = (tPlkFrame*)OPLK_MALLOC(FrameSize);
    if (pFrame == NULL)
    {
        ret = kErrorNoResource;
    }
    else
    {
        // Set up frame
        OPLK_MEMSET(pFrame, 0, FrameSize);

        pSequDst = &pFrame->data.asnd.payload.sdoSequenceFrame;
        OPLK_MEMCPY(pSequDst, pSdoSeq_p, sdoSize_p);

        ami_setUint8Le(&pFrame->data.asnd.serviceId, (BYTE)kDllAsndSdo);

        // Send data
        ret = pCon->pFuncTable->pfnSendData(pCon->sdoConHandle, pFrame, sdoSize_p);

        OPLK_FREE(pFrame);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO sequence layer close connection

This function closes a SDO sequence layer connection

\return The function returns a tOplkError error code.

\ingroup module_sdotest_seq
*/
//------------------------------------------------------------------------------
tOplkError sdotestseq_closeCon(void)
{
    tOplkError           ret = kErrorOk;
    tSdoTestSeqCon*      pCon;

    pCon = &sdoTestSeqInst.seqCon;
    ret = pCon->pFuncTable->pfnDelCon(pCon->sdoConHandle);
    pCon->state = kOplkTestSdoSequConIdle;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  SDO sequence layer callback

Callback to get called by lower layers on
reception of SDO sequence layer frames.

\param  sdoConHdl_p       Connection handle
\param  pSdoSeqData_p     SDO sequence layer frame
\param  dataSize_p        Size of SDO sequence layer frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError  sdotestseq_conCb(tSdoConHdl sdoConHdl_p, tAsySdoSeq* pSdoSeqData_p,
                             UINT dataSize_p)
{
    tOplkError ret = kErrorOk;

    // Ignore unused parameters
    (void)sdoConHdl_p;

    // Forward frame to API layer
    if (sdoTestSeqInst.apiCb != NULL)
    {
        ret = sdoTestSeqInst.apiCb(pSdoSeqData_p, dataSize_p);
    }
    else
    {
        ret = kErrorOk;
    }

    return ret;
}

/// \}
