/**
********************************************************************************
\file   sdoasnd.c

\brief  Implementation of SDO over ASnd protocol

This file contains the implementation of the SDO over ASnd protocol.

\ingroup module_sdo_asnd
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <common/oplkinc.h>
#include <user/sdoasnd.h>
#include <user/dllucal.h>
#include <common/ami.h>

#if defined(CONFIG_INCLUDE_SDO_ASND)

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef CONFIG_SDO_MAX_CONNECTION_ASND
#define CONFIG_SDO_MAX_CONNECTION_ASND     5
#endif

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

// instance table
typedef struct
{
    UINT                aSdoAsndConnection[CONFIG_SDO_MAX_CONNECTION_ASND];
    tSequLayerReceiveCb pfnSdoAsySeqCb;
} tSdoAsndInstance;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoAsndInstance  sdoAsndInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError sdoAsndCb(const tFrameInfo* pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize first instance of SDO over ASnd module

The function initializes a first instance of the SDO over ASnd module.

\param[in]      pfnReceiveCb_p      Pointer to SDO sequence layer callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_asnd
*/
//------------------------------------------------------------------------------
tOplkError sdoasnd_init(tSequLayerReceiveCb pfnReceiveCb_p)
{
    tOplkError  ret;

    OPLK_MEMSET(&sdoAsndInstance_l, 0x00, sizeof(sdoAsndInstance_l));

    if (pfnReceiveCb_p != NULL)
        sdoAsndInstance_l.pfnSdoAsySeqCb = pfnReceiveCb_p;
    else
    {
        return kErrorSdoUdpMissCb; //TODO: Wrong error code?
    }

    ret = dllucal_regAsndService(kDllAsndSdo, sdoAsndCb, kDllAsndFilterLocal);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down SDO over ASnd module

The function shuts down the SDO over ASnd module.

\return The function returns a tOplkError error code.

\ingroup module_sdo_asnd
*/
//------------------------------------------------------------------------------
tOplkError sdoasnd_exit(void)
{
    tOplkError  ret;

    ret = dllucal_regAsndService(kDllAsndSdo, NULL, kDllAsndFilterNone);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Init SDO over ASnd connection

The function initializes a SDO over ASnd connection to a node.

\param[out]     pSdoConHandle_p     Pointer to store the connection handle for this
                                    new connection.
\param[in]      targetNodeId_p      Node ID of the target to connect to.

\return The function returns a tOplkError error code.

\ingroup module_sdo_asnd
*/
//------------------------------------------------------------------------------
tOplkError sdoasnd_initCon(tSdoConHdl* pSdoConHandle_p, UINT targetNodeId_p)
{
    tOplkError  ret = kErrorOk;
    UINT        count;
    UINT        freeCon;
    UINT*       pConnection;

    // Check parameter validity
    ASSERT(pSdoConHandle_p != NULL);

    if ((targetNodeId_p == C_ADR_INVALID) ||
        (targetNodeId_p >= C_ADR_BROADCAST))
        return kErrorSdoAsndInvalidNodeId;

    // get free entry in control structure
    count = 0;
    freeCon = CONFIG_SDO_MAX_CONNECTION_ASND;
    pConnection = &sdoAsndInstance_l.aSdoAsndConnection[0];
    while (count < CONFIG_SDO_MAX_CONNECTION_ASND)
    {
        if (*pConnection == targetNodeId_p)
        {   // existing connection to target node found
            // save handle for higher layer
            *pSdoConHandle_p = (tSdoConHdl)(count | SDO_ASND_HANDLE);
            return ret;
        }
        else if (*pConnection == 0)
        {   // free entry-> save target nodeId
            freeCon = count;
        }

        count++;
        pConnection++;
    }

    if (freeCon == CONFIG_SDO_MAX_CONNECTION_ASND)
    {
        // no free connection
        ret = kErrorSdoAsndNoFreeHandle;
    }
    else
    {
        pConnection = &sdoAsndInstance_l.aSdoAsndConnection[freeCon];
        *pConnection = targetNodeId_p;
        // save handle for higher layer
        *pSdoConHandle_p = (tSdoConHdl)(freeCon | SDO_ASND_HANDLE);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send data via an existing connection

The function sends data via an existing SDO over ASnd connection.

\param[in]      sdoConHandle_p      Connection handle of the connection to use.
\param[in,out]  pSrcData_p          Pointer to data which shall be sent.
\param[in]      dataSize_p          Size of data to be sent.

\return The function returns a tOplkError error code.

\ingroup module_sdo_asnd
*/
//------------------------------------------------------------------------------
tOplkError sdoasnd_sendData(tSdoConHdl sdoConHandle_p,
                            tPlkFrame* pSrcData_p,
                            size_t dataSize_p)
{
    tOplkError  ret;
    UINT        array;
    tFrameInfo  frameInfo;

    array = ((UINT)sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);

    if (array > CONFIG_SDO_MAX_CONNECTION_ASND)
        return kErrorSdoAsndInvalidHandle;

    // fill Asnd header
    // own node id not needed -> filled by DLL
    ami_setUint8Le(&pSrcData_p->messageType, (UINT8)kMsgTypeAsnd);      // ASnd == 0x06
    ami_setUint8Le(&pSrcData_p->dstNodeId, (UINT8)sdoAsndInstance_l.aSdoAsndConnection[array]);
    ami_setUint8Le(&pSrcData_p->srcNodeId, 0x00);                       // set source-nodeid (filled by DLL 0)
    // calc size (add Ethernet and ASnd header size)
    dataSize_p += (size_t)((UINT8*)&pSrcData_p->data.asnd.payload.sdoSequenceFrame - (UINT8*)pSrcData_p);

    // send function of DLL
    frameInfo.frameSize = (UINT)dataSize_p;
    frameInfo.frame.pBuffer = pSrcData_p;

    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete an existing connection

The function deletes an existing SDO over ASnd connection.

\param[in]      sdoConHandle_p      Connection handle of the connection to delete.

\return The function returns a tOplkError error code.

\ingroup module_sdo_asnd
*/
//------------------------------------------------------------------------------
tOplkError sdoasnd_deleteCon(tSdoConHdl sdoConHandle_p)
{
    tOplkError  ret = kErrorOk;
    UINT        array;

    array = ((UINT)sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if (array > CONFIG_SDO_MAX_CONNECTION_ASND)
        return kErrorSdoAsndInvalidHandle;

    // set target nodeId to 0
    sdoAsndInstance_l.aSdoAsndConnection[array] = 0;

    return ret;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for ASnd frames

The function implements the callback function which should be called when
receiving ASnd frames.

\param[in]      pFrameInfo_p        Pointer to received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoAsndCb(const tFrameInfo* pFrameInfo_p)
{
    tOplkError      ret = kErrorOk;
    UINT            count;
    UINT*           pConnection;
    UINT            nodeId;
    UINT            freeEntry = 0xFFFF;
    tSdoConHdl      sdoConHdl;
    tPlkFrame*      pFrame;

    pFrame = pFrameInfo_p->frame.pBuffer;
    nodeId = ami_getUint8Le(&pFrame->srcNodeId);

    // search corresponding entry in control structure
    count = 0;
    pConnection = &sdoAsndInstance_l.aSdoAsndConnection[0];
    while (count < CONFIG_SDO_MAX_CONNECTION_ASND)
    {
        if (nodeId == *pConnection)
            break;
        else if ((*pConnection == 0) && (freeEntry == 0xFFFF))
        {   // free entry
            freeEntry = count;
        }

        count++;
        pConnection++;
    }

    if (count == CONFIG_SDO_MAX_CONNECTION_ASND)
    {
        if (freeEntry != 0xFFFF)
        {
            pConnection = &sdoAsndInstance_l.aSdoAsndConnection[freeEntry];
            *pConnection = nodeId;
            count = freeEntry;
        }
        else
        {
            DEBUG_LVL_SDO_TRACE("%s(): no free handle\n", __func__);
            return ret;
        }
    }

    sdoConHdl = (tSdoConHdl)(count | SDO_ASND_HANDLE);
    sdoAsndInstance_l.pfnSdoAsySeqCb(sdoConHdl,
                                     &pFrame->data.asnd.payload.sdoSequenceFrame,
                                     (pFrameInfo_p->frameSize - 18));

    return ret;
}

/// \}

#endif
