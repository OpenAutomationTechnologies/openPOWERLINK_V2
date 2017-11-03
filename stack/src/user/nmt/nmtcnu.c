/**
********************************************************************************
\file   nmtcnu.c

\brief  Implementation of NMT CNU module

This file contains the implementation of the NMT CNU module.

\ingroup module_nmtcnu
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
#include <user/nmtcnu.h>
#include <user/dllucal.h>
#include <common/ami.h>
#include <oplk/frame.h>

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
typedef struct
{
    UINT                        nodeId;
    UINT                        extNmtCmdByteOffset;
    UINT8                       extNmtCmdBitMask;
    tNmtuCheckEventCallback     pfnCheckEventCb;
} tNmtCnuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtCnuInstance  nmtCnuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init nmtcnu module

The function initializes an instance of the nmtcnu module.

\param[in]      nodeId_p            Node ID of the local node.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_init(UINT nodeId_p)
{
    tOplkError  ret = kErrorOk;

    OPLK_MEMSET(&nmtCnuInstance_g, 0, sizeof(nmtCnuInstance_g));

    nmtCnuInstance_g.nodeId = nodeId_p;

    // Byte offset --> nodeid divide by 8
    // Bit offset  --> 2 ^ (nodeid AND 0b111)
    nmtCnuInstance_g.extNmtCmdByteOffset = (UINT)(nodeId_p >> 3);
    nmtCnuInstance_g.extNmtCmdBitMask = 1 << ((UINT8)nodeId_p & 7);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down nmtcnu module instance

The function shuts down the nmtcnu module instance.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_exit(void)
{
    tOplkError  ret;

    // deregister callback function from DLL
    ret = dllucal_regAsndService(kDllAsndNmtCommand, NULL, kDllAsndFilterNone);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an NMT-Request to the MN

The function is used to send an NMT-Request to the MN.

\param[in]      nodeId_p            Node ID of the local node.
\param[in]      nmtCommand_p        NMT command to request from MN.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_sendNmtRequest(UINT nodeId_p, tNmtCommand nmtCommand_p)
{
    return nmtcnu_sendNmtRequestEx(nodeId_p, nmtCommand_p, NULL, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Send an NMT-Request with command data to the MN

The function is used to send an NMT-Request to the MN.

\param[in]      nodeId_p            Node ID of the local node.
\param[in]      nmtCommand_p        NMT command to request from MN.
\param[in]      pNmtCommandData_p   Pointer to NMT command data (32 Byte).
\param[in]      dataSize_p          Size of NMT command data.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_sendNmtRequestEx(UINT nodeId_p,
                                   tNmtCommand nmtCommand_p,
                                   const void* pNmtCommandData_p,
                                   size_t dataSize_p)
{
    tOplkError  ret = kErrorOk;
    tFrameInfo  nmtRequestFrameInfo;
    tPlkFrame   nmtRequestFrame;

    // check NMTCmdData size for correct length
    if (dataSize_p > C_MAX_NMT_CMD_DATA_SIZE)
        return kErrorNmtInvalidParam;

    // build frame
    OPLK_MEMSET(&nmtRequestFrame.aDstMac[0], 0x00, sizeof(nmtRequestFrame.aDstMac)); // set by DLL
    OPLK_MEMSET(&nmtRequestFrame.aSrcMac[0], 0x00, sizeof(nmtRequestFrame.aSrcMac)); // set by DLL
    ami_setUint16Be(&nmtRequestFrame.etherType, C_DLL_ETHERTYPE_EPL);
    ami_setUint8Le(&nmtRequestFrame.dstNodeId, (UINT8)C_ADR_MN_DEF_NODE_ID); // node id of the MN
    ami_setUint8Le(&nmtRequestFrame.messageType, (UINT8)kMsgTypeAsnd);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.serviceId, (UINT8)kDllAsndNmtRequest);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.payload.nmtRequestService.nmtCommandId,
                   (UINT8)nmtCommand_p);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.payload.nmtRequestService.targetNodeId,
                   (UINT8)nodeId_p); // target for the nmt command

    if (pNmtCommandData_p && (dataSize_p != 0))
    {
        OPLK_MEMCPY(&nmtRequestFrame.data.asnd.payload.nmtRequestService.aNmtCommandData[0],
                    pNmtCommandData_p,
                    dataSize_p);
    }
    // build info-structure
    nmtRequestFrameInfo.frame.pBuffer = &nmtRequestFrame;
    nmtRequestFrameInfo.frameSize = C_DLL_MINSIZE_NMTREQ + (UINT)dataSize_p;

    // send NMT request
    ret = dllucal_sendAsyncFrame(&nmtRequestFrameInfo, kDllAsyncReqPrioNmt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register state change callback function

The function registers a callback function to get informed about an
NMT-Change-State-Event.

\param[in]      pfnNmtCheckEventCb_p    Pointer to check event callback function.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_registerCheckEventCb(tNmtuCheckEventCallback pfnNmtCheckEventCb_p)
{
    nmtCnuInstance_g.pfnCheckEventCb = pfnNmtCheckEventCb_p;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
