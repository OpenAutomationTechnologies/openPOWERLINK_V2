/**
********************************************************************************
\file   nmtcnu.c

\brief  Implementation of NMT CNU module

This file contains the implementation of the NMT CNU module.

\ingroup module_nmtcnu
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
#include "EplInc.h"
#include "user/nmtcnu.h"
#include "user/dllucal.h"

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
    tNmtuCheckEventCallback     pfnCheckEventCb;
} tNmtCnuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtCnuInstance   nmtCnuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tNmtCommand   getNmtCommand(tFrameInfo * pFrameInfo_p);
static BOOL             checkNodeIdList(BYTE* pbNmtCommandDate_p);
static tEplKernel       commandCb(tFrameInfo * pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init nmtcnu module

The function initializes an instance of the nmtcnu module

\param  nodeId_p                Node ID of the local node

\return The function returns a tEplKernel error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtcnu_init(UINT nodeId_p)
{
    return nmtcnu_addInstance(nodeId_p);
}

//------------------------------------------------------------------------------
/**
\brief  Add nmtcnu module instance

The function adds a nmtcnu module instance.

\param  nodeId_p                Node ID of the local node

\return The function returns a tEplKernel error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtcnu_addInstance(UINT nodeId_p)
{
    tEplKernel ret = kEplSuccessful;

    EPL_MEMSET(&nmtCnuInstance_g, 0, sizeof (nmtCnuInstance_g));

    nmtCnuInstance_g.nodeId = nodeId_p;

    // register callback-function for NMT-commands
    ret = dllucal_regAsndService(kDllAsndNmtCommand, commandCb, kDllAsndFilterLocal);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete nmtcnu module instance

The function deletes an nmtcnu module instance.

\return The function returns a tEplKernel error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtcnu_delInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    // deregister callback function from DLL
    ret = dllucal_regAsndService(kDllAsndNmtCommand, NULL, kDllAsndFilterNone);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send a NMT-Request to the MN

The function is used to send a NMT-Request to the MN.

\param  nodeId_p            Node ID of the local node.
\param  nmtCommand_p        NMT command to request from MN.

\return The function returns a tEplKernel error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtcnu_sendNmtRequest(UINT nodeId_p, tNmtCommand nmtCommand_p)
{
    tEplKernel      ret;
    tFrameInfo      nmtRequestFrameInfo;
    tEplFrame       nmtRequestFrame;

    ret = kEplSuccessful;

    // build frame
    EPL_MEMSET(&nmtRequestFrame.m_be_abDstMac[0], 0x00, sizeof(nmtRequestFrame.m_be_abDstMac)); // set by DLL
    EPL_MEMSET(&nmtRequestFrame.m_be_abSrcMac[0], 0x00, sizeof(nmtRequestFrame.m_be_abSrcMac)); // set by DLL
    AmiSetWordToBe(&nmtRequestFrame.m_be_wEtherType, EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetByteToLe(&nmtRequestFrame.m_le_bDstNodeId, (BYTE) EPL_C_ADR_MN_DEF_NODE_ID); // node id of the MN
    AmiSetByteToLe(&nmtRequestFrame.m_le_bMessageType, (BYTE)kEplMsgTypeAsnd);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_le_bServiceId, (BYTE) kDllAsndNmtRequest);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_bNmtCommandId,
        (BYTE)nmtCommand_p);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_bTargetNodeId,
        (BYTE)nodeId_p); // target for the nmt command
    EPL_MEMSET(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_abNmtCommandData[0], 0x00, sizeof(nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_abNmtCommandData));

    // build info-structure
    nmtRequestFrameInfo.pFrame = &nmtRequestFrame;
    nmtRequestFrameInfo.frameSize = EPL_C_DLL_MINSIZE_NMTREQ; // sizeof(nmtRequestFrame);

    // send NMT-Request
    ret = dllucal_sendAsyncFrame(&nmtRequestFrameInfo, kDllAsyncReqPrioNmt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register state change callback function

The function registers a callback function to get informed about a
NMT-Change-State-Event.

\param  pfnNmtCheckEventCb_p        Pointer to check event callback function.

\return The function returns a tEplKernel error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtcnu_registerCheckEventCb(tNmtuCheckEventCallback pfnNmtCheckEventCb_p)
{
    nmtCnuInstance_g.pfnCheckEventCb = pfnNmtCheckEventCb_p;
    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT commands

The function processes NMT commands.

\param  pFrameInfo_p        Pointer to frame containing the NMT command

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel commandCb(tFrameInfo* pFrameInfo_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtCommand     nmtCommand;
    BOOL            fNodeIdInList;
    tNmtEvent       nmtEvent = kNmtEventNoEvent;

    if(pFrameInfo_p == NULL)
        return kEplNmtInvalidFramePointer;

    nmtCommand = getNmtCommand(pFrameInfo_p);
    switch(nmtCommand)
    {
        //------------------------------------------------------------------------
        // plain NMT state commands
        case kNmtCmdStartNode:
            nmtEvent = kNmtEventStartNode;
            break;

        case kNmtCmdStopNode:
            nmtEvent = kNmtEventStopNode;
            break;

        case kNmtCmdEnterPreOperational2:
            nmtEvent = kNmtEventEnterPreOperational2;
            break;

        case kNmtCmdEnableReadyToOperate:
            nmtEvent = kNmtEventEnableReadyToOperate;
            break;

        case kNmtCmdResetNode:
            nmtEvent = kNmtEventResetNode;
            break;

        case kNmtCmdResetCommunication:
            nmtEvent = kNmtEventResetCom;
            break;

        case kNmtCmdResetConfiguration:
            nmtEvent = kNmtEventResetConfig;
            break;

        case kNmtCmdSwReset:
            nmtEvent = kNmtEventSwReset;
            break;

        //------------------------------------------------------------------------
        // extended NMT state commands
        case kNmtCmdStartNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&(pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]));
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStartNode;
            }
            break;

        case kNmtCmdStopNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStopNode;
            }
            break;

        case kNmtCmdEnterPreOperational2Ex:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnterPreOperational2;
            }
            break;

        case kNmtCmdEnableReadyToOperateEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnableReadyToOperate;
            }
            break;

        case kNmtCmdResetNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetNode;
            }
            break;

        case kNmtCmdResetCommunicationEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetCom;
            }
            break;

        case kNmtCmdResetConfigurationEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetConfig;
            }
            break;

        case kNmtCmdSwResetEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventSwReset;
            }
            break;

        //------------------------------------------------------------------------
        // NMT managing commands
        // TODO: add functions to process managing command (optional)
        case kNmtCmdNetHostNameSet:
            break;

        case kNmtCmdFlushArpEntry:
            break;

        //------------------------------------------------------------------------
        // NMT info services
        // TODO: forward event with infos to the application (optional)
        case kNmtCmdPublishConfiguredCN:
            break;

        case kNmtCmdPublishActiveCN:
            break;

        case kNmtCmdPublishPreOperational1:
            break;

        case kNmtCmdPublishPreOperational2:
            break;

        case kNmtCmdPublishReadyToOperate:
            break;

        case kNmtCmdPublishOperational:
            break;

        case kNmtCmdPublishStopped:
            break;

        case kNmtCmdPublishEmergencyNew:
            break;

        case kNmtCmdPublishTime:
            break;

        //-----------------------------------------------------------------------
        // error from MN
        // -> requested command not supported by MN
        case kNmtCmdInvalidService:
            // TODO: errorevent to application
            break;

        //------------------------------------------------------------------------
        // default
        default:
            return kEplNmtUnknownCommand;
            break;
    } // end of switch(NmtCommand)

    if (nmtEvent != kNmtEventNoEvent)
    {
        if (nmtCnuInstance_g.pfnCheckEventCb != NULL)
        {
            ret = nmtCnuInstance_g.pfnCheckEventCb(nmtEvent);
            if (ret == kEplReject)
            {
                return kEplSuccessful;
            }
            else if (ret != kEplSuccessful)
            {
                return ret;
            }
        }
        ret = nmtu_postNmtEvent(nmtEvent);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get NMT command

The function extracts the nmt command from the frame.

\param  pFrameInfo_p        Pointer to frame containing the NMT command

\return The function returns the extracted NMT command
*/
//------------------------------------------------------------------------------
static tNmtCommand getNmtCommand(tFrameInfo* pFrameInfo_p)
{
    tNmtCommand          nmtCommand;
    tEplNmtCommandService*  pNmtCommandService;

    pNmtCommandService = &pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService;
    nmtCommand = (tNmtCommand)AmiGetByteFromLe(&pNmtCommandService->m_le_bNmtCommandId);

    return nmtCommand;
}

//------------------------------------------------------------------------------
/**
\brief  Check node ID list

The function checks if the own node ID is set in the node list.

\param  pbNmtCommandDate_p        Pointer to the date of the NMT command.

\return The function returns \b TRUE if the node is found in the node list or
        \b FALSE if it is not found in the node list.
*/
//------------------------------------------------------------------------------
static BOOL checkNodeIdList(UINT8* pbNmtCommandDate_p)
{
    BOOL            fNodeIdInList;
    UINT            byteOffset;
    UINT8           bitOffset;
    UINT8           nodeListByte;

    // get byte-offset of the own nodeid in NodeIdList
    // devide though 8
    byteOffset = (UINT)(nmtCnuInstance_g.nodeId >> 3);
    // get bitoffset
    bitOffset = (UINT8) nmtCnuInstance_g.nodeId % 8;

    nodeListByte = AmiGetByteFromLe(&pbNmtCommandDate_p[byteOffset]);
    if((nodeListByte & bitOffset) == 0)
        fNodeIdInList = FALSE;
    else
        fNodeIdInList = TRUE;

    return  fNodeIdInList;
}

///\}

