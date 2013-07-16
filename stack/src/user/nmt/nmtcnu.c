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

#if defined(CONFIG_INCLUDE_NMT_CN)

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
    tEplNmtuCheckEventCallback  pfnCheckEventCb;
} tNmtCnuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtCnuInstance   nmtCnuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplNmtCommand   getNmtCommand(tEplFrameInfo * pFrameInfo_p);
static BOOL             checkNodeIdList(BYTE* pbNmtCommandDate_p);
static tEplKernel       commandCb(tEplFrameInfo * pFrameInfo_p);

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
#if defined(CONFIG_INCLUDE_DLLU)
    ret = dllucal_regAsndService(kEplDllAsndNmtCommand, commandCb, kEplDllAsndFilterLocal);
#endif

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

#if defined(CONFIG_INCLUDE_DLLU)
    // deregister callback function from DLL
    ret = dllucal_regAsndService(kEplDllAsndNmtCommand, NULL, kEplDllAsndFilterNone);
#endif

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
tEplKernel nmtcnu_sendNmtRequest(UINT nodeId_p, tEplNmtCommand nmtCommand_p)
{
    tEplKernel      ret;
    tEplFrameInfo   nmtRequestFrameInfo;
    tEplFrame       nmtRequestFrame;

    ret = kEplSuccessful;

    // build frame
    EPL_MEMSET(&nmtRequestFrame.m_be_abDstMac[0], 0x00, sizeof(nmtRequestFrame.m_be_abDstMac)); // set by DLL
    EPL_MEMSET(&nmtRequestFrame.m_be_abSrcMac[0], 0x00, sizeof(nmtRequestFrame.m_be_abSrcMac)); // set by DLL
    AmiSetWordToBe(&nmtRequestFrame.m_be_wEtherType, EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetByteToLe(&nmtRequestFrame.m_le_bDstNodeId, (BYTE) EPL_C_ADR_MN_DEF_NODE_ID); // node id of the MN
    AmiSetByteToLe(&nmtRequestFrame.m_le_bMessageType, (BYTE)kEplMsgTypeAsnd);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_le_bServiceId, (BYTE) kEplDllAsndNmtRequest);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_bNmtCommandId,
        (BYTE)nmtCommand_p);
    AmiSetByteToLe(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_bTargetNodeId,
        (BYTE)nodeId_p); // target for the nmt command
    EPL_MEMSET(&nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_abNmtCommandData[0], 0x00, sizeof(nmtRequestFrame.m_Data.m_Asnd.m_Payload.m_NmtRequestService.m_le_abNmtCommandData));

    // build info-structure
    nmtRequestFrameInfo.m_pFrame = &nmtRequestFrame;
    nmtRequestFrameInfo.m_uiFrameSize = EPL_C_DLL_MINSIZE_NMTREQ; // sizeof(nmtRequestFrame);

    // send NMT-Request
#if defined(CONFIG_INCLUDE_DLLU)
    ret = dllucal_sendAsyncFrame(&nmtRequestFrameInfo, kEplDllAsyncReqPrioNmt);
#endif

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
tEplKernel nmtcnu_registerCheckEventCb(tEplNmtuCheckEventCallback pfnNmtCheckEventCb_p)
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
static tEplKernel commandCb(tEplFrameInfo* pFrameInfo_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplNmtCommand  nmtCommand;
    BOOL            fNodeIdInList;
    tEplNmtEvent    nmtEvent = kEplNmtEventNoEvent;

    if(pFrameInfo_p == NULL)
        return kEplNmtInvalidFramePointer;

    nmtCommand = getNmtCommand(pFrameInfo_p);
    switch(nmtCommand)
    {
        //------------------------------------------------------------------------
        // plain NMT state commands
        case kEplNmtCmdStartNode:
            nmtEvent = kEplNmtEventStartNode;
            break;

        case kEplNmtCmdStopNode:
            nmtEvent = kEplNmtEventStopNode;
            break;

        case kEplNmtCmdEnterPreOperational2:
            nmtEvent = kEplNmtEventEnterPreOperational2;
            break;

        case kEplNmtCmdEnableReadyToOperate:
            nmtEvent = kEplNmtEventEnableReadyToOperate;
            break;

        case kEplNmtCmdResetNode:
            nmtEvent = kEplNmtEventResetNode;
            break;

        case kEplNmtCmdResetCommunication:
            nmtEvent = kEplNmtEventResetCom;
            break;

        case kEplNmtCmdResetConfiguration:
            nmtEvent = kEplNmtEventResetConfig;
            break;

        case kEplNmtCmdSwReset:
            nmtEvent = kEplNmtEventSwReset;
            break;

        //------------------------------------------------------------------------
        // extended NMT state commands
        case kEplNmtCmdStartNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&(pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]));
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventStartNode;
            }
            break;

        case kEplNmtCmdStopNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventStopNode;
            }
            break;

        case kEplNmtCmdEnterPreOperational2Ex:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventEnterPreOperational2;
            }
            break;

        case kEplNmtCmdEnableReadyToOperateEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventEnableReadyToOperate;
            }
            break;

        case kEplNmtCmdResetNodeEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventResetNode;
            }
            break;

        case kEplNmtCmdResetCommunicationEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventResetCom;
            }
            break;

        case kEplNmtCmdResetConfigurationEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventResetConfig;
            }
            break;

        case kEplNmtCmdSwResetEx:
            // check if own nodeid is in EPL node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0]);
            if(fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kEplNmtEventSwReset;
            }
            break;

        //------------------------------------------------------------------------
        // NMT managing commands
        // TODO: add functions to process managing command (optional)
        case kEplNmtCmdNetHostNameSet:
            break;

        case kEplNmtCmdFlushArpEntry:
            break;

        //------------------------------------------------------------------------
        // NMT info services
        // TODO: forward event with infos to the application (optional)
        case kEplNmtCmdPublishConfiguredCN:
            break;

        case kEplNmtCmdPublishActiveCN:
            break;

        case kEplNmtCmdPublishPreOperational1:
            break;

        case kEplNmtCmdPublishPreOperational2:
            break;

        case kEplNmtCmdPublishReadyToOperate:
            break;

        case kEplNmtCmdPublishOperational:
            break;

        case kEplNmtCmdPublishStopped:
            break;

        case kEplNmtCmdPublishEmergencyNew:
            break;

        case kEplNmtCmdPublishTime:
            break;

        //-----------------------------------------------------------------------
        // error from MN
        // -> requested command not supported by MN
        case kEplNmtCmdInvalidService:
            // TODO: errorevent to application
            break;

        //------------------------------------------------------------------------
        // default
        default:
            return kEplNmtUnknownCommand;
            break;
    } // end of switch(NmtCommand)

    if (nmtEvent != kEplNmtEventNoEvent)
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
#if defined(CONFIG_INCLUDE_NMTU)
        ret = EplNmtuNmtEvent(nmtEvent);
#endif
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
static tEplNmtCommand getNmtCommand(tEplFrameInfo* pFrameInfo_p)
{
    tEplNmtCommand          nmtCommand;
    tEplNmtCommandService*  pNmtCommandService;

    pNmtCommandService = &pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService;
    nmtCommand = (tEplNmtCommand)AmiGetByteFromLe(&pNmtCommandService->m_le_bNmtCommandId);

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

#endif // #if defined(CONFIG_INCLUDE_NMT_CN)

