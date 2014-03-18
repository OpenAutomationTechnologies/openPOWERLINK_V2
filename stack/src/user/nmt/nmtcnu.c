/**
********************************************************************************
\file   nmtcnu.c

\brief  Implementation of NMT CNU module

This file contains the implementation of the NMT CNU module.

\ingroup module_nmtcnu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/ami.h>
#include <user/nmtcnu.h>
#include <user/dllucal.h>

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
static tNmtCommand getNmtCommand(tFrameInfo* pFrameInfo_p);
static BOOL checkNodeIdList(BYTE* pbNmtCommandDate_p);
static tOplkError commandCb(tFrameInfo* pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init nmtcnu module

The function initializes an instance of the nmtcnu module

\param  nodeId_p                Node ID of the local node

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_init(UINT nodeId_p)
{
    return nmtcnu_addInstance(nodeId_p);
}

//------------------------------------------------------------------------------
/**
\brief  Add nmtcnu module instance

The function adds a nmtcnu module instance.

\param  nodeId_p                Node ID of the local node

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_addInstance(UINT nodeId_p)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&nmtCnuInstance_g, 0, sizeof(nmtCnuInstance_g));

    nmtCnuInstance_g.nodeId = nodeId_p;

    // register callback-function for NMT-commands
    ret = dllucal_regAsndService(kDllAsndNmtCommand, commandCb, kDllAsndFilterLocal);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete nmtcnu module instance

The function deletes an nmtcnu module instance.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_delInstance(void)
{
    tOplkError ret = kErrorOk;

    // deregister callback function from DLL
    ret = dllucal_regAsndService(kDllAsndNmtCommand, NULL, kDllAsndFilterNone);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an NMT-Request to the MN

The function is used to send an NMT-Request to the MN.

\param  nodeId_p            Node ID of the local node.
\param  nmtCommand_p        NMT command to request from MN.

\return The function returns a tOplkError error code.

\ingroup module_nmtcnu
*/
//------------------------------------------------------------------------------
tOplkError nmtcnu_sendNmtRequest(UINT nodeId_p, tNmtCommand nmtCommand_p)
{
    tOplkError      ret;
    tFrameInfo      nmtRequestFrameInfo;
    tPlkFrame       nmtRequestFrame;

    ret = kErrorOk;

    // build frame
    OPLK_MEMSET(&nmtRequestFrame.aDstMac[0], 0x00, sizeof(nmtRequestFrame.aDstMac)); // set by DLL
    OPLK_MEMSET(&nmtRequestFrame.aSrcMac[0], 0x00, sizeof(nmtRequestFrame.aSrcMac)); // set by DLL
    ami_setUint16Be(&nmtRequestFrame.etherType, C_DLL_ETHERTYPE_EPL);
    ami_setUint8Le(&nmtRequestFrame.dstNodeId, (BYTE)C_ADR_MN_DEF_NODE_ID); // node id of the MN
    ami_setUint8Le(&nmtRequestFrame.messageType, (BYTE)kMsgTypeAsnd);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.serviceId, (BYTE) kDllAsndNmtRequest);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.payload.nmtRequestService.nmtCommandId,
                   (BYTE)nmtCommand_p);
    ami_setUint8Le(&nmtRequestFrame.data.asnd.payload.nmtRequestService.targetNodeId,
                   (BYTE)nodeId_p); // target for the nmt command
    OPLK_MEMSET(&nmtRequestFrame.data.asnd.payload.nmtRequestService.aNmtCommandData[0], 0x00,
                sizeof(nmtRequestFrame.data.asnd.payload.nmtRequestService.aNmtCommandData));

    // build info-structure
    nmtRequestFrameInfo.pFrame = &nmtRequestFrame;
    nmtRequestFrameInfo.frameSize = C_DLL_MINSIZE_NMTREQ; // sizeof(nmtRequestFrame);

    // send NMT request
    ret = dllucal_sendAsyncFrame(&nmtRequestFrameInfo, kDllAsyncReqPrioNmt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register state change callback function

The function registers a callback function to get informed about an
NMT-Change-State-Event.

\param  pfnNmtCheckEventCb_p        Pointer to check event callback function.

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

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT commands

The function processes NMT commands.

\param  pFrameInfo_p        Pointer to frame containing the NMT command

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError commandCb(tFrameInfo* pFrameInfo_p)
{
    tOplkError      ret = kErrorOk;
    tNmtCommand     nmtCommand;
    BOOL            fNodeIdInList;
    tNmtEvent       nmtEvent = kNmtEventNoEvent;

    if(pFrameInfo_p == NULL)
        return kErrorNmtInvalidFramePointer;

    nmtCommand = getNmtCommand(pFrameInfo_p);
    switch (nmtCommand)
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
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&(pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]));
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStartNode;
            }
            break;

        case kNmtCmdStopNodeEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStopNode;
            }
            break;

        case kNmtCmdEnterPreOperational2Ex:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnterPreOperational2;
            }
            break;

        case kNmtCmdEnableReadyToOperateEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnableReadyToOperate;
            }
            break;

        case kNmtCmdResetNodeEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetNode;
            }
            break;

        case kNmtCmdResetCommunicationEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetCom;
            }
            break;

        case kNmtCmdResetConfigurationEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetConfig;
            }
            break;

        case kNmtCmdSwResetEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(&pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0]);
            if (fNodeIdInList != FALSE)
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
            return kErrorNmtUnknownCommand;
            break;
    } // end of switch (nmtCommand)

    if (nmtEvent != kNmtEventNoEvent)
    {
        if (nmtCnuInstance_g.pfnCheckEventCb != NULL)
        {
            ret = nmtCnuInstance_g.pfnCheckEventCb(nmtEvent);
            if (ret == kErrorReject)
            {
                return kErrorOk;
            }
            else if (ret != kErrorOk)
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

The function extracts the NMT command from the frame.

\param  pFrameInfo_p        Pointer to frame containing the NMT command

\return The function returns the extracted NMT command
*/
//------------------------------------------------------------------------------
static tNmtCommand getNmtCommand(tFrameInfo* pFrameInfo_p)
{
    tNmtCommand          nmtCommand;
    tNmtCommandService*  pNmtCommandService;

    pNmtCommandService = &pFrameInfo_p->pFrame->data.asnd.payload.nmtCommandService;
    nmtCommand = (tNmtCommand)ami_getUint8Le(&pNmtCommandService->nmtCommandId);

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
    bitOffset = (UINT8)nmtCnuInstance_g.nodeId % 8;

    nodeListByte = ami_getUint8Le(&pbNmtCommandDate_p[byteOffset]);
    if((nodeListByte & bitOffset) == 0)
        fNodeIdInList = FALSE;
    else
        fNodeIdInList = TRUE;

    return  fNodeIdInList;
}

///\}

