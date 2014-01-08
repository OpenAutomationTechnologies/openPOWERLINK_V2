/**
********************************************************************************
\file   generic.c

\brief  Generic API function

This file contains the implementation of the generic API functions.

\ingroup module_api
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <stddef.h>
#include <limits.h>

#include <Epl.h>

#include <user/pdoucal.h>
#include <user/dllucal.h>
#include <user/nmtcnu.h>
#include <user/nmtmnu.h>
#include <user/sdocom.h>
#include <user/identu.h>
#include <user/cfmu.h>
#include <user/ctrlu.h>

#include <target.h>

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
#include "obdcdc.h"
#endif

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

#if defined(CONFIG_INCLUDE_SDOC)
static tEplKernel cbSdoCon(tSdoComFinished* pSdoComFinished_p);
#endif
static tEplKernel cbReceivedAsnd(tFrameInfo *pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack. After the stack is initialized
the application must start it by performing a software reset. This is done by
sending the NMT event kNmtEventSwReset. The event can be sent by calling
oplk_execNmtCommand(kNmtEventSwReset).

\param  pInitParam_p            Pointer to the init parameters. The init
                                parameters must be set by the application.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          Stack was successfully initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_init(tEplApiInitParam* pInitParam_p)
{
    tEplKernel          ret;

    target_init();

    if ((ret = ctrlu_init()) != kEplSuccessful)
    {
        target_cleanup();
        return ret;
    }

    return ctrlu_initStack(pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the openPOWERLINK stack

The function shuts down the openPOWERLINK stack. Before shutting down the stack
it should be stopped by sending the NMT command kNmtEventSwitchOff. The command
can be sent by calling oplk_execNmtCommand(kNmtEventSwitchOff);

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          Stack was successfully shut down.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_shutdown(void)
{
    tEplKernel          ret;

    ret = ctrlu_shutdownStack();
    ctrlu_exit();
    target_cleanup();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Execute a NMT command

The function executes a NMT command, i.e. post the NMT event to the NMT module,
NMT commands which are not appropriate in the current NMT state are silently
ignored. Please keep in mind that the NMT state may change until the NMT command
is actually executed.

\param  nmtEvent_p              NMT command to send.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_execNmtCommand(tNmtEvent nmtEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    ret = nmtu_postNmtEvent(nmtEvent_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Link application variable into the OD

The function links an array of application variables onto the specified object
in the object dictionary (OD).

\param  objIndex_p          Object ID of object to link the variable to.
\param  pVar_p              Pointer to the application variable that should be
                            linked.
\param  pVarEntries_p       Pointer to the number of entries to link. The function
                            stores the number of actually used entries at this
                            location.
\param  pEntrySize_p        Pointer to the size of one entry. If the size is
                            zero, the actual size will be read from the object
                            dictionary. The function stores the entire size of
                            all linked entries at this location.
\param  firstSubindex_p     Specifies the first subindex to be linked.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          The variables are successfully linked to the
                                object dictionary.
\retval kEplObdIndexNotExist    The object index does not exist in the object
                                dictionary.
\retval kEplObdSubindexNotExist The subindex does not exist in the object
                                dictionary.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_linkObject(UINT objIndex_p, void* pVar_p, UINT* pVarEntries_p,
                           tObdSize* pEntrySize_p, UINT firstSubindex_p)
{
    UINT8           varEntries;
    UINT8           indexEntries;
    UINT8 MEM*      pData;
    UINT            subindex;
    tVarParam       varParam;
    tObdSize        entrySize;
    tObdSize        usedSize;

    tEplKernel      ret = kEplSuccessful;

    if ((pVar_p == NULL) || (pVarEntries_p == NULL) || (*pVarEntries_p == 0) || (pEntrySize_p == NULL))
        return kEplApiInvalidParam;

    pData      = (UINT8 MEM*) pVar_p;
    varEntries = (UINT8)*pVarEntries_p;
    usedSize   = 0;

    // init varParam structure with default values
    varParam.index = objIndex_p;
    varParam.validFlag = kVarValidAll;

    if (firstSubindex_p != 0)
    {   // check if object exists by reading subindex 0x00,
        // because user wants to link a variable to a subindex unequal 0x00
        // read number of entries
        entrySize = (tObdSize)sizeof(indexEntries);
        ret = obd_readEntry (objIndex_p, 0x00, (void GENERIC*) &indexEntries, &entrySize );
        if ((ret != kEplSuccessful) || (indexEntries == 0x00))
        {
            // Object doesn't exist or invalid entry number
            TRACE("%s() Object %04x not existing\n", __func__, objIndex_p);
            return kEplObdIndexNotExist;
        }
    }
    else
    {   // user wants to link a variable to subindex 0x00 -> that's OK
        indexEntries = 0;
    }

    // Correct number of entries if number read from OD is greater than the specified number.
    // This is done, so that we do not set more entries than subindexes the
    // object actually has.
    if ((indexEntries > (varEntries + firstSubindex_p - 1)) && (varEntries != 0x00))
    {
        indexEntries = (UINT8)(varEntries + firstSubindex_p - 1);
    }

    // map entries
    for (subindex = firstSubindex_p; subindex <= indexEntries; subindex++)
    {
        // if passed entry size is 0, then get size from OD
        if (*pEntrySize_p == 0x00)
        {
            if ((entrySize = obd_getDataSize(objIndex_p, subindex)) == 0x00)
            {
                // invalid entry size (maybe object doesn't exist or entry of type DOMAIN is empty)
                return kEplObdSubindexNotExist;
            }
        }
        else
        {   // use passed entry size
            entrySize = *pEntrySize_p;
        }

        varParam.subindex = subindex;
        varParam.size = entrySize;
        varParam.pData = pData;

        usedSize += entrySize;
        pData   += entrySize;

        if ((ret = obd_defineVar(&varParam)) != kEplSuccessful)
            break;
    }

    // set number of mapped entries and entry size
    *pVarEntries_p = ((indexEntries - firstSubindex_p) + 1);
    *pEntrySize_p = usedSize;
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read entry from object dictionary

The function reads the specified entry from the object dictionary of the specified
node. If this node is a remote node, it performs a SDO transfer, which means this
function returns kEplApiTaskDeferred and the application is informed via the
event callback function when the task is completed.

\param  pSdoComConHdl_p     A pointer to the SDO connection handle. It may be
                            NULL in case of local OD access.
\param  nodeId_p            Node ID of the node to read. If node ID is 0 the
                            local OD will be read.
\param  index_p             The index of the object to read.
\param  subindex_p          The subindex of the object to read.
\param  pDstData_le_p       Pointer where to store the read data. The data is in
                            little endian byte order.
\param  pSize_p             Pointer to the size of the buffer. For local reads
                            the function stores the size of the object at this
                            location.
\param  sdoType_p           The type of the SDO transfer (SDO over ASnd, SDO over
                            UDP or SDO over PDO)
\param  pUserArg_p          User defined argument which will be passed to the
                            event callback function.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_readObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                           UINT subindex_p, void* pDstData_le_p, UINT* pSize_p,
                           tSdoType sdoType_p, void* pUserArg_p)
{
    tEplKernel      ret = kEplSuccessful;
    tObdSize        obdSize;

    if ((index_p == 0) || (pDstData_le_p == NULL) || (pSize_p == NULL) || (*pSize_p == 0))
        return kEplApiInvalidParam;

    if (nodeId_p == 0 || nodeId_p == obd_getNodeId())
    {   // local OD access can be performed
        obdSize = (tObdSize) *pSize_p;
        ret = obd_readEntryToLe(index_p, subindex_p, pDstData_le_p, &obdSize);
        *pSize_p = (UINT) obdSize;
    }
    else
    {   // perform SDO transfer
#if defined(CONFIG_INCLUDE_SDOC)
        tSdoComTransParamByIndex transParamByIndex;

        // check if application provides space for handle
        if (pSdoComConHdl_p == NULL)
            return kEplApiInvalidParam;

#if defined(CONFIG_INCLUDE_CFM)
        if (cfmu_isSdoRunning(nodeId_p))
            return kEplApiSdoBusyIntern;
#endif

        // init command layer connection
        ret = sdocom_defineConnection(pSdoComConHdl_p, nodeId_p, sdoType_p);
        if ((ret != kEplSuccessful) && (ret != kEplSdoComHandleExists))
        {
            return ret;
        }

        transParamByIndex.pData = pDstData_le_p;
        transParamByIndex.sdoAccessType = kSdoAccessTypeRead;
        transParamByIndex.sdoComConHdl = *pSdoComConHdl_p;
        transParamByIndex.dataSize = *pSize_p;
        transParamByIndex.index = index_p;
        transParamByIndex.subindex = subindex_p;
        transParamByIndex.pfnSdoFinishedCb = cbSdoCon;
        transParamByIndex.pUserArg = pUserArg_p;

        if ((ret = sdocom_initTransferByIndex(&transParamByIndex)) != kEplSuccessful)
            return ret;

        ret = kEplApiTaskDeferred;

#else
        ret = kEplApiInvalidParam;
#endif
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write entry to object dictionary

The function writes the specified entry to the object dictionary of the specified
node. If this node is a remote node, it performs a SDO transfer, which means this
function returns kEplApiTaskDeferred and the application is informed via the
event callback function when the task is completed.

\param  pSdoComConHdl_p     A pointer to the SDO connection handle. It may be
                            NULL in case of local OD access.
\param  nodeId_p            Node ID of the node to write. If node ID is 0 the
                            local OD will be read.
\param  index_p             The index of the object to write.
\param  subindex_p          The subindex of the object to write.
\param  pSrcData_le_p       Pointer to data. The data must be in little endian
                            byte order.
\param  size_p              Size of the data to write.
\param  sdoType_p           The type of the SDO transfer (SDO over ASnd, SDO over
                            UDP or SDO over PDO)
\param  pUserArg_p          User defined argument which will be passed to the
                            event callback function.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                            UINT subindex_p, void* pSrcData_le_p, UINT size_p,
                            tSdoType sdoType_p, void* pUserArg_p)
{
    tEplKernel      ret = kEplSuccessful;

    if ((index_p == 0) || (pSrcData_le_p == NULL) || (size_p == 0))
        return kEplApiInvalidParam;

    if (nodeId_p == 0 || nodeId_p == obd_getNodeId())
    {   // local OD access can be performed
        ret = obd_writeEntryFromLe(index_p, subindex_p, pSrcData_le_p, size_p);
    }
    else
    {   // perform SDO transfer
#if defined(CONFIG_INCLUDE_SDOC)
        tSdoComTransParamByIndex transParamByIndex;

        // check if application provides space for handle
        if (pSdoComConHdl_p == NULL)
            return kEplApiInvalidParam;

#if defined(CONFIG_INCLUDE_CFM)
        if (cfmu_isSdoRunning(nodeId_p))
            return kEplApiSdoBusyIntern;
#endif
        // d.k.: How to recycle command layer connection?
        //       Try to redefine it, which will return kEplSdoComHandleExists
        //       and the existing command layer handle.
        //       If the returned handle is busy, sdocom_initTransferByIndex()
        //       will return with error.

        // init command layer connection
        ret = sdocom_defineConnection(pSdoComConHdl_p, nodeId_p, sdoType_p);
        if ((ret != kEplSuccessful) && (ret != kEplSdoComHandleExists))
            return ret;

        transParamByIndex.pData = pSrcData_le_p;
        transParamByIndex.sdoAccessType = kSdoAccessTypeWrite;
        transParamByIndex.sdoComConHdl = *pSdoComConHdl_p;
        transParamByIndex.dataSize = size_p;
        transParamByIndex.index = index_p;
        transParamByIndex.subindex = subindex_p;
        transParamByIndex.pfnSdoFinishedCb = cbSdoCon;
        transParamByIndex.pUserArg = pUserArg_p;

        if ((ret = sdocom_initTransferByIndex(&transParamByIndex)) != kEplSuccessful)
            return ret;

        ret = kEplApiTaskDeferred;

#else
        ret = kEplApiInvalidParam;
#endif
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free SDO channel

The function frees the specified SDO channel. It must be called when the SDO
channel to a remote node is not needed anymore. This may be done in the event
callback function when the last SDO transfer to a remote node has completed.

\param  sdoComConHdl_p      The SDO connection handle.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p)
{
    tEplKernel      ret = kEplSuccessful;

#if defined(CONFIG_INCLUDE_SDOC)

#if defined(CONFIG_INCLUDE_CFM)
    if (cfmu_isSdoRunning(sdocom_getNodeId(sdoComConHdl_p)))
    {
        ret = kEplApiSdoBusyIntern;
    }
    else
#endif
    {
        // delete command layer connection
        ret = sdocom_undefineConnection(sdoComConHdl_p);
    }
#else
    ret = kEplApiInvalidParam;
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort a SDO transfer

The function aborts the running SDO transfer on the specified SDO channel.

\param  sdoComConHdl_p      The SDO connection handle.
\param  abortCode_p         The abort code which shall be sent to the remote
                            node.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_abortSdo(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tEplKernel      ret = kEplSuccessful;

#if defined(CONFIG_INCLUDE_SDOC)

#if defined(CONFIG_INCLUDE_CFM)
    if (cfmu_isSdoRunning(sdocom_getNodeId(sdoComConHdl_p)))
    {
        ret = kEplApiSdoBusyIntern;
    }
    else
#endif
    {
        ret = sdocom_abortTransfer(sdoComConHdl_p, abortCode_p);
    }
#else
    ret = kEplApiInvalidParam;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read entry from local object dictionary

The function reads the specified entry from the local object dictionary.

\param  index_p             The index of the object to read.
\param  subindex_p          The subindex of the object to read.
\param  pDstData_p          Pointer where to store the read data. The data is in
                            platform byte order.
\param  pSize_p             Pointer to the size of the buffer. The function
                            stores the size of the object at this location.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_readLocalObject(UINT index_p, UINT subindex_p, void* pDstData_p,
                                UINT* pSize_p)
{
    tEplKernel      ret = kEplSuccessful;
    tObdSize        obdSize;

    obdSize = (tObdSize)*pSize_p;
    ret = obd_readEntry(index_p, subindex_p, pDstData_p, &obdSize);
    *pSize_p = (UINT)obdSize;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write entry to local object dictionary

The function writes the specified entry to the local object dictionary.

\param  index_p             The index of the object to write.
\param  subindex_p          The subindex of the object to write.
\param  pSrcData_p          Pointer to data. The data must be in platform byte
                            order.
\param  size_p              Size of the data to write.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_writeLocalObject(UINT index_p, UINT subindex_p, void* pSrcData_p,
                                        UINT size_p)
{
    return obd_writeEntry(index_p, subindex_p, pSrcData_p, (tObdSize)size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send a generic ASnd frame

The function sends a generic ASnd frame.

\param  dstNodeId_p         Destination Node ID
\param  pAsndFrame_p        Pointer to ASnd frame which should be sent.
\param  asndSize_p          Size of ASnd frame to send. The size contains the
                            service ID and the payload.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_sendAsndFrame(UINT8 dstNodeId_p, tEplAsndFrame *pAsndFrame_p,
                              size_t asndSize_p)
{
    tEplKernel      ret;
    tFrameInfo      frameInfo;
    BYTE            buffer[EPL_C_DLL_MAX_ASYNC_MTU];

    // Calculate size of frame (Asnd data + header)
    frameInfo.frameSize = asndSize_p + offsetof(tEplFrame, m_Data);

    // Check for correct input
    if ((pAsndFrame_p == NULL) || (frameInfo.frameSize >= sizeof(buffer)))
        return  kEplReject;

    // Calculate size of frame (Asnd data + header)
    frameInfo.frameSize = asndSize_p + offsetof(tEplFrame, m_Data);
    frameInfo.pFrame = (tEplFrame *)buffer;

    // Copy Asnd data
    EPL_MEMSET(frameInfo.pFrame, 0x00, frameInfo.frameSize);
    EPL_MEMCPY(&frameInfo.pFrame->m_Data.m_Asnd, pAsndFrame_p, asndSize_p);

    // Fill in additional data (SrcNodeId is filled by DLL if it is set to 0)
    AmiSetByteToLe(&frameInfo.pFrame->m_le_bMessageType, (UINT8) kEplMsgTypeAsnd);
    AmiSetByteToLe(&frameInfo.pFrame->m_le_bDstNodeId, (UINT8) dstNodeId_p );
    AmiSetByteToLe(&frameInfo.pFrame->m_le_bSrcNodeId, (UINT8) 0);

    // Request frame transmission
    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set forwarding of received ASnd frames

The function enables or disables the forwarding of received ASnd frames
 to the application.

\param  serviceId_p         The ASnd service ID for which the forwarding will
                            be set.
\param  filterType_p        Specifies which types of ASnd frames should be
                            received. Could be none, unicast or all frames.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_setAsndForward(UINT8 serviceId_p, tEplApiAsndFilter filterType_p)
{
    tEplKernel          ret;
    tDllAsndFilter      dllFilter;

    // Map API filter types to stack internal filter types
    switch(filterType_p)
    {
        case kEplApiAsndFilterLocal:
            dllFilter = kDllAsndFilterLocal;
            break;

        case kEplApiAsndFilterAny:
            dllFilter = kDllAsndFilterAny;
            break;

        default:
        case kEplApiAsndFilterNone:
            dllFilter = kDllAsndFilterNone;
            break;
    }

    ret = dllucal_regAsndService(serviceId_p, cbReceivedAsnd, dllFilter);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Post user defined event

The function posts user-defined events to event processing thread, i.e. calls
user event callback function with event kEplApiEventUserDef. This function is
thread safe and is meant for synchronization.

\param  pUserArg_p          User defined pointer.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_postUserEvent(void* pUserArg_p)
{
    tEplKernel  ret;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkApi;
    event.m_NetTime.m_dwNanoSec = 0;
    event.m_NetTime.m_dwSec = 0;
    event.m_EventType = kEplEventTypeApiUserDef;
    event.m_pArg = &pUserArg_p;
    event.m_uiSize = sizeof(pUserArg_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Trigger NMT state change

The function triggers a NMT state change by sending the specified node command
for the specified node.

\param  nodeId_p            The Node ID for which the node command will be executed.
\param  nodeCommand_p       The Node command to execute.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_triggerMnStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
#if defined(CONFIG_INCLUDE_NMT_MN)
    return nmtmnu_triggerStateChange(nodeId_p, nodeCommand_p);
#else
    return kEplApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set CDC buffer

The function sets the concise device description (CDC) buffer to be used by
the stack to read the configuration.

\param  pCdc_p          Pointer to the concise device description.
\param  cdcSize_p       Size of the concise device description

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_setCdcBuffer(BYTE* pCdc_p, UINT cdcSize_p)
{
#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    obdcdc_setBuffer(pCdc_p, cdcSize_p);
    return kEplSuccessful;
#else
    return kEplApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set CDC filename

The function sets the concise device description (CDC) file to be used by
the stack to read the configuration.

\param  pCdcFilename_p  Filename to be used for reading the concise device
                        description.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_setCdcFilename(char* pCdcFilename_p)
{
#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    obdcdc_setFilename(pCdcFilename_p);
    return kEplSuccessful;
#else
    return kEplApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Stack process function

The process function is used in single threaded environment e.g. without any OS.
It gives processing time to several tasks in the openPOWERLINK stack.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_process(void)
{
    return ctrlu_processStack();
}

//------------------------------------------------------------------------------
/**
\brief Check if kernel stack is alive

The function checks if the kernel part of the stack is alive.

\return Returns the status of the kernel stack.
\retval TRUE        If the kernel stack is alive.
\retval FALSE       IF the kernel stack is dead.

\ingroup module_api
*/
//------------------------------------------------------------------------------
BOOL oplk_checkKernelStack(void)
{
    return ctrlu_checkKernelStack();
}

//------------------------------------------------------------------------------
/**
\brief Wait for sync event

The function waits for a sync event.

\param  timeout_p       Time to wait for event

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_waitSyncEvent(ULONG timeout_p)
{
    return pdoucal_waitSyncEvent(timeout_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get IdentResponse of node

The function returns the stored IdentResponse frame of the specified node.

\param  nodeId_p            Node ID of which to get the Ident Response frame.
\param  ppIdentResponse_p   Pointer to store the address of the IdentResponse
                            frame.

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel oplk_getIdentResponse(UINT nodeId_p, tEplIdentResponse** ppIdentResponse_p)
{
#if defined(CONFIG_INCLUDE_NMT_MN)
    return identu_getIdentResponse(nodeId_p, ppIdentResponse_p);
#else
    return kEplApiInvalidParam;
#endif
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for SDO transfers

The function implements the callback function for SDO transfers. It will be
registered for a SDO transfer. When it is called by the SDO stack it sends a
SDO event to the application.

\param  pSdoComFinished_p   SDO parameter.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SDOC)
static tEplKernel cbSdoCon(tSdoComFinished* pSdoComFinished_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplApiEventArg     eventArg;

    eventArg.m_Sdo = *pSdoComFinished_p;
    ret = ctrlu_callUserEventCallback(kEplApiEventSdo, &eventArg);
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Callback function for received ASnds

The function implements the callback function to handle received ASnd frames.
Frames will be forwarded to the application by sending a user event.

\param  pFrameInfo_p   Pointer to information about the received frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbReceivedAsnd(tFrameInfo *pFrameInfo_p)
{
    tEplKernel              ret = kEplSuccessful;
    UINT                    asndOffset;
    tEplApiEventArg         apiEventArg;
    tEplApiEventType        eventType;

    // Check for correct input
    asndOffset = offsetof(tEplFrame, m_Data.m_Asnd);

    if ((pFrameInfo_p->frameSize <= asndOffset + 1) ||
        (pFrameInfo_p->frameSize > EPL_C_DLL_MAX_ASYNC_MTU))
        return kEplReject;

    // Forward received ASnd frame
    apiEventArg.m_RcvAsnd.m_pFrame = pFrameInfo_p->pFrame;
    apiEventArg.m_RcvAsnd.m_FrameSize = pFrameInfo_p->frameSize;

    eventType = kEplApiEventReceivedAsnd;
    ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
    return ret;
}

/// \}

