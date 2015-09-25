/**
********************************************************************************
\file   generic.c

\brief  Generic API functions

This file contains the implementation of the generic API functions.

\ingroup module_api
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <common/oplkinc.h>
#include <common/target.h>
#include <common/ami.h>
#include <user/ctrlu.h>
#include <user/nmtu.h>
#include <user/dllucal.h>
#include <user/eventu.h>
#include <user/pdoucal.h>
#include <oplk/obd.h>
#include <oplk/sdo.h>

#if defined(CONFIG_INCLUDE_CFM)
#include <user/cfmu.h>
#include <oplk/obdcdc.h>
#endif

#if defined(CONFIG_INCLUDE_SDOC)
#include <user/sdocom.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <user/nmtmnu.h>
#include <user/identu.h>
#endif

#include <common/target.h>
#include <common/memmap.h>

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
#include <oplk/obdconf.h>
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
static BOOL fStackInitialized_l = FALSE;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

#if defined(CONFIG_INCLUDE_SDOC)
static tOplkError cbSdoCon(tSdoComFinished* pSdoComFinished_p);
#endif
static tOplkError cbReceivedAsnd(tFrameInfo *pFrameInfo_p);
#if defined(CONFIG_INCLUDE_VETH)
static tOplkError cbReceivedEth(tFrameInfo* pFrameInfo_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize openPOWERLINK environment

The function initializes the necessary environment for openPOWERLINK. After this
function is called successfully the openPOWERLINK stack can be created by
calling \ref oplk_create.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                Initialization was successful.
\retval Other                   Error occurred during initialization.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_initialize(void)
{
    tOplkError          ret;

    target_init();

    if ((ret = ctrlu_init()) != kErrorOk)
    {
        target_cleanup();
        return ret;
    }

    if (memmap_init() != kMemMapOk)
    {
        target_cleanup();
        return kErrorNoResource;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create openPOWERLINK stack

The function creates the openPOWERLINK stack with the given initialization
parameters. Before creating the stack it is required to call
\ref oplk_initialize!
After the stack is initialized the application must start it by performing a
software reset. This is done by sending the NMT event \ref kNmtEventSwReset.
The event can be sent by calling \b oplk_execNmtCommand(kNmtEventSwReset).

\param  pInitParam_p            Pointer to the initialization parameters which
                                must be set by the application.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                Stack initialization was successful.
\retval Other                   Error occurred during stack initialization.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_create(tOplkApiInitParam* pInitParam_p)
{
    tOplkError  ret;

    ret = ctrlu_checkKernelStackInfo();
    if (ret != kErrorOk)
        return ret;

    ret = ctrlu_initStack(pInitParam_p);
    if (ret == kErrorOk)
        fStackInitialized_l = TRUE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Destroy openPOWERLINK stack

The function shuts down the openPOWERLINK stack without cleaning up the stack
environment. Before calling this function it is recommended to stop the stack
by sending the NMT command kNmtEventSwitchOff. The command can be sent by calling
\b oplk_execNmtCommand(kNmtEventSwitchOff).

\note   After cleaning up the openPOWERLINK stack with calling this function it
        is possible to re-create the openPOWERLINK stack with \ref oplk_create.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Stack was successfully shut down.
\retval Other             Error occurred while shutting down the openPOWERLINK stack.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_destroy(void)
{
    tOplkError  ret;

    fStackInitialized_l = FALSE;

    ret = ctrlu_shutdownStack();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down openPOWERLINK environment

The function shuts down the environment used by openPOWERLINK.

\note   If the openPOWERLINK stack wasn't cleaned up before with calling
        \ref oplk_destroy, this function also cleans up the stack.

\ingroup module_api
*/
//------------------------------------------------------------------------------
void oplk_exit(void)
{
    if (fStackInitialized_l)
    {
        fStackInitialized_l = FALSE;
        ctrlu_shutdownStack();
    }

    ctrlu_exit();
    memmap_shutdown();
    target_cleanup();
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack. After the stack is initialized
the application must start it by performing a software reset. This is done by
sending the NMT event \ref kNmtEventSwReset. The event can be sent by calling
\b oplk_execNmtCommand(kNmtEventSwReset).

\deprecated The initialization function is replaced by \ref oplk_initialize and
            \ref oplk_create. It is recommended using the new functions for
            stack initialization!

\param  pInitParam_p            Pointer to the init parameters. The init
                                parameters must be set by the application.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                Stack was successfully initialized.
\retval Other                   Error occurred while initializing the openPOWERLINK stack.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_init(tOplkApiInitParam* pInitParam_p)
{
    tOplkError          ret;

    ret = oplk_initialize();
    if (ret != kErrorOk)
        return ret;

    ret = oplk_create(pInitParam_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the openPOWERLINK stack

The function shuts down the openPOWERLINK stack. Before shutting down the stack
it should be stopped by sending the NMT command kNmtEventSwitchOff. The command
can be sent by calling oplk_execNmtCommand(kNmtEventSwitchOff);

\deprecated The shutdown function is replaced by \ref oplk_destroy and
            \ref oplk_exit. It is recommended using the new functions for
            stack shutdown!

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Stack was successfully shut down.
\retval Other             Error occurred while shutting down the openPOWERLINK stack.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_shutdown(void)
{
    tOplkError          ret = kErrorApiNotInitialized;

    ret = oplk_destroy();
    oplk_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Execute an NMT command

The function executes an NMT command, i.e. post the NMT event to the NMT module.
NMT commands which are not appropriate in the current NMT state are silently
ignored. Please keep in mind that the NMT state may change until the NMT command
is actually executed.

\param  nmtEvent_p              NMT command to send.

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_execNmtCommand(tNmtEvent nmtEvent_p)
{
    tOplkError      ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    ret = nmtu_postNmtEvent(nmtEvent_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Link application variable into the OD

The function links an array of application variables onto the specified object
in the object dictionary (OD).

\param  objIndex_p          Index of the object to link the variable to.
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                  The variables are successfully linked to the
                                  object dictionary.
\retval kErrorObdIndexNotExist    The object index does not exist in the object
                                  dictionary.
\retval kErrorObdSubindexNotExist The subindex does not exist in the object
                                  dictionary.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_linkObject(UINT objIndex_p, void* pVar_p, UINT* pVarEntries_p,
                           tObdSize* pEntrySize_p, UINT firstSubindex_p)
{
    UINT8           varEntries;
    UINT8           indexEntries;
    UINT8 MEM*      pData;
    UINT            subindex;
    tVarParam       varParam;
    tObdSize        entrySize;
    tObdSize        usedSize;

    tOplkError      ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if ((pVar_p == NULL) || (pVarEntries_p == NULL) || (*pVarEntries_p == 0) || (pEntrySize_p == NULL))
        return kErrorApiInvalidParam;

    pData      = (UINT8 MEM*)pVar_p;
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
        ret = obd_readEntry(objIndex_p, 0x00, (void*)&indexEntries, &entrySize);
        if ((ret != kErrorOk) || (indexEntries == 0x00))
        {
            // Object doesn't exist or invalid entry number
            DEBUG_LVL_ERROR_TRACE("%s() Object 0x%04x not existing\n", __func__, objIndex_p);
            return kErrorObdIndexNotExist;
        }
    }
    else
    {   // user wants to link a variable to subindex 0x00 -> that's OK
        indexEntries = 0;
    }

    // Correct number of entries if number read from OD is greater than the specified number.
    // This is done in order to avoid setting more entries than there are subindexes in
    // the object.
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
                return kErrorObdSubindexNotExist;
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
        pData += entrySize;

        if ((ret = obd_defineVar(&varParam)) != kErrorOk)
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
node. If this node is a remote node, it performs an SDO transfer. In such case this
function returns kErrorApiTaskDeferred and the application is informed via the
event callback function when the task is completed.

\param  pSdoComConHdl_p     A pointer to the SDO connection handle. It may be
                            NULL in case of local OD access.
\param  nodeId_p            Node ID of the node to read. If node ID is 0, the
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Entry was successfully read from OD.
\retval Other             Error occurred while reading the OD.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_readObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                           UINT subindex_p, void* pDstData_le_p, UINT* pSize_p,
                           tSdoType sdoType_p, void* pUserArg_p)
{
    tOplkError      ret = kErrorOk;
    tObdSize        obdSize;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if ((index_p == 0) || (pDstData_le_p == NULL) || (pSize_p == NULL) || (*pSize_p == 0))
        return kErrorApiInvalidParam;

    if (nodeId_p == 0 || nodeId_p == obd_getNodeId())
    {   // local OD access can be performed
        obdSize = (tObdSize)*pSize_p;
        ret = obd_readEntryToLe(index_p, subindex_p, pDstData_le_p, &obdSize);
        *pSize_p = (UINT)obdSize;
    }
    else
    {   // perform SDO transfer
#if defined(CONFIG_INCLUDE_SDOC)
        tSdoComTransParamByIndex transParamByIndex;

        // check if application provides space for handle
        if (pSdoComConHdl_p == NULL)
            return kErrorApiInvalidParam;

#if defined(CONFIG_INCLUDE_CFM)
        if (cfmu_isSdoRunning(nodeId_p))
            return kErrorApiSdoBusyIntern;
#endif

        // init command layer connection
        ret = sdocom_defineConnection(pSdoComConHdl_p, nodeId_p, sdoType_p);
        if ((ret != kErrorOk) && (ret != kErrorSdoComHandleExists))
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

        if ((ret = sdocom_initTransferByIndex(&transParamByIndex)) != kErrorOk)
            return ret;

        ret = kErrorApiTaskDeferred;

#else
        ret = kErrorApiInvalidParam;
#endif
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write entry to object dictionary

The function writes the specified entry to the object dictionary of the specified
node. If this node is a remote node, it performs an SDO transfer. In such case this
function returns kErrorApiTaskDeferred and the application is informed via the
event callback function when the task is completed.

\param  pSdoComConHdl_p     A pointer to the SDO connection handle. It may be
                            NULL in case of local OD access.
\param  nodeId_p            Node ID of the node to write. If node ID is 0, the
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Entry was successfully written to the OD.
\retval Other             Error occurred while writing to the OD.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                            UINT subindex_p, void* pSrcData_le_p, UINT size_p,
                            tSdoType sdoType_p, void* pUserArg_p)
{
    tOplkError      ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if ((index_p == 0) || (pSrcData_le_p == NULL) || (size_p == 0))
        return kErrorApiInvalidParam;

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
            return kErrorApiInvalidParam;

#if defined(CONFIG_INCLUDE_CFM)
        if (cfmu_isSdoRunning(nodeId_p))
            return kErrorApiSdoBusyIntern;
#endif
        // d.k.: How to recycle command layer connection?
        //       Try to redefine it, which will return kErrorSdoComHandleExists
        //       and the existing command layer handle.
        //       If the returned handle is busy, sdocom_initTransferByIndex()
        //       will return with error.

        // init command layer connection
        ret = sdocom_defineConnection(pSdoComConHdl_p, nodeId_p, sdoType_p);
        if ((ret != kErrorOk) && (ret != kErrorSdoComHandleExists))
            return ret;

        transParamByIndex.pData = pSrcData_le_p;
        transParamByIndex.sdoAccessType = kSdoAccessTypeWrite;
        transParamByIndex.sdoComConHdl = *pSdoComConHdl_p;
        transParamByIndex.dataSize = size_p;
        transParamByIndex.index = index_p;
        transParamByIndex.subindex = subindex_p;
        transParamByIndex.pfnSdoFinishedCb = cbSdoCon;
        transParamByIndex.pUserArg = pUserArg_p;

        if ((ret = sdocom_initTransferByIndex(&transParamByIndex)) != kErrorOk)
            return ret;

        ret = kErrorApiTaskDeferred;

#else
        ret = kErrorApiInvalidParam;
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          SDO channel was successfully freed.
\retval Other             Error occurred while freeing the SDO channel.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p)
{
    tOplkError      ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_SDOC)

#if defined(CONFIG_INCLUDE_CFM)
    if (cfmu_isSdoRunning(sdocom_getNodeId(sdoComConHdl_p)))
    {
        ret = kErrorApiSdoBusyIntern;
    }
    else
#endif
    {
        // delete command layer connection
        ret = sdocom_undefineConnection(sdoComConHdl_p);
    }
#else
    ret = kErrorApiInvalidParam;
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort an SDO transfer

The function aborts the running SDO transfer on the specified SDO channel.

\param  sdoComConHdl_p      The SDO connection handle.
\param  abortCode_p         The abort code which shall be sent to the remote
                            node.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          SDO transfer was successfully freed.
\retval Other             Error occurred while aborting the SDO transfer.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_abortSdo(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_SDOC)

#if defined(CONFIG_INCLUDE_CFM)
    if (cfmu_isSdoRunning(sdocom_getNodeId(sdoComConHdl_p)))
    {
        ret = kErrorApiSdoBusyIntern;
    }
    else
#endif
    {
        ret = sdocom_abortTransfer(sdoComConHdl_p, abortCode_p);
    }
#else
    ret = kErrorApiInvalidParam;
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Entry was successfully read from local OD.
\retval Other             Error occurred while reading the OD.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_readLocalObject(UINT index_p, UINT subindex_p, void* pDstData_p,
                                UINT* pSize_p)
{
    tOplkError      ret = kErrorOk;
    tObdSize        obdSize;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Entry was successfully written to local OD.
\retval Other             Error occurred while writing to the OD.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_writeLocalObject(UINT index_p, UINT subindex_p, void* pSrcData_p,
                                 UINT size_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    return obd_writeEntry(index_p, subindex_p, pSrcData_p, (tObdSize)size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send a generic ASnd frame

The function sends a generic ASnd frame to the specified node. The function
queues the frame into the generic ASnd queue and immediately returns. The
sending of the frame is then controlled by the asynchronous scheduler.

\param  dstNodeId_p         Destination Node ID to send the ASnd frame to.
\param  pAsndFrame_p        Pointer to ASnd frame which should be sent.
\param  asndSize_p          Size of ASnd frame to send. The size contains the
                            service ID and the payload. The size cannot
                            exceed the maximum asynchronous size configured
                            in AsyncMTU.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          The ASnd frame was successfully queued into the
                          generic ASnd buffer.
\retval Other             Error occurred while adding the ASnd frame into
                          the generic ASnd buffer.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_sendAsndFrame(UINT8 dstNodeId_p, tAsndFrame* pAsndFrame_p,
                              size_t asndSize_p)
{
    tOplkError      ret;
    tFrameInfo      frameInfo;
    BYTE            buffer[C_DLL_MAX_ASYNC_MTU];
    UINT            frameSize;
    UINT16          asyncMtu;
    tObdSize        obdSize;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    // Calculate size of frame (Asnd data + header)
    frameSize = asndSize_p + offsetof(tPlkFrame, data);

    // Check for correct input
    if ((pAsndFrame_p == NULL) || (frameSize >= sizeof(buffer)))
        return kErrorReject;

    // Check size against configured AsyncMTU value
    obdSize = sizeof(UINT16);
    ret = obd_readEntry(0x1F98, 8, &asyncMtu, &obdSize);
    if (ret != kErrorOk)
        return kErrorReject;

    if (asndSize_p > asyncMtu)
        return kErrorReject;

    // Set up frame info
    frameInfo.frameSize = frameSize;
    frameInfo.frame.pBuffer = (tPlkFrame*)buffer;

    // Copy Asnd data
    OPLK_MEMSET(frameInfo.frame.pBuffer, 0x00, frameInfo.frameSize);
    OPLK_MEMCPY(&frameInfo.frame.pBuffer->data.asnd, pAsndFrame_p, asndSize_p);

    // Fill in additional data (SrcNodeId is filled by DLL if it is set to 0)
    ami_setUint8Le(&frameInfo.frame.pBuffer->messageType, (UINT8)kMsgTypeAsnd);
    ami_setUint8Le(&frameInfo.frame.pBuffer->dstNodeId, (UINT8)dstNodeId_p);
    ami_setUint8Le(&frameInfo.frame.pBuffer->srcNodeId, (UINT8)0);

    // Request frame transmission
    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an Ethernet frame

The function sends an Ethernet frame with generic priority. The given frame's
EtherType must be set to a valid pattern unequal 0x0000 and 0x88AB. The lower
layer inserts the node's MAC address if the source MAC address is set to 0.

\param  pFrame_p        Pointer to frame which should be sent.
\param  frameSize_p     Size of frame which should be sent.
                        The size shall include Ethernet header and payload
                        (e.g. min. Ethernet frame 14 byte + 46 byte = 60 byte).

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                Ethernet frame was successfully sent.
\retval kErrorInvalidOperation  EtherType set in frame is invalid.
\retval Other                   Error occurred while sending the Ethernet frame.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_sendEthFrame(tPlkFrame* pFrame_p, UINT frameSize_p)
{
    tOplkError  ret = kErrorOk;
    tFrameInfo  frameInfo;
    UINT16      etherType;

    // Check for correct input
    if ((pFrame_p == NULL) || (frameSize_p > C_DLL_MAX_ETH_FRAME))
        return kErrorReject;

    etherType = ami_getUint16Be(&pFrame_p->etherType);
    if ((etherType == 0) || (etherType == C_DLL_ETHERTYPE_EPL))
        return kErrorInvalidOperation;

    // Set frame info
    frameInfo.frameSize = frameSize_p;
    frameInfo.frame.pBuffer = pFrame_p;

    // Forward frame to DLLuCAL
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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Forwarding was successfully set.
\retval Other             Error occurred while setting ASnd forwarding.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setAsndForward(UINT8 serviceId_p, tOplkApiAsndFilter filterType_p)
{
    tOplkError          ret;
    tDllAsndFilter      dllFilter;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    // Map API filter types to stack internal filter types
    switch (filterType_p)
    {
        case tOplkApiAsndFilterLocal:
            dllFilter = kDllAsndFilterLocal;
            break;

        case tOplkApiAsndFilterAny:
            dllFilter = kDllAsndFilterAny;
            break;

        default:
        case tOplkApiAsndFilterNone:
            dllFilter = kDllAsndFilterNone;
            break;
    }

    ret = dllucal_regAsndService((tDllAsndServiceId)serviceId_p,
                                 cbReceivedAsnd, dllFilter);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set forwarding of received non-POWERLINK Ethernet frames

The function enables or disables the forwarding of received non-POWERLINK
Ethernet frames to the application.

\param  fEnable_p           Enable received Ethernet frame forwarding with TRUE.
                            Disable received Ethernet frame forwarding with FALSE.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                Forwarding was successfully set.
\retval kErrorIllegalInstance   Virtual Ethernet is not enabled.
\retval Other                   Error occurred while setting Ethernet forwarding.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setNonPlkForward(BOOL fEnable_p)
{
    tOplkError ret;

#if defined(CONFIG_INCLUDE_VETH)
    if (fEnable_p)
        ret = dllucal_regNonPlkHandler(cbReceivedEth);
    else
        ret = dllucal_regNonPlkHandler(NULL);
#else
    UNUSED_PARAMETER(fEnable_p);
    ret = kErrorIllegalInstance;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Post user defined event

The function posts user-defined events to event processing thread, i.e. calls
user event callback function with event \ref kOplkApiEventUserDef. This function
is thread safe and is meant for synchronization.

\param  pUserArg_p          User defined pointer.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          Event was successfully posted.
\retval Other             Error while posting the event.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_postUserEvent(void* pUserArg_p)
{
    tOplkError  ret;
    tEvent      event;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    event.eventSink = kEventSinkApi;
    event.netTime.nsec = 0;
    event.netTime.sec = 0;
    event.eventType = kEventTypeApiUserDef;
    event.eventArg.pEventArg = &pUserArg_p;
    event.eventArgSize = sizeof(pUserArg_p);

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

\note   The function is only used on an MN. On a CN it always returns
        \ref kErrorApiInvalidParam.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk          NMT node command was successfully sent.
\retval Other             Error occurred while sending NMT node command.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_triggerMnStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_NMT_MN)
    return nmtmnu_triggerStateChange(nodeId_p, nodeCommand_p);
#else
    UNUSED_PARAMETER(nodeId_p);
    UNUSED_PARAMETER(nodeCommand_p);

    return kErrorApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set CDC buffer

The function sets the concise device description (CDC) buffer to be used by
the stack to read the configuration. It can be used instead of
oplk_setCdcFilename() when no file system is available (e.g. on an
embedded system).

\param  pCdc_p          Pointer to the concise device description.
\param  cdcSize_p       Size of the concise device description

\note   The function is only used if the CDC functionality is included in the
        openPOWERLINK stack.

\see oplk_setCdcFilename()

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The buffer has successfully been set.
\retval kErrorApiInvalidParam       The function is not available due to missing
                                    CDC module.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setCdcBuffer(BYTE* pCdc_p, UINT cdcSize_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_CFM)
    obdcdc_setBuffer(pCdc_p, cdcSize_p);
    return kErrorOk;
#else
    UNUSED_PARAMETER(pCdc_p);
    UNUSED_PARAMETER(cdcSize_p);

    return kErrorApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set CDC filename

The function sets the concise device description (CDC) file to be used by
the stack to read the configuration.

\param  pCdcFilename_p  Filename to be used for reading the concise device
                        description.

\note   The function is only used if the CDC functionality is included in the
        openPOWERLINK stack.

\see oplk_setCdcBuffer()

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The filename has successfully been set.
\retval kErrorApiInvalidParam       The function is not available due to missing
                                    CDC module.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setCdcFilename(char* pCdcFilename_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_CFM)
    obdcdc_setFilename(pCdcFilename_p);
    return kErrorOk;
#else
    UNUSED_PARAMETER(pCdcFilename_p);

    return kErrorApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set OD archive path

The function sets the object dictionary (OD) configuration archive file path
to be used by the stack to store/restore OD configuration.

\param  pBackupPath_p   Path to be used for storing/restoring the OD archive.

\note   The function is only used if the configuration store restore
        functionality is included in the openPOWERLINK stack.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    The path has successfully been set.
\retval kErrorApiInvalidParam       The function is not available due to missing
                                    configuration store restore module.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setOdArchivePath(const char* pBackupPath_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
    return obdconf_setBackupArchivePath(pBackupPath_p);
#else
    UNUSED_PARAMETER(pBackupPath_p);

    return kErrorApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Stack process function

The process function is used in single threaded environments e.g. without any OS.
It gives processing time to several tasks in the openPOWERLINK stack.

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_process(void)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    return ctrlu_processStack();
}

//------------------------------------------------------------------------------
/**
\brief Check if kernel stack is alive

The function checks if the kernel part of the stack is alive.

\return Returns the status of the kernel stack.
\retval TRUE        The kernel stack is alive.
\retval FALSE       The kernel stack is dead.

\ingroup module_api
*/
//------------------------------------------------------------------------------
BOOL oplk_checkKernelStack(void)
{
    if (!ctrlu_stackIsInitialized())
        return FALSE;

    return ctrlu_checkKernelStack();
}

//------------------------------------------------------------------------------
/**
\brief Get openPOWERLINK Version

The function identifies the openPOWERLINK version of the stack.
The version is represented by a 32 bit number, which contains the major-, minor-,
build- and release-candidate-number.
Additionally the macros \ref PLK_STACK_VER, \ref PLK_STACK_REF,
\ref PLK_STACK_REL and \ref PLK_STACK_RC can be used to get the respective value
of the major-, minor-, build- or release-candidate-number.

\return Returns the openPOWERLINK version
\retval Returns a 32 bit number, which contains the major-, minor-, build- and
        release-candidate-number.

*/
//------------------------------------------------------------------------------
UINT32 oplk_getVersion(void)
{
    return PLK_DEFINED_STACK_VERSION;
}

//------------------------------------------------------------------------------
/**
\brief Get openPOWERLINK Version string

The function identifies the openPOWERLINK version of the stack.
The version is represented by a string, which contains the major-, minor-,
build- and release-candidate-number.

\return Returns the openPOWERLINK version string

*/
//------------------------------------------------------------------------------
char* oplk_getVersionString(void)
{
    static char* pVersionString = PLK_DEFINED_STRING_VERSION;

    return pVersionString;
}

//------------------------------------------------------------------------------
/**
\brief Get openPOWERLINK stack configuration

The function returns the openPOWERLINK kernel feature flags (configuration options) of the stack.
The stack can be compiled conditionally with different features. The user library requires these
features from the kernel driver. The kernel feature flags are located in the header file \ref oplk/oplkdefs.h.

\return Returns the configured kernel features
\retval Returns a UINT32 variable with the kernel feature flags.

*/
//------------------------------------------------------------------------------
UINT32 oplk_getStackConfiguration(void)
{
    return ctrlu_getFeatureFlags();
}

//------------------------------------------------------------------------------
/**
\brief  Get stack information

The function obtains the stack information.

\param  pStackInfo_p    Pointer to memory where the stack info should be stored.

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_getStackInfo(tOplkApiStackInfo* pStackInfo_p)
{
    tOplkError      ret;
    tCtrlKernelInfo kernelInfo;

    if (pStackInfo_p == NULL)
        return kErrorApiInvalidParam;

    ret = ctrlu_getKernelInfo(&kernelInfo);
    if (ret != kErrorOk)
        return ret;

    pStackInfo_p->userVersion = PLK_DEFINED_STACK_VERSION;
    pStackInfo_p->userFeature = ctrlu_getFeatureFlags();
    pStackInfo_p->kernelVersion = kernelInfo.version;
    pStackInfo_p->kernelFeature = kernelInfo.featureFlags;

    return ret;
}

//------------------------------------------------------------------------------

/**
\brief Wait for sync event

The function waits for a sync event. It blocks until the sync event occurred or
the specified timeout elapsed.

\note In a single process solution where the whole stack is linked to the
      application, the function immediately returns! In this case, the
      application must register its sync function as callback function so that
      it is directly called from the stack (see pfnCbSync in
      \ref tOplkApiInitParam).

\param  timeout_p       Specifies a timeout in microseconds. If 0 it waits
                        forever.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk            The sync event occurred.
\retval kErrorGeneralError  An error or timeout occurred while waiting for the
                            sync event.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_waitSyncEvent(ULONG timeout_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    return pdoucal_waitSyncEvent(timeout_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get IdentResponse of node

The function returns the stored IdentResponse frame of the specified node.

\param  nodeId_p            Node ID of which to get the Ident Response frame.
\param  ppIdentResponse_p   Pointer to store the address of the IdentResponse
                            frame.

\note   The function is only used on an MN. On a CN it returns always
        \ref kErrorApiInvalidParam.

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_getIdentResponse(UINT nodeId_p, tIdentResponse** ppIdentResponse_p)
{
    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

#if defined(CONFIG_INCLUDE_NMT_MN)
    return identu_getIdentResponse(nodeId_p, ppIdentResponse_p);
#else

    UNUSED_PARAMETER(nodeId_p);
    UNUSED_PARAMETER(ppIdentResponse_p);

    return kErrorApiInvalidParam;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Get Ethernet Interface MAC address

The function provides the Ethernet Interface MAC address used by the
Ethernet controller.

\param  pMacAddr_p      Pointer to memory buffer which is used to copy the MAC
                        address into. The memory buffer must have a size of
                        6 bytes!

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_getEthMacAddr(UINT8* pMacAddr_p)
{
    tOplkError ret = kErrorOk;

    if (pMacAddr_p != NULL)
        OPLK_MEMCPY(pMacAddr_p, ctrlu_getEthMacAddr(), 6);
    else
        ret = kErrorInvalidOperation;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Trigger PRes forward

The function triggers the forwarding of a PRes frame from Node \p nodeId_p
to the application. It can be used by the application for diagnosis purpose
(e.g. conformance test). After request "one" PRes frame form the specified
node will be forwarded to the application. The PRes frame is forwarded by
a \ref kOplkApiEventReceivedPres event. The application has to handle this event
to get the frame.

\param  nodeId_p            Node ID of which to get the PRes frame.

\return The function returns a \ref tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_triggerPresForward(UINT nodeId_p)
{
#if defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD)
    tEvent      event;

    event.eventSink     = kEventSinkDllk;
    event.netTime.nsec  = 0;
    event.netTime.sec   = 0;
    event.eventType     = kEventTypeRequPresForward;
    event.eventArg.pEventArg = &nodeId_p;
    event.eventArgSize  = sizeof(nodeId_p);

    return eventu_postEvent(&event);
#else
    UNUSED_PARAMETER(nodeId_p);
    return kErrorApiInvalidParam;
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
registered for an SDO transfer. When it is called by the SDO stack it sends an
SDO event to the application.

\param  pSdoComFinished_p   SDO parameter.

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SDOC)
static tOplkError cbSdoCon(tSdoComFinished* pSdoComFinished_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.sdoInfo = *pSdoComFinished_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventSdo, &eventArg);
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Callback function for received ASnds

The function implements the callback function to handle received ASnd frames.
Frames will be forwarded to the application by sending a user event.

\param  pFrameInfo_p   Pointer to information about the received frame.

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbReceivedAsnd(tFrameInfo* pFrameInfo_p)
{
    tOplkError              ret = kErrorOk;
    UINT                    asndOffset;
    tOplkApiEventArg        apiEventArg;
    tOplkApiEventType       eventType;

    // Check for correct input
    asndOffset = offsetof(tPlkFrame, data.asnd);

    if ((pFrameInfo_p->frameSize <= asndOffset + 1) ||
        (pFrameInfo_p->frameSize > C_DLL_MAX_ASYNC_MTU))
        return kErrorReject;

    // Forward received ASnd frame
    apiEventArg.receivedAsnd.pFrame = pFrameInfo_p->frame.pBuffer;
    apiEventArg.receivedAsnd.frameSize = pFrameInfo_p->frameSize;

    eventType = kOplkApiEventReceivedAsnd;
    ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
    return ret;
}

#if defined(CONFIG_INCLUDE_VETH)
//------------------------------------------------------------------------------
/**
\brief  Callback function for received Ethernet frames

The function implements the callback function to handle received Ethernet frames.
Frames will be forwarded to the application by sending a user event.

\param  pFrameInfo_p   Pointer to information about the received frame.

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbReceivedEth(tFrameInfo* pFrameInfo_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.receivedEth.pFrame = pFrameInfo_p->frame.pBuffer;
    eventArg.receivedEth.frameSize = pFrameInfo_p->frameSize;

    ret = ctrlu_callUserEventCallback(kOplkApiEventReceivedNonPlk, &eventArg);

    return ret;
}
#endif

/// \}
