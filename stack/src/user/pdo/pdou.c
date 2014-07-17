/**
********************************************************************************
\file   pdou.c

\brief  Implementation of user PDO module

This file contains the implementation of the user PDO module.

\ingroup module_pdou
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <common/oplkinc.h>
#include <user/pdou.h>
#include <user/pdoucal.h>
#include <common/pdo.h>
#include <common/target.h>
#include <common/ami.h>
#include <oplk/sdoabortcodes.h>
#include <oplk/obd.h>

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
// PDO mapping related OD defines
#define PDOU_OBD_IDX_RX_COMM_PARAM      0x1400
#define PDOU_OBD_IDX_RX_MAPP_PARAM      0x1600
#define PDOU_OBD_IDX_TX_COMM_PARAM      0x1800
#define PDOU_OBD_IDX_TX_MAPP_PARAM      0x1A00
#define PDOU_OBD_IDX_MAPP_PARAM         0x0200
#define PDOU_OBD_IDX_MASK               0xFF00
#define PDOU_PDO_ID_MASK                0x00FF

#define PDOU_MAX_PDO_OBJECTS            256

#define PDO_COMMUNICATION_PROFILE_START 0x1000

#define PDO_MAPPOBJECT_GET_VAR(pPdoMappObject_p) \
            pPdoMappObject_p->pVar

#define PDO_MAPPOBJECT_SET_VAR(pPdoMappObject_p, pVar_p) \
            (pPdoMappObject_p->pVar = pVar_p)

#define PDO_MAPPOBJECT_GET_BITOFFSET(pPdoMappObject_p) \
            pPdoMappObject_p->bitOffset

#define PDO_MAPPOBJECT_SET_BITOFFSET(pPdoMappObject_p, wBitOffset_p) \
            (pPdoMappObject_p->bitOffset = wBitOffset_p)

#define PDO_MAPPOBJECT_GET_BYTESIZE(pPdoMappObject_p) \
            (pPdoMappObject_p->byteSizeOrType - PDO_COMMUNICATION_PROFILE_START)

#define PDO_MAPPOBJECT_GET_TYPE(pPdoMappObject_p) \
            ((tObdType)pPdoMappObject_p->byteSizeOrType)

#define PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pPdoMappObject_p, wByteSize_p, ObdType_p) \
            if ((ObdType_p == kObdTypeVString) || (ObdType_p == kObdTypeOString) || (ObdType_p == kObdTypeDomain)) \
            { \
                pPdoMappObject_p->byteSizeOrType = wByteSize_p + PDO_COMMUNICATION_PROFILE_START; \
            } \
            else \
            { \
                pPdoMappObject_p->byteSizeOrType = ObdType_p; \
            }


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief PDO mapping object

This structure structure specifies a PDO mapping object.
*/
typedef struct
{
    void*               pVar;                   ///< Pointer to PDO data
    UINT16              bitOffset;              ///< Frame offset in bits
    UINT16              byteSizeOrType;         ///< The size of the data in bytes
} tPdoMappObject;

/**
\brief User PDO module instance

The following structure defines the instance variable of the user PDO module.
*/
typedef struct
{
    BYTE                    aPdoIdToChannelIdRx[(PDOU_PDO_ID_MASK + 1)]; ///< RXPDO to channel ID conversion table
#if defined(CONFIG_INCLUDE_NMT_MN)
    // only MN supports multiple TPDOs where indexing is necessary
    BYTE                    aPdoIdToChannelIdTx[(PDOU_PDO_ID_MASK + 1)]; ///< TXPDO to channel ID conversion table
#endif
    tPdoChannelSetup        pdoChannels;                ///< PDO channel setup
    tPdoMappObject*         paRxObject;                 ///< Pointer to RX channel objects
    tPdoMappObject*         paTxObject;                 ///< Pointer to TX channel objects
    BOOL                    fAllocated;                 ///< Flag determines if PDOs are allocated
    BOOL                    fRunning;                   ///< Flag determines if PDO engine is running
    tPdoCbEventPdoChange    pfnCbEventPdoChange;
    //BYTE*                   pPdoMem;                    ///< pointer to PDO memory
} tPdouInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdouInstance  pdouInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError callPdoChangeCb(BOOL fActivated_p, UINT nodeId_p, UINT mappParamIndex_p,
                                  UINT8 mappObjectCount_p, BOOL fTx_p);
static tOplkError setupRxPdoChannelTables(BYTE abChannelIdToPdoIdRx_p[D_PDO_RPDOChannels_U16],
                                          UINT* pCountChannelIdRx_p);
static tOplkError setupTxPdoChannelTables(BYTE abChannelIdToPdoIdTx_p[D_PDO_TPDOChannels_U16],
                                          UINT* pCountChannelIdTx_p);
static tOplkError allocatePdoChannels(tPdoAllocationParam* pAllocationParam_p);
static tOplkError freePdoChannels(void);
static tOplkError configureAllPdos(void);
static tOplkError checkAndConfigurePdos(UINT16 mappParamIndex_p, UINT channelCount_p,
                                        BYTE* pChannelToPdoTable_p, UINT32* pAbortCode_p);
static tOplkError checkAndConfigurePdo(UINT16 mappParamIndex_p, BYTE mappObjectCount_p,
                                       UINT32* pAbortCode_p);
static tOplkError checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p);
static void decodeObjectMapping(QWORD objectMapping_p, UINT* pIndex_p,
                                UINT* pSubIndex_p, UINT* pBitOffset_p,
                                UINT* pBitSize_p);
static tOplkError checkAndSetObjectMapping(QWORD objectMapping_p, tObdAccess neededAccessType_p,
                                           tPdoMappObject* pMappObject_p,
                                           UINT32* pAbortCode_p, UINT* pPdoSize_p);
static tOplkError setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p, BYTE mappObjectCount_p,
                                      UINT16 maxPdoSize_p, UINT32* pAbortCode_p,
                                      UINT* pCalcPdoSize_p, UINT* pCount_p);
static tOplkError configurePdoChannel(tPdoChannelConf* pChannelConf_p);
static tOplkError getMaxPdoSize(BYTE nodeId_p, BOOL fTxPdo_p,
                                UINT16* pMaxPdoSize_p, UINT32* pAbortCode_p);
static tOplkError getPdoChannelId(UINT pdoId_p, BOOL fTxPdo_p, UINT* pChannelId_p);
static UINT calcPdoMemSize(tPdoChannelSetup* pPdoChannels_p, size_t* pRxPdoMemSize_p,
                           size_t* pTxPdoMemSize_p);
static tOplkError copyVarToPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p);
static tOplkError copyVarFromPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user module

The function initializes the PDO user module.

\param  pfnSyncCb_p             function that is called in case of sync event

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_init(tSyncCb pfnSyncCb_p)
{
    OPLK_MEMSET(&pdouInstance_g, 0, sizeof(pdouInstance_g));
    pdouInstance_g.fAllocated = FALSE;
    pdouInstance_g.fRunning = FALSE;
    pdouInstance_g.pfnCbEventPdoChange = NULL;

    return pdoucal_init(pfnSyncCb_p);
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO user module

The function cleans up the PDO user module.

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_exit(void)
{
    pdouInstance_g.fRunning = FALSE;
    pdouInstance_g.pfnCbEventPdoChange = NULL;
    freePdoChannels();
    pdoucal_cleanupPdoMem();
    return pdoucal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state changes

The function implements the callback function which is called on NMT state
changes.

\param  nmtStateChange_p         NMT state change event.

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tOplkError      ret = kErrorOk;

    switch (nmtStateChange_p.newNmtState)
    {
        case kNmtGsOff:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
            if (pdouInstance_g.fAllocated)
            {
                UINT    mapParamIndex;
                UINT32  abortCode;

                for (mapParamIndex = PDOU_OBD_IDX_RX_MAPP_PARAM;
                     mapParamIndex < PDOU_OBD_IDX_RX_MAPP_PARAM + sizeof(pdouInstance_g.aPdoIdToChannelIdRx);
                     mapParamIndex++)
                {
                    ret = checkAndConfigurePdo(mapParamIndex, 0, &abortCode);
                }

                for (mapParamIndex = PDOU_OBD_IDX_TX_MAPP_PARAM;

#if defined(CONFIG_INCLUDE_NMT_MN)
                     mapParamIndex < PDOU_OBD_IDX_TX_MAPP_PARAM + sizeof(pdouInstance_g.aPdoIdToChannelIdRx);
#else
                     mapParamIndex < PDOU_OBD_IDX_TX_MAPP_PARAM + (PDOU_PDO_ID_MASK + 1);
#endif
                     mapParamIndex++)
                {
                    ret = checkAndConfigurePdo(mapParamIndex, 0, &abortCode);
                }

                ret = kErrorOk;
                pdouInstance_g.fAllocated = FALSE;
                pdouInstance_g.fRunning = FALSE;
            }
            break;

        case kNmtGsResetConfiguration:
            pdouInstance_g.fAllocated = FALSE;
            pdouInstance_g.fRunning = FALSE;

            // forward PDO configuration to Pdok module
            ret = configureAllPdos();
            if (ret != kErrorOk)
            {
                goto Exit;
            }
            pdouInstance_g.fRunning = TRUE;
            break;

        default:
            // do nothing
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for OD accesses

The function implements the callback function which is called when an object
is accessed which belongs to the PDO module.

\param  pParam_p                OBD parameter

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_cbObdAccess(tObdCbParam MEM* pParam_p)
{
    tOplkError          ret = kErrorOk;
    UINT                indexType;
    BYTE                mappObjectCount;
    UINT                curPdoSize;
    tObdAccess          neededAccessType;

    pParam_p->abortCode = 0;

    if (pParam_p->obdEvent != kObdEvPreWrite)
    {   // read accesses, post write events etc. are OK
        return ret;
    }

    // fetch object index type
    indexType = pParam_p->index & PDOU_OBD_IDX_MASK;

    // check index type
    switch (indexType)
    {
        case PDOU_OBD_IDX_RX_COMM_PARAM:
        case PDOU_OBD_IDX_TX_COMM_PARAM:
            ret = checkPdoValidity((PDOU_OBD_IDX_MAPP_PARAM | pParam_p->index),
                                   &pParam_p->abortCode);
            return ret;
            break;

        case PDOU_OBD_IDX_RX_MAPP_PARAM:
            // RPDO mapping parameter accessed
            neededAccessType = kObdAccWrite;
            break;

        case PDOU_OBD_IDX_TX_MAPP_PARAM:
            // TPDO mapping parameter accessed
            neededAccessType = kObdAccRead;
            break;

        default:
            // this callback function is only for PDO mapping and communication
            // parameters therfore we shouldn't come here!
            pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
            ret = kErrorPdoInvalidObjIndex;
            return ret;
            break;
    }

    // RPDO and TPDO mapping parameter accessed
    if (pParam_p->subIndex == 0)
    {   // object mapping count accessed
        // PDO is enabled or disabled
        mappObjectCount = *((BYTE*)pParam_p->pArg);
        ret = checkAndConfigurePdo(pParam_p->index, mappObjectCount,
                                   &pParam_p->abortCode);
        if (ret != kErrorOk)
            return ret;
    }
    else
    {
        // ObjectMapping
        tPdoMappObject      mappObject;     // temporary object for check
        QWORD               objectMapping;

        ret = checkPdoValidity(pParam_p->index, &pParam_p->abortCode);
        if (ret != kErrorOk)
        {   // PDO is valid or does not exist
            return ret;
        }

        // check existence of object and validity of object length
        objectMapping = *((QWORD*)pParam_p->pArg);
        ret = checkAndSetObjectMapping(objectMapping, neededAccessType, &mappObject,
                                       &pParam_p->abortCode, &curPdoSize);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy RXPDO to process image

The function copies RXPDOs into the process image

\return The function returns a tOplkError error code.

\ingroup module_pdou
*/
//------------------------------------------------------------------------------
tOplkError pdou_copyRxPdoToPi(void)
{
    tOplkError          Ret;
    UINT                mappObjectCount;
    tPdoChannel*        pPdoChannel;
    tPdoMappObject*     pMappObject;
    UINT                channelId;
    BYTE*               pPdo;

    if (!pdouInstance_g.fRunning)
    {
        DEBUG_LVL_PDO_TRACE("%s() PDO channels not running!\n", __func__);
        return kErrorOk;
    }

    for (channelId = 0;
         channelId < pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount;
         channelId++)
    {
        pPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[channelId];

        if (pPdoChannel->nodeId == PDO_INVALID_NODE_ID)
        {
            continue;
        }

        Ret = pdoucal_getRxPdo(&pPdo, channelId, pPdoChannel->pdoSize);

        //TRACE("%s() Channel:%d Node:%d pPdo:%p\n", __func__, channelId, pPdoChannel->nodeId, pPdo);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paRxObject + (channelId * D_PDO_RPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            Ret = copyVarFromPdo(pPdo, pMappObject);
            if (Ret != kErrorOk)
            {   // other fatal error occurred
                return Ret;
            }
        }
    }
    return kErrorOk;
}


//------------------------------------------------------------------------------
/**
\brief  Copy TXPDO from process image

The function copies the TXPDOs from the process image into the PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdou
*/
//------------------------------------------------------------------------------
tOplkError pdou_copyTxPdoFromPi(void)
{
    tOplkError          ret = kErrorOk;
    UINT                mappObjectCount;
    tPdoChannel*        pPdoChannel;
    tPdoMappObject*     pMappObject;
    UINT                channelId;
    BYTE*               pPdo;

    //TRACE_FUNC_ENTRY;

    if (!pdouInstance_g.fRunning)
    {
        return kErrorOk;
    }

    for (channelId = 0;
         channelId < pdouInstance_g.pdoChannels.allocation.txPdoChannelCount;
         channelId++)
    {
        pPdoChannel = &pdouInstance_g.pdoChannels.pTxPdoChannel[channelId];

        if (pPdoChannel->nodeId == PDO_INVALID_NODE_ID)
        {
            continue;
        }

        pPdo = pdoucal_getTxPdoAdrs(channelId);
        //TRACE("%s() pPdo: %p\n", __func__, pPdo);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paTxObject + (channelId * D_PDO_TPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            ret = copyVarToPdo(pPdo, pMappObject);
            if (ret != kErrorOk)
            {   // other fatal error occurred
                return ret;
            }
        }

        // send PDO data to kernel layer
        ret = pdoucal_setTxPdo(channelId, pPdo, pPdoChannel->pdoSize);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register PDO change callback function

The function is used to register a funtion for receiving PDO change events.

\param  pfnCbEventPdoChange_p       Pointer to callback function.

\return The function returns a tOplkError error code.

\ingroup module_pdou
*/
//------------------------------------------------------------------------------
tOplkError pdou_registerEventPdoChangeCb(tPdoCbEventPdoChange pfnCbEventPdoChange_p)
{
    pdouInstance_g.pfnCbEventPdoChange = pfnCbEventPdoChange_p;
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Call PDO change callback function

The function is used to call the registered callback function for PDO
change events.

\param  fActivated_p        Determines if PDO is activated or disabled.
\param  nodeId_p            Node ID this PDO mapping belongs to.
\param  mappParamIndex_p    Object index of mapping parameter object.
\param  mappObjectCount_p   Number of mapped objects.
\param  fTx_p               TRUE for TXPDO and FALSE for RXPDO.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callPdoChangeCb(BOOL fActivated_p, UINT nodeId_p, UINT mappParamIndex_p,
                                  UINT8 mappObjectCount_p, BOOL fTx_p)
{
    tOplkError              ret = kErrorOk;
    tPdoEventPdoChange      eventPdoChange;
    tObdSize                obdSize;

    if (fActivated_p == FALSE)
    {
        obdSize = sizeof(mappObjectCount_p);
        // read PDO mapping version
        ret = obd_readEntry(mappParamIndex_p, 0x00, &mappObjectCount_p, &obdSize);
        if (ret != kErrorOk)
        {   // other fatal error occurred
            return ret;
        }

        if (mappObjectCount_p == 0)
        {   // PDO already disabled -> do not inform user
            return ret;
        }
    }

    eventPdoChange.fActivated = fActivated_p;
    eventPdoChange.fTx = fTx_p;
    eventPdoChange.nodeId = nodeId_p;
    eventPdoChange.mappParamIndex = mappParamIndex_p;
    eventPdoChange.mappObjectCount = mappObjectCount_p;

    return pdouInstance_g.pfnCbEventPdoChange(&eventPdoChange);
}

//------------------------------------------------------------------------------
/**
\brief  Setup RXPDO channel tables

This function allocates memory for PDO channel tables. It scans the PDO_RxCommParam
objects to calculate the number of "PDO channels". It creates two tables:
- PDO-to-ChannelId mapping
- ChannelId to PDO mapping

The number of PDO channels will be transfered to the PDOk module which allocates
memory to store the mapping information.

\param  abChannelIdToPdoIdRx_p      Pointer to array to store TX channel to PDO
                                    mapping
\param  pCountChannelIdRx_p         Pointer to store number of RX channels

\return The function returns a tOplkError error code.

\internal
**/
//------------------------------------------------------------------------------
static tOplkError setupRxPdoChannelTables(
                       BYTE abChannelIdToPdoIdRx_p[D_PDO_RPDOChannels_U16],
                       UINT* pCountChannelIdRx_p)
{
    tOplkError              ret = kErrorOk;
    tObdSize                obdSize;
    BYTE                    nodeId;
    UINT                    pdoId;
    UINT                    commParamIndex;
    UINT                    channelCount;

    channelCount = 0;

    OPLK_MEMSET(pdouInstance_g.aPdoIdToChannelIdRx, 0,
                sizeof(pdouInstance_g.aPdoIdToChannelIdRx));

    // Loops through RX Mapping objects (0x1400)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_RX_COMM_PARAM;
         pdoId < PDOU_MAX_PDO_OBJECTS;
         pdoId++, commParamIndex++)
    {
        // read node ID from OD (ID:0x14XX Sub:1)
        obdSize = sizeof(nodeId);
        ret = obd_readEntry(commParamIndex, 0x01, &nodeId, &obdSize);
        switch (ret)
        {
            case kErrorObdIndexNotExist:
            case kErrorObdSubindexNotExist:
            case kErrorObdIllegalPart:
                // PDO does not exist
                break;

            case kErrorOk:
                channelCount++;
                if (channelCount > D_PDO_RPDOChannels_U16)
                    return kErrorPdoTooManyPdos;

                pdouInstance_g.aPdoIdToChannelIdRx[pdoId] = (BYTE)channelCount - 1;
                abChannelIdToPdoIdRx_p[channelCount - 1] = (BYTE)pdoId;
                break;

            default:
                return ret;
                break;
        }
    }
    *pCountChannelIdRx_p = channelCount;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Setup TXPDO channel tables

This function allocates memory for PDO channel tables. It scans the PDO_TRxCommParam
objects to calculate the number of "PDO channels". It creates two tables:
- PDO-to-ChannelId mapping
- ChannelId to PDO mapping

The number of PDO channels will be transfered to the PDOk module which allocates
memory to store the mapping information.

\param  abChannelIdToPdoIdTx_p      Pointer to array to store RX channel to PDO
                                    mapping
\param  pCountChannelIdTx_p         Pointer to store number of TX channels

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError setupTxPdoChannelTables(
                        BYTE abChannelIdToPdoIdTx_p[D_PDO_TPDOChannels_U16],
                        UINT* pCountChannelIdTx_p)
{
    tOplkError              ret = kErrorOk;
    tObdSize                obdSize;
    BYTE                    bNodeId;
    UINT                    pdoId;
    UINT                    commParamIndex;
    UINT                    channelCount;

    channelCount = 0;

#if defined(CONFIG_INCLUDE_NMT_MN)
    OPLK_MEMSET(pdouInstance_g.aPdoIdToChannelIdTx, 0,
                sizeof(pdouInstance_g.aPdoIdToChannelIdTx));
#endif

    abChannelIdToPdoIdTx_p[0] = 0;

    // Loops through TX Mapping objects (0x1800)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_TX_COMM_PARAM;
         pdoId < PDOU_MAX_PDO_OBJECTS;
         pdoId++, commParamIndex++)
    {
        obdSize = sizeof(bNodeId);
        // read node ID from OD (ID:0x18XX Sub:1)
        ret = obd_readEntry(commParamIndex, 0x01, &bNodeId, &obdSize);
        switch (ret)
        {
            case kErrorObdIndexNotExist:
            case kErrorObdSubindexNotExist:
            case kErrorObdIllegalPart:
                // PDO does not exist
                break;

            case kErrorOk:
                channelCount ++;
                if (channelCount > D_PDO_TPDOChannels_U16)
                    return kErrorPdoTooManyTxPdos;

#if defined(CONFIG_INCLUDE_NMT_MN)
                pdouInstance_g.aPdoIdToChannelIdTx[pdoId] = (BYTE)channelCount - 1;
#endif
                abChannelIdToPdoIdTx_p[channelCount - 1] = (BYTE)pdoId;
                break;

            default:
                return ret;
                break;
        }
    }
    *pCountChannelIdTx_p = channelCount;

    DEBUG_LVL_PDO_TRACE("%s() TX channel count: %d\n", __func__, channelCount);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels

This function allocates memory for PDOs channels

\param  pAllocationParam_p      Pointer to allocation parameters.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError allocatePdoChannels(tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError      ret = kErrorOk;
    UINT            index;

    if (pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount != pAllocationParam_p->rxPdoChannelCount)
    {   // allocation should be changed
        pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount = pAllocationParam_p->rxPdoChannelCount;
        if (pdouInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            OPLK_FREE(pdouInstance_g.pdoChannels.pRxPdoChannel);
            pdouInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }

        if (pdouInstance_g.paRxObject != NULL)
        {
            OPLK_FREE(pdouInstance_g.paRxObject);
            pdouInstance_g.paRxObject = NULL;
        }

        if (pAllocationParam_p->rxPdoChannelCount > 0)
        {
            pdouInstance_g.pdoChannels.pRxPdoChannel =
                  (tPdoChannel*)OPLK_MALLOC(sizeof(tPdoChannel) * pAllocationParam_p->rxPdoChannelCount);

            if (pdouInstance_g.pdoChannels.pRxPdoChannel == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }

            pdouInstance_g.paRxObject =
                    (tPdoMappObject*)OPLK_MALLOC(sizeof(tPdoMappObject)
                               * pAllocationParam_p->rxPdoChannelCount
                               * D_PDO_RPDOChannelObjects_U8);

            if (pdouInstance_g.paRxObject == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }
        }
    }

    // disable all RPDOs
    for (index = 0; index < pAllocationParam_p->rxPdoChannelCount; index++)
    {
        pdouInstance_g.pdoChannels.pRxPdoChannel[index].nodeId = PDO_INVALID_NODE_ID;
    }

    //--------------------------------------------------------------------------
    if (pdouInstance_g.pdoChannels.allocation.txPdoChannelCount != pAllocationParam_p->txPdoChannelCount)
    {   // allocation should be changedconfigureChannel

        pdouInstance_g.pdoChannels.allocation.txPdoChannelCount = pAllocationParam_p->txPdoChannelCount;
        if (pdouInstance_g.pdoChannels.pTxPdoChannel != NULL)
        {
            OPLK_FREE(pdouInstance_g.pdoChannels.pTxPdoChannel);
            pdouInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }

        if (pdouInstance_g.paTxObject != NULL)
        {
            OPLK_FREE(pdouInstance_g.paTxObject);
            pdouInstance_g.paTxObject = NULL;
        }

        if (pAllocationParam_p->txPdoChannelCount > 0)
        {
            pdouInstance_g.pdoChannels.pTxPdoChannel =
                    (tPdoChannel*)OPLK_MALLOC(sizeof(tPdoChannel) * pAllocationParam_p->txPdoChannelCount);
            if (pdouInstance_g.pdoChannels.pTxPdoChannel == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }

            pdouInstance_g.paTxObject =
                    (tPdoMappObject*)OPLK_MALLOC(sizeof(tPdoMappObject)
                               * pAllocationParam_p->txPdoChannelCount
                               * D_PDO_TPDOChannelObjects_U8);
            if (pdouInstance_g.paTxObject == NULL)
            {
                ret = kErrorPdoInitError;
                goto Exit;
            }
        }
    }

    // disable all TPDOs
    for (index = 0; index < pAllocationParam_p->txPdoChannelCount; index++)
    {
        pdouInstance_g.pdoChannels.pTxPdoChannel[index].nodeId = PDO_INVALID_NODE_ID;
    }

Exit:
    //TRACE("%s() = %s\n", __func__, debugstr_getRetValStr(Ret));
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels

This function frees memory of PDOs channels

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError freePdoChannels(void)
{
    tOplkError      ret = kErrorOk;

    if (pdouInstance_g.pdoChannels.pRxPdoChannel != NULL)
    {
        OPLK_FREE(pdouInstance_g.pdoChannels.pRxPdoChannel);
        pdouInstance_g.pdoChannels.pRxPdoChannel = NULL;
    }

    if (pdouInstance_g.paRxObject != NULL)
    {
        OPLK_FREE(pdouInstance_g.paRxObject);
        pdouInstance_g.paRxObject = NULL;
    }

    if (pdouInstance_g.pdoChannels.pTxPdoChannel != NULL)
    {
        OPLK_FREE(pdouInstance_g.pdoChannels.pTxPdoChannel);
        pdouInstance_g.pdoChannels.pTxPdoChannel = NULL;
    }

    if (pdouInstance_g.paTxObject != NULL)
    {
        OPLK_FREE(pdouInstance_g.paTxObject);
        pdouInstance_g.paTxObject = NULL;
    }

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    configure all PDOs in Pdok module

The function configures the whole PDO mapping information in the Pdok module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError configureAllPdos(void)
{
    tOplkError              ret = kErrorOk;
    BYTE                    aChannelIdToPdoIdRx[D_PDO_RPDOChannels_U16];
    BYTE                    aChannelIdToPdoIdTx[D_PDO_TPDOChannels_U16];
    tPdoAllocationParam     allocParam;
    UINT32                  abortCode = 0;
    size_t                  txPdoMemSize;
    size_t                  rxPdoMemSize;

    ret = setupRxPdoChannelTables(aChannelIdToPdoIdRx, &allocParam.rxPdoChannelCount);
    if (ret != kErrorOk)
        goto Exit;
    ret = setupTxPdoChannelTables(aChannelIdToPdoIdTx, &allocParam.txPdoChannelCount);
    if (ret != kErrorOk)
        goto Exit;

    ret = allocatePdoChannels(&allocParam);
    if (ret != kErrorOk)
        goto Exit;
    ret = pdoucal_postPdokChannelAlloc(&allocParam);
    if (ret != kErrorOk)
        goto Exit;
    pdouInstance_g.fAllocated = TRUE;

    // configure the PDOs
    ret = checkAndConfigurePdos(PDOU_OBD_IDX_RX_MAPP_PARAM,
                                allocParam.rxPdoChannelCount, aChannelIdToPdoIdRx,
                                &abortCode);
    if (ret != kErrorOk)
        goto Exit;

    ret = checkAndConfigurePdos(PDOU_OBD_IDX_TX_MAPP_PARAM,
                                allocParam.txPdoChannelCount, aChannelIdToPdoIdTx,
                                &abortCode);
    if (ret != kErrorOk)
        goto Exit;

    calcPdoMemSize(&pdouInstance_g.pdoChannels, &rxPdoMemSize, &txPdoMemSize);
    pdoucal_postSetupPdoBuffers(rxPdoMemSize, txPdoMemSize);

    // TODO how to be sure that kernel is ready before starting??
    target_msleep(500);

    ret = pdoucal_initPdoMem(&pdouInstance_g.pdoChannels, rxPdoMemSize,
                             txPdoMemSize);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  checks and configures all PDOs for a single direction

The functions checks and configures all PDOs for a single direction.

\param  mappParamIndex_p        ID of the mapping parameter object.
\param  channelCount_p          Number of PDO channels.
\param  pChannelToPdoTable_p    Pointer to channel-PDO table.
\param  pAbortCode_p            Pointer to store the abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndConfigurePdos(UINT16 mappParamIndex_p, UINT channelCount_p,
                                        BYTE* pChannelToPdoTable_p, UINT32* pAbortCode_p)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tObdSize            obdSize;
    BYTE                mappObjectCount;
    UINT                mappParamIndex;

    for (index = 0; index < channelCount_p; index++)
    {
        // Calculate the MappParam object ID for the channel
        mappParamIndex = mappParamIndex_p + *(pChannelToPdoTable_p + index);

        obdSize = sizeof(mappObjectCount);
        // read mapping object count from OD
        ret = obd_readEntry(mappParamIndex, 0x00, &mappObjectCount, &obdSize);
        if (ret != kErrorOk)
            return ret;

        ret = checkAndConfigurePdo(mappParamIndex, mappObjectCount, pAbortCode_p);
        if (ret != kErrorOk)
            return ret;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  checks and configures the specified PDO

The function checks and configures a specified PDO.

\param  mappParamIndex_p            Object ID of PDO-MappParam object.
\param  mappObjectCount_p           Number of mapped objects.
\param  pAbortCode_p                Pointer to store abort code

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndConfigurePdo(UINT16 mappParamIndex_p,
                                       BYTE  mappObjectCount_p, UINT32* pAbortCode_p)
{
    tOplkError          ret = kErrorOk;
    UINT16              pdoId;
    UINT16              commParamIndex;
    tObdSize            obdSize;
    BYTE                nodeId;
    WORD                maxPdoSize;
    tPdoChannelConf     pdoChannelConf;
    BOOL                fTxPdo;
    tPdoMappObject*     pMappObject;
    UINT                calcPdoSize;
    UINT                count;

    DEBUG_LVL_PDO_TRACE("%s() mappParamIndex:%04x mappObjectCount:%d\n",
                        __func__, mappParamIndex_p, mappObjectCount_p);

    pdoId = mappParamIndex_p & PDOU_PDO_ID_MASK;
    commParamIndex = ~PDOU_OBD_IDX_MAPP_PARAM & mappParamIndex_p;
    fTxPdo = (mappParamIndex_p >= PDOU_OBD_IDX_TX_MAPP_PARAM) ? TRUE : FALSE;

    if ((!fTxPdo && (mappObjectCount_p > D_PDO_RPDOChannelObjects_U8)) ||
        (fTxPdo && (mappObjectCount_p > D_PDO_TPDOChannelObjects_U8)))
    {
        DEBUG_LVL_ERROR_TRACE("%s() %d exceeds object!\n",
                              __func__, mappObjectCount_p);
        *pAbortCode_p = SDO_AC_VALUE_RANGE_EXCEEDED;
        ret = kErrorObdValueTooHigh;
        goto Exit;
    }

    pdoChannelConf.fTx = fTxPdo;
    ret = getPdoChannelId(pdoId, fTxPdo, &pdoChannelConf.channelId);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error get channelID!\n", __func__);
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        ret = kErrorPdoInvalidObjIndex;
        goto Exit;
    }

    // read node ID from OD
    obdSize = sizeof(nodeId);
    ret = obd_readEntry(commParamIndex, 0x01, &nodeId, &obdSize);
    if (ret != kErrorOk)
    {   // fatal error occurred
        goto Exit;
    }
    pdoChannelConf.pdoChannel.nodeId = nodeId;

    if (mappObjectCount_p == 0)
    {   // PDO shall be disabled (see 6.4.9.2)
        pdoChannelConf.pdoChannel.nodeId = PDO_INVALID_NODE_ID;
        pdoChannelConf.pdoChannel.mappObjectCount = 0;
        pdoChannelConf.pdoChannel.pdoSize = 0;
        pdouInstance_g.fRunning = FALSE;
        ret = configurePdoChannel(&pdoChannelConf);

        if ((pdouInstance_g.fAllocated) && (pdouInstance_g.pfnCbEventPdoChange != NULL))
        {
            ret = callPdoChangeCb(FALSE, nodeId, mappParamIndex_p,
                                  mappObjectCount_p, pdoChannelConf.fTx);
            if (ret != kErrorOk)
                 *pAbortCode_p = SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL;
        }
        goto Exit;
    }

    if (!pdouInstance_g.fAllocated)
        goto Exit;

    // PDO shall be enabled
    ret = checkPdoValidity(mappParamIndex_p, pAbortCode_p);
    if (ret != kErrorOk)
    {   // PDO is invalid or does not exist
        goto Exit;
    }

    obdSize = sizeof(pdoChannelConf.pdoChannel.mappingVersion);
    // read PDO mapping version
    ret = obd_readEntry(commParamIndex, 0x02,
                        &pdoChannelConf.pdoChannel.mappingVersion, &obdSize);
    if (ret != kErrorOk)
    {   // other fatal error occurred
        goto Exit;
    }

    ret = getMaxPdoSize(nodeId, fTxPdo, &maxPdoSize, pAbortCode_p);
    if (ret != kErrorOk)
    {   // PDO is valid or does not exist
        goto Exit;
    }

    if (fTxPdo)
        pMappObject = &pdouInstance_g.paTxObject[pdoChannelConf.channelId *
                                                 D_PDO_TPDOChannelObjects_U8];
    else
        pMappObject = &pdouInstance_g.paRxObject[pdoChannelConf.channelId *
                                                 D_PDO_RPDOChannelObjects_U8];

    ret = setupMappingObjects(pMappObject, mappParamIndex_p, mappObjectCount_p,
                              maxPdoSize, pAbortCode_p, &calcPdoSize, &count);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
    pdoChannelConf.pdoChannel.pdoSize = calcPdoSize;
    pdoChannelConf.pdoChannel.mappObjectCount = count;

    // do not make the call before Alloc has been called
    ret = configurePdoChannel(&pdoChannelConf);
    if (ret != kErrorOk)
    {   // fatal error occurred
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        goto Exit;
    }

    if ((pdouInstance_g.fAllocated) && (pdouInstance_g.pfnCbEventPdoChange != NULL))
    {
        ret = callPdoChangeCb(TRUE, nodeId, mappParamIndex_p,
                              mappObjectCount_p, pdoChannelConf.fTx);
        if (ret != kErrorOk)
            *pAbortCode_p = SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure the specified PDO channel

The function configures the specified PDO channel.

\param  pChannelConf_p              PDO channel configuration

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError configurePdoChannel(tPdoChannelConf* pChannelConf_p)
{
    tOplkError          ret = kErrorOk;
    tPdoChannel*        pDestPdoChannel;

    if (pdouInstance_g.fAllocated != FALSE)
    {
        if (pChannelConf_p->fTx)
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId];
        else
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[pChannelConf_p->channelId];

        // Setup user channel configuration
        OPLK_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel, sizeof(tPdoChannel));

        // TRACE("postConfigureChannel: TX:%d channel:%d size:%d\n",
        //       pChannelConf_p->fTx, pChannelConf_p->channelId, pChannelConf_p->pdoChannel.pdoSize);
        ret = pdoucal_postConfigureChannel(pChannelConf_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  get max PDO size

The function reads the maximum allows PDO size from the object dictionary.
Depending on the mapping object the right payload limit objects will be
read.

\param  nodeId_p                Node ID of mapped object.
\param  fTxPdo_p                True for TXPDO, false for RXPDO.
\param  pMaxPdoSize_p           Pointer to store maximum PDO size.
\param  pAbortCode_p            Pointer to store abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getMaxPdoSize(BYTE nodeId_p, BOOL fTxPdo_p,
                                UINT16* pMaxPdoSize_p, UINT32* pAbortCode_p)
{
    tOplkError          ret = kErrorOk;
    tObdSize            obdSize;
    WORD                maxPdoSize;
    UINT                payloadLimitIndex;
    UINT                payloadLimitSubIndex;
    UINT8               subIndexCount;

    // Get right payload limit object depending on 1) MN/CN 2) RPDO/TPDO
    if (fTxPdo_p)
    {
        if (nodeId_p == PDO_PRES_NODE_ID)
        {
            payloadLimitIndex = 0x1F98;   // NMT_CycleTiming_REC
            payloadLimitSubIndex = 0x05;  // PResActPayloadLimit_U16
        }
        else
        {
            payloadLimitIndex = 0x1F8B;   // NMT_MNPReqPayloadLimitList_AU16
            payloadLimitSubIndex = nodeId_p;
            obdSize = sizeof(subIndexCount);
            ret = obd_readEntry(payloadLimitIndex, 0, &subIndexCount, &obdSize);
            if (ret != kErrorOk)
            {   // other fatal error occurred
                *pAbortCode_p = SDO_AC_GENERAL_ERROR;
                return ret;
            }
            if (subIndexCount < payloadLimitSubIndex)
            {   // sub-index is not valid
                *pAbortCode_p = SDO_AC_GEN_PARAM_INCOMPATIBILITY;
                return kErrorPdoLengthExceeded;
            }
        }
    }
    else
    {
        if (nodeId_p == PDO_PREQ_NODE_ID)
        {
            payloadLimitIndex = 0x1F98;   // NMT_CycleTiming_REC
            payloadLimitSubIndex = 0x04;  // PReqActPayloadLimit_U16
        }
        else
        {
            payloadLimitIndex = 0x1F8D;   // NMT_PResPayloadLimitList_AU16
            payloadLimitSubIndex = nodeId_p;
            obdSize = sizeof(subIndexCount);
            ret = obd_readEntry(payloadLimitIndex, 0, &subIndexCount, &obdSize);
            if (ret != kErrorOk)
             {   // other fatal error occurred
                 *pAbortCode_p = SDO_AC_GENERAL_ERROR;
                 return ret;
             }
             if (subIndexCount < payloadLimitSubIndex)
             {   // sub-index is not valid
                 *pAbortCode_p = SDO_AC_GEN_PARAM_INCOMPATIBILITY;
                 return kErrorPdoLengthExceeded;
             }
        }
    }

    // fetch maximum PDO size from OD
    obdSize = sizeof(maxPdoSize);
    ret = obd_readEntry(payloadLimitIndex, payloadLimitSubIndex,
                        &maxPdoSize, &obdSize);
    if (ret != kErrorOk)
    {   // other fatal error occurred
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
    }
    else
    {
        *pMaxPdoSize_p = maxPdoSize;
    }
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Convert RPDO to channel ID

The function converts RPDO-ID (i.e. lower part of object index) to channel IDs.

\param  pdoId_p                     ID of PDO (node ID).
\param  fTxPdo_p                    TRUE for TXPDO or FALSE for RXPDO.
\param  pChannelId_p                Pointer to store channel ID.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError getPdoChannelId(UINT pdoId_p, BOOL fTxPdo_p, UINT* pChannelId_p)
{
    tOplkError          Ret = kErrorOk;

    if (fTxPdo_p)
    {
        // TPDO mapping parameter accessed
#if defined(CONFIG_INCLUDE_NMT_MN)
        *pChannelId_p = pdouInstance_g.aPdoIdToChannelIdTx[pdoId_p];
#else
        *pChannelId_p = 0;
#endif
    }
    else
    {
        // RPDO mapping parameter accessed
        *pChannelId_p = pdouInstance_g.aPdoIdToChannelIdRx[pdoId_p];
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  check if PDO is valid

The function checks if the PDO is valid. The PDO is valid, if the PDOs not yet
configured or if the mapping of this POD is disabled.

\param  mappParamIndex_p            Object index of mapping parameter.
\param  pAbortCode_p                Pointer to store SDO abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p)
{
    tOplkError          ret = kErrorOk;
    tObdSize            obdSize;
    BYTE                mappObjectCount;

    if (pdouInstance_g.fRunning)
    {
        // outside from NMT reset states the PDO should have been disabled before changing it
        obdSize = sizeof(mappObjectCount);
        // read number of mapped objects from OD; this indicates if the PDO is valid
        ret = obd_readEntry(mappParamIndex_p, 0x00, &mappObjectCount, &obdSize);
        if (ret != kErrorOk)
        {   // other fatal error occurred
            *pAbortCode_p = SDO_AC_GEN_INTERNAL_INCOMPATIBILITY;
        }
        else
        {
            // entry read successfully
            if (mappObjectCount != 0)
            {   // PDO in OD is still valid
                *pAbortCode_p = SDO_AC_GEN_PARAM_INCOMPATIBILITY;
                ret = kErrorPdoConfWhileEnabled;
            }
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the mapping objects in a PDO channel

\param  objectMapping_p             Object mapping entry.
\param  neededAccessType_p          The needed access type.
\param  pMappObject_p               Pointer to mapping object structure
                                    which will be filled out.
\param  pAbortCode_p                Pointer to SDO abort code;
                                    0 if mapping is possible
\param  pPdoSize_p                  Pointer to covered PDO size
                                    (offset + size) in [byte];
                                    0 if mapping failed

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndSetObjectMapping(QWORD objectMapping_p,
                                           tObdAccess neededAccessType_p,
                                           tPdoMappObject* pMappObject_p,
                                           UINT32* pAbortCode_p, UINT* pPdoSize_p)
{
    tOplkError          ret = kErrorOk;
    tObdSize            obdSize;
    UINT                index;
    UINT                subIndex;
    UINT                bitOffset;
    UINT                bitSize;
    UINT                byteSize;
    tObdAccess          accessType;
    BOOL                fNumerical;
    tObdType            obdType;
    void*               pVar;

    if (objectMapping_p == 0)
    {   // discard zero value
        *pPdoSize_p = 0;
        goto Exit;
    }

    // decode object mapping
    decodeObjectMapping(objectMapping_p, &index, &subIndex, &bitOffset, &bitSize);

    if ((bitOffset & 0x7) != 0x0)
    {   // bit mapping is not supported
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        ret = kErrorPdoGranularityMismatch;
        goto Exit;
    }

    ret = obd_getType(index, subIndex, &obdType);
    if (ret != kErrorOk)
    {   // entry doesn't exist
        *pAbortCode_p = SDO_AC_OBJECT_NOT_EXIST;
        ret = kErrorPdoVarNotFound;
        goto Exit;
    }

    if (((bitSize & 0x7) != 0x0) &&
        ((bitSize != 1) || (obdType != kObdTypeBool)))
    {   // bit mapping is not supported, except for BOOLEAN objects on byte boundaries
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        ret = kErrorPdoGranularityMismatch;
        goto Exit;
    }

    // check access type
    ret = obd_getAccessType(index, subIndex, &accessType);
    if (ret != kErrorOk)
    {   // entry doesn't exist
        *pAbortCode_p = SDO_AC_OBJECT_NOT_EXIST;
        ret = kErrorPdoVarNotFound;
        goto Exit;
    }

    if ((accessType & kObdAccPdo) == 0)
    {   // object is not mappable
        *pAbortCode_p = SDO_AC_OBJECT_NOT_MAPPABLE;
        ret = kErrorPdoVarNotMappable;
        goto Exit;
    }

    if ((accessType & neededAccessType_p) == 0)
    {   // object is not writeable (RPDO) or readable (TPDO) respectively
        *pAbortCode_p = SDO_AC_OBJECT_NOT_MAPPABLE;
        ret = kErrorPdoVarNotMappable;
        goto Exit;
    }

    if (obdType == kObdTypeBool)
    {   // bit size of BOOLEAN object was checked above
        byteSize = 1;
    }
    else
    {
        byteSize = (bitSize >> 3);
    }

    obdSize = obd_getDataSize(index, subIndex);
    if (obdSize < byteSize)
    {   // object does not exist or has smaller size
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        ret = kErrorPdoSizeMismatch;
        // todo really don't want to exit here?
    }

    ret = obd_isNumerical(index, subIndex, &fNumerical);
    if (ret != kErrorOk)
    {   // entry doesn't exist
        *pAbortCode_p = SDO_AC_OBJECT_NOT_EXIST;
        goto Exit;
    }

    if ((fNumerical != FALSE) && (byteSize != obdSize))
    {
        // object is numerical,
        // therefore size has to fit, but it does not.
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
        ret = kErrorPdoVarNotFound;
        goto Exit;
    }

    pVar = obd_getObjectDataPtr(index, subIndex);
    if (pVar == NULL)
    {   // entry doesn't exist
        *pAbortCode_p = SDO_AC_OBJECT_NOT_EXIST;
        ret = kErrorPdoVarNotFound;
        goto Exit;
    }

    // setup mapping object
    PDO_MAPPOBJECT_SET_BITOFFSET(pMappObject_p, (WORD)bitOffset);
    PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pMappObject_p, (WORD)byteSize, obdType);
    PDO_MAPPOBJECT_SET_VAR(pMappObject_p, pVar);

    // Calculate needed PDO size
    *pPdoSize_p = (bitOffset >> 3) + byteSize;

Exit:
    //TRACE("%s() = %s\n", __func__, debugstr_getRetValStr(ret));
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  setup mapping objects in PDO channel configuration

The function sets up the mapping objects of a PDO channel.

\param  pMappObject_p           Pointer to PDO mapping object.
\param  mappParamIndex_p        ID of mapping parameter object.
\param  mappObjectCount_p       Number of mapping objects.
\param  maxPdoSize_p            Maximum PDO size.
\param  pAbortCode_p            Pointer to store the abort code.
\param  pCalcPdoSize_p          Pointer to store the calculated PDO size.
\param  pCount_p                Pointer to store number of mapped objects.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p, BYTE mappObjectCount_p,
                                      UINT16  maxPdoSize_p, UINT32* pAbortCode_p,
                                      UINT* pCalcPdoSize_p, UINT* pCount_p)
{
    tOplkError          ret;
    tObdSize            obdSize;
    QWORD               objectMapping;
    UINT                count;
    BYTE                mappSubindex;
    BYTE                mappObjectCount;
    UINT                curPdoSize;
    WORD                calcPdoSize = 0;
    tObdAccess          neededAccessType;

    mappObjectCount = mappObjectCount_p;
    count = 0;

    for (mappSubindex = 1; mappSubindex <= mappObjectCount; mappSubindex++)
    {
        // read object mapping from OD
        obdSize = sizeof(objectMapping); //&pdouInstance_g.pRxPdoChannel[0] QWORD
        ret = obd_readEntry(mappParamIndex_p, mappSubindex, &objectMapping,
                            &obdSize);
        if (ret != kErrorOk)
        {   // other fatal error occurred
            *pAbortCode_p = SDO_AC_GENERAL_ERROR;
            goto Exit;
        }

        if (mappParamIndex_p >= PDOU_OBD_IDX_TX_MAPP_PARAM)
            neededAccessType = kObdAccRead;
        else
            neededAccessType = kObdAccWrite;

        ret = checkAndSetObjectMapping(objectMapping, neededAccessType, pMappObject_p,
                                       pAbortCode_p, &curPdoSize);
        if (ret != kErrorOk)
        {   // illegal object mapping
            goto Exit;
        }

        if (curPdoSize == 0)
        {   // null mapping, go to next subindex
            continue;
        }

        if (curPdoSize > maxPdoSize_p)
        {   // mapping exceeds object size
            *pAbortCode_p = SDO_AC_PDO_LENGTH_EXCEEDED;
            ret = kErrorPdoLengthExceeded;
            goto Exit;
        }

        if (curPdoSize > calcPdoSize)
        {
            calcPdoSize = (WORD)curPdoSize;
        }

        pMappObject_p++;
        count ++;
    }

    *pCalcPdoSize_p = calcPdoSize;
    *pCount_p = count;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  decode object mapping entry

The function decodes the given object mapping entry into index, subindex,
bit offset and bit size.

\param  objectMapping_p         Object mapping entry
\param  pIndex_p                Pointer to store object index.
\param  pSubIndex_p             Pointer to store subindex.
\param  pBitOffset_p            Pointer to store bit offset.
\param  pBitSize_p              Pointer to store bit size.
*/
//------------------------------------------------------------------------------
static void decodeObjectMapping(QWORD objectMapping_p, UINT* pIndex_p,
                                UINT* pSubIndex_p, UINT* pBitOffset_p,
                                UINT* pBitSize_p)
{
    *pIndex_p =     (UINT) (objectMapping_p & 0x000000000000FFFFLL);
    *pSubIndex_p =  (UINT)((objectMapping_p & 0x0000000000FF0000LL) >> 16);
    *pBitOffset_p = (UINT)((objectMapping_p & 0x0000FFFF00000000LL) >> 32);
    *pBitSize_p =   (UINT)((objectMapping_p & 0xFFFF000000000000LL) >> 48);
}

//------------------------------------------------------------------------------
/**
\brief  Copy variable to PDO

This function copies a variable specified by the mapping object to the PDO
payload.

\param  pPayload_p          Pointer to PDO payload in destination frame.
\param  pMappObject_p       Pointer to mapping object.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError copyVarToPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p)
{
    tOplkError      ret = kErrorOk;
    UINT            byteOffset;
    void*           pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p += byteOffset;
    pVar = PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kObdTypeVString:
        case kObdTypeOString:
        case kObdTypeDomain:
        default:
            // read value from object
            OPLK_MEMCPY(pPayload_p, pVar, PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));
            break;

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
            ami_setUint8Le(pPayload_p, *((BYTE*)pVar));
            break;

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
            ami_setUint16Le(pPayload_p, *((WORD*)pVar));
            break;

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
            ami_setUint24Le(pPayload_p, *((DWORD*)pVar));
            break;

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            ami_setUint32Le(pPayload_p, *((DWORD*)pVar));
            break;

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
            ami_setUint40Le(pPayload_p, *((QWORD*)pVar));
            break;

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
            ami_setUint48Le(pPayload_p, *((QWORD*)pVar));
            break;

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
            ami_setUint56Le(pPayload_p, *((QWORD*)pVar));
            break;

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            ami_setUint64Le(pPayload_p, *((QWORD*)pVar));
            break;

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_setTimeOfDay(pPayload_p, ((tTimeOfDay*)pVar));
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy variable from PDO

This function copies a variable specified by the mapping object from the PDO
payload.

\param  pPayload_p                  Pointer to PDO payload in destination frame.
\param  pMappObject_p               Pointer to mapping object.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError copyVarFromPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p)
{
    tOplkError      Ret = kErrorOk;
    UINT            byteOffset;
    void*           pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p += byteOffset;
    pVar = PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kObdTypeVString:
        case kObdTypeOString:
        case kObdTypeDomain:
        default:
            // read value from object
            OPLK_MEMCPY(pVar, pPayload_p, PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));
            break;

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
            *((BYTE*)pVar) = ami_getUint8Le(pPayload_p);
            break;

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
            *((WORD*)pVar) = ami_getUint16Le(pPayload_p);
            break;

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
            *((DWORD*)pVar) = ami_getUint24Le(pPayload_p);
            break;

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            *((DWORD*)pVar) = ami_getUint32Le(pPayload_p);
            break;

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
            *((QWORD*)pVar) = ami_getUint40Le(pPayload_p);
            break;

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
            *((QWORD*)pVar) = ami_getUint48Le(pPayload_p);
            break;

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
            *((QWORD*)pVar) = ami_getUint56Le(pPayload_p);
            break;

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            *((QWORD*)pVar) = ami_getUint64Le(pPayload_p);
            break;

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_getTimeOfDay(pVar, ((tTimeOfDay*)pPayload_p));
            break;
    }
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate PDO memory size

The function calculates the size needed for the PDO memory.

\param  pPdoChannels_p      Pointer to PDO channel setup.
\param  pRxPdoMemSize_p     Pointer to store size of RX PDO buffers.
\param  pTxPdoMemSize_p     Pointer to store size of TX PDO buffers.

\return The function returns the size of the used PDO memory
*/
//------------------------------------------------------------------------------
static UINT calcPdoMemSize(tPdoChannelSetup* pPdoChannels_p, size_t* pRxPdoMemSize_p,
                           size_t* pTxPdoMemSize_p)
{
    UINT                channelId;
    size_t              rxSize;
    size_t              txSize;
    tPdoChannel*        pPdoChannel;

    rxSize = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        rxSize += pPdoChannel->pdoSize;
    }
    if (pRxPdoMemSize_p != NULL)
        *pRxPdoMemSize_p = rxSize;

    txSize = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        txSize += pPdoChannel->pdoSize;
    }
    if (pTxPdoMemSize_p != NULL)
        *pTxPdoMemSize_p = txSize;

    return rxSize + txSize;
}

///\}
