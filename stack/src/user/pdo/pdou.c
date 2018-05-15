/**
********************************************************************************
\file   pdou.c

\brief  Implementation of user PDO module

This file contains the implementation of the user PDO module.

\ingroup module_pdou
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <user/obdu.h>

#include <common/pdo.h>
#include <common/target.h>
#include <common/ami.h>

#include <oplk/sdoabortcodes.h>
#include <oplk/debugstr.h>

#include <limits.h>

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

#define PDO_MAPPOBJECT_SET_BITOFFSET(pPdoMappObject_p, bitOffset_p) \
            (pPdoMappObject_p->bitOffset = bitOffset_p)

#define PDO_MAPPOBJECT_GET_BYTESIZE(pPdoMappObject_p) \
            (pPdoMappObject_p->byteSizeOrType - PDO_COMMUNICATION_PROFILE_START)

#define PDO_MAPPOBJECT_GET_TYPE(pPdoMappObject_p) \
            ((tObdType)pPdoMappObject_p->byteSizeOrType)

#define PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pPdoMappObject_p, byteSize_p, obdType_p) \
            if ((obdType_p == kObdTypeVString) || (obdType_p == kObdTypeOString) || (obdType_p == kObdTypeDomain)) \
            { \
                pPdoMappObject_p->byteSizeOrType = byteSize_p + PDO_COMMUNICATION_PROFILE_START; \
            } \
            else \
            { \
                pPdoMappObject_p->byteSizeOrType = obdType_p; \
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
    void*                   pVar;                   ///< Pointer to PDO data
    UINT16                  bitOffset;              ///< Frame offset in bits
    UINT16                  byteSizeOrType;         ///< The size of the data in bytes
} tPdoMappObject;

/**
\brief User PDO module instance

The following structure defines the instance variable of the user PDO module.
*/
typedef struct
{
    UINT8                   aPdoIdToChannelIdRx[(PDOU_PDO_ID_MASK + 1)]; ///< RXPDO to channel ID conversion table
    UINT8                   aPdoIdToChannelIdTx[(PDOU_PDO_ID_MASK + 1)]; ///< TXPDO to channel ID conversion table
    tPdoChannelSetup        pdoChannels;                ///< PDO channel setup
    tPdoMappObject*         paRxObject;                 ///< Pointer to RX channel objects
    tPdoMappObject*         paTxObject;                 ///< Pointer to TX channel objects
    BOOL                    fAllocated;                 ///< Flag determines if PDOs are allocated
    BOOL                    fRunning;                   ///< Flag determines if PDO engine is running
    BOOL                    fInitialized;               ///< Flag determines if PDO module is initialized
    tPdoCbEventPdoChange    pfnCbEventPdoChange;
    OPLK_MUTEX_T            lockMutex;                  ///< Mutex used to protect stack from disabling PDOs while copy is in progress
} tPdouInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdouInstance  pdouInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError callPdoChangeCb(BOOL fActivated_p,
                                  UINT nodeId_p,
                                  UINT mappParamIndex_p,
                                  UINT8 mappObjectCount_p,
                                  BOOL fTx_p);
static tOplkError setupRxPdoChannelTables(UINT8 abChannelIdToPdoIdRx_p[D_PDO_RPDOChannels_U16],
                                          UINT* pCountChannelIdRx_p);
static tOplkError setupTxPdoChannelTables(UINT8 abChannelIdToPdoIdTx_p[D_PDO_TPDOChannels_U16],
                                          UINT* pCountChannelIdTx_p);
static tOplkError allocatePdoChannels(const tPdoAllocationParam* pAllocationParam_p);
static tOplkError freePdoChannels(void);
static tOplkError configureAllPdos(void);
static tOplkError checkAndConfigurePdos(UINT16 mappParamIndex_p,
                                        UINT channelCount_p,
                                        const UINT8* pChannelToPdoTable_p,
                                        UINT32* pAbortCode_p);
static tOplkError checkAndConfigurePdo(UINT16 mappParamIndex_p,
                                       UINT8 mappObjectCount_p,
                                       UINT32* pAbortCode_p);
static tOplkError checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p);
static void decodeObjectMapping(UINT64 objectMapping_p,
                                UINT* pIndex_p,
                                UINT* pSubIndex_p,
                                UINT* pBitOffset_p,
                                UINT* pBitSize_p);
static tOplkError checkAndSetObjectMapping(UINT64 objectMapping_p,
                                           tObdAccess neededAccessType_p,
                                           tPdoMappObject* pMappObject_p,
                                           UINT32* pAbortCode_p,
                                           UINT* pOffset_p,
                                           UINT* pNextObjectOffset_p);
static tOplkError setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p,
                                      UINT8 mappObjectCount_p,
                                      UINT16 maxPdoSize_p,
                                      UINT32* pAbortCode_p,
                                      UINT16* pOffset_p,
                                      UINT16* pNextChannelOffset_p,
                                      UINT16* pCount_p);
static tOplkError configurePdoChannel(const tPdoChannelConf* pChannelConf_p);
static tOplkError getMaxPdoSize(UINT8 nodeId_p,
                                BOOL fTxPdo_p,
                                UINT16* pMaxPdoSize_p,
                                UINT32* pAbortCode_p);
static tOplkError getPdoChannelId(UINT pdoId_p, BOOL fTxPdo_p, UINT8* pChannelId_p);
static size_t calcPdoMemSize(const tPdoChannelSetup* pPdoChannels_p,
                             size_t* pRxPdoMemSize_p,
                             size_t* pTxPdoMemSize_p);
static tOplkError copyVarToPdo(void* pPayload_p,
                               const tPdoMappObject* pMappObject_p,
                               UINT16 offsetInFrame_p);
static tOplkError copyVarFromPdo(const void* pPayload_p,
                                 const tPdoMappObject* pMappObject_p,
                                 UINT16 offsetInFrame_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user module

The function initializes the PDO user module.

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_init(void)
{
    tOplkError  ret;

    OPLK_MEMSET(&pdouInstance_g, 0, sizeof(pdouInstance_g));
    pdouInstance_g.fAllocated = FALSE;
    pdouInstance_g.fRunning = FALSE;
    pdouInstance_g.pfnCbEventPdoChange = NULL;
    if (target_createMutex("/pdoMutex", &pdouInstance_g.lockMutex) != kErrorOk)
        return kErrorNoFreeInstance;

    ret = pdoucal_init();
    pdouInstance_g.fInitialized = TRUE;

    return ret;
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
    tOplkError  ret = kErrorOk;

    if (pdouInstance_g.fInitialized)
    {
        target_lockMutex(pdouInstance_g.lockMutex);
        pdouInstance_g.fRunning = FALSE;
        target_unlockMutex(pdouInstance_g.lockMutex);
        target_destroyMutex(pdouInstance_g.lockMutex);

        pdouInstance_g.pfnCbEventPdoChange = NULL;
        freePdoChannels();
        pdoucal_cleanupPdoMem();
        pdouInstance_g.fInitialized = FALSE;
        ret = pdoucal_exit();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state changes

The function implements the callback function which is called on NMT state
changes.

\param[in]      nmtStateChange_p    NMT state change event.

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tOplkError  ret = kErrorOk;

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

                target_lockMutex(pdouInstance_g.lockMutex);
                pdouInstance_g.fAllocated = FALSE;
                pdouInstance_g.fRunning = FALSE;
                target_unlockMutex(pdouInstance_g.lockMutex);

                for (mapParamIndex = PDOU_OBD_IDX_RX_MAPP_PARAM;
                     mapParamIndex < PDOU_OBD_IDX_RX_MAPP_PARAM + sizeof(pdouInstance_g.aPdoIdToChannelIdRx);
                     mapParamIndex++)
                {
                    ret = checkAndConfigurePdo(mapParamIndex, 0, &abortCode);
                    if ((ret != kErrorOk) && (ret != kErrorObdIndexNotExist))
                    {
                        DEBUG_LVL_ERROR_TRACE("%s() checkAndConfigurePdo for RPDO failed with 0x%X\n",
                                              __func__,
                                              ret);
                    }
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
                    if ((ret != kErrorOk) && (ret != kErrorObdIndexNotExist))
                    {
                        DEBUG_LVL_ERROR_TRACE("%s() checkAndConfigurePdo for TPDO failed with 0x%X\n",
                                              __func__,
                                              ret);
                    }
                }

                ret = kErrorOk;
            }
            break;

        case kNmtGsResetConfiguration:
            target_lockMutex(pdouInstance_g.lockMutex);
            pdouInstance_g.fAllocated = FALSE;
            pdouInstance_g.fRunning = FALSE;
            target_unlockMutex(pdouInstance_g.lockMutex);

            // forward PDO configuration to pdok module
            ret = configureAllPdos();
            if (ret != kErrorOk)
                goto Exit;

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

\param[in,out]  pParam_p            OBD parameter

\return The function returns a tOplkError error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tOplkError pdou_cbObdAccess(tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;
    UINT        indexType;
    UINT8       mappObjectCount;
    UINT        offset;
    UINT        nextObjectOffset;
    tObdAccess  neededAccessType;

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
            // parameters therefore we shouldn't come here!
            pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
            ret = kErrorPdoInvalidObjIndex;
            return ret;
    }

    // RPDO and TPDO mapping parameter accessed
    if (pParam_p->subIndex == 0)
    {   // object mapping count accessed
        // PDO is enabled or disabled
        mappObjectCount = *((UINT8*)pParam_p->pArg);
        ret = checkAndConfigurePdo(pParam_p->index, mappObjectCount,
                                   &pParam_p->abortCode);
        if (ret != kErrorOk)
            return ret;
    }
    else
    {
        // ObjectMapping
        tPdoMappObject  mappObject;     // temporary object for check
        UINT64          objectMapping;

        ret = checkPdoValidity(pParam_p->index, &pParam_p->abortCode);
        if (ret != kErrorOk)
        {   // PDO is valid or does not exist
            return ret;
        }

        // check existence of object and validity of object length
        objectMapping = *((UINT64*)pParam_p->pArg);
        ret = checkAndSetObjectMapping(objectMapping,
                                       neededAccessType,
                                       &mappObject,
                                       &pParam_p->abortCode,
                                       &offset,
                                       &nextObjectOffset);
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
    tOplkError              ret;
    UINT                    mappObjectCount;
    const tPdoChannel*      pPdoChannel;
    const tPdoMappObject*   pMappObject;
    UINT8                   channelId;
    void*                   pPdo;

    if (target_lockMutex(pdouInstance_g.lockMutex) != kErrorOk)
        return kErrorIllegalInstance;

    if (!pdouInstance_g.fRunning)
    {
        DEBUG_LVL_PDO_TRACE("%s() PDO channels not running!\n", __func__);
        target_unlockMutex(pdouInstance_g.lockMutex);
        return kErrorOk;
    }

    for (channelId = 0;
         channelId < pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount;
         channelId++)
    {
        pPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[channelId];

        if (pPdoChannel->nodeId == PDO_INVALID_NODE_ID)
            continue;

        ret = pdoucal_getRxPdo(&pPdo, channelId, pPdoChannel->nextChannelOffset - pPdoChannel->offset);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s pdoucal_getRxPdo failed with 0x%X\n",
                                  __func__,
                                  ret);
        }

        DEBUG_LVL_PDO_TRACE("%s() Channel:%d Node:%d pPdo:%p\n",
                            __func__,
                            channelId,
                            pPdoChannel->nodeId,
                            pPdo);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paRxObject + (channelId * D_PDO_RPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            ret = copyVarFromPdo(pPdo, pMappObject, pPdoChannel->offset);
            if (ret != kErrorOk)
            {   // other fatal error occurred
                target_unlockMutex(pdouInstance_g.lockMutex);
                return ret;
            }
        }
    }

    target_unlockMutex(pdouInstance_g.lockMutex);

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
    tOplkError              ret = kErrorOk;
    UINT                    mappObjectCount;
    const tPdoChannel*      pPdoChannel;
    const tPdoMappObject*   pMappObject;
    UINT8                   channelId;
    void*                   pPdo;

    //TRACE_FUNC_ENTRY;
    if (target_lockMutex(pdouInstance_g.lockMutex) != kErrorOk)
        return kErrorIllegalInstance;

    if (!pdouInstance_g.fRunning)
    {
        target_unlockMutex(pdouInstance_g.lockMutex);
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

        DEBUG_LVL_PDO_TRACE("%s() channelId:%d pPdo: %p\n",
                            __func__,
                            channelId,
                            pPdo);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paTxObject + (channelId * D_PDO_TPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            ret = copyVarToPdo(pPdo, pMappObject, pPdoChannel->offset);
            if (ret != kErrorOk)
            {   // other fatal error occurred
                target_unlockMutex(pdouInstance_g.lockMutex);
                return ret;
            }
        }

        // send PDO data to kernel layer
        ret = pdoucal_setTxPdo(channelId,
                               pPdo,
                               pPdoChannel->nextChannelOffset - pPdoChannel->offset);
    }

    target_unlockMutex(pdouInstance_g.lockMutex);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register PDO change callback function

The function is used to register a function for receiving PDO change events.

\param[in]      pfnCbEventPdoChange_p   Pointer to callback function.

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

\param[in]      fActivated_p        Determines if PDO is activated or disabled.
\param[in]      nodeId_p            Node ID this PDO mapping belongs to.
\param[in]      mappParamIndex_p    Object index of mapping parameter object.
\param[in]      mappObjectCount_p   Number of mapped objects.
\param[in]      fTx_p               TRUE for TXPDO and FALSE for RXPDO.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callPdoChangeCb(BOOL fActivated_p,
                                  UINT nodeId_p,
                                  UINT mappParamIndex_p,
                                  UINT8 mappObjectCount_p,
                                  BOOL fTx_p)
{
    tOplkError          ret;
    tPdoEventPdoChange  eventPdoChange;
    tObdSize            obdSize;

    if (fActivated_p == FALSE)
    {
        obdSize = (tObdSize)sizeof(mappObjectCount_p);
        // read PDO mapping version
        ret = obdu_readEntry(mappParamIndex_p, 0x00, &mappObjectCount_p, &obdSize);
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

\param[in,out]  abChannelIdToPdoIdRx_p  Pointer to array to store TX channel to PDO
                                        mapping
\param[out]     pCountChannelIdRx_p     Pointer to store number of RX channels

\return The function returns a tOplkError error code.

\internal
**/
//------------------------------------------------------------------------------
static tOplkError setupRxPdoChannelTables(
                      UINT8 abChannelIdToPdoIdRx_p[D_PDO_RPDOChannels_U16],
                      UINT* pCountChannelIdRx_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT8       nodeId;
    UINT        pdoId;
    UINT        commParamIndex;
    UINT        channelCount = 0;

    OPLK_MEMSET(pdouInstance_g.aPdoIdToChannelIdRx,
                0,
                sizeof(pdouInstance_g.aPdoIdToChannelIdRx));

    // Loops through RX Mapping objects (0x1400)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_RX_COMM_PARAM;
         pdoId < PDOU_MAX_PDO_OBJECTS;
         pdoId++, commParamIndex++)
    {
        // read node ID from OD (ID:0x14XX Sub:1)
        obdSize = (tObdSize)sizeof(nodeId);
        ret = obdu_readEntry(commParamIndex, 0x01, &nodeId, &obdSize);
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

                pdouInstance_g.aPdoIdToChannelIdRx[pdoId] = (UINT8)channelCount - 1;
                abChannelIdToPdoIdRx_p[channelCount - 1] = (UINT8)pdoId;
                break;

            default:
                return ret;
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

\param[in,out]  abChannelIdToPdoIdTx_p  Pointer to array to store RX channel to PDO
                                        mapping
\param[out]     pCountChannelIdTx_p     Pointer to store number of TX channels

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError setupTxPdoChannelTables(
                      UINT8 abChannelIdToPdoIdTx_p[D_PDO_TPDOChannels_U16],
                      UINT* pCountChannelIdTx_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT8       nodeId;
    UINT        pdoId;
    UINT        commParamIndex;
    UINT        channelCount = 0;

    OPLK_MEMSET(pdouInstance_g.aPdoIdToChannelIdTx,
                0,
                sizeof(pdouInstance_g.aPdoIdToChannelIdTx));

    abChannelIdToPdoIdTx_p[0] = 0;

    // Loops through TX Mapping objects (0x1800)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_TX_COMM_PARAM;
         pdoId < PDOU_MAX_PDO_OBJECTS;
         pdoId++, commParamIndex++)
    {
        obdSize = (tObdSize)sizeof(nodeId);
        // read node ID from OD (ID:0x18XX Sub:1)
        ret = obdu_readEntry(commParamIndex, 0x01, &nodeId, &obdSize);
        switch (ret)
        {
            case kErrorObdIndexNotExist:
            case kErrorObdSubindexNotExist:
            case kErrorObdIllegalPart:
                // PDO does not exist
                break;

            case kErrorOk:
                channelCount++;
                if (channelCount > D_PDO_TPDOChannels_U16)
                    return kErrorPdoTooManyTxPdos;

                pdouInstance_g.aPdoIdToChannelIdTx[pdoId] = (UINT8)channelCount - 1;
                abChannelIdToPdoIdTx_p[channelCount - 1] = (UINT8)pdoId;
                break;

            default:
                return ret;
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

\param[in]      pAllocationParam_p  Pointer to allocation parameters.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError allocatePdoChannels(const tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError  ret = kErrorOk;
    UINT        index;

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
    DEBUG_LVL_PDO_TRACE("%s() = %s\n", __func__, debugstr_getRetValStr(ret));
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
    tOplkError  ret = kErrorOk;

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
\brief    configure all PDOs in pdok module

The function configures the whole PDO mapping information in the pdok module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError configureAllPdos(void)
{
    tOplkError          ret = kErrorOk;
    UINT8               aChannelIdToPdoIdRx[D_PDO_RPDOChannels_U16];
    UINT8               aChannelIdToPdoIdTx[D_PDO_TPDOChannels_U16];
    tPdoAllocationParam allocParam;
    UINT32              abortCode = 0;
    size_t              txPdoMemSize;
    size_t              rxPdoMemSize;

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
                                allocParam.rxPdoChannelCount,
                                aChannelIdToPdoIdRx,
                                &abortCode);
    if (ret != kErrorOk)
        goto Exit;

    ret = checkAndConfigurePdos(PDOU_OBD_IDX_TX_MAPP_PARAM,
                                allocParam.txPdoChannelCount,
                                aChannelIdToPdoIdTx,
                                &abortCode);
    if (ret != kErrorOk)
        goto Exit;

    calcPdoMemSize(&pdouInstance_g.pdoChannels, &rxPdoMemSize, &txPdoMemSize);
    pdoucal_postSetupPdoBuffers(rxPdoMemSize, txPdoMemSize);

    // TODO how to be sure that kernel is ready before starting??
    target_msleep(CONFIG_PDO_SETUP_WAIT_TIME);

    ret = pdoucal_initPdoMem(&pdouInstance_g.pdoChannels,
                             rxPdoMemSize,
                             txPdoMemSize);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  checks and configures all PDOs for a single direction

The functions checks and configures all PDOs for a single direction.

\param[in]      mappParamIndex_p        ID of the mapping parameter object.
\param[in]      channelCount_p          Number of PDO channels.
\param[in]      pChannelToPdoTable_p    Pointer to channel-PDO table.
\param[out]     pAbortCode_p            Pointer to store the abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndConfigurePdos(UINT16 mappParamIndex_p,
                                        UINT channelCount_p,
                                        const UINT8* pChannelToPdoTable_p,
                                        UINT32* pAbortCode_p)
{
    tOplkError  ret = kErrorOk;
    UINT        index;
    tObdSize    obdSize;
    UINT8       mappObjectCount;
    UINT        mappParamIndex;

    for (index = 0; index < channelCount_p; index++)
    {
        // Calculate the MappParam object ID for the channel
        mappParamIndex = mappParamIndex_p + *(pChannelToPdoTable_p + index);

        obdSize = sizeof(mappObjectCount);
        // read mapping object count from OD
        ret = obdu_readEntry(mappParamIndex, 0x00, &mappObjectCount, &obdSize);
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

\param[in]      mappParamIndex_p    Object ID of PDO-MappParam object.
\param[in]      mappObjectCount_p   Number of mapped objects.
\param[out]     pAbortCode_p        Pointer to store abort code

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndConfigurePdo(UINT16 mappParamIndex_p,
                                       UINT8 mappObjectCount_p,
                                       UINT32* pAbortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT16          pdoId;
    UINT16          commParamIndex;
    tObdSize        obdSize;
    UINT8           nodeId;
    UINT16          maxPdoSize;
    tPdoChannelConf pdoChannelConf;
    BOOL            fTxPdo;
    tPdoMappObject* pMappObject;
    UINT16          offset;
    UINT16          nextChannelOffset;
    UINT16          count;

    DEBUG_LVL_PDO_TRACE("%s() mappParamIndex:%04x mappObjectCount:%d\n",
                        __func__,
                        mappParamIndex_p,
                        mappObjectCount_p);

    pdoId = mappParamIndex_p & PDOU_PDO_ID_MASK;
    commParamIndex = ~PDOU_OBD_IDX_MAPP_PARAM & mappParamIndex_p;
    fTxPdo = (mappParamIndex_p >= PDOU_OBD_IDX_TX_MAPP_PARAM) ? TRUE : FALSE;

    if ((!fTxPdo && (mappObjectCount_p > D_PDO_RPDOChannelObjects_U8)) ||
        (fTxPdo && (mappObjectCount_p > D_PDO_TPDOChannelObjects_U8)))
    {
        DEBUG_LVL_ERROR_TRACE("%s() %d exceeds object!\n",
                              __func__,
                              mappObjectCount_p);

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
    obdSize = (tObdSize)sizeof(nodeId);
    ret = obdu_readEntry(commParamIndex, 0x01, &nodeId, &obdSize);
    if (ret != kErrorOk)
    {   // fatal error occurred
        goto Exit;
    }
    pdoChannelConf.pdoChannel.nodeId = nodeId;

    if (mappObjectCount_p == 0)
    {   // PDO shall be disabled (see 6.4.9.2)
        pdoChannelConf.pdoChannel.nodeId = PDO_INVALID_NODE_ID;
        pdoChannelConf.pdoChannel.mappObjectCount = 0;
        pdoChannelConf.pdoChannel.offset = 0;
        pdoChannelConf.pdoChannel.nextChannelOffset = 0;
        pdouInstance_g.fRunning = FALSE;
        ret = configurePdoChannel(&pdoChannelConf);

        if ((pdouInstance_g.fAllocated) && (pdouInstance_g.pfnCbEventPdoChange != NULL))
        {
            ret = callPdoChangeCb(FALSE,
                                  nodeId,
                                  mappParamIndex_p,
                                  mappObjectCount_p,
                                  pdoChannelConf.fTx);
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

    obdSize = (tObdSize)sizeof(pdoChannelConf.pdoChannel.mappingVersion);
    // read PDO mapping version
    ret = obdu_readEntry(commParamIndex,
                         0x02,
                         &pdoChannelConf.pdoChannel.mappingVersion,
                         &obdSize);
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

    ret = setupMappingObjects(pMappObject,
                              mappParamIndex_p,
                              mappObjectCount_p,
                              maxPdoSize,
                              pAbortCode_p,
                              &offset,
                              &nextChannelOffset,
                              &count);
    if (ret != kErrorOk)
        goto Exit;

    pdoChannelConf.pdoChannel.offset = offset;
    pdoChannelConf.pdoChannel.nextChannelOffset = nextChannelOffset;
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
        ret = callPdoChangeCb(TRUE,
                              nodeId,
                              mappParamIndex_p,
                              mappObjectCount_p,
                              pdoChannelConf.fTx);
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

\param[in]      pChannelConf_p      PDO channel configuration

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError configurePdoChannel(const tPdoChannelConf* pChannelConf_p)
{
    tOplkError      ret = kErrorOk;
    tPdoChannel*    pDestPdoChannel;

    if (pdouInstance_g.fAllocated != FALSE)
    {
        if (pChannelConf_p->fTx)
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId];
        else
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[pChannelConf_p->channelId];

        // Setup user channel configuration
        OPLK_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel, sizeof(tPdoChannel));

        DEBUG_LVL_PDO_TRACE("%s(): pdoucal_postConfigureChannel(): TX:%d channel:%d offset:%d\n",
                            __func__,
                            pChannelConf_p->fTx,
                            pChannelConf_p->channelId,
                            pChannelConf_p->pdoChannel.offset);
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

\param[in]      nodeId_p            Node ID of mapped object.
\param[in]      fTxPdo_p            True for TXPDO, false for RXPDO.
\param[out]     pMaxPdoSize_p       Pointer to store maximum PDO size.
\param[out]     pAbortCode_p        Pointer to store abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getMaxPdoSize(UINT8 nodeId_p,
                                BOOL fTxPdo_p,
                                UINT16* pMaxPdoSize_p,
                                UINT32* pAbortCode_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT16      maxPdoSize;
    UINT        payloadLimitIndex;
    UINT        payloadLimitSubIndex;
    UINT8       subIndexCount;

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
            ret = obdu_readEntry(payloadLimitIndex, 0, &subIndexCount, &obdSize);
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
            ret = obdu_readEntry(payloadLimitIndex, 0, &subIndexCount, &obdSize);
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
    ret = obdu_readEntry(payloadLimitIndex,
                         payloadLimitSubIndex,
                         &maxPdoSize,
                         &obdSize);
    if (ret != kErrorOk)
    {   // other fatal error occurred
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;
    }
    else
        *pMaxPdoSize_p = maxPdoSize;

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Convert RPDO to channel ID

The function converts RPDO-ID (i.e. lower part of object index) to channel IDs.

\param[in]      pdoId_p             ID of PDO (node ID).
\param[in]      fTxPdo_p            TRUE for TXPDO or FALSE for RXPDO.
\param[out]     pChannelId_p        Pointer to store channel ID.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError getPdoChannelId(UINT pdoId_p,
                                  BOOL fTxPdo_p,
                                  UINT8* pChannelId_p)
{
    tOplkError  ret = kErrorOk;

    if (fTxPdo_p)
    {
        // TPDO mapping parameter accessed
        *pChannelId_p = pdouInstance_g.aPdoIdToChannelIdTx[pdoId_p];
    }
    else
    {
        // RPDO mapping parameter accessed
        *pChannelId_p = pdouInstance_g.aPdoIdToChannelIdRx[pdoId_p];
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  check if PDO is valid

The function checks if the PDO is valid. The PDO is valid, if the PDOs not yet
configured or if the mapping of this POD is disabled.

\param[in]      mappParamIndex_p    Object index of mapping parameter.
\param[out]     pAbortCode_p        Pointer to store SDO abort code.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT8       mappObjectCount;

    if (pdouInstance_g.fRunning)
    {
        // outside from NMT reset states the PDO should have been disabled before changing it
        obdSize = sizeof(mappObjectCount);
        // read number of mapped objects from OD; this indicates if the PDO is valid
        ret = obdu_readEntry(mappParamIndex_p, 0x00, &mappObjectCount, &obdSize);
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

\param[in]      objectMapping_p     Object mapping entry.
\param[in]      neededAccessType_p  The needed access type.
\param[in,out]  pMappObject_p       Pointer to mapping object structure
                                    which will be filled out.
\param[out]     pAbortCode_p        Pointer to SDO abort code;
                                    0 if mapping is possible
\param[out]     pOffset_p           Pointer to store calculated byte offset of
                                    this object.
\param[out]     pNextObjectOffset_p Pointer to store calculated offset of the
                                    following object (offset + size) in [byte].
                                    It is 0 if mapping failed

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndSetObjectMapping(UINT64 objectMapping_p,
                                           tObdAccess neededAccessType_p,
                                           tPdoMappObject* pMappObject_p,
                                           UINT32* pAbortCode_p,
                                           UINT* pOffset_p,
                                           UINT* pNextObjectOffset_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT        index;
    UINT        subIndex;
    UINT        bitOffset;
    UINT        bitSize;
    UINT        byteSize;
    tObdAccess  accessType;
    BOOL        fNumerical;
    tObdType    obdType;
    void*       pVar;

    if (objectMapping_p == 0)
    {   // discard zero value
        *pNextObjectOffset_p = 0;
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

    ret = obdu_getType(index, subIndex, &obdType);
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
    ret = obdu_getAccessType(index, subIndex, &accessType);
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
    {   // object is not writable (RPDO) or readable (TPDO), respectively
        *pAbortCode_p = SDO_AC_OBJECT_NOT_MAPPABLE;
        ret = kErrorPdoVarNotMappable;
        goto Exit;
    }

    if (obdType == kObdTypeBool)
    {   // bit size of BOOLEAN object was checked above
        byteSize = 1;
    }
    else
        byteSize = (bitSize >> 3);

    obdSize = obdu_getDataSize(index, subIndex);
    if (obdSize < byteSize)
    {   // object does not exist or has smaller size
        *pAbortCode_p = SDO_AC_GENERAL_ERROR;

        DEBUG_LVL_ERROR_TRACE("%s obdu_getDataSize for 0x%X/%X returned size %d (should be %d)\n",
                              __func__,
                              index,
                              subIndex,
                              obdSize,
                              byteSize);

        //TODO: Really don't want to exit here with kErrorPdoSizeMismatch?
    }

    ret = obdu_isNumerical(index, subIndex, &fNumerical);
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

    pVar = obdu_getObjectDataPtr(index, subIndex);
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

    // Calculate start and end offset (PDO size)
    *pOffset_p = (bitOffset >> 3);
    *pNextObjectOffset_p = (bitOffset >> 3) + byteSize;

Exit:
    DEBUG_LVL_PDO_TRACE("%s() = %s\n", __func__, debugstr_getRetValStr(ret));

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  setup mapping objects in PDO channel configuration

The function sets up the mapping objects of a PDO channel.

\param[in,out]  pMappObject_p           Pointer to PDO mapping object.
\param[in]      mappParamIndex_p        ID of mapping parameter object.
\param[in]      mappObjectCount_p       Number of mapping objects.
\param[in]      maxPdoSize_p            Maximum PDO size.
\param[out]     pAbortCode_p            Pointer to store the abort code.
\param[out]     pOffset_p               Pointer to store the calculated offset
                                        of the first object in the PDO channel.
\param[out]     pNextChannelOffset_p    Pointer to store the offset of the next PDO
                                        channel.
\param[out]     pCount_p                Pointer to store number of mapped objects.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p,
                                      UINT8 mappObjectCount_p,
                                      UINT16 maxPdoSize_p,
                                      UINT32* pAbortCode_p,
                                      UINT16* pOffset_p,
                                      UINT16* pNextChannelOffset_p,
                                      UINT16* pCount_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT64      objectMapping;
    UINT        count = 0;
    UINT8       mappSubindex;
    UINT        offset;
    UINT        nextObjectOffset;
    UINT16      calcNextObjectOffset = 0;
    UINT16      calcOffset = USHRT_MAX;
    tObdAccess  neededAccessType;

    for (mappSubindex = 1; mappSubindex <= mappObjectCount_p; mappSubindex++)
    {
        // read object mapping from OD
        obdSize = sizeof(objectMapping); //&pdouInstance_g.pRxPdoChannel[0] QWORD
        ret = obdu_readEntry(mappParamIndex_p,
                             mappSubindex,
                             &objectMapping,
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

        ret = checkAndSetObjectMapping(objectMapping,
                                       neededAccessType,
                                       pMappObject_p,
                                       pAbortCode_p,
                                       &offset,
                                       &nextObjectOffset);
        if (ret != kErrorOk)
            goto Exit;      // illegal object mapping

        if (nextObjectOffset == 0)
            continue;       // null mapping, go to next subindex

        if (offset < calcOffset)
            calcOffset = offset;

        if (nextObjectOffset > maxPdoSize_p)
        {   // mapping exceeds object size
            *pAbortCode_p = SDO_AC_PDO_LENGTH_EXCEEDED;
            ret = kErrorPdoLengthExceeded;
            goto Exit;
        }

        if (nextObjectOffset > calcNextObjectOffset)
            calcNextObjectOffset = nextObjectOffset;

        pMappObject_p++;
        count ++;
    }

    *pOffset_p = calcOffset;
    *pNextChannelOffset_p = calcNextObjectOffset;
    *pCount_p = count;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  decode object mapping entry

The function decodes the given object mapping entry into index, subindex,
bit offset and bit size.

\param[in]      objectMapping_p     Object mapping entry
\param[out]     pIndex_p            Pointer to store object index.
\param[out]     pSubIndex_p         Pointer to store subindex.
\param[out]     pBitOffset_p        Pointer to store bit offset.
\param[out]     pBitSize_p          Pointer to store bit size.
*/
//------------------------------------------------------------------------------
static void decodeObjectMapping(UINT64 objectMapping_p,
                                UINT* pIndex_p,
                                UINT* pSubIndex_p,
                                UINT* pBitOffset_p,
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

\param[in,out]  pPayload_p          Pointer to PDO payload in destination frame.
\param[in]      pMappObject_p       Pointer to mapping object.
\param[in]      offsetInFrame_p     Offset of the PDO data in the frame.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError copyVarToPdo(void* pPayload_p,
                               const tPdoMappObject* pMappObject_p,
                               UINT16 offsetInFrame_p)
{
    tOplkError  ret = kErrorOk;
    UINT        byteOffset;
    void*       pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p = (UINT8*)pPayload_p + byteOffset - offsetInFrame_p;
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
            ami_setUint8Le(pPayload_p, *((const UINT8*)pVar));
            break;

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
            ami_setUint16Le(pPayload_p, *((const UINT16*)pVar));
            break;

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
            ami_setUint24Le(pPayload_p, *((const UINT32*)pVar));
            break;

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            ami_setUint32Le(pPayload_p, *((const UINT32*)pVar));
            break;

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
            ami_setUint40Le(pPayload_p, *((const UINT64*)pVar));
            break;

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
            ami_setUint48Le(pPayload_p, *((const UINT64*)pVar));
            break;

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
            ami_setUint56Le(pPayload_p, *((const UINT64*)pVar));
            break;

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            ami_setUint64Le(pPayload_p, *((const UINT64*)pVar));
            break;

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_setTimeOfDay(pPayload_p, (const tTimeOfDay*)pVar);
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy variable from PDO

This function copies a variable specified by the mapping object from the PDO
payload.

\param[in]      pPayload_p          Pointer to PDO payload in destination frame.
\param[in]      pMappObject_p       Pointer to mapping object.
\param[in]      offsetInFrame_p     Offset of the PDO data in the frame.

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError copyVarFromPdo(const void* pPayload_p,
                                 const tPdoMappObject* pMappObject_p,
                                 UINT16 offsetInFrame_p)
{
    tOplkError  ret = kErrorOk;
    UINT        byteOffset;
    void*       pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p = (const UINT8*)pPayload_p + byteOffset - offsetInFrame_p;
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
            *((UINT8*)pVar) = ami_getUint8Le(pPayload_p);
            break;

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
            *((UINT16*)pVar) = ami_getUint16Le(pPayload_p);
            break;

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
            *((UINT32*)pVar) = ami_getUint24Le(pPayload_p);
            break;

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            *((UINT32*)pVar) = ami_getUint32Le(pPayload_p);
            break;

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
            *((UINT64*)pVar) = ami_getUint40Le(pPayload_p);
            break;

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
            *((UINT64*)pVar) = ami_getUint48Le(pPayload_p);
            break;

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
            *((UINT64*)pVar) = ami_getUint56Le(pPayload_p);
            break;

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            *((UINT64*)pVar) = ami_getUint64Le(pPayload_p);
            break;

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_getTimeOfDay(pPayload_p, (tTimeOfDay*)pVar);
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate PDO memory size

The function calculates the size needed for the PDO memory.

\param[in]      pPdoChannels_p      Pointer to PDO channel setup.
\param[out]     pRxPdoMemSize_p     Pointer to store size of RX PDO buffers.
\param[out]     pTxPdoMemSize_p     Pointer to store size of TX PDO buffers.

\return The function returns the size of the used PDO memory
*/
//------------------------------------------------------------------------------
static size_t calcPdoMemSize(const tPdoChannelSetup* pPdoChannels_p,
                             size_t* pRxPdoMemSize_p,
                             size_t* pTxPdoMemSize_p)
{
    UINT8               channelId;
    size_t              rxSize;
    size_t              txSize;
    const tPdoChannel*  pPdoChannel;

    rxSize = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        rxSize += (pPdoChannel->nextChannelOffset - pPdoChannel->offset);
    }
    if (pRxPdoMemSize_p != NULL)
        *pRxPdoMemSize_p = rxSize;

    txSize = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        txSize += (pPdoChannel->nextChannelOffset - pPdoChannel->offset);
    }
    if (pTxPdoMemSize_p != NULL)
        *pTxPdoMemSize_p = txSize;

    return rxSize + txSize;
}

/// \}
