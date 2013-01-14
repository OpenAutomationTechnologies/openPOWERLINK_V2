/**
********************************************************************************
\file   pdou.c

\brief  Implementation of user PDO module

This file contains the implementation of the user PDO module.

\ingroup module_pdou
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <Epl.h>
#include <EplInc.h>
#include <user/pdoucal.h>
#include <user/EplObdu.h>
#include <user/pdou.h>
#include <EplSdoAc.h>

#include <pdo.h>

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) == 0) && (EPL_OBD_USE_KERNEL == FALSE)
#error "EPL PDOu module needs EPL module OBDU or OBDK!"
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

/**
\brief User PDO module instance

The following structure defines the instance variable of the user PDO module.
*/
typedef struct
{
    BYTE                    aPdoIdToChannelIdRx[(PDOU_PDO_ID_MASK + 1)]; ///< RXPDO to channel ID conversion table
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // only MN supports multiple TPDOS where indexing is necessary
    BYTE                    aPdoIdToChannelIdTx[(PDOU_PDO_ID_MASK + 1)]; ///< TXPDO to channel ID conversion table
#endif
    tPdoChannelSetup        pdoChannels;                ///< PDO channel setup
    tPdoMappObject*         paRxObject;                 ///< Pointer to RX channel objects
    tPdoMappObject*         paTxObject;                 ///< Pointer to TX channel objects
    BOOL                    fAllocated;                 ///< Flag determines if PDOs are allocated
    BOOL                    fRunning;                   ///< Flag determines if PDO engine is running
    BYTE*                   pPdoMem;                    ///< pointer to PDO memory
} tPdouInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdouInstance  pdouInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel   setupRxPdoChannelTables(BYTE abChannelIdToPdoIdRx_p[EPL_D_PDO_RPDOChannels_U16],
                                          UINT* pCountChannelIdRx_p);
static tEplKernel   setupTxPdoChannelTables(BYTE abChannelIdToPdoIdTx_p[EPL_D_PDO_TPDOChannels_U16],
                                          UINT* pCountChannelIdTx_p);
static tEplKernel   allocatePdoChannels(tPdoAllocationParam* pAllocationParam_p);
static tEplKernel   configureAllPdos(void);
static tEplKernel   checkAndConfigurePdos(UINT16 mappParamIndex_p, UINT channelCount_p,
                                          BYTE *pChannelToPdoTable_p, UINT32 *pAbortCode_p);
static tEplKernel   checkAndConfigurePdo(UINT16 mappParamIndex_p, BYTE  mappObjectCount_p,
                                         UINT32* pAbortCode_p);
static tEplKernel   checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p);
static void         decodeObjectMapping(QWORD objectMapping_p, UINT* pIndex_p,
                                UINT* pSubIndex_p, UINT* pBitOffset_p,
                                UINT* pBitSize_p);
static tEplKernel   checkAndSetObjectMapping(QWORD objectMapping_p, tEplObdAccess neededAccessType_p,
                                     tPdoMappObject* pMappObject_p,
                                     DWORD* pAbortCode_p, UINT* pPdoSize_p);
static tEplKernel   setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p, BYTE mappObjectCount_p,
                                      UINT16  maxPdoSize_p, UINT32* pAbortCode_p,
                                      UINT* pCalcPdoSize_p, UINT* pCount_p);
static tEplKernel   configurePdoChannel(tPdoChannelConf* pChannelConf_p);
static tEplKernel   getMaxPdoSize(BYTE nodeId_p, BOOL fTxPdo_p,
                         UINT16 *pMaxPdoSize_p, UINT32* pAbortCode_p);
static tEplKernel   getPdoChannelId(UINT pdoId_p, BOOL fTxPdo_p, UINT *pChannelId_p);
static tEplKernel   setupPdoBuffers(void);
static tEplKernel   copyVarToPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p);
static tEplKernel   copyVarFromPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user module

The function initializes the PDO user module.

\return The function returns a tEplKernel error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tEplKernel pdou_init(void)
{
    EPL_MEMSET(&pdouInstance_g, 0, sizeof(pdouInstance_g));
    pdouInstance_g.fAllocated = FALSE;
    pdouInstance_g.fRunning = FALSE;

    return pdoucal_init();
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO user module

The function cleans up the PDO user module.

\return The function returns a tEplKernel error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tEplKernel pdou_exit(void)
{
    pdoucal_cleanupPdoMem(pdouInstance_g.pPdoMem);
    return pdoucal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  callback function for NMT state changes

\brief  nmtStateChange_p         NMT state change event

\return The function returns a tEplKernel error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tEplKernel PUBLIC pdou_cbNmtStateChange(tEplEventNmtStateChange nmtStateChange_p)
{
    tEplKernel      ret = kEplSuccessful;

    switch (nmtStateChange_p.m_NewNmtState)
    {
        case kEplNmtGsOff:
        case kEplNmtGsInitialising:
        case kEplNmtGsResetApplication:
        case kEplNmtGsResetCommunication:
            pdouInstance_g.fAllocated = FALSE;
            pdouInstance_g.fRunning = FALSE;
            break;

        case kEplNmtGsResetConfiguration:
            pdouInstance_g.fAllocated = FALSE;
            pdouInstance_g.fRunning = FALSE;

            // forward PDO configuration to Pdok module
            ret = configureAllPdos();
            if (ret != kEplSuccessful)
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
\brief  callback function for OD accesses

\brief  pParam_p                OBD parameter

\return The function returns a tEplKernel error code.

\ingroup module_pdou
**/
//------------------------------------------------------------------------------
tEplKernel PUBLIC pdou_cbObdAccess(tEplObdCbParam MEM* pParam_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT                indexType;
    BYTE                mappObjectCount;
    UINT                curPdoSize;
    tEplObdAccess       neededAccessType;

    pParam_p->m_dwAbortCode = 0;

    if (pParam_p->m_ObdEvent != kEplObdEvPreWrite)
    {   // read accesses, post write events etc. are OK
        return ret;
    }

    // fetch object index type
    indexType = pParam_p->m_uiIndex & PDOU_OBD_IDX_MASK;

    // check index type
    switch (indexType)
    {
        case PDOU_OBD_IDX_RX_COMM_PARAM:
        case PDOU_OBD_IDX_TX_COMM_PARAM:
            ret = checkPdoValidity((PDOU_OBD_IDX_MAPP_PARAM | pParam_p->m_uiIndex),
                                   &pParam_p->m_dwAbortCode);
            return ret;
            break;

        case PDOU_OBD_IDX_RX_MAPP_PARAM:
            // RPDO mapping parameter accessed
            neededAccessType = kEplObdAccWrite;
            break;

        case PDOU_OBD_IDX_TX_MAPP_PARAM:
            // TPDO mapping parameter accessed
            neededAccessType = kEplObdAccRead;
            break;

        default:
            // this callback function is only for PDO mapping and communication
            // parameters therfore we shouldn't come here!
            pParam_p->m_dwAbortCode = EPL_SDOAC_GENERAL_ERROR;
            ret = kEplPdoInvalidObjIndex;
            return ret;
            break;
    }

    // RPDO and TPDO mapping parameter accessed
    if (pParam_p->m_uiSubIndex == 0)
    {   // object mapping count accessed
        // PDO is enabled or disabled
        mappObjectCount = *((BYTE*) pParam_p->m_pArg);
        ret = checkAndConfigurePdo(pParam_p->m_uiIndex, mappObjectCount,
                                   &pParam_p->m_dwAbortCode);
        if (ret != kEplSuccessful)
            return ret;
    }
    else
    {
        // ObjectMapping
        tPdoMappObject      mappObject;     // temporary object for check
        QWORD               objectMapping;

        ret = checkPdoValidity(pParam_p->m_uiIndex, &pParam_p->m_dwAbortCode);
        if (ret != kEplSuccessful)
        {   // PDO is valid or does not exist
            return ret;
        }

        // check existence of object and validity of object length
        objectMapping = *((QWORD*) pParam_p->m_pArg);
        ret = checkAndSetObjectMapping(objectMapping, neededAccessType, &mappObject,
                                       &pParam_p->m_dwAbortCode, &curPdoSize);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy RXPDO to process image

The function copies RXPDOs into the process image

\return The function returns a tEplKernel error code.

\ingroup module_pdou
*/
//------------------------------------------------------------------------------
tEplKernel pdou_copyRxPdoToPi (void)
{
    tEplKernel          Ret;
    UINT                mappObjectCount;
    tPdoChannel*        pPdoChannel;
    tPdoMappObject*     pMappObject;
    UINT                channelId;
    BYTE*               pPdo;

    if (!pdouInstance_g.fRunning)
    {
        EPL_DBGLVL_PDO_TRACE ("%s() PDO channels not running!\n", __func__);
        return kEplSuccessful;
    }

    for (channelId = 0;
         channelId < pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount;
         channelId++)
    {
        pPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[channelId];

#if 0
        printf ("\nPDO channel %d\n", channelId);
        printf ("Node ID: %d\n", pPdoChannel->nodeId);
        printf ("pVar: %p\n", pPdoChannel->pVar);
#endif

        if (pPdoChannel->nodeId == PDO_INVALID_NODE_ID)
        {
            continue;
        }

        Ret = pdoucal_getRxPdo(&pPdo, pPdoChannel->pVar, pPdoChannel->pdoSize);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paRxObject + (channelId * EPL_D_PDO_RPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            Ret = copyVarFromPdo(pPdo, pMappObject);
            if (Ret != kEplSuccessful)
            {   // other fatal error occurred
                return Ret;
            }

        }
    }
    return kEplSuccessful;
}


//------------------------------------------------------------------------------
/**
\brief  Copy TXPDO from process image

The function copies the TXPDOs from the process image into the PDO buffers.

\return The function returns a tEplKernel error code.

\ingroup module_pdou
*/
//------------------------------------------------------------------------------
tEplKernel pdou_copyTxPdoFromPi (void)
{
    tEplKernel          ret = kEplSuccessful;
    UINT                mappObjectCount;
    tPdoChannel*        pPdoChannel;
    tPdoMappObject*     pMappObject;
    UINT                channelId;
    BYTE*               pPdo;

    //TRACE_FUNC_ENTRY;

    if (!pdouInstance_g.fRunning)
    {
        return kEplSuccessful;
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

        pPdo = pdoucal_getPdoAdrs(TRUE, channelId);

        for (mappObjectCount = pPdoChannel->mappObjectCount,
             pMappObject = pdouInstance_g.paTxObject + (channelId * EPL_D_PDO_TPDOChannelObjects_U8);
             mappObjectCount > 0;
             mappObjectCount--, pMappObject++)
        {
            ret = copyVarToPdo(pPdo, pMappObject);
            if (ret != kEplSuccessful)
            {   // other fatal error occurred
                return ret;
            }
        }

        // send PDO data to kernel layer
        ret = pdoucal_setTxPdo(pPdoChannel->pVar, pPdo, pPdoChannel->pdoSize);
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

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

\return The function returns a tEplKernel error code.

\internal
**/
//------------------------------------------------------------------------------
static tEplKernel setupRxPdoChannelTables(
                       BYTE abChannelIdToPdoIdRx_p[EPL_D_PDO_RPDOChannels_U16],
                       UINT* pCountChannelIdRx_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplObdSize             obdSize;
    BYTE                    nodeId;
    UINT                    pdoId;
    UINT                    commParamIndex;
    UINT                    channelCount;

    channelCount = 0;

    EPL_MEMSET(pdouInstance_g.aPdoIdToChannelIdRx, 0,
               sizeof (pdouInstance_g.aPdoIdToChannelIdRx));

    // Loops through RX Mapping objects (0x1400)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_RX_COMM_PARAM;
         pdoId < PDOU_MAX_PDO_OBJECTS;
         pdoId++, commParamIndex++)
    {
        // read node ID from OD (ID:0x14XX Sub:1)
        obdSize = sizeof (nodeId);
        ret = EplObduReadEntry(commParamIndex, 0x01, &nodeId, &obdSize);
        switch (ret)
        {
            case kEplObdIndexNotExist:
            case kEplObdSubindexNotExist:
            case kEplObdIllegalPart:
                // PDO does not exist
                break;

            case kEplSuccessful:
                channelCount++;
                if (channelCount > EPL_D_PDO_RPDOChannels_U16)
                    return kEplPdoTooManyPdos;

                pdouInstance_g.aPdoIdToChannelIdRx[pdoId] = (BYTE) channelCount - 1;
                abChannelIdToPdoIdRx_p[channelCount - 1] = (BYTE) pdoId;
                break;

            default:
                return ret;
                break;
        }

    }
    *pCountChannelIdRx_p = channelCount;

    return kEplSuccessful;
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

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel setupTxPdoChannelTables(
                        BYTE abChannelIdToPdoIdTx_p[EPL_D_PDO_TPDOChannels_U16],
                        UINT* pCountChannelIdTx_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplObdSize             obdSize;
    BYTE                    bNodeId;
    UINT                    pdoId;
    UINT                    commParamIndex;
    UINT                    channelCount;

    channelCount = 0;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    EPL_MEMSET(pdouInstance_g.aPdoIdToChannelIdTx, 0,
               sizeof (pdouInstance_g.aPdoIdToChannelIdTx));
#endif

    // Loops through TX Mapping objects (0x1800)
    for (pdoId = 0, commParamIndex = PDOU_OBD_IDX_TX_COMM_PARAM;
        pdoId < PDOU_MAX_PDO_OBJECTS;
        pdoId++, commParamIndex++)
    {
        obdSize = sizeof (bNodeId);
        // read node ID from OD (ID:0x18XX Sub:1)
        ret = EplObduReadEntry(commParamIndex, 0x01, &bNodeId, &obdSize);
        switch (ret)
        {
            case kEplObdIndexNotExist:
            case kEplObdSubindexNotExist:
            case kEplObdIllegalPart:
                // PDO does not exist
                break;

            case kEplSuccessful:
                channelCount ++;
                if (channelCount > EPL_D_PDO_TPDOChannels_U16)
                    return kEplPdoTooManyTxPdos;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                pdouInstance_g.aPdoIdToChannelIdTx[pdoId] = (BYTE) channelCount - 1;
#endif
                abChannelIdToPdoIdTx_p[channelCount - 1] = (BYTE) pdoId;
                break;

            default:
                return ret;
                break;
        }
    }
    *pCountChannelIdTx_p = channelCount;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels

This function allocates memory for PDOs channels

\param  pAllocationParam_p      Pointer to allocation parameters.

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel allocatePdoChannels(tPdoAllocationParam* pAllocationParam_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            index;

    if (pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount != pAllocationParam_p->rxPdoChannelCount)
    {   // allocation should be changed
        pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount = pAllocationParam_p->rxPdoChannelCount;
        if (pdouInstance_g.pdoChannels.pRxPdoChannel != NULL)
        {
            EPL_FREE(pdouInstance_g.pdoChannels.pRxPdoChannel);
            pdouInstance_g.pdoChannels.pRxPdoChannel = NULL;
        }

        if (pdouInstance_g.paRxObject != NULL)
        {
            EPL_FREE(pdouInstance_g.paRxObject);
            pdouInstance_g.paRxObject = NULL;
        }

        if (pAllocationParam_p->rxPdoChannelCount > 0)
        {
            pdouInstance_g.pdoChannels.pRxPdoChannel =
                  EPL_MALLOC(sizeof(tPdoChannel) * pAllocationParam_p->rxPdoChannelCount);

            if (pdouInstance_g.pdoChannels.pRxPdoChannel == NULL)
            {
                ret = kEplPdoInitError;
                goto Exit;
            }

            pdouInstance_g.paRxObject =
                    EPL_MALLOC(sizeof(tPdoMappObject)
                               * pAllocationParam_p->rxPdoChannelCount
                               * EPL_D_PDO_RPDOChannelObjects_U8);

            if (pdouInstance_g.paRxObject == NULL)
            {
                ret = kEplPdoInitError;
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
            EPL_FREE(pdouInstance_g.pdoChannels.pTxPdoChannel);
            pdouInstance_g.pdoChannels.pTxPdoChannel = NULL;
        }

        if (pdouInstance_g.paTxObject != NULL)
        {
            EPL_FREE(pdouInstance_g.paTxObject);
            pdouInstance_g.paTxObject = NULL;
        }

        if (pAllocationParam_p->txPdoChannelCount > 0)
        {
            pdouInstance_g.pdoChannels.pTxPdoChannel =
                    EPL_MALLOC(sizeof(tPdoChannel) * pAllocationParam_p->txPdoChannelCount);
            if (pdouInstance_g.pdoChannels.pTxPdoChannel == NULL)
            {
                ret = kEplPdoInitError;
                goto Exit;
            }

            pdouInstance_g.paTxObject =
                    EPL_MALLOC(sizeof(tPdoMappObject)
                               * pAllocationParam_p->txPdoChannelCount
                               * EPL_D_PDO_TPDOChannelObjects_U8);
            if (pdouInstance_g.paTxObject == NULL)
            {
                ret = kEplPdoInitError;
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
    //TRACE("%s() = %s\n", __func__, EplGetEplconfigureChannelKernelStr(Ret));
    return ret;
}



//------------------------------------------------------------------------------
/**
\brief	configure all PDOs in Pdok module

The function configures the whole PDO mapping information in the Pdok module.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel configureAllPdos(void)
{
    tEplKernel              ret = kEplSuccessful;
    BYTE                    aChannelIdToPdoIdRx[EPL_D_PDO_RPDOChannels_U16];
    BYTE                    aChannelIdToPdoIdTx[EPL_D_PDO_TPDOChannels_U16];
    tPdoAllocationParam     allocParam;
    DWORD                   dwAbortCode = 0;

    ret = setupRxPdoChannelTables(aChannelIdToPdoIdRx, &allocParam.rxPdoChannelCount);
    if (ret != kEplSuccessful)
        goto Exit;
    ret = setupTxPdoChannelTables(aChannelIdToPdoIdTx, &allocParam.txPdoChannelCount);
    if (ret != kEplSuccessful)
        goto Exit;

    ret = allocatePdoChannels(&allocParam);
    if (ret != kEplSuccessful)
        goto Exit;
    ret = pdoucal_postPdokChannelAlloc(&allocParam);
    if (ret != kEplSuccessful)
        goto Exit;
    pdouInstance_g.fAllocated = TRUE;

    // configure the PDOs
    ret = checkAndConfigurePdos(PDOU_OBD_IDX_RX_MAPP_PARAM,
                                allocParam.rxPdoChannelCount, aChannelIdToPdoIdRx,
                                &dwAbortCode);
    if (ret != kEplSuccessful)
        goto Exit;

    ret = checkAndConfigurePdos(PDOU_OBD_IDX_TX_MAPP_PARAM,
                                allocParam.txPdoChannelCount, aChannelIdToPdoIdTx,
                                &dwAbortCode);
    if (ret != kEplSuccessful)
        goto Exit;

    setupPdoBuffers();
    pdoucal_postSetupPdoBuffers();

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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel checkAndConfigurePdos(UINT16 mappParamIndex_p, UINT channelCount_p,
                                        BYTE *pChannelToPdoTable_p, UINT32 *pAbortCode_p)
{
    tEplKernel          ret;
    UINT                index;
    tEplObdSize         obdSize;
    BYTE                mappObjectCount;
    UINT                mappParamIndex;

    for (index = 0; index < channelCount_p; index++)
    {
        // Calculate the MappParam object ID for the channel
        mappParamIndex = mappParamIndex_p + *(pChannelToPdoTable_p + index);

        obdSize = sizeof (mappObjectCount);
        // read mapping object count from OD
        ret = EplObduReadEntry(mappParamIndex, 0x00, &mappObjectCount, &obdSize);
        if (ret != kEplSuccessful)
            return ret;

        ret = checkAndConfigurePdo(mappParamIndex, mappObjectCount, pAbortCode_p);
        if (ret != kEplSuccessful)
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel checkAndConfigurePdo(UINT16 mappParamIndex_p,
                                       BYTE  mappObjectCount_p, UINT32* pAbortCode_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT16              pdoId;
    UINT16              commParamIndex;
    tEplObdSize         obdSize;
    BYTE                nodeId;
    WORD                maxPdoSize;
    tPdoChannelConf     pdoChannelConf;
    BOOL                fTxPdo;
    tPdoMappObject*     pMappObject;
    UINT                calcPdoSize;
    UINT                count;

    EPL_DBGLVL_PDO_TRACE ("%s() mappParamIndex:%04x mappObjectCount:%d\n",
                          __func__, mappParamIndex_p, mappObjectCount_p);

    pdoId = mappParamIndex_p & PDOU_PDO_ID_MASK;
    commParamIndex = ~PDOU_OBD_IDX_MAPP_PARAM & mappParamIndex_p;
    fTxPdo = (mappParamIndex_p >= PDOU_OBD_IDX_TX_MAPP_PARAM) ? TRUE : FALSE;

    if ((!fTxPdo && (mappObjectCount_p > EPL_D_PDO_RPDOChannelObjects_U8)) ||
        (fTxPdo && (mappObjectCount_p > EPL_D_PDO_TPDOChannelObjects_U8)))
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() %d exceeds object!\n",
                                __func__, mappObjectCount_p);
        *pAbortCode_p = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
        ret = kEplObdValueTooHigh;
        goto Exit;
    }

    pdoChannelConf.fTx = fTxPdo;
    ret = getPdoChannelId(pdoId, fTxPdo, &pdoChannelConf.channelId);
    if (ret != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() error get channelID!\n", __func__);
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        ret = kEplPdoInvalidObjIndex;
        goto Exit;
    }

    if (mappObjectCount_p == 0)
    {   // PDO shall be disabled (see 6.4.9.2)
        pdoChannelConf.pdoChannel.nodeId = PDO_INVALID_NODE_ID;
        pdoChannelConf.pdoChannel.mappObjectCount = 0;
        pdoChannelConf.pdoChannel.pdoSize = 0;
        pdouInstance_g.fRunning = FALSE;
        ret = configurePdoChannel(&pdoChannelConf);
        goto Exit;
    }

    if (!pdouInstance_g.fAllocated)
        goto Exit;

    // PDO shall be enabled
    ret = checkPdoValidity(mappParamIndex_p, pAbortCode_p);
    if (ret != kEplSuccessful)
    {   // PDO is invalid or does not exist
        goto Exit;
    }

    // read node ID from OD
    obdSize = sizeof (nodeId);
    ret = EplObduReadEntry(commParamIndex, 0x01, &nodeId, &obdSize);
    if (ret != kEplSuccessful)
    {   // fatal error occurred
        goto Exit;
    }
    pdoChannelConf.pdoChannel.nodeId = nodeId;

    obdSize = sizeof (pdoChannelConf.pdoChannel.mappingVersion);
    // read PDO mapping version
    ret = EplObdReadEntry(commParamIndex, 0x02,
                          &pdoChannelConf.pdoChannel.mappingVersion, &obdSize);
    if (ret != kEplSuccessful)
    {   // other fatal error occurred
        goto Exit;
    }

    ret = getMaxPdoSize(nodeId, fTxPdo, &maxPdoSize, pAbortCode_p);
    if (ret != kEplSuccessful)
    {   // PDO is valid or does not exist
        goto Exit;
    }

    if (fTxPdo)
        pMappObject = &pdouInstance_g.paTxObject[pdoChannelConf.channelId *
                                                 EPL_D_PDO_TPDOChannelObjects_U8];
    else
        pMappObject = &pdouInstance_g.paRxObject[pdoChannelConf.channelId *
                                                 EPL_D_PDO_RPDOChannelObjects_U8];

    ret = setupMappingObjects(pMappObject, mappParamIndex_p, mappObjectCount_p,
                              maxPdoSize, pAbortCode_p, &calcPdoSize, &count);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
    pdoChannelConf.pdoChannel.pdoSize = calcPdoSize;
    pdoChannelConf.pdoChannel.mappObjectCount = count;

    // do not make the call before Alloc has been called
    ret = configurePdoChannel(&pdoChannelConf);
    if (ret != kEplSuccessful)
    {   // fatal error occurred
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure the specified PDO channel

The function configures the specified PDO channel.

\param  pChannelConf_p              PDO channel configuration

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel configurePdoChannel(tPdoChannelConf* pChannelConf_p)
{
    tEplKernel          ret = kEplSuccessful;
    tPdoChannel*        pDestPdoChannel;

    if (pdouInstance_g.fAllocated != FALSE)
    {
        if (pChannelConf_p->fTx)
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pTxPdoChannel[pChannelConf_p->channelId];
        else
            pDestPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[pChannelConf_p->channelId];

        // Setup user channel configuration
        EPL_MEMCPY(pDestPdoChannel, &pChannelConf_p->pdoChannel, sizeof (tPdoChannel));

        // TRACE ("postConfigureChannel: TX:%d channel:%d size:%d\n",
        //        pChannelConf_p->fTx, pChannelConf_p->channelId, pChannelConf_p->pdoChannel.pdoSize);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel getMaxPdoSize(BYTE nodeId_p, BOOL fTxPdo_p,
                         UINT16 *pMaxPdoSize_p, UINT32* pAbortCode_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplObdSize         obdSize;
    WORD                maxPdoSize;
    UINT                payloadLimitIndex;
    UINT                payloadLimitSubIndex;

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
        }
    }

    // fetch maximum PDO size from OD
    obdSize = sizeof (maxPdoSize);
    ret = EplObduReadEntry(payloadLimitIndex, payloadLimitSubIndex,
                           &maxPdoSize, &obdSize);
    if (ret != kEplSuccessful)
    {   // other fatal error occurred
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
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

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel getPdoChannelId(UINT pdoId_p, BOOL fTxPdo_p, UINT *pChannelId_p)
{
    tEplKernel          Ret = kEplSuccessful;

    if (fTxPdo_p)
    {
        // TPDO mapping parameter accessed
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel checkPdoValidity(UINT mappParamIndex_p, UINT32* pAbortCode_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplObdSize         obdSize;
    BYTE                mappObjectCount;

    if (pdouInstance_g.fRunning)
    {
        // outside from NMT reset states the PDO should have been disabled before changing it
        obdSize = sizeof (mappObjectCount);
        // read number of mapped objects from OD; this indicates if the PDO is valid
        ret = EplObduReadEntry(mappParamIndex_p, 0x00, &mappObjectCount, &obdSize);
        if (ret != kEplSuccessful)
        {   // other fatal error occurred
            *pAbortCode_p = EPL_SDOAC_GEN_INTERNAL_INCOMPATIBILITY;
        }
        else
        {
            // entry read successfully
            if (mappObjectCount != 0)
            {   // PDO in OD is still valid
                *pAbortCode_p = EPL_SDOAC_GEN_PARAM_INCOMPATIBILITY;
                ret = kEplPdoConfWhileEnabled;
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel checkAndSetObjectMapping(QWORD objectMapping_p,
                                   tEplObdAccess neededAccessType_p,
                                   tPdoMappObject* pMappObject_p,
                                   DWORD* pAbortCode_p, UINT* pPdoSize_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplObdSize         obdSize;
    UINT                index;
    UINT                subIndex;
    UINT                bitOffset;
    UINT                bitSize;
    UINT                byteSize;
    tEplObdAccess       accessType;
    BOOL                fNumerical;
    tEplObdType         obdType;
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
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        ret = kEplPdoGranularityMismatch;
        goto Exit;
    }

    ret = EplObduGetType(index, subIndex, &obdType);
    if (ret != kEplSuccessful)
    {   // entry doesn't exist
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        ret = kEplPdoVarNotFound;
        goto Exit;
    }

    if (((bitSize & 0x7) != 0x0) &&
        ((bitSize != 1) || (obdType != kEplObdTypBool)))
    {   // bit mapping is not supported, except for BOOLEAN objects on byte boundaries
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        ret = kEplPdoGranularityMismatch;
        goto Exit;
    }

    // check access type
    ret = EplObduGetAccessType(index, subIndex, &accessType);
    if (ret != kEplSuccessful)
    {   // entry doesn't exist
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        ret = kEplPdoVarNotFound;
        goto Exit;
    }

    if ((accessType & kEplObdAccPdo) == 0)
    {   // object is not mappable
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_MAPPABLE;
        ret = kEplPdoVarNotMappable;
        goto Exit;
    }

    if ((accessType & neededAccessType_p) == 0)
    {   // object is not writeable (RPDO) or readable (TPDO) respectively
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_MAPPABLE;
        ret = kEplPdoVarNotMappable;
        goto Exit;
    }

    if (obdType == kEplObdTypBool)
    {   // bit size of BOOLEAN object was checked above
        byteSize = 1;
    }
    else
    {
        byteSize = (bitSize >> 3);
    }

    obdSize = EplObduGetDataSize(index, subIndex);
    if (obdSize < byteSize)
    {   // object does not exist or has smaller size
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        ret = kEplPdoSizeMismatch;
        // todo really don't want to exit here?
    }

    ret = EplObduIsNumerical(index, subIndex, &fNumerical);
    if (ret != kEplSuccessful)
    {   // entry doesn't exist
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        goto Exit;
    }

    if ((fNumerical != FALSE) && (byteSize != obdSize))
    {
        // object is numerical,
        // therefore size has to fit, but it does not.
        *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        ret = kEplPdoVarNotFound;
        goto Exit;
    }

    pVar = EplObduGetObjectDataPtr(index, subIndex);
    if (pVar == NULL)
    {   // entry doesn't exist
        *pAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        ret = kEplPdoVarNotFound;
        goto Exit;
    }

    // setup mapping object
    PDO_MAPPOBJECT_SET_BITOFFSET(pMappObject_p, (WORD) bitOffset);
    PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pMappObject_p, (WORD) byteSize, obdType);
    PDO_MAPPOBJECT_SET_VAR(pMappObject_p, pVar);

    // Calculate needed PDO size
    *pPdoSize_p = (bitOffset >> 3) + byteSize;

Exit:
    //TRACE("%s() = %s\n", __func__, EplGetEplKernelStr(ret));
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel setupMappingObjects(tPdoMappObject* pMappObject_p,
                                      UINT mappParamIndex_p, BYTE mappObjectCount_p,
                                      UINT16  maxPdoSize_p, UINT32* pAbortCode_p,
                                      UINT* pCalcPdoSize_p, UINT* pCount_p)
{
    tEplKernel          ret;
    tEplObdSize         obdSize;
    QWORD               objectMapping;
    UINT                count;
    BYTE                mappSubindex;
    BYTE                mappObjectCount;
    UINT                curPdoSize;
    WORD                calcPdoSize = 0;
    tEplObdAccess       neededAccessType;

    mappObjectCount = mappObjectCount_p;
    count = 0;

    for (mappSubindex = 1; mappSubindex <= mappObjectCount; mappSubindex++)
    {
        // read object mapping from OD
        obdSize = sizeof (objectMapping); //&pdouInstance_g.pRxPdoChannel[0] QWORD
        ret = EplObduReadEntry(mappParamIndex_p, mappSubindex, &objectMapping,
                               &obdSize);
        if (ret != kEplSuccessful)
        {   // other fatal error occurred
            *pAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
            goto Exit;
        }

        if (mappParamIndex_p >= PDOU_OBD_IDX_TX_MAPP_PARAM)
            neededAccessType = kEplObdAccRead;
        else
            neededAccessType = kEplObdAccWrite;

        ret = checkAndSetObjectMapping(objectMapping, neededAccessType, pMappObject_p,
                               pAbortCode_p, &curPdoSize);
        if (ret != kEplSuccessful)
        {   // illegal object mapping
            goto Exit;
        }

        if (curPdoSize == 0)
        {   // null mapping, go to next subindex
            continue;
        }

        if (curPdoSize > maxPdoSize_p)
        {   // mapping exceeds object size
            *pAbortCode_p = EPL_SDOAC_PDO_LENGTH_EXCEEDED;
            ret = kEplPdoLengthExceeded;
            goto Exit;
        }

        if (curPdoSize > calcPdoSize)
        {
            calcPdoSize = (WORD) curPdoSize;
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
\brief  setup PDO buffers

The function sets up the memory used to store PDO frames.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel setupPdoBuffers(void)
{
    tEplKernel              ret;
    UINT                    channelId;
    tPdoChannel*            pPdoChannel;

    ret = pdoucal_initPdoMem(&pdouInstance_g.pdoChannels, &pdouInstance_g.pPdoMem);
    if (ret != kEplSuccessful)
        return ret;

    // calculate pointers for TPDOs
    for (channelId = 0, pPdoChannel = &pdouInstance_g.pdoChannels.pTxPdoChannel[0];
         channelId < pdouInstance_g.pdoChannels.allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        pPdoChannel->pVar = pdoucal_allocatePdoMem(TRUE, channelId);
    }

    // calculate pointers for RPDOs
    for (channelId = 0, pPdoChannel = &pdouInstance_g.pdoChannels.pRxPdoChannel[0];
         channelId < pdouInstance_g.pdoChannels.allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        pPdoChannel->pVar = pdoucal_allocatePdoMem(FALSE, channelId);
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Copy variable to PDO

This function copies a variable specified by the mapping object to the PDO
payload.

\param  pPayload_p          Pointer to PDO payload in destination frame.
\param  pMappObject_p       Pointer to mapping object.

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel copyVarToPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            byteOffset;
    void*           pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p += byteOffset;
    pVar = PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kEplObdTypVString:
        case kEplObdTypOString:
        case kEplObdTypDomain:
        default:
            // read value from object
            EPL_MEMCPY (pPayload_p, pVar, PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));
            break;

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kEplObdTypBool:
        case kEplObdTypInt8:
        case kEplObdTypUInt8:
            AmiSetByteToLe(pPayload_p, *((BYTE*)pVar));
            break;

        // 16 bit values
        case kEplObdTypInt16:
        case kEplObdTypUInt16:
            AmiSetWordToLe(pPayload_p, *((WORD*)pVar));
            break;

        // 24 bit values
        case kEplObdTypInt24:
        case kEplObdTypUInt24:
            AmiSetDword24ToLe(pPayload_p, *((DWORD*)pVar));
            break;

        // 32 bit values
        case kEplObdTypInt32:
        case kEplObdTypUInt32:
        case kEplObdTypReal32:
            AmiSetDwordToLe(pPayload_p, *((DWORD*)pVar));
            break;

        // 40 bit values
        case kEplObdTypInt40:
        case kEplObdTypUInt40:
            AmiSetQword40ToLe(pPayload_p, *((QWORD*)pVar));
            break;

        // 48 bit values
        case kEplObdTypInt48:
        case kEplObdTypUInt48:
            AmiSetQword48ToLe(pPayload_p, *((QWORD*)pVar));
            break;

        // 56 bit values
        case kEplObdTypInt56:
        case kEplObdTypUInt56:
            AmiSetQword56ToLe(pPayload_p, *((QWORD*)pVar));
            break;

        // 64 bit values
        case kEplObdTypInt64:
        case kEplObdTypUInt64:
        case kEplObdTypReal64:
            AmiSetQword64ToLe(pPayload_p, *((QWORD*)pVar));
            break;

        // time of day
        case kEplObdTypTimeOfDay:
        case kEplObdTypTimeDiff:
            AmiSetTimeOfDay(pPayload_p, ((tTimeOfDay*)pVar));
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

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel copyVarFromPdo(BYTE* pPayload_p, tPdoMappObject* pMappObject_p)
{
    tEplKernel      Ret = kEplSuccessful;
    UINT            byteOffset;
    void*           pVar;

    byteOffset = PDO_MAPPOBJECT_GET_BITOFFSET(pMappObject_p) >> 3;
    pPayload_p += byteOffset;
    pVar = PDO_MAPPOBJECT_GET_VAR(pMappObject_p);

    switch (PDO_MAPPOBJECT_GET_TYPE(pMappObject_p))
    {
        //-----------------------------------------------
        // types without ami
        case kEplObdTypVString:
        case kEplObdTypOString:
        case kEplObdTypDomain:
        default:
            // read value from object
            EPL_MEMCPY (pVar, pPayload_p, PDO_MAPPOBJECT_GET_BYTESIZE(pMappObject_p));
            break;

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kEplObdTypBool:
        case kEplObdTypInt8:
        case kEplObdTypUInt8:
            *((BYTE*)pVar) = AmiGetByteFromLe(pPayload_p);
            break;

        // 16 bit values
        case kEplObdTypInt16:
        case kEplObdTypUInt16:
            *((WORD*)pVar) = AmiGetWordFromLe(pPayload_p);
            break;

        // 24 bit values
        case kEplObdTypInt24:
        case kEplObdTypUInt24:
            *((DWORD*)pVar) = AmiGetDword24FromLe(pPayload_p);
            break;

        // 32 bit values
        case kEplObdTypInt32:
        case kEplObdTypUInt32:
        case kEplObdTypReal32:
            *((DWORD*)pVar) = AmiGetDwordFromLe(pPayload_p);
            break;

        // 40 bit values
        case kEplObdTypInt40:
        case kEplObdTypUInt40:
            *((QWORD*)pVar) = AmiGetQword40FromLe(pPayload_p);
            break;

        // 48 bit values
        case kEplObdTypInt48:
        case kEplObdTypUInt48:
            *((QWORD*)pVar) = AmiGetQword48FromLe(pPayload_p);
            break;

        // 56 bit values
        case kEplObdTypInt56:
        case kEplObdTypUInt56:
            *((QWORD*)pVar) = AmiGetQword56FromLe(pPayload_p);
            break;

        // 64 bit values
        case kEplObdTypInt64:
        case kEplObdTypUInt64:
        case kEplObdTypReal64:
            *((QWORD*)pVar) = AmiGetQword64FromLe(pPayload_p);
            break;

        // time of day
        case kEplObdTypTimeOfDay:
        case kEplObdTypTimeDiff:
            AmiGetTimeOfDay(pVar, ((tTimeOfDay*)pPayload_p));
            break;
    }
    return Ret;
}

///\}
