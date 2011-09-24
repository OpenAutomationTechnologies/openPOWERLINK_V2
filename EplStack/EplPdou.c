/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for user part PDO module
                This module implements an OD callback function
                to check if the PDO configuration is valid
                and forwards the configuration to the corresponding
                kernel part module.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/05/22 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "EplInc.h"
#include "user/EplPdouCal.h"
#include "user/EplObdu.h"
#include "user/EplPdou.h"
#include "EplSdoAc.h"
#include "EplPdo.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOU)) != 0)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) == 0) && (EPL_OBD_USE_KERNEL == FALSE)
#error "EPL PDOu module needs EPL module OBDU or OBDK!"
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EPL_PDOU_OBD_IDX_RX_COMM_PARAM  0x1400
#define EPL_PDOU_OBD_IDX_RX_MAPP_PARAM  0x1600
#define EPL_PDOU_OBD_IDX_TX_COMM_PARAM  0x1800
#define EPL_PDOU_OBD_IDX_TX_MAPP_PARAM  0x1A00
#define EPL_PDOU_OBD_IDX_MAPP_PARAM     0x0200
#define EPL_PDOU_OBD_IDX_MASK           0xFF00
#define EPL_PDOU_PDO_ID_MASK            0x00FF


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplPdou                                             */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    BYTE    m_abPdoIdToChannelIdRx[(EPL_PDOU_PDO_ID_MASK + 1)];
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // only MN supports multiple TPDOS where indexing is necessary
    BYTE    m_abPdoIdToChannelIdTx[(EPL_PDOU_PDO_ID_MASK + 1)];
#endif

    BOOL    m_fAllocated;
    BOOL    m_fRunning;

} tEplPdouInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplPdouInstance  EplPdouInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplPdouAlloc(BYTE abChannelIdToPdoIdRx_p[EPL_D_PDO_RPDOChannels_U16], unsigned int* puiCountChannelIdRx_p,
                               BYTE abChannelIdToPdoIdTx_p[EPL_D_PDO_TPDOChannels_U16], unsigned int* puiCountChannelIdTx_p);

static tEplKernel EplPdouConfigureAllPdos(void);

static tEplKernel EplPdouCheckAndConfigurePdo(unsigned int uiIndex_p, BYTE  bMappObjectCount_p, tEplObdAccess AccessType_p, DWORD* pdwAbortCode_p);

static tEplKernel EplPdouCheckPdoValidity(unsigned int uiMappParamIndex_p, DWORD* pdwAbortCode_p);

static void EplPdouDecodeObjectMapping(QWORD qwObjectMapping_p,
                                    unsigned int* puiIndex_p,
                                    unsigned int* puiSubIndex_p,
                                    unsigned int* puiBitOffset_p,
                                    unsigned int* puiBitSize_p);

static tEplKernel EplPdouCheckObjectMapping(QWORD qwObjectMapping_p,
                                       tEplObdAccess AccessType_p,
                                       tEplPdoMappObject* pMappObject_p,
                                       DWORD* pdwAbortCode_p,
                                       unsigned int* puiPdoSize_p);

static tEplKernel EplPdouConfigureChannel(tEplPdoChannelConf* pChannelConf_p);

static unsigned int EplPdouRxPdoIdToChannelId(unsigned int uiPdoId_p);

static unsigned int EplPdouTxPdoIdToChannelId(unsigned int uiPdoId_p);



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplPdouAddInstance()
//
// Description: add and initialize new instance of EPL stack
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdouAddInstance(void)
{
    EPL_MEMSET(&EplPdouInstance_g, 0, sizeof(EplPdouInstance_g));
    EplPdouInstance_g.m_fAllocated = FALSE;
    EplPdouInstance_g.m_fRunning = FALSE;

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouDelInstance()
//
// Description: deletes an instance of EPL stack
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdouDelInstance(void)
{

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCbNmtStateChange
//
// Description: callback function for NMT state changes
//
// Parameters:  NmtStateChange_p        = NMT state change event
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplPdouCbNmtStateChange(tEplEventNmtStateChange NmtStateChange_p)
{
tEplKernel      Ret = kEplSuccessful;

    switch (NmtStateChange_p.m_NewNmtState)
    {
        case kEplNmtGsOff:
        case kEplNmtGsInitialising:
        case kEplNmtGsResetApplication:
        case kEplNmtGsResetCommunication:
        {
            EplPdouInstance_g.m_fAllocated = FALSE;
            EplPdouInstance_g.m_fRunning = FALSE;
            break;
        }

        case kEplNmtGsResetConfiguration:
        {
            EplPdouInstance_g.m_fAllocated = FALSE;
            EplPdouInstance_g.m_fRunning = FALSE;

            // forward PDO configuration to Pdok module
            Ret = EplPdouConfigureAllPdos();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            EplPdouInstance_g.m_fRunning = TRUE;

            break;
        }

        default:
        {   // do nothing
            break;
        }

    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCbObdAccess
//
// Description: callback function for OD accesses
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplPdouCbObdAccess(tEplObdCbParam MEM* pParam_p)
{
tEplKernel          Ret = kEplSuccessful;
unsigned int        uiIndexType;
BYTE                bMappObjectCount;
tEplObdAccess       AccessType;
unsigned int        uiCurPdoSize;

    pParam_p->m_dwAbortCode = 0;

    if (pParam_p->m_ObdEvent != kEplObdEvPreWrite)
    {   // read accesses, post write events etc. are OK
        goto Exit;
    }

    // fetch object index type
    uiIndexType = pParam_p->m_uiIndex & EPL_PDOU_OBD_IDX_MASK;

    // check index type
    switch (uiIndexType)
    {
        case EPL_PDOU_OBD_IDX_RX_COMM_PARAM:
            // RPDO communication parameter accessed
        case EPL_PDOU_OBD_IDX_TX_COMM_PARAM:
        {   // TPDO communication parameter accessed
            Ret = EplPdouCheckPdoValidity(
                    (EPL_PDOU_OBD_IDX_MAPP_PARAM | pParam_p->m_uiIndex),
                    &pParam_p->m_dwAbortCode);
            if (Ret != kEplSuccessful)
            {   // PDO is valid or does not exist
                goto Exit;
            }

            goto Exit;
        }

        case EPL_PDOU_OBD_IDX_RX_MAPP_PARAM:
        {   // RPDO mapping parameter accessed

            AccessType = kEplObdAccWrite;
            break;
        }

        case EPL_PDOU_OBD_IDX_TX_MAPP_PARAM:
        {   // TPDO mapping parameter accessed

            AccessType = kEplObdAccRead;
            break;
        }

        default:
        {   // this callback function is only for
            // PDO mapping and communication parameters
            pParam_p->m_dwAbortCode = EPL_SDOAC_GENERAL_ERROR;
            Ret = kEplPdoInvalidObjIndex;
            goto Exit;
        }
    }

    // RPDO and TPDO mapping parameter accessed

    if (pParam_p->m_uiSubIndex == 0)
    {   // object mapping count accessed

        // PDO is enabled or disabled
        bMappObjectCount = *((BYTE*) pParam_p->m_pArg);

        Ret = EplPdouCheckAndConfigurePdo(pParam_p->m_uiIndex, bMappObjectCount, AccessType, &pParam_p->m_dwAbortCode);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    else
    {   // ObjectMapping
    tEplPdoMappObject   MappObject;
    QWORD               qwObjectMapping;

        Ret = EplPdouCheckPdoValidity(pParam_p->m_uiIndex, &pParam_p->m_dwAbortCode);
        if (Ret != kEplSuccessful)
        {   // PDO is valid or does not exist
            goto Exit;
        }

        // check existence of object and validity of object length

        qwObjectMapping = *((QWORD*) pParam_p->m_pArg);

        Ret = EplPdouCheckObjectMapping(qwObjectMapping,
                               AccessType,
                               &MappObject,
                               &pParam_p->m_dwAbortCode,
                               &uiCurPdoSize);

    }

Exit:
    return Ret;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplPdouAlloc
//
// Description: allocate necessary memory for all existing PDOs
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouAlloc(BYTE abChannelIdToPdoIdRx_p[EPL_D_PDO_RPDOChannels_U16], unsigned int* puiCountChannelIdRx_p,
                               BYTE abChannelIdToPdoIdTx_p[EPL_D_PDO_TPDOChannels_U16], unsigned int* puiCountChannelIdTx_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdSize         ObdSize;
BYTE                bNodeId;
unsigned int        uiPdoId;
unsigned int        uiCommParamIndex;
tEplPdoAllocationParam  AllocParam;

    AllocParam.m_uiRxPdoChannelCount = 0;
    AllocParam.m_uiTxPdoChannelCount = 0;

    EPL_MEMSET(EplPdouInstance_g.m_abPdoIdToChannelIdRx, 0, sizeof (EplPdouInstance_g.m_abPdoIdToChannelIdRx));
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    EPL_MEMSET(EplPdouInstance_g.m_abPdoIdToChannelIdTx, 0, sizeof (EplPdouInstance_g.m_abPdoIdToChannelIdTx));
#endif

    for (uiPdoId = 0, uiCommParamIndex = EPL_PDOU_OBD_IDX_RX_COMM_PARAM;
        uiPdoId < sizeof (EplPdouInstance_g.m_abPdoIdToChannelIdRx);
        uiPdoId++, uiCommParamIndex++)
    {
        ObdSize = sizeof (bNodeId);
        // read node ID from OD
        Ret = EplObduReadEntry(uiCommParamIndex, 0x01, &bNodeId, &ObdSize);
        if ((Ret == kEplObdIndexNotExist)
            || (Ret == kEplObdSubindexNotExist)
            || (Ret == kEplObdIllegalPart))
        {   // PDO does not exist
            continue;
        }
        else if (Ret != kEplSuccessful)
        {   // other fatal error occurred
            goto Exit;
        }

        if (AllocParam.m_uiRxPdoChannelCount > EPL_D_PDO_RPDOChannels_U16)
        {
            Ret = kEplPdoTooManyPdos;
            goto Exit;
        }

        EplPdouInstance_g.m_abPdoIdToChannelIdRx[uiPdoId] = (BYTE) AllocParam.m_uiRxPdoChannelCount;
        abChannelIdToPdoIdRx_p[AllocParam.m_uiRxPdoChannelCount] = (BYTE) uiPdoId;

        AllocParam.m_uiRxPdoChannelCount++;
    }
    *puiCountChannelIdRx_p = AllocParam.m_uiRxPdoChannelCount;

    for (uiPdoId = 0, uiCommParamIndex = EPL_PDOU_OBD_IDX_TX_COMM_PARAM;
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        uiPdoId < sizeof (EplPdouInstance_g.m_abPdoIdToChannelIdRx);
#else
        uiPdoId < (EPL_PDOU_PDO_ID_MASK + 1);
#endif
        uiPdoId++, uiCommParamIndex++)
    {
        ObdSize = sizeof (bNodeId);
        // read node ID from OD
        Ret = EplObduReadEntry(uiCommParamIndex, 0x01, &bNodeId, &ObdSize);
        if ((Ret == kEplObdIndexNotExist)
            || (Ret == kEplObdSubindexNotExist)
            || (Ret == kEplObdIllegalPart))
        {   // PDO does not exist
            continue;
        }
        else if (Ret != kEplSuccessful)
        {   // other fatal error occurred
            goto Exit;
        }

        if (AllocParam.m_uiTxPdoChannelCount > EPL_D_PDO_TPDOChannels_U16)
        {
            Ret = kEplPdoTooManyTxPdos;
            goto Exit;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        EplPdouInstance_g.m_abPdoIdToChannelIdTx[uiPdoId] = (BYTE) AllocParam.m_uiTxPdoChannelCount;
#endif
        abChannelIdToPdoIdTx_p[AllocParam.m_uiTxPdoChannelCount] = (BYTE) uiPdoId;

        AllocParam.m_uiTxPdoChannelCount++;
    }
    *puiCountChannelIdTx_p = AllocParam.m_uiTxPdoChannelCount;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0)
    // no MN support, so we must not have more than one TPDO
    if (AllocParam.m_uiTxPdoChannelCount > 1)
    {
        Ret = kEplPdoTooManyTxPdos;
        goto Exit;
    }
#endif

    // inform Pdok module
    Ret = EplPdouCalAlloc(&AllocParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    EplPdouInstance_g.m_fAllocated = TRUE;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouConfigureAllPdos
//
// Description: configures all PDOs in Pdok module
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouConfigureAllPdos(void)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdSize         ObdSize;
BYTE                bMappObjectCount;
unsigned int        uiIndex;
unsigned int        uiMappParamIndex;
BYTE                abChannelIdToPdoIdRx[EPL_D_PDO_RPDOChannels_U16];
unsigned int        uiCountChannelIdRx;
BYTE                abChannelIdToPdoIdTx[EPL_D_PDO_TPDOChannels_U16];
unsigned int        uiCountChannelIdTx;
DWORD               dwAbortCode = 0;

    // check number of PDOs and allocate memory for them
    Ret = EplPdouAlloc(abChannelIdToPdoIdRx, &uiCountChannelIdRx, abChannelIdToPdoIdTx, &uiCountChannelIdTx);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    for (uiIndex = 0; uiIndex < uiCountChannelIdRx; uiIndex++)
    {
        uiMappParamIndex = EPL_PDOU_OBD_IDX_RX_MAPP_PARAM + abChannelIdToPdoIdRx[uiIndex];

        ObdSize = sizeof (bMappObjectCount);
        // read mapping object count from OD
        Ret = EplObduReadEntry(uiMappParamIndex, 0x00, &bMappObjectCount, &ObdSize);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EplPdouCheckAndConfigurePdo(uiMappParamIndex, bMappObjectCount, kEplObdAccWrite, &dwAbortCode);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    for (uiIndex = 0; uiIndex < uiCountChannelIdTx; uiIndex++)
    {
        uiMappParamIndex = EPL_PDOU_OBD_IDX_TX_MAPP_PARAM + abChannelIdToPdoIdTx[uiIndex];

        ObdSize = sizeof (bMappObjectCount);
        // read mapping object count from OD
        Ret = EplObduReadEntry(uiMappParamIndex, 0x00, &bMappObjectCount, &ObdSize);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EplPdouCheckAndConfigurePdo(uiMappParamIndex, bMappObjectCount, kEplObdAccRead, &dwAbortCode);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCheckAndConfigurePdo
//
// Description: checks and configures the specified PDO in Pdok module
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouCheckAndConfigurePdo(unsigned int uiMappParamIndex_p, BYTE  bMappObjectCount_p, tEplObdAccess AccessType_p, DWORD* pdwAbortCode_p)
{
tEplKernel          Ret = kEplSuccessful;
unsigned int        uiPdoId;
unsigned int        uiCommParamIndex;
tEplObdSize         ObdSize;
BYTE                bNodeId;
QWORD               qwObjectMapping;
BYTE                bMappSubindex;
unsigned int        uiCurPdoSize;
WORD                wMaxPdoSize;
WORD                wCalcPdoSize = 0;
unsigned int        uiPayloadLimitIndex;
unsigned int        uiPayloadLimitSubIndex;
tEplPdoChannelConf  PdoChannelConf;
tEplPdoMappObject*  pMappObject;

    // fetch PDO ID
    uiPdoId = uiMappParamIndex_p & EPL_PDOU_PDO_ID_MASK;

    uiCommParamIndex = ~EPL_PDOU_OBD_IDX_MAPP_PARAM & uiMappParamIndex_p;

    // check access type
    switch (AccessType_p)
    {
        case kEplObdAccWrite:
        {   // RPDO mapping parameter accessed

            PdoChannelConf.m_uiChannelId = EplPdouRxPdoIdToChannelId(uiPdoId);
            PdoChannelConf.m_fTx = FALSE;
            break;
        }

        case kEplObdAccRead:
        {   // TPDO mapping parameter accessed

            PdoChannelConf.m_uiChannelId = EplPdouTxPdoIdToChannelId(uiPdoId);
            PdoChannelConf.m_fTx = TRUE;
            break;
        }

        default:
        {
            *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
            Ret = kEplPdoInvalidObjIndex;
            goto Exit;
        }
    }

    if (bMappObjectCount_p == 0)
    {   // PDO shall be disabled

        PdoChannelConf.m_PdoChannel.m_uiNodeId = EPL_PDO_INVALID_NODE_ID;
        PdoChannelConf.m_PdoChannel.m_uiMappObjectCount = 0;

        Ret = EplPdouConfigureChannel(&PdoChannelConf);

        goto Exit;
    }

    // PDO shall be enabled

    Ret = EplPdouCheckPdoValidity(uiMappParamIndex_p, pdwAbortCode_p);
    if (Ret != kEplSuccessful)
    {   // PDO is valid or does not exist
        goto Exit;
    }

    ObdSize = sizeof (bNodeId);
    // read node ID from OD
    Ret = EplObduReadEntry(uiCommParamIndex, 0x01, &bNodeId, &ObdSize);
    if (Ret != kEplSuccessful)
    {   // fatal error occurred
        goto Exit;
    }

    if (AccessType_p == kEplObdAccWrite)
    {
        if (bNodeId == EPL_PDO_PREQ_NODE_ID)
        {
            uiPayloadLimitIndex = 0x1F98;   // NMT_CycleTiming_REC
            uiPayloadLimitSubIndex = 0x04;  // PReqActPayloadLimit_U16
        }
        else
        {
            uiPayloadLimitIndex = 0x1F8D;   // NMT_PResPayloadLimitList_AU16
            uiPayloadLimitSubIndex = bNodeId;
        }
    }
    else
    {
        if (bNodeId == EPL_PDO_PRES_NODE_ID)
        {
            uiPayloadLimitIndex = 0x1F98;   // NMT_CycleTiming_REC
            uiPayloadLimitSubIndex = 0x05;  // PResActPayloadLimit_U16
        }
        else
        {
            uiPayloadLimitIndex = 0x1F8B;   // NMT_MNPReqPayloadLimitList_AU16
            uiPayloadLimitSubIndex = bNodeId;
        }
    }

    // fetch maximum PDO size from OD
    ObdSize = sizeof (wMaxPdoSize);
    Ret = EplObduReadEntry(uiPayloadLimitIndex, uiPayloadLimitSubIndex, &wMaxPdoSize, &ObdSize);
    if (Ret != kEplSuccessful)
    {   // other fatal error occurred
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        goto Exit;
    }

    // check number of mappings agains size of array
    if (bMappObjectCount_p > tabentries(PdoChannelConf.m_aMappObject))
    {
        *pdwAbortCode_p = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
        Ret = kEplObdValueTooHigh;
        goto Exit;
    }

    pMappObject = &PdoChannelConf.m_aMappObject[0];

    // check all objectmappings
    for (bMappSubindex = 1; bMappSubindex <= bMappObjectCount_p; bMappSubindex++)
    {
        // read object mapping from OD
        ObdSize = sizeof (qwObjectMapping); // QWORD
        Ret = EplObduReadEntry(uiMappParamIndex_p,
                            bMappSubindex, &qwObjectMapping, &ObdSize);
        if (Ret != kEplSuccessful)
        {   // other fatal error occurred
            *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
            goto Exit;
        }

        // check object mapping
        Ret = EplPdouCheckObjectMapping(qwObjectMapping,
                               AccessType_p,
                               pMappObject,
                               pdwAbortCode_p,
                               &uiCurPdoSize);
        if (Ret != kEplSuccessful)
        {   // illegal object mapping
            goto Exit;
        }

        if (uiCurPdoSize == 0)
        {   // null mapping, go to next subindex
            continue;
        }

        if (uiCurPdoSize > wMaxPdoSize)
        {   // mapping exceeds object size
            *pdwAbortCode_p = EPL_SDOAC_PDO_LENGTH_EXCEEDED;
            Ret = kEplPdoLengthExceeded;
            goto Exit;
        }

        if (uiCurPdoSize > wCalcPdoSize)
        {
            wCalcPdoSize = (WORD) uiCurPdoSize;
        }

        pMappObject++;

    }

    PdoChannelConf.m_PdoChannel.m_uiNodeId = bNodeId;

    ObdSize = sizeof (PdoChannelConf.m_PdoChannel.m_bMappingVersion);
    // read PDO mapping version
    Ret = EplObdReadEntry(uiCommParamIndex, 0x02, &PdoChannelConf.m_PdoChannel.m_bMappingVersion, &ObdSize);
    if (Ret != kEplSuccessful)
    {   // other fatal error occurred
        goto Exit;
    }

    PdoChannelConf.m_PdoChannel.m_wPdoSize = wCalcPdoSize;
    PdoChannelConf.m_PdoChannel.m_uiMappObjectCount = (unsigned int) (pMappObject - &PdoChannelConf.m_aMappObject[0]);

    // do not make the call before Alloc has been called
    Ret = EplPdouConfigureChannel(&PdoChannelConf);
    if (Ret != kEplSuccessful)
    {   // fatal error occurred
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        goto Exit;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouConfigureChannel()
//
// Description: This function configures the specified PDO channel.
//
// Parameters:  pChannelConf_p          = PDO channel configuration
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouConfigureChannel(tEplPdoChannelConf* pChannelConf_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (EplPdouInstance_g.m_fAllocated != FALSE)
    {
        Ret = EplPdouCalConfigureChannel(pChannelConf_p);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouRxPdoIdToChannelId
//
// Description: converts RPDO-ID (i.e. lower part of object index) to channel ID
//              which is used by Pdok module.
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static unsigned int EplPdouRxPdoIdToChannelId(unsigned int uiPdoId_p)
{
    return EplPdouInstance_g.m_abPdoIdToChannelIdRx[uiPdoId_p];
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouTxPdoIdToChannelId
//
// Description: converts TPDO-ID (i.e. lower part of object index) to channel ID
//              which is used by Pdok module.
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static unsigned int EplPdouTxPdoIdToChannelId(unsigned int uiPdoId_p)
{
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    return EplPdouInstance_g.m_abPdoIdToChannelIdTx[uiPdoId_p];
#else
    return 0;
#endif
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCheckPdoValidity
//
// Description: check if PDO is valid
//
// Parameters:  uiMappParamIndex_p      = object index of mapping parameter
//              pdwAbortCode_p          = [OUT] pointer to SDO abort code
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouCheckPdoValidity(unsigned int uiMappParamIndex_p, DWORD* pdwAbortCode_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdSize         ObdSize;
BYTE                bMappObjectCount;

    if (EplPdouInstance_g.m_fRunning != FALSE)
    {
        // outside from NMT reset states the PDO should have been disabled before changing it

        ObdSize = sizeof (bMappObjectCount);
        // read number of mapped objects from OD; this indicates if the PDO is valid
        Ret = EplObduReadEntry(uiMappParamIndex_p, 0x00, &bMappObjectCount, &ObdSize);
        if (Ret != kEplSuccessful)
        {   // other fatal error occurred
            *pdwAbortCode_p = EPL_SDOAC_GEN_INTERNAL_INCOMPATIBILITY;
            goto Exit;
        }
        // entry read successfully
        if (bMappObjectCount != 0)
        {   // PDO in OD is still valid
            *pdwAbortCode_p = EPL_SDOAC_GEN_PARAM_INCOMPATIBILITY;
            Ret = kEplPdoConfWhileEnabled;
            goto Exit;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCheckObjectMapping
//
// Description: checks the given object mapping entry.
//
// Parameters:  qwObjectMapping_p       = object mapping entry
//              AccessType_p            = access type to mapped object:
//                                        write = RPDO and read = TPDO
//              pMappObject             = [OUT] pointer to mapping object structure
//                                              which will be filled out
//              puiPdoSize_p            = [OUT] pointer to covered PDO size
//                                        (offset + size) in [byte];
//                                        0 if mapping failed
//              pdwAbortCode_p          = [OUT] pointer to SDO abort code;
//                                        0 if mapping is possible
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdouCheckObjectMapping(QWORD qwObjectMapping_p,
                                       tEplObdAccess AccessType_p,
                                       tEplPdoMappObject* pMappObject_p,
                                       DWORD* pdwAbortCode_p,
                                       unsigned int* puiPdoSize_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdSize         ObdSize;
unsigned int        uiIndex;
unsigned int        uiSubIndex;
unsigned int        uiBitOffset;
unsigned int        uiBitSize;
unsigned int        uiByteSize;
tEplObdAccess       AccessType;
BOOL                fNumerical;
tEplObdType         ObdType;
void*               pVar;

    if (qwObjectMapping_p == 0)
    {   // discard zero value
        *puiPdoSize_p = 0;
        goto Exit;
    }

    // decode object mapping
    EplPdouDecodeObjectMapping(qwObjectMapping_p,
                               &uiIndex,
                               &uiSubIndex,
                               &uiBitOffset,
                               &uiBitSize);

    if ((uiBitOffset & 0x7) != 0x0)
    {   // bit mapping is not supported
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        Ret = kEplPdoGranularityMismatch;
        goto Exit;
    }

    Ret = EplObduGetType(uiIndex, uiSubIndex, &ObdType);
    if (Ret != kEplSuccessful)
    {   // entry doesn't exist
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        Ret = kEplPdoVarNotFound;
        goto Exit;
    }

    if (((uiBitSize & 0x7) != 0x0)
        && ((uiBitSize != 1) || (ObdType != kEplObdTypBool)))
    {   // bit mapping is not supported, except for BOOLEAN objects on byte boundaries
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        Ret = kEplPdoGranularityMismatch;
        goto Exit;
    }

    // check access type
    Ret = EplObduGetAccessType(uiIndex, uiSubIndex, &AccessType);
    if (Ret != kEplSuccessful)
    {   // entry doesn't exist
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        Ret = kEplPdoVarNotFound;
        goto Exit;
    }

    if ((AccessType & kEplObdAccPdo) == 0)
    {   // object is not mappable
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_MAPPABLE;
        Ret = kEplPdoVarNotMappable;
        goto Exit;
    }

    if ((AccessType & AccessType_p) == 0)
    {   // object is not writeable (RPDO) or readable (TPDO) respectively
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_MAPPABLE;
        Ret = kEplPdoVarNotMappable;
        goto Exit;
    }

    if (ObdType == kEplObdTypBool)
    {   // bit size of BOOLEAN object was checked above
        uiByteSize = 1;
    }
    else
    {
        uiByteSize = (uiBitSize >> 3);
    }

    ObdSize = EplObduGetDataSize(uiIndex, uiSubIndex);
    if (ObdSize < uiByteSize)
    {   // object does not exist or has smaller size
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        Ret = kEplPdoSizeMismatch;
    }

    Ret = EplObduIsNumerical(uiIndex, uiSubIndex, &fNumerical);
    if (Ret != kEplSuccessful)
    {   // entry doesn't exist
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        goto Exit;
    }

    if ((fNumerical != FALSE)
        && (uiByteSize != ObdSize))
    {
        // object is numerical,
        // therefore size has to fit, but it does not.
        *pdwAbortCode_p = EPL_SDOAC_GENERAL_ERROR;
        Ret = kEplPdoVarNotFound;
        goto Exit;
    }

    pVar = EplObduGetObjectDataPtr(uiIndex, uiSubIndex);
    if (pVar == NULL)
    {   // entry doesn't exist
        *pdwAbortCode_p = EPL_SDOAC_OBJECT_NOT_EXIST;
        Ret = kEplPdoVarNotFound;
        goto Exit;
    }

    EPL_PDO_MAPPOBJECT_SET_BITOFFSET(pMappObject_p, (WORD) uiBitOffset);
    EPL_PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pMappObject_p, (WORD) uiByteSize, ObdType);
    EPL_PDO_MAPPOBJECT_SET_VAR(pMappObject_p, pVar);

    // Calculate needed PDO size
    *puiPdoSize_p = (uiBitOffset >> 3) + uiByteSize;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouDecodeObjectMapping
//
// Description: decodes the given object mapping entry into index, subindex,
//              bit offset and bit size.
//
// Parameters:  qwObjectMapping_p       = object mapping entry
//              puiIndex_p              = [OUT] pointer to object index
//              puiSubIndex_p           = [OUT] pointer to subindex
//              puiBitOffset_p          = [OUT] pointer to bit offset
//              puiBitSize_p            = [OUT] pointer to bit size
//
// Returns:     (void)
//
// State:
//
//---------------------------------------------------------------------------

static void EplPdouDecodeObjectMapping(QWORD qwObjectMapping_p,
                                    unsigned int* puiIndex_p,
                                    unsigned int* puiSubIndex_p,
                                    unsigned int* puiBitOffset_p,
                                    unsigned int* puiBitSize_p)
{
    *puiIndex_p = (unsigned int)
                    (qwObjectMapping_p & 0x000000000000FFFFLL);

    *puiSubIndex_p = (unsigned int)
                    ((qwObjectMapping_p & 0x0000000000FF0000LL) >> 16);

    *puiBitOffset_p = (unsigned int)
                    ((qwObjectMapping_p & 0x0000FFFF00000000LL) >> 32);

    *puiBitSize_p = (unsigned int)
                    ((qwObjectMapping_p & 0xFFFF000000000000LL) >> 48);

}


#endif // #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOU)) != 0)

// EOF

