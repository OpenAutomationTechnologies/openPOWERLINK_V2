/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      EPL

  Description:  source file for user PDO module

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

#include "user/EplPdou.h"
#include "user/EplPdouCal.h"
#include "user/EplObdu.h"

#if ((EPL_MODULE_INTEGRATION & EPL_MODULE_PDOK) != 0)

/*#if ((EPL_MODULE_INTEGRATION & EPL_MODULE_OBDU) == 0) 

    #error 'ERROR: Missing OBDu-Modul!'

#endif*/

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
#define EPL_PDOU_OBD_IDX_MASK           0xFF00
#define EPL_PDOU_PDO_ID_MASK            0x00FF

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// modul globale vars
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

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


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
unsigned int        uiPdoId;
tEplObdSize         ObdSize;
BYTE                bObjectCount;
QWORD               qwObjectMapping;
unsigned int        uiIndex;
unsigned int        uiSubIndex;
unsigned int        uiBitOffset;
unsigned int        uiBitSize;

    // fetch PDO ID
    uiPdoId = pParam_p->m_uiIndex & EPL_PDOU_PDO_ID_MASK;

    if ((pParam_p->m_ObdEvent != kEplObdEvPreWrite)
        && (pParam_p->m_ObdEvent != kEplObdEvPostWrite))
    {   // read accesses etc. are OK
        pParam_p->m_dwAbortCode = 0;
        goto Exit;
    }

    // check index
    if ((pParam_p->m_uiIndex & EPL_PDOU_OBD_IDX_MASK) == EPL_PDOK_OBD_IDX_RX_COMM_PARAM)
    {
        if (pParam_p->m_ObdEvent == kEplObdEvPreWrite)
        {
            Ret = EplPdouCheckPdoValidity(pParam_p, (EPL_PDOK_OBD_IDX_RX_MAPP_PARAM | uiPdoId));
            if (Ret != kEplSuccessful)
            {   // other fatal error occured
                goto Exit;
            }
        }
    }
    else if ((pParam_p->m_uiIndex & EPL_PDOU_OBD_IDX_MASK) == EPL_PDOK_OBD_IDX_RX_MAPP_PARAM)
    {
        if (pParam_p->m_uiSubIndex == 0)
        {
        }
        else
        {   // ObjectMapping
            if (pParam_p->m_ObdEvent == kEplObdEvPreWrite)
            {
                Ret = EplPdouCheckPdoValidity(pParam_p, (EPL_PDOK_OBD_IDX_RX_MAPP_PARAM | uiPdoId));
                if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // check existence of object and validity of object length
                // decode object mapping -> move to separate function
                qwObjectMapping = *((QWORD*) pParam_p->m_pArg);
                uiIndex = (unsigned int) (qwObjectMapping & 0x000000000000FFFFLL);
                uiSubIndex = (unsigned int) ((qwObjectMapping & 0x0000000000FF0000LL) >> 16);
                uiBitOffset = (unsigned int) ((qwObjectMapping & 0x0000FFFF00000000LL) >> 32);
                uiBitSize = (unsigned int) ((qwObjectMapping & 0xFFFF000000000000LL) >> 48);
                ObdSize = EplObdGetDataSize(uiIndex, uiSubIndex);
                if (ObdSize < (uiBitSize >> 3))
                {   // object does not exist or has smaller size
                    pParam_p->m_dwAbortCode = EPL_SDOAC_OBJECT_NOT_MAPPABLE;
                    Ret = kEplPdoVarNotFound;
                }
                // check access type
            }
        }
    }
    else if ((pParam_p->m_uiIndex & EPL_PDOU_OBD_IDX_MASK) == EPL_PDOK_OBD_IDX_TX_COMM_PARAM)
    {
        Ret = EplPdouCheckPdoValidity(pParam_p, (EPL_PDOK_OBD_IDX_TX_MAPP_PARAM | uiPdoId));
        if (Ret != kEplSuccessful)
        {   // other fatal error occured
            goto Exit;
        }
    }
    else if ((pParam_p->m_uiIndex & EPL_PDOU_OBD_IDX_MASK) == EPL_PDOK_OBD_IDX_TX_MAPP_PARAM)
    {
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplPdouProcess
//
// Description: This function processes all received and transmitted PDOs.
//              This function must not be interrupted by any other task
//              except ISRs (like the ethernet driver ISR, which may call
//              EplPdouCbFrameReceived() or EplPdouCbFrameTransmitted()).
//
// Parameters:  pEvent_p                = pointer to event structure
// 
// Returns:     tEplKernel              = error code
//
//
// State:        
//
//---------------------------------------------------------------------------

tEplKernel EplPdouProcess(tEplEvent * pEvent_p)
{
tEplKernel      Ret = kEplSuccessful;
WORD    wPdoSize;
WORD    wBitOffset;
WORD    wBitSize;
WORD    wVarSize;
QWORD   qwObjectMapping;
BYTE    bMappSubindex;
BYTE    bObdSubindex;
WORD    wObdMappIndex;
WORD    wObdCommIndex;
WORD    wPdoId;
BYTE    bObdData;
BYTE    bObjectCount;
BYTE    bFrameData;
BOOL    fValid;
tEplObdSize     ObdSize;
tEplFrame      *pFrame;
tEplFrameInfo  *pFrameInfo;
unsigned int    uiNodeId;
tEplMsgType     MsgType;

    // 0xFF=invalid, RPDO: 0x00=PReq, localNodeId=PRes, remoteNodeId=PRes
    //               TPDO: 0x00=PRes, MN: CnNodeId=PReq

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypePdoRx:  // RPDO received
            pFrame = (tEplFrame *) pEvent_p->m_pArg;

            // check if received RPDO is valid
            bFrameData = AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bFlag1);
            if ((bFrameData & EPL_FRAME_FLAG1_RD) == 0)
            {   // RPDO invalid
                goto Exit;
            }

            // retrieve EPL message type
            MsgType = AmiGetByteFromLe(&pFrame->m_le_bMessageType);
            if (MsgType == kEplMsgTypePreq)
            {   // RPDO is PReq frame
                uiNodeId = EPL_PDO_PREQ_NODE_ID;  // 0x00
            }
            else
            {   // RPDO is PRes frame
                // retrieve node ID
                uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);
            }

            // search for appropriate valid RPDO in OD
            wObdMappIndex = EPL_PDOK_OBD_IDX_RX_MAPP_PARAM;
            wObdCommIndex = EPL_PDOK_OBD_IDX_RX_COMM_PARAM;
            for (wPdoId = 0; ; wPdoId++, wObdCommIndex++, wObdMappIndex++)
            {
                ObdSize = 1;
                // read node ID from OD
                Ret = EplObdReadEntry(wObdCommIndex, 0x01, &bObdData, &ObdSize);
                if ((Ret == kEplObdIndexNotExist)
                    || (Ret == kEplObdSubindexNotExist)
                    || (Ret == kEplObdIllegalPart))
                {   // PDO does not exist; last PDO reached
                    Ret = kEplSuccessful;
                    goto Exit;
                }
                else if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                if (bObdData != uiNodeId)
                {   // node ID does not equal - wrong PDO, try next PDO in OD
                    continue;
                }
                ObdSize = 1;
                // read number of mapped objects from OD; this indicates if the PDO is valid
                Ret = EplObdReadEntry(wObdMappIndex, 0x00, &bObjectCount, &ObdSize);
                if ((Ret == kEplObdIndexNotExist)
                    || (Ret == kEplObdSubindexNotExist)
                    || (Ret == kEplObdIllegalPart))
                {   // PDO does not exist; last PDO reached
                    Ret = kEplSuccessful;
                    goto Exit;
                }
                else if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                if (bObjectCount == 0)
                {   // PDO in OD not valid, try next PDO in OD
                    continue;
                }

                ObdSize = 1;
                // check PDO mapping version
                Ret = EplObdReadEntry(wObdCommIndex, 0x02, &bObdData, &ObdSize);
                if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                // retrieve PDO version from frame
                bFrameData = AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bPdoVersion);
                if ((bObdData != 0)
                    && (bFrameData != 0)
                    && ((bObdData & EPL_VERSION_MAIN) != (bFrameData & EPL_VERSION_MAIN)))
                {   // PDO versions do not match
                    // $$$ raise PDO error
                    // termiate processing of this RPDO
                    goto Exit;
                }

                // valid RPDO found

                // retrieve PDO size
                wPdoSize = AmiGetWordFromLe(&pFrame->m_Data.m_Pres.m_le_wSize);

                // process mapping
                for (bMappSubindex = 1; bMappSubindex <= bObjectCount; bMappSubindex++)
                {
                    ObdSize = 8;    // QWORD
                    // read object mapping from OD
                    Ret = EplObdReadEntry(wObdMappIndex, bMappSubindex, &qwObjectMapping, &ObdSize);
                    if (Ret != kEplSuccessful)
                    {   // other fatal error occured
                        goto Exit;
                    }

                    // decode object mapping
                    wObdCommIndex = (WORD) (qwObjectMapping & 0x000000000000FFFFLL);
                    bObdSubindex = (BYTE) ((qwObjectMapping & 0x0000000000FF0000LL) >> 16);
                    wBitOffset = (WORD) ((qwObjectMapping & 0x0000FFFF00000000LL) >> 32);
                    wBitSize = (WORD) ((qwObjectMapping & 0xFFFF000000000000LL) >> 48);

                    // check if object exceeds PDO size
                    if (((wBitOffset + wBitSize) >> 3) > wPdoSize)
                    {   // wrong object mapping; PDO size is too low
                        // $$$ raise PDO error
                        // terminate processing of this RPDO
                        goto Exit;
                    }

                    // copy object from RPDO to process/OD variable
                    ObdSize = wBitSize >> 3;
                    Ret = EplObdWriteEntryFromLe(wObdCommIndex, bObdSubindex, &pFrame->m_Data.m_Pres.m_le_abPayload[(wBitOffset >> 3)], ObdSize);
                    if (Ret != kEplSuccessful)
                    {   // other fatal error occured
                        goto Exit;
                    }

                }

                // processing finished successfully
                goto Exit;
            }
            break;

        case kEplEventTypePdoTx:  // TPDO transmitted
            pFrameInfo = (tEplFrameInfo *) pEvent_p->m_pArg;
            pFrame = pFrameInfo->m_pFrame;

            // set TPDO invalid, so that only fully processed TPDOs are sent as valid
            bFrameData = AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bFlag1);
            AmiSetByteToLe(&pFrame->m_Data.m_Pres.m_le_bFlag1, (bFrameData & ~EPL_FRAME_FLAG1_RD));

            // retrieve EPL message type
            MsgType = AmiGetByteFromLe(&pFrame->m_le_bMessageType);
            if (MsgType == kEplMsgTypePres)
            {   // TPDO is PRes frame
                uiNodeId = EPL_PDO_PRES_NODE_ID;  // 0x00
            }
            else
            {   // TPDO is PReq frame
                // retrieve node ID
                uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bDstNodeId);
            }

            // search for appropriate valid TPDO in OD
            wObdMappIndex = EPL_PDOK_OBD_IDX_TX_MAPP_PARAM;
            wObdCommIndex = EPL_PDOK_OBD_IDX_TX_COMM_PARAM;
            for (wPdoId = 0; ; wPdoId++, wObdCommIndex++, wObdMappIndex++)
            {
                ObdSize = 1;
                // read node ID from OD
                Ret = EplObdReadEntry(wObdCommIndex, 0x01, &bObdData, &ObdSize);
                if ((Ret == kEplObdIndexNotExist)
                    || (Ret == kEplObdSubindexNotExist)
                    || (Ret == kEplObdIllegalPart))
                {   // PDO does not exist; last PDO reached
                    Ret = kEplSuccessful;
                    goto Exit;
                }
                else if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                if (bObdData != uiNodeId)
                {   // node ID does not equal - wrong PDO, try next PDO in OD
                    continue;
                }
                ObdSize = 1;
                // read number of mapped objects from OD; this indicates if the PDO is valid
                Ret = EplObdReadEntry(wObdMappIndex, 0x00, &bObjectCount, &ObdSize);
                if ((Ret == kEplObdIndexNotExist)
                    || (Ret == kEplObdSubindexNotExist)
                    || (Ret == kEplObdIllegalPart))
                {   // PDO does not exist; last PDO reached
                    Ret = kEplSuccessful;
                    goto Exit;
                }
                else if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                if (bObjectCount == 0)
                {   // PDO in OD not valid, try next PDO in OD
                    continue;
                }

                // valid TPDO found

                ObdSize = 1;
                // get PDO mapping version from OD
                Ret = EplObdReadEntry(wObdCommIndex, 0x02, &bObdData, &ObdSize);
                if (Ret != kEplSuccessful)
                {   // other fatal error occured
                    goto Exit;
                }
                // entry read successfully
                // set PDO version in frame
                AmiSetByteToLe(&pFrame->m_Data.m_Pres.m_le_bPdoVersion, bObdData);

                // calculate PDO size
                wPdoSize = 0;

                // process mapping
                for (bMappSubindex = 1; bMappSubindex <= bObjectCount; bMappSubindex++)
                {
                    ObdSize = 8;    // QWORD
                    // read object mapping from OD
                    Ret = EplObdReadEntry(wObdMappIndex, bMappSubindex, &qwObjectMapping, &ObdSize);
                    if (Ret != kEplSuccessful)
                    {   // other fatal error occured
                        goto Exit;
                    }

                    // decode object mapping
                    wObdCommIndex = (WORD) (qwObjectMapping & 0x000000000000FFFFLL);
                    bObdSubindex = (BYTE) ((qwObjectMapping & 0x0000000000FF0000LL) >> 16);
                    wBitOffset = (WORD) ((qwObjectMapping & 0x0000FFFF00000000LL) >> 32);
                    wBitSize = (WORD) ((qwObjectMapping & 0xFFFF000000000000LL) >> 48);

                    // calculate max PDO size
                    ObdSize = wBitSize >> 3;
                    wVarSize = (wBitOffset >> 3) + (WORD) ObdSize;
                    if ((unsigned int)(wVarSize + 24) > pFrameInfo->m_uiFrameSize)
                    {   // TPDO is too short
                        // $$$ raise PDO error, set Ret
                        goto Exit;
                    }
                    if (wVarSize > wPdoSize)
                    {   // memorize new PDO size
                        wPdoSize = wVarSize;
                    }

                    // copy object from process/OD variable to TPDO
                    Ret = EplObdReadEntryToLe(wObdCommIndex, bObdSubindex, &pFrame->m_Data.m_Pres.m_le_abPayload[(wBitOffset >> 3)], &ObdSize);
                    if (Ret != kEplSuccessful)
                    {   // other fatal error occured
                        goto Exit;
                    }

                }

                // set PDO size in frame
                AmiSetWordToLe(&pFrame->m_Data.m_Pres.m_le_wSize, wPdoSize);

                Ret = EplPdouCalAreTpdosValid(&fValid);
                if (fValid != FALSE)
                {
                    // set TPDO valid
                    bFrameData = AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bFlag1);
                    AmiSetByteToLe(&pFrame->m_Data.m_Pres.m_le_bFlag1, (bFrameData | EPL_FRAME_FLAG1_RD));
                }

                // processing finished successfully

                goto Exit;
            }
            break;

        case kEplEventTypePdoSoa: // SoA received

            // invalidate TPDOs
            Ret = EplPdouCalSetTpdosValid(FALSE);
            break;

        default:
        {
            ASSERTMSG(FALSE, "EplPdouProcess(): unhandled event type!\n");
        }
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
// Function:    EplPdouCheckPdoValidity
//
// Description: check if PDO is valid
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplPdouCheckPdoValidity(tEplObdCbParam MEM* pParam_p, unsigned int uiIndex_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdSize         ObdSize;
BYTE                bObjectCount;

    ObdSize = 1;
    // read number of mapped objects from OD; this indicates if the PDO is valid
    Ret = EplObdReadEntry(uiIndex_p, 0x00, &bObjectCount, &ObdSize);
    if (Ret != kEplSuccessful)
    {   // other fatal error occured
        pParam_p->m_dwAbortCode = EPL_SDOAC_GEN_INTERNAL_INCOMPATIBILITY;
        goto Exit;
    }
    // entry read successfully
    if (bObjectCount != 0)
    {   // PDO in OD is still valid
        pParam_p->m_dwAbortCode = EPL_SDOAC_GEN_PARAM_INCOMPATIBILITY;
        Ret = kEplPdoNotExist;
        goto Exit;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:     
//
// Description:  
//               
//
//
// Parameters:   
//
// 
// Returns:      
//
//
// State:        
//
//---------------------------------------------------------------------------

#endif // #if ((EPL_MODULE_INTEGRATION & EPL_MODULE_PDOK) != 0)

// EOF

