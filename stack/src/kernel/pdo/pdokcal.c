/**
********************************************************************************
\file   pdokcal.c

\brief  Implementation of kernel PDO CAL module

This file contains the implementation of the kernel PDO CAL module.

\ingroup module_pdokcal
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
#include "kernel/pdokcal.h"
#include <kernel/pdok.h>
#include "kernel/dllk.h"
#include "kernel/eventk.h"

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
static tEplKernel cbProcessRpdo(tFrameInfo * pFrameInfo_p) SECTION_PDOK_PROCESS_RPDO;


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize the PDO kernel CAL module

The function initializes the PDO user CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_init(void)
{
    tEplKernel      Ret = kEplSuccessful;

    if ((Ret = pdokcal_openMem()) != kEplSuccessful)
        return Ret;

    if ((Ret = pdokcal_initSync()) != kEplSuccessful)
        return Ret;

    dllk_regRpdoHandler(cbProcessRpdo);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO kernel CAL module

The function deinitializes the PDO kernel CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_exit(void)
{
    pdokcal_exitSync();
    pdokcal_closeMem();
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process events from PdouCal module.

\param  pEvent_p                Pointer to event structure

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
**/
//------------------------------------------------------------------------------
tEplKernel pdokcal_process(tEplEvent * pEvent_p)
{
    tEplKernel                  Ret = kEplSuccessful;

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypePdokAlloc:
            {
                tPdoAllocationParam* pAllocationParam;
                pAllocationParam = (tPdoAllocationParam*) pEvent_p->m_pArg;
                Ret = pdok_allocChannelMem(pAllocationParam);
            }
            break;

        case kEplEventTypePdokConfig:
            {
                tPdoChannelConf* pChannelConf;
                pChannelConf = (tPdoChannelConf*) pEvent_p->m_pArg;
                Ret = pdok_configureChannel(pChannelConf);
            }
            break;

        case kEplEventTypePdokSetupPdoBuf:
            {
                tPdoMemSize*     pPdoMemSize;
                pPdoMemSize = (tPdoMemSize*)pEvent_p->m_pArg;
                Ret = pdok_setupPdoBuffers(pPdoMemSize->rxPdoMemSize,
                                           pPdoMemSize->txPdoMemSize);
            }
            break;

        case kEplEventTypePdoRx:
            {
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
                tFrameInfo*  pFrameInfo;
                pFrameInfo = (tFrameInfo *) pEvent_p->m_pArg;
                Ret = pdok_processRxPdo(pFrameInfo->pFrame, pFrameInfo->frameSize);
#else
                tEplFrame*  pFrame;

                pFrame = (tEplFrame *) pEvent_p->m_pArg;

                Ret = pdok_processRxPdo(pFrame, pEvent_p->m_uiSize);
#endif
            }
            break;

        case kEplEventTypePdokControlSync:
            Ret = pdokcal_controlSync(*((BOOL*)pEvent_p->m_pArg));
            break;

        default:
            Ret = kEplInvalidEvent;
            break;
    }
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process received PDO

This function is called by DLL if PRes or PReq frame was received. It posts
the frame to the event queue. It is called in states NMT_CS_READY_TO_OPERATE
and NMT_CS_OPERATIONAL. The passed PDO needs not to be valid.

\param  pFrameInfo_p            pointer to frame info structure

\return The function returns a tEplKernel error code.
**/
//------------------------------------------------------------------------------
static tEplKernel cbProcessRpdo(tFrameInfo * pFrameInfo_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplEvent       event;

    event.m_EventSink = kEplEventSinkPdokCal;
    event.m_EventType = kEplEventTypePdoRx;
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
    event.m_uiSize    = sizeof(tFrameInfo);
    event.m_pArg      = pFrameInfo_p;
#else
    // limit copied data to size of PDO (because from some CNs the frame is larger than necessary)
    event.m_uiSize = AmiGetWordFromLe(&pFrameInfo_p->pFrame->m_Data.m_Pres.m_le_wSize) + EPL_FRAME_OFFSET_PDO_PAYLOAD; // pFrameInfo_p->frameSize;
    event.m_pArg = pFrameInfo_p->pFrame;
#endif
    ret = eventk_postEvent(&event);
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
    if (ret == kEplSuccessful)
    {
        ret = kEplReject; // Reject release of rx buffer
    }
#endif

    return ret;
}

///\}

