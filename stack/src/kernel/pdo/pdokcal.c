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
#include <oplk/ami.h>
#include <kernel/pdokcal.h>
#include <kernel/pdok.h>
#include <kernel/dllk.h>
#include <kernel/eventk.h>

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
static tOplkError cbProcessRpdo(tFrameInfo * pFrameInfo_p) SECTION_PDOK_PROCESS_RPDO;


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize the PDO kernel CAL module

The function initializes the PDO user CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_init(void)
{
    tOplkError      Ret = kErrorOk;

    if ((Ret = pdokcal_openMem()) != kErrorOk)
        return Ret;

    if ((Ret = pdokcal_initSync()) != kErrorOk)
        return Ret;

    dllk_regRpdoHandler(cbProcessRpdo);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO kernel CAL module

The function deinitializes the PDO kernel CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_exit(void)
{
    pdokcal_exitSync();
    pdokcal_closeMem();
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process events from PdouCal module.

\param  pEvent_p                Pointer to event structure

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
**/
//------------------------------------------------------------------------------
tOplkError pdokcal_process(tEvent * pEvent_p)
{
    tOplkError                  Ret = kErrorOk;

    switch (pEvent_p->eventType)
    {
        case kEventTypePdokAlloc:
            {
                tPdoAllocationParam* pAllocationParam;
                pAllocationParam = (tPdoAllocationParam*) pEvent_p->pEventArg;
                Ret = pdok_allocChannelMem(pAllocationParam);
            }
            break;

        case kEventTypePdokConfig:
            {
                tPdoChannelConf* pChannelConf;
                pChannelConf = (tPdoChannelConf*) pEvent_p->pEventArg;
                Ret = pdok_configureChannel(pChannelConf);
            }
            break;

        case kEventTypePdokSetupPdoBuf:
            {
                tPdoMemSize*     pPdoMemSize;
                pPdoMemSize = (tPdoMemSize*)pEvent_p->pEventArg;
                Ret = pdok_setupPdoBuffers(pPdoMemSize->rxPdoMemSize,
                                           pPdoMemSize->txPdoMemSize);
            }
            break;

        case kEventTypePdoRx:
            {
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE
                tFrameInfo*  pFrameInfo;
                pFrameInfo = (tFrameInfo *) pEvent_p->pEventArg;
                Ret = pdok_processRxPdo(pFrameInfo->pFrame, pFrameInfo->frameSize);
#else
                tPlkFrame* pFrame;

                pFrame = (tPlkFrame *) pEvent_p->pEventArg;

                Ret = pdok_processRxPdo(pFrame, pEvent_p->eventArgSize);
#endif
            }
            break;

        case kEventTypePdokControlSync:
            Ret = pdokcal_controlSync(*((BOOL*)pEvent_p->pEventArg));
            break;

        default:
            Ret = kErrorInvalidEvent;
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

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError cbProcessRpdo(tFrameInfo * pFrameInfo_p)
{
    tOplkError      ret = kErrorOk;
    tEvent          event;

    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdoRx;
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE
    event.eventArgSize    = sizeof(tFrameInfo);
    event.pEventArg      = pFrameInfo_p;
#else
    // limit copied data to size of PDO (because from some CNs the frame is larger than necessary)
    event.eventArgSize = ami_getUint16Le(&pFrameInfo_p->pFrame->data.pres.sizeLe) + PLK_FRAME_OFFSET_PDO_PAYLOAD; // pFrameInfo_p->frameSize;
    event.pEventArg = pFrameInfo_p->pFrame;
#endif
    ret = eventk_postEvent(&event);
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE
    if (ret == kErrorOk)
    {
        ret = kErrorReject; // Reject release of rx buffer
    }
#endif

    return ret;
}

///\}

