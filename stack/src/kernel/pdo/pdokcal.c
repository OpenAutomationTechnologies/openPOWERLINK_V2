/**
********************************************************************************
\file   pdokcal.c

\brief  Implementation of kernel PDO CAL module

This file contains the implementation of the kernel PDO CAL module.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <kernel/pdokcal.h>
#include <kernel/pdok.h>
#include <kernel/dllk.h>
#include <kernel/eventk.h>
#include <common/ami.h>

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
static tOplkError cbProcessRpdo(const tFrameInfo* pFrameInfo_p) SECTION_PDOK_PROCESS_RPDO;


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the PDO kernel CAL module

The function initializes the PDO user CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_init(void)
{
    tOplkError  ret;

    ret = pdokcal_openMem();
    if (ret != kErrorOk)
        return ret;

    dllk_regRpdoHandler(cbProcessRpdo);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO kernel CAL module

The function de-initializes the PDO kernel CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_exit(void)
{
    pdokcal_closeMem();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process events from PdouCal module.

\param[in]      pEvent_p            Pointer to event structure

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
**/
//------------------------------------------------------------------------------
tOplkError pdokcal_process(const tEvent* pEvent_p)
{
    tOplkError  ret;

    switch (pEvent_p->eventType)
    {
        case kEventTypePdokAlloc:
            {
                const tPdoAllocationParam* pAllocationParam;

                pAllocationParam = (const tPdoAllocationParam*)pEvent_p->eventArg.pEventArg;
                ret = pdok_allocChannelMem(pAllocationParam);
            }
            break;

        case kEventTypePdokConfig:
            {
                const tPdoChannelConf* pChannelConf;

                pChannelConf = (const tPdoChannelConf*)pEvent_p->eventArg.pEventArg;
                ret = pdok_configureChannel(pChannelConf);
            }
            break;

        case kEventTypePdokSetupPdoBuf:
            {
                const tPdoMemSize* pPdoMemSize;

                pPdoMemSize = (const tPdoMemSize*)pEvent_p->eventArg.pEventArg;
                ret = pdok_setupPdoBuffers(pPdoMemSize->rxPdoMemSize,
                                           pPdoMemSize->txPdoMemSize);
            }
            break;

        case kEventTypePdoRx:
            {
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE)
                const tFrameInfo* pFrameInfo;

                pFrameInfo = (const tFrameInfo*)pEvent_p->eventArg.pEventArg;
                ret = pdok_processRxPdo(pFrameInfo->frame.pBuffer, pFrameInfo->frameSize);
#else
                const tPlkFrame* pFrame;

                pFrame = (const tPlkFrame*)pEvent_p->eventArg.pEventArg;
                ret = pdok_processRxPdo(pFrame, pEvent_p->eventArgSize);
#endif
            }
            break;

        default:
            ret = kErrorInvalidEvent;
            break;
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
\brief  Process received PDO

This function is called by the DLL if a PRes or a PReq frame have been received.
It posts the frame to the event queue. It is called in states
NMT_CS_READY_TO_OPERATE and NMT_CS_OPERATIONAL. The passed PDO needs not to be
valid.

\param[in]      pFrameInfo_p        Pointer to frame info structure

\return The function returns a tOplkError error code.
**/
//------------------------------------------------------------------------------
static tOplkError cbProcessRpdo(const tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdoRx;
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE)
    event.eventArgSize = sizeof(tFrameInfo);
    event.eventArg.pEventArg = (void*)pFrameInfo_p;
#else
    // limit copied data to size of PDO (because from some CNs the frame is larger than necessary)
    event.eventArgSize = ami_getUint16Le(&pFrameInfo_p->frame.pBuffer->data.pres.sizeLe) +
                          PLK_FRAME_OFFSET_PDO_PAYLOAD; // pFrameInfo_p->frameSize;
    event.eventArg.pEventArg = (void*)pFrameInfo_p->frame.pBuffer;
#endif
    ret = eventk_postEvent(&event);
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE)
    if (ret == kErrorOk)
        ret = kErrorReject; // Reject release of rx buffer
#endif

    return ret;
}

/// \}
