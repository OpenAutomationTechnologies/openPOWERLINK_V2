/**
********************************************************************************
\file   pdoucal.c

\brief  Generic functions of user PDO CAL module

This file contains the generic functions of the user PDO CAL module.

\ingroup module_pdoucal
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
#include <user/pdoucal.h>
#include <user/eventu.h>

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

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user CAL module

The function initializes the PDO user CAL module.

\param  pfnSyncCb_p             function that is called in case of sync event

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_init(tEplSyncCb pfnSyncCb_p)
{
    tOplkError      ret;

    if ((ret = pdoucal_openMem()) != kErrorOk)
        return ret;

    return pdoucal_initSync(pfnSyncCb_p);
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO user CAL module

The function cleans up the PDO user CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_exit(void)
{
    pdoucal_closeMem();
    pdoucal_exitSync();
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels in Pdok

This function allocates memory for PDOs according to the specified parameter
in the Pdok module by sending the appropriate event to Pdok.

\param  pAllocationParam_p      Allocation parameters containing info
                                needed for memory allocation.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postPdokChannelAlloc(tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError  Ret = kErrorOk;
    tEplEvent   Event;

    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdokAlloc;
    Event.m_pArg = pAllocationParam_p;
    Event.m_uiSize = sizeof (*pAllocationParam_p);

    Ret = eventu_postEvent(&Event);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send channel configuration to kernel PDO module

The function configures the specified PDO channel in the kernel by sending a
kEplEventTypePdokConfig to the kernel PDO module.

\param  pChannelConf_p          PDO channel configuration.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postConfigureChannel(tPdoChannelConf* pChannelConf_p)
{
    tOplkError      ret = kErrorOk;
    tEplEvent       Event;

    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdokConfig;
    Event.m_pArg = pChannelConf_p;
    Event.m_uiSize = sizeof(tPdoChannelConf);
    ret = eventu_postEvent(&Event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send PDO buffer setup to kernel PDO module

The function sends the PDO buffer setup to the kernel PDO module by posting
a kEplEventTypePdokSetupPdoBuf event.

\param  rxPdoMemSize_p          Size of RX PDO buffers.
\param  txPdoMemSize_p          Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postSetupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p)
{
    tOplkError      Ret = kErrorOk;
    tEplEvent       Event;
    tPdoMemSize     pdoMemSize;

    pdoMemSize.rxPdoMemSize = rxPdoMemSize_p;
    pdoMemSize.txPdoMemSize = txPdoMemSize_p;
    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdokSetupPdoBuf;
    Event.m_pArg = &pdoMemSize;
    Event.m_uiSize = sizeof(tPdoMemSize);
    Ret = eventu_postEvent(&Event);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

