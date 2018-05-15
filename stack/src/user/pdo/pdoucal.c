/**
********************************************************************************
\file   pdoucal.c

\brief  Generic functions of user PDO CAL module

This file contains the generic functions of the user PDO CAL module.

\ingroup module_pdoucal
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

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_init(void)
{
    tOplkError  ret;

    ret = pdoucal_openMem();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO user CAL module

The function cleans up the PDO user CAL module.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_exit(void)
{
    pdoucal_closeMem();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for PDO channels in pdok

This function allocates memory for PDOs according to the specified parameter
in the pdok module by sending the appropriate event to pdok.

\param[in]      pAllocationParam_p  Allocation parameters containing info
                                    needed for memory allocation.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postPdokChannelAlloc(const tPdoAllocationParam* pAllocationParam_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdokAlloc;
    event.eventArg.pEventArg = (void*)pAllocationParam_p;
    event.eventArgSize = sizeof(*pAllocationParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send channel configuration to kernel PDO module

The function configures the specified PDO channel in the kernel by sending a
kEventTypePdokConfig to the kernel PDO module.

\param[in]      pChannelConf_p      PDO channel configuration.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postConfigureChannel(const tPdoChannelConf* pChannelConf_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdokConfig;
    event.eventArg.pEventArg = (void*)pChannelConf_p;
    event.eventArgSize = sizeof(tPdoChannelConf);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send PDO buffer setup to kernel PDO module

The function sends the PDO buffer setup to the kernel PDO module by posting
a kEventTypePdokSetupPdoBuf event.

\param[in]      rxPdoMemSize_p      Size of RX PDO buffers.
\param[in]      txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_postSetupPdoBuffers(size_t rxPdoMemSize_p,
                                       size_t txPdoMemSize_p)
{
    tOplkError  ret;
    tEvent      event;
    tPdoMemSize pdoMemSize;

    pdoMemSize.rxPdoMemSize = (UINT32)rxPdoMemSize_p;
    pdoMemSize.txPdoMemSize = (UINT32)txPdoMemSize_p;
    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdokSetupPdoBuf;
    event.eventArg.pEventArg = &pdoMemSize;
    event.eventArgSize = sizeof(tPdoMemSize);

    ret = eventu_postEvent(&event);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
