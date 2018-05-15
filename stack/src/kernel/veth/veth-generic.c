/**
********************************************************************************
\file   veth-generic.c

\brief  Generic implementation of virtual Ethernet driver

This file contains the virtual Ethernet driver generic implementation which
forwards non-POWERLINK frames through the kernel-to-user event queue to the
user layer.

\ingroup module_veth
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <kernel/veth.h>
#include <kernel/dllk.h>
#include <kernel/dllkcal.h>

#if defined(CONFIG_INCLUDE_VETH)
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
static tOplkError receiveFrameCb(tFrameInfo* pFrameInfo_p,
                                 tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize virtual Ethernet

The function initializes the virtual Ethernet module.

\param[in]      aSrcMac_p           MAC address to set for virtual Ethernet interface.

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_init(const UINT8 aSrcMac_p[6])
{
    tOplkError ret;

    UNUSED_PARAMETER(aSrcMac_p);

    // Register receive callback function for incoming virtual Ethernet frames
    ret = dllk_regAsyncHandler(receiveFrameCb);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down virtual Ethernet

The function shuts down the virtual Ethernet module.

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_exit(void)
{
    tOplkError ret;

    // Unregister the receive callback function
    ret = dllk_deregAsyncHandler(receiveFrameCb);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Receive frame call back function

This function is called by DLLk if a non POWERLINK frame is received.

\param[in,out]  pFrameInfo_p        Frame info of the received frame
\param[out]     pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                                    set this flag to determine if the RxBuffer could be
                                    released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveFrameCb(tFrameInfo* pFrameInfo_p,
                                 tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tOplkError ret;

    ret = dllkcal_asyncFrameReceived(pFrameInfo_p);
    if (ret == kErrorReject)
    {
        ret = kErrorOk;
        *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
    }

    return ret;
}

/// \}

#endif // CONFIG_INCLUDE_VETH
