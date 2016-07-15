/**
********************************************************************************
\file   veth-ndisintemediate.c

\brief  Implementation of virtual Ethernet for Windows kernel

This file contains implementation for virtual Ethernet interface for openPOWERLINK
stack in Windows kernel.

The module uses the NDIS driver library for Tx and Rx packet exchange from
application layer into POWERLINK network.

\ingroup module_veth
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <kernel/veth.h>
#include <kernel/dllkcal.h>
#include <kernel/dllk.h>
#include <common/ami.h>

#include <ndisintermediate/ndis-im.h>

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
static void        veth_xmit(void* pVEthTxData_p, size_t size_p);
static tOplkError  veth_receiveFrame(tFrameInfo* pFrameInfo_p,
                                     tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize virtual Ethernet

The function initializes the virtual Ethernet module.

\param  aSrcMac_p       MAC address to set for virtual Ethernet interface.

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_init(const UINT8 aSrcMac_p[6])
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(aSrcMac_p);

    // Register Transmit routine for non-PLK packets
    ndis_registerVethHandler(veth_xmit);

    // register callback function in DLL
    ret = dllk_regAsyncHandler(veth_receiveFrame);

    if (ret != kErrorOk)
    {
        DEBUG_LVL_VETH_TRACE("veth_open: dllk_regAsyncHandler returned 0x%02X\n", ret);
        return ret;
    }

    return kErrorOk;
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
    ndis_registerVethHandler(NULL);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Transmit entry point of virtual Ethernet driver

The function contains the transmit function for the virtual Ethernet driver.

\param  pVEthTxData_p          Pointer to the packet which has to be transmitted.
\param  size_p                 Size of the packet.

*/
//------------------------------------------------------------------------------
static void veth_xmit(void* pVEthTxData_p, size_t size_p)
{
    tOplkError      ret = kErrorOk;
    tFrameInfo      frameInfo;

    if (pVEthTxData_p == NULL)
        return;

    OPLK_MEMSET(&frameInfo, 0, sizeof(tFrameInfo));

    frameInfo.frame.pBuffer = (tPlkFrame*)pVEthTxData_p;
    frameInfo.frameSize = (UINT32)size_p;

    //call send on DLL
    ret = dllkcal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_VETH_TRACE("veth_xmit: dllkcal_sendAsyncFrame returned 0x%02X\n", ret);
        goto Exit;
    }
    else
    {
        DEBUG_LVL_VETH_TRACE("veth_xmit: frame passed to DLL\n");
    }

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Receive frame from virtual Ethernet interface

The function receives a frame from the virtual Ethernet interface.

\param  pFrameInfo_p        Pointer to frame information of received frame.
\param  pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                            set this flag to determine if the RxBuffer could be
                            released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError veth_receiveFrame(tFrameInfo* pFrameInfo_p,
                                    tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tNdisErrorStatus status = kNdisStatusSuccess;

    status = ndis_vethReceive((void*)pFrameInfo_p->frame.pBuffer,
                              pFrameInfo_p->frameSize);
    if (status != kNdisStatusSuccess)
    {
        DEBUG_LVL_VETH_TRACE("%s() Unable to indicate received frames error 0x%2X\n",
                             __func__, status);
        return kErrorNoResource;
    }

    *pReleaseRxBuffer_p = kEdrvReleaseRxBufferImmediately;
    return kErrorOk;
}

/// \}

#endif // CONFIG_INCLUDE_VETH
