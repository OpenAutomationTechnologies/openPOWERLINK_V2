/**
********************************************************************************
\file   edrv-ndisintermediate.c

\brief  Implementation of edrv module using NDIS intermediate driver interface

This file contains the implementation of Ethernet driver in Windows kernel which
uses NDIS interface to communicate with native miniport driver.

The driver uses the protocol access points of NDIS intermediate driver along
with the NDIS miniport access APIs to communicate with lower drivers and
exchange packets and signals.

The buffer handling for the TX and RX paths are done within the NDIS library
using linked lists. Callbacks for receive and transmit complete indication
are registered with NDIS.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <common/ami.h>
#include <kernel/edrv.h>

#include <ndisintermediate/ndis-im.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     256
#endif

#ifndef EDRV_MAX_TX_QUEUE
#define EDRV_MAX_TX_QUEUE       16
#endif
#define EDRV_TX_QUEUE_MASK      (EDRV_MAX_TX_QUEUE - 1)

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS     256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS       16
#endif
#define EDRV_RX_DESC_MASK       (EDRV_MAX_RX_DESCS - 1)

#define EDRV_MAX_BUFFER_SIZE    0x0600

#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_BUFFER_SIZE)    // n * (MTU + 14 + 4)

#define EDRV_MIN_FRAME_SIZE     60

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
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                   ///< Init parameters
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance    edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void edrvTxHandler(void* pTxBuff_p);
static void edrvRxHandler(void* pRxBuffer_p, size_t size_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(const tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError    ret = kErrorOk;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    // Check if NDIS intermediate driver is ready
    if (!ndis_checkBindingState())
    {
        // NDIS driver is not initialized
        DEBUG_LVL_ERROR_TRACE("%s() NDIS Driver not initialized\n", __func__);
        return kErrorEdrvInit;
    }

    // Allocate and prepare transmit and receive NetBufferLists
    ret = ndis_allocateTxRxBuff(EDRV_MAX_TX_BUFFERS, EDRV_MAX_RX_BUFFERS);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() TX and RX buffer allocation failed 0x%X\n", __func__, ret);
        return ret;
    }

    // Register Tx and Rx callbacks
    ndis_registerTxRxHandler(edrvTxHandler, edrvRxHandler);

    // Save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // Retrieve MAC address of the binding NIC
    ndis_getMacAddress(edrvInstance_l.initParam.aMacAddr);

    // Enable the NDIS intermediate driver
    ndis_setBindingState(kNdisBindingRunning);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down Ethernet driver

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_exit(void)
{
    // Fall-back to ready state
    ndis_setBindingState(kNdisBindingReady);

    // Free buffers
    ndis_freeTxRxBuff();
    ndis_registerTxRxHandler(NULL, NULL);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address

This function returns the MAC address of the Ethernet controller

\return The function returns a pointer to the MAC address.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
const UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    // Check if OID_802_3_DELETE_MULTICAST_ADDRESS along with
    // OID_GEN_CURRENT_PACKET_FILTER can be used to update the filter.
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);
    // Check if OID_802_3_DELETE_MULTICAST_ADDRESS along with
    // OID_GEN_CURRENT_PACKET_FILTER can be used to update the filter.
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                               UINT count_p,
                               UINT entryChanged_p,
                               UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tNdisErrorStatus    ndisStatus;
    void*               pTxBuffer = NULL;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_BUFFER_SIZE)
        return kErrorEdrvNoFreeBufEntry;

    ndisStatus = ndis_getTxBuff(&pTxBuffer, pBuffer_p->maxBufferSize,
                                &pBuffer_p->txBufferNumber.pArg);

    if (ndisStatus != kNdisStatusSuccess)
    {
        DEBUG_LVL_ERROR_TRACE("%s Tx buffers currently not allocated %x\n", __func__, ndisStatus);
        return kErrorEdrvNoFreeBufEntry;
    }

    if ((pTxBuffer != NULL) && (pBuffer_p->txBufferNumber.pArg != NULL))
    {
        pBuffer_p->pBuffer = pTxBuffer;
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("%s Error Allocating buffer\n", __func__);
        return kErrorEdrvNoFreeBufEntry;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    if (pBuffer_p == NULL)
        return kErrorEdrvBufNotExisting;

    ndis_freeTxBuff(pBuffer_p->txBufferNumber.pArg);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if ((pBuffer_p->txFrameSize > EDRV_MAX_BUFFER_SIZE))
    {
        return kErrorEdrvInvalidParam;
    }

    if (pBuffer_p->txFrameSize < EDRV_MIN_FRAME_SIZE)
    {
        pBuffer_p->txFrameSize = EDRV_MIN_FRAME_SIZE;
    }

    ndis_sendPacket(pBuffer_p, pBuffer_p->txFrameSize, pBuffer_p->txBufferNumber.pArg);

    return kErrorOk;
}

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Get Edrv module diagnostics

This function returns the Edrv diagnostics to a provided buffer.

\param[out]     pBuffer_p           Pointer to buffer filled with diagnostics.
\param[in]      size_p              Size of buffer

\return The function returns the size of the diagnostics information.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
int edrv_getDiagnostics(char* pBuffer_p, size_t size_p)
{
    UNUSED_PARAMETER(pBuffer_p);
    UNUSED_PARAMETER(size_p);

    return 0;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Receive Handler routine

Receive handler registered with NDIS intermediate driver. Intermediate driver
invokes receive from the NetBufferLists Receive handler.

\param[in,out]  pRxBuffer_p         A pointer to the Receive buffer.
\param[in]      size_p              Size of the received data.

*/
//------------------------------------------------------------------------------
static void edrvRxHandler(void* pRxBuffer_p, size_t size_p)
{
    tEdrvRxBuffer           rxBuffer;
    tEdrvReleaseRxBuffer    retReleaseRxBuffer;

    rxBuffer.pBuffer = pRxBuffer_p;
    rxBuffer.rxFrameSize = size_p;
    rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

    retReleaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
}

//------------------------------------------------------------------------------
/**
\brief  Transmit complete Handler routine

Transmit complete handler registered with NDIS intermediate driver.

\param[in]      pTxBuff_p           Pointer to Tx buffer.

*/
//------------------------------------------------------------------------------
static void edrvTxHandler(void* pTxBuff_p)
{
    tEdrvTxBuffer* const pBuffer = (tEdrvTxBuffer*)pTxBuff_p;

    if (pBuffer != NULL)
    {
        if (pBuffer->pfnTxHandler != NULL)
        {
            pBuffer->pfnTxHandler(pBuffer);
        }
    }
}

/// \}
