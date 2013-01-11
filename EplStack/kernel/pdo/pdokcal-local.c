/**
********************************************************************************
\file   pdokcal-local.c

\brief  Local implementation for kernel PDO CAL module

This file contains an implementation for the kernel PDO CAL module which has to
be used for single process solutions.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <EplInc.h>
#include <pdo.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
extern BOOL fPdou_bufCreated_g;
extern BYTE *pPdou_bufBase_g;
extern UINT pdou_bufSize_g;

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
typedef struct
{
    ULONG       channelOffset;
    UINT        channelSize;
} tChannelSetup;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tChannelSetup    rxChannelSetup[EPL_D_PDO_RPDOChannels_U16];
static tChannelSetup    txChannelSetup[EPL_D_PDO_TPDOChannels_U16];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static UINT setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize PDO memory

The function initializes the memory needed to transfer PDOs.

\param  pPdoChannels        Pointer to PDO channel configuration
\param  ppMem_p             Pointer to store the PDO memory pointer.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_initPdoMem(tPdoChannelSetup* pPdoChannels, BYTE** ppMem_p)
{
    setupPdoMemInfo(pPdoChannels);

    // for reads and writes the offset is used.
    // Therefore we return 0 so that addresses are equal to offsets.
    if (ppMem_p != NULL)
    {
        *ppMem_p = NULL;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO memory

The function cleans the memory allocated for PDO buffers.

\param  pMem_p                  Pointer to allocated memory.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_cleanupPdoMem(BYTE* pMem_p)
{
    UNUSED_PARAMETER(pMem_p);

    // in pdoucal-local the memory is freed, so we are done here
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO memory buffer

The function allocates a buffer to store a PDO and returns the pointer to it.

\param  fTxPdo_p                Determines if TXPDO or RXPDO buffer should be
                                allocated. Is TRUE for TXPDO.
\param  channelId               The PDO channel ID for which to allocate the
                                buffer.

\return Returns the address of the allocated PDO buffer.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
BYTE *pdokcal_allocatePdoMem(BOOL fTxPdo_p, UINT channelId)
{
    if (fTxPdo_p)
    {
        return (BYTE *)txChannelSetup[channelId].channelOffset;
    }
    else
    {
        return (BYTE *)rxChannelSetup[channelId].channelOffset;
    }
}

//------------------------------------------------------------------------------
/**
\brief	Write RXPDO to PDO memory

The function writes a received RXPDO into the PDO memory range.

\param  pPdo_p                  Pointer where to store the PDO in PDO memory.
\param  pPayload_p              Pointer to received PDO payload.
\param  pdoSize_p               Size of received PDO.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_writeRxPdo(BYTE* pPdo_p, BYTE *pPayload_p, UINT16 pdoSize_p)
{
    ULONG           offset;

    // offsets are used to access the buffer. As the base address was set
    // to 0 in PdokCal_AllocatePdoMem() the pointer is the same as the offset.
    offset = (ULONG)pPdo_p;

    if (fPdou_bufCreated_g != FALSE)
    {
        EPL_MEMCPY(&pPdou_bufBase_g[offset], pPayload_p, pdoSize_p);
    }
    else
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error writing to shared memory\n",
                               __func__);
    }

    //TRACE ("%s() *pPayload_p:%02x\n", __func__, *pPayload_p);
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief	Read TXPDO from PDO memory

The function reads a TXPDO to be sent into the PDO memory range.

\param  pPdo_p                  Pointer where to read the PDO in PDO memory.
\param  pPayload_p              Pointer to PDO payload which will be transmitted.
\param  pdoSize_p               Size of PDO to be transmitted.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_readTxPdo(BYTE* pPdo_p, BYTE* pPayload_p, UINT16 pdoSize_p)
{
    ULONG           offset;

    // offsets are used to access the buffer. As the base address was set
    // to 0 in PdokCal_AllocatePdoMem() the pointer is the same as the offset.
    offset = (ULONG)pPdo_p;

    //TRACE ("%s() size:%d\n", __func__, pdoSize_p);

    if (fPdou_bufCreated_g != FALSE)
    {
        EPL_MEMCPY(pPayload_p, &pPdou_bufBase_g[offset], pdoSize_p);
    }
    else
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error reading from shared memory\n",
                             __func__);
    }
    //TRACE ("%s() offset:%d *pPayload:%02x\n", __func__, ulOffset, *pPayload_p);

    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Setup PDO memory info

The function sets up the PDO memory info. For each channel the offset in the
shared buffer and the size is stored.

\param  pPdoChannels_p      Pointer to PDO channel setup.

\return The function returns the size of the used PDO memory
*/
//------------------------------------------------------------------------------
static UINT setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p)
{
    UINT                channelId;
    UINT                offset;
    tPdoChannel*        pPdoChannel;

    //TRACE ("-->%s()\n", __func__);
    offset = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE ("RX Channel: %d\n", channelId);
        //TRACE ("size: %d\n", pPdoChannel->pdoSize);
        rxChannelSetup[channelId].channelOffset = offset;
        rxChannelSetup[channelId].channelSize = pPdoChannel->pdoSize;
        offset += pPdoChannel->pdoSize;
    }

    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE ("TX Channel: %d\n", channelId);
        //TRACE ("size: %d\n", pPdoChannel->pdoSize);
        txChannelSetup[channelId].channelOffset = offset;
        txChannelSetup[channelId].channelSize = pPdoChannel->pdoSize;
        offset += pPdoChannel->pdoSize;
    }
    return offset;
}

///\}
