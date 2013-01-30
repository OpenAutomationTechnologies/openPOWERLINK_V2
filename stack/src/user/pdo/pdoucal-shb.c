/**
********************************************************************************
\file   pdoucal-shb.c

\brief  Shared buffer implementation for user PDO CAL module

This file contains an implementation for the user PDO CAL module which uses
shared buffers to transfer PDO data to the kernel layer.

\ingroup module_pdoucal
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
#include <SharedBuff.h>

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
static tShbInstance     shbInstance_l;
static BYTE             aTxPdo_l[EPL_C_DLL_ISOCHR_MAX_PAYL];
static BYTE             aRxPdo_l[EPL_C_DLL_ISOCHR_MAX_PAYL];

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
\brief  Initialize PDO memory

The function initializes the memory used to store PDO buffers.

\param  pPdoChannels            Pointer to PDO channel configuration.
\param  ppMem_p                 Pointer to store pointer to allocated memory.

\return The function returns a tEplKernel error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_initPdoMem(tPdoChannelSetup* pPdoChannels, BYTE** ppMem_p)
{
    tShbError       shbError;
    UINT            fCreated;
    UINT            memSize;

    memSize = setupPdoMemInfo(pPdoChannels);
    if (memSize != 0)
    {
        shbError = ShbLinAllocBuffer(memSize, PDO_SHB_BUF_ID, &shbInstance_l, &fCreated);
        if (shbError != kShbOk)
            return kEplNoFreeInstance;
    }

    // for shared buffers we have no pointer, for reads and writes the offset
    // in the shared buffer is used. Therefore we return 0 so that addresses
    // are equal to offsets.
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

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
void pdoucal_cleanupPdoMem(BYTE* pMem_p)
{
    tShbError       shbError;

    UNUSED_PARAMETER (pMem_p);

    shbError = ShbLinReleaseBuffer(shbInstance_l);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Releasing shared buffer failed (%d)\n",
                               __func__, shbError);
    }
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

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
BYTE *pdoucal_allocatePdoMem(BOOL fTxPdo_p, UINT channelId)
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
\brief  Get Address of PDO buffer

The function returns the address of the PDO buffer specified.

\param  fTxPdo_p                Determines if TXPDO or RXPDO buffer should be
                                allocated. Is TRUE for TXPDO.
\param  channelId               The PDO channel ID for which to allocate the
                                buffer.

\return Returns the address of the specified PDO buffer.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
BYTE *pdoucal_getPdoAdrs(BOOL fTxPdo_p, UINT channelId)
{
    UNUSED_PARAMETER(channelId);

    if (fTxPdo_p)
    {
        return aTxPdo_l;
    }
    else
    {
        return aRxPdo_l;
    }
}


//------------------------------------------------------------------------------
/**
\brief  Write TXPDO to PDO memory

The function writes a TXPDO to the PDO buffer.

\param  pPayload_p              Pointer to PDO payload. (Offset in shared buffer)
\param  pPdo_p                  Pointer to PDO data.
\param  pdoSize_p               Size of PDO to write.

\return The function returns a tEplKernel error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_setTxPdo(BYTE *pPayload_p, BYTE* pPdo_p,  WORD pdoSize_p)
{
    tShbError       shbError;
    ULONG           offset;

    // we need an offset in the shared buffer. As the base address was set
    // to 0 in PdokCal_AllocatePdoMem() the pointer is the same as the offset.
    offset = (ULONG)pPayload_p;

    shbError = ShbLinWriteDataBlock(shbInstance_l, offset, pPdo_p, pdoSize_p);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error writing to shared memory (%d)\n",
                             __func__, shbError);
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Read RXPDO from PDO memory

The function reads a RXPDO from the PDO buffer.

\param  ppPdo_p                 Pointer where to copy PDO data.
\param  pPayload_p              Pointer from where to read payload.
\param  pdoSize_p               Size of PDO.

\return The function returns a tEplKernel error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tEplKernel pdoucal_getRxPdo(BYTE** ppPdo_p, BYTE* pPayload_p, WORD pdoSize_p)
{
    tShbError       shbError;
    ULONG           offset;

    // we need an offset in the shared buffer. As the base address was set
    // to 0 in PdokCal_AllocatePdoMem() the pointer is the same as the offset.
    offset = (ULONG)pPayload_p;

    shbError = ShbLinReadDataBlock(shbInstance_l, aRxPdo_l, offset, pdoSize_p);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error reading from shared memory (%d)\n",
                             __func__, shbError);
    }

    *ppPdo_p = aRxPdo_l;

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

    offset = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        rxChannelSetup[channelId].channelOffset = offset;
        rxChannelSetup[channelId].channelSize = pPdoChannel->pdoSize;
        offset += pPdoChannel->pdoSize;
    }

    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        txChannelSetup[channelId].channelOffset = offset;
        txChannelSetup[channelId].channelSize = pPdoChannel->pdoSize;
        offset += pPdoChannel->pdoSize;
    }
    return offset;
}

///\}
