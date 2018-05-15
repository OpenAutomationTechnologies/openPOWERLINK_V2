/**
********************************************************************************
\file   pdoklut.c

\brief  Implementation of kernel PDO lookup table module

This file contains the implementation of the kernel PDO lookup table. It is
used for fast searching of PDO channels for a specific node.

\ingroup module_pdoklut
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <kernel/pdok.h>
#include <kernel/pdoklut.h>

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
\brief  Cleanup PDO lookup table

The function clears the PDO lookup table.

\param[in,out]  pLut_p              Pointer to the PDO lookup table.
\param[in]      numEntries_p        Number of entries in the PDO lookup table.

\ingroup module_pdoklut
**/
//------------------------------------------------------------------------------
void pdoklut_clear(tPdoklutEntry* pLut_p, size_t numEntries_p)
{
    size_t  i;
    size_t  j;

    // Check parameter validity
    ASSERT(pLut_p != NULL);

    for (i = 0; i < numEntries_p; ++i)
    {
        for (j = 0; j < PDOKLUT_MAX_CHANNELS_PER_NODE; ++j)
        {
            pLut_p[i].channelId[j] = PDOKLUT_INVALID_CHANNEL;
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Add a new PDO channel to the lookup table

This function adds a new PDO channel to the lookup table.

\param[in,out]  pLut_p              Pointer to the PDO lookup table
\param[in]      pPdoChannel_p       Pointer to the PDO channel which should be added to
                                    the lookup table.
\param[in]      channelId_p         Channel ID of the PDO channel to be added.

\return The function returns a tOplkError error code.

\ingroup module_pdoklut
**/
//------------------------------------------------------------------------------
tOplkError pdoklut_addChannel(tPdoklutEntry* pLut_p,
                              const tPdoChannel* pPdoChannel_p,
                              UINT8 channelId_p)
{
    tOplkError      ret = kErrorIllegalInstance;
    int             i;
    UINT8           nodeId;

    // Check parameter validity
    ASSERT(pLut_p != NULL);
    ASSERT(pPdoChannel_p != NULL);

    nodeId = pPdoChannel_p->nodeId;
    if (nodeId == 255)
        return ret;

    for (i = 0; i < PDOKLUT_MAX_CHANNELS_PER_NODE; ++i)
    {
        if (pLut_p[nodeId].channelId[i] == PDOKLUT_INVALID_CHANNEL)
        {
            DEBUG_LVL_PDO_TRACE ("Adding PDO Lut channel:%d node:%d index:%d\n", channelId_p, nodeId, i);
            pLut_p[nodeId].channelId[i] = channelId_p;
            ret = kErrorOk;
            break;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO channel from lookup table

The function gets the PDO channel with index \p searchIndex_p from the
PDO lookup table of node \p nodeId_p.

\param[in]      pLut_p              Pointer to the PDO lookup table.
\param[in]      index_p             The index of the entry to get from the lookup table.
\param[in]      nodeId_p            The node for which to get the channel.

\return The function returns the channel ID. If no more channel is found,
        PDOKLUT_INVALID_CHANNEL is returned.

\ingroup module_pdoklut
**/
//------------------------------------------------------------------------------
UINT8 pdoklut_getChannel(const tPdoklutEntry* pLut_p, UINT8 index_p, UINT8 nodeId_p)
{
    // Check parameter validity
    ASSERT(pLut_p != NULL);

    if (index_p >= PDOKLUT_MAX_CHANNELS_PER_NODE)
    {
        DEBUG_LVL_PDO_TRACE("%s() INVALID CHANNEL: index:%d nodeId:%d\n", __func__, index_p, nodeId_p);
        return PDOKLUT_INVALID_CHANNEL;
    }

    DEBUG_LVL_PDO_TRACE ("%s() channel:%d index:%d nodeId:%d\n",
            __func__, pLut_p[nodeId_p].channelId[index_p], index_p, nodeId_p);

   return pLut_p[nodeId_p].channelId[index_p];
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
