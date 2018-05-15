/**
********************************************************************************
\file   kernel/pdoklut.h

\brief  Include file for kernel PDO lookup table module

This file contains the definitions needed by the PDO lookup table module.
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
#ifndef _INC_kernel_pdoklut_H_
#define _INC_kernel_pdoklut_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/pdo.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// We can have up to 254 mapping entries per channel. If we map only bytes we
// require up to 1490 / 254 --> 6 PDO channels per frame.
#define PDOKLUT_MAX_CHANNELS_PER_NODE       6
#define PDOKLUT_INVALID_CHANNEL             0xff

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief PDO lookup table entry

The following structure defines a PDO lookup table entry.
*/
typedef struct
{
    UINT8               channelId[PDOKLUT_MAX_CHANNELS_PER_NODE];       ///< Array to store PDO channel IDs for the node.
} tPdoklutEntry;


//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

void       pdoklut_clear(tPdoklutEntry* pLut_p, size_t numEntries_p);
tOplkError pdoklut_addChannel(tPdoklutEntry* pLut_p,
                              const tPdoChannel* pPdoChannel_p,
                              UINT8 channelId_p);
UINT8      pdoklut_getChannel(const tPdoklutEntry* pLut_p,
                              UINT8 index_p,
                              UINT8 nodeId_p)
                              SECTION_PDOKLUT_GETCHANNEL;

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_pdoklut_H_ */
