/**
********************************************************************************
\file   linmem.c

\brief  This is the implementation of a linear memory buffer.

The linear memory buffer module enables a shared memory region for multiple
processes. Note that there is no data corruption protection provided.

\ingroup module_hostiflib
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
#include "linmem.h"

#include <stdlib.h>
#include <string.h>

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
#define ALIGN16(ptr)    (((UINT32)(ptr) + 1U) & 0xFFFFFFFEU)
#define ALIGN32(ptr)    (((UINT32)(ptr) + 3U) & 0xFFFFFFFCU)

#define UNALIGNED32(ptr)  ((UINT32)(ptr) & 3U)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Linear memory instance type

The linear memory instance type holds the buffer configuration.
*/
typedef struct sLim
{
    tLimConfig      config;   ///< local copy of the configuration
    UINT8           *pBase;   ///< base address of the linear memory instance
    UINT16          span;    ///< size of the linear memory instance
} tLim;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static void freePtr(void *p);
static void writeMemory (UINT8 *pBase_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 srcSpan_p);
static void readMemory (UINT8 *pBase_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 dstSpan_p);

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Create linear memory instance

The function initializes a linear memory instance.

\param  pConfig             The caller provides the configuration parameters.
\param  ppInstance_p        The function returns with that pointer the created
                            instance.

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_create (tLimConfig *pConfig, tLimInstance *ppInstance_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = NULL;

    if((pConfig->fAllocHeap == FALSE && pConfig->pBase == NULL) ||
            pConfig->span == 0)
    {
        Ret = kLimInvalidParameter;
        goto Exit;
    }

    pLim = (tLim*)malloc(sizeof(tLim));

    if(pLim == NULL)
    {
        Ret = kLimNoResource;
        goto Exit;
    }

    memset(pLim, 0, sizeof(tLim));

    pLim->config = *pConfig;
    pLim->span = pLim->config.span;

    if(pLim->config.fAllocHeap != FALSE)
    {
        pLim->pBase = (UINT8*)HOSTIF_UNCACHED_MALLOC(pLim->span);

        if(pLim->pBase == NULL)
        {
            Ret = kLimNoResource;
            goto Exit;
        }
    }
    else
    {
        pLim->pBase = HOSTIF_MAKE_NONCACHEABLE(pLim->config.pBase);
    }

    if(UNALIGNED32(pLim->pBase))
    {
        Ret = kLimAlignment;
        goto Exit;
    }

    memset(pLim->pBase, 0, pLim->span);

    *ppInstance_p = pLim;

Exit:
    if(Ret != kLimSuccessful)
    {
        lim_delete(pLim);
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete linear memory instance

The function deletes a linear memory instance.

\param  pInstance_p         The linear memory instance to be deleted

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_delete (tLimInstance pInstance_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = (tLim*)pInstance_p;

    if(pLim == NULL)
    {
        Ret = kLimInvalidInstance;
        goto Exit;
    }

    if(pLim->config.fAllocHeap != FALSE)
    {
        HOSTIF_UNCACHED_FREE(pLim->pBase);

        pLim->pBase = NULL;
    }

    freePtr(pLim);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get the base address of a linear memory instance

The function returns the base address of a created linear memory instance.
Note that this enables to directly access the linear buffer similar to use
lim_write and lim_read.

\param  pInstance_p         The linear memory instance of interest.
\param  ppBase_p            The function returns with this pointer the base
                            address of the linear memory instance.

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_getBase (tLimInstance pInstance_p, UINT8 **ppBase_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = (tLim*)pInstance_p;

    if(pLim == NULL)
    {
        Ret = kLimInvalidInstance;
        goto Exit;
    }

    *ppBase_p = pLim->pBase;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get the size of a linear memory instance

The function returns the size of a created linear memory instance.
Note that this enables to directly access the linear buffer similar to use
lim_write and lim_read.

\param  pInstance_p         The linear memory instance of interest.
\param  pSpan_p             The function returns with this pointer the size
                            of the linear memory instance.

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_getSpan (tLimInstance pInstance_p, UINT16 *pSpan_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = (tLim*)pInstance_p;

    if(pLim == NULL)
    {
        Ret = kLimInvalidInstance;
        goto Exit;
    }

    *pSpan_p = pLim->span;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write data to a linear memory instance

The function initiates a write to the specified linear memory instance.
Note that the source data size is checked before writing to the buffer instance.

\param  pInstance_p         The linear memory instance of interest.
\param  offset_p            The write destination byte-offset within the linear
                            buffer instance.
\param  pSrc_p              Pointer to the source data to be written.
\param  size_p              Size of the data to be written.

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_write (tLimInstance pInstance_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 size_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = (tLim*)pInstance_p;

    if(pLim == NULL)
    {
        Ret = kLimInvalidInstance;
        goto Exit;
    }

    if(pSrc_p == NULL || size_p == 0)
    {
        Ret = kLimInvalidParameter;
        goto Exit;
    }

    if(UNALIGNED32(offset_p) || UNALIGNED32(pSrc_p) || UNALIGNED32(size_p))
    {
        Ret = kLimAlignment;
        goto Exit;
    }

    if(offset_p + size_p > pLim->span)
    {
        Ret = kLimOverflow;
        goto Exit;
    }

    writeMemory(pLim->pBase, offset_p, pSrc_p, size_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read data from a linear memory instance

The function initiates a read from the specified linear memory instance.
Note that the destination data size is checked before reading from the buffer
instance.

\param  pInstance_p         The linear memory instance of interest.
\param  offset_p            The read destination byte-offset within the linear
                            buffer instance.
\param  pDst_p              Pointer to the destination data buffer.
\param  size_p              Size of the data to be read.

\return The function returns an error code.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tLimReturn lim_read (tLimInstance pInstance_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 size_p)
{
    tLimReturn Ret = kLimSuccessful;
    tLim *pLim = (tLim*)pInstance_p;

    if(pLim == NULL)
    {
        Ret = kLimInvalidInstance;
        goto Exit;
    }

    if(pDst_p == NULL || size_p == 0)
    {
        Ret = kLimInvalidParameter;
        goto Exit;
    }

    if(UNALIGNED32(offset_p) || UNALIGNED32(pDst_p) || UNALIGNED32(size_p))
    {
        Ret = kLimAlignment;
        goto Exit;
    }

    if(offset_p + size_p > pLim->span)
    {
        Ret = kLimOverflow;
        goto Exit;
    }

    readMemory(pLim->pBase, offset_p, pDst_p, size_p);

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Free pointers which are not NULL

\param  p                       Pointer to be freed
*/
//------------------------------------------------------------------------------
static void freePtr(void *p)
{
    if(p != NULL)
        free(p);
}

//------------------------------------------------------------------------------
/**
\brief    Write data to memory

This function writes data from a source to memory.

\param  pBase_p                 Base address of memory writing to
\param  offset_p                Offset where to start writing to
\param  pSrc_p                  Source data base address
\param  srcSpan_p               Source data size
*/
//------------------------------------------------------------------------------
static void writeMemory (UINT8 *pBase_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 srcSpan_p)
{
    memcpy(pBase_p + offset_p, pSrc_p, srcSpan_p);
}

//------------------------------------------------------------------------------
/**
\brief    Read data from memory

This function reads data from memory.

\param  pBase_p                 Base address of memory reading from
\param  offset_p                Offset where to start reading from
\param  pDst_p                  Destination data base address
\param  dstSpan_p               Destination data size
*/
//------------------------------------------------------------------------------
static void readMemory (UINT8 *pBase_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 dstSpan_p)
{
    memcpy(pDst_p, pBase_p + offset_p, dstSpan_p);
}
