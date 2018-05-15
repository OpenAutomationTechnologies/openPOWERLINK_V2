/**
********************************************************************************
\file   bufalloc.c

\brief  Buffer allocation library

This file contains the implementation of a buffer allocation library. The
interface of the buffer allocation library is defined in bufalloc.h.


\ingroup module_lib_bufalloc
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

/**
********************************************************************************

\defgroup   module_lib_bufalloc    Buffer Allocation Library
\ingroup    libraries

The buffer allocation library is designed to quickly access free buffers.
Therefore, a stack-based buffer allocation library is implemented.

An instance of the buffer allocation is created by calling bufalloc_init().
Then, the instance is filled by passing a buffer to bufalloc_addBuffer().
By calling bufalloc_getBuffer() a pointer to a free buffer is returned for
further usage. Releasing a free buffer to the buffer allocation instance is
done by passing a buffer to the bufalloc_releaseBuffer() function.

Calling bufalloc_exit() frees all stored buffers from the buffer allocation.

*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/bufalloc.h>

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
#define BUFALLOC_CHECKID    "bufaloc"       // Incl. trailing '0' -> 8 chars

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
\brief  Initialize buffer allocation

The function initializes a buffer allocation instance.

\param[in]      maxBuffer_p         The maximum amount of buffers

\return The function returns a pointer to the buffer instance.
\retval NULL                        Buffer allocation instance initializing failed.
\retval other                       Buffer allocation instance successfully initialized.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tBufAlloc* bufalloc_init(UINT maxBuffer_p)
{
    tBufAlloc*          pBufAllocInstance;
    tBufData*           pBufData;

    // Check buffer number
    if (maxBuffer_p == 0)
        return NULL;

    // Allocate buffer allocation instance
    pBufAllocInstance = (tBufAlloc*)OPLK_MALLOC(sizeof(tBufAlloc));
    if (pBufAllocInstance == NULL)
        return NULL;

    // initialize instance
    OPLK_MEMSET(pBufAllocInstance, 0, sizeof(tBufAlloc));
    OPLK_MEMCPY(pBufAllocInstance->checkId, BUFALLOC_CHECKID, sizeof(BUFALLOC_CHECKID));
    pBufAllocInstance->maxSize = maxBuffer_p;

    // Allocate buffer data instances
    pBufData = (tBufData*)OPLK_MALLOC(sizeof(tBufData) * maxBuffer_p);
    if (pBufData == NULL)
    {
        OPLK_FREE(pBufAllocInstance);
        return NULL;
    }

    // Fill buffer allocation instance
    pBufAllocInstance->pBufData = pBufData;
    pBufAllocInstance->pBufDataBegin = pBufData;

    return pBufAllocInstance;
}

//------------------------------------------------------------------------------
/**
\brief  Exit buffer allocation

The function deletes the buffer allocation instance.

\param[in]      pBufAlloc_p         Pointer to the buffer allocation instance.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
void bufalloc_exit(tBufAlloc* pBufAlloc_p)
{
    // Check if pBufAlloc_p is a valid buffer allocation instance
    if (pBufAlloc_p != NULL)
    {
        ASSERT(!OPLK_MEMCMP(pBufAlloc_p->checkId, BUFALLOC_CHECKID, sizeof(BUFALLOC_CHECKID)));

        // Check and free the buffer data instances
        ASSERT(pBufAlloc_p->pBufDataBegin != NULL);
        OPLK_FREE(pBufAlloc_p->pBufDataBegin);

        // Free the buffer allocation instance
        OPLK_FREE(pBufAlloc_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Add a free buffer

The function adds a buffer to the buffer allocation instance.

\param[in]      pBufAlloc_p         Pointer to the buffer allocation instance.
\param[in]      pBufData_p          Pointer to the buffer data structure that
                                    is added to the buffer pool.

\return The function returns a tOplkError error code.
\retval kErrorGeneralError          Not enough buffer storage in allocation instance.
\retval kErrorOk                    Buffer successfully registered.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_addBuffer(tBufAlloc* pBufAlloc_p, const tBufData* pBufData_p)
{
    return bufalloc_releaseBuffer(pBufAlloc_p, pBufData_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get a free buffer pointer

The function gets a buffer from the buffer allocation instance and removes
it from the instance.

\param[in]      pBufAlloc_p         Pointer to the buffer allocation instance.
\param[out]     pBufData_p          Pointer to store the buffer data structure
                                    that is taken from the buffer pool.

\return The function returns a tOplkError error code.
\retval kErrorGeneralError          No free buffer available in allocation instance.
\retval kErrorOk                    Buffer successfully returned.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_getBuffer(tBufAlloc* pBufAlloc_p, tBufData* pBufData_p)
{
    tOplkError          ret = kErrorGeneralError;
    UINT                availBufCnt;

    // Check if pBufAlloc_p is a valid buffer allocation instance
    ASSERT(pBufAlloc_p != NULL);
    ASSERT(!OPLK_MEMCMP(pBufAlloc_p->checkId, BUFALLOC_CHECKID, sizeof(BUFALLOC_CHECKID)));

    // Check if output parameter pointers are valid
    ASSERT(pBufData_p != NULL);

    availBufCnt = pBufAlloc_p->releasedBufCnt - pBufAlloc_p->allocatedBufCnt;
    if (availBufCnt > 0)
    {
        pBufAlloc_p->pBufData--;
        pBufAlloc_p->allocatedBufCnt++;

        *pBufData_p = *(pBufAlloc_p->pBufData);

        ret = kErrorOk;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Release an allocated buffer

The function releases an allocated buffer to the buffer allocation instance.

\param[in,out]  pBufAlloc_p         Pointer to the buffer allocation instance.
\param[in]      pBufData_p          Pointer to the buffer data structure that
                                    is given back to the buffer pool.

\return The function returns a tOplkError error code.
\retval kErrorGeneralError          Not enough buffer storage in allocation instance.
\retval kErrorOk                    Buffer successfully registered.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_releaseBuffer(tBufAlloc* pBufAlloc_p, const tBufData* pBufData_p)
{
    tOplkError          ret = kErrorGeneralError;
    UINT                availBufCnt;

    // Check if pBufAlloc_p is a valid buffer allocation instance
    ASSERT(pBufAlloc_p != NULL);
    ASSERT(!OPLK_MEMCMP(pBufAlloc_p->checkId, BUFALLOC_CHECKID, sizeof(BUFALLOC_CHECKID)));

    // Check if pBufData_p is a valid buffer data structure (i.e. not NULL)
    ASSERT(pBufData_p != NULL);

    availBufCnt = pBufAlloc_p->releasedBufCnt - pBufAlloc_p->allocatedBufCnt;
    if (availBufCnt < pBufAlloc_p->maxSize)
    {
        *(pBufAlloc_p->pBufData) = *pBufData_p;

        pBufAlloc_p->pBufData++;
        pBufAlloc_p->releasedBufCnt++;

        ret = kErrorOk;
    }

    return ret;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
