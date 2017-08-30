/**
********************************************************************************
\file   bufalloc.c

\brief  Buffer allocation library

This file contains the implementation of a buffer allocation library. The
interface of the buffer allocation library is defined in bufalloc.h.


\ingroup module_lib_bufalloc
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

\param  maxBuffer_p     The maximum amount of buffers

\return The function returns a pointer to the buffer instance.
\retval 0           Buffer allocation instance initializing failed.
\retval other       Buffer allocation instance successfully initialized.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tBufAlloc* bufalloc_init(UINT maxBuffer_p)
{
    tBufAlloc*          pBufAlloc;
    tBufData*           pBufData;

    if (!maxBuffer_p)
    {
        pBufAlloc = NULL;
        goto Exit;
    }
    if ((pBufAlloc = OPLK_MALLOC(sizeof(*pBufAlloc))) == NULL)
    {
        pBufAlloc = NULL;
        goto Exit;
    }
    if ((pBufData = OPLK_MALLOC(sizeof(*pBufData) * maxBuffer_p)) == NULL)
    {
        OPLK_FREE(pBufAlloc);
        pBufAlloc = NULL;
        goto Exit;
    }

    pBufAlloc->maxSize = maxBuffer_p;
    pBufAlloc->pBufData = pBufData;
    pBufAlloc->pBufDataBegin = (void*)pBufData;
    pBufAlloc->allocatedBufCnt = 0;
    pBufAlloc->releasedBufCnt = 0;
    strncpy(pBufAlloc->checkId, BUFALLOC_CHECKID, 9);

Exit:
    return pBufAlloc;
}

//------------------------------------------------------------------------------
/**
\brief  Exit buffer allocation

The function deletes the buffer allocation instance.

\param  pBufAlloc_p     Pointer to buffer allocation instance.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Buffer allocation instance successfully deleted.
\retval kErrorGeneralError          Buffer allocation instance deletion failed.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_exit(tBufAlloc* pBufAlloc_p)
{
    tOplkError          ret = kErrorGeneralError;
    tBufData*           pBufDataBegin = NULL;

    if (pBufAlloc_p == NULL)
    {
        ret = kErrorGeneralError;
        goto Exit;
    }
    // Check if pBufAlloc is a valid buffer allocation instance.
    if (strncmp(pBufAlloc_p->checkId, BUFALLOC_CHECKID, 9) == 0)
    {
        pBufDataBegin = pBufAlloc_p->pBufDataBegin;
        if (pBufDataBegin != NULL)
        {
            OPLK_FREE(pBufDataBegin);
            ret = kErrorOk;
        }
        OPLK_FREE(pBufAlloc_p);
    }
Exit:
    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Add a free buffer

The function adds a buffer to the buffer allocation instance.

\param  pBufAlloc_p     Pointer to buffer allocation instance.
\param  pFreeBuf_p      Pointer to buffer.
\param  bufferNumber_p  Index number of the allocated buffer.

\return The function returns a tOplkError error code.
\retval kErrorGeneralError          Not enough buffer storage in allocation instance.
\retval kErrorOk                    Buffer successfully registered.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_addBuffer(tBufAlloc* pBufAlloc_p, void* pFreeBuf_p, UINT bufferNumber_p)
{
    return bufalloc_releaseBuffer(pBufAlloc_p, pFreeBuf_p, bufferNumber_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get a free buffer pointer

The function gets a buffer from the buffer allocation instance and removes
it from the instance.

\param  pBufAlloc_p     Pointer to buffer allocation instance.

\return The function returns a BufData pointer.
\retval 0           No free buffer returned from buffer allocation instance.
\retval other       Successfully returned a free buffer.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tBufData* bufalloc_getBuffer(tBufAlloc* pBufAlloc_p)
{
    tBufData*           pBuffer = NULL;

    if (pBufAlloc_p == NULL)
    {
        pBuffer = NULL;
        goto Exit;
    }
    if ((pBufAlloc_p->releasedBufCnt - pBufAlloc_p->allocatedBufCnt) > 0)
    {
        pBufAlloc_p->pBufData--;
        pBufAlloc_p->allocatedBufCnt++;
        pBuffer = pBufAlloc_p->pBufData;
    }
Exit:
    return pBuffer;
}

//------------------------------------------------------------------------------
/**
\brief  Release an allocated buffer

The function releases an allocated buffer to the buffer allocation instance.

\param  pBufAlloc_p     Pointer to buffer allocation instance.
\param  pFreeBuf_p      Pointer to buffer.
\param  bufferNumber_p  Index number of the allocated buffer.

\return The function returns a tOplkError error code.
\retval kErrorGeneralError          Not enough buffer storage in allocation instance.
\retval kErrorOk                    Buffer successfully registered.

\ingroup module_lib_bufalloc
*/
//------------------------------------------------------------------------------
tOplkError bufalloc_releaseBuffer(tBufAlloc* pBufAlloc_p, void* pFreeBuf_p, UINT bufferNumber_p)
{
    tOplkError          ret = kErrorGeneralError;
    UINT                bufCnt = 0;

    if (pBufAlloc_p == NULL)
    {
        ret = kErrorGeneralError;
        goto Exit;
    }
    if (pFreeBuf_p == NULL)
    {
        ret = kErrorGeneralError;
        goto Exit;
    }

    bufCnt = pBufAlloc_p->releasedBufCnt - pBufAlloc_p->allocatedBufCnt;
    if (bufCnt < pBufAlloc_p->maxSize)
    {
        pBufAlloc_p->pBufData->pBuffer = pFreeBuf_p;
        pBufAlloc_p->pBufData->bufferNumber = bufferNumber_p;
        pBufAlloc_p->pBufData++;
        pBufAlloc_p->releasedBufCnt++;
        ret = kErrorOk;
    }
Exit:
    return ret;
}
//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
