/**
********************************************************************************
\file   circbuf-win32.c

\brief  Circular buffer implementation for Windows

This file contains the architecture specific circular buffer functions for
Windows.

__NOTE__: This implementation requires that the circular buffer library
is accessed only within one single process! Therefore normal malloc() could
be used instead of special shared memory functions.

\ingroup module_lib_circbuf
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include "circbuf-arch.h"

#include <Windows.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMEOUT_ENTER_ATOMIC    1000

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
/** \brief Architecture specific part of circular buffer instance */
typedef struct
{
    HANDLE              lockMutex;      ///< Mutex used for locking
} tCircBufArchInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tCircBufHeader*         pHeader_l[NR_OF_CIRC_BUFFERS];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create circular buffer instance

The function allocates the memory needed for the circular buffer instance.

\param  id_p                ID of the circular buffer.

\return The function returns the pointer to the buffer instance or NULL on error.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufInstance* circbuf_createInstance(UINT8 id_p)
{
    tCircBufInstance*           pInstance;
    tCircBufArchInstance*       pArch;
    TCHAR                       mutexName[MAX_PATH];

    if ((pInstance = EPL_MALLOC(sizeof(tCircBufInstance) +
                                sizeof(tCircBufArchInstance))) == NULL)
    {
        TRACE("%s() malloc failed!\n", __func__);
        return NULL;
    }
    EPL_MEMSET(pInstance, 0, sizeof(tCircBufInstance) + sizeof(tCircBufArchInstance));
    pInstance->pCircBufArchInstance = (BYTE*)pInstance + sizeof(tCircBufInstance);
    pInstance->bufferId = id_p;

    pArch = (tCircBufArchInstance*)pInstance->pCircBufArchInstance;

    sprintf(mutexName, "Local\\circbufMutex%d", id_p);
    if ((pArch->lockMutex = CreateMutex (NULL, FALSE, mutexName)) == NULL)
    {
        TRACE("%s() creating mutex failed!\n", __func__);
        EPL_FREE(pInstance);
        return NULL;
    }

    return pInstance;
}

//------------------------------------------------------------------------------
/**
\brief  Free circular buffer instance

The function frees the allocated memory used by the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeInstance(tCircBufInstance* pInstance_p)
{
    tCircBufArchInstance* pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    CloseHandle(pArch->lockMutex);
    EPL_FREE(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for circular buffer

The function allocates the memory needed for the circular buffer.

\param  pInstance_p         Pointer to the circular buffer instance.
\param  size_p              Size of memory to allocate.

\return The function returns a tCircBuf Error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_allocBuffer(tCircBufInstance* pInstance_p, size_t size_p)
{
    size_t                      size;
    tCircBufArchInstance*       pArch;

    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    size = size_p + sizeof(tCircBufHeader);

    pInstance_p->pCircBufHeader = EPL_MALLOC(size);
    if (pInstance_p->pCircBufHeader == NULL)
    {
        TRACE("%s() malloc failed!\n", __func__);
        return kCircBufNoResource;
    }

    pInstance_p->pCircBuf = ((BYTE*)pInstance_p->pCircBufHeader) + sizeof(tCircBufHeader);

    /* save for other threads - shared memory */
    pHeader_l[pInstance_p->bufferId] = pInstance_p->pCircBufHeader;

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free memory used by circular buffer

The function frees the allocated memory used by the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeBuffer(tCircBufInstance* pInstance_p)
{
    EPL_FREE(pInstance_p->pCircBufHeader);
}

//------------------------------------------------------------------------------
/**
\brief  Connect to circular buffer

The function connects the calling thread to the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\return The function returns a tCircBuf Error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_connectBuffer(tCircBufInstance* pInstance_p)
{
    /* read from "shared memory" */
    pInstance_p->pCircBufHeader = pHeader_l[pInstance_p->bufferId];
    pInstance_p->pCircBuf = ((BYTE*)pInstance_p->pCircBufHeader) + sizeof(tCircBufHeader);

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from circular buffer

The function disconnects the calling thread from the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_disconnectBuffer(tCircBufInstance* pInstance_p)
{
    UNUSED_PARAMETER(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Lock circular buffer

The function enters a locked section of the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_lock(tCircBufInstance* pInstance_p)
{
    DWORD                   waitResult;
    tCircBufArchInstance*   pArchInstance =
                      (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    waitResult = WaitForSingleObject (pArchInstance->lockMutex, INFINITE);
    switch (waitResult)
    {
        case WAIT_OBJECT_0:
            break;
        default:
            TRACE("%s() Mutex wait unknown error! Error:%ld\n", __func__,
                  GetLastError());
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Unlock circular buffer

The function leaves a locked section of the circular buffer.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_unlock(tCircBufInstance* pInstance_p)
{
    tCircBufArchInstance* pArchInstance =
                      (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    ReleaseMutex (pArchInstance->lockMutex);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}







