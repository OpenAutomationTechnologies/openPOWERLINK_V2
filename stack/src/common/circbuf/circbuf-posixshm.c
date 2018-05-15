/**
********************************************************************************
\file   circbuf/circbuf-posixshm.c

\brief  Circular buffer implementation using Posix shared memory

This file contains the architecture specific circular buffer functions
using posix shared memory and BSD semaphores for locking.

\ingroup module_lib_circbuf
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

#include "circbuf-arch.h"

#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <semaphore.h>
#include <errno.h>


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

/** \brief Architecture specific part of circular buffer instance */
typedef struct
{
    int                 fd;             ///< Shared memory file descriptor
    sem_t*              lockSem;        ///< Semaphore used for locking
} tCircBufArchInstance;

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
\brief  Create circular buffer instance

The function allocates the memory needed for the circular buffer instance.

\param[in]      id_p                ID of the circular buffer.
\param[in]      fNew_p              The parameter determines if a new circular buffer
                                    instance should be created (TRUE) or if it should
                                    connect to an existing instance (FALSE).

\return The function returns the pointer to the buffer instance or NULL on error.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufInstance* circbuf_createInstance(UINT8 id_p, BOOL fNew_p)
{
    tCircBufInstance*           pInstance;
    tCircBufArchInstance*       pArch;
    char                        semName[16];

    pInstance = (tCircBufInstance*)OPLK_MALLOC(sizeof(tCircBufInstance) +
                                               sizeof(tCircBufArchInstance));
    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() malloc failed!\n", __func__);
        return NULL;
    }

    OPLK_MEMSET(pInstance, 0, sizeof(tCircBufInstance) + sizeof(tCircBufArchInstance));
    pInstance->pCircBufArchInstance = (UINT8*)pInstance + sizeof(tCircBufInstance);
    pInstance->bufferId = id_p;

    pArch = (tCircBufArchInstance*)pInstance->pCircBufArchInstance;

    sprintf(semName, "/semCircbuf-%d", id_p);

    if (fNew_p)
    {
        sem_unlink(semName);
    }

    if ((pArch->lockSem = sem_open(semName, O_CREAT, S_IRWXG, 1)) == SEM_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() open sem failed!\n", __func__);
        OPLK_FREE(pInstance);
        return NULL;
    }

    return pInstance;
}

//------------------------------------------------------------------------------
/**
\brief  Free circular buffer instance

The function frees the allocated memory used by the circular buffer instance.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeInstance(tCircBufInstance* pInstance_p)
{
    tCircBufArchInstance* pArch;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    sem_close(pArch->lockSem);
    OPLK_FREE(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for circular buffer

The function allocates the memory needed for the circular buffer.

\param[in]      pInstance_p         Pointer to the circular buffer instance.
\param[in,out]  pSize_p             Size of memory to allocate.
                                    Returns the actually allocated buffer size.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_allocBuffer(tCircBufInstance* pInstance_p, size_t* pSize_p)
{
    char                        shmName[16];
    size_t                      size;
    tCircBufArchInstance*       pArch;
    size_t                      pageSize;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);
    ASSERT(pSize_p != NULL);

    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    sprintf(shmName, "/shmCircbuf-%d", pInstance_p->bufferId);
    pageSize = (sizeof(tCircBufHeader) + (size_t)sysconf(_SC_PAGE_SIZE) - 1) & (~((size_t)sysconf(_SC_PAGE_SIZE) - 1));
    size = *pSize_p + pageSize;

    if ((pArch->fd = shm_open(shmName, O_RDWR | O_CREAT, 0)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() shm_open failed!\n", __func__);
        return kCircBufNoResource;
    }

    if (ftruncate(pArch->fd, size) == -1)
    {
        DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
        close(pArch->fd);
        shm_unlink(shmName);
        return kCircBufNoResource;
    }

    pInstance_p->pCircBufHeader = mmap(NULL, sizeof(tCircBufHeader),
                                       PROT_READ | PROT_WRITE, MAP_SHARED, pArch->fd, 0);
    if (pInstance_p->pCircBufHeader == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap header failed!\n", __func__);
        close(pArch->fd);
        shm_unlink(shmName);
        return kCircBufNoResource;
    }

    pInstance_p->pCircBuf = mmap(NULL, *pSize_p, PROT_READ | PROT_WRITE, MAP_SHARED,
                                 pArch->fd, pageSize);
    if (pInstance_p->pCircBuf == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap buffer failed! (%s)\n", __func__, strerror(errno));
        munmap(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));
        close(pArch->fd);
        shm_unlink(shmName);
        return kCircBufNoResource;
    }

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free memory used by circular buffer

The function frees the allocated memory used by the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeBuffer(tCircBufInstance* pInstance_p)
{
    char                        shmName[16];
    tCircBufArchInstance*       pArch;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    sprintf (shmName, "/shmCircbuf-%d", pInstance_p->bufferId);

    munmap(pInstance_p->pCircBuf, pInstance_p->pCircBufHeader->bufferSize);
    munmap(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));
    close(pArch->fd);
    shm_unlink(shmName);
}

//------------------------------------------------------------------------------
/**
\brief  Connect to circular buffer

The function connects the calling thread to the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_connectBuffer(tCircBufInstance* pInstance_p)
{
    char                        shmName[16];
    size_t                      size;
    tCircBufArchInstance*       pArch;
    size_t                      pageSize;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pageSize = (size_t)sysconf(_SC_PAGE_SIZE);
    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    sprintf(shmName, "/shmCircbuf-%d", pInstance_p->bufferId);
    if ((pArch->fd = shm_open(shmName, O_RDWR, 0)) < 0)
    {
        return kCircBufNoResource;
    }

    pInstance_p->pCircBufHeader = mmap(NULL, sizeof(tCircBufHeader),
                                       PROT_READ | PROT_WRITE, MAP_SHARED, pArch->fd, 0);
    if (pInstance_p->pCircBufHeader == MAP_FAILED)
    {
        close(pArch->fd);
        return kCircBufNoResource;
    }

    size = (size_t)pInstance_p->pCircBufHeader->bufferSize;
    pInstance_p->pCircBuf = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                                 pArch->fd, pageSize);
    if (pInstance_p->pCircBuf == MAP_FAILED)
    {
        munmap(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));
        close(pArch->fd);
        return kCircBufNoResource;
    }

     return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from circular buffer

The function disconnects the calling thread from the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_disconnectBuffer(tCircBufInstance* pInstance_p)
{
    tCircBufArchInstance*       pArch;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    munmap(pInstance_p->pCircBuf, pInstance_p->pCircBufHeader->bufferSize);
    munmap(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));
    close(pArch->fd);
}

//------------------------------------------------------------------------------
/**
\brief  Lock circular buffer

The function enters a locked section of the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_lock(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    tCircBufArchInstance* pArchInstance =
                              (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    sem_wait(pArchInstance->lockSem);
}

//------------------------------------------------------------------------------
/**
\brief  Unlock circular buffer

The function leaves a locked section of the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_unlock(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    tCircBufArchInstance* pArchInstance =
                              (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    sem_post(pArchInstance->lockSem);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
