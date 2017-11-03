/**
********************************************************************************
\file   circbuf/circbuf-linuxkernel.c

\brief  Circular buffer implementation for Linux Kernel

This file contains the architecture specific circular buffer functions for the
Linux kernel implementation. This implementation does not use shared memory as
it assumes all accesses to the circular buffer are made from kernel context.

User processes have to access the circular buffer by ioctl calls.
The locking is performed by spinlocks.

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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>

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
    spinlock_t          spinlock;       ///< spinlock used for locking
    unsigned long       flags;          ///< IRQ flags
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

    UNUSED_PARAMETER(fNew_p);

    pInstance = (tCircBufInstance*)OPLK_MALLOC(sizeof(tCircBufInstance) +
                                               sizeof(tCircBufArchInstance));
    if (pInstance == NULL)
        return NULL;

    OPLK_MEMSET(pInstance, 0, sizeof(tCircBufInstance) + sizeof(tCircBufArchInstance));
    pInstance->pCircBufArchInstance = (UINT8*)pInstance + sizeof(tCircBufInstance);
    pInstance->bufferId = id_p;

    pArch = (tCircBufArchInstance*)pInstance->pCircBufArchInstance;
    spin_lock_init(&pArch->spinlock);

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
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

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
    // Check parameter validity
    ASSERT(pInstance_p != NULL);
    ASSERT(pSize_p != NULL);

    pInstance_p->pCircBufHeader = (tCircBufHeader*)OPLK_MALLOC(sizeof(tCircBufHeader));
    if (pInstance_p->pCircBufHeader == NULL)
        return kCircBufNoResource;

    pInstance_p->pCircBuf = OPLK_MALLOC(*pSize_p);
    if (pInstance_p->pCircBuf == NULL)
    {
        OPLK_FREE(pInstance_p->pCircBufHeader);
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
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    OPLK_FREE(pInstance_p->pCircBuf);
    OPLK_FREE(pInstance_p->pCircBufHeader);
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
    UNUSED_PARAMETER(pInstance_p);

    return kCircBufNoResource;
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
    UNUSED_PARAMETER(pInstance_p);
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
    tCircBufArchInstance* pArchInstance;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pArchInstance = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    spin_lock_irqsave(&pArchInstance->spinlock, pArchInstance->flags);
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
    tCircBufArchInstance* pArchInstance;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pArchInstance = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    spin_unlock_irqrestore(&pArchInstance->spinlock, pArchInstance->flags);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
