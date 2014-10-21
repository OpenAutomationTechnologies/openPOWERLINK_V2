/**
********************************************************************************
\file   circbuf-noosdual.c

\brief  Circular buffer implementation for dual processor non-os systems

This file contains the architecture specific circular buffer functions
for systems running on dual processor without operating system
(e.g. microcontrollers, niosII, microblaze).

This implementation stores the circular buffer instances in a global variable
because there's no multitasking and this memory could be accessed from normal
execution context as well as from interrupt context. Locking is performed using
a shared memory byte for each circular buffer.

\ingroup module_lib_circbuf
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#include <oplk/oplkinc.h>
#include <common/target.h>

#include "circbuf-arch.h"
#include "dualprocshm.h"
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
typedef struct
{
    tDualprocDrvInstance    dualProcDrvInstance;    ///< Dual processor driver instance
    BOOL                    fAlloc;                 ///< Allocation flag for buffer
}tCircBufArchInstance;
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tCircBufInstance    instance_l[NR_OF_CIRC_BUFFERS];

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
    tCircBufInstance*       pInstance;
    tCircBufArchInstance*   pArch;
    pArch = OPLK_MALLOC(sizeof(tCircBufArchInstance));

    OPLK_MEMSET(pArch, 0, sizeof(tCircBufArchInstance));

    pInstance = &instance_l[id_p];

    pInstance->pCircBufArchInstance = pArch;
    pInstance->bufferId = id_p;

    return pInstance;
}

//------------------------------------------------------------------------------
/**
\brief  Free circular buffer instance

The function frees the allocated memory used by the circular buffer instance.

\param  pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeInstance(tCircBufInstance* pInstance_p)
{
    OPLK_FREE(pInstance_p->pCircBufArchInstance);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for circular buffer

The function allocates the memory needed for the circular buffer.

\param  pInstance_p         Pointer to the circular buffer instance.
\param  pSize_p             Size of memory to allocate.
                            Returns the actually allocated buffer size.

\return The function returns a tCircBuf Error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_allocBuffer(tCircBufInstance* pInstance_p, size_t* pSize_p)
{
    tDualprocReturn         ret;
    size_t                  size;
    tDualprocDrvInstance    dualProcDrvInst;
    UINT8*                  pBuffAddr;
    tCircBufArchInstance*   pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;
    size = *pSize_p + sizeof(tCircBufHeader);

    dualProcDrvInst = dualprocshm_getDrvInst(kDualProcFirst);

    if (dualProcDrvInst == NULL)
    {
        return kCircBufNoResource;
    }

    pArch->dualProcDrvInstance = dualProcDrvInst;
    pArch->fAlloc = TRUE;
    ret = dualprocshm_getMemory(pArch->dualProcDrvInstance, pInstance_p->bufferId, &pBuffAddr, &size, pArch->fAlloc);
    if (ret != kDualprocSuccessful)
    {
        TRACE("%s() Memory Not available error %X!\n", __func__, ret);
        return kCircBufNoResource;
    }

    pInstance_p->pCircBufHeader = (tCircBufHeader*)pBuffAddr;

    if (pInstance_p->pCircBufHeader == NULL)
    {
        return kCircBufNoResource;
    }

    pInstance_p->pCircBuf = ((UINT8*)pInstance_p->pCircBufHeader) + sizeof(tCircBufHeader);

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
    tDualprocReturn         ret;
    tCircBufArchInstance*   pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    ret = dualprocshm_freeMemory(pArch->dualProcDrvInstance, pInstance_p->bufferId, pArch->fAlloc);
    if (ret != kDualprocSuccessful)
    {
        TRACE("%s() unable to free Memory error %X!\n", __func__, ret);
    }

    pInstance_p->pCircBufHeader = NULL;
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
    tDualprocReturn         ret;
    size_t                  size;
    tDualprocDrvInstance    dualProcDrvInst;
    UINT8*                  pBuffAddr;
    tCircBufArchInstance*   pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    dualProcDrvInst = dualprocshm_getDrvInst(kDualProcSecond);

    if (dualProcDrvInst == NULL)
    {
        return kCircBufNoResource;
    }

    pArch->dualProcDrvInstance = dualProcDrvInst;
    pArch->fAlloc = FALSE;
    ret = dualprocshm_getMemory(pArch->dualProcDrvInstance, pInstance_p->bufferId, &pBuffAddr, &size, pArch->fAlloc);
    if (ret != kDualprocSuccessful)
    {
        TRACE("%s() Memory Not available error %X!\n", __func__, ret);
        return kCircBufNoResource;
    }

    pInstance_p->pCircBufHeader = (tCircBufHeader*)pBuffAddr;
    if (pInstance_p->pCircBufHeader == NULL)
    {
        return kCircBufNoResource;
    }

    pInstance_p->pCircBuf = ((UINT8*)pInstance_p->pCircBufHeader) + sizeof(tCircBufHeader);

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
    pInstance_p->pCircBufHeader = NULL;
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
    tDualprocReturn         ret;
    tCircBufArchInstance*   pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    ret = dualprocshm_acquireBuffLock(pArch->dualProcDrvInstance, pInstance_p->bufferId);
    if (ret != kDualprocSuccessful)
    {
        TRACE("%s() Failed to acquire lock for bufffer 0x%X error 0x%X!\n", \
              __func__, pInstance_p->bufferId, ret);
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
    tCircBufArchInstance*   pArch = (tCircBufArchInstance*)pInstance_p->pCircBufArchInstance;

    dualprocshm_releaseBuffLock(pArch->dualProcDrvInstance, pInstance_p->bufferId);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}
