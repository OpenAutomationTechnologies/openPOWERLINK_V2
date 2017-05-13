/**
********************************************************************************
\file   dualprocshm-pcp.c

\brief  Dual processor library - PCP on external PCIe card

This file provides specific function definition for PCP running on a external
PCIe cards.

\ingroup module_dualprocshm
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <dualprocshm-target.h>
#include <trace/trace.h>

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

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get common memory address

Target specific routine to retrieve the base address of common memory between
two processors.

\param[in,out]  pSize_p             Minimum size of the common memory, returns the
                                    actual size of common memory.

\return Pointer to base address of common memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void* dualprocshm_getCommonMemAddr(UINT16* pSize_p)
{
    void*   pAddr;

    if (*pSize_p > MAX_COMMON_MEM_SIZE )
    {
        TRACE("%s(): Common memory not available\n", __func__);
        return NULL;
    }

    pAddr = DPSHM_MAKE_NONCACHEABLE(COMMON_MEM_BASE);

    *pSize_p = MAX_COMMON_MEM_SIZE - 1;

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free common memory address

Target specific routine to release the common memory.

\param[in]      pSize_p             Size of the common memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseCommonMemAddr(UINT16 pSize_p)
{
    UNUSED_PARAMETER(pSize_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get shared memory base address and size

Target specific routine to retrieve the base address of shared memory region
between two processors.

\param[in,out]  pSize_p             Minimum size of the shared memory, returns the
                                    actual size of shared memory.

\return Pointer to base address of shared memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void* dualprocshm_getSharedMemInst(UINT32* pSize_p)
{
    void*   pAddr;

    if (*pSize_p > SHARED_MEM_SPAN)
    {
        TRACE("%s(): Shared memory not available\n", __func__);
        *pSize_p = 0;
        return NULL;
    }

    pAddr = DPSHM_MAKE_NONCACHEABLE(SHARED_MEM_BASE);

    *pSize_p = SHARED_MEM_SPAN;

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Get dynamic mapping table base address

Target specific routine to retrieve the base address for storing the
dynamic mapping table.

\return Pointer to base address of dynamic mapping table.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void* dualprocshm_getDynMapTableAddr(void)
{
    void*   pAddr;

    pAddr = DPSHM_MAKE_NONCACHEABLE(MEM_ADDR_TABLE_BASE);

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free dynamic mapping table base address

Target specific routine to free the base address used for storing the
dynamic mapping table.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseDynMapTableAddr(void)
{

}

//------------------------------------------------------------------------------
/**
\brief  Get interrupt synchronization base address

Target specific routine to retrieve the base address for storing
interrupt synchronization registers.

\return Pointer to base address of interrupt synchronization registers.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void* dualprocshm_getIntrMemAddr(void)
{
    void*   pAddr;

    pAddr = DPSHM_MAKE_NONCACHEABLE(MEM_INTR_BASE);

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free interrupt synchronization base address

Target specific routine to free the base address used for storing
interrupt synchronization registers.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseIntrMemAddr()
{
    // nothing to be done on PCIe
}

//------------------------------------------------------------------------------
/**
\brief  Read data from memory

Target specific memory read routine.

\param[in]      pBase_p             Address to read data from.
\param[in]      size_p              Number of bytes to be read.
\param[out]     pData_p             Pointer to store the read data.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReadData(const void* pBase_p,
                                size_t size_p,
                                void* pData_p)
{
    if ((pBase_p == NULL) ||
        (pData_p == NULL))
    {
        TRACE("%s(): Invalid parameters\n", __func__);
        return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE((UINT32)pBase_p, size_p);

    DUALPROCSHM_MEMCPY(pData_p, pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write data to memory

Target specific routine used to write data to the specified memory address.

\param[out]     pBase_p             Address to write data to.
\param[in]      size_p              Number of bytes to be written.
\param[in]      pData_p             Pointer to memory containing data to be written.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetWriteData(void* pBase_p,
                                 size_t size_p,
                                 const void* pData_p)
{
    if ((pBase_p == NULL) ||
        (pData_p == NULL))
    {
        TRACE("%s(): Invalid parameters\n", __func__);
        return;
    }

    DUALPROCSHM_MEMCPY(pBase_p, pData_p, size_p);

    DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32)pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(acquire)

This routine implements a target specific locking mechanism using the shared
memory between two processors/processes. The caller needs to pass the base
address and processor instance of the calling processor.

The locking is achieved using Peterson's algorithm
\ref https://en.wikipedia.org/wiki/Peterson's_algorithm

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetAcquireLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    tDualprocLock*      pLock = pBase_p;
    tDualProcInstance   otherProcInstance;

    if (pLock == NULL)
        return;

    switch (procInstance_p)
    {
        case kDualProcFirst:
            otherProcInstance = kDualProcSecond;
            break;

        case kDualProcSecond:
            otherProcInstance = kDualProcFirst;
            break;

        default:
            TRACE("Invalid processor instance\n");
            return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pLock, sizeof(tDualprocLock));

    DPSHM_WRITE8(&pLock->afFlag[procInstance_p], 1);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->afFlag[procInstance_p],
                                   sizeof(pLock->afFlag[procInstance_p]));

    DPSHM_WRITE8(&pLock->turn, otherProcInstance);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->turn, sizeof(pLock->turn));

    DPSHM_DMB();

    do
    {
        DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pLock, sizeof(tDualprocLock));
    } while (DPSHM_READ8(&pLock->afFlag[otherProcInstance]) &&
             (DPSHM_READ8(&pLock->turn) == otherProcInstance));
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory unlock routine (release)

This routine is used to release a lock acquired at a specified address.

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReleaseLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    tDualprocLock*  pLock = pBase_p;

    if (pLock == NULL)
        return;

    DPSHM_WRITE8(&pLock->afFlag[procInstance_p], 0);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->afFlag[procInstance_p],
                                   sizeof(pLock->afFlag[procInstance_p]));
}

//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for the target specific synchronization interrupt
used by the application for synchronization.

\param[in]      callback_p          Interrupt handler.
\param[in]      pArg_p              Argument to be passed when calling the handler.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p,
                               void* pArg_p)
{
    DPSHM_REG_SYNC_INTR(callback_p, pArg_p);
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt.

\param[in]      fEnable_p           Enable if TRUE, disable if FALSE.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_enableSyncIrq(BOOL fEnable_p)
{
    if (fEnable_p)
        DPSHM_ENABLE_SYNC_INTR();
    else
        DPSHM_DISABLE_SYNC_INTR();
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
