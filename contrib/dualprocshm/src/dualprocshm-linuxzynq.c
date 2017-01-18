/**
********************************************************************************
\file   dualprocshm-linuxzynq.c

\brief  Dual Processor Library Support File - Linux zynq

This file provides target specific function definitions to support
dual processor shared memory interface on Zynq Linux.

\ingroup module_dualprocshm
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Kalycito Infotech Private Limited
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
#include <zynqdrv.h>
#include <dualprocshm.h>
#include <trace/trace.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEFAULT_LOCK_ID             0x00    ///< Default lock Id

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
\brief  Get common memory address for platform

Target specific routine to retrieve the base address of common memory between
two processors.

\param[in,out]  pSize_p             Minimum size of the common memory on input,
                                    returns the actual size of common memory as
                                    output.

\return Pointer to base address of common memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void* dualprocshm_getCommonMemAddr(UINT16* pSize_p)
{
    void*   pAddr;

    if ((pSize_p == NULL) ||
        (*pSize_p > MAX_COMMON_MEM_SIZE ))
    {
        TRACE("%s Common memory not available\n", __func__);
        return NULL;
    }

    pAddr = zynqdrv_getMemRegionAddr(kIoMemRegionCommon);

    *pSize_p = MAX_COMMON_MEM_SIZE - 1;

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free common memory address

Target specific routine to release the base address of common memory.

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
\brief  Get shared memory information for platform

Target specific routine to retrieve the shared memory base and size.

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

     pAddr = zynqdrv_getMemRegionAddr(kIoMemRegionShared);
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
    void*     pAddr = zynqdrv_getMemRegionAddr(kIoMemRegionCommon);

    if (pAddr == NULL)
        return NULL;

    pAddr = (UINT8*)pAddr + MEM_ADDR_TABLE_OFFSET;

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
    // nothing to be done on Linux
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
    void*     pAddr = zynqdrv_getMemRegionAddr(kIoMemRegionCommon);

    if (pAddr == NULL)
        return NULL;

    pAddr = (UINT8*)pAddr + MEM_INTR_OFFSET;

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
    // nothing to be done for Linux
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

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pBase_p, size_p);

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

    DUALPROCSHM_FLUSH_DCACHE_RANGE(pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(acquire)

This routine implements a target specific locking mechanism using the shared
memory between two processors/processes. The caller needs to pass the base
address and processor instance of the calling processor.

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetAcquireLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    volatile UINT8  lock = 0;
    UINT8           lockToken;

     if (pBase_p == NULL)
         return;

     switch (procInstance_p)
     {
         case kDualProcFirst:
         case kDualProcSecond:
             lockToken = procInstance_p + 1;
             break;

         default:
             TRACE("Invalid processor instance\n");
             return;
     }

     // spin till the passed token is written into memory
     do
     {
         DUALPROCSHM_INVALIDATE_DCACHE_RANGE((UINT32)&pBase_p->lockToken, 1);
         lock = DPSHM_READ8((UINT32)&pBase_p->lockToken);

         if (lock == DEFAULT_LOCK_ID)
         {
             DPSHM_WRITE8((UINT32)&pBase_p->lockToken, lockToken);
             DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32)&pBase_p->lockToken, 1);
             // Avoid re-ordering of load-store instruction to avoid
             // memory corruption. This ensures the successful write of
             // lock token in memory before the successive read.
             DPSHM_DMB();
             continue;
         }
     } while (lock != lockToken);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(release)

This routine is used to release a lock acquired at a specified address.

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReleaseLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    volatile UINT8  defaultlock = DEFAULT_LOCK_ID;

    UNUSED_PARAMETER(procInstance_p);

    if (pBase_p == NULL)
        return;

    DPSHM_WRITE8((UINT32)&pBase_p->lockToken, defaultlock);
    DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32)&pBase_p->lockToken, sizeof(UINT8));
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
    zynqdrv_enableSync(fEnable_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
