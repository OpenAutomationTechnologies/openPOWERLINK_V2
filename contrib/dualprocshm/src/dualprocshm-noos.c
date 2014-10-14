/**
********************************************************************************
\file   dualprocshm-noos.c

\brief  Dual Processor Library - Using shared memory for non-OS system

This file contains implementation of dual processor library for a non-OS system.

The library provides an interfacing bridge for systems having two processors
which uses a common memory(or different memories accessible through an interface)
for data exchange and communication.

It divides the shared memory into two parts- Common memory and Dynamic Memory
which can be implemented using two different memories or a single memory on hardware.
The common memory can be used by application for sharing status and control
information and is also used for implementing the dynamic address mapping table
to exchange memory buffer addresses. The dynamic memory is allocated during
runtime by one of the processors and can be used for implementing queue, arrays,
or used as a shared memory.

Dual processor library also requires a set of platform specific support routines
for memory initialization and data exchange.

\ingroup module_dualprocshm
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
#include <dualprocshm.h>

#include <stdlib.h>
#include <string.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DUALPROC_INSTANCE_COUNT     2   ///< Number of supported instances
#define MEM_LOCK_SIZE               1   ///< Memory lock size
#define DYN_MEM_TABLE_ENTRY_SIZE    4   ///< Size of Dynamic table entry

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
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/**
\brief Dynamic buffer configuration

Stores the configuration settings for the dynamic buffers
*/
static tDualprocDynResConfig    aDynResInit[MAX_DYNAMIC_BUFF_COUNT];

/**
\brief Instance array

This array holds all Dual Processor Driver instances available.
*/
static tDualProcDrv*            paDualProcDrvInstance[DUALPROC_INSTANCE_COUNT] =
{
    NULL, NULL
};
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void     setDynBuffAddr(tDualprocDrvInstance pDrvInst_p, UINT16 index_p, UINT32 addr_p);
static UINT32   getDynBuffAddr(tDualprocDrvInstance pDrvInst_p, UINT16 index_p);
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a dual processor driver instance

This function creates a dual processor driver instance, and
initializes it depending on the pConfig_p parameters.

\param  pConfig_p               The caller provides the configuration
                                parameters with this pointer.
\param  ppInstance_p            The function returns with this double-pointer
                                the created instance pointer. (return)

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful         The dual processor driver is configured successfully
                                    with the provided parameters.
\retval kDualprocInvalidParameter   The caller has provided incorrect parameters.
\retval kDualprocNoResource         Heap allocation was impossible or to many
                                    instances are present.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_create(tDualprocConfig* pConfig_p, tDualprocDrvInstance* ppInstance_p)
{
    tDualprocReturn     ret = kDualprocSuccessful;
    tDualProcDrv*       pDrvInst = NULL;
    INT                 iIndex;

    if (pConfig_p->procInstance != kDualProcFirst && pConfig_p->procInstance != kDualProcSecond )
    {
        return kDualprocInvalidParameter;
    }

    //create driver instance
    pDrvInst = (tDualProcDrv*) malloc(sizeof(tDualProcDrv));

    if (pDrvInst == NULL)
    {
        return kDualprocNoResource;
    }

    memset(pDrvInst, 0, sizeof(tDualProcDrv));

    // store the configuration

    pDrvInst->config = *pConfig_p;

    // get the common memory address
    pDrvInst->pCommMemBase = dualprocshm_getCommonMemAddr(&pDrvInst->config.commMemSize);

    if (pConfig_p->procInstance == kDualProcFirst)
    {
        memset(pDrvInst->pCommMemBase, 0, MAX_COMMON_MEM_SIZE);
    }
    // get the address to store address mapping table
    pDrvInst->pAddrTableBase = dualprocshm_getDynMapTableAddr();

    if (pConfig_p->procInstance == kDualProcFirst)
    {
        memset(pDrvInst->pAddrTableBase, 0, (MAX_DYNAMIC_BUFF_COUNT * 4));
    }
    pDrvInst->iMaxDynBuffEntries = MAX_DYNAMIC_BUFF_COUNT;
    pDrvInst->pDynResTbl = (tDualprocDynResConfig*)aDynResInit;

    for (iIndex = 0; iIndex < pDrvInst->iMaxDynBuffEntries; iIndex++)
    {
        pDrvInst->pDynResTbl[iIndex].pfnSetDynAddr = setDynBuffAddr;
        pDrvInst->pDynResTbl[iIndex].pfnGetDynAddr = getDynBuffAddr;
    }

    // store driver instance in array
    for (iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        if (paDualProcDrvInstance[iIndex] == NULL)
        {
            // free entry found
            paDualProcDrvInstance[iIndex] = pDrvInst;

            break;
        }
    }

    if (DUALPROC_INSTANCE_COUNT == iIndex)
    {
        ret = kDualprocNoResource;
        goto Exit;
    }

    // Return the driver instance
    *ppInstance_p = pDrvInst;

Exit:
    if (ret != kDualprocSuccessful)
    {
        dualprocshm_delete(pDrvInst);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor driver instance

This function deletes a dual processor driver instance.

\param  pInstance_p             The driver instance that should be
                                deleted.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_delete(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    INT             iIndex;

    if (pInstance_p == NULL)
    {
        return kDualprocInvalidParameter;
    }

    // release common memory and addr mapping table
    dualprocshm_releaseCommonMemAddr(pDrvInst->config.commMemSize);
    dualprocshm_releaseDynMapTableAddr();

    for (iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        if (pDrvInst == paDualProcDrvInstance[iIndex])
        {
            // delete the driver instance
            paDualProcDrvInstance[iIndex] = NULL;
            break;
        }
    }

    free(pDrvInst);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the driver instance of the given processor instance

If the instance is not found NULL is returned.

\param  instance_p              Processor instance

\return This returns the driver instance requested if found.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocDrvInstance dualprocshm_getDrvInst(tDualProcInstance instance_p)
{
    tDualProcDrv*   pDrvInst = NULL;
    INT             iIndex;

    for (iIndex = 0; iIndex < DUALPROC_INSTANCE_COUNT; iIndex++)
    {
        pDrvInst = (tDualProcDrv*)paDualProcDrvInstance[iIndex];

        if (instance_p == pDrvInst->config.procInstance)
        {
            break;
        }
    }

    return pDrvInst;
}

//------------------------------------------------------------------------------
/**
\brief  Retrieves a dynamic memory buffer specified with Id

\param  pInstance_p  Driver instance.
\param  id_p         Id of memory instance, used to index into the memory mapping
                     table to identify a dynamic memory instance and book keeping
                     in a local memory instance array.
\param  ppAddr_p     Base address of the requested memory.
\param  pSize_p      Size of the memory to be allocated/ Return Size.
\param  fAlloc_p     Allocate memory if TRUE, Retrieve address from address mapping
                     table if FALSE.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       requested resources not available

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getMemory(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                      UINT8** ppAddr_p, size_t* pSize_p, BOOL fAlloc_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          pMemBase;

    if (pInstance_p == NULL || ppAddr_p == NULL || pSize_p == NULL )
        return kDualprocInvalidParameter;

    if (id_p > MAX_DYNAMIC_BUFF_COUNT)
    {
        return kDualprocInvalidParameter;
    }

    if (fAlloc_p)
    {
        // allocate dynamic buffer
        pMemBase = DUALPROCSHM_MALLOC(*pSize_p + sizeof(tDualprocMemInst) );

        if (pMemBase == NULL)
            return kDualprocNoResource;

        memset(pMemBase, 0, (*pSize_p + sizeof(tDualprocMemInst)));

        pDrvInst->pDynResTbl[id_p].memInst = (tDualprocMemInst*) pMemBase;
        pDrvInst->pDynResTbl[id_p].pBase = pMemBase + sizeof(tDualprocMemInst);
        pDrvInst->pDynResTbl[id_p].memInst->span = (UINT16)*pSize_p;

        // write the address in mapping table
        pDrvInst->pDynResTbl[id_p].pfnSetDynAddr(pDrvInst, id_p, (UINT32)pMemBase);
    }
    else
    {
        pMemBase = (UINT8*)pDrvInst->pDynResTbl[id_p].pfnGetDynAddr(pDrvInst, id_p);

        if (pMemBase == NULL)
            return kDualprocNoResource;

        pDrvInst->pDynResTbl[id_p].memInst = (tDualprocMemInst*) pMemBase;
        pDrvInst->pDynResTbl[id_p].pBase = pMemBase + sizeof(tDualprocMemInst);
        *pSize_p = (size_t) pDrvInst->pDynResTbl[id_p].memInst->span;
    }

    *ppAddr_p = pDrvInst->pDynResTbl[id_p].pBase;

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Retrieves a dynamic memory buffer specified with Id

\param  pInstance_p  Driver instance.
\param  id_p         Id to be used for memory, used for indexing the memory
                     instance array.
\param  fFree_p      Free memory if TRUE, clear address from address mapping table
                     if FALSE.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       requested resources not available.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_freeMemory(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                       BOOL fFree_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          pMemBase;

    if (pInstance_p == NULL )
        return kDualprocInvalidParameter;

    if (id_p > MAX_DYNAMIC_BUFF_COUNT)
        return kDualprocInvalidParameter;

    if (fFree_p)
    {
        pDrvInst->pDynResTbl[id_p].pfnSetDynAddr(pDrvInst, id_p, 0);
        pMemBase = (UINT8*)pDrvInst->pDynResTbl[id_p].memInst;
        pDrvInst->pDynResTbl[id_p].pBase = NULL;
        free(pMemBase);
    }
    else
    {
        pDrvInst->pDynResTbl[id_p].memInst = NULL;
        pDrvInst->pDynResTbl[id_p].pBase = NULL;
    }

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Read specified number of bytes from a specified buffer.

\param  pInstance_p  Driver instance.
\param  id_p         Id of the buffer.
\param  offset_p     Offset from the base to be read.
\param  Size_p       Number of bytes to be read.
\param  pData_p      Pointer to memory to receive the read data.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       Requested resources not available.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_readData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                     UINT32 offset_p, size_t Size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base;
    UINT32          highAddr;
    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    base = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(base + pDrvInst->pDynResTbl[id_p].memInst->span);

    if ((offset_p + Size_p) > highAddr)
    {
        return kDualprocNoResource;
    }

    dualprocshm_targetReadData(base + offset_p, Size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Write specified number of bytes to a specified buffer

\param  pInstance_p  Driver instance.
\param  id_p         Id of the buffer.
\param  offset_p     Offset from the base to be written.
\param  Size_p       Number of bytes to write.
\param  pData_p      Pointer to memory containing data to be written.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       Requested resources not available.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_writeData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                      UINT32 offset_p, size_t Size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base;
    UINT32          highAddr;
    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    base = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(base + pDrvInst->pDynResTbl[id_p].memInst->span);

    if ((offset_p + Size_p) > highAddr)
        return kDualprocNoResource;

    dualprocshm_targetWriteData(base + offset_p, Size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Read specified number of bytes from a specified offset from the common memory

\param  pInstance_p  Driver instance.
\param  offset_p     Offset from the base of common memory to be read.
\param  Size_p       Number of bytes to be read.
\param  pData_p      Pointer to memory to receive the read data.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_readDataCommon(tDualprocDrvInstance pInstance_p,
                                           UINT32 offset_p, size_t Size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base = pDrvInst->pCommMemBase;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_targetReadData(base + offset_p, Size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Read specified number of bytes from a specified offset from the common memory

\param  pInstance_p  driver instance.
\param  offset_p     Offset from the base of common memory to be written.
\param  Size_p       Number of bytes to write.
\param  pData_p      Pointer to memory containing data to be written.

\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_writeDataCommon(tDualprocDrvInstance pInstance_p,
                                            UINT32 offset_p, size_t Size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base = pDrvInst->pCommMemBase;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_targetWriteData(base + offset_p, Size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Acquire lock for a buffer

Acquires lock for the specified buffer for the calling processor.

\param  pInstance_p  Driver instance.
\param  id_p         Buffer id.


\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The lock is acquired successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_acquireBuffLock(tDualprocDrvInstance pInstance_p, UINT8 id_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;
    // Enter critical region
    target_enableGlobalInterrupt(FALSE);

#if (OPLK_OPTIMIZE != FALSE)
    // Acquire lock only for shared queues
    switch (id_p)
    {
        case kDualprocUsertoKernelQ:
        case kDualprocKerneltoUserQ:
        case kDualprocDllCalTxGenQ:
        case kDualprocDllCalTxNmtQ:
        case kDualprocDllCalTxSyncQ:
        case kDualprocErrorHandlerBuff:
        case kDualprocPdoBuff:
            dualprocshm_targetAcquireLock(&pDrvInst->pDynResTbl[id_p].memInst->lock,
                                          pDrvInst->config.procId);
            break;
        default:
            // No memory lock needed do nothing
            break;
    }
#else
    dualprocshm_targetAcquireLock(&pDrvInst->pDynResTbl[id_p].memInst->lock,
                                  pDrvInst->config.procId);
#endif
    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Release lock for a buffer

Release lock acquired before by the calling processor.

\param  pInstance_p  Driver instance.
\param  id_p         Buffer id.


\return The function returns a tDualprocReturn Error code.
\retval kDualprocSuccessful       The lock was released successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_releaseBuffLock(tDualprocDrvInstance pInstance_p, UINT8 id_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;

#if (OPLK_OPTIMIZE != FALSE)
    // Acquire lock only for shared queues
    switch (id_p)
    {
        case kDualprocUsertoKernelQ:
        case kDualprocKerneltoUserQ:
        case kDualprocDllCalTxGenQ:
        case kDualprocDllCalTxNmtQ:
        case kDualprocDllCalTxSyncQ:
        case kDualprocErrorHandlerBuff:
        case kDualprocPdoBuff:
            dualprocshm_targetReleaseLock(&pDrvInst->pDynResTbl[id_p].memInst->lock);
            break;
        default:
            // No memory lock needed do nothing
            break;
    }
#else
    dualprocshm_targetReleaseLock(&pDrvInst->pDynResTbl[id_p].memInst->lock);
#endif
    // Exit critical region
    target_enableGlobalInterrupt(TRUE);
    return kDualprocSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Write the buffer address in dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.
\param  addr_p       Address of the buffer.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
static void setDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p, UINT32 addr_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;

    dualprocshm_targetWriteData(tableBase + tableEntryOffs,
                                DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&addr_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read the buffer address from dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.

\return Address of the buffer requested.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
static UINT32 getDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          buffAddr;

    dualprocshm_targetReadData(tableBase + tableEntryOffs,
                               DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&buffAddr);
    return buffAddr;
}
