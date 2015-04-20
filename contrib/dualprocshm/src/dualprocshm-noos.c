/**
********************************************************************************
\file   dualprocshm-noos.c

\brief  Dual Processor Library - Using shared memory for non-OS system

This file contains the implementation of the dual processor library for non-OS
systems.

The library provides an interface for systems having two processors which use a
common memory for data exchange and communication.

It divides the shared memory into two parts (Common and Dynamic Memory), which
can be implemented using two different memories or a single memory on hardware.
The common memory can be used by applications for sharing status and control
information and is also used for implementing the dynamic address mapping table
to exchange memory buffer addresses. The dynamic memory is allocated during
runtime by one of the processors and can be used for implementing queues, arrays,
or shared memory.

The dual processor library also requires a set of platform specific support
routines for memory initialization and data exchange.

\ingroup module_dualprocshm
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015 Kalycito Infotech Private Limited
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
//------------------------------------------------------------------------------
#define BRIDGE_ENABLED    0xAB12

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Structure for the dualprocshm instance

This structure holds the current configured instance of the dual processor
shared memory library.
*/
typedef struct sDualProcShmInst
{
    tDualProcInstance       localProcessor;         ///< Local processor instance
    tDualProcInstance       remoteProcessor;        ///< Remote processor instance
    BOOL                    fInitialized;           ///< Flag for driver init state
    BOOL                    fBridgeEnabled;         ///< Flag for hardware bridge state
}tDualProcShmInst;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/**
\brief Dynamic buffer configuration

Stores the configuration settings for the dynamic buffers.
*/
static tDualprocDynResConfig    aDynResInit[MAX_DYNAMIC_BUFF_COUNT];

/**
\brief Driver instance array

This array holds all dual processor library instances available.
*/
static tDualProcDrv*            paDualProcDrvInstance[DUALPROC_INSTANCE_COUNT] =
{
    NULL, NULL
};

/**
\brief current instance of dual processor shared memory library

This variable holds the current configured instance of dual processor shared
memory library.
*/
static tDualProcShmInst         instance_l = {kDualProcLast,
                                              kDualProcLast,
                                              FALSE, FALSE};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int              setDynBuffAddr(tDualprocDrvInstance pDrvInst_p,
                                       UINT16 index_p, UINT32 addr_p);
static UINT32           getDynBuffAddr(tDualprocDrvInstance pDrvInst_p,
                                       UINT16 index_p);
static tDualprocReturn  configureCommMemHeader(tDualProcInstance procInstance_p,
                                               tDualprocHeader* pCommMemHeader_p);
tDualprocDrvInstance    getDrvInst(tDualProcInstance procInstance_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a dual processor driver instance

This function creates a dual processor driver instance and initializes it
depending on the pConfig_p parameters.

\param  pConfig_p               The caller provides the configuration
                                parameters with this pointer.
\param  ppInstance_p            Returns the created instance pointer.

\return The function returns a tDualprocReturn error code.
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
    INT                 index;

    if (pConfig_p->procInstance != kDualProcFirst && pConfig_p->procInstance != kDualProcSecond)
    {
        return kDualprocInvalidParameter;
    }

    //create driver instance
    pDrvInst = (tDualProcDrv*) DUALPROCSHM_MALLOC(sizeof(tDualProcDrv));

    if (pDrvInst == NULL)
    {
        return kDualprocNoResource;
    }

    DUALPROCSHM_MEMSET(pDrvInst, 0, sizeof(tDualProcDrv));

    // store the configuration
    pDrvInst->config = *pConfig_p;

    // get the common memory address
    pDrvInst->commMemInst.pCommMemHeader = (tDualprocHeader*) dualprocshm_getCommonMemAddr(&pDrvInst->config.commMemSize);

    if (pConfig_p->procInstance == kDualProcFirst)
    {
        DUALPROCSHM_MEMSET(pDrvInst->commMemInst.pCommMemHeader, 0, MAX_COMMON_MEM_SIZE);
    }

    // get the control segment base address
    pDrvInst->commMemInst.pCommMemBase =
        (UINT8*) ((UINT32) pDrvInst->commMemInst.pCommMemHeader + sizeof(tDualprocHeader));

    // get the address to store address mapping table
    pDrvInst->pAddrTableBase = dualprocshm_getDynMapTableAddr();

    if (pConfig_p->procInstance == kDualProcFirst)
    {
        DUALPROCSHM_MEMSET(pDrvInst->pAddrTableBase, 0, (MAX_DYNAMIC_BUFF_COUNT * 4));
        instance_l.localProcessor = kDualProcFirst;
        instance_l.remoteProcessor = kDualProcSecond;
    }
    else
    {
        instance_l.localProcessor = kDualProcSecond;
        instance_l.remoteProcessor = kDualProcFirst;
    }

    pDrvInst->maxDynBuffEntries = MAX_DYNAMIC_BUFF_COUNT;
    pDrvInst->pDynResTbl = (tDualprocDynResConfig*)aDynResInit;

    for (index = 0; index < pDrvInst->maxDynBuffEntries; index++)
    {
        pDrvInst->pDynResTbl[index].pfnSetDynAddr = setDynBuffAddr;
        pDrvInst->pDynResTbl[index].pfnGetDynAddr = getDynBuffAddr;
    }

    // store driver instance in array
    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        if (paDualProcDrvInstance[index] == NULL)
        {
            // free entry found
            paDualProcDrvInstance[index] = pDrvInst;

            break;
        }
    }

    if (DUALPROC_INSTANCE_COUNT == index)
    {
        ret = kDualprocNoResource;
    }
    else
    {
        // Return the driver instance
        *ppInstance_p = pDrvInst;
    }

    if (ret != kDualprocSuccessful)
    {
        dualprocshm_delete(pDrvInst);
        goto Exit;
    }

    // Set the driver instance init status to TRUE
    instance_l.fInitialized = TRUE;

    ret = configureCommMemHeader(pConfig_p->procInstance,
                                 pDrvInst->commMemInst.pCommMemHeader);
    if (ret != kDualprocSuccessful)
    {
        ret = kDualprocInvalidCommHeader;

        // Set the driver instance init status to FALSE
        instance_l.fInitialized = FALSE;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor driver instance

This function deletes a dual processor driver instance.

\param  pInstance_p             The driver instance to be deleted.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The driver instance is deleted successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_delete(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    INT             index;

    if (pInstance_p == NULL)
    {
        return kDualprocInvalidParameter;
    }

    // release common memory and address mapping table
    dualprocshm_releaseCommonMemAddr(pDrvInst->config.commMemSize);
    dualprocshm_releaseDynMapTableAddr();

    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        if (pDrvInst == paDualProcDrvInstance[index])
        {
            // delete the driver instance
            paDualProcDrvInstance[index] = NULL;
            break;
        }
    }

    DUALPROCSHM_FREE(pDrvInst);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the driver instance of the local processor

This function returns the dualprocshm driver instance of the local processor.

\return This returns the driver instance requested, if found; else returns NULL.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocDrvInstance dualprocshm_getLocalProcDrvInst(void)
{
    return getDrvInst(instance_l.localProcessor);
}

//------------------------------------------------------------------------------
/**
\brief  Returns the local processor instance

If the instance is not initialized last processor instance is returned.

\return This returns the local processor instance.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualProcInstance dualprocshm_getLocalProcInst(void)
{
    return instance_l.localProcessor;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the remote processor instance

If the instance is not initialized last processor instance is returned.

\return This returns the remote processor instance.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualProcInstance dualprocshm_getRemoteProcInst(void)
{
    return instance_l.remoteProcessor;
}

//------------------------------------------------------------------------------
/**
\brief  Retrieves a dynamic memory buffer

This function retrieves a dynamic buffer for the specified ID. The caller has
to specify the buffer size if the function shall allocate the memory.

\param  pInstance_p  Driver instance.
\param  id_p         ID of memory instance, used to index into the memory mapping
                     table to identify a dynamic memory instance and book keeping
                     in a local memory instance array.
\param  ppAddr_p     Base address of the requested memory.
\param  pSize_p      Size of the memory to be allocated.
                     If fAlloc_p is FALSE, the retrieved memory size is returned.
\param  fAlloc_p     Allocate memory if TRUE, retrieve address from address mapping
                     table if FALSE.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The memory is allocated/obtained successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       The requested resources is not available.

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
        // Allocate dynamic buffer
        pMemBase = DUALPROCSHM_MALLOC(*pSize_p + sizeof(tDualprocMemInst));

        if (pMemBase == NULL)
            return kDualprocNoResource;

        DUALPROCSHM_MEMSET(pMemBase, 0, (*pSize_p + sizeof(tDualprocMemInst)));

        pDrvInst->pDynResTbl[id_p].memInst = (tDualprocMemInst*) pMemBase;
        pDrvInst->pDynResTbl[id_p].pBase = pMemBase + sizeof(tDualprocMemInst);
        pDrvInst->pDynResTbl[id_p].memInst->span = (UINT16)*pSize_p;

        // Write the address in mapping table
        if (pDrvInst->pDynResTbl[id_p].pfnSetDynAddr(pDrvInst, id_p, (UINT32)pMemBase) != 0)
            return kDualprocNoResource;
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
\brief  Frees a dynamic memory buffer

The function frees the specified dynamic buffer.

\param  pInstance_p  Driver instance.
\param  id_p         ID addressing the dynamic memory buffer.
\param  fFree_p      Free memory if TRUE, clear address from address mapping table
                     if FALSE.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The dynamic memory buffer is freed successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       The requested resources not available.

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
        DUALPROCSHM_FREE(pMemBase);
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
\brief  Read data from a specified buffer

The function reads from a specified dynamic buffer.

\param  pInstance_p  Driver instance.
\param  id_p         ID of the buffer.
\param  offset_p     Offset within the buffer to be read.
\param  size_p       Number of bytes to be read.
\param  pData_p      Pointer to buffer where the read data is to be stored.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The dynamic buffer is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       The requested resources not available.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_readData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                     UINT32 offset_p, size_t size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base;
    UINT32          highAddr;

    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    base = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(base + pDrvInst->pDynResTbl[id_p].memInst->span);

    if ((offset_p + size_p) > highAddr)
    {
        return kDualprocNoResource;
    }

    dualprocshm_targetReadData(base + offset_p, size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Write data to a specified buffer

The function writes to a specified dynamic buffer.

\param  pInstance_p  Driver instance.
\param  id_p         ID of the buffer.
\param  offset_p     Offset within the buffer to be written.
\param  size_p       Number of bytes to write.
\param  pData_p      Pointer to memory containing data to be written.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The dynamic buffer is written successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocNoResource       Requested resources not available.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_writeData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                      UINT32 offset_p, size_t size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base;
    UINT32          highAddr;

    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    base = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(base + pDrvInst->pDynResTbl[id_p].memInst->span);

    if ((offset_p + size_p) > highAddr)
        return kDualprocNoResource;

    dualprocshm_targetWriteData(base + offset_p, size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Read from the common memory

The function reads from the common memory region.

\param  pInstance_p  Driver instance.
\param  offset_p     Offset from the base of common memory to be read.
\param  size_p       Number of bytes to be read.
\param  pData_p      Pointer to buffer where the read data is to be stored.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The common buffer is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_readDataCommon(tDualprocDrvInstance pInstance_p,
                                           UINT32 offset_p, size_t size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base = pDrvInst->commMemInst.pCommMemBase;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_targetReadData(base + offset_p, size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Write to the common memory

The function writes to the common memory region.

\param  pInstance_p  Driver instance.
\param  offset_p     Offset from the base of common memory to be written.
\param  size_p       Number of bytes to write.
\param  pData_p      Pointer to memory containing data to be written.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The common buffer is written successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_writeDataCommon(tDualprocDrvInstance pInstance_p,
                                            UINT32 offset_p, size_t size_p, UINT8* pData_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          base = pDrvInst->commMemInst.pCommMemBase;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_targetWriteData(base + offset_p, size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Enable dual processor bridge in common memory header

The function enables the dual processor shared memory bridge in the
common memory header, to indicate that the memory is initialized

\param  pInstance_p  Driver instance.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The shared memory address is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm

*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_enableBridge(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*       pDrvInst = (tDualProcDrv*) pInstance_p;
    tDualprocHeader*    pCommMemHeader = NULL;
    UINT16              bridgeState = BRIDGE_ENABLED;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;
    else if (instance_l.fInitialized != TRUE)
        return kDualprocInvalidInstance;

    pCommMemHeader = pDrvInst->commMemInst.pCommMemHeader;

    dualprocshm_targetWriteData((UINT8*) (&pCommMemHeader->dpshmBridge),
                                sizeof(pCommMemHeader->dpshmBridge),
                                (UINT8*) (&bridgeState));

    instance_l.fBridgeEnabled = TRUE;

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Check the state of dual processor bridge in common memory header

The function checks the dual processor shared memory bridge state in the
common memory header, used to wait for the bridge to be initialized.

\param  pInstance_p  Driver instance.

\return The function returns a tDualprocReturn error code.
\retval kDualprocBridgeEnabled    The common memory bridge is enabled.
\retval kDualprocBridgeDisabled   The common memory bridge is disabled.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance  This dpshm instance is invalid.

\ingroup module_dualprocshm

*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_checkBridgeState(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*       pDrvInst = (tDualProcDrv*) pInstance_p;
    tDualprocHeader*    pCommMemHeader = NULL;
    UINT16              bridgeState = 0x0;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    else if (instance_l.fInitialized != TRUE)
        return kDualprocInvalidInstance;

    pCommMemHeader = pDrvInst->commMemInst.pCommMemHeader;

    dualprocshm_targetReadData((UINT8*) (&pCommMemHeader->dpshmBridge),
                               sizeof(pCommMemHeader->dpshmBridge),
                               (UINT8*) (&bridgeState));

    if (bridgeState == BRIDGE_ENABLED)
    {
        instance_l.fBridgeEnabled = TRUE;
        return kDualprocBridgeEnabled;
    }
    else
    {
        instance_l.fBridgeEnabled = FALSE;
        return kDualprocBridgeDisabled;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Read shared memory address from common memory header

The function reads the shared memory base address of the driver instance provided,
from the common memory header segment.

\param  pInstance_p         Driver instance.
\param  pShmBaseAddr_p      Pointer to buffer where the read data is to be stored.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The shared memory address is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance  This dpshm instance is invalid.
\retval kDualprocHwReadError      The shared memory address could not be read.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getSharedMemAddr(tDualprocDrvInstance pInstance_p,
                                             tDualProcInstance procInstance_p,
                                             UINT8* pShmBaseAddr_p)
{
    tDualProcDrv*       pDrvInst = (tDualProcDrv*) pInstance_p;
    tDualprocHeader*    pCommMemHeader = NULL;

    if (pInstance_p == NULL || pShmBaseAddr_p == NULL ||
        procInstance_p >= kDualProcLast)
        return kDualprocInvalidParameter;
    else if (instance_l.fInitialized != TRUE)   // Check if driver instance is initialized
        return kDualprocInvalidInstance;
    else if ((procInstance_p != instance_l.localProcessor) &&
             (instance_l.fBridgeEnabled != TRUE)) // For remote processor's shmem base address,
        return kDualprocBridgeDisabled;           // bridge has to be enabled.

    pCommMemHeader = pDrvInst->commMemInst.pCommMemHeader;

    dualprocshm_targetReadData((UINT8*) &pCommMemHeader->sharedMemBase[procInstance_p],
                               sizeof(UINT32), pShmBaseAddr_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Acquire lock for a buffer

This function locks the specified buffer for the calling processor.

\param  pInstance_p  The driver instance of the calling processor.
\param  id_p         The ID of the buffer to be locked.

\return The function returns a tDualprocReturn error code.
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

#if (OPLK_OPTIMIZE == TRUE)
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

This function releases the lock of the specified buffer for the calling processor.

\param  pInstance_p  The driver instance of the calling processor.
\param  id_p         The ID of the buffer to be released.

\return The function returns a tDualprocReturn error code.
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
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Returns the driver instance of the given processor instance

This function returns the dualprocshm driver instance of the specified processor.

\param  procInstance_p          Processor instance

\return This returns the driver instance requested, if found; else returns NULL.

*/
//------------------------------------------------------------------------------
tDualprocDrvInstance getDrvInst(tDualProcInstance procInstance_p)
{
    tDualProcDrv*   pDrvInst = NULL;
    INT             index;

    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        pDrvInst = (tDualProcDrv*)paDualProcDrvInstance[index];

        if (procInstance_p == pDrvInst->config.procInstance)
        {
            break;
        }
    }

    return pDrvInst;
}

//------------------------------------------------------------------------------
/**
\brief  Write the buffer address in dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.
\param  addr_p       Address of the buffer.

\return The function returns an integer return code.
\retval 0       Buffer address was successfully written.
\retval -1      Buffer address could not be written.

*/
//------------------------------------------------------------------------------
static int setDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p, UINT32 addr_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          offset = 0;
    UINT32          sharedMemSize = 0;
    UINT32          sharedMemBaseAddr = (UINT32) dualprocshm_getSharedMemInst(&sharedMemSize);

    if (addr_p <= sharedMemBaseAddr)
    {
        TRACE("The buffer address(0x%X) lies below the shared memory region start address(0x%X)\n"
                , addr_p, sharedMemBaseAddr);
        return -1;
    }
    else
    {
        offset = addr_p - sharedMemBaseAddr;

        if (offset >= sharedMemSize)
        {
            TRACE("The buffer address(0x%X) lies above the shared memory region end address(0x%X)\n"
                    , addr_p, sharedMemBaseAddr + sharedMemSize);
            return -1;
        }
    }

    dualprocshm_targetWriteData(tableBase + tableEntryOffs,
                                DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&offset);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read the buffer address from dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.

\return Address of the buffer requested.
\retval 0       Invalid buffer address read.

*/
//------------------------------------------------------------------------------
static UINT32 getDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          offset = 0x00;
    UINT32          buffAddr;
    UINT32          sharedMemSize = 0;
    UINT32          sharedMemBaseAddr = (UINT32) dualprocshm_getSharedMemInst(&sharedMemSize);

    dualprocshm_targetReadData(tableBase + tableEntryOffs,
                               DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&offset);

    if (offset == 0)
    {
        return 0;
    }
    else
    {
        buffAddr = (sharedMemBaseAddr + offset);
        
        if ((buffAddr <= sharedMemBaseAddr) || (buffAddr >= sharedMemBaseAddr + sharedMemSize))
        {
            TRACE("The buffer address(0x%X) lies outside the shared memory region(0x%X to 0x%X)\n"
                    , buffAddr, sharedMemBaseAddr, sharedMemBaseAddr + sharedMemSize);
            buffAddr = 0;
        }
    }
    return buffAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Configure the header segment of common memory.

Writes the header data as per the configured dualprocshm instance.
The header can not be written if dualprocshm instance is not initialized.

\param  procInstance_p        Driver instance.
\param  pCommMemHeader_p      Common memory header address.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful         The common memory header segment was
                                    configured successfully.
\retval kDualprocInvalidParameter   The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance    The dualprocshm instance is not initialized.

*/
//------------------------------------------------------------------------------
static tDualprocReturn configureCommMemHeader(tDualProcInstance procInstance_p,
                                              tDualprocHeader* pCommMemHeader_p)
{
    UINT32      sharedMemSize = 0;
    UINT32      sharedMemBaseAddr = (UINT32) dualprocshm_getSharedMemInst(&sharedMemSize);
    UINT8       dpshmInstState = instance_l.fInitialized;

    if (pCommMemHeader_p == NULL || procInstance_p >= kDualProcLast)
        return kDualprocInvalidParameter;

    // Check if the driver is already initialized
    if (dpshmInstState == TRUE)
    {
        dualprocshm_targetWriteData((UINT8*) (&pCommMemHeader_p->sharedMemBase[procInstance_p]),
                                    sizeof(UINT32), (UINT8*) (&sharedMemBaseAddr));
    }
    else
        return kDualprocInvalidInstance;

    return kDualprocSuccessful;
}

/// \}
