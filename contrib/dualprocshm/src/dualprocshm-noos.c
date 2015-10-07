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
Copyright (c) 2015, Kalycito Infotech Private Limited
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

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DYN_MEM_TABLE_ENTRY_SIZE   4    ///< Size of Dynamic table entry

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
#define DPSHM_MAGIC    0xAB12

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Structure for the dualprocshm instance

This structure holds the currently configured instance of the dual processor
shared memory library.
*/
typedef struct sDualProcShmInst
{
    tDualProcInstance       localProcessor;         ///< Local processor instance
    tDualProcInstance       remoteProcessor;        ///< Remote processor instance
    BOOL                    fInitialized;           ///< Flag for driver init state
    BOOL                    fShmEnabled;            ///< Flag for dpshm interface state
} tDualProcShmInst;

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
static tDualProcDrv*            apDualProcDrvInstance_l[DUALPROC_INSTANCE_COUNT];

/**
\brief Current instance of dual processor shared memory library

This variable holds the currently configured instance of the dual processor
shared memory library.
*/
static tDualProcShmInst         dualProcInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tDualprocDrvInstance getDrvInst(tDualProcInstance procInstance_p);
static INT                  setDynBuffAddr(tDualprocDrvInstance pDrvInst_p,
                                           UINT16 index_p, UINT64 addr_p);
static UINT8*               getDynBuffAddr(tDualprocDrvInstance pDrvInst_p,
                                           UINT16 index_p);
static tDualprocReturn      configureCommonMemHeader(tDualProcInstance procInstance_p,
                                                     tDualprocHeader* pCommonMemHeader_p);

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
        return kDualprocInvalidParameter;

    // Create driver instance
    pDrvInst = (tDualProcDrv*)DUALPROCSHM_MALLOC(sizeof(tDualProcDrv));

    if (pDrvInst == NULL)
        return kDualprocNoResource;

    // Initialize the global driver instance variables
    DUALPROCSHM_MEMSET(pDrvInst, 0, sizeof(tDualProcDrv));
    DUALPROCSHM_MEMSET(apDualProcDrvInstance_l, 0, sizeof(apDualProcDrvInstance_l));

    dualProcInstance_l.localProcessor = kDualProcLast;
    dualProcInstance_l.remoteProcessor = kDualProcLast;
    dualProcInstance_l.fInitialized = FALSE;
    dualProcInstance_l.fShmEnabled = FALSE;

    // Store the configuration
    pDrvInst->config = *pConfig_p;

    // Get the common memory address
    pDrvInst->commonMemInst.pCommonMemHeader = (tDualprocHeader*)dualprocshm_getCommonMemAddr(&pDrvInst->config.commonMemSize);

    if (pDrvInst->commonMemInst.pCommonMemHeader == NULL)
    {
        ret = kDualprocNoResource;
        goto Exit;
    }

    if (pConfig_p->procInstance == kDualProcFirst)
        DUALPROCSHM_MEMSET(pDrvInst->commonMemInst.pCommonMemHeader, 0, MAX_COMMON_MEM_SIZE);

    // Get the control segment base address
    pDrvInst->commonMemInst.pCommonMemBase =
        (UINT8*)(((UINT8*)pDrvInst->commonMemInst.pCommonMemHeader) + sizeof(tDualprocHeader));

    // Get the address to store address mapping table
    pDrvInst->pAddrTableBase = dualprocshm_getDynMapTableAddr();

    if (pDrvInst->pAddrTableBase == NULL)
    {
        ret = kDualprocNoResource;
        goto Exit;
    }

    if (pConfig_p->procInstance == kDualProcFirst)
    {
        DUALPROCSHM_MEMSET(pDrvInst->pAddrTableBase, 0, (MAX_DYNAMIC_BUFF_COUNT * 4));
        dualProcInstance_l.localProcessor = kDualProcFirst;
        dualProcInstance_l.remoteProcessor = kDualProcSecond;
    }
    else
    {
        dualProcInstance_l.localProcessor = kDualProcSecond;
        dualProcInstance_l.remoteProcessor = kDualProcFirst;
    }

    pDrvInst->maxDynBuffEntries = MAX_DYNAMIC_BUFF_COUNT;
    pDrvInst->pDynResTbl = (tDualprocDynResConfig*)aDynResInit;

    for (index = 0; index < pDrvInst->maxDynBuffEntries; index++)
    {
        pDrvInst->pDynResTbl[index].pfnSetDynAddr = setDynBuffAddr;
        pDrvInst->pDynResTbl[index].pfnGetDynAddr = getDynBuffAddr;
    }

    // Store driver instance in array
    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        if (apDualProcDrvInstance_l[index] == NULL)
        {
            // Free entry found
            apDualProcDrvInstance_l[index] = pDrvInst;
            break;
        }
    }

    if (DUALPROC_INSTANCE_COUNT == index)
        ret = kDualprocNoResource;
    else
        *ppInstance_p = pDrvInst; // Return the driver instance

    if (ret != kDualprocSuccessful)
    {
        dualprocshm_delete(pDrvInst);
        goto Exit;
    }

    // Set the driver instance init status to TRUE
    dualProcInstance_l.fInitialized = TRUE;

    ret = configureCommonMemHeader(pConfig_p->procInstance,
                                   pDrvInst->commonMemInst.pCommonMemHeader);
    if (ret != kDualprocSuccessful)
    {
        ret = kDualprocInvalidCommHeader;

        // Set the driver instance init status to FALSE
        dualProcInstance_l.fInitialized = FALSE;
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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    INT             index;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    // release common memory and address mapping table
    dualprocshm_releaseCommonMemAddr(pDrvInst->config.commonMemSize);
    dualprocshm_releaseDynMapTableAddr();

    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        if (pDrvInst == apDualProcDrvInstance_l[index])
        {
            // delete the driver instance
            apDualProcDrvInstance_l[index] = NULL;
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

\return This function returns the local driver instance.
\retval NULL   Local driver instance is not found.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocDrvInstance dualprocshm_getLocalProcDrvInst(void)
{
    return getDrvInst(dualProcInstance_l.localProcessor);
}

//------------------------------------------------------------------------------
/**
\brief  Returns the local processor instance

The function returns the local processor instance if it is initialized.

\return This returns the local processor instance
\retval kDualProcLast   The driver instance is not initialized.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualProcInstance dualprocshm_getLocalProcInst(void)
{
    return dualProcInstance_l.localProcessor;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the remote processor instance

The function returns the remote processor instance if it is initialized.

\return This returns the remote processor instance.
\retval kDualProcLast   The driver instance is not initialized.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualProcInstance dualprocshm_getRemoteProcInst(void)
{
    return dualProcInstance_l.remoteProcessor;
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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pMemBase;

    if (pInstance_p == NULL || ppAddr_p == NULL || pSize_p == NULL)
        return kDualprocInvalidParameter;

    if (id_p > MAX_DYNAMIC_BUFF_COUNT)
        return kDualprocInvalidParameter;

    if (fAlloc_p)
    {
        // Allocate dynamic buffer
        pMemBase = (UINT8*)DUALPROCSHM_MALLOC(*pSize_p + sizeof(tDualprocMemInst));

        if (pMemBase == NULL)
            return kDualprocNoResource;

        DUALPROCSHM_MEMSET(pMemBase, 0, (*pSize_p + sizeof(tDualprocMemInst)));

        pDrvInst->pDynResTbl[id_p].pMemInst = (tDualprocMemInst*)pMemBase;
        pDrvInst->pDynResTbl[id_p].pBase = pMemBase + sizeof(tDualprocMemInst);
        pDrvInst->pDynResTbl[id_p].pMemInst->span = (UINT16)*pSize_p;

        // Write the address in mapping table
        if (pDrvInst->pDynResTbl[id_p].pfnSetDynAddr(pDrvInst, id_p, (UINT64)((PTR_T)pMemBase)) != 0)
            return kDualprocNoResource;
    }
    else
    {
        pMemBase = (UINT8*)pDrvInst->pDynResTbl[id_p].pfnGetDynAddr(pDrvInst, id_p);

        if (pMemBase == NULL)
            return kDualprocNoResource;

        pDrvInst->pDynResTbl[id_p].pMemInst = (tDualprocMemInst*)pMemBase;
        pDrvInst->pDynResTbl[id_p].pBase = pMemBase + sizeof(tDualprocMemInst);
        *pSize_p = (size_t)pDrvInst->pDynResTbl[id_p].pMemInst->span;
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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pMemBase;

    if (pInstance_p == NULL )
        return kDualprocInvalidParameter;

    if (id_p > MAX_DYNAMIC_BUFF_COUNT)
        return kDualprocInvalidParameter;

    if (fFree_p)
    {
        pDrvInst->pDynResTbl[id_p].pfnSetDynAddr(pDrvInst, id_p, 0);
        pMemBase = (UINT8*)pDrvInst->pDynResTbl[id_p].pMemInst;
        pDrvInst->pDynResTbl[id_p].pBase = NULL;
        DUALPROCSHM_FREE(pMemBase);
    }
    else
    {
        pDrvInst->pDynResTbl[id_p].pMemInst = NULL;
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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pBase;
    UINT32          highAddr;

    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    pBase = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(pBase + pDrvInst->pDynResTbl[id_p].pMemInst->span);

    if ((offset_p + size_p) > highAddr)
        return kDualprocNoResource;

    dualprocshm_targetReadData(pBase + offset_p, (UINT16)size_p, pData_p);

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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pBase;
    UINT32          highAddr;

    if (pInstance_p == NULL ||  id_p > MAX_DYNAMIC_BUFF_COUNT || pData_p == NULL)
        return kDualprocInvalidParameter;

    pBase = pDrvInst->pDynResTbl[id_p].pBase;
    highAddr = (UINT32)(pBase + pDrvInst->pDynResTbl[id_p].pMemInst->span);

    if ((offset_p + size_p) > highAddr)
        return kDualprocNoResource;

    dualprocshm_targetWriteData(pBase + offset_p, (UINT16)size_p, pData_p);

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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pCommMemBase = pDrvInst->commonMemInst.pCommonMemBase;
    UINT8*          pReadBase = pCommMemBase + offset_p;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    if ((pReadBase + size_p) > (pCommMemBase + pDrvInst->config.commonMemSize))
        return kDualprocBufferError;

    dualprocshm_targetReadData(pReadBase, (UINT16)size_p, pData_p);

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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          pCommMemBase = pDrvInst->commonMemInst.pCommonMemBase;
    UINT8*          pWriteBase = pCommMemBase + offset_p;

    if (pInstance_p == NULL || pData_p == NULL)
        return kDualprocInvalidParameter;

    if ((pWriteBase + size_p) > (pCommMemBase + pDrvInst->config.commonMemSize))
        return kDualprocBufferOverflow;

    dualprocshm_targetWriteData(pWriteBase, (UINT16)size_p, pData_p);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Enable dual processor shared memory interface in common memory header

The function enables the dual processor shared memory interface
in the common memory header to indicate that the memory is initialized.

\param  pInstance_p  Driver instance.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The shared memory address is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.

\ingroup module_dualprocshm

*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_enableShmIntf(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*       pDrvInst = (tDualProcDrv*)pInstance_p;
    tDualprocHeader*    pCommonMemHeader = NULL;
    UINT16              shmMagic = DPSHM_MAGIC;
    UINT16              shmIntfState = 0x0001;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    if (dualProcInstance_l.fInitialized != TRUE)
        return kDualprocInvalidInstance;

    pCommonMemHeader = pDrvInst->commonMemInst.pCommonMemHeader;

    // Write the dpshm magic
    dualprocshm_targetWriteData((UINT8*)(&pCommonMemHeader->shmMagic),
                                sizeof(pCommonMemHeader->shmMagic),
                                (UINT8*)(&shmMagic));

    // Enable the dual processor shared memory interface
    dualprocshm_targetWriteData((UINT8*)(&pCommonMemHeader->shmIntfState),
                                sizeof(pCommonMemHeader->shmIntfState),
                                (UINT8*)(&shmIntfState));

    dualProcInstance_l.fShmEnabled = TRUE;

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Check the state of the dual processor shared memory interface

The function checks the dual processor shared memory interface state in the
common memory header. It is used to wait for the shared memory interface to be
initialized.

\param  pInstance_p  Driver instance.

\return The function returns a tDualprocReturn error code.
\retval kDualprocshmIntfEnabled     The dualprocshm interface is enabled.
\retval kDualprocshmIntfDisabled    The dualprocshm interface is disabled.
\retval kDualprocInvalidParameter   The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance    This dualprocshm instance is invalid.

\ingroup module_dualprocshm

*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_checkShmIntfState(tDualprocDrvInstance pInstance_p)
{
    tDualprocReturn     ret;
    tDualProcDrv*       pDrvInst = (tDualProcDrv*)pInstance_p;
    tDualprocHeader*    pCommonMemHeader = NULL;
    UINT16              shmMagic = 0x0000;
    UINT16              shmIntfState = 0x0000;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    if (dualProcInstance_l.fInitialized != TRUE)
        return kDualprocInvalidInstance;

    pCommonMemHeader = pDrvInst->commonMemInst.pCommonMemHeader;

    dualprocshm_targetReadData((UINT8*)(&pCommonMemHeader->shmMagic),
                               sizeof(pCommonMemHeader->shmMagic),
                               (UINT8*)(&shmMagic));

    // Read the dpshm magic to check validity of header data
    if (shmMagic == DPSHM_MAGIC)
    {
        // Read the dpshm interface state
        dualprocshm_targetReadData((UINT8*)(&pCommonMemHeader->shmIntfState),
                                   sizeof(pCommonMemHeader->shmIntfState),
                                   (UINT8*)(&shmIntfState));

        if (shmIntfState == 0x0000)
        {
            dualProcInstance_l.fShmEnabled = FALSE;
            ret = kDualprocshmIntfDisabled;
        }
        else
        {
            dualProcInstance_l.fShmEnabled = TRUE;
            ret = kDualprocshmIntfEnabled;
        }
    }
    else
    {
        dualProcInstance_l.fShmEnabled = FALSE;
        ret = kDualprocInvalidCommHeader;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read shared memory address from common memory header

The function reads the shared memory base address of the provided driver instance
from the common memory header segment.

\param  pInstance_p         Driver instance.
\param  procInstance_p      The processor instance whose shared memory instance is queried.
\param  pShmMemInst_p       Pointer to shared memory instance to store the queried
                            information.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful       The shared memory address is read successfully.
\retval kDualprocInvalidParameter The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance  This instance is invalid.
\retval kDualprocHwReadError      The shared memory address could not be read.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_getSharedMemInfo(tDualprocDrvInstance pInstance_p,
                                             tDualProcInstance procInstance_p,
                                             tDualprocSharedMemInst* pShmMemInst_p)
{
    tDualProcDrv*       pDrvInst = (tDualProcDrv*)pInstance_p;
    tDualprocHeader*    pCommonMemHeader = NULL;

    if (pInstance_p == NULL || pShmMemInst_p == NULL || procInstance_p >= kDualProcLast)
        return kDualprocInvalidParameter;

    if (dualProcInstance_l.fInitialized != TRUE)
    {
        // Check if driver instance is initialized
        return kDualprocInvalidInstance;
    }

    if ((procInstance_p != dualProcInstance_l.localProcessor) &&
        (dualProcInstance_l.fShmEnabled != TRUE))
    {
        // For remote processor's base address, interface has to be enabled.
        return kDualprocshmIntfDisabled;
    }

    pCommonMemHeader = pDrvInst->commonMemInst.pCommonMemHeader;

    dualprocshm_targetReadData((UINT8*)&pCommonMemHeader->sharedMemBase[procInstance_p],
                               sizeof(UINT64), (UINT8*)&pShmMemInst_p->baseAddr);
    dualprocshm_targetReadData((UINT8*)&pCommonMemHeader->span, sizeof(UINT32),
                               (UINT8*)&pShmMemInst_p->span);

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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;

    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_targetAcquireLock(&pDrvInst->pDynResTbl[id_p].pMemInst->lock,
                                  pDrvInst->config.procInstance);
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
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;

    dualprocshm_targetReleaseLock(&pDrvInst->pDynResTbl[id_p].pMemInst->lock,
                                  pDrvInst->config.procInstance);

    return kDualprocSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get the driver instance

This function returns the dualprocshm driver instance of the specified processor.

\param  procInstance_p          Processor instance

\return This returns the driver instance requested, if found; else returns NULL.

*/
//------------------------------------------------------------------------------
static tDualprocDrvInstance getDrvInst(tDualProcInstance procInstance_p)
{
    tDualProcDrv*   pDrvInst = NULL;
    INT             index;

    for (index = 0; index < DUALPROC_INSTANCE_COUNT; index++)
    {
        pDrvInst = apDualProcDrvInstance_l[index];

        if (procInstance_p == pDrvInst->config.procInstance)
            break;
    }

    return pDrvInst;
}

//------------------------------------------------------------------------------
/**
\brief  Write the buffer address in dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.
\param  addr_p       Address of the buffer.

\return The function returns 0 if the address has been set successfully, otherwise -1.
*/
//------------------------------------------------------------------------------
static INT setDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p, UINT64 addr_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          offset = 0;
    UINT32          sharedMemSize = 0;
    UINT64          sharedMemBaseAddr = (UINT64)((PTR_T)dualprocshm_getSharedMemInst(&sharedMemSize));

    if (addr_p == 0x0)
    {
            dualprocshm_targetWriteData(tableBase + tableEntryOffs,
                                DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&offset);
            return 0;
    }

    if (addr_p <= sharedMemBaseAddr)
    {
        TRACE("The buffer address(0x%X) lies below the shared memory region start address(0x%X)\n",
              (UINT)addr_p, (UINT)sharedMemBaseAddr);
        return -1;
    }
    else
    {
        offset = (UINT32)(addr_p - sharedMemBaseAddr);

        if (offset >= sharedMemSize)
        {
            TRACE("The buffer address(0x%X) lies above the shared memory region end address(0x%X)\n",
                  (UINT)addr_p, (UINT)(sharedMemBaseAddr + sharedMemSize));
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
static UINT8* getDynBuffAddr(tDualprocDrvInstance pInstance_p, UINT16 index_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*)pInstance_p;
    UINT8*          tableBase = pDrvInst->pAddrTableBase;
    UINT32          tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32          offset = 0x00000000;
    UINT8*          pBuffAddr = NULL;
    UINT32          sharedMemSize = 0;
    UINT8*          pSharedMemBaseAddr = dualprocshm_getSharedMemInst(&sharedMemSize);

    if (pSharedMemBaseAddr == NULL)
        return NULL;

    dualprocshm_targetReadData(tableBase + tableEntryOffs,
                               DYN_MEM_TABLE_ENTRY_SIZE, (UINT8*)&offset);

    if (offset == 0)
    {
        TRACE("Failed to get address for index %d\n", index_p);
        return NULL;
    }

    pBuffAddr = (pSharedMemBaseAddr + offset);

    if ((pBuffAddr <= pSharedMemBaseAddr) || (pBuffAddr >= pSharedMemBaseAddr + sharedMemSize))
    {
        TRACE("The buffer address(0x%p) lies outside the shared memory region(0x%p to 0x%p)\n",
              pBuffAddr, pSharedMemBaseAddr, (pSharedMemBaseAddr + sharedMemSize));
        pBuffAddr = NULL;
    }

    return pBuffAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Configure the header segment of common memory.

Writes the header data as per the configured dualprocshm instance.
The header cannot be written if dualprocshm instance is not initialized.

\param  procInstance_p        Driver instance.
\param  pCommonMemHeader_p    Common memory header address.

\return The function returns a tDualprocReturn error code.
\retval kDualprocSuccessful         The common memory header segment was
                                    configured successfully.
\retval kDualprocInvalidParameter   The caller has provided incorrect parameters.
\retval kDualprocInvalidInstance    The dualprocshm instance is not initialized.

*/
//------------------------------------------------------------------------------
static tDualprocReturn configureCommonMemHeader(tDualProcInstance procInstance_p,
                                                tDualprocHeader* pCommonMemHeader_p)
{
    UINT32      sharedMemSize = 0;
    UINT64      sharedMemBaseAddr = (UINT64)((PTR_T)dualprocshm_getSharedMemInst(&sharedMemSize));
    UINT8       dpshmInstState = dualProcInstance_l.fInitialized;

    if (pCommonMemHeader_p == NULL || procInstance_p >= kDualProcLast)
        return kDualprocInvalidParameter;

    // Check if the driver is already initialized
    if (dpshmInstState == TRUE)
    {
        dualprocshm_targetWriteData((UINT8*)(&pCommonMemHeader_p->sharedMemBase[procInstance_p]),
                                    sizeof(UINT64), (UINT8*)(&sharedMemBaseAddr));

        if (procInstance_p == kDualProcFirst)
            dualprocshm_targetWriteData((UINT8*)(&pCommonMemHeader_p->span), sizeof(UINT32),
                                        (UINT8*)(&sharedMemSize));
    }
    else
    {
        return kDualprocInvalidInstance;
    }

    return kDualprocSuccessful;
}

/// \}
