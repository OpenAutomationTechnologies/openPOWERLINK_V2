/**
********************************************************************************
\file   errhnducal-posixshm.c

\brief  Implementation of user CAL module for error handler

This module implements the user layer CAL functions of the error handler.
This implementation uses posix shared memory to share the error objects
between user and kernel part.

\ingroup module_errhnducal
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
#include "errhnducal.h"

#include <unistd.h>
#include <stddef.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>

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
// global variable declaration
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
#define ERRHND_SHM_NAME         "/shmErrHnd"

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief User error handler instance type

The structure contains all necessary information needed by the user error
handler module.
*/
typedef struct
{
    tErrHndObjects* pLocalObjects;              ///< Pointer to user error objects
    int             fd;                         ///< File descriptor
    BOOL            fCreator;                   ///< Flag indicating the creator of the instance
    void*           pErrHndMem;                 ///< Pointer to the error handler shared memory
} tErrhnduCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrhnduCalInstance  instance_l =
{
    NULL,
    0,
    FALSE,
    NULL
};                                              ///< Instance variable of user error handler CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize error handler user CAL module

The function initializes the user layer CAL module of the error handler.

\param[in]      pLocalObjects_p     Pointer to local error objects

\return Always returns kErrorOk

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_init(tErrHndObjects* pLocalObjects_p)
{
    struct stat stat;

    instance_l.pLocalObjects = pLocalObjects_p;

    if (instance_l.pErrHndMem != NULL)
        return kErrorNoFreeInstance;

    instance_l.fd = shm_open(ERRHND_SHM_NAME, O_RDWR | O_CREAT, 0);
    if (instance_l.fd < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() shm_open failed!\n", __func__);
        return kErrorNoResource;
    }

    if (fstat(instance_l.fd, &stat) != 0)
    {
        close(instance_l.fd);
        return kErrorNoResource;
    }

    if (stat.st_size == 0)
    {
        if (ftruncate(instance_l.fd, sizeof(tErrHndObjects)) == -1)
        {
            DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
            close(instance_l.fd);
            shm_unlink(ERRHND_SHM_NAME);
            return kErrorNoResource;
        }
        instance_l.fCreator = TRUE;
    }

    instance_l.pErrHndMem = mmap(NULL, sizeof(tErrHndObjects), PROT_READ | PROT_WRITE, MAP_SHARED, instance_l.fd, 0);
    if (instance_l.pErrHndMem == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap error handler objects failed!\n", __func__);
        close(instance_l.fd);
        if (instance_l.fCreator)
            shm_unlink(ERRHND_SHM_NAME);
        return kErrorNoResource;
    }

    if (instance_l.fCreator)
    {
        OPLK_MEMSET(instance_l.pErrHndMem, 0, sizeof(tErrHndObjects));
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Shutdown error handler user CAL module

The function is used to de-initialize and shutdown the user layer CAL module of
the error handler.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
void errhnducal_exit(void)
{
    if (instance_l.pErrHndMem != NULL)
    {
        munmap(instance_l.pErrHndMem, sizeof(tErrHndObjects));
        close(instance_l.fd);
        if (instance_l.fCreator)
            shm_unlink(ERRHND_SHM_NAME);
        instance_l.fd = 0;
        instance_l.pErrHndMem = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Write an error handler object

The function writes an error handler object to the shared memory region used
by user and kernel modules.

\param[in]      index_p             Index of object in object dictionary
\param[in]      subIndex_p          Subindex of object
\param[in]      pParam_p            Pointer to object in error handlers memory space

\return Returns a tOplkError error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_writeErrorObject(UINT index_p,
                                       UINT subIndex_p,
                                       const UINT32* pParam_p)
{
    ptrdiff_t   offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    // Check parameter validity
    ASSERT(pParam_p != NULL);

    offset = (UINT8*)pParam_p - (UINT8*)instance_l.pLocalObjects;
    *(UINT32*)((UINT8*)instance_l.pErrHndMem + offset) = *pParam_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Read an error handler object

The function reads an error handler object from the shared memory region used
by user and kernel modules.

\param[in]      index_p             Index of object in object dictionary
\param[in]      subIndex_p          Subindex of object
\param[out]     pParam_p            Pointer to object in error handlers memory space

\return Returns a tOplkError error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_readErrorObject(UINT index_p,
                                      UINT subIndex_p,
                                      UINT32* pParam_p)
{
    ptrdiff_t   offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    // Check parameter validity
    ASSERT(pParam_p != NULL);

    offset = (UINT8*)pParam_p - (UINT8*)instance_l.pLocalObjects;
    *pParam_p = *(UINT32*)((UINT8*)instance_l.pErrHndMem + offset);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
