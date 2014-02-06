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
Copyright (c) 2012-2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/EplInc.h>

#include <common/errhnd.h>
#include <unistd.h>
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHndObjects           *pLocalObjects_l;       ///< pointer to user error objects
static int                      fd_l;
static BYTE*                    pErrHndMem_l;
static BOOL                     fCreator_l;

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

\param  pLocalObjects_p         Pointer to local error objects

\return Always returns kEplSuccessful

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tEplKernel errhnducal_init (tErrHndObjects *pLocalObjects_p)
{
    struct stat             stat;

    pLocalObjects_l = pLocalObjects_p;

    if (pErrHndMem_l != NULL)
        return kEplNoFreeInstance;

    if ((fd_l = shm_open(ERRHND_SHM_NAME, O_RDWR | O_CREAT, 0)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() shm_open failed!\n", __func__);
        return kEplNoResource;
    }

    if (fstat(fd_l, &stat) != 0)
    {
        close (fd_l);
        return kEplNoResource;
    }

    if (stat.st_size == 0)
    {
        if (ftruncate(fd_l, sizeof(tErrHndObjects)) == -1)
        {
            DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
            close (fd_l);
            shm_unlink(ERRHND_SHM_NAME);
            return kEplNoResource;
        }
        fCreator_l = TRUE;
    }

    pErrHndMem_l = mmap(NULL, sizeof(tErrHndObjects), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    if (pErrHndMem_l == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap error handler objects failed!\n", __func__);
        close (fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);
        return kEplNoResource;
    }

    if (fCreator_l)
    {
        EPL_MEMSET(pErrHndMem_l, 0, sizeof(tErrHndObjects));
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    shutdown error handler user CAL module

The function is used to deinitialize and shutdown the user layer
CAL module of the error handler.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
void errhnducal_exit (void)
{
    if (pErrHndMem_l != NULL)
    {
        munmap(pErrHndMem_l, sizeof(tErrHndObjects));
        close(fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);
        fd_l = 0;
        pErrHndMem_l = 0;
    }
}

//------------------------------------------------------------------------------
/**
\brief    write an error handler object

The function writes an error handler object to the shared memory region used
by user and kernel modules.

\param  index_p             Index of object in object dictionary
\param  subIndex_p          Subindex of object
\param  pParam_p            Pointer to object in error handlers memory space

\return Returns a tEplKernel error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tEplKernel errhnducal_writeErrorObject(UINT index_p, UINT subIndex_p, UINT32 *pParam_p)
{
    UINT    offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    offset = (char *)pParam_p - (char *)pLocalObjects_l;
    *(UINT32*)(pErrHndMem_l + offset) = *pParam_p;
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    read an error handler object

The function reads an error handler object from the shared memory region used
by user and kernel modules.

\param  index_p             Index of object in object dictionary
\param  subIndex_p          Subindex of object
\param  pParam_p            Pointer to object in error handlers memory space

\return Returns a tEplKernel error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tEplKernel errhnducal_readErrorObject(UINT index_p, UINT subIndex_p, UINT32* pParam_p)
{
    UINT    offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    offset = (char *)pParam_p - (char *)pLocalObjects_l;
    *pParam_p = *(UINT32*)(pErrHndMem_l + offset);
    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//


