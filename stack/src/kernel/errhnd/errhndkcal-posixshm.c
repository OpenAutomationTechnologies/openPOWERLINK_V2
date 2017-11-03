/**
********************************************************************************
\file   errhndkcal-posixshm.c

\brief  Implementation of kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses posix shared memory to share the error objects
between user and kernel part.

\ingroup module_errhndkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include "errhndkcal.h"

#include <unistd.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define ERRHND_SHM_NAME "/shmErrHnd"

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHndObjects*  pErrHndObjects_l = NULL;
static int              fd_l;
static BOOL             fCreator_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel layer error handler memory

The function initializes the kernel layer error handler memory.

\return Returns always kErrorOk

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_initMemory(void)
{
    struct stat     stat;

    if (pErrHndObjects_l != NULL)
        return kErrorNoFreeInstance;

    fd_l = shm_open(ERRHND_SHM_NAME, O_RDWR | O_CREAT, 0);
    if (fd_l < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() shm_open failed!\n", __func__);
        return kErrorNoResource;
    }

    if (fstat(fd_l, &stat) != 0)
    {
        close(fd_l);
        return kErrorNoResource;
    }

    if (stat.st_size == 0)
    {
        if (ftruncate(fd_l, sizeof(tErrHndObjects)) == -1)
        {
            DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
            close(fd_l);
            shm_unlink(ERRHND_SHM_NAME);
            return kErrorNoResource;
        }
        fCreator_l = TRUE;
    }

    pErrHndObjects_l = mmap(NULL, sizeof(tErrHndObjects), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    if (pErrHndObjects_l == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap header failed!\n", __func__);
        close(fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);
        return kErrorNoResource;
    }

    if (fCreator_l)
    {
        OPLK_MEMSET(pErrHndObjects_l, 0, sizeof(tErrHndObjects));
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  De-initialize kernel layer error handler memory

The function is used to de-initialize the kernel layer error handler memory.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_deinitMemory(void)
{
    if (pErrHndObjects_l != NULL)
    {
        munmap(pErrHndObjects_l, sizeof(tErrHndObjects));
        close(fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);

        fd_l = 0;
        pErrHndObjects_l = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to error handler objects

The function returns a pointer to the memory block where the error handler
objects are stored.

\return The function returns a pointer to the error handler objects.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tErrHndObjects* errhndkcal_getMemPtr(void)
{
    return pErrHndObjects_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
