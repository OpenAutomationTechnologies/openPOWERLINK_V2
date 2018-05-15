/**
********************************************************************************
\file   ctrlcal-posixshm.c

\brief  Posix shared memory implementation for control CAL module

The file contains a posix shared memory implementation which can be used by the
memory block control CAL modules.

\ingroup module_ctrl
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
#include <common/ctrlcal.h>

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
#define CTRL_SHM_NAME       "/shmCtrlCal"

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
/**
\brief Control module instance

The control module instance stores the local parameters used by the
control CAL module during runtime
*/
typedef struct
{
    int                     fd;             ///< File descriptor
    void*                   pCtrlMem;       ///< Pointer to control memory
    size_t                  size;           ///< Size of the control memory
    BOOL                    fCreator;       ///< Flag indicating the creator of the memory
} tCtrlCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlCalInstance     instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize control CAL module

The function initializes the control CAL module.

\param[in]      size_p              The size of the memory control block.

\return The function returns a tOplkError error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlcal_init(size_t size_p)
{
    struct stat stat;

    instance_l.fd = shm_open(CTRL_SHM_NAME, O_RDWR | O_CREAT, 0);
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
        if (ftruncate(instance_l.fd, size_p) == -1)
        {
            DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
            close(instance_l.fd);
            shm_unlink(CTRL_SHM_NAME);
            return kErrorNoResource;
        }
        instance_l.fCreator = TRUE;
    }

    instance_l.pCtrlMem = mmap(NULL, size_p, PROT_READ | PROT_WRITE, MAP_SHARED, instance_l.fd, 0);
    if (instance_l.pCtrlMem == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap header failed!\n", __func__);
        close(instance_l.fd);
        if (instance_l.fCreator)
            shm_unlink(CTRL_SHM_NAME);
        return kErrorNoResource;
    }

    if (instance_l.fCreator)
    {
        OPLK_MEMSET(instance_l.pCtrlMem, 0, size_p);
    }
    instance_l.size = size_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up control module

The function cleans up the control CAL module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlcal_exit(void)
{
    tOplkError  ret = kErrorOk;

    if (instance_l.pCtrlMem != NULL)
    {
        munmap(instance_l.pCtrlMem, instance_l.size);
        close(instance_l.fd);
        if (instance_l.fCreator)
            shm_unlink(CTRL_SHM_NAME);
        instance_l.fd = 0;
        instance_l.pCtrlMem = NULL;
        instance_l.size = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Write data to control block

The function writes data to the control block.

\param[in]      offset_p            Offset in memory block to store the data.
\param[in]      pSrc_p              Pointer to the data which should be stored.
\param[in]      length_p            The length of the data to be stored.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
void ctrlcal_writeData(size_t offset_p,
                       const void* pSrc_p,
                       size_t length_p)
{
    // Check parameter validity
    ASSERT(pSrc_p != NULL);

    if (instance_l.pCtrlMem == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() instance == NULL!\n", __func__);
        return;
    }

    OPLK_MEMCPY((UINT8*)instance_l.pCtrlMem + offset_p, pSrc_p, length_p);
}

//------------------------------------------------------------------------------
/**
\brief Read data from control block

The function reads data from the control block.

\param[out]     pDest_p             Pointer to store the read data.
\param[in]      offset_p            Offset in memory block from which to read.
\param[in]      length_p            The length of the data to be read.

\return The function returns a tOplkError error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlcal_readData(void* pDest_p,
                            size_t offset_p,
                            size_t length_p)
{
    // Check parameter validity
    ASSERT(pDest_p != NULL);

    if (instance_l.pCtrlMem == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() instance_l.pCtrlMem == NULL!\n", __func__);
        return kErrorGeneralError;
    }

    OPLK_MEMCPY(pDest_p, (UINT8*)instance_l.pCtrlMem + offset_p, length_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
