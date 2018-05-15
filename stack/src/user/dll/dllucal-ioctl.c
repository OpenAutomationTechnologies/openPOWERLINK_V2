/**
********************************************************************************
\file   dllucal-ioctl.c

\brief  Source file for user DLL CAL module

This file contains an implementation of the user dll CAL module which uses
Linux ioctl for communication with the kernel layer.

\ingroup module_dllucal
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
#include <common/dllcal.h>
#include <user/ctrlucal.h>
#include <common/driver.h>

#include <sys/ioctl.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief DLL CAL ioctl instance type

This struct is used for every event queue using the Linux ioctl for
event posting.
*/
typedef struct
{
    tDllCalQueue                dllCalQueue;        ///< DLL CAL queue
    tDllAsyncReqPriority        priority;           ///< Request priority
    OPLK_FILE_HANDLE            fd;                 ///< File descriptor of openPOWERLINK driver
} tDllCalIoctlInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance* ppDllCalQueue_p,
                              tDllCalQueue DllCalQueue_p);
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p);
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p,
                                  const void* pData_p,
                                  size_t dataSize_p);

/* define external function interface */
static tDllCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    insertDataBlock,
    NULL,
    NULL,
    NULL
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return pointer to function interface

This function returns a pointer to the function interface structure which
is used to access the dllcal functions of the shared buffer implementation.

\return Returns a pointer to the local function interface

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllcalioctl_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Add DLL CAL instance

Add an instance for TX packet forwarding in DLL CAL.

\param[out]     ppDllCalQueue_p     Double-pointer to DllCal Queue instance
\param[in]      dllCalQueue_p       Parameter that determines the queue

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance* ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tOplkError              ret = kErrorOk;
    tDllCalIoctlInstance*   pInstance;

    // Check parameter validity
    ASSERT(ppDllCalQueue_p != NULL);

    pInstance = (tDllCalIoctlInstance*)OPLK_MALLOC(sizeof(tDllCalIoctlInstance));
    if (pInstance == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    //store parameters in instance
    pInstance->dllCalQueue = dllCalQueue_p;
    pInstance->fd = ctrlucal_getFd();

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete DLL CAL instance

Delete the DLL CAL instance.

\param[in]      pDllCalQueue_p      Pointer to DllCal Queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalIoctlInstance*   pInstance = (tDllCalIoctlInstance*)pDllCalQueue_p;

    OPLK_FREE(pInstance);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Insert data block into queue

Inserts a data block into the DLL CAL queue.

\param[in]      pDllCalQueue_p      Pointer to DllCal Queue instance
\param[in]      pData_p             Pointer to the data block to be inserted
\param[in]      dataSize_p          Size of the data block to be inserted

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p,
                                  const void* pData_p,
                                  size_t dataSize_p)
{
    tOplkError              ret = kErrorOk;
    tDllCalIoctlInstance*   pInstance = (tDllCalIoctlInstance*)pDllCalQueue_p;
    tIoctlDllCalAsync       ioctlAsyncFrame;
    int                     ioctlRet;

    // Check parameter validity
    ASSERT(pData_p != NULL);

    if (pInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    DEBUG_LVL_DLL_TRACE("%s() send async frame: size:%lu\n", __func__, (ULONG)dataSize_p);

    ioctlAsyncFrame.size = dataSize_p;
    ioctlAsyncFrame.queue = pInstance->dllCalQueue;
    ioctlAsyncFrame.pData = (void*)pData_p;

    ioctlRet = ioctl(pInstance->fd, PLK_CMD_DLLCAL_ASYNCSEND, (ULONG)&ioctlAsyncFrame);
    if (ioctlRet < 0)
        return kErrorDllAsyncTxBufferFull;

    return kErrorOk;

Exit:
    return ret;
}

/// \}
