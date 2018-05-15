/**
********************************************************************************
\file   dllucal-circbuf.c

\brief  Source file for user DLL CAL module

This file contains an implementation of the user dll CAL module which uses
the circular buffer library for communication with the kernel layer.

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
#include <common/circbuffer.h>

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
\brief DLL CAL shared buffer instance type

The EventShbInstance is used for every event queue using the shared buffer for
event posting.
*/
typedef struct
{
    tDllCalQueue        dllCalQueue;            ///< DLL CAL queue
    tCircBufInstance*   pCircBufInstance;       ///< Shared buffer instance
} tDllCalCircBufInstance;

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
static tOplkError getDataBlock(tDllCalQueueInstance pDllCalQueue_p,
                               void* pData_p,
                               size_t* pDataSize_p);
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p,
                                    UINT* pDataBlockCount_p);
static tOplkError resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p);

/* define external function interface */
static tDllCalFuncIntf funcintf_l =
{
    addInstance,
    delInstance,
    insertDataBlock,
    getDataBlock,
    getDataBlockCount,
    resetDataBlockQueue
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
tDllCalFuncIntf* dllucalcircbuf_getInterface(void)
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
    tCircBufError           error = kCircBufOk;
    tDllCalCircBufInstance* pDllCalCircBufInstance;

    // Check parameter validity
    ASSERT(ppDllCalQueue_p != NULL);

    pDllCalCircBufInstance = (tDllCalCircBufInstance*)OPLK_MALLOC(sizeof(tDllCalCircBufInstance));
    if (pDllCalCircBufInstance == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    //store parameters in instance
    pDllCalCircBufInstance->dllCalQueue = dllCalQueue_p;
    //initialize shared buffer
    switch (pDllCalCircBufInstance->dllCalQueue)
    {
        case kDllCalQueueTxGen:
            error = circbuf_connect(CIRCBUF_DLLCAL_TXGEN, &pDllCalCircBufInstance->pCircBufInstance);
            break;

        case kDllCalQueueTxNmt:
            error = circbuf_connect(CIRCBUF_DLLCAL_TXNMT, &pDllCalCircBufInstance->pCircBufInstance);
            break;

        case kDllCalQueueTxSync:
            error = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC, &pDllCalCircBufInstance->pCircBufInstance);
            break;

        case kDllCalQueueTxVeth:
            error = circbuf_connect(CIRCBUF_DLLCAL_TXVETH, &pDllCalCircBufInstance->pCircBufInstance);
            break;

        default:
            ret = kErrorInvalidInstanceParam;
            break;
    }

    if (error != kCircBufOk)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pDllCalCircBufInstance;

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
    tCircBufError           error;
    tDllCalCircBufInstance* pDllCalCircBufInstance =
                                (tDllCalCircBufInstance*)pDllCalQueue_p;

    error = circbuf_disconnect(pDllCalCircBufInstance->pCircBufInstance);
    if (error != kCircBufOk)
        return kErrorNoResource;

    OPLK_FREE(pDllCalCircBufInstance);

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
    tCircBufError           error;
    tDllCalCircBufInstance* pDllCalCircBufInstance =
                                (tDllCalCircBufInstance*)pDllCalQueue_p;

    // Check parameter validity
    ASSERT(pData_p != NULL);

    if (pDllCalCircBufInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_writeData(pDllCalCircBufInstance->pCircBufInstance,
                              pData_p,
                              dataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufBufferFull:
            ret = kErrorDllAsyncTxBufferFull;
            break;

        case kCircBufInvalidArg:
        default:
            ret = kErrorNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data block from queue

Gets a data block from the DLL CAL queue.

\param[in]      pDllCalQueue_p      Pointer to DllCal Queue instance
\param[out]     pData_p             Pointer to data buffer
\param[in,out]  pDataSize_p         Pointer to the size of the data buffer
                                    (will be replaced with actual data block size)

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlock(tDllCalQueueInstance pDllCalQueue_p,
                               void* pData_p,
                               size_t* pDataSize_p)
{
    tOplkError              ret = kErrorOk;
    tCircBufError           error;
    tDllCalCircBufInstance* pDllCalCircBufInstance =
                                (tDllCalCircBufInstance*)pDllCalQueue_p;
    size_t                  actualDataSize;

    // Check parameter validity
    ASSERT(pData_p != NULL);
    ASSERT(pDataSize_p != NULL);

    if (pDllCalCircBufInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_readData(pDllCalCircBufInstance->pCircBufInstance,
                             pData_p,
                             *pDataSize_p,
                             &actualDataSize);
    if (error != kCircBufOk)
    {
        if (error == kCircBufNoReadableData)
            ret = kErrorDllAsyncTxBufferEmpty;
        else
            ret = kErrorNoResource;
        goto Exit;
    }

    *pDataSize_p = actualDataSize;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data block count of queue

Returns the data block counter.

\param[in]      pDllCalQueue_p      Pointer to DllCal Queue instance
\param[out]     pDataBlockCount_p   Pointer which returns the data block count

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p,
                                    UINT* pDataBlockCount_p)
{
    tDllCalCircBufInstance* pDllCalCircBufInstance =
                                (tDllCalCircBufInstance*)pDllCalQueue_p;

    // Check parameter validity
    ASSERT(pDataBlockCount_p != NULL);

    if (pDllCalCircBufInstance == NULL)
        return kErrorInvalidInstanceParam;

    *pDataBlockCount_p = (UINT)circbuf_getDataCount(pDllCalCircBufInstance->pCircBufInstance);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Reset queue

Resets the DLL CAL queue instance after a given timeout.

\param[in]      pDllCalQueue_p      Pointer to DllCal Queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalCircBufInstance* pDllCalCircBufInstance =
                                (tDllCalCircBufInstance*)pDllCalQueue_p;

    if (pDllCalCircBufInstance == NULL)
        return kErrorInvalidInstanceParam;

    circbuf_reset(pDllCalCircBufInstance->pCircBufInstance);

    return kErrorOk;
}

/// \}
