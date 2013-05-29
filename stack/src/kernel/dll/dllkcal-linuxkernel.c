/**
********************************************************************************
\file   dllkcal-linuxkernel.c

\brief  Source file for Kernel DLL CAL circular buffer module

This file contains the implementation of the DLL CAL module for the
Linux kernel.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <dllcal.h>
#include <circbuffer.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DLLCAL_CIRCBUF_SIZE_TX_NMT   32767
#define DLLCAL_CIRCBUF_SIZE_TX_GEN   32767
#define DLLCAL_CIRCBUF_SIZE_TX_SYNC  8192

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
    tDllCalQueue        dllCalQueue;        ///< DLL CAL queue
    tCircBufInstance*   pCircBufInstance;
} tDllCalCircbufInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel addInstance(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
static tEplKernel delInstance(tDllCalQueueInstance pDllCalQueue_p);
static tEplKernel insertDataBlock(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
static tEplKernel getDataBlock(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
static tEplKernel getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p, ULONG* pDataBlockCount_p);
static tEplKernel resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p, ULONG timeOutMs_p);

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
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllkcalcircbuf_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Add DLL CAL instance

Add an instance for TX packet forwarding in DLL CAL.

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  dllCalQueue_p           parameter that determines the queue

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel addInstance(tDllCalQueueInstance* ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tCircBufError               circError = kCircBufOk;
    tDllCalCircbufInstance*     pDllCalCircbufInstance;

    pDllCalCircbufInstance = (tDllCalCircbufInstance *)EPL_MALLOC(sizeof(tDllCalCircbufInstance));
    if(pDllCalCircbufInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pDllCalCircbufInstance->dllCalQueue = dllCalQueue_p;
    //initialize shared buffer
    switch(pDllCalCircbufInstance->dllCalQueue)
    {
        case kDllCalQueueTxGen:
            circError = circbuf_alloc(CIRCBUF_DLLCAL_TXGEN, DLLCAL_CIRCBUF_SIZE_TX_GEN,
                                      &pDllCalCircbufInstance->pCircBufInstance);
            break;

        case kDllCalQueueTxNmt:
            circError = circbuf_alloc(CIRCBUF_DLLCAL_TXNMT, DLLCAL_CIRCBUF_SIZE_TX_NMT,
                                     &pDllCalCircbufInstance->pCircBufInstance);
            break;

        case kDllCalQueueTxSync:
            circError = circbuf_alloc(CIRCBUF_DLLCAL_TXSYNC, DLLCAL_CIRCBUF_SIZE_TX_SYNC,
                                      &pDllCalCircbufInstance->pCircBufInstance);
            break;

        default:
            ret = kEplInvalidInstanceParam;
            break;
    }

    if(circError != kCircBufOk)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pDllCalCircbufInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete DLL CAL instance

Delete the DLL CAL instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tCircBufError           error;
    tDllCalCircbufInstance*   pDllCalCircbufInstance =
                                    (tDllCalCircbufInstance*)pDllCalQueue_p;

    error = circbuf_free(pDllCalCircbufInstance->pCircBufInstance);
    if(error != kCircBufOk)
    {
        return kEplNoResource;
    }
    EPL_FREE(pDllCalCircbufInstance);
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Insert data block into queue

Inserts a data block into the DLL CAL queue.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to the data block to be insert
\param  pDataSize_p             Pointer to the size of the data block to be
                                insert

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel insertDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                   BYTE *pData_p, UINT *pDataSize_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tCircBufError                   error;
    tDllCalCircbufInstance*     pDllCalCircbufInstance =
                                            (tDllCalCircbufInstance*)pDllCalQueue_p;

    if(pDllCalCircbufInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_writeData(pDllCalCircbufInstance->pCircBufInstance, pData_p, *pDataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufExceedDataSizeLimit:
        //case kCircBufBufferFull: jba check for error flag
            ret = kEplDllAsyncTxBufferFull;
            break;

        case kCircBufInvalidArg:
        default:
            ret = kEplNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data block from queue

Gets a data block from the DLL CAL queue.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to data buffer
\param  pDataSize_p             Pointer to the size of the data buffer
                                (will be replaced with actual data block size)

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel getDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                BYTE *pData_p, UINT *pDataSize_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tCircBufError                   error;
    tDllCalCircbufInstance*     pDllCalCircbufInstance =
                                            (tDllCalCircbufInstance*)pDllCalQueue_p;
    size_t                      actualDataSize;

    if(pDllCalCircbufInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_readData(pDllCalCircbufInstance->pCircBufInstance, pData_p,
                             (size_t)*pDataSize_p, &actualDataSize);
    if(error != kCircBufOk)
    {
        if(error == kCircBufNoReadableData)
        {
            ret = kEplDllAsyncTxBufferEmpty;
        }
        else
        {
            ret = kEplNoResource;
        }

        goto Exit;
    }

    *pDataSize_p = (UINT)actualDataSize;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data block count of queue

Returns the data block counter.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pDataBlockCount_p       Pointer which returns the data block count

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p,
                                    ULONG* pDataBlockCount_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tDllCalCircbufInstance*     pDllCalCircbufInstance =
                                        (tDllCalCircbufInstance*)pDllCalQueue_p;
    UINT32                       count;

    if(pDllCalCircbufInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    count = circbuf_getDataCount(pDllCalCircbufInstance->pCircBufInstance);
    *pDataBlockCount_p = (ULONG)count;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset queue

Resets the DLL CAL queue instance after a given timeout.

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  timeOutMs_p             Timeout before buffer reset is done

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p,
                                      ULONG timeOutMs_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tDllCalCircbufInstance*     pDllCalCircbufInstance =
                                        (tDllCalCircbufInstance*)pDllCalQueue_p;

    if(pDllCalCircbufInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }
    circbuf_reset(pDllCalCircbufInstance->pCircBufInstance);

Exit:
    return ret;
}


