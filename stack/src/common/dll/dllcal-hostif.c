/**
********************************************************************************
\file   dllcal-hostif.c

\brief  Source file for DLL CAL Host Interface module

This DLL CAL queue implementation applies the host interface queues for
TX packet forwarding.

\ingroup module_dllcal
*******************************************************************************/

/*------------------------------------------------------------------------------
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
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/dllcal.h>
#include <hostiflib.h>
#include <lfqueue.h>

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
\brief DLL CAL host interface queue instance type

The EventHifInstance is used for every dllcal queue using the host interface
queues.
*/
typedef struct
{
    tDllCalQueue            dllCalQueue; ///< DLL CAL queue
    tQueueInstance          pQueueInstance; ///< host interface queue instance
} tDllCalHifInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p);
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
static tOplkError getDataBlock(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p, ULONG* pDataBlockCount_p);
static tOplkError resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p, ULONG timeOutMs_p);

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
is used to access the dllcal functions of the host interface implementation.

\return Returns a pointer to the local function interface

\ingroup module_dllcal
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllcalhostif_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Add host interface DLL CAL instance

Add a host interface queue instance for TX packet forwarding in DLL CAL

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  dllCalQueue_p           parameter that determines the queue

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError addInstance(tDllCalQueueInstance *ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tOplkError                  ret = kErrorOk;
    tHostifReturn               hifRet;
    tDllCalHifInstance*         pDllCalInstance;
    tHostifInstance             pHifInstance;
    tHostifInstanceId           hifInstanceId;
    UINT8*                      pBufBase = NULL;
    UINT                        bufSize;
    tQueueConfig                lfqConfig;
    tQueueReturn                lfqRet;

    pDllCalInstance = (tDllCalHifInstance *) OPLK_MALLOC(sizeof(tDllCalHifInstance));

    if(pDllCalInstance == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    //store parameters in instance
    pDllCalInstance->dllCalQueue = dllCalQueue_p;

    // get host interface instance
#if defined(CONFIG_HOSTIF_PCP)

#if (CONFIG_HOSTIF_PCP != FALSE)
    lfqConfig.queueRole = kQueueConsumer;
#else
    lfqConfig.queueRole = kQueueProducer;
#endif
    pHifInstance = hostif_getInstance(0);

#else
    pHifInstance = NULL;
#endif

    if(pHifInstance == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    //initialize host interface queue
    switch(pDllCalInstance->dllCalQueue)
    {
        case kDllCalQueueTxGen:
            hifInstanceId = kHostifInstIdTxGenQueue;
            break;

        case kDllCalQueueTxNmt:
            hifInstanceId = kHostifInstIdTxNmtQueue;
            break;

        case kDllCalQueueTxSync:
            hifInstanceId = kHostifInstIdTxSyncQueue;
            break;

        default:
            ret = kErrorInvalidInstanceParam;
            goto Exit;
    }

    hifRet = hostif_getBuf(pHifInstance, hifInstanceId, &pBufBase, &bufSize);

    if(hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                __func__, hifRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    lfqConfig.fAllocHeap = FALSE; // malloc done in hostif
    lfqConfig.pBase = pBufBase;
    lfqConfig.span = (UINT16)bufSize;

    lfqRet = lfq_create(&lfqConfig, &pDllCalInstance->pQueueInstance);

    if(lfqRet != kQueueSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Queue create failed (%d)\n",
                __func__, lfqRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pDllCalInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete host interface DLL CAL instance

Delete the host interface queue instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tQueueReturn            lfqRet;
    tDllCalHifInstance*     pDllCalInstance = (tDllCalHifInstance*)pDllCalQueue_p;

    lfqRet = lfq_delete(pDllCalInstance->pQueueInstance);
    if(lfqRet != kQueueSuccessful)
    {
        return kErrorNoResource;
    }

    //free dllcal hif instance
    OPLK_FREE(pDllCalInstance);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Insert data block into host interface queue

Inserts a data block into the host interface queue instance. The data block can
be of any type (e.g. TX packet).

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to the data block to be insert
\param  pDataSize_p             Pointer to the size of the data block to be
                                insert

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                   BYTE *pData_p, UINT *pDataSize_p)
{
    tOplkError                  ret = kErrorOk;
    tQueueReturn                lfqRet;
    tDllCalHifInstance*         pDllCalInstance = (tDllCalHifInstance*)pDllCalQueue_p;

    if(pDllCalInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    lfqRet = lfq_entryEnqueue(pDllCalInstance->pQueueInstance, pData_p, *pDataSize_p);

    // error handling
    switch (lfqRet)
    {
        case kQueueSuccessful:
            break;

        case kQueueFull:
            ret = kErrorDllAsyncTxBufferFull;
            break;

        default:
            DEBUG_LVL_ERROR_TRACE("%s() Insert queue failed (0x%X)\n", __func__, lfqRet);
            ret = kErrorNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block from host interface queue

Gets a data block from the host interface queue instance. The data block can be
of any type (e.g. TX packet).

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to data buffer
\param  pDataSize_p             Pointer to the size of the data buffer
                                (will be replaced with actual data block size)

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                BYTE *pData_p, UINT *pDataSize_p)
{
    tOplkError              ret = kErrorOk;
    tQueueReturn            lfqRet;
    tDllCalHifInstance*     pDllCalInstance = (tDllCalHifInstance*)pDllCalQueue_p;
    WORD                    actualDataSize = (WORD)*pDataSize_p;

    if(pDllCalInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    lfqRet = lfq_entryDequeue(pDllCalInstance->pQueueInstance, pData_p, &actualDataSize);
    if(lfqRet != kQueueSuccessful)
    {
        if(lfqRet == kQueueEmpty)
        {
            ret = kErrorDllAsyncTxBufferEmpty;
        }
        else
        {
            DEBUG_LVL_ERROR_TRACE("%s() Extract queue failed (0x%X)\n", __func__, lfqRet);
            ret = kErrorNoResource;
        }

        goto Exit;
    }

    *pDataSize_p = (UINT)actualDataSize;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data block count from host interface queue

Returns the data block counter.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pDataBlockCount_p       Pointer which returns the data block count

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p,
                                    ULONG* pDataBlockCount_p)
{
    tOplkError              ret = kErrorOk;
    tQueueReturn            lfqRet;
    tDllCalHifInstance*     pDllCalInstance = (tDllCalHifInstance*)pDllCalQueue_p;
    WORD                    dataBlockCount;

    if(pDllCalInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    lfqRet = lfq_getEntryCount(pDllCalInstance->pQueueInstance, &dataBlockCount);

    *pDataBlockCount_p = (ULONG)dataBlockCount;

    if(lfqRet != kQueueSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Getting queue count failed (0x%X)\n", __func__, lfqRet);
        ret = kErrorNoResource;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Reset host interface queue

Resets the host interface queue instance after a given timeout.

\param  pDllCalQueue_p          pPinter to DllCal Queue instance
\param  timeOutMs_p             Timeout before buffer reset is done

\return The function returns a tOplkError error code.
\retval kErrorOk          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p,
                                           ULONG timeOutMs_p)
{
    tOplkError              ret = kErrorOk;
    tQueueReturn            lfqRet;
    tDllCalHifInstance*     pDllCalInstance = (tDllCalHifInstance*)pDllCalQueue_p;

    UNUSED_PARAMETER(timeOutMs_p);

    if(pDllCalInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    lfqRet = lfq_reset(pDllCalInstance->pQueueInstance);
    if(lfqRet != kQueueSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Resetting queue failed (0x%X)\n", __func__, lfqRet);
        ret = kErrorNoResource;
    }

Exit:
    return ret;
}


