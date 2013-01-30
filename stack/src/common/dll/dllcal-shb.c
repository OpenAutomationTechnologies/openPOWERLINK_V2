/**
********************************************************************************
\file   dllcal-shb.c

\brief  Source file for DLL CAL Shared Buffer module

This DLL CAL queue implementation applies the shared buffer for TX packet
forwarding.
The shared buffer is available for different architectures (e.g. NoOS).

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
#include <SharedBuff.h>

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
    tDllCalQueue        dllCalQueue;        ///< DLL CAL queue
    tShbInstance        pShbInstance;       ///< shared buffer instance
} tDllCalShbInstance;

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
tDllCalFuncIntf* dllcalshb_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Add shared buffer DLL CAL instance

Add a shared buffer instance for TX packet forwarding in DLL CAL

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  dllCalQueue_p           parameter that determines the queue

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel addInstance(tDllCalQueueInstance *ppDllCalQueue_p,
                              tDllCalQueue dllCalQueue_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tShbError                   shbError = kShbOk;
    tDllCalShbInstance*         pDllCalShbInstance;
    UINT                        fShbNewCreated;

    pDllCalShbInstance = (tDllCalShbInstance *) EPL_MALLOC(sizeof(tDllCalShbInstance));

    if(pDllCalShbInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pDllCalShbInstance->dllCalQueue = dllCalQueue_p;
    //initialize shared buffer
    switch(pDllCalShbInstance->dllCalQueue)
    {
        case kDllCalQueueTxGen:
            shbError = ShbCirAllocBuffer(DLLCAL_BUFFER_SIZE_TX_GEN,
                                         DLLCAL_BUFFER_ID_TX_GEN,
                                         &pDllCalShbInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        case kDllCalQueueTxNmt:
            shbError = ShbCirAllocBuffer(DLLCAL_BUFFER_SIZE_TX_NMT,
                                         DLLCAL_BUFFER_ID_TX_NMT,
                                         &pDllCalShbInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        case kDllCalQueueTxSync:
            shbError = ShbCirAllocBuffer(DLLCAL_BUFFER_SIZE_TX_SYNC,
                                         DLLCAL_BUFFER_ID_TX_SYNC,
                                         &pDllCalShbInstance->pShbInstance,
                                         &fShbNewCreated);
            break;

        default:
            ret = kEplInvalidInstanceParam;
            break;
    }

    if(shbError != kShbOk)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    *ppDllCalQueue_p = (tDllCalQueueInstance*)pDllCalShbInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete shared buffer DLL CAL instance

Delete the shared buffer instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tShbError               shbError;
    tDllCalShbInstance*     pDllCalShbInstance =
                                    (tDllCalShbInstance*)pDllCalQueue_p;

    shbError = ShbCirReleaseBuffer(pDllCalShbInstance->pShbInstance);
    if(shbError != kShbOk)
    {
        return kEplNoResource;
    }

    //free dllcal shb instance
    EPL_FREE(pDllCalShbInstance);

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Insert data block into shared buffer

Inserts a data block into the shared buffer instance. The data block can be of
any type (e.g. TX packet).

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
    tShbError                   shbError;
    tDllCalShbInstance*         pDllCalShbInstance =
                                            (tDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    shbError = ShbCirWriteDataBlock(pDllCalShbInstance->pShbInstance,
            pData_p, *pDataSize_p);

    // error handling
    switch (shbError)
    {
        case kShbOk:
            break;

        case kShbExceedDataSizeLimit:
        case kShbBufferFull:
            ret = kEplDllAsyncTxBufferFull;
            break;

        case kShbInvalidArg:
        default:
            ret = kEplNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block from shared buffer

Gets a data block from the shared buffer instance. The data block can be of any
type (e.g. TX packet).

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
    tEplKernel              ret = kEplSuccessful;
    tShbError               shbError;
    tDllCalShbInstance*     pDllCalShbInstance =
                                            (tDllCalShbInstance*)pDllCalQueue_p;
    ULONG                   actualDataSize;

    if(pDllCalShbInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    shbError = ShbCirReadDataBlock(pDllCalShbInstance->pShbInstance,
                                   (void*)pData_p, (unsigned long)*pDataSize_p,
                                   &actualDataSize);
    if(shbError != kShbOk)
    {
        if(shbError == kShbNoReadableData)
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
\brief  Get data block count from shared buffer

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
    tEplKernel              ret = kEplSuccessful;
    tShbError               shbError;
    tDllCalShbInstance*     pDllCalShbInstance =
                                        (tDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    shbError = ShbCirGetReadBlockCount(pDllCalShbInstance->pShbInstance,
                                       pDataBlockCount_p);
    if(shbError != kShbOk)
    {
        ret = kEplNoResource;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Reset shared buffer

Resets the shared buffer instance after a given timeout.

\param  pDllCalQueue_p          pPinter to DllCal Queue instance
\param  timeOutMs_p             Timeout before buffer reset is done

\return Returns an error code
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tEplKernel resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p,
                                           ULONG timeOutMs_p)
{
    tEplKernel              ret = kEplSuccessful;
    tShbError               shbError;
    tDllCalShbInstance*     pDllCalShbInstance =
                                        (tDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    shbError = ShbCirResetBuffer(pDllCalShbInstance->pShbInstance,
                                 timeOutMs_p, NULL);
    if(shbError != kShbOk)
    {
        ret = kEplNoResource;
    }

Exit:
    return ret;
}


