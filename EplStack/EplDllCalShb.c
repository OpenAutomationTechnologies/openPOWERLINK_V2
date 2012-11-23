/**
********************************************************************************
\file   EplDllCalShb.c

\brief  source file for DLL CAL Shared Buffer module

This DLL CAL queue implementation applies the shared buffer for TX packet
forwarding.
The shared buffer is available for different architectures (e.g. NoOS).

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
#include "EplDllCalShb.h"
#include "EplDllCal.h"

#include "SharedBuff.h"

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
    tEplDllCalQueue         DllCalQueue_m;
        ///< DLL CAL queue
    tShbInstance            pShbInstance_m;
        ///< shared buffer instance
} tEplDllCalShbInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    add shared buffer DLL CAL instance

Add a shared buffer instance for TX packet forwarding in DLL CAL

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  DllCalQueue_p           parameter that determines the queue

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbAddInstance (tEplDllCalQueueInstance *ppDllCalQueue_p,
        tEplDllCalQueue DllCalQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError = kShbOk;
    tEplDllCalShbInstance *pDllCalShbInstance;
    unsigned int fShbNewCreated;

    pDllCalShbInstance = (tEplDllCalShbInstance *)
            EPL_MALLOC(sizeof(tEplDllCalShbInstance));

    if(pDllCalShbInstance == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    //store parameters in instance
    pDllCalShbInstance->DllCalQueue_m = DllCalQueue_p;

    //initialize shared buffer
    switch(pDllCalShbInstance->DllCalQueue_m)
    {
        case kEplDllCalQueueTxGen:
            ShbError = ShbCirAllocBuffer(
                    EPL_DLLCAL_BUFFER_SIZE_TX_GEN,
                    EPL_DLLCAL_BUFFER_ID_TX_GEN,
                    &pDllCalShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        case kEplDllCalQueueTxNmt:
            ShbError = ShbCirAllocBuffer(
                    EPL_DLLCAL_BUFFER_SIZE_TX_NMT,
                    EPL_DLLCAL_BUFFER_ID_TX_NMT,
                    &pDllCalShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        case kEplDllCalQueueTxSync:
            ShbError = ShbCirAllocBuffer(
                    EPL_DLLCAL_BUFFER_SIZE_TX_SYNC,
                    EPL_DLLCAL_BUFFER_ID_TX_SYNC,
                    &pDllCalShbInstance->pShbInstance_m,
                    &fShbNewCreated);
            break;
        default:
            Ret = kEplInvalidInstanceParam;
            break;
    }

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    *ppDllCalQueue_p = (tEplDllCalQueueInstance*)pDllCalShbInstance;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    delete shared buffer DLL CAL instance

Delete the shared buffer instance

\param  pDllCalQueue_p          pointer to DllCal Queue instance

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbDelInstance (tEplDllCalQueueInstance pDllCalQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplDllCalShbInstance *pDllCalShbInstance =
            (tEplDllCalShbInstance*)pDllCalQueue_p;

    //free shared buffer
    ShbError = ShbCirReleaseBuffer(pDllCalShbInstance->pShbInstance_m);

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    //free dllcal shb instance
    EPL_FREE(pDllCalShbInstance);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    insert data block into shared buffer

Inserts a data block into the shared buffer instance. The data block can be of
any type (e.g. TX packet).

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  pData_p                 pointer to the data block to be insert
\param  puiDataSize             pointer to the size of the data block to be
                                insert

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbInsertDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplDllCalShbInstance *pDllCalShbInstance =
            (tEplDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    ShbError = ShbCirWriteDataBlock(pDllCalShbInstance->pShbInstance_m,
            pData_p, *puiDataSize);

    // error handling
    switch (ShbError)
    {
        case kShbOk:
            break;

        case kShbExceedDataSizeLimit:
        case kShbBufferFull:
            Ret = kEplDllAsyncTxBufferFull;
            break;

        case kShbInvalidArg:
        default:
            Ret = kEplNoResource;
            break;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    get data block from shared buffer

Gets a data block from the shared buffer instance. The data block can be of any
type (e.g. TX packet).

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  pData_p                 pointer to data buffer
\param  puiDataSize             pointer to the size of the data buffer
                                (will be replaced with actual data block size)

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbGetDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplDllCalShbInstance *pDllCalShbInstance =
            (tEplDllCalShbInstance*)pDllCalQueue_p;
    unsigned long ulActualDataSize;

    if(pDllCalShbInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    ShbError = ShbCirReadDataBlock(pDllCalShbInstance->pShbInstance_m,
            (void*)pData_p, (unsigned long)*puiDataSize, &ulActualDataSize);

    if(ShbError != kShbOk)
    {
        if(ShbError == kShbNoReadableData)
        {
            Ret = kEplDllAsyncTxBufferEmpty;
        }
        else
        {
            Ret = kEplNoResource;
        }

        goto Exit;
    }

    *puiDataSize = (unsigned int)ulActualDataSize;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    get data block count from shared buffer

Returns the data block counter

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  pulDataBlockCount       pointer which returns the data block count

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbGetDataBlockCount (
        tEplDllCalQueueInstance pDllCalQueue_p,
        unsigned long *pulDataBlockCount)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplDllCalShbInstance *pDllCalShbInstance =
            (tEplDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    ShbError = ShbCirGetReadBlockCount(pDllCalShbInstance->pShbInstance_m,
            pulDataBlockCount);

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    reset shared buffer

Resets the shared buffer instance after a given timeout.

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  ulTimeOutMs_p           timeout before buffer reset is done

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalShbResetDataBlockQueue (
        tEplDllCalQueueInstance pDllCalQueue_p,
        unsigned long ulTimeOutMs_p)
{
    tEplKernel Ret = kEplSuccessful;
    tShbError ShbError;
    tEplDllCalShbInstance *pDllCalShbInstance =
            (tEplDllCalShbInstance*)pDllCalQueue_p;

    if(pDllCalShbInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    ShbError = ShbCirResetBuffer(pDllCalShbInstance->pShbInstance_m,
            ulTimeOutMs_p, NULL);

    if(ShbError != kShbOk)
    {
        Ret = kEplNoResource;
    }

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

