/**
********************************************************************************
\file   dllcal-direct.c

\brief  source file for DLL CAL direct call module

This DLL CAL queue implementation applies a single buffer for each queue
instance.

\ingroup module_dllcal
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
#include <common/dllcal.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EPL_DLLCALDIRECT_TXBUF_SIZE     EPL_C_IP_MAX_MTU    ///< TX buffer size
#define EPL_DLLCALDIRECT_TXBUF_EMPTY    0                   ///< TX buffer marked as empty
#define EPL_DLLCALDIRECT_TXBUF_FILLING  1                   ///< TX buffer makred as being filled

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
\brief DLL CAL direct call instance type

The EventDirectInstance is used for every event queue using the direct call
event posting.
*/
typedef struct sDllCalDirectInstance
{
    tDllCalQueue        dllCalQueue;            ///< DLL CAL queue
    UINT                frameSize;              ///< size of frame in frame buffer (empty if zero)
    BYTE                aFrameBuffer[EPL_DLLCALDIRECT_TXBUF_SIZE];   ///< frame buffer
    struct sDllCalDirectInstance *pNext;   ///< pointer to next instance in direct module
} tDllCalDirectInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDllCalDirectInstance* pDllCalQueueHead_l = NULL; ///< pointer to the head of Dll Cal queue instances

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
\brief	Return pointer to function interface

This function returns a pointer to the function interface structure which
is used to access the dllcal functions of the direct call implementation.

\return Returns a pointer to the local function interface

\ingroup module_dllcal
*/
//------------------------------------------------------------------------------
tDllCalFuncIntf* dllcaldirect_getInterface(void)
{
    return &funcintf_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Add direct call DLL CAL instance

Add a direct call instance for TX packet forwarding in DLL CAL

\param  ppDllCalQueue_p         Double-pointer to DllCal Queue instance
\param  dllCalQueue_p           Parameter that determines the queue

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError addInstance (tDllCalQueueInstance *ppDllCalQueue_p,
                               tDllCalQueue dllCalQueue_p)
{
    tOplkError                  ret = kEplSuccessful;
    tDllCalDirectInstance*      pSearch;
    tDllCalDirectInstance*      pDllCalDirectInstance;
    BOOL                        fInstanceFound;

    //go through linked list and search for already available instance
    pSearch = pDllCalQueueHead_l;
    fInstanceFound = FALSE;

    while(pSearch != NULL)
    {
        if(pSearch->dllCalQueue == dllCalQueue_p)
        {
            //instance already exists
            fInstanceFound = TRUE;
            break;
        }

        if(pSearch->pNext == NULL)
        {
            //reached tail of linked list
            break;
        }

        //get next linked list member
        pSearch = (tDllCalDirectInstance*)pSearch->pNext;
    }

    //no instance was found
    if(!fInstanceFound)
    {
        //create new instance
        pDllCalDirectInstance =
             (tDllCalDirectInstance *)EPL_MALLOC(sizeof(tDllCalDirectInstance));

        if(pDllCalDirectInstance == NULL)
        {
            ret = kEplNoResource;
            goto Exit;
        }

        if(pSearch == NULL)
        {
            //insert new instance at head of linked list
            pDllCalQueueHead_l = pDllCalDirectInstance;
        }
        else
        {
            //insert new instance at tail of linked list
            pSearch->pNext = (tDllCalDirectInstance*)pDllCalDirectInstance;
        }

        //set new instance next to NULL
        pDllCalDirectInstance->pNext = NULL;

        //store parameters in instance
        pDllCalDirectInstance->dllCalQueue = dllCalQueue_p;

        //reset TX buffer
        pDllCalDirectInstance->frameSize = EPL_DLLCALDIRECT_TXBUF_EMPTY;
    }
    else
    {
        //set instance pointer to found instance
        pDllCalDirectInstance = pSearch;
    }

    //return pointer to created or found instance
    *ppDllCalQueue_p = (tDllCalQueueInstance*)pDllCalDirectInstance;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete direct call DLL CAL instance

Delete the direct call instance.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError delInstance (tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalDirectInstance*      pDllCalDirectInstance =
                                    (tDllCalDirectInstance*)pDllCalQueue_p;
    tDllCalDirectInstance*      pSearch;
    tDllCalDirectInstance*      pPrev;
    BOOL                        fInstanceFound;

    //start at head of linked list
    pSearch = pPrev = pDllCalQueueHead_l;
    fInstanceFound = FALSE;

    while(pSearch != NULL)
    {
        if(pSearch == pDllCalDirectInstance)
        {
            //instance found
            fInstanceFound = TRUE;
            break;
        }

        //store current member to previous
        pPrev = pSearch;

        //get next linked list member
        pSearch = (tDllCalDirectInstance*)pSearch->pNext;
    }

    if(fInstanceFound)
    {
        if(pSearch == pDllCalQueueHead_l)
        {
            //head is freed
            pDllCalQueueHead_l = (tDllCalDirectInstance*)pSearch->pNext;
        }
        else
        {
            //bypass found member
            pPrev->pNext = pSearch->pNext;
        }

        //free found member instance
        EPL_FREE(pSearch);
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Insert data block into direct call instance

Inserts a data block into the direct call instance. The data block can be of
any type (e.g. TX packet).

\param  pDllCalQueue_p          Pointer to DllCal Queue instance.
\param  pData_p                 Pointer to the data block to be inserted.
\param  pDataSize_p             Pointer to the size of the data block to be
                                inserted.

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                   BYTE *pData_p, UINT *pDataSize_p)
{
    tOplkError                  ret = kEplSuccessful;
    tDllCalDirectInstance*      pDllCalDirectInstance =
                                    (tDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pDllCalDirectInstance->frameSize != EPL_DLLCALDIRECT_TXBUF_EMPTY)
    {
        //TX buffer is not free
        ret = kEplDllAsyncTxBufferFull;
        goto Exit;
    }

    //mark buffer that it is being filled
    pDllCalDirectInstance->frameSize = EPL_DLLCALDIRECT_TXBUF_FILLING;

    EPL_MEMCPY(pDllCalDirectInstance->aFrameBuffer, pData_p, *pDataSize_p);

    //mark buffer that it is filled, with the size of the frame
    pDllCalDirectInstance->frameSize = *pDataSize_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block from direct call instance

Gets a data block from the direct call instance. The data block can be of any
type (e.g. TX packet).

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pData_p                 Pointer to data buffer
\param  pDataSize_p             Pointer to the size of the data buffer
                                (will be replaced with actual data block size)

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlock (tDllCalQueueInstance pDllCalQueue_p,
                                BYTE *pData_p, UINT *pDataSize_p)
{
    tOplkError                  ret = kEplSuccessful;
    tDllCalDirectInstance*      pDllCalDirectInstance =
                                    (tDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pDllCalDirectInstance->frameSize == EPL_DLLCALDIRECT_TXBUF_EMPTY ||
       pDllCalDirectInstance->frameSize == EPL_DLLCALDIRECT_TXBUF_FILLING)
    {
        //TX buffer is empty or not ready
        ret = kEplDllAsyncTxBufferEmpty;
        goto Exit;
    }

    if(pDllCalDirectInstance->frameSize > *pDataSize_p)
    {
        //provided data buffer is too small
        ret = kEplNoResource;
        goto Exit;
    }

    EPL_MEMCPY(pData_p, pDllCalDirectInstance->aFrameBuffer,
            pDllCalDirectInstance->frameSize);

    //return frame size
    *pDataSize_p = pDllCalDirectInstance->frameSize;

    //mark buffer is empty
    pDllCalDirectInstance->frameSize = EPL_DLLCALDIRECT_TXBUF_EMPTY;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block count from direct call instance

Returns the data block counter.

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  pDataBlockCount_p       Pointer which returns the data block count

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlockCount (tDllCalQueueInstance pDllCalQueue_p,
                                     ULONG* pDataBlockCount_p)
{
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        return kEplInvalidInstanceParam;
    }

    if(pDllCalDirectInstance->frameSize == EPL_DLLCALDIRECT_TXBUF_EMPTY ||
       pDllCalDirectInstance->frameSize == EPL_DLLCALDIRECT_TXBUF_FILLING)
    {
        *pDataBlockCount_p = 0;
    }
    else
    {
        *pDataBlockCount_p = 1;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Reset direct call insatnce

Resets the direct call instance

\param  pDllCalQueue_p          Pointer to DllCal Queue instance
\param  timeOutMs_p             Timeout in milliseconds.

\return The function returns a tOplkError error code.
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
static tOplkError resetDataBlockQueue (tDllCalQueueInstance pDllCalQueue_p,
                                       ULONG timeOutMs_p)
{
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    UNUSED_PARAMETER(timeOutMs_p);

    if(pDllCalDirectInstance == NULL)
    {
        return kEplInvalidInstanceParam;
    }

    //empty the buffer
    pDllCalDirectInstance->frameSize = EPL_DLLCALDIRECT_TXBUF_EMPTY;

    return kEplSuccessful;
}



