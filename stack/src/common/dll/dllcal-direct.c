/**
********************************************************************************
\file   dll/dllcal-direct.c

\brief  source file for DLL CAL direct call module

This DLL CAL queue implementation applies a single buffer for each queue
instance.

\ingroup module_dllcal
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

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DLLCALDIRECT_TXBUF_SIZE     C_DLL_MAX_ETH_FRAME ///< TX buffer size
#define DLLCALDIRECT_TXBUF_EMPTY    0                   ///< TX buffer marked as empty
#define DLLCALDIRECT_TXBUF_FILLING  1                   ///< TX buffer marked as being filled

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
    tDllCalQueue                  dllCalQueue;                            ///< DLL CAL queue
    size_t                        frameSize;                              ///< Size of frame in frame buffer (empty if zero)
    UINT8                         aFrameBuffer[DLLCALDIRECT_TXBUF_SIZE];  ///< Frame buffer
    struct sDllCalDirectInstance* pNext;                                  ///< Pointer to the next instance in the direct module
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
static tOplkError insertDataBlock(tDllCalQueueInstance pDllCalQueue_p, const void* pData_p, size_t dataSize_p);
static tOplkError getDataBlock(tDllCalQueueInstance pDllCalQueue_p, void* pData_p, size_t* pDataSize_p);
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p, UINT* pDataBlockCount_p);
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
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Add direct call DLL CAL instance

Add a direct call instance for TX packet forwarding in DLL CAL

\param[out]     ppDllCalQueue_p     Double-pointer to DllCal queue instance
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
    tDllCalDirectInstance*  pSearch;
    tDllCalDirectInstance*  pDllCalDirectInstance;
    BOOL                    fInstanceFound;

    //go through linked list and search for already available instance
    pSearch = pDllCalQueueHead_l;
    fInstanceFound = FALSE;

    while (pSearch != NULL)
    {
        if (pSearch->dllCalQueue == dllCalQueue_p)
        {
            //instance already exists
            fInstanceFound = TRUE;
            break;
        }

        if (pSearch->pNext == NULL)
        {
            //reached tail of linked list
            break;
        }

        //get next linked list member
        pSearch = (tDllCalDirectInstance*)pSearch->pNext;
    }

    //no instance was found
    if (!fInstanceFound)
    {
        //create new instance
        pDllCalDirectInstance =
            (tDllCalDirectInstance*)OPLK_MALLOC(sizeof(tDllCalDirectInstance));

        if (pDllCalDirectInstance == NULL)
        {
            ret = kErrorNoResource;
            goto Exit;
        }

        if (pSearch == NULL)
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
        pDllCalDirectInstance->frameSize = DLLCALDIRECT_TXBUF_EMPTY;
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

\param[in]      pDllCalQueue_p      Pointer to DllCal queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError delInstance(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;
    tDllCalDirectInstance*  pSearch;
    tDllCalDirectInstance*  pPrev;
    BOOL                    fInstanceFound;

    //start at head of linked list
    pSearch = pPrev = pDllCalQueueHead_l;
    fInstanceFound = FALSE;

    while (pSearch != NULL)
    {
        if (pSearch == pDllCalDirectInstance)
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

    if (fInstanceFound)
    {
        if (pSearch == pDllCalQueueHead_l)
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
        OPLK_FREE(pSearch);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Insert data block into direct call instance

Inserts a data block into the direct call instance. The data block can be of
any type (e.g. TX packet).

\param[in,out]  pDllCalQueue_p      Pointer to DllCal queue instance.
\param[in]      pData_p             Pointer to the data block to be inserted.
\param[in]      dataSize_p          Size of the data block to be inserted.

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
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    if (pDllCalDirectInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    if (pDllCalDirectInstance->frameSize != DLLCALDIRECT_TXBUF_EMPTY)
    {
        //TX buffer is not free
        ret = kErrorDllAsyncTxBufferFull;
        goto Exit;
    }

    //mark buffer that it is being filled
    pDllCalDirectInstance->frameSize = DLLCALDIRECT_TXBUF_FILLING;
    OPLK_MEMCPY(pDllCalDirectInstance->aFrameBuffer, pData_p, dataSize_p);

    // Mark buffer as filled by storing the size of the frame
    pDllCalDirectInstance->frameSize = dataSize_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block from direct call instance

Gets a data block from the direct call instance. The data block can be of any
type (e.g. TX packet).

\param[in,out]  pDllCalQueue_p      Pointer to DllCal queue instance
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
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    if (pDllCalDirectInstance == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    if ((pDllCalDirectInstance->frameSize == DLLCALDIRECT_TXBUF_EMPTY) ||
        (pDllCalDirectInstance->frameSize == DLLCALDIRECT_TXBUF_FILLING))
    {
        //TX buffer is empty or not ready
        ret = kErrorDllAsyncTxBufferEmpty;
        goto Exit;
    }

    if (pDllCalDirectInstance->frameSize > *pDataSize_p)
    {
        //provided data buffer is too small
        ret = kErrorNoResource;
        goto Exit;
    }

    OPLK_MEMCPY(pData_p,
                pDllCalDirectInstance->aFrameBuffer,
                pDllCalDirectInstance->frameSize);

    //return frame size
    *pDataSize_p = pDllCalDirectInstance->frameSize;

    //mark buffer is empty
    pDllCalDirectInstance->frameSize = DLLCALDIRECT_TXBUF_EMPTY;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get data block count from direct call instance

Returns the data block counter.

\param[in,out]  pDllCalQueue_p      Pointer to DllCal queue instance
\param[out]     pDataBlockCount_p   Pointer which returns the data block count

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError getDataBlockCount(tDllCalQueueInstance pDllCalQueue_p,
                                    UINT* pDataBlockCount_p)
{
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    if (pDllCalDirectInstance == NULL)
        return kErrorInvalidInstanceParam;

    if ((pDllCalDirectInstance->frameSize == DLLCALDIRECT_TXBUF_EMPTY) ||
        (pDllCalDirectInstance->frameSize == DLLCALDIRECT_TXBUF_FILLING))
        *pDataBlockCount_p = 0;
    else
        *pDataBlockCount_p = 1;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Reset direct call instance

Resets the direct call instance

\param[in,out]  pDllCalQueue_p      Pointer to DllCal queue instance

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other                       Error
*/
//------------------------------------------------------------------------------
static tOplkError resetDataBlockQueue(tDllCalQueueInstance pDllCalQueue_p)
{
    tDllCalDirectInstance*  pDllCalDirectInstance =
                                (tDllCalDirectInstance*)pDllCalQueue_p;

    if (pDllCalDirectInstance == NULL)
        return kErrorInvalidInstanceParam;

    //empty the buffer
    pDllCalDirectInstance->frameSize = DLLCALDIRECT_TXBUF_EMPTY;

    return kErrorOk;
}

/// \}
