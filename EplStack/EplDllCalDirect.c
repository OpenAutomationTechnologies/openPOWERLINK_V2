/**
********************************************************************************
\file   EplDllCalDirect.c

\brief  source file for DLL CAL direct call module

This DLL CAL queue implementation applies a single buffer for each queue
instance.

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
#include "EplDllCalDirect.h"
#include "EplDllCal.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EPL_DLLCALDIRECT_TXBUF_SIZE     EPL_C_IP_MAX_MTU
    ///< TX buffer size
#define EPL_DLLCALDIRECT_TXBUF_EMPTY    0
    ///< TX buffer marked as empty
#define EPL_DLLCALDIRECT_TXBUF_FILLING  1
    ///< TX buffer makred as being filled

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
typedef struct _tEplDllCalDirectInstance
{
    tEplDllCalQueue             DllCalQueue_m;
        ///< DLL CAL queue
    unsigned int                uiFrameSize_m;
        ///< size of frame in frame buffer (empty if zero)
    BYTE                        abFrameBuffer_m[EPL_DLLCALDIRECT_TXBUF_SIZE];
        ///< frame buffer
    struct tEplDllCalDirectInstance *pNext_m;
        ///< pointer to next instance in direct module
} tEplDllCalDirectInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tEplDllCalDirectInstance *pDllCalQueueHead_l = NULL;
    ///< pointer to the head of Dll Cal queue instances

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    add direct call DLL CAL instance

Add a direct call instance for TX packet forwarding in DLL CAL

\param  ppDllCalQueue_p         double-pointer to DllCal Queue instance
\param  DllCalQueue_p           parameter that determines the queue

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalDirectAddInstance (tEplDllCalQueueInstance *ppDllCalQueue_p,
        tEplDllCalQueue DllCalQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pSearch;
    tEplDllCalDirectInstance *pDllCalDirectInstance;
    BOOL fInstanceFound;

    //go through linked list and search for already available instance
    pSearch = pDllCalQueueHead_l;
    fInstanceFound = FALSE;

    while(pSearch != NULL)
    {
        if(pSearch->DllCalQueue_m == DllCalQueue_p)
        {
            //instance already exists
            fInstanceFound = TRUE;
            break;
        }

        if(pSearch->pNext_m == NULL)
        {
            //reached tail of linked list
            break;
        }

        //get next linked list member
        pSearch = (tEplDllCalDirectInstance*)pSearch->pNext_m;
    }

    //no instance was found
    if(!fInstanceFound)
    {
        //create new instance
        pDllCalDirectInstance = (tEplDllCalDirectInstance *)
                EPL_MALLOC(sizeof(tEplDllCalDirectInstance));

        if(pDllCalDirectInstance == NULL)
        {
            Ret = kEplNoResource;
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
            pSearch->pNext_m =
                    (struct tEplDllCalDirectInstance*)pDllCalDirectInstance;
        }

        //set new instance next to NULL
        pDllCalDirectInstance->pNext_m = NULL;

        //store parameters in instance
        pDllCalDirectInstance->DllCalQueue_m = DllCalQueue_p;

        //reset TX buffer
        pDllCalDirectInstance->uiFrameSize_m = EPL_DLLCALDIRECT_TXBUF_EMPTY;
    }
    else
    {
        //set instance pointer to found instance
        pDllCalDirectInstance = pSearch;
    }

    //return pointer to created or found instance
    *ppDllCalQueue_p = (tEplDllCalQueueInstance*)pDllCalDirectInstance;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    delete direct call DLL CAL instance

Delete the direct call instance

\param  pDllCalQueue_p          pointer to DllCal Queue instance

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalDirectDelInstance (tEplDllCalQueueInstance pDllCalQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pDllCalDirectInstance =
            (tEplDllCalDirectInstance*)pDllCalQueue_p;
    tEplDllCalDirectInstance *pSearch;
    tEplDllCalDirectInstance *pPrev;
    BOOL fInstanceFound;

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
        pSearch = (tEplDllCalDirectInstance*)pSearch->pNext_m;
    }

    if(fInstanceFound)
    {
        if(pSearch == pDllCalQueueHead_l)
        {
            //head is freed
            pDllCalQueueHead_l = (tEplDllCalDirectInstance*)pSearch->pNext_m;
        }
        else
        {
            //bypass found member
            pPrev->pNext_m = pSearch->pNext_m;
        }

        //free found member instance
        EPL_FREE(pSearch);
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    insert data block into direct call instance

Inserts a data block into the direct call instance. The data block can be of
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
tEplKernel EplDllCalDirectInsertDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pDllCalDirectInstance =
            (tEplDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pDllCalDirectInstance->uiFrameSize_m != EPL_DLLCALDIRECT_TXBUF_EMPTY)
    {
        //TX buffer is not free
        Ret = kEplDllAsyncTxBufferFull;
        goto Exit;
    }

    //mark buffer that it is being filled
    pDllCalDirectInstance->uiFrameSize_m = EPL_DLLCALDIRECT_TXBUF_FILLING;

    EPL_MEMCPY(pDllCalDirectInstance->abFrameBuffer_m, pData_p, *puiDataSize_p);

    //mark buffer that it is filled, with the size of the frame
    pDllCalDirectInstance->uiFrameSize_m = *puiDataSize_p;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    get data block from direct call instance

Gets a data block from the direct call instance. The data block can be of any
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
tEplKernel EplDllCalDirectGetDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pDllCalDirectInstance =
            (tEplDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pDllCalDirectInstance->uiFrameSize_m == EPL_DLLCALDIRECT_TXBUF_EMPTY ||
       pDllCalDirectInstance->uiFrameSize_m == EPL_DLLCALDIRECT_TXBUF_FILLING)
    {
        //TX buffer is empty or not ready
        Ret = kEplDllAsyncTxBufferEmpty;
        goto Exit;
    }

    if(pDllCalDirectInstance->uiFrameSize_m > *puiDataSize)
    {
        //provided data buffer is too small
        Ret = kEplNoResource;
        goto Exit;
    }

    EPL_MEMCPY(pData_p, pDllCalDirectInstance->abFrameBuffer_m,
            pDllCalDirectInstance->uiFrameSize_m);

    //return frame size
    *puiDataSize = pDllCalDirectInstance->uiFrameSize_m;

    //mark buffer is empty
    pDllCalDirectInstance->uiFrameSize_m = EPL_DLLCALDIRECT_TXBUF_EMPTY;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    get data block count from direct call instance

Returns the data block counter

\param  pDllCalQueue_p          pointer to DllCal Queue instance
\param  pulDataBlockCount       pointer which returns the data block count

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalDirectGetDataBlockCount (
        tEplDllCalQueueInstance pDllCalQueue_p,
        unsigned long *pulDataBlockCount)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pDllCalDirectInstance =
            (tEplDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    if(pDllCalDirectInstance->uiFrameSize_m == EPL_DLLCALDIRECT_TXBUF_EMPTY ||
       pDllCalDirectInstance->uiFrameSize_m == EPL_DLLCALDIRECT_TXBUF_FILLING)
    {
        *pulDataBlockCount = 0;
    }
    else
    {
        *pulDataBlockCount = 1;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    reset direct call insatnce

Resets the direct call instance

\param  pDllCalQueue_p          pointer to DllCal Queue instance

\return tEplKernel
\retval kEplSuccessful          if function executes correctly
\retval other                   error
*/
//------------------------------------------------------------------------------
tEplKernel EplDllCalDirectResetDataBlockQueue (
        tEplDllCalQueueInstance pDllCalQueue_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplDllCalDirectInstance *pDllCalDirectInstance =
            (tEplDllCalDirectInstance*)pDllCalQueue_p;

    if(pDllCalDirectInstance == NULL)
    {
        Ret = kEplInvalidInstanceParam;
        goto Exit;
    }

    //empty the buffer
    pDllCalDirectInstance->uiFrameSize_m = EPL_DLLCALDIRECT_TXBUF_EMPTY;

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

