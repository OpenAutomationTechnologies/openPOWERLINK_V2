/**
********************************************************************************
\file   errsigk.c

\brief  Implementation of kernel error signalling module

This module implements the kernel part of the error signalling module.
It is responsible for storing and sending the errors on CN to MN.

\ingroup module_errsigk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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
#include <oplk/nmt.h>
#include <oplk/benchmark.h>
#include <oplk/obd.h>
#include <oplk/ami.h>
#include <kernel/eventk.h>
#include <kernel/dllk.h>

#include <common/errhnd.h>
#include <kernel/errhndk.h>
#include <kernel/errsigk.h>

#include "errhndkcal.h"


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
//          P R I V A T E   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define MAX_STATUS_FRAMES            3
#define MAX_STATUS_ENTRY_PER_BUFFER  5
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief  Instance of kernel error signaller

The structure defines the instance variables of the kernel error signaller
*/
typedef struct sErrSigkInstance
{
    tErrSigkBufferStatus    status;               ///< current status of the Error Status Buffer
    tErrSigkBuffer*         errorBufferHead;      ///< The first Error Status Buffer
    tErrSigkBuffer*         pCurrentErrorBuffer;  ///< Current Error Status Buffer used for storing status entries
    tErrSigkBuffer*         pReservedErrorBuffer; ///< Reserved Error Status Buffer for storing any sudden error status changes
}tErrSigkInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tErrSigkInstance instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError allocateErrStatusBuffers(void);
static tOplkError freeErrStatusBuffers(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel error signaller module

The function initializes the kernel error signaller module.

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tOplkError errsigk_init(void)
{
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    instance_l.status = kNoBuffer;
    instance_l.errorBufferHead = NULL;
    instance_l.pCurrentErrorBuffer = NULL;
    instance_l.pReservedErrorBuffer = NULL;

    return kErrorOk;
}


//------------------------------------------------------------------------------
/**
\brief    Stops kernel error signaller module

The function stops the kernel error signaller module.

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tOplkError errsigk_exit(void)
{
    tOplkError      ret;

    ret = kErrorOk;
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    if (instance_l.status != kNoBuffer)
    {
        ret = freeErrStatusBuffers();
        if ( ret != kErrorOk)
        {
            goto Exit;
        }
    }
    instance_l.status = kNoBuffer;
    instance_l.errorBufferHead = NULL;
    instance_l.pCurrentErrorBuffer = NULL;
    instance_l.pReservedErrorBuffer = NULL;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Creates buffers kernel error signaller module

The function initialises error status buffers for the kernel
error signaller module when called by dllk and decides the ownership of the
buffers

\param      dllErrStatusBuffer      Pointer to where the dllk-owned error status
                                    buffer shall be stored

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tOplkError errsigk_createErrStatusBuffers(tErrSigkBuffer** ppdllErrStatusBuffer_p)
{
    tOplkError      ret;

    ret = kErrorOk;

    if (instance_l.status != kNoBuffer)
    {
        ret = kErrorInvalidEvent;
        goto Exit;
    }
    ret = allocateErrStatusBuffers();

    if (ret != kErrorOk)
    {
        goto Exit;
    }
    instance_l.status = kBuffersEmpty;
    (*ppdllErrStatusBuffer_p) = instance_l.errorBufferHead;
    (*ppdllErrStatusBuffer_p)->bufOwner = kOwnerDll;
    instance_l.pCurrentErrorBuffer = (*ppdllErrStatusBuffer_p)->pNextErrorBuffer;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Deallocates the buffers in kernel error signaller module

The function deallocates error status buffers and resets the resets the
error signalling module structure

\param      dllErrStatusBuffer      Pointer to where the dllk-owned error status
                                    buffer

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tOplkError errsigk_cleanErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer)
{
    tOplkError      ret;

    ret = kErrorOk;

    if (instance_l.status == kNoBuffer)
    {
        ret = kErrorInvalidEvent;
        goto Exit;
    }
    ret = freeErrStatusBuffers();

    if (ret != kErrorOk)
    {
        goto Exit;
    }
    instance_l.status = kNoBuffer;
    (*dllErrStatusBuffer) = NULL;
    instance_l.errorBufferHead = NULL;
    instance_l.pCurrentErrorBuffer = NULL;

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name    private functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Allocates the buffers for error signaller module

The function allocates the error status buffers.

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

static tOplkError allocateErrStatusBuffers(void)
{
    tOplkError      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* previousErrSigkBuffer;

    ret = kErrorOk;

    currentErrSigkBuffer = (tErrSigkBuffer*) OPLK_MALLOC(sizeof (tErrSigkBuffer));

    if (currentErrSigkBuffer == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    OPLK_MEMSET(currentErrSigkBuffer, 0, sizeof(tErrSigkBuffer));

    currentErrSigkBuffer->bufOwner = kOwnerReserved;
    instance_l.pReservedErrorBuffer = currentErrSigkBuffer;

    for (loopCount = 1; loopCount < MAX_STATUS_FRAMES; loopCount++)
    {
        currentErrSigkBuffer = (tErrSigkBuffer*) OPLK_MALLOC(sizeof (tErrSigkBuffer));

        if (currentErrSigkBuffer == NULL)
        {
            ret = kErrorNoResource;
            break;
        }

        OPLK_MEMSET(currentErrSigkBuffer,0,sizeof (tErrSigkBuffer));
        currentErrSigkBuffer->bufOwner = kOwnerErrSigk;
        if (loopCount == 1)
        {
            instance_l.errorBufferHead = currentErrSigkBuffer;
        }
        else
        {
            if (loopCount == (MAX_STATUS_FRAMES - 1))
            {
                currentErrSigkBuffer->pNextErrorBuffer = instance_l.errorBufferHead;
            }

            previousErrSigkBuffer->pNextErrorBuffer = currentErrSigkBuffer;
        }
        previousErrSigkBuffer = currentErrSigkBuffer;
    }

Exit:

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    Deallocates the buffers for error signaller module

The function deallocates the error status buffers.

\return Returns a tOplkError error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

static tOplkError freeErrStatusBuffers(void)
{
    tOplkError      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* nextErrSigkBuffer;

    ret = kErrorOk;

    currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.pReservedErrorBuffer;

    if (currentErrSigkBuffer != NULL)
    {
       OPLK_FREE(currentErrSigkBuffer);
    }
    instance_l.pReservedErrorBuffer = NULL;

    for (loopCount = 1; loopCount < (MAX_STATUS_FRAMES - 1); loopCount++)
    {

        if (loopCount == 1)
        {
            currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.errorBufferHead;
        }

        nextErrSigkBuffer = currentErrSigkBuffer->pNextErrorBuffer;

        if (currentErrSigkBuffer != NULL)
        {
           OPLK_FREE(currentErrSigkBuffer);
        }
        currentErrSigkBuffer = nextErrSigkBuffer;
    }

    return ret;
}
/// \}
