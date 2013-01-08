/**
********************************************************************************
\file   processimage.c

\brief  Process Image Functions

This source file contains the implementation of the process image functions.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <Epl.h>

#include <user/pdou.h>

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
#ifndef EPL_API_PI_MAX_SIZE
#define EPL_API_PI_MAX_SIZE         65536
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    tEplApiProcessImage m_In;
    tEplApiProcessImage m_Out;

    //tEplSyncCb          m_pfnOrgCbSync;
} tApiProcessImageInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tApiProcessImageInstance  instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Allocate process images

The function allocates the input and output process images

\param  sizeProcessImageIn_p          Size for input process image
\param  sizeProcessImageOut_p         Size for output process image

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC api_processImageAlloc(UINT sizeProcessImageIn_p, UINT sizeProcessImageOut_p)
{
    tEplKernel      ret = kEplSuccessful;

    TRACE("%s(): Alloc(%u, %u)\n", __func__, sizeProcessImageIn_p,
                                   sizeProcessImageOut_p);

    if ((instance_l.m_In.m_pImage != NULL)|| (instance_l.m_Out.m_pImage != NULL))
    {
        ret = kEplApiPIAlreadyAllocated;
        goto Exit;
    }

    instance_l.m_In.m_pImage = EPL_MALLOC(sizeProcessImageIn_p);
    if (instance_l.m_In.m_pImage == NULL)
    {
        ret = kEplApiPIOutOfMemory;
        goto Exit;
    }
    instance_l.m_In.m_uiSize = sizeProcessImageIn_p;

    instance_l.m_Out.m_pImage = EPL_MALLOC(sizeProcessImageOut_p);
    if (instance_l.m_Out.m_pImage == NULL)
    {
        ret = kEplApiPIOutOfMemory;
        goto Exit;
    }
    instance_l.m_Out.m_uiSize = sizeProcessImageOut_p;

    TRACE("%s: Alloc(%p, %u, %p, %u)\n", __func__,
          instance_l.m_In.m_pImage,  instance_l.m_In.m_uiSize,
          instance_l.m_Out.m_pImage, instance_l.m_Out.m_uiSize);


Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the allocated process images

The function frees the allocated process images

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------

tEplKernel PUBLIC api_processImageFree(void)
{
    tEplKernel      Ret = kEplSuccessful;

    if ((instance_l.m_In.m_pImage == NULL) &&
        (instance_l.m_Out.m_pImage == NULL))
    {
        goto Exit;
    }

    instance_l.m_In.m_uiSize = 0;
    instance_l.m_Out.m_uiSize = 0;

    EPL_FREE(instance_l.m_In.m_pImage);
    instance_l.m_In.m_pImage = NULL;
    EPL_FREE(instance_l.m_Out.m_pImage);
    instance_l.m_Out.m_pImage = NULL;

Exit:
    return Ret;
}


//------------------------------------------------------------------------------
/**
\brief  Link object in process image

The function links an object in the OD into a location in the process image.

\param  objIndex_p              The Object index of the object to link.
\param  firstSubindex_p         The sub-index of the object where the first
                                variable should be linked to.
\param  offsetPI_p              The offset of the first process variable in the
                                process image.
\param  fOutputPI_p             Determines if input image or output image should
                                be used: TRUE = output image, FALSE = imput image
\param  entrySize_p             The size of one process variable.
\param  pVarEntries_p           The number of process variables, which shall be
                                linked to the object dictionary. It returns the
                                actual number of process variables which were
                                linked to the object dictionary.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------

tEplKernel PUBLIC api_processImageLinkObject(UINT objIndex_p, UINT firstSubindex_p,
                                             UINT offsetPI_p, BOOL fOutputPI_p,
                                             tEplObdSize entrySize_p, UINT* pVarEntries_p)
{
    tEplKernel      ret = kEplSuccessful;
    void*           pVar;

    if (pVarEntries_p == NULL)
        return kEplApiInvalidParam;

    if ((instance_l.m_In.m_pImage == NULL) || (instance_l.m_Out.m_pImage == NULL))
        return kEplApiPINotAllocated;

    if (fOutputPI_p)
    {
        pVar = ((BYTE*) instance_l.m_Out.m_pImage) + offsetPI_p;
        if ((offsetPI_p + entrySize_p) > instance_l.m_Out.m_uiSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kEplApiPISizeExceeded;
        }
        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.m_Out.m_uiSize)
        {   // limit the number of entries
            *pVarEntries_p = (instance_l.m_Out.m_uiSize - offsetPI_p) / entrySize_p;
        }
    }
    else
    {
        pVar = ((BYTE*) instance_l.m_In.m_pImage) + offsetPI_p;
        if ((offsetPI_p + entrySize_p) > instance_l.m_In.m_uiSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kEplApiPISizeExceeded;
        }
        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.m_In.m_uiSize)
        {   // limit the number of entries
            *pVarEntries_p = (instance_l.m_In.m_uiSize - offsetPI_p) / entrySize_p;
        }
    }

    ret = EplApiLinkObject(objIndex_p, pVar, pVarEntries_p, &entrySize_p, firstSubindex_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exchange input process image

The function exchanges the input process image.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel api_processImageExchangeIn(void)
{
    tEplKernel      ret;

    ret = pdou_copyTxPdoFromPi();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exchange output process image

The function exchanges the output process image.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel api_processImageExchangeOut(void)
{
    tEplKernel      ret;

    ret = pdou_copyRxPdoToPi();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to input process image

The function returns the pointer to the input process image.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void *api_processImageGetInputImage(void)
{
    return instance_l.m_In.m_pImage;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to output process image

The function returns the pointer to the output process image.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void *api_processImageGetOutputImage(void)
{
    return instance_l.m_Out.m_pImage;
}


