/**
********************************************************************************
\file   processimage.c

\brief  Process Image Functions

This source file contains the implementation of the process image functions.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplk.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    tOplkApiProcessImage     inputImage;
    tOplkApiProcessImage     outputImage;
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

\return The function returns a tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_allocProcessImage(UINT sizeProcessImageIn_p, UINT sizeProcessImageOut_p)
{
    tOplkError      ret = kErrorOk;

    TRACE("%s(): Alloc(%u, %u)\n", __func__, sizeProcessImageIn_p,
                                   sizeProcessImageOut_p);

    if ((instance_l.inputImage.pImage != NULL)|| (instance_l.outputImage.pImage != NULL))
    {
        ret = kErrorApiPIAlreadyAllocated;
        goto Exit;
    }

    instance_l.inputImage.pImage = OPLK_MALLOC(sizeProcessImageIn_p);
    if (instance_l.inputImage.pImage == NULL)
    {
        ret = kErrorApiPIOutOfMemory;
        goto Exit;
    }
    instance_l.inputImage.imageSize = sizeProcessImageIn_p;

    instance_l.outputImage.pImage = OPLK_MALLOC(sizeProcessImageOut_p);
    if (instance_l.outputImage.pImage == NULL)
    {
        ret = kErrorApiPIOutOfMemory;
        goto Exit;
    }
    instance_l.outputImage.imageSize = sizeProcessImageOut_p;

    TRACE("%s: Alloc(%p, %u, %p, %u)\n", __func__,
          instance_l.inputImage.pImage,  instance_l.inputImage.imageSize,
          instance_l.outputImage.pImage, instance_l.outputImage.imageSize);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the allocated process images

The function frees the allocated process images

\return The function returns a tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------

tOplkError oplk_freeProcessImage(void)
{
    tOplkError      Ret = kErrorOk;

    if ((instance_l.inputImage.pImage == NULL) &&
        (instance_l.outputImage.pImage == NULL))
    {
        goto Exit;
    }

    instance_l.inputImage.imageSize = 0;
    instance_l.outputImage.imageSize = 0;

    OPLK_FREE(instance_l.inputImage.pImage);
    instance_l.inputImage.pImage = NULL;
    OPLK_FREE(instance_l.outputImage.pImage);
    instance_l.outputImage.pImage = NULL;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Link object in process image

The function links an object in the OD into a location in the process image.

\param  objIndex_p              The object index of the object to link.
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

\return The function returns a tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_linkProcessImageObject(UINT objIndex_p, UINT firstSubindex_p,
                                       UINT offsetPI_p, BOOL fOutputPI_p,
                                       tObdSize entrySize_p, UINT* pVarEntries_p)
{
    tOplkError      ret = kErrorOk;
    void*           pVar;

    if (pVarEntries_p == NULL)
        return kErrorApiInvalidParam;

    if ((instance_l.inputImage.pImage == NULL) || (instance_l.outputImage.pImage == NULL))
        return kErrorApiPINotAllocated;

    if (fOutputPI_p)
    {
        pVar = ((BYTE*)instance_l.outputImage.pImage) + offsetPI_p;
        if ((offsetPI_p + entrySize_p) > instance_l.outputImage.imageSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kErrorApiPISizeExceeded;
        }
        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.outputImage.imageSize)
        {   // limit the number of entries
            *pVarEntries_p = (instance_l.outputImage.imageSize - offsetPI_p) / entrySize_p;
        }
    }
    else
    {
        pVar = ((BYTE*)instance_l.inputImage.pImage) + offsetPI_p;
        if ((offsetPI_p + entrySize_p) > instance_l.inputImage.imageSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kErrorApiPISizeExceeded;
        }
        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.inputImage.imageSize)
        {   // limit the number of entries
            *pVarEntries_p = (instance_l.inputImage.imageSize - offsetPI_p) / entrySize_p;
        }
    }

    ret = oplk_linkObject(objIndex_p, pVar, pVarEntries_p, &entrySize_p, firstSubindex_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exchange input process image

The function exchanges the input process image.

\return The function returns a tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_exchangeProcessImageIn(void)
{
    tOplkError      ret;

    if (instance_l.inputImage.pImage != NULL)
        ret = pdou_copyTxPdoFromPi();
    else
        ret = kErrorApiPINotAllocated;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exchange output process image

The function exchanges the output process image.

\return The function returns a tOplkError error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_exchangeProcessImageOut(void)
{
    tOplkError      ret;

    if (instance_l.outputImage.pImage != NULL)
        ret = pdou_copyRxPdoToPi();
    else
        ret = kErrorApiPINotAllocated;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to input process image

The function returns the pointer to the input process image.

\return The function returns a pointer to the input process image.

\ingroup module_api
*/
//------------------------------------------------------------------------------
void* oplk_getProcessImageIn(void)
{
    return instance_l.inputImage.pImage;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to output process image

The function returns the pointer to the output process image.

\return The function returns a pointer to the output process image.

\ingroup module_api
*/
//------------------------------------------------------------------------------
void* oplk_getProcessImageOut(void)
{
    return instance_l.outputImage.pImage;
}

