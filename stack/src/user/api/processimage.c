/**
********************************************************************************
\file   processimage.c

\brief  Process Image Functions

This source file contains the implementation of the process image functions.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <user/pdou.h>
#include <user/ctrlu.h>

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
\brief Structure describing a process image instance

This structure describes a process image instance consisting of an input and an
output process image.
*/
typedef struct
{
    tOplkApiProcessImage        inputImage;     ///< Input process image
    tOplkApiProcessImage        outputImage;    ///< Output process image
} tApiProcessImageInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tApiProcessImageInstance instance_l =
{
    { NULL, 0 },
    { NULL, 0 }
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Allocate process images

The function allocates memory for the input and output process images.

\param[in]      sizeProcessImageIn_p    Size for input process image.
\param[in]      sizeProcessImageOut_p   Size for output process image.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                        Process images are successfully allocated.
\retval kErrorApiPIAlreadyAllocated     Process images were already allocated.
\retval kErrorApiPIOutOfMemory          Process images could not be allocated.
\retval kErrorApiNotInitialized         openPOWERLINK stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_allocProcessImage(size_t sizeProcessImageIn_p,
                                  size_t sizeProcessImageOut_p)
{
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_ALWAYS_TRACE("%s(): Alloc(%lu, %lu)\n",
                           __func__,
                           (ULONG)sizeProcessImageIn_p,
                           (ULONG)sizeProcessImageOut_p);

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if ((instance_l.inputImage.pImage != NULL) ||
        (instance_l.outputImage.pImage != NULL))
        return kErrorApiPIAlreadyAllocated;

    if (sizeProcessImageIn_p != 0)
    {
        instance_l.inputImage.pImage = OPLK_MALLOC(sizeProcessImageIn_p);
        if (instance_l.inputImage.pImage == NULL)
            return kErrorApiPIOutOfMemory;

        instance_l.inputImage.imageSize = sizeProcessImageIn_p;
        OPLK_MEMSET(instance_l.inputImage.pImage, 0x00, sizeProcessImageIn_p);
    }

    if (sizeProcessImageOut_p != 0)
    {
        instance_l.outputImage.pImage = OPLK_MALLOC(sizeProcessImageOut_p);
        if (instance_l.outputImage.pImage == NULL)
        {
            // Output image allocation failed, therefore we free input image to be consistent
            oplk_freeProcessImage();
            return kErrorApiPIOutOfMemory;
        }

        instance_l.outputImage.imageSize = sizeProcessImageOut_p;
        OPLK_MEMSET(instance_l.outputImage.pImage, 0x00, sizeProcessImageOut_p);
    }

    DEBUG_LVL_ALWAYS_TRACE("%s: Alloc(%p, %lu, %p, %lu)\n",
                           __func__,
                           instance_l.inputImage.pImage,
                           (ULONG)instance_l.inputImage.imageSize,
                           instance_l.outputImage.pImage,
                           (ULONG)instance_l.outputImage.imageSize);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the allocated process images

The function frees the allocated process images

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                        Process images are successfully freed.
\retval kErrorApiNotInitialized         openPOWERLINK stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_freeProcessImage(void)
{
    tOplkError  ret = kErrorOk;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if (instance_l.inputImage.pImage != NULL)
    {
        OPLK_FREE(instance_l.inputImage.pImage);
        instance_l.inputImage.pImage = NULL;
        instance_l.inputImage.imageSize = 0;
    }

    if (instance_l.outputImage.pImage != NULL)
    {
        OPLK_FREE(instance_l.outputImage.pImage);
        instance_l.outputImage.pImage = NULL;
        instance_l.outputImage.imageSize = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Link object in process image

The function links an object in the OD into a location in the process image.

\param[in]      objIndex_p          The object index of the object to link.
\param[in]      firstSubindex_p     The sub-index of the object where the first
                                    variable should be linked to.
\param[in]      offsetPI_p          The offset of the first process variable in the
                                    process image.
\param[in]      fOutputPI_p         Determines if input image or output image should
                                    be used: TRUE = output image, FALSE = input image
\param[in]      entrySize_p         The size of one process variable.
\param[in,out]  pVarEntries_p       The number of process variables, which shall be
                                    linked to the object dictionary. It returns the
                                    actual number of process variables which were
                                    linked to the object dictionary.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    Object is successfully linked.
\retval kErrorApiInvalidParam       Invalid parameters specified.
\retval kErrorApiPISizeExceeded     Size of process image is exceeded.
\retval kErrorApiNotInitialized     openPOWERLINK stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_linkProcessImageObject(UINT objIndex_p,
                                       UINT firstSubindex_p,
                                       size_t offsetPI_p,
                                       BOOL fOutputPI_p,
                                       tObdSize entrySize_p,
                                       UINT* pVarEntries_p)
{
    tOplkError  ret = kErrorOk;
    void*       pVar;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

    if (pVarEntries_p == NULL)
        return kErrorApiInvalidParam;

    if (fOutputPI_p)
    {
        if (instance_l.outputImage.pImage == NULL)
            return kErrorApiPISizeExceeded;     // If there's no image, we are too big

        pVar = ((UINT8*)instance_l.outputImage.pImage) + offsetPI_p;

        if ((offsetPI_p + entrySize_p) > instance_l.outputImage.imageSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kErrorApiPISizeExceeded;
        }

        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.outputImage.imageSize)
        {   // limit the number of entries
            *pVarEntries_p = (UINT)((instance_l.outputImage.imageSize - offsetPI_p) / entrySize_p);
        }
    }
    else
    {
        if (instance_l.inputImage.pImage == NULL)
            return kErrorApiPISizeExceeded;     // If there's no image, we are too big

        pVar = ((UINT8*)instance_l.inputImage.pImage) + offsetPI_p;

        if ((offsetPI_p + entrySize_p) > instance_l.inputImage.imageSize)
        {   // at least one entry should fit into the PI, but it doesn't
            return kErrorApiPISizeExceeded;
        }

        if ((offsetPI_p + (*pVarEntries_p * entrySize_p)) > instance_l.inputImage.imageSize)
        {   // limit the number of entries
            *pVarEntries_p = (UINT)((instance_l.inputImage.imageSize - offsetPI_p) / entrySize_p);
        }
    }

    ret = oplk_linkObject(objIndex_p, pVar, pVarEntries_p, &entrySize_p, firstSubindex_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Exchange input process image

The function exchanges the input process image.

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    Input process image is successfully exchanged.
\retval kErrorApiPINotAllocated     Memory for process images is not allocated.
\retval kErrorApiNotInitialized     openPOWERLINK stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_exchangeProcessImageIn(void)
{
    tOplkError  ret;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

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

\return The function returns a \ref tOplkError error code.
\retval kErrorOk                    Output process image is successfully exchanged.
\retval kErrorApiPINotAllocated     Memory for process images is not allocated.
\retval kErrorApiNotInitialized     openPOWERLINK stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_exchangeProcessImageOut(void)
{
    tOplkError  ret;

    if (!ctrlu_stackIsInitialized())
        return kErrorApiNotInitialized;

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

\return The function returns a pointer to the input process image or NULL if the
        stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
void* oplk_getProcessImageIn(void)
{
    if (!ctrlu_stackIsInitialized())
        return NULL;

    return instance_l.inputImage.pImage;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to output process image

The function returns the pointer to the output process image.

\return The function returns a pointer to the output process image or NULL if
        the stack is not initialized.

\ingroup module_api
*/
//------------------------------------------------------------------------------
void* oplk_getProcessImageOut(void)
{
    if (!ctrlu_stackIsInitialized())
        return NULL;

    return instance_l.outputImage.pImage;
}

//------------------------------------------------------------------------------
/**
\brief  Setup process image

The function sets up the process image according to the CiA302-4 profile.

\deprecated The process image setup has to be done in the application since the
            object dictionary is defined in the application.

\return The function returns a \ref tOplkError error code.
\retval kErrorApiNotSupported       This call is not supported.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tOplkError oplk_setupProcessImage(void)
{
    DEBUG_LVL_ALWAYS_TRACE("Since openPOWERLINK 2.5.0 the process image setup is done in the application.\n");

    return kErrorApiNotSupported;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
