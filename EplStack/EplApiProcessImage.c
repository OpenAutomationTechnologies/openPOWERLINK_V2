/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for EPL API module (process image)

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/10/10 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "Epl.h"
#include "kernel/EplDllk.h"
#include "kernel/EplEventk.h"

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
#include <asm/uaccess.h>
#include <linux/completion.h>
#include <asm/current.h>
#endif

#if EPL_USE_SHAREDBUFF == FALSE
#error "EplApiProcessImage needs SharedBuff support"
#else
#include "SharedBuff.h"
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplApi                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#ifndef EPL_API_PI_BUFFER_ID_LO
#define EPL_API_PI_BUFFER_ID_LO     "EplApiPIJobQueueLo"
#endif

#ifndef EPL_API_PI_BUFFER_ID_HI
#define EPL_API_PI_BUFFER_ID_HI     "EplApiPIJobQueueHi"
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tEplApiProcessImageCopyJob  m_CopyJob;
    union
    {
        tEplEventSink           m_EventSink;
        // more OS specific event types
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        struct completion*      m_pCompletion;
#elif (TARGET_SYSTEM == _WIN32_)
        HANDLE                  m_hEvent;
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif
    } m_Event;

} tEplApiProcessImageCopyJobInt;


typedef struct
{
    tEplApiProcessImage m_In;
    tEplApiProcessImage m_Out;
    tShbInstance        m_ShbInstanceJobQueueLo;
    tShbInstance        m_ShbInstanceJobQueueHi;

    tEplSyncCb          m_pfnOrgCbSync;
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    struct task_struct* m_pCurrentTask;
#elif (TARGET_SYSTEM == _WIN32_)
    DWORD               m_dwCurrentThreadId;
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

} tEplApiProcessImageInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplApiProcessImageInstance  EplApiProcessImageInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageExchangeInt(
    tEplApiProcessImageCopyJob* pCopyJob_p);

static tEplKernel EplApiProcessImagePostCopyJob(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel EplApiProcessImageFetchCopyJob(
    unsigned int uiPriority_p,
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel EplApiProcessImageCreateCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel EplApiProcessImageWaitForCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel EplApiProcessImageDeleteCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel EplApiProcessImageSignalCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);

static tEplKernel PUBLIC EplApiProcessImageCbSync(void);



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageAlloc()
//
// Description: allocates a dynamic process image
//
// Parameters:  uiSizeProcessImageIn_p  = size of input process image
//              uiSizeProcessImageOut_p = size of output process image
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageAlloc(
    unsigned int uiSizeProcessImageIn_p,
    unsigned int uiSizeProcessImageOut_p,
    unsigned int uiQueueEntriesLo_p,
    unsigned int uiQueueEntriesHi_p)
{
tEplKernel      Ret = kEplSuccessful;
tShbError       ShbError;

    if ((EplApiProcessImageInstance_g.m_In.m_pImage != NULL)
        || (EplApiProcessImageInstance_g.m_Out.m_pImage != NULL))
    {
        Ret = kEplApiPIAlreadyAllocated;
        goto Exit;
    }

    EplApiProcessImageInstance_g.m_In.m_pImage = EPL_MALLOC(uiSizeProcessImageIn_p);
    if (EplApiProcessImageInstance_g.m_In.m_pImage == NULL)
    {
        Ret = kEplApiPIOutOfMemory;
        goto Exit;
    }
    EplApiProcessImageInstance_g.m_In.m_uiSize = uiSizeProcessImageIn_p;

    EplApiProcessImageInstance_g.m_Out.m_pImage = EPL_MALLOC(uiSizeProcessImageOut_p);
    if (EplApiProcessImageInstance_g.m_Out.m_pImage == NULL)
    {
        Ret = kEplApiPIOutOfMemory;
        goto Exit;
    }
    EplApiProcessImageInstance_g.m_Out.m_uiSize = uiSizeProcessImageOut_p;

    ShbError = ShbCirAllocBuffer (uiQueueEntriesLo_p * sizeof (tEplApiProcessImageCopyJobInt), EPL_API_PI_BUFFER_ID_LO,
        &EplApiProcessImageInstance_g.m_ShbInstanceJobQueueLo, &fShbNewCreated);
    // returns kShbOk, kShbOpenMismatch, kShbOutOfMem or kShbInvalidArg
    if (ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    ShbError = ShbCirAllocBuffer (uiQueueEntriesHi_p * sizeof (tEplApiProcessImageCopyJobInt), EPL_API_PI_BUFFER_ID_HI,
        &EplApiProcessImageInstance_g.m_ShbInstanceJobQueueHi, &fShbNewCreated);
    // returns kShbOk, kShbOpenMismatch, kShbOutOfMem or kShbInvalidArg
    if (ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    EplApiProcessImageInstance_g.m_pfnOrgCbSync = EplDllkRegSyncHandler(EplApiProcessImageCbSync);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageFree()
//
// Description: frees the dynamic process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageFree(void)
{
tEplKernel      Ret = kEplSuccessful;
tShbError       ShbError;
tEplApiProcessImageCopyJobInt   CopyJob;

    if ((EplApiProcessImageInstance_g.m_In.m_pImage == NULL)
        && (EplApiProcessImageInstance_g.m_Out.m_pImage == NULL))
    {
        goto Exit;
    }

    EplDllkRegSyncHandler(EplApiProcessImageInstance_g.m_pfnOrgCbSync);

    EplApiProcessImageInstance_g.m_In.m_uiSize = 0;
    EplApiProcessImageInstance_g.m_Out.m_uiSize = 0;

    // signal completion to all queued copy jobs
    for (;;)
    {
        Ret = EplApiProcessImageFetchCopyJob(0, &CopyJob);
        if (Ret == kEplSuccessful)
        {
            Ret = EplApiProcessImageSignalCompletion(&CopyJob);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {
            break;
        }
    }

    for (;;)
    {
        Ret = EplApiProcessImageFetchCopyJob(1, &CopyJob);
        if (Ret == kEplSuccessful)
        {
            Ret = EplApiProcessImageSignalCompletion(&CopyJob);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {
            break;
        }
    }

    EPL_FREE(EplApiProcessImageInstance_g.m_In.m_pImage);
    EplApiProcessImageInstance_g.m_In.m_pImage = NULL;
    EPL_FREE(EplApiProcessImageInstance_g.m_Out.m_pImage);
    EplApiProcessImageInstance_g.m_Out.m_pImage = NULL;

    ShbError = ShbCirReleaseBuffer (EplApiProcessImageInstance_g.m_ShbInstanceJobQueueLo);
    if (ShbError != kShbOk)
    {
        Ret = kEplNoResource;
    }
    EplApiProcessImageInstance_g.m_ShbInstanceJobQueueLo = NULL;

    ShbError = ShbCirReleaseBuffer (EplApiProcessImageInstance_g.m_ShbInstanceJobQueueHi);
    if (ShbError != kShbOk)
    {
        Ret = kEplNoResource;
    }
    EplApiProcessImageInstance_g.m_ShbInstanceJobQueueHi = NULL;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageLinkObject()
//
// Description: link process variable from process image to object in OD
//
// Parameters:  uiObjIndex_p            = object index
//              uiFirstSubindex_p       = sub-index of object where first variable shall be linked to
//              uiOffsetPI_p            = offset of first process variable in process image
//              fOutputPI_p             = FALSE, input process image
//                                        TRUE, output process image
//              EntrySize_p             = size of one process variable
//              puiVarEntries_p         = [IN] number of process variables, which shall be linked to OD
//                                        [OUT] actual number of process variable, which were linked to OD
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageLinkObject(
    unsigned int    uiObjIndex_p,
    unsigned int    uiFirstSubindex_p,
    unsigned int    uiOffsetPI_p,
    BOOL            fOutputPI_p,
    tEplObdSize     EntrySize_p,
    unsigned int*   puiVarEntries_p)
{
tEplKernel      Ret = kEplSuccessful;
void*           pVar;

    if (puiVarEntries_p == NULL)
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if ((EplApiProcessImageInstance_g.m_In.m_pImage == NULL)
        || (EplApiProcessImageInstance_g.m_Out.m_pImage == NULL))
    {
        Ret = kEplApiPINotAllocated;
        goto Exit;
    }

    if (fOutputPI_p == FALSE)
    {   // input PI
        pVar = ((BYTE*) EplApiProcessImageInstance_g.m_In.m_pImage) + uiOffsetPI_p;
        if ((uiOffsetPI_p + (*puiVarEntries_p * EntrySize_p)) > EplApiProcessImageInstance_g.m_In.m_uiSize)
        {
            Ret = kEplApiPISizeExceeded;
            goto Exit;
        }
    }
    else
    {   // output PI
        pVar = ((BYTE*) EplApiProcessImageInstance_g.m_Out.m_pImage) + uiOffsetPI_p;
        if ((uiOffsetPI_p + (*puiVarEntries_p * EntrySize_p)) > EplApiProcessImageInstance_g.m_Out.m_uiSize)
        {
            Ret = kEplApiPISizeExceeded;
            goto Exit;
        }
    }

    Ret = EplApiLinkObject(uiObjIndex_p, pVar, puiVarEntries_p, &EntrySize_p, uiFirstSubindex_p);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageExchange()
//
// Description: Perform a copy job.
//
// Parameters:  pCopyJob_p              = pointer to copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchange(
    tEplApiProcessImageCopyJob* pCopyJob_p)
{
tEplKernel                      Ret = kEplSuccessful;
tEplApiProcessImageCopyJobInt   IntCopyJob;

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    if (EplApiProcessImageInstance_g.m_pCurrentTask == get_current())
#elif (TARGET_SYSTEM == _WIN32_)
    if (EplApiProcessImageInstance_g.m_dwCurrentThreadId == GetCurrentThreadId())
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif
    {
        Ret = EplApiProcessImageExchangeInt(pCopyJob_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        goto Exit;
    }

    if ((EplApiProcessImageInstance_g.m_In.m_uiSize == 0)
        || (EplApiProcessImageInstance_g.m_Out.m_uiSize = 0))
    {   // the process image has been freed
        // therefor, indicate shutdown to application thread
        Ret = kEplShutdown;
        goto Exit;
    }

    IntCopyJob.m_CopyJob = *pCopyJob_p;

    if (pCopyJob_p->m_fNonBlocking == FALSE)
    {
        Ret = EplApiProcessImageCreateCompletion(&IntCopyJob);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    Ret = EplApiProcessImagePostCopyJob(&IntCopyJob);

    if (pCopyJob_p->m_fNonBlocking == FALSE)
    {
        if (Ret == kEplSuccessful)
        {
            EplApiProcessImageWaitForCompletion(&IntCopyJob);

            if ((EplApiProcessImageInstance_g.m_In.m_uiSize == 0)
                || (EplApiProcessImageInstance_g.m_Out.m_uiSize = 0))
            {   // in the mean time the process image has been freed
                // therefor, indicate shutdown to application thread
                Ret = kEplShutdown;
            }
        }

        EplApiProcessImageDeleteCompletion(&IntCopyJob);
    }

Exit:
    return Ret;
}

#if 0
//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSetup()
//
// Description: sets up static process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageSetup(void)
{
tEplKernel      Ret = kEplSuccessful;
#if ((EPL_API_PROCESS_IMAGE_SIZE_IN > 0) || (EPL_API_PROCESS_IMAGE_SIZE_OUT > 0))
unsigned int    uiVarEntries;
tEplObdSize     ObdSize;
#endif

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2000,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2001,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2010,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2011,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2020,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_IN / ObdSize;
    Ret = EplApiLinkObject(
                            0x2021,
                            EplApiProcessImageInstance_g.m_abProcessImageInput,
                            &uiVarEntries,
                            &ObdSize,
                            1);
#endif

#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2030,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT;
    ObdSize = 1;
    Ret = EplApiLinkObject(
                            0x2031,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2040,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 2;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2041,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2050,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);

    ObdSize = 4;
    uiVarEntries = EPL_API_PROCESS_IMAGE_SIZE_OUT / ObdSize;
    Ret = EplApiLinkObject(
                            0x2051,
                            EplApiProcessImageInstance_g.m_abProcessImageOutput,
                            &uiVarEntries,
                            &ObdSize,
                            1);
#endif

    return Ret;
}

//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeIn()
//
// Description: replaces passed input process image with the one of EPL stack
//
// Parameters:  pPI_p                   = input process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeIn(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;

#if EPL_API_PROCESS_IMAGE_SIZE_IN > 0
    #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        copy_to_user(pPI_p->m_pImage,
            EplApiProcessImageInstance_g.m_abProcessImageInput,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageInput)));
    #else
        EPL_MEMCPY(pPI_p->m_pImage,
            EplApiProcessImageInstance_g.m_abProcessImageInput,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageInput)));
    #endif
#endif

    return Ret;
}


//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeOut()
//
// Description: copies passed output process image to EPL stack.
//
// Parameters:  pPI_p                   = output process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeOut(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;

#if EPL_API_PROCESS_IMAGE_SIZE_OUT > 0
    #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        copy_from_user(EplApiProcessImageInstance_g.m_abProcessImageOutput,
            pPI_p->m_pImage,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageOutput)));
    #else
        EPL_MEMCPY(EplApiProcessImageInstance_g.m_abProcessImageOutput,
            pPI_p->m_pImage,
            min(pPI_p->m_uiSize, sizeof (EplApiProcessImageInstance_g.m_abProcessImageOutput)));
    #endif
#endif

    return Ret;
}
#endif


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageExchangeInt()
//
// Description: Perform the actual copy job.
//
// Parameters:  pCopyJob_p              = pointer to copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageExchangeInt(
    tEplApiProcessImageCopyJob* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (pCopyJob_p->m_In.m_uiSize > 0)
    {
        if ((pCopyJob_p->m_In.m_pPart != NULL)
            && (EplApiProcessImageInstance_g.m_In.m_pImage != NULL)
            && ((pCopyJob_p->m_In.m_uiOffset + pCopyJob_p->m_In.m_uiSize) <= EplApiProcessImageInstance_g.m_In.m_uiSize))
        {
        void*   pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_In.m_pImage) + pCopyJob_p->m_In.m_uiOffset;

            #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
                copy_to_user(pCopyJob_p->m_In.m_pPart,
                    pPIVar,
                    pCopyJob_p->m_In.m_uiSize);
            #else
                EPL_MEMCPY(pCopyJob_p->m_In.m_pPart,
                    pPIVar,
                    pCopyJob_p->m_In.m_uiSize);
            #endif
        }
    }

    if (pCopyJob_p->m_Out.m_uiSize > 0)
    {
        if ((pCopyJob_p->m_Out.m_pPart != NULL)
            && (EplApiProcessImageInstance_g.m_Out.m_pImage != NULL)
            && ((pCopyJob_p->m_Out.m_uiOffset + pCopyJob_p->m_Out.m_uiSize) <= EplApiProcessImageInstance_g.m_Out.m_uiSize))
        {
        void*   pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_Out.m_pImage) + pCopyJob_p->m_Out.m_uiOffset;

            #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
                copy_from_user(pPIVar,
                    pCopyJob_p->m_Out.m_pPart,
                    pCopyJob_p->m_Out.m_uiSize);
            #else
                EPL_MEMCPY(pPIVar,
                    pCopyJob_p->m_Out.m_pPart,
                    pCopyJob_p->m_Out.m_uiSize);
            #endif
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImagePostCopyJob()
//
// Description: Post the internal copy job to the queue.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImagePostCopyJob(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;
tShbError       ShbError;
tShbInstance    ShbInstance;

    if (pCopyJob_p == NULL)
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if (pCopyJob_p->m_CopyJob.m_uiPriority == 0)
    {
        ShbInstance = EplApiProcessImageInstance_g.m_ShbInstanceJobQueueHi;
    }
    else
    {
        ShbInstance = EplApiProcessImageInstance_g.m_ShbInstanceJobQueueLo;
    }

    ShbError = ShbCirWriteDataBlock(ShbInstance, pCopyJob_p, sizeof (*pCopyJob_p));
    if (ShbError != kShbOk)
    {
        Ret = kEplApiPIJobQueueFull;
        goto Exit;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageFetchCopyJob()
//
// Description: Fetches an internal copy job from the specified queue.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageFetchCopyJob(
    unsigned int uiPriority_p,
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;
tShbError       ShbError;
tShbInstance    ShbInstance;
unsigned long   ulCopyJobCount = 0;
unsigned long   ulCopyJobSize = 0;

    if (pCopyJob_p == NULL)
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if (uiPriority_p == 0)
    {
        ShbInstance = EplApiProcessImageInstance_g.m_ShbInstanceJobQueueHi;
    }
    else
    {
        ShbInstance = EplApiProcessImageInstance_g.m_ShbInstanceJobQueueLo;
    }

    ShbError = ShbCirGetReadBlockCount(ShbInstance, &ulCopyJobCount);
    if (ShbError != kShbOk)
    {
        Ret = kEplNoResource;
        goto Exit;
    }
    if (ulCopyJobCount > 0)
    {
        ShbError = ShbCirReadDataBlock (ShbInstanceTxSync, pCopyJob_p, sizeof (*pCopyJob_p), &ulCopyJobSize);
        if (ShbError != kShbOk)
        {
            Ret = kEplNoResource;
            goto Exit;
        }
        if (sizeof (*pCopyJob_p) != ulCopyJobSize)
        {
            Ret = kEplApiPIInvalidJobSize;
            goto Exit;
        }
    }
    else
    {
        Ret = kEplApiPIJobQueueEmpty;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageCreateCompletion()
//
// Description: Create completion for the copy job.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageCreateCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    pCopyJob_p->m_Event.m_pCompletion = EPL_MALLOC(sizeof (*pCopyJob_p->m_Event.m_pCompletion));
    if (pCopyJob_p->m_Event.m_pCompletion == NULL)
    {
        Ret = kEplApiPIOutOfMemory;
        goto Exit;
    }
    init_completion(pCopyJob_p->m_Event.m_pCompletion);

#elif (TARGET_SYSTEM == _WIN32_)
    pCopyJob_p->m_Event.m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageWaitForCompletion()
//
// Description: Wait for completion of the copy job.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageWaitForCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    wait_for_complete(pCopyJob_p->m_Event.m_pCompletion);
#elif (TARGET_SYSTEM == _WIN32_)
    WaitForSingleObject(pCopyJob_p->m_Event.m_hEvent, INFINITE);
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageDeleteCompletion()
//
// Description: Delete completion of the copy job.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageDeleteCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    EPL_FREE(pCopyJob_p->m_Event.m_pCompletion);
    pCopyJob_p->m_Event.m_pCompletion = NULL;
#elif (TARGET_SYSTEM == _WIN32_)
    CloseHandle(pCopyJob_p->m_Event.m_hEvent);
    pCopyJob_p->m_Event.m_hEvent = -1;
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

//Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSignalCompletion()
//
// Description: Signal completion of the copy job.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageSignalCompletion(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (pCopyJob_p->m_CopyJob.m_fNonBlocking == FALSE)
    {
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        complete(pCopyJob_p->m_Event.m_pCompletion);
#elif (TARGET_SYSTEM == _WIN32_)
        SetEvent(pCopyJob_p->m_Event.m_hEvent);
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif
    }
    else
    {
        // $$$ post special event to event queue
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageCbSync()
//
// Description: Sync callback function.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplApiProcessImageCbSync(void)
{
tEplKernel                      Ret = kEplSuccessful;
tEplApiProcessImageCopyJobInt   CopyJob;

    // memorize current thread, so that Exchange() can detect if it is called from within this function
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_pCurrentTask = get_current();
#elif (TARGET_SYSTEM == _WIN32_)
    EplApiProcessImageInstance_g.m_dwCurrentThreadId = GetCurrentThreadId();
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

    if (EplApiProcessImageInstance_g.m_pfnOrgCbSync != NULL)
    {
        Ret = EplApiProcessImageInstance_g.m_pfnOrgCbSync();
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    Ret = EplApiProcessImageFetchCopyJob(0, &CopyJob);
    if (Ret == kEplSuccessful)
    {
        Ret = EplApiProcessImageExchangeInt(&CopyJob.m_CopyJob);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EplApiProcessImageSignalCompletion(&CopyJob);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    else if (Ret != kEplApiPIJobQueueEmpty)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageFetchCopyJob(1, &CopyJob);
    if (Ret == kEplSuccessful)
    {
        Ret = EplApiProcessImageExchangeInt(&CopyJob.m_CopyJob);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EplApiProcessImageSignalCompletion(&CopyJob);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    else if (Ret != kEplApiPIJobQueueEmpty)
    {
        goto Exit;
    }

Exit:
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_pCurrentTask = NULL;
#elif (TARGET_SYSTEM == _WIN32_)
    EplApiProcessImageInstance_g.m_dwCurrentThreadId = ~0UL;
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

    return Ret;
}

// EOF

