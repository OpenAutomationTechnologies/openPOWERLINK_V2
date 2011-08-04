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
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/highmem.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <asm/current.h>
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
#include <sys/types.h>
#include <semaphore.h>
#include <pthread.h>
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

#ifndef EPL_API_PI_MAX_SIZE
#define EPL_API_PI_MAX_SIZE         65536
#endif

#ifndef EPL_API_PI_BUFFER_ID_LO
#define EPL_API_PI_BUFFER_ID_LO     "EplApiPIJobQueueLo"
#endif

#ifndef EPL_API_PI_BUFFER_ID_HI
#define EPL_API_PI_BUFFER_ID_HI     "EplApiPIJobQueueHi"
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)

#define EPL_API_PI_PAGE_COUNT       ((EPL_API_PI_MAX_SIZE >> PAGE_SHIFT) + 1)

// special completion structure for Linux Kernel
typedef struct
{
    struct completion       m_Completion;
    struct page*            m_apPageIn[EPL_API_PI_PAGE_COUNT];
    struct page*            m_apPageOut[EPL_API_PI_PAGE_COUNT];

} tEplApiProcessImageCompletion;
#endif


typedef struct
{
    tEplApiProcessImageCopyJob  m_CopyJob;
    union
    {
        tEplEventSink           m_EventSink;
        // more OS specific event types
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        tEplApiProcessImageCompletion* m_pCompletion;
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
        sem_t                   m_semCompletion;
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
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    pthread_t           m_currentThreadId;
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

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
static tEplKernel EplApiProcessImageGetUserPages(
    tEplApiProcessImage* pImage_p,
    tEplApiProcessImagePart* pPart_p, BOOL fOut_p,
    struct page** ppPage_p);

static void EplApiProcessImagePutUserPages(
    struct page** ppPage_p, BOOL fDirty_p);

static tEplKernel EplApiProcessImageExchangeIntUserPages(
    tEplApiProcessImageCopyJobInt* pCopyJob_p);
#endif


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
unsigned int    fShbNewCreated;

    TRACE5("%s: Alloc(%u, %u, %u, %u)\n",
          __func__,
          uiSizeProcessImageIn_p,
          uiSizeProcessImageOut_p,
          uiQueueEntriesLo_p,
          uiQueueEntriesHi_p);

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

    TRACE5("%s: Alloc(%p, %u, %p, %u)\n",
          __func__,
          EplApiProcessImageInstance_g.m_In.m_pImage,
          EplApiProcessImageInstance_g.m_In.m_uiSize,
          EplApiProcessImageInstance_g.m_Out.m_pImage,
          EplApiProcessImageInstance_g.m_Out.m_uiSize);

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

    if (Ret == kEplApiPIJobQueueEmpty)
    {
        Ret = kEplSuccessful;
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
        if ((uiOffsetPI_p + EntrySize_p) > EplApiProcessImageInstance_g.m_In.m_uiSize)
        {   // at least one entry should fit into the PI, but it doesn't
            Ret = kEplApiPISizeExceeded;
            goto Exit;
        }
        if ((uiOffsetPI_p + (*puiVarEntries_p * EntrySize_p)) > EplApiProcessImageInstance_g.m_In.m_uiSize)
        {   // limit the number of entries
            *puiVarEntries_p = (EplApiProcessImageInstance_g.m_In.m_uiSize - uiOffsetPI_p) / EntrySize_p;
        }
    }
    else
    {   // output PI
        pVar = ((BYTE*) EplApiProcessImageInstance_g.m_Out.m_pImage) + uiOffsetPI_p;
        if ((uiOffsetPI_p + EntrySize_p) > EplApiProcessImageInstance_g.m_Out.m_uiSize)
        {   // at least one entry should fit into the PI, but it doesn't
            Ret = kEplApiPISizeExceeded;
            goto Exit;
        }
        if ((uiOffsetPI_p + (*puiVarEntries_p * EntrySize_p)) > EplApiProcessImageInstance_g.m_Out.m_uiSize)
        {   // limit the number of entries
            *puiVarEntries_p = (EplApiProcessImageInstance_g.m_Out.m_uiSize - uiOffsetPI_p) / EntrySize_p;
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
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    if (pthread_equal(EplApiProcessImageInstance_g.m_currentThreadId, pthread_self()))
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
        || (EplApiProcessImageInstance_g.m_Out.m_uiSize == 0))
    {   // the process image has been freed
        // therefore, indicate shutdown to application thread
        Ret = kEplShutdown;
        goto Exit;
    }

    IntCopyJob.m_CopyJob = *pCopyJob_p;

    if (pCopyJob_p->m_fNonBlocking == FALSE)
    {
        Ret = EplApiProcessImageCreateCompletion(&IntCopyJob);
        if (Ret != kEplSuccessful)
        {
            EplApiProcessImageDeleteCompletion(&IntCopyJob);
            goto Exit;
        }
    }
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    else
    {
        Ret = kEplApiPINonBlockingNotSupp;
        goto Exit;
    }
#endif

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
#endif

    Ret = EplApiProcessImagePostCopyJob(&IntCopyJob);

    if (pCopyJob_p->m_fNonBlocking == FALSE)
    {
        if (Ret == kEplSuccessful)
        {
            Ret = EplApiProcessImageWaitForCompletion(&IntCopyJob);

            if ((Ret != kEplSuccessful)
                || (EplApiProcessImageInstance_g.m_In.m_uiSize == 0)
                || (EplApiProcessImageInstance_g.m_Out.m_uiSize == 0))
            {   // in the mean time the process image has been freed
                // therefore, indicate shutdown to application thread
                Ret = kEplShutdown;
            }
        }

        EplApiProcessImageDeleteCompletion(&IntCopyJob);
    }

Exit:
    return Ret;
}



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
            && ((pCopyJob_p->m_In.m_uiOffset + pCopyJob_p->m_In.m_uiSize)
                <= EplApiProcessImageInstance_g.m_In.m_uiSize))
        {
        void*   pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_In.m_pImage)
                            + pCopyJob_p->m_In.m_uiOffset;

            #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
            int     iErr;

                iErr = copy_from_user(pPIVar,
                    pCopyJob_p->m_In.m_pPart,
                    pCopyJob_p->m_In.m_uiSize);
                if (iErr != 0)
                {
                    PRINTF("%s: copy_from_user(%p, %p, %u) returned %i\n",
                            __func__,
                            pPIVar,
                            pCopyJob_p->m_In.m_pPart,
                            pCopyJob_p->m_In.m_uiSize,
                            iErr);

                    Ret = kEplApiPIInvalidPIPointer;
                    goto Exit;
                }
            #else
                EPL_MEMCPY(pPIVar,
                    pCopyJob_p->m_In.m_pPart,
                    pCopyJob_p->m_In.m_uiSize);
            #endif
        }
    }

    if (pCopyJob_p->m_Out.m_uiSize > 0)
    {
        if ((pCopyJob_p->m_Out.m_pPart != NULL)
            && (EplApiProcessImageInstance_g.m_Out.m_pImage != NULL)
            && ((pCopyJob_p->m_Out.m_uiOffset + pCopyJob_p->m_Out.m_uiSize)
                <= EplApiProcessImageInstance_g.m_Out.m_uiSize))
        {
        void*   pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_Out.m_pImage)
                            + pCopyJob_p->m_Out.m_uiOffset;

            #if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
            int     iErr;

                iErr = copy_to_user(pCopyJob_p->m_Out.m_pPart,
                    pPIVar,
                    pCopyJob_p->m_Out.m_uiSize);
                if (iErr != 0)
                {
                    PRINTF("%s: copy_to_user(%p, %p, %u) returned %i\n",
                            __func__,
                            pCopyJob_p->m_Out.m_pPart,
                            pPIVar,
                            pCopyJob_p->m_Out.m_uiSize,
                            iErr);

                    Ret = kEplApiPIInvalidPIPointer;
                    goto Exit;
                }
            #else
                EPL_MEMCPY(pCopyJob_p->m_Out.m_pPart,
                    pPIVar,
                    pCopyJob_p->m_Out.m_uiSize);
            #endif
        }
    }

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
Exit:
#endif
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
        ShbError = ShbCirReadDataBlock (ShbInstance, pCopyJob_p, sizeof (*pCopyJob_p), &ulCopyJobSize);
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


#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageGetUserPages()
//
// Description: Gets the page entries for the referred process image part.
//
// Parameters:  pImage_p        = pointer to process image
//              pPart_p         = pointer to part structure
//              fOut_p          = TRUE if output copy job
//              ppPage_p        = pointer to page entry pointers
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageGetUserPages(
    tEplApiProcessImage* pImage_p,
    tEplApiProcessImagePart* pPart_p, BOOL fOut_p,
    struct page** ppPage_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;
unsigned int    uiNrOfPages;

    if (pPart_p->m_uiSize > EPL_API_PI_MAX_SIZE)
    {
        Ret = kEplApiPISizeExceeded;
        goto Exit;
    }

    if ((pPart_p->m_uiSize > 0)
        && (pPart_p->m_pPart != NULL)
        && (pImage_p->m_pImage != NULL)
        && ((pPart_p->m_uiOffset + pPart_p->m_uiSize)
            <= pImage_p->m_uiSize))
    {
        uiNrOfPages = (((unsigned long)pPart_p->m_pPart & ~PAGE_MASK)
                + pPart_p->m_uiSize + PAGE_SIZE - 1) >> PAGE_SHIFT;
        down_read(&current->mm->mmap_sem);
        iRet = get_user_pages(current, current->mm,
                (unsigned long)pPart_p->m_pPart,
                uiNrOfPages, fOut_p, 0, ppPage_p,
                NULL);
        up_read(&current->mm->mmap_sem);
        if (iRet != uiNrOfPages)
        {
            PRINTF("%s: get_user_pages(%p, %u, %d, %p) returned %d\n",
                    __func__,
                    pPart_p->m_pPart,
                    uiNrOfPages, fOut_p,
                    ppPage_p, iRet);
            EplApiProcessImagePutUserPages(ppPage_p, FALSE);
            Ret = kEplApiPIOutOfMemory;
            goto Exit;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImagePutUserPages()
//
// Description: Puts the specified page entries.
//
// Parameters:  ppPage_p        = pointer to page entry pointers
//              fDirty_p        = TRUE, if pages should be marked dirty
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static void EplApiProcessImagePutUserPages(
    struct page** ppPage_p, BOOL fDirty_p)
{
unsigned int    nIndex;

    for (nIndex = 0; nIndex < EPL_API_PI_PAGE_COUNT; nIndex++)
    {
        if (ppPage_p[nIndex] == NULL)
        {
            break;
        }
        if (fDirty_p != FALSE)
        {
            set_page_dirty_lock(ppPage_p[nIndex]);
        }
        put_page(ppPage_p[nIndex]);
        ppPage_p[nIndex] = NULL;
    }
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageExchangeIntUserPages()
//
// Description: Perform the actual copy job on mapped user pages.
//
// Parameters:  pCopyJob_p              = pointer to internal copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplApiProcessImageExchangeIntUserPages(
    tEplApiProcessImageCopyJobInt* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    nIndex;
struct page**   ppPage;
unsigned long   ulOffset;
unsigned long   ulLength;
unsigned long   ulSize;
void*           pPIVar;
tEplApiProcessImagePart* pPart;
void*           pVirtUserPart;

    // copy input image
    ppPage = pCopyJob_p->m_Event.m_pCompletion->m_apPageIn;

    pPart = &pCopyJob_p->m_CopyJob.m_In;

    if ((pPart->m_uiSize > 0)
        && (pPart->m_pPart != NULL)
        && (EplApiProcessImageInstance_g.m_In.m_pImage != NULL)
        && ((pPart->m_uiOffset + pPart->m_uiSize)
            <= EplApiProcessImageInstance_g.m_In.m_uiSize))
    {
        pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_In.m_pImage)
                    + pPart->m_uiOffset;

        ulOffset = ((unsigned long)pPart->m_pPart) & (PAGE_SIZE - 1);

        ulLength = 0;

        for (nIndex = 0; nIndex < EPL_API_PI_PAGE_COUNT; nIndex++)
        {
            if (ppPage[nIndex] == NULL)
            {
                break;
            }

            ulSize = min ((PAGE_SIZE - ulOffset), pPart->m_uiSize - ulLength);
            pVirtUserPart = kmap_atomic(ppPage[nIndex], KM_USER0);

            EPL_MEMCPY(pPIVar,
                pVirtUserPart + ulOffset,
                ulSize);

            kunmap_atomic(pVirtUserPart, KM_USER0);

            pPIVar += ulSize;
            ulLength += ulSize;
            ulOffset = 0;

            if (ulLength >= pPart->m_uiSize)
            {
                break;
            }
        }
    }

    // copy output image
    ppPage = pCopyJob_p->m_Event.m_pCompletion->m_apPageOut;

    pPart = &pCopyJob_p->m_CopyJob.m_Out;

    if ((pPart->m_uiSize > 0)
        && (pPart->m_pPart != NULL)
        && (EplApiProcessImageInstance_g.m_Out.m_pImage != NULL)
        && ((pPart->m_uiOffset + pPart->m_uiSize)
            <= EplApiProcessImageInstance_g.m_Out.m_uiSize))
    {
        pPIVar = ((BYTE*) EplApiProcessImageInstance_g.m_Out.m_pImage)
                    + pPart->m_uiOffset;

        ulOffset = ((unsigned long)pPart->m_pPart) & (PAGE_SIZE - 1);

        ulLength = 0;

        for (nIndex = 0; nIndex < EPL_API_PI_PAGE_COUNT; nIndex++)
        {
            if (ppPage[nIndex] == NULL)
            {
                break;
            }

            ulSize = min ((PAGE_SIZE - ulOffset), pPart->m_uiSize - ulLength);
            pVirtUserPart = kmap_atomic(ppPage[nIndex], KM_USER0);

            EPL_MEMCPY(pVirtUserPart + ulOffset,
                pPIVar,
                ulSize);

            kunmap_atomic(pVirtUserPart, KM_USER0);

            pPIVar += ulSize;
            ulLength += ulSize;
            ulOffset = 0;

            if (ulLength >= pPart->m_uiSize)
            {
                break;
            }
        }
    }

    return Ret;
}
#endif


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
    init_completion(&pCopyJob_p->m_Event.m_pCompletion->m_Completion);

    EPL_MEMSET (pCopyJob_p->m_Event.m_pCompletion->m_apPageIn,
            0, sizeof (pCopyJob_p->m_Event.m_pCompletion->m_apPageIn));
    EPL_MEMSET (pCopyJob_p->m_Event.m_pCompletion->m_apPageOut,
            0, sizeof (pCopyJob_p->m_Event.m_pCompletion->m_apPageOut));

    // fetch page pointers for userspace memory
    Ret = EplApiProcessImageGetUserPages(&EplApiProcessImageInstance_g.m_In,
            &pCopyJob_p->m_CopyJob.m_In, FALSE,
            pCopyJob_p->m_Event.m_pCompletion->m_apPageIn);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplApiProcessImageGetUserPages(&EplApiProcessImageInstance_g.m_Out,
            &pCopyJob_p->m_CopyJob.m_Out, TRUE,
            pCopyJob_p->m_Event.m_pCompletion->m_apPageOut);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    sem_init(&pCopyJob_p->m_Event.m_semCompletion, 0, 0);

#elif (TARGET_SYSTEM == _WIN32_)
    pCopyJob_p->m_Event.m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
Exit:
#endif
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
int             iRes;

    iRes = wait_for_completion_interruptible(&pCopyJob_p->m_Event.m_pCompletion->m_Completion);
    if (iRes != 0)
    {
        Ret = kEplShutdown;
    }

#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    sem_wait(&pCopyJob_p->m_Event.m_semCompletion);
#elif (TARGET_SYSTEM == _WIN32_)
    WaitForSingleObject(pCopyJob_p->m_Event.m_hEvent, INFINITE);
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

//Exit:
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
    if (pCopyJob_p->m_Event.m_pCompletion != NULL)
    {
        EplApiProcessImagePutUserPages(
                pCopyJob_p->m_Event.m_pCompletion->m_apPageIn, FALSE);
        EplApiProcessImagePutUserPages(
                pCopyJob_p->m_Event.m_pCompletion->m_apPageOut, TRUE);
        EPL_FREE(pCopyJob_p->m_Event.m_pCompletion);
        pCopyJob_p->m_Event.m_pCompletion = NULL;
    }

#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    sem_destroy(&pCopyJob_p->m_Event.m_semCompletion);

#elif (TARGET_SYSTEM == _WIN32_)
    if ((pCopyJob_p->m_Event.m_hEvent != NULL)
        && (pCopyJob_p->m_Event.m_hEvent != INVALID_HANDLE_VALUE))
    {
        CloseHandle(pCopyJob_p->m_Event.m_hEvent);
        pCopyJob_p->m_Event.m_hEvent = INVALID_HANDLE_VALUE;
    }

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
        complete(&pCopyJob_p->m_Event.m_pCompletion->m_Completion);
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
        sem_post(&pCopyJob_p->m_Event.m_semCompletion);
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

//Exit:
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
tEplKernel                      RetExchange;
tEplApiProcessImageCopyJobInt   CopyJob;

    // memorize current thread, so that Exchange() can detect if it is called from within this function
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_pCurrentTask = get_current();
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_currentThreadId = pthread_self();
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
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        RetExchange = EplApiProcessImageExchangeIntUserPages(&CopyJob);
#else
        RetExchange = EplApiProcessImageExchangeInt(&CopyJob.m_CopyJob);
#endif

        Ret = EplApiProcessImageSignalCompletion(&CopyJob);
        if (RetExchange != kEplSuccessful)
        {
            Ret = RetExchange;
            goto Exit;
        }

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
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
        RetExchange = EplApiProcessImageExchangeIntUserPages(&CopyJob);
#else
        RetExchange = EplApiProcessImageExchangeInt(&CopyJob.m_CopyJob);
#endif

        Ret = EplApiProcessImageSignalCompletion(&CopyJob);
        if (RetExchange != kEplSuccessful)
        {
            Ret = RetExchange;
            goto Exit;
        }

        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

    }
    else if (Ret != kEplApiPIJobQueueEmpty)
    {
        goto Exit;
    }

    Ret = kEplSuccessful;

Exit:
#if (TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_pCurrentTask = NULL;
#elif (TARGET_SYSTEM == _LINUX_) && !defined(__KERNEL__)
    EplApiProcessImageInstance_g.m_currentThreadId = 0;
#elif (TARGET_SYSTEM == _WIN32_)
    EplApiProcessImageInstance_g.m_dwCurrentThreadId = ~0UL;
#else
#error "OS currently not supported by EplApiProcessImage!"
#endif

    return Ret;
}

// EOF

