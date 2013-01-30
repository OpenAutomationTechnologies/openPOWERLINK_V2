/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  Implementation of platform specific part for the
                shared buffer
                (Implementation for use without any operating system)

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

                Tested Build Environment:
                    Altera Nios2 GCC V3.4.6

  -------------------------------------------------------------------------

  Revision History:

  2009/08/24 -mu:   V 1.00 (initial version)

****************************************************************************/


#include "global.h"
#include <stdlib.h>
#include <string.h>
#include "ShbTarget.h"

#include "SharedBuff.h"
#include "ShbIpc.h"
#include "Debug.h"



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

#if (!defined(SHBIPC_INLINED)) || defined(SHBIPC_INLINE_ENABLED)

//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

#define MAX_LEN_BUFFER_ID       256

#define TIMEOUT_ENTER_ATOMIC    1000        // (ms) for debgging: INFINITE
#define TIMEOUT_TERM_THREAD     1000
#define INFINITE                3600

#define SBI_MAGIC_ID            0x5342492B  // magic ID ("SBI+")
#define SBH_MAGIC_ID            0x5342482A  // magic ID ("SBH*")

#define INVALID_ID              NULL

#define TABLE_SIZE              10


//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------

// This structure is the common header for the shared memory region used
// by all processes attached this shared memory. It includes common
// information to administrate/manage the shared buffer from a couple of
// separated processes (e.g. the refernce counter). This structure is
// located at the start of the shared memory region itself and exists
// consequently only one times per shared memory instance.
typedef struct
{
    unsigned long       m_ulShMemSize;
    unsigned long       m_ulRefCount;
    int                 m_iBufferId;
    BOOL                m_fNewData;
    BOOL                m_fJobReady;
    tShbInstance*       m_pShbInstMaster;

} tShbMemHeader;

// This structure is the "external entry point" from a separate process
// to get access to a shared buffer. This structure includes all platform
// resp. target specific information to administrate/manage the shared
// buffer from a separate process. Every process attached to the shared
// buffer has its own runtime instance of this structure with its individual
// runtime data (e.g. the scope of an event handle is limitted to the
// owner process only). The structure member <m_pShbMemHeader> points
// to the (process specific) start address of the shared memory region
// itself.
typedef struct _tShbMemInst
{
    unsigned long       m_SbiMagicID;           // magic ID ("SBI+")
    struct _tShbMemInst* m_pProcessListNewDataNext;
    struct _tShbMemInst* m_pProcessListJobReadyNext;
    tShbPriority        m_PriorityNewData;
    tSigHndlrNewData    m_pfnSigHndlrNewData;
    unsigned long       m_ulTimeOutMsJobReady;
    unsigned long       m_ulStartTimeMsJobReady;
    tSigHndlrJobReady   m_pfnSigHndlrJobReady;
    tShbMemHeader*      m_pShbMemHeader;

} tShbMemInst;


//---------------------------------------------------------------------------
//  Prototypes of internal functions
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//  Get pointer to process local information structure
//---------------------------------------------------------------------------

static inline tShbMemInst*  ShbIpcGetShbMemInst (
    tShbInstance pShbInstance_p)
{
tShbMemInst*  pShbMemInst;

    pShbMemInst = (tShbMemInst*)pShbInstance_p;

    return (pShbMemInst);
}



//---------------------------------------------------------------------------
//  Get instance pointer
//---------------------------------------------------------------------------

static inline tShbInstance  ShbIpcGetInstance (
    tShbMemInst* pShbMemInst_p)
{
tShbInstance  pShbInstance;

    pShbInstance = (tShbInstance)pShbMemInst_p;

    return (pShbInstance);
}



//---------------------------------------------------------------------------
//  Get pointer to shared memory header
//---------------------------------------------------------------------------

static inline tShbMemHeader*  ShbIpcGetShbMemHeader (
    tShbMemInst* pShbMemInst_p)
{
tShbMemHeader*  pShbMemHeader;

    pShbMemHeader = pShbMemInst_p->m_pShbMemHeader;

    return (pShbMemHeader);
}

static tShbError        ShbIpcProcessNewData (void);
static tShbError        ShbIpcProcessJobReady (void);


#endif

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

#if !defined(SHBIPC_INLINE_ENABLED)

struct sShbMemTable*    psMemTableElementFirst_g;

tShbMemInst*     pShbIpcProcessListNewDataFirst_g;
tShbMemInst**    ppShbIpcProcessListNewDataCurrent_g;
tShbMemInst*     pShbIpcProcessListJobReadyFirst_g;
tShbMemInst**    ppShbIpcProcessListJobReadyCurrent_g;


static int              ShbIpcFindListElement       (int iBufferId, struct sShbMemTable **ppsReturnMemTableElement);
static void             ShbIpcAppendListElement     (struct sShbMemTable *sNewMemTableElement);
static void             ShbIpcDeleteListElement     (int iBufferId);

static void             ShbIpcCrc32GenTable         (unsigned long aulCrcTable[256]);
static unsigned long    ShbIpcCrc32GetCrc           (const char *pcString, unsigned long aulCrcTable[256]);

#else

extern tShbMemInst*     pShbIpcProcessListNewDataFirst_g;
extern tShbMemInst**    ppShbIpcProcessListNewDataCurrent_g;
extern tShbMemInst*     pShbIpcProcessListJobReadyFirst_g;
extern tShbMemInst**    ppShbIpcProcessListJobReadyCurrent_g;

#endif


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

#if !defined(SHBIPC_INLINE_ENABLED)
// not inlined external functions

//---------------------------------------------------------------------------
//  Initialize IPC for Shared Buffer Module
//---------------------------------------------------------------------------

tShbError  ShbIpcInit (void)
{
    psMemTableElementFirst_g       = NULL;
    pShbIpcProcessListNewDataFirst_g     = NULL;
    ppShbIpcProcessListNewDataCurrent_g  = NULL;
    pShbIpcProcessListJobReadyFirst_g    = NULL;
    ppShbIpcProcessListJobReadyCurrent_g = NULL;

    return (kShbOk);
}


//---------------------------------------------------------------------------
//  Deinitialize IPC for Shared Buffer Module
//---------------------------------------------------------------------------

tShbError  ShbIpcExit (void)
{
    return (kShbOk);
}


//---------------------------------------------------------------------------
//  Allocate Shared Buffer
//---------------------------------------------------------------------------

tShbError  ShbIpcAllocBuffer (
    unsigned long ulBufferSize_p,
    const char* pszBufferID_p,
    tShbInstance* ppShbInstance_p,
    unsigned int* pfShbNewCreated_p)
{
tShbError               ShbError;
int                     iBufferId=0;
unsigned long           ulCrc32=0;
unsigned int            uiFirstProcess=0;
unsigned long           ulShMemSize;
tShbMemHeader*          pShbMemHeader;
tShbMemInst*            pShbMemInst=NULL;
tShbInstance            pShbInstance;
unsigned int            fShMemNewCreated=FALSE;
void                    *pSharedMem=NULL;
struct sShbMemTable     *psMemTableElement;
unsigned long           aulCrcTable[256];

    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer \n");
    if (ShbTgtIsInterruptContext())
    {
        ShbError = kShbInterruptContextNotAllowed;
        goto Exit;
    }

    ulShMemSize      = ulBufferSize_p + sizeof(tShbMemHeader);

    //create Buffer ID
    ShbIpcCrc32GenTable(aulCrcTable);
    ulCrc32 = ShbIpcCrc32GetCrc(pszBufferID_p, aulCrcTable);

    iBufferId=ulCrc32;
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer BufferSize:%d sizeof(tShb..):%d\n",ulBufferSize_p,sizeof(tShbMemHeader));
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer BufferId:%d MemSize:%d\n",iBufferId,ulShMemSize);
    //---------------------------------------------------------------
    // (1) open an existing or create a new shared memory
    //---------------------------------------------------------------
    //test if buffer already exists
    if (ShbIpcFindListElement(iBufferId, &psMemTableElement) == 0)
    {
        //Buffer already exists
        fShMemNewCreated=FALSE;
        pSharedMem = psMemTableElement->m_pBuffer;
        DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer attach Buffer at:%p Id:%d\n",pSharedMem, iBufferId);
        uiFirstProcess=1;
    }
    else
    {
        //create new Buffer
        fShMemNewCreated = TRUE;
        uiFirstProcess=0;
        pSharedMem = malloc(ulShMemSize);
        DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer Create New Buffer at:%p Id:%d\n",pSharedMem,iBufferId);
        if (pSharedMem == NULL)
        {
            //unable to create mem
            ShbError = kShbOutOfMem;
            goto Exit;
        }
        // append Element to Mem Table
        psMemTableElement = malloc(sizeof(struct sShbMemTable));
        psMemTableElement->m_iBufferId = iBufferId;
        psMemTableElement->m_pBuffer = pSharedMem;
        psMemTableElement->m_psNextMemTableElement = NULL;
        ShbIpcAppendListElement (psMemTableElement);
    }

    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer update header\n");
    //update header
    pShbMemHeader = (tShbMemHeader*)pSharedMem;

    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer 0 pShbMemHeader->m_ulShMemSize: %d\n",pShbMemHeader->m_ulShMemSize);
    // allocate a memory block for instance local information
    // to administrate/manage the shared buffer
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer alloc private mem\n");

    pShbMemInst = (tShbMemInst*) malloc(sizeof(tShbMemInst));
    if (pShbMemInst == NULL)
    {
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    // reset complete header to default values
    //pShbMemInst->m_SbiMagicID                             = SBI_MAGIC_ID;
    pShbMemInst->m_pfnSigHndlrNewData                       = NULL;
    pShbMemInst->m_ulTimeOutMsJobReady                      = 0;
    pShbMemInst->m_ulStartTimeMsJobReady                    = 0;
    pShbMemInst->m_pfnSigHndlrJobReady                      = NULL;
    pShbMemInst->m_pShbMemHeader                            = pShbMemHeader;
    pShbMemInst->m_pProcessListNewDataNext                  = NULL;
    pShbMemInst->m_pProcessListJobReadyNext                 = NULL;

    // initialize
    ShbError         = kShbOk;
    if ( fShMemNewCreated )
    {
        // this process was the first who wanted to use the shared memory,
        // so a new shared memory was created
        // -> setup new header information inside the shared memory region
        //    itself
        pShbMemHeader->m_ulShMemSize = ulShMemSize;
        pShbMemHeader->m_ulRefCount  = 1;
        pShbMemHeader->m_iBufferId   = iBufferId;
        pShbMemHeader->m_pShbInstMaster = NULL;
    }
    else
    {
        // any other process has created the shared memory and this
        // process only has to attach to it
        // -> check and update existing header information inside the
        //    shared memory region itself
        if (pShbMemHeader->m_ulShMemSize != ulShMemSize)
        {
            ShbError = kShbOpenMismatch;
            goto Exit;
        }
        pShbMemHeader->m_ulRefCount++;
    }

Exit:
    pShbInstance = (tShbInstance*)pShbMemInst;
    *pfShbNewCreated_p = fShMemNewCreated;
    *ppShbInstance_p   = pShbInstance;
    return (ShbError);

}



//---------------------------------------------------------------------------
//  Release Shared Buffer
//---------------------------------------------------------------------------

tShbError  ShbIpcReleaseBuffer (tShbInstance pShbInstance_p)
{
tShbMemInst*    pShbMemInst;
tShbMemInst**   ppShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    DEBUG_LVL_26_TRACE("ShbIpcReleaseBuffer(%p)\n", pShbInstance_p);
    if (ShbTgtIsInterruptContext())
    {
        return (kShbInterruptContextNotAllowed);
    }

    if (pShbInstance_p == NULL)
    {
        return (kShbOk);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    ShbIpcStopSignalingNewData (pShbInstance_p);

    // remove ShbMemInst from JobReady process list if signaling
    ppShbMemInst = &pShbIpcProcessListJobReadyFirst_g;
    while (*ppShbMemInst != NULL)
    {
        if (*ppShbMemInst == pShbMemInst)
        {
            if (pShbMemInst->m_pProcessListJobReadyNext == NULL)
            {
                ShbIpcEnterAtomicSection(NULL);
                *ppShbMemInst = pShbMemInst->m_pProcessListJobReadyNext;
                ShbIpcLeaveAtomicSection(NULL);
            }
            else
            {
                *ppShbMemInst = pShbMemInst->m_pProcessListJobReadyNext;
            }
            if (ppShbIpcProcessListJobReadyCurrent_g == &pShbMemInst->m_pProcessListJobReadyNext)
            {
                ppShbIpcProcessListJobReadyCurrent_g = ppShbMemInst;
            }
            break;
        }
        ppShbMemInst = &(*ppShbMemInst)->m_pProcessListJobReadyNext;
    }

    if ( !--pShbMemHeader->m_ulRefCount )
    {
        ShbError = kShbOk;
        // delete mem table element
        ShbIpcDeleteListElement(pShbMemHeader->m_iBufferId);
        // delete shared mem
        free(pShbMemInst->m_pShbMemHeader);
    }
    else
    {
        ShbError = kShbMemUsedByOtherProcs;
    }
    //delete privat mem
    free(pShbMemInst);

    return (ShbError);
}


//---------------------------------------------------------------------------
//  Signal new data (called from writing instance)
//---------------------------------------------------------------------------

tShbError  ShbIpcSignalNewData (
    tShbInstance pShbInstance_p)
{
tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    //set semaphore
    pShbMemHeader->m_fNewData = TRUE;
    DEBUG_LVL_29_TRACE("ShbIpcSignalNewData set Sem -> New Data\n");

    if (pShbMemHeader->m_pShbInstMaster != NULL)
    {
        return ShbIpcSignalNewData(pShbMemHeader->m_pShbInstMaster);
    }

    return (kShbOk);
}



#endif  // !defined(SHBIPC_INLINE_ENABLED)

#if (!defined(SHBIPC_INLINED)) || defined(SHBIPC_INLINE_ENABLED) || defined(MICROBLAZE_SHBIPC_SIGNALLING_REQ)


//---------------------------------------------------------------------------
//  Process function for NewData and JobReady signaling
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcProcess (void)
{
tShbError       ShbError = kShbOk;

    if (ShbTgtIsInterruptContext())
    {
        ShbError = kShbInterruptContextNotAllowed;
        goto Exit;
    }

    ShbError = ShbIpcProcessNewData();
    if (ShbError != kShbOk)
    {
        goto Exit;
    }

    ShbError = ShbIpcProcessJobReady();

Exit:
    return ShbError;

}



//---------------------------------------------------------------------------
//  Enter atomic section for Shared Buffer access
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcEnterAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbError       ShbError = kShbOk;

    // disable interrupts
    ShbTgtEnableGlobalInterrupt(FALSE);

    return ShbError;

}



//---------------------------------------------------------------------------
//  Leave atomic section for Shared Buffer access
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcLeaveAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbError       ShbError = kShbOk;

    // restore interrupts
    ShbTgtEnableGlobalInterrupt(TRUE);

    return ShbError;

}



//---------------------------------------------------------------------------
//  Set master instance of this slave instance
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcSetMaster (
    tShbInstance pShbInstance_p,
    tShbInstance pShbInstanceMaster_p)
{

tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    pShbMemHeader->m_pShbInstMaster = pShbInstanceMaster_p;

    return (kShbOk);

}



//---------------------------------------------------------------------------
//  Start signaling of new data (called from reading instance)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStartSignalingNewData (
    tShbInstance pShbInstance_p,
    tSigHndlrNewData pfnSignalHandlerNewData_p,
    tShbPriority ShbPriority_p)
{
tShbMemInst*    pShbMemInst;
tShbMemInst**   ppShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    DEBUG_LVL_29_TRACE("------->ShbIpcStartSignalingNewData\n");
    if (ShbTgtIsInterruptContext())
    {
        return (kShbInterruptContextNotAllowed);
    }

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerNewData_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst(pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader(pShbMemInst);
    ShbError = kShbOk;

    if (pShbMemInst->m_pfnSigHndlrNewData != NULL)
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }
    DEBUG_LVL_26_TRACE("ShbIpcStartSignalingNewData(%p) m_pfnSigHndlrNewData = %p\n", pShbInstance_p, pfnSignalHandlerNewData_p);
    pShbMemInst->m_pfnSigHndlrNewData = pfnSignalHandlerNewData_p;
    pShbMemInst->m_PriorityNewData = ShbPriority_p;
    pShbMemHeader->m_fNewData = FALSE;

    // insert ShbMemInst into NewData process list
    // higher priority entries first
    ppShbMemInst = &pShbIpcProcessListNewDataFirst_g;
    while (*ppShbMemInst != NULL)
    {
        if ((*ppShbMemInst)->m_PriorityNewData < pShbMemInst->m_PriorityNewData)
        {
            break;
        }
        ppShbMemInst = &(*ppShbMemInst)->m_pProcessListNewDataNext;
    }
    pShbMemInst->m_pProcessListNewDataNext = *ppShbMemInst;
    *ppShbMemInst = pShbMemInst;

Exit:
    return ShbError;

}



//---------------------------------------------------------------------------
//  Stop signaling of new data (called from reading instance)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStopSignalingNewData (
    tShbInstance pShbInstance_p)
{
tShbMemInst*    pShbMemInst;
tShbMemInst**   ppShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    DEBUG_LVL_29_TRACE("------->ShbIpcStopSignalingNewData\n");
    if (ShbTgtIsInterruptContext())
    {
        return (kShbInterruptContextNotAllowed);
    }

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    ShbError = kShbOk;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    DEBUG_LVL_26_TRACE("ShbIpcStopSignalingNewData(%p) pfnSignHndlrNewData=%p\n", pShbInstance_p, pShbMemInst->m_pfnSigHndlrNewData);
    if (pShbMemInst->m_pfnSigHndlrNewData != NULL)
    {   // signal handler was set before
        // remove pShbMemInst from NewData process list
        ShbError = kShbInvalidSigHndlr;
        ppShbMemInst = &pShbIpcProcessListNewDataFirst_g;
        while (*ppShbMemInst != NULL)
        {
            if (*ppShbMemInst == pShbMemInst)
            {
                *ppShbMemInst = pShbMemInst->m_pProcessListNewDataNext;
                pShbMemInst->m_pProcessListNewDataNext = NULL;
                pShbMemInst->m_pfnSigHndlrNewData = NULL;

                if (ppShbIpcProcessListNewDataCurrent_g == &pShbMemInst->m_pProcessListNewDataNext)
                {
                    ppShbIpcProcessListNewDataCurrent_g = ppShbMemInst;
                }

                ShbError = kShbOk;
                break;
            }
            ppShbMemInst = &(*ppShbMemInst)->m_pProcessListNewDataNext;
        }
    }

    return ShbError;

}



//---------------------------------------------------------------------------
//  Start signaling for job ready (called from waiting instance)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStartSignalingJobReady (
    tShbInstance pShbInstance_p,
    unsigned long ulTimeOutMs_p,
    tSigHndlrJobReady pfnSignalHandlerJobReady_p)
{
tShbMemInst*    pShbMemInst;
tShbMemInst**   ppShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerJobReady_p == NULL))
    {
        return (kShbInvalidArg);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    ShbError = kShbOk;
    if (pShbMemInst->m_pfnSigHndlrJobReady != NULL)
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }
    pShbMemInst->m_ulTimeOutMsJobReady      = ulTimeOutMs_p;
    pShbMemInst->m_ulStartTimeMsJobReady    = ShbTgtGetTickCountMs();
    pShbMemInst->m_pfnSigHndlrJobReady      = pfnSignalHandlerJobReady_p;
    pShbMemInst->m_pProcessListNewDataNext  = NULL;
    pShbMemHeader->m_fJobReady = FALSE;

    // insert ShbMemInst at the end of JobReady list
    // thus other operations on the list are not disturbed
    ppShbMemInst = &pShbIpcProcessListJobReadyFirst_g;
    while (*ppShbMemInst != NULL)
    {
        ppShbMemInst = &(*ppShbMemInst)->m_pProcessListJobReadyNext;
    }
    ShbIpcEnterAtomicSection(NULL);
    while (*ppShbMemInst != NULL)
    {
        ppShbMemInst = &(*ppShbMemInst)->m_pProcessListJobReadyNext;
    }
    *ppShbMemInst = pShbMemInst;
    ShbIpcLeaveAtomicSection(NULL);

Exit:
    return ShbError;
}



//---------------------------------------------------------------------------
//  Signal job ready (called from executing instance)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcSignalJobReady (
    tShbInstance pShbInstance_p)
{
tShbMemHeader*  pShbMemHeader;

    DEBUG_LVL_29_TRACE("ShbIpcSignalJobReady\n");
    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    //set semaphore
    pShbMemHeader->m_fJobReady = TRUE;
    DEBUG_LVL_29_TRACE("ShbIpcSignalJobReady set Sem -> Job Ready \n");

    return (kShbOk);
}



//---------------------------------------------------------------------------
//  Get pointer to common used share memory area
//---------------------------------------------------------------------------

INLINE_FUNCTION void*  ShbIpcGetShMemPtr (tShbInstance pShbInstance_p)
{

tShbMemHeader*  pShbMemHeader;
void*  pShbShMemPtr;


    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    if (pShbMemHeader != NULL)
    {
        pShbShMemPtr = (BYTE*)pShbMemHeader + sizeof(tShbMemHeader);
    }
    else
    {
        pShbShMemPtr = NULL;
    }

    return (pShbShMemPtr);

}

#endif



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


#if (!defined(SHBIPC_INLINED)) || defined(SHBIPC_INLINE_ENABLED)

//---------------------------------------------------------------------------
//  Process function for NewData signaling
//---------------------------------------------------------------------------

static tShbError  ShbIpcProcessNewData (void)
{
tShbMemInst*        pShbMemInst;
tShbMemHeader*      pShbMemHeader;
tShbInstance        pShbInstance = NULL;
int                 fCallAgain;
tSigHndlrNewData    pfnSigHndlrNewData;
tShbPriority        Priority = kShbPriorityLow;
BOOL                fEndOfProcessList = FALSE;
tShbError           ShbError = kShbOk;

    ppShbIpcProcessListNewDataCurrent_g = &pShbIpcProcessListNewDataFirst_g;

    while (fEndOfProcessList == FALSE)
    {
        pfnSigHndlrNewData = NULL;

        pShbMemInst = *ppShbIpcProcessListNewDataCurrent_g;
        if (pShbMemInst == NULL)
        {
            fEndOfProcessList = TRUE;
        }
        else
        {
            pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

            if (pShbMemHeader->m_fNewData != FALSE)
            {
                pfnSigHndlrNewData = pShbMemInst->m_pfnSigHndlrNewData;
                pShbInstance       = ShbIpcGetInstance(pShbMemInst);
                Priority           = pShbMemInst->m_PriorityNewData;

                pShbMemHeader->m_fNewData = FALSE;
            }

            ppShbIpcProcessListNewDataCurrent_g = &pShbMemInst->m_pProcessListNewDataNext;
        }

        if (pfnSigHndlrNewData != NULL)
        {
            // signal all NewData events if high priority
            // signal just one for lower priorities
            do
            {
                fCallAgain = pfnSigHndlrNewData(pShbInstance);
            }
            while ((fCallAgain != FALSE) && (Priority == kShbPriorityHigh));

            if (fCallAgain != FALSE)
            {
                pShbMemHeader->m_fNewData = TRUE;
            }
        }
    }
    ppShbIpcProcessListNewDataCurrent_g = NULL;

    return ShbError;
}



//---------------------------------------------------------------------------
//  Process function for JobReady signaling
//---------------------------------------------------------------------------

static tShbError  ShbIpcProcessJobReady (void)
{
tShbMemInst*        pShbMemInst;
tShbMemHeader*      pShbMemHeader;
unsigned long       ulTimeOutMs;
unsigned long       ulStartTimeMs;
tShbInstance        pShbInstance;
tSigHndlrJobReady   pfnSigHndlrJobReady;
BOOL                fJobReady;
BOOL                fEndOfProcessList = FALSE;
tShbError           ShbError = kShbOk;

    ppShbIpcProcessListJobReadyCurrent_g = &pShbIpcProcessListJobReadyFirst_g;

    while (fEndOfProcessList == FALSE)
    {
        pfnSigHndlrJobReady = NULL;

        pShbMemInst = *ppShbIpcProcessListJobReadyCurrent_g;
        if (pShbMemInst == NULL)
        {
            fEndOfProcessList = TRUE;
        }
        else
        {
            pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);
            ulTimeOutMs   = pShbMemInst->m_ulTimeOutMsJobReady;
            ulStartTimeMs = pShbMemInst->m_ulStartTimeMsJobReady;

            if ((pShbMemHeader->m_fJobReady != FALSE) ||
                (   (ulTimeOutMs != 0) &&
                    (ulTimeOutMs <= ShbTgtGetTickCountMs() - ulStartTimeMs)))
            {
                pfnSigHndlrJobReady = pShbMemInst->m_pfnSigHndlrJobReady;
                pShbInstance        = ShbIpcGetInstance(pShbMemInst);
                fJobReady           = pShbMemHeader->m_fJobReady;

                // remove ShbMemInst from JobReady process list
                if (pShbMemInst->m_pProcessListJobReadyNext == NULL)
                {
                    ShbIpcEnterAtomicSection(NULL);
                    *ppShbIpcProcessListJobReadyCurrent_g = pShbMemInst->m_pProcessListJobReadyNext;
                    ShbIpcLeaveAtomicSection(NULL);
                }
                else
                {
                    *ppShbIpcProcessListJobReadyCurrent_g = pShbMemInst->m_pProcessListJobReadyNext;
                }
                pShbMemInst->m_pProcessListJobReadyNext = NULL;
                pShbMemInst->m_pfnSigHndlrJobReady      = NULL;
            }
            else
            {
                ppShbIpcProcessListJobReadyCurrent_g = &pShbMemInst->m_pProcessListJobReadyNext;
            }
        }

        if (pfnSigHndlrJobReady != NULL)
        {
            pfnSigHndlrJobReady(pShbInstance, fJobReady);
        }
    }
    ppShbIpcProcessListJobReadyCurrent_g = NULL;

    return ShbError;
}

#endif


#if !defined(SHBIPC_INLINE_ENABLED)

//Build the crc table
static void ShbIpcCrc32GenTable(unsigned long aulCrcTable[256])
{
    unsigned long       ulCrc,ulPoly;
    int                 iIndexI,iIndexJ;

    ulPoly = 0xEDB88320L;
    for (iIndexI = 0; iIndexI < 256; iIndexI++)
    {
        ulCrc = iIndexI;
        for (iIndexJ = 8; iIndexJ > 0; iIndexJ--)
        {
            if (ulCrc & 1)
            {
                ulCrc = (ulCrc >> 1) ^ ulPoly;
            }
            else
            {
                ulCrc >>= 1;
            }
        }
        aulCrcTable[iIndexI] = ulCrc;
    }
}

//Calculate the crc value
static unsigned long ShbIpcCrc32GetCrc(const char *pcString,unsigned long aulCrcTable[256])
{
    unsigned long   ulCrc;
    int             iIndex;

    ulCrc = 0xFFFFFFFF;
    for (iIndex=0;iIndex<strlen(pcString);iIndex++)
    {
        ulCrc = ((ulCrc>>8) & 0x00FFFFFF) ^ aulCrcTable[ (ulCrc^pcString[iIndex]) & 0xFF ];
    }
    return( ulCrc^0xFFFFFFFF );

}

static void ShbIpcAppendListElement (struct sShbMemTable *psNewMemTableElement)
{
    struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
    psNewMemTableElement->m_psNextMemTableElement=NULL;

    if (psMemTableElementFirst_g!= NULL )
    {      /* sind Elemente vorhanden */
       while (psMemTableElement->m_psNextMemTableElement != NULL )
       {    /* suche das letzte Element */
           psMemTableElement=psMemTableElement->m_psNextMemTableElement;
       }
       psMemTableElement->m_psNextMemTableElement=psNewMemTableElement;              /*  Haenge das Element hinten an */
    }
    else
    {                           /* wenn die liste leer ist, bin ich das erste Element */
        psMemTableElementFirst_g=psNewMemTableElement;
    }
}




static int ShbIpcFindListElement (int iBufferId, struct sShbMemTable **ppsReturnMemTableElement)
{
    struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
    while (psMemTableElement!=NULL)
    {
        if(psMemTableElement->m_iBufferId==iBufferId)
        {
             *ppsReturnMemTableElement=psMemTableElement;
             return 0;
        }
        psMemTableElement=psMemTableElement->m_psNextMemTableElement;
    }
    return -1;
}

static void ShbIpcDeleteListElement(int iBufferId)
{
   struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
   struct sShbMemTable *psMemTableElementOld=psMemTableElementFirst_g;
   if (psMemTableElement!=NULL)
   {
        while((psMemTableElement!=NULL)&&(psMemTableElement->m_iBufferId!=iBufferId))
        {
            psMemTableElementOld=psMemTableElement;
            psMemTableElement=psMemTableElement->m_psNextMemTableElement;
        }
        if (psMemTableElement!=NULL)
        {
            if (psMemTableElement!=psMemTableElementFirst_g)
            {
                psMemTableElementOld->m_psNextMemTableElement=psMemTableElement->m_psNextMemTableElement;
                free(psMemTableElement);
            }
            else
            {
                free(psMemTableElement);
                psMemTableElementFirst_g=NULL;
            }

        }
   }

}

#endif
