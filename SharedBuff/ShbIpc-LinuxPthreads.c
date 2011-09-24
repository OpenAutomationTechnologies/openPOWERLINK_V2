/*******************************************************************************

  File:         ShbIpc-LinuxPthreads.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  Pthreads based implementation for Linux userspace
                shared buffer module.

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
       permission, please contact office@br-automation.com.

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

*******************************************************************************/

/******************************************************************************/
/* Includes */
/******************************************************************************/
#include "EplInc.h"

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include "global.h"
#include "SharedBuff.h"
#include "ShbIpc.h"
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <sys/syscall.h>

/******************************************************************************/
/*          G L O B A L   D E F I N I T I O N S                               */
/******************************************************************************/

//------------------------------------------------------------------------------
//  Configuration
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  Constant definitions
//------------------------------------------------------------------------------
#define MAX_SHARED_BUFFERS              30
#define MAX_LEN_BUFFER_ID               256

#define TIMEOUT_ENTER_ATOMIC            1000        // timeout in us
#define TIMEOUT_CANCEL_THREAD           50000       // timeout in us
#define TIMEOUT_WAITING_THREAD          80000       // timeout in us

#define SBI_MAGIC_ID                    0x5342492B  // magic ID ("SBI+")
#define SBH_MAGIC_ID                    0x5342482A  // magic ID ("SBH*")

/* macro for adding timespec values */
#define timespecadd(vvp, uvp)                                           \
        {                                                               \
                (vvp)->tv_sec += (uvp)->tv_sec;                         \
                (vvp)->tv_nsec += (uvp)->tv_nsec;                       \
                if ((vvp)->tv_nsec >= 1000000000) {                     \
                        (vvp)->tv_sec++;                                \
                        (vvp)->tv_nsec -= 1000000000;                   \
                }                                                       \
        }

//------------------------------------------------------------------------------
//  Local types
//------------------------------------------------------------------------------

// This structure is the common header for the shared memory region used
// by all processes attached to this shared memory. It includes common
// information to administrate/manage the shared buffer from a couple of
// separated processes (e.g. the reference counter). This structure is
// located at the start of the shared memory region itself and exists
// consequently only one times per shared memory instance.
typedef struct
{
    UINT                m_uiBufferKey;
    ULONG               m_ulShMemSize;
    ULONG               m_ulRefCount;
    pthread_mutex_t     m_mutexBuffAccess;
    sem_t               m_semNewData;
    sem_t               m_semStopSignalingNewData;
    sem_t               m_semJobReady;
    tShbInstance*       m_pShbInstMaster;
} tShbMemHeader;

// This structure is the "external entry point" from a separate process
// to get access to a shared buffer. This structure includes all platform
// resp. target specific information to administrate/manage the shared
// buffer from a separate process. Every process attached to the shared
// buffer has its own runtime instance of this structure with its individual
// runtime data (e.g. the scope of an event handle is limited to the
// owner process only). The structure member <m_pShbMemHeader> points
// to the (process specific) start address of the shared memory region
// itself.
typedef struct
{
    ULONG               m_SbiMagicID;                   // magic ID ("SBI+")
    UINT                m_uiSharedMemId;
    char                m_sBufId[MAX_LEN_BUFFER_ID];
    BOOL                m_fThreadTermFlag;
    pthread_t           m_tThreadNewDataId;
    BOOL                m_fNewDataThreadStarted;
    pthread_t           m_tThreadJobReadyId;
    INT                 m_fJobReadyThreadStarted;
    tSigHndlrNewData    m_pfnSigHndlrNewData;
    ULONG               m_ulTimeOutMsJobReady;
    tSigHndlrJobReady   m_pfnSigHndlrJobReady;
    tShbMemHeader*      m_pShbMemHeader;
} tShbMemInst;

typedef struct
{
    UINT                m_key;
    void *              m_addr;
} tShbMemPoolEntry;

//------------------------------------------------------------------------------
//  Global variables
//------------------------------------------------------------------------------
static tShbMemPoolEntry        shbMemPool_g[MAX_SHARED_BUFFERS];
static UINT                    aulCrcTable_g[256];

//------------------------------------------------------------------------------
//  Prototypes of internal functions
//------------------------------------------------------------------------------
static inline tShbMemInst* ShbIpcGetShbMemInst(tShbInstance pShbInstance_p);
static inline tShbMemHeader* ShbIpcGetShbMemHeader(tShbInstance pShbInstance_p);

void* ShbIpcAllocPrivateMem(size_t ulMemSize_p);
void ShbIpcReleasePrivateMem (tShbMemInst *pShbMemInst_p);

void *ShbIpcThreadSignalNewData(void* pvThreadParam_p);
void *ShbIpcThreadSignalJobReady(void* pvThreadParam_p);

static void * ShbIpcFindMem(UINT bufferKey_p);
static void * ShbIpcAlloc(UINT bufferKey_p, size_t bufSize_p);
static int ShbIpcFree(UINT bufferKey_p);

static void ShbIpcCrc32GenTable();
static UINT ShbIpcCrc32GetCrc(const char *pcString);

//============================================================================//
//          P U B L I C   F U N C T I O N S                                   //
//============================================================================//

//------------------------------------------------------------------------------
// Function:    ShbIpcInit
//
// Description: Initialize IPC of shared buffer module
//
//              Nothing to do in this implementation!
//
// Parameters:  void
//
// Return:      tShbError = error
//------------------------------------------------------------------------------
tShbError  ShbIpcInit (void)
{
    memset (shbMemPool_g, 0, sizeof(tShbMemPoolEntry) * MAX_SHARED_BUFFERS);
    ShbIpcCrc32GenTable();

    return (kShbOk);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcExit
//
// Description: Deinitialize IPC of shared buffer module
//
//              Nothing to do in this implementation!
//
// Parameters:  void
//
// Return:      tShbError = error
//------------------------------------------------------------------------------
tShbError  ShbIpcExit (void)
{
    return (kShbOk);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcAllocBuffer
//
// Description: Allocates memory for the shared buffers
//
// Parameters:
//   ulBufferSize_p          size of the shared buffer to allocate
//   pszBufferId_p           string containing the shared buffer identifier
//   ppShbInstance_p         pointer to store the instance of this shared
//                           buffer
//   pfShbNewCreated_p       pointer to store the buffer creation flag
//
// Return:      tShbError = error
//------------------------------------------------------------------------------
tShbError ShbIpcAllocBuffer (ULONG ulBufferSize_p, const char* pszBufferID_p,
                              tShbInstance* ppShbInstance_p, UINT* pfShbNewCreated_p)
{
    tShbError               ShbError;
    UINT                    uiBufferKey;
    BOOL                    fShbNewCreated;
    ULONG                   ulShMemSize;
    tShbMemHeader*          pShbMemHeader;
    tShbMemInst*            pShbMemInst = NULL;
    tShbInstance            pShbInstance;

    ulShMemSize      = ulBufferSize_p + sizeof(tShbMemHeader);

    //create Buffer Key
    uiBufferKey = ShbIpcCrc32GetCrc(pszBufferID_p);

    EPL_DBGLVL_SHB_TRACE4("%s() Allocate %lu Bytes, sBufferID:%s BufferKey:%08x\n", __func__, ulShMemSize, pszBufferID_p, (key_t)uiBufferKey);
    //---------------------------------------------------------------
    // (1) open an existing or create a new shared memory
    //---------------------------------------------------------------
    // try to create shared memory

    if ((pShbMemHeader = ShbIpcFindMem(uiBufferKey)) == NULL)
    {
        fShbNewCreated = TRUE;
        if ((pShbMemHeader = ShbIpcAlloc(uiBufferKey, ulShMemSize)) == NULL)
        {
            //unable to create mem
            EPL_DBGLVL_ERROR_TRACE1("%s() Shared memory allocation error!\n", __func__);
            ShbError = kShbOutOfMem;
            goto Exit;
        }
        else
        {
            EPL_DBGLVL_SHB_TRACE4("%s() Shared memory allocated, Addr:%p Key:%08x size:%ld\n", __func__, (void *)pShbMemHeader, uiBufferKey, ulShMemSize);
        }
    }
    else
    {
        EPL_DBGLVL_SHB_TRACE4("%s() Attached to shared memory, Addr:%p Key:%08x size:%ld\n", __func__, (void *)pShbMemHeader, uiBufferKey, ulShMemSize);
        fShbNewCreated = FALSE;
    }

    //---------------------------------------------------------------
    // (2) setup or update header and management information
    //---------------------------------------------------------------

    // allocate a memory block from process specific mempool to save
    // process local information to administrate/manage the shared buffer
    if ((pShbMemInst = (tShbMemInst*)ShbIpcAllocPrivateMem(sizeof(tShbMemInst))) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() Couldn't alloc private mem!\n", __func__);
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    //TRACEX("%s() pShbMemInst:%08x\n", __func__, (unsigned int)pShbMemInst);
    memset(pShbMemInst, 0, sizeof(tShbMemInst));

    // reset complete header to default values
    pShbMemInst->m_SbiMagicID = SBI_MAGIC_ID;
    pShbMemInst->m_uiSharedMemId = uiBufferKey;
    strncpy(pShbMemInst->m_sBufId, pszBufferID_p, MAX_LEN_BUFFER_ID - 1);
    pShbMemInst->m_pfnSigHndlrNewData = NULL;
    pShbMemInst->m_fNewDataThreadStarted = FALSE;
    pShbMemInst->m_ulTimeOutMsJobReady = 0;
    pShbMemInst->m_pfnSigHndlrJobReady = NULL;
    pShbMemInst->m_pShbMemHeader = pShbMemHeader;
    pShbMemInst->m_fThreadTermFlag = FALSE;

    ShbError = kShbOk;

    if (fShbNewCreated)
    {
        memset (pShbMemHeader, 0, sizeof(tShbMemHeader));

        // this process was the first who wanted to use the shared memory,
        // so a new shared memory was created
        // -> setup new header information inside the shared memory region
        //    itself
        pShbMemHeader->m_ulShMemSize = ulShMemSize;
        pShbMemHeader->m_ulRefCount = 1;
        pShbMemHeader->m_uiBufferKey = uiBufferKey;

        //create semaphores for buffer access and signal new data
        if (pthread_mutex_init(&pShbMemHeader->m_mutexBuffAccess, NULL) != 0)
        {
            ShbError = kShbOutOfMem;
            goto Exit;
        }

        if (sem_init(&pShbMemHeader->m_semNewData, 0, 0) == -1)
        {
            ShbError = kShbOutOfMem;
            goto Exit;
        }

        if (sem_init(&pShbMemHeader->m_semStopSignalingNewData, 0, 0) == -1)
        {
            ShbError = kShbOutOfMem;
            goto Exit;
        }

        if (sem_init(&pShbMemHeader->m_semJobReady, 0, 0) == -1)
        {
            ShbError = kShbOutOfMem;
            goto Exit;
        }
    }
    else
    {
        // any other process has created the shared memory and this
        // process has only attached to it
        // -> check and update existing header information inside the
        //    shared memory region itself
        if (pShbMemHeader->m_uiBufferKey != uiBufferKey)
        {
            EPL_DBGLVL_ERROR_TRACE3("%s() Shared Mem mismatch buffer key %x:%x!\n", __func__, uiBufferKey, pShbMemHeader->m_uiBufferKey);
            ShbError = kShbOpenMismatch;
            goto Exit;

        }
        //TRACEX("%s() Check mem size is:%ld should be:%ld \n", __func__, pShbMemHeader->m_ulShMemSize, ulShMemSize);
        if (pShbMemHeader->m_ulShMemSize != ulShMemSize)
        {
            EPL_DBGLVL_ERROR_TRACE3("%s() Shared Mem mismatch size! %ld:%ld\n", __func__, ulShMemSize, pShbMemHeader->m_ulShMemSize);
            ShbError = kShbOpenMismatch;
            goto Exit;
        }
        pShbMemHeader->m_ulRefCount++;
    }

Exit:
    if (ShbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s() allocating shared buf failed!\n (%d)", __func__, ShbError);
        if (pShbMemInst != NULL)
        {
            ShbIpcReleasePrivateMem (pShbMemInst);
        }
        if ((pShbMemHeader != NULL) && (fShbNewCreated))
        {
            ShbIpcFree(uiBufferKey);
        }
    }

    pShbInstance = (tShbInstance*)pShbMemInst;
    *pfShbNewCreated_p = fShbNewCreated;
    *ppShbInstance_p   = pShbInstance;

    return (ShbError);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcReleaseBuffer
//
// Description: Releases a shared buffer
//
// Parameters:  pShbInstance_p          pointer to shared buffer instance
//
// Return:      tShbError = error code
//------------------------------------------------------------------------------
tShbError  ShbIpcReleaseBuffer (tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemInst = ShbIpcGetShbMemInst(pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader(pShbInstance_p);

    EPL_DBGLVL_SHB_TRACE3("%s() pShbInstance=%p pShbMemHeader=%p\n", __func__, (void *)pShbInstance_p, (void *)pShbMemHeader);

    // stop threads in any case, because they are bound to that specific instance
    ShbIpcStopSignalingNewData (pShbInstance_p);

    pShbMemHeader->m_ulRefCount--;

    if(pShbMemHeader->m_ulRefCount == 0)
    {
        EPL_DBGLVL_SHB_TRACE1("%s() refCount = 0,  destroy shared mem\n", __func__);
        //delete semaphores
        pthread_mutex_destroy(&pShbMemHeader->m_mutexBuffAccess);
        sem_destroy(&pShbMemHeader->m_semNewData);
        sem_destroy(&pShbMemHeader->m_semJobReady);
        sem_destroy(&pShbMemHeader->m_semStopSignalingNewData);

        //destroy Buffer
        ShbIpcFree(pShbMemHeader->m_uiBufferKey);

        ShbError = kShbOk;
    }
    else
    {
        EPL_DBGLVL_SHB_TRACE1("%s() refCount > 0, detach from shared mem\n", __func__);
        ShbError = kShbMemUsedByOtherProcs;
    }

    //delete private mem
    ShbIpcReleasePrivateMem (pShbMemInst);

    return (ShbError);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcProcess
//
// Description: Process function (only used for implementations without threads)
//
// Parameters:  void
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------

tShbError  ShbIpcProcess (void)
{
    tShbError       ShbError = kShbOk;

    // Nothing to do, we have threads!

    return ShbError;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcEnterAtomicSection
//
// Description: Enter atomic section for Shared Buffer access
//
// Parameters:  pShbInstance_p          pointer to shared buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError  ShbIpcEnterAtomicSection (tShbInstance pShbInstance_p)
{
    tShbMemInst         *pShbMemInst;
    tShbMemHeader       *pShbMemHeader;
    tShbError           ShbError;
    struct timespec     curTime, timeout;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    clock_gettime(CLOCK_REALTIME, &curTime);
    timeout.tv_sec = 0;
    timeout.tv_nsec = TIMEOUT_ENTER_ATOMIC * 1000;
    timespecadd(&timeout, &curTime);

    if (pthread_mutex_timedlock(&pShbMemHeader->m_mutexBuffAccess, &timeout) == 0)
    {
        ShbError = kShbOk;
    }
    else
    {
        ShbError = kShbBufferInvalid;
    }

    return (ShbError);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcLeaveAtomicSection
//
// Description: Leave atomic section for Shared Buffer access
//
// Parameters:  pShbInstance_p          pointer to shared buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError ShbIpcLeaveAtomicSection(tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    pthread_mutex_unlock(&pShbMemHeader->m_mutexBuffAccess);

    return (kShbOk);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcSetMaster
//
// Description: Set master instance of this slave instance
//
// Parameters:  pShbInstance_p          pointer to shared buffer instance
//              pShbInstanceMaster_p    pointer to master buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError  ShbIpcSetMaster(tShbInstance pShbInstance_p, tShbInstance pShbInstanceMaster_p)
{
    tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    /* store the pointer to the master in this instance */
    pShbMemHeader->m_pShbInstMaster = pShbInstanceMaster_p;

    return (kShbOk);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcStartSignalingNewData
//
// Description: Start signaling of new data (called from reading process)
//
// Parameters:  pShbInstance_p                  pointer to shared buffer instance
//              pfnSignalHandlerNewData_p       pointer to master buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError  ShbIpcStartSignalingNewData(tShbInstance pShbInstance_p,
                                       tSigHndlrNewData pfnSignalHandlerNewData_p,
                                       tShbPriority ShbPriority_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;
    struct sched_param  schedParam;
    INT                 iSchedPriority;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerNewData_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    ShbError = kShbOk;

    if ((pShbMemInst->m_fNewDataThreadStarted) ||
        (pShbMemInst->m_pfnSigHndlrNewData != NULL))
    {
        ShbError = kShbAlreadySignaling;
        EPL_DBGLVL_ERROR_TRACE1("%s() Thread already started!\n", __func__);
        goto Exit;
    }

    pShbMemInst->m_pfnSigHndlrNewData = pfnSignalHandlerNewData_p;

    iSchedPriority = EPL_THREAD_PRIORITY_MEDIUM;
    switch (ShbPriority_p)
    {
        case kShbPriorityLow:
            iSchedPriority -= 5;
            break;

        case kShbPriorityNormal:
            iSchedPriority += 0;
            break;

        case kShbPriorityHigh:
            iSchedPriority += 5;
            break;
    }

    //create thread for signaling new data
    if (pthread_create(&pShbMemInst->m_tThreadNewDataId, NULL,
                   &ShbIpcThreadSignalNewData, pShbInstance_p) != 0)
    {
        pShbMemInst->m_pfnSigHndlrNewData = NULL;
        ShbError = kShbInvalidSigHndlr;
        goto Exit;
    }

    schedParam.__sched_priority = iSchedPriority;
    if (pthread_setschedparam(pShbMemInst->m_tThreadNewDataId, SCHED_FIFO,
                              &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }

Exit:
    return (ShbError);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcStopSignalingNewData
//
// Description: Stop signaling of new data (called from reading process)
//
// Parameters:  pShbInstance_p        pointer to shared buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError ShbIpcStopSignalingNewData(tShbInstance pShbInstance_p)
{
    tShbMemInst         *pShbMemInst;
    tShbMemHeader       *pShbMemHeader;
    tShbError           ShbError;
    INT                 iRetVal = -1;
    sem_t*              pSemStopSignalingNewData;
    sem_t*              pSemNewData;
    struct timespec     curTime, timeout;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);

    pSemNewData = &pShbMemHeader->m_semNewData;
    pSemStopSignalingNewData = &pShbMemHeader->m_semStopSignalingNewData;
    ShbError = kShbOk;

    if (!pShbMemInst->m_fNewDataThreadStarted)
    {
        ShbError = kShbBufferAlreadyCompleted;
        goto Exit;
    }

    //set termination flag and signal new data to terminate thread
    pShbMemInst->m_fThreadTermFlag = TRUE;
    sem_post(pSemNewData);

    // waiting for thread to terminate
    clock_gettime(CLOCK_REALTIME, &curTime);
    timeout.tv_sec = 1;
    timeout.tv_nsec = TIMEOUT_WAITING_THREAD * 1000;
    timespecadd(&timeout, &curTime);
    iRetVal = sem_timedwait(pSemStopSignalingNewData, &timeout);
    if (iRetVal != 0)
    {
        EPL_DBGLVL_ERROR_TRACE3("%s() Stop Sem TIMEOUT %d (%s)\n", __func__, iRetVal, strerror(errno));
    }

Exit:

    return (ShbError);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcSignalNewData
//
// Description: Signal new data (called from writing process)
//
// Parameters:  pShbInstance_p        pointer to shared buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError ShbIpcSignalNewData(tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;

    //TRACEX("%s() pShbInstance_p=%p\n", __func__, pShbInstance_p);

    if (pShbInstance_p == NULL)
    {
        //TRACEX("%s() Invalid instance!\n", __func__);
        return (kShbInvalidArg);
    }
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    ShbError = kShbOk;

    //set semaphore
    if (sem_post(&pShbMemHeader->m_semNewData) < 0)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s() sem_post failed! (%s)\n", __func__, strerror(errno));
    }

    /* if this shared buffer is connected to a master buffer, signal the
     * data also to the master buffer.
     */
    if (pShbMemHeader->m_pShbInstMaster != NULL)
    {
        return ShbIpcSignalNewData(pShbMemHeader->m_pShbInstMaster);
    }

    return ShbError;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcStartSignalingJobReady
//
// Description: Start signaling for job ready (called from waiting process)
//
// Parameters:
//      pShbInstance_p                  pointer to shared buffer instance
//      ulTimeOut_p                     job ready timeout
//      pfnSignalHandlerJobReady_p      function pointer to callback function
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError ShbIpcStartSignalingJobReady(tShbInstance pShbInstance_p,
                                       unsigned long ulTimeOut_p,
                                       tSigHndlrJobReady pfnSignalHandlerJobReady_p)
{
    tShbMemInst*    pShbMemInst;
    tShbMemHeader*  pShbMemHeader;
    tShbError       ShbError;
    struct sched_param  schedParam;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerJobReady_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    ShbError = kShbOk;

    if ((pShbMemInst->m_fJobReadyThreadStarted) ||
        (pShbMemInst->m_pfnSigHndlrJobReady != NULL))
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }

    pShbMemInst->m_ulTimeOutMsJobReady = ulTimeOut_p;
    pShbMemInst->m_pfnSigHndlrJobReady = pfnSignalHandlerJobReady_p;

    //create thread for job ready signaling
    if (pthread_create(&pShbMemInst->m_tThreadJobReadyId, NULL,
                   &ShbIpcThreadSignalJobReady, pShbInstance_p) != 0)
    {
        pShbMemInst->m_pfnSigHndlrJobReady = NULL;
        ShbError = kShbInvalidSigHndlr;
        goto Exit;
    }

    schedParam.__sched_priority = EPL_THREAD_PRIORITY_LOW;
    if (pthread_setschedparam(pShbMemInst->m_tThreadNewDataId, SCHED_FIFO,
                              &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }

Exit:

    return ShbError;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcSignalJobReady
//
// Description: Signal job ready (called from executing process)
//
// Parameters:
//      pShbInstance_p                  pointer to shared buffer instance
//
// Return:      tShbError      = error code
//------------------------------------------------------------------------------
tShbError ShbIpcSignalJobReady(tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    ShbError = kShbOk;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    //set semaphore
    sem_post(&pShbMemHeader->m_semJobReady);

    return ShbError;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcGetShMemPtr
//
// Description: Get pointer to common used shared memory area
//
// Parameters:
//      pShbInstance_p          pointer to shared buffer instance
//
// Return:      void *          pointer to shared memory
//------------------------------------------------------------------------------
void*  ShbIpcGetShMemPtr (tShbInstance pShbInstance_p)
{
    tShbMemHeader*  pShbMemHeader;
    void*           pShbShMemPtr;

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
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

//=========================================================================//
//          P R I V A T E   F U N C T I O N S                              //
//=========================================================================//

//------------------------------------------------------------------------------
// Function:    ShbIpcGetShbMemInst
//
// Description: Get pointer to process local information structure
//
// Parameters:
//      pShbInstance_p          pointer to shared buffer instance
//
// Return:      tShbMemInst*    pointer to local information structure
//------------------------------------------------------------------------------
static inline tShbMemInst* ShbIpcGetShbMemInst (tShbInstance pShbInstance_p)
{
    tShbMemInst*  pShbMemInst;
    pShbMemInst = (tShbMemInst*)pShbInstance_p;
    return (pShbMemInst);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcGetShbMemHeader
//
// Description:  Get pointer to shared memory header
//
// Parameters:
//      pShbInstance_p          pointer to shared buffer instance
//
// Return:      tShbMemHeader*  pointer to shared memory header
//------------------------------------------------------------------------------
static inline tShbMemHeader* ShbIpcGetShbMemHeader (tShbInstance pShbInstance_p)
{
    tShbMemInst*    pShbMemInst;
    tShbMemHeader*  pShbMemHeader;

    pShbMemInst   = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = pShbMemInst->m_pShbMemHeader;
    return (pShbMemHeader);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcAllocPrivateMem
//
// Description:  Allocate a memory block from process specific mempool
//
// Parameters:
//      ulMemSize_p          size of private memory to allocate
//      iBufferKey           buffer key of shared memory block
//      iBufferId_p          pointer to store shared buffer ID of private mem
//
// Return:      void*        pointer to private memory
//------------------------------------------------------------------------------
void* ShbIpcAllocPrivateMem (size_t ulMemSize_p)
{
    return malloc(ulMemSize_p);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcReleasePrivateMem
//
// Description: Release a memory block from process specific mempool
//
// Parameters:
//      iBufferId_p          shared buffer ID of private memory
//      pMem_p               pointer to private memory
//
// Return:      void
//------------------------------------------------------------------------------
void ShbIpcReleasePrivateMem (tShbMemInst *pShbMemInst_p)
{
    free(pShbMemInst_p);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcThreadSignalNewData
//
// Description: Thread for new data signaling
//
// Parameters:
//      pvThreadParam_p         user parameters for thread
//
// Return:      void *          thread exit value (not used)
//------------------------------------------------------------------------------
void *ShbIpcThreadSignalNewData (void *pvThreadParam_p)
{
    tShbInstance        pShbInstance;
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    sem_t*              pSemNewData;
    sem_t*              pSemStopSignalingNewData;
    struct timespec     curTime, timeout;
    INT                 iRetVal;

    EPL_DBGLVL_SHB_TRACE2("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    /* thread parameter contains pointer to shared memory */
    pShbInstance = (tShbMemInst*)pvThreadParam_p;

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance);

    /* signal that thread is running */
    pShbMemInst->m_fNewDataThreadStarted = TRUE;

    pSemNewData = &pShbMemHeader->m_semNewData;
    pSemStopSignalingNewData = &pShbMemHeader->m_semStopSignalingNewData;

    do
    {
        clock_gettime(CLOCK_REALTIME, &curTime);
        timeout.tv_sec = 0;
        timeout.tv_nsec = TIMEOUT_CANCEL_THREAD * 1000;
        timespecadd(&timeout, &curTime);

        if ((iRetVal = sem_timedwait(pSemNewData, &timeout)) == 0)
        {
            //check terminate flag
            if (!pShbMemInst->m_fThreadTermFlag)
            {
                //call Rx Handler
                //TRACEX("%s() ShbIpcThreadSignalNewData call Rx Handler (%s) sem:%08x instance:%08x\n", __func__, pShbMemInst->m_bufName, (unsigned int)iSemNewDataId, (unsigned int)pShbInstance);
                pShbMemInst->m_pfnSigHndlrNewData(pShbInstance);
            }
        }
    } while(!pShbMemInst->m_fThreadTermFlag);

    //set sem thread terminated
    pShbMemInst->m_fNewDataThreadStarted = FALSE;
    sem_post(pSemStopSignalingNewData);

    return NULL;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcThreadSignalJobReady
//
// Description: Thread for new data Job Ready signaling
//
// Parameters:
//      pvThreadParam_p         user parameters for thread
//
// Return:      void *          thread exit value (not used)
//------------------------------------------------------------------------------
void *ShbIpcThreadSignalJobReady (void *pvThreadParam_p)
{
    tShbInstance        pShbInstance;
    tShbMemInst         *pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    DWORD               ulTimeOut;
    INT                 iRetVal;
    UINT                fTimeOut;
    struct timespec     timeout;

    EPL_DBGLVL_SHB_TRACE2("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance);

    pShbMemInst->m_fJobReadyThreadStarted = TRUE;

    if (pShbMemInst->m_ulTimeOutMsJobReady != 0)
    {
        ulTimeOut = pShbMemInst->m_ulTimeOutMsJobReady;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += ulTimeOut;
        iRetVal = sem_timedwait(&pShbMemHeader->m_semJobReady, &timeout);
    }
    else
    {
        iRetVal = sem_wait(&pShbMemHeader->m_semJobReady);
    }

    if (iRetVal == 0)
    {
        fTimeOut = FALSE;
    }
    else
    {
        /* timeout or error */
        fTimeOut = TRUE;
    }

    if (pShbMemInst->m_pfnSigHndlrJobReady != NULL)
    {
        //call Handler
        pShbMemInst->m_pfnSigHndlrJobReady(pShbInstance, fTimeOut);
    }

    pShbMemInst->m_pfnSigHndlrJobReady = NULL;
    pShbMemInst->m_fJobReadyThreadStarted = FALSE;

    return NULL;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//------------------------------------------------------------------------------
// Function:    ShbIpcCrc32GenTable
//
// Description: generate the CRC32 generation table
//
// Parameters:  aulCrcTable             pointer to store the table
//
// Return:      void
//------------------------------------------------------------------------------
static void ShbIpcCrc32GenTable()
{
    UINT        uiCrc, uiPoly;
    INT         iIndexI, iIndexJ;

    uiPoly = 0xEDB88320L;
    for (iIndexI = 0; iIndexI < 256; iIndexI++)
    {
        uiCrc = iIndexI;
        for (iIndexJ = 8; iIndexJ > 0; iIndexJ--)
        {
            if (uiCrc & 1)
            {
                uiCrc = (uiCrc >> 1) ^ uiPoly;
            }
            else
            {
                uiCrc >>= 1;
            }
        }
        aulCrcTable_g[iIndexI] = uiCrc;
    }
}

//------------------------------------------------------------------------------
// Function:    ShbIpcCrc32GetCrc
//
// Description: create CRC32 checksum of buffer identifier
//
// Parameters:
//      pcString                buffer identifier string
//      aulCrcTable             CRC32 generation table
//
// Return:      void
//------------------------------------------------------------------------------
static UINT ShbIpcCrc32GetCrc(const char *pcString)
{
    UINT   uiCrc;
    UINT   uiIndex;

    uiCrc = 0xFFFFFFFF;
    for (uiIndex = 0; uiIndex < strlen(pcString); uiIndex++)
    {
        uiCrc = ((uiCrc >> 8) & 0x00FFFFFF) ^ aulCrcTable_g[(uiCrc ^ pcString[uiIndex]) & 0xFF];
    }
    return(uiCrc ^ 0xFFFFFFFF);
}

//------------------------------------------------------------------------------
// Function:    ShbIpcFindMem
//
// Description: find a buffer key in the list of used buffer keys
//
// Parameters:
//      bufferKey_p             buffer key to search for
//
// Return:      TRUE, if key was found
//              FALSE, if key wasn't found
//------------------------------------------------------------------------------
static void * ShbIpcFindMem(UINT bufferKey_p)
{
    UINT        uiIndex;

    for (uiIndex = 0; uiIndex < MAX_SHARED_BUFFERS; uiIndex++)
    {
        if (bufferKey_p == shbMemPool_g[uiIndex].m_key)
        {
            return shbMemPool_g[uiIndex].m_addr;
        }
    }
    return NULL;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcAlloc
//
// Description: adds a buffer key to the list of used buffer keys
//
// Parameters:
//      bufferKey_p             buffer key to add to the list
//
// Return:      TRUE, if key was added
//              FALSE, if there was no free space
//------------------------------------------------------------------------------
static void * ShbIpcAlloc(UINT bufferKey_p, size_t bufSize_p)
{
    UINT        uiIndex;
    void        * pAddr;

    for (uiIndex = 0; uiIndex < MAX_SHARED_BUFFERS; uiIndex++)
    {
        if (shbMemPool_g[uiIndex].m_key == 0)
        {
            if ((pAddr = malloc(bufSize_p)) == NULL)
            {
                return NULL;
            }
            else
            {
                shbMemPool_g[uiIndex].m_key = bufferKey_p;
                shbMemPool_g[uiIndex].m_addr = pAddr;
                return pAddr;
            }
        }
    }
    return NULL;
}

//------------------------------------------------------------------------------
// Function:    ShbIpcFree
//
// Description: removes a buffer key of the list of used buffer keys
//
// Parameters:
//      bufferKey_p             buffer key to remove from list
//
// Return:      TRUE, if key was removed
//              FALSE, if key couldn't be found
//------------------------------------------------------------------------------
static int ShbIpcFree(UINT bufferKey_p)
{
    UINT        uiIndex;

    for (uiIndex = 0; uiIndex < MAX_SHARED_BUFFERS; uiIndex++)
    {
        if (shbMemPool_g[uiIndex].m_key == bufferKey_p)
        {
            free (shbMemPool_g[uiIndex].m_addr);
            shbMemPool_g[uiIndex].m_key = 0;
            shbMemPool_g[uiIndex].m_addr = NULL;
            return 0;
        }
    }
    return -1;
}

