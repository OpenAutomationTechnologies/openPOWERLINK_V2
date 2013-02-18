/**
********************************************************************************
\file   ShbIpc-LinuxUser.c

\brief  Linux userspace implementation of shared buffers

This file contains the Linux userspace implementation of the shared buffer
library. This implementation uses SYSV shared memory and BSD semaphores for
inter process communication.

\ingroup module_shbipc
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
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <sys/stat.h>

#include <EplInc.h>
#include "global.h"
#include "SharedBuff.h"
#include "ShbIpc.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define MAX_LEN_BUFFER_ID               256

#define IDX_EVENT_NEW_DATA              0
#define IDX_EVENT_TERM_REQU             1
#define IDX_EVENT_TERM_RESP             2

#define TIMEOUT_ENTER_ATOMIC            1000        // (ms) for debgging: INFINITE
#define TIMEOUT_CANCEL_THREAD           50000       // timeout in us
#define TIMEOUT_WAITING_THREAD          80000       // timeout in us

#define SBI_MAGIC_ID                    0x5342492B  // magic ID ("SBI+")
#define SBH_MAGIC_ID                    0x5342482A  // magic ID ("SBH*")

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//  Local types
//-----------------------------------------------------------------------------

// This structure is the common header for the shared memory region used
// by all processes attached this shared memory. It includes common
// information to administrate/manage the shared buffer from a couple of
// separated processes (e.g. the refernce counter). This structure is
// located at the start of the shared memory region itself and exists
// consequently only one times per shared memory instance.
typedef struct
{
    int                 m_iBufferId;
    ULONG               m_ulShMemSize;
    ULONG               m_ulRefCount;

    #ifndef NDEBUG
        unsigned long   m_ulOwnerProcID;
    #endif
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
typedef struct
{
    unsigned long       m_SbiMagicID;           // magic ID ("SBI+")
    int                 m_iSharedMemId;
    unsigned long       m_iBufferKey;
    BOOL                m_fThreadTermFlag;
    pthread_t           m_tThreadNewDataId;
    BOOL                m_fNewDataThreadStarted;
    pthread_t           m_tThreadJobReadyId;
    INT                 m_fJobReadyThreadStarted;
    sem_t               *m_mutexBuffAccess;
    sem_t               *m_semNewData;
    sem_t               *m_semStopSignalingNewData;
    sem_t               *m_semJobReady;
    tSigHndlrNewData    m_pfnSigHndlrNewData;
    ULONG               m_ulTimeOutMsJobReady;
    tSigHndlrJobReady   m_pfnSigHndlrJobReady;
    tShbMemHeader*      m_pShbMemHeader;

    #ifndef NDEBUG
        unsigned long   m_ulThreadIDNewData;
        unsigned long   m_ulThreadIDJobReady;
    #endif
} tShbMemInst;

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT                    aulCrcTable_g[256];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tShbMemInst*     ShbIpcGetShbMemInst(tShbInstance pShbInstance_p);
static tShbMemHeader*   ShbIpcGetShbMemHeader(tShbInstance pShbInstance_p);

static void*            ShbIpcAllocPrivateMem(ULONG ulMemSize_p,
                                              int iBufferKey,int *iBufferId_p);
static void             ShbIpcReleasePrivateMem(tShbMemInst *pShbMemInst_p);

static void             *ShbIpcThreadSignalNewData(void* pvThreadParam_p);
static void             *ShbIpcThreadSignalJobReady(void* pvThreadParam_p);

static void             ShbIpcCrc32GenTable(void);
static ULONG            ShbIpcCrc32GetCrc(const char *pcString);
static void             timespecadd(struct timespec *time1_p,
                                    struct timespec *time2_p);


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize shared buffer IPCmodule

This function initializes the shared buffer IPC module.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError  ShbIpcInit (void)
{
    ShbIpcCrc32GenTable();
    return (kShbOk);
}

//------------------------------------------------------------------------------
/**
\brief  Deinitialize the shared buffer IPC module

This function deinitializes the shared buffer IPC module.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError  ShbIpcExit (void)
{
    return (kShbOk);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate shared buffer memory

This function allocates the shared buffer memory.

\param  ulBufferSize_p          The size of the shared buffer.
\param  pszBufferID_p           The shared buffer ID.
\param  ppShbInstance_p         A pointer to store the instance pointer of the
                                allocated shared buffer.
\param  pfShbNewCreated_p       A pointer to store the flag which determines
                                if the buffer was created.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError  ShbIpcAllocBuffer (ULONG ulBufferSize_p, const char* pszBufferID_p,
                              tShbInstance* ppShbInstance_p, UINT* pfShbNewCreated_p)
{
    tShbError               ShbError;
    int                     iBufferId;
    int                     iPrivBufferId;
    unsigned long           ulCrc32=0;
    int                     iBufferKey;
    unsigned long           ulShMemSize;
    tShbMemHeader*          pShbMemHeader;
    tShbMemInst*            pShbMemInst;
    tShbInstance            pShbInstance;
    unsigned int            fShMemNewCreated=FALSE;
    void                    *pSharedMem=NULL;
    char                    semName[256];
    struct shmid_ds         shminfo;

    ulShMemSize = ulBufferSize_p + sizeof(tShbMemHeader);
    ShbError = kShbOk;

    //create Buffer Key
    ulCrc32 = ShbIpcCrc32GetCrc(pszBufferID_p);
    iBufferKey = (int)ulCrc32;
    EPL_DBGLVL_SHB_TRACE("%s(): Name:%s Key:%08x Size:%d\n",
                          __func__, pszBufferID_p, iBufferKey, ulBufferSize_p);
    pShbMemInst = NULL;
    pShbMemHeader = NULL;

    //---------------------------------------------------------------
    // (1) open an existing or create a new shared memory
    //---------------------------------------------------------------
    // try to create an shared memory

    iBufferId = shmget(iBufferKey, ulShMemSize, 0666|IPC_CREAT|IPC_EXCL);
    EPL_DBGLVL_SHB_TRACE("          iBufferId:%08x iBufferKey:%08x\n", iBufferId, iBufferKey);
    // todo: check ernno for EEXIST to be shure!
    if (iBufferId == -1)
    {
        // a shared memory already exist, get buffer id
        iBufferId = shmget(iBufferKey, ulShMemSize, 0666);
        EPL_DBGLVL_SHB_TRACE("          iBufferId:%08x iBufferKey:%08x\n", iBufferId, iBufferKey);
        if (iBufferId == -1)
        {
            EPL_DBGLVL_ERROR_TRACE ("%s() Allocate memory failed!\n", __func__);
            ShbError = kShbOutOfMem;
            goto Exit;
        }

        if (shmctl(iBufferId, IPC_STAT, &shminfo) != -1)
        {
            if (shminfo.shm_nattch == 0)
                fShMemNewCreated = TRUE;
            else
                fShMemNewCreated = FALSE;
        }
        else
        {
            ShbError = kShbOutOfMem;
            goto Exit;
        }
    }
    else
    {
        //shared memory is new created
        fShMemNewCreated = TRUE;
    }

    // attach shared buf
    pSharedMem = shmat(iBufferId, NULL, 0);
    if (pSharedMem == (void *)-1)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() Attaching shared memory failed!\n", __func__);
        //unable to create mem
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    //update header
    pShbMemHeader = (tShbMemHeader*)pSharedMem;

    //---------------------------------------------------------------
    // (2) setup or update header and management information
    //---------------------------------------------------------------

    // allocate a memory block from process specific mempool to save
    // process local information to administrate/manage the shared buffer
    pShbMemInst = (tShbMemInst*) ShbIpcAllocPrivateMem (sizeof(tShbMemInst),
                                                        iBufferKey,
                                                        &iPrivBufferId);
    if (pShbMemInst == NULL)
    {
        TRACE ("Alloc priv failed!\n");
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    memset(pShbMemInst, 0, sizeof(tShbMemInst));

    // reset complete header to default values
    pShbMemInst->m_SbiMagicID = SBI_MAGIC_ID;
    pShbMemInst->m_iSharedMemId = iPrivBufferId;
    pShbMemInst->m_iBufferKey = iBufferKey;
    pShbMemInst->m_pfnSigHndlrNewData = NULL;

    pShbMemInst->m_ulTimeOutMsJobReady = 0;
    pShbMemInst->m_pfnSigHndlrJobReady = NULL;
    pShbMemInst->m_pShbMemHeader = pShbMemHeader;
    pShbMemInst->m_fThreadTermFlag = FALSE;

    //create semaphores for buffer access and signal new data
    sprintf (semName, "/semShbMutBufAcc-%d", iBufferId);
    if ((pShbMemInst->m_mutexBuffAccess =
                    sem_open(semName, O_CREAT, S_IRWXG, 1)) == SEM_FAILED)
    {
        EPL_DBGLVL_ERROR_TRACE ("creating sem %s failed!\n", semName);
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    sprintf (semName, "/semShbNewData-%d", iBufferId);
    if ((pShbMemInst->m_semNewData =
                    sem_open(semName, O_CREAT, S_IRWXG, 0)) == SEM_FAILED)
    {
        EPL_DBGLVL_ERROR_TRACE ("creating sem %s failed!\n", semName);
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    sprintf (semName, "/semShbStopSigNewData-%d", iBufferId);
    if ((pShbMemInst->m_semStopSignalingNewData =
                    sem_open(semName, O_CREAT, S_IRWXG, 0)) == SEM_FAILED)
    {
        EPL_DBGLVL_ERROR_TRACE ("creating sem %s failed!\n", semName);
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    sprintf (semName, "/semShbJobReady-%d", iBufferId);
    if ((pShbMemInst->m_semJobReady =
                    sem_open(semName, O_CREAT, S_IRWXG, 0)) == SEM_FAILED)
    {
        EPL_DBGLVL_ERROR_TRACE ("creating sem %s failed!\n", semName);
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    if (fShMemNewCreated)
    {
        memset (pShbMemHeader, 0, sizeof(tShbMemHeader));

        // this process was the first who wanted to use the shared memory,
        // so a new shared memory was created
        // -> setup new header information inside the shared memory region
        //    itself
        pShbMemHeader->m_ulShMemSize = ulShMemSize;
        pShbMemHeader->m_ulRefCount = 1;
        pShbMemHeader->m_iBufferId = iBufferId;
    }
    else
    {
        // any other process has created the shared memory and this
        // process has only attached to it
        // -> check and update existing header information inside the
        //    shared memory region itself
        EPL_DBGLVL_SHB_TRACE("MEM %d %d \n", pShbMemHeader->m_ulShMemSize, ulShMemSize);
        if (pShbMemHeader->m_ulShMemSize != ulShMemSize)
        {
            EPL_DBGLVL_ERROR_TRACE("%s() Shared Mem mismatch size! %ld:%ld\n",
                                    __func__, ulShMemSize, pShbMemHeader->m_ulShMemSize);
            ShbError = kShbOpenMismatch;
            goto Exit;
        }
        pShbMemHeader->m_ulRefCount++;
    }

Exit:
    if (ShbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() allocating shared buf failed! (%d)\n", __func__, ShbError);
        if (pShbMemInst != NULL)
        {
            ShbIpcReleasePrivateMem (pShbMemInst);
        }

        if (pShbMemHeader != NULL)
        {
            shmdt(pShbMemHeader);

            if (fShMemNewCreated)
            {
                shmctl(iBufferId, IPC_RMID, 0);             //destroy Buffer

            }
        }
    }

    pShbInstance = (tShbInstance*)pShbMemInst;
    *pfShbNewCreated_p = fShMemNewCreated;
    *ppShbInstance_p   = pShbInstance;

    return ShbError;
}

//------------------------------------------------------------------------------
/**
\brief  Release shared buffer memory

This function releases the shared buffer memory.

\param  pShbInstance_p          The shared buffer instance to release.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError  ShbIpcReleaseBuffer (tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;
    int                 iBufferId;
    char                semName[256];

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    EPL_DBGLVL_SHB_TRACE("%s() %08x\n", __func__, pShbMemInst->m_iBufferKey);
    // stop threads in any case, because they are bound to that specific instance
    ShbIpcStopSignalingNewData (pShbInstance_p);

    pShbMemHeader->m_ulRefCount--;

    sem_close(pShbMemInst->m_mutexBuffAccess);
    sem_close(pShbMemInst->m_semNewData);
    sem_close(pShbMemInst->m_semJobReady);
    sem_close(pShbMemInst->m_semStopSignalingNewData);

    if (pShbMemHeader->m_ulRefCount == 0)
    {
        EPL_DBGLVL_SHB_TRACE("%s() refCount = 0,  destroy shared mem\n", __func__);
        sprintf (semName, "/semShbMutBufAcc-%d", pShbMemHeader->m_iBufferId);
        sem_unlink(semName);
        sprintf (semName, "/semShbNewData-%d", pShbMemHeader->m_iBufferId);
        sem_unlink(semName);
        sprintf (semName, "/semShbStopSigNewData-%d", pShbMemHeader->m_iBufferId);
        sem_unlink(semName);
        sprintf (semName, "/semShbJobReady-%d", pShbMemHeader->m_iBufferId);
        sem_unlink(semName);

        iBufferId = pShbMemHeader->m_iBufferId;
        shmdt(pShbMemHeader);
        shmctl(iBufferId, IPC_RMID, 0);

        ShbError = kShbOk;
    }
    else
    {
        EPL_DBGLVL_SHB_TRACE("%s() refCount > 0, detach from shared mem\n", __func__);
        shmdt(pShbMemHeader);
        ShbError = kShbOk;
    }

    //delete privat mem
    ShbIpcReleasePrivateMem (pShbMemInst);

    return (ShbError);
}

//------------------------------------------------------------------------------
/**
\brief Shared buffer process function

This function is the process handler function. It is needed only for
implementations without threads. Therefore it is not implemented.

\return Returns an tShbError code. (Always kShbOk)

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcProcess (void)
{
    tShbError       ShbError = kShbOk;

    return ShbError;
}

//------------------------------------------------------------------------------
/**
\brief Enter atomic section

This function is called to enter an atomic section. It uses a semaphore for
implementing locking.

\param  pShbInstance_p          The shared buffer instance to use.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcEnterAtomicSection (tShbInstance pShbInstance_p)
{
    tShbMemInst         *pShbMemInst;
    int                 iRetVal = -1;
    struct timespec     curTime, timeout;
    tShbError           shbError;

    if (pShbInstance_p == NULL)
        return kShbInvalidArg;

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);

    // waiting for thread to terminate
    clock_gettime(CLOCK_REALTIME, &curTime);
    timeout.tv_sec = 1;
    timeout.tv_nsec = TIMEOUT_ENTER_ATOMIC * 1000;
    timespecadd(&timeout, &curTime);
    iRetVal = sem_timedwait(pShbMemInst->m_mutexBuffAccess, &timeout);
    if (iRetVal == 0)
    {
        shbError = kShbOk;
    }
    else
    {
        EPL_DBGLVL_SHB_TRACE("ShbIpcEnterAtomicSection Enter Atomic not ok %d\n",iRetVal);
        shbError = kShbBufferInvalid;
    }
    return shbError;
}

//------------------------------------------------------------------------------
/**
\brief Leave atomic section

This function is called to leave an atomic section. It uses a semaphore for
implementing locking.

\param  pShbInstance_p          The shared buffer instance to use.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcLeaveAtomicSection (tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;

    if (pShbInstance_p == NULL)
        return (kShbInvalidArg);

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);

    /* TODO works only if semaphore is not accidently posted by process that
     doesn't have the lock! */
    sem_post(pShbMemInst->m_mutexBuffAccess);

    return (kShbOk);
}

//------------------------------------------------------------------------------
/**
\brief Set master shared buffer

This function sets the master shared buffer instance of this (slave) instance.

\param  pShbInstance_p          The shared buffer instance to use.
\param  pShbInstanceMaster_p    The instance of the master shared buffer.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcSetMaster(tShbInstance pShbInstance_p,
                           tShbInstance pShbInstanceMaster_p)
{
    tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
        return (kShbInvalidArg);

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    pShbMemHeader->m_pShbInstMaster = pShbInstanceMaster_p;

    return (kShbOk);
}

//------------------------------------------------------------------------------
/**
\brief Start signaling new data

This function starts the signaling of new data. It creates and sets up the
thread which is responsible to read data posted in this shared buffer.

\param  pShbInstance_p              The shared buffer instance to use.
\param  pfnSignalHandlerNewData_p   Function pointer to signal handler which
                                    is connected to the receiving thread.
\param  ShbPriority_p               The priority of the receiving thread.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcStartSignalingNewData (tShbInstance pShbInstance_p,
                                       tSigHndlrNewData pfnSignalHandlerNewData_p,
                                       tShbPriority ShbPriority_p)
{
    tShbMemInst*        pShbMemInst;
    tShbError           ShbError;
    struct sched_param  schedParam;
    INT                 iSchedPriority;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerNewData_p == NULL))
        return kShbInvalidArg;

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    ShbError = kShbOk;

    EPL_DBGLVL_SHB_TRACE("%s() ID:%08x func:%08x Prio:%d)\n", __func__,
                         pShbMemInst->m_iSharedMemId,
                         pfnSignalHandlerNewData_p, ShbPriority_p);

    if ((pShbMemInst->m_fNewDataThreadStarted) ||
        (pShbMemInst->m_pfnSigHndlrNewData != NULL))
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Already signaling!\n", __func__);
        ShbError = kShbAlreadySignaling;
        return ShbError;
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

    //create thread for signalling new data
    if (pthread_create(&pShbMemInst->m_tThreadNewDataId, NULL,
                   &ShbIpcThreadSignalNewData, pShbInstance_p) != 0)
    {
        pShbMemInst->m_pfnSigHndlrNewData = NULL;
        ShbError = kShbInvalidSigHndlr;
        return ShbError;
    }

    schedParam.__sched_priority = iSchedPriority;
    if (pthread_setschedparam(pShbMemInst->m_tThreadNewDataId, SCHED_FIFO,
                              &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }
    return ShbError;
}

//------------------------------------------------------------------------------
/**
\brief Stop signaling new data

This function stops the signaling of new data. It notifies the receiving thread
that it should terminate.

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcStopSignalingNewData (tShbInstance pShbInstance_p)
{
    tShbMemInst         *pShbMemInst;
    int                 iRetVal = -1;
    struct timespec     curTime, timeout;
    tShbError           ShbError;

    if (pShbInstance_p == NULL)
        return (kShbInvalidArg);

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);

    ShbError = kShbOk;
    if (!pShbMemInst->m_fNewDataThreadStarted)
    {
        ShbError = kShbBufferAlreadyCompleted;
        return ShbError;
    }

    //set termination flag and signal new data to terminate thread
    pShbMemInst->m_fThreadTermFlag = TRUE;
    sem_post(pShbMemInst->m_semNewData);

    // waiting for thread to terminate
    clock_gettime(CLOCK_REALTIME, &curTime);
    timeout.tv_sec = 1;
    timeout.tv_nsec = TIMEOUT_WAITING_THREAD * 1000;
    timespecadd(&timeout, &curTime);
    iRetVal = sem_timedwait(pShbMemInst->m_semStopSignalingNewData, &timeout);
    if (iRetVal != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Stop Sem TIMEOUT %d (%s)\n", __func__,
                               iRetVal, strerror(errno));
    }
    return (ShbError);
}

//------------------------------------------------------------------------------
/**
\brief Signaling new data

This function is used by the writing process to signal that new data is
available.

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcSignalNewData (tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbMemHeader*      pShbMemHeader;
    tShbError           ShbError;

    if (pShbInstance_p == NULL)
            return (kShbInvalidArg);

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    EPL_DBGLVL_SHB_TRACE ("%s() ID:%08x\n", __func__, pShbMemHeader->m_iBufferId);
    ShbError = kShbOk;

    //set semaphore
    if (sem_post(pShbMemInst->m_semNewData) < 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() sem_post failed! (%s)\n", __func__, strerror(errno));
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
/**
\brief Starts job ready signaling

This function start a job ready signaling. It creates a thread for signaling.

\param  pShbInstance_p              The shared buffer instance to use.
\param  ulTimeOut_p                 The job ready timeout.
\param  pfnSignalHandlerJobReady_p  The function pointer to the job ready
                                    callback function.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcStartSignalingJobReady (tShbInstance pShbInstance_p,
                                         ULONG ulTimeOut_p,
                                         tSigHndlrJobReady pfnSignalHandlerJobReady_p)
{
    tShbMemInst*        pShbMemInst;
    tShbError           ShbError;
    struct sched_param  schedParam;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerJobReady_p == NULL))
        return (kShbInvalidArg);

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    ShbError = kShbOk;

    if ((pShbMemInst->m_fJobReadyThreadStarted) ||
        (pShbMemInst->m_pfnSigHndlrJobReady != NULL))
    {
        ShbError = kShbAlreadySignaling;
        return ShbError;
    }

    pShbMemInst->m_ulTimeOutMsJobReady = ulTimeOut_p;
    pShbMemInst->m_pfnSigHndlrJobReady = pfnSignalHandlerJobReady_p;

    //create thread for job ready signaling
    if (pthread_create(&pShbMemInst->m_tThreadJobReadyId, NULL,
                   &ShbIpcThreadSignalJobReady, pShbInstance_p) != 0)
    {
        pShbMemInst->m_pfnSigHndlrJobReady = NULL;
        ShbError = kShbInvalidSigHndlr;
        return ShbError;
    }

    schedParam.__sched_priority = EPL_THREAD_PRIORITY_LOW;
    if (pthread_setschedparam(pShbMemInst->m_tThreadJobReadyId, SCHED_FIFO,
                              &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief Signaling that a job is ready

This function signals that a job is ready

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns an tShbError code.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbError ShbIpcSignalJobReady (tShbInstance pShbInstance_p)
{
    tShbMemInst*        pShbMemInst;
    tShbError           ShbError;

    if (pShbInstance_p == NULL)
        return (kShbInvalidArg);

    ShbError = kShbOk;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);

    //set semaphore
    sem_post(pShbMemInst->m_semJobReady);
    return ShbError;
}

//------------------------------------------------------------------------------
/**
\brief Get pointer to common shared memory area

This function returns the pointer to the common shared memory area of the
specified shared buffer.

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns the pointer to the shared memory.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
void* ShbIpcGetShMemPtr (tShbInstance pShbInstance_p)
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

//------------------------------------------------------------------------------
/**
\brief Get pointer to local shared buffer information

This function returns the pointer to the process local shared buffer
information structure.

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns the pointer to the share buffer information structure.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbMemInst* ShbIpcGetShbMemInst (tShbInstance pShbInstance_p)
{
    tShbMemInst*  pShbMemInst;

    pShbMemInst = (tShbMemInst*)pShbInstance_p;
    return (pShbMemInst);
}

//------------------------------------------------------------------------------
/**
\brief Get pointer to shared memory header

This function returns the pointer to the shared memory header.

\param  pShbInstance_p              The shared buffer instance to use.

\return Returns the pointer to the share buffer information structure.

\ingroup module_shbipc
*/
//------------------------------------------------------------------------------
tShbMemHeader* ShbIpcGetShbMemHeader (tShbInstance pShbInstance_p)
{
    tShbMemInst*    pShbMemInst;
    tShbMemHeader*  pShbMemHeader;

    pShbMemInst   = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = pShbMemInst->m_pShbMemHeader;
    return (pShbMemHeader);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Allocate shared buffer private memory

This function allocates the private memory block for the shared buffer.

\param ulMemSize_p          Size of private memory to allocate.
\param iBufferKey           Buffer key of shared memory block.
\param iBufferId_p          Pointer to store shared buffer ID of private mem.

\return Returns the pointer to the allocated memory.
*/
//------------------------------------------------------------------------------
static void* ShbIpcAllocPrivateMem (ULONG ulMemSize_p, int iBufferKey,
                                    int *iBufferId_p)
{
    int             iBufferId;
    void*           pMem;

    iBufferKey += 4080;     // generate an individual ID for the private mem
    iBufferId = shmget(IPC_PRIVATE, ulMemSize_p, 0666|IPC_CREAT);

    if (iBufferId == -1)
    {
        pMem = NULL;
        goto Exit;
    }

    pMem = shmat(iBufferId, NULL, 0);
    if (pMem == (void *)-1)
    {
        pMem = NULL;
        goto Exit;
    }

Exit:
    *iBufferId_p = iBufferId;
    return (pMem);
}

//------------------------------------------------------------------------------
/**
\brief Relaese shared buffer private memory

This function releases the private memory block of the shared buffer.

\param pShbMemInst_p        Pointer to shared buffer memory instance.
*/
//------------------------------------------------------------------------------
static void ShbIpcReleasePrivateMem (tShbMemInst *pShbMemInst_p)
{
    int     iBufferId;

    EPL_DBGLVL_SHB_TRACE("%s() %08x\n", __func__, pShbMemInst_p->m_iSharedMemId);
    iBufferId = pShbMemInst_p->m_iSharedMemId;
    shmdt(pShbMemInst_p);
    shmctl(iBufferId, IPC_RMID, 0);
    return;
}

//------------------------------------------------------------------------------
/**
\brief Main function for data signaling thread

This function implements the main function for the data signaling thread.

\param pvThreadParam_p      Thread parameters.

\return Returns always NULL.
*/
//------------------------------------------------------------------------------
static void *ShbIpcThreadSignalNewData (void *pvThreadParam_p)
{
    tShbInstance        pShbInstance;
    tShbMemInst*        pShbMemInst;
    int                 iRetVal = -1;
    struct timespec     curTime, timeout;

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);

    EPL_DBGLVL_SHB_TRACE ("Starting signaling thread for ID %08x\n",
                          pShbMemInst->m_iBufferKey);

    /* signal that thread is running */
    pShbMemInst->m_fNewDataThreadStarted = TRUE;
    do
    {
        clock_gettime(CLOCK_REALTIME, &curTime);
        timeout.tv_sec = 0;
        timeout.tv_nsec = TIMEOUT_CANCEL_THREAD * 1000;
        timespecadd(&timeout, &curTime);

        if ((iRetVal = sem_timedwait(pShbMemInst->m_semNewData, &timeout)) == 0)
        {
            if (!pShbMemInst->m_fThreadTermFlag)
            {
                EPL_DBGLVL_SHB_TRACE ("%s() signalhandler:%08x\n", __func__,
                                      pShbMemInst->m_pfnSigHndlrNewData);
                //call Rx Handler
                EPL_DBGLVL_SHB_TRACE("%s() ShbIpcThreadSignalNewData call Rx "
                                     "Handler (%s) sem:%08x instance:%08x\n",
                                     __func__, pShbMemInst->m_bufName,
                                     (UINT)iSemNewDataId, (UINT)pShbInstance);
                pShbMemInst->m_pfnSigHndlrNewData(pShbInstance);
            }
        }
    } while(!pShbMemInst->m_fThreadTermFlag);

    //set sem thread terminated
    EPL_DBGLVL_SHB_TRACE ("Stopping thread for ID %08x\n", pShbMemInst->m_iBufferKey);
    pShbMemInst->m_fNewDataThreadStarted = FALSE;
    sem_post(pShbMemInst->m_semStopSignalingNewData);

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief Main function for job ready signaling thread

This function implements the main function for the job ready signaling thread.

\param pvThreadParam_p      Thread parameters.

\return Returns always NULL.
*/
//------------------------------------------------------------------------------
void *ShbIpcThreadSignalJobReady (void *pvThreadParam_p)
{
    tShbInstance        pShbInstance;
    tShbMemInst         *pShbMemInst;
    DWORD               ulTimeOut;
    int                 iRetVal = -1;
    BOOL                fTimeOut = FALSE;
    struct timespec     timeout;

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);

    pShbMemInst->m_fJobReadyThreadStarted = TRUE;

    if (pShbMemInst->m_ulTimeOutMsJobReady != 0)
    {
        ulTimeOut = pShbMemInst->m_ulTimeOutMsJobReady;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += ulTimeOut;
        iRetVal = sem_timedwait(pShbMemInst->m_semJobReady, &timeout);
    }
    else
    {
        iRetVal = sem_wait(pShbMemInst->m_semJobReady);
    }

    if (iRetVal == 0)
        fTimeOut = FALSE;
    else
        fTimeOut = TRUE;                /* timeout or error */

    if (pShbMemInst->m_pfnSigHndlrJobReady != NULL)
    {
        pShbMemInst->m_pfnSigHndlrJobReady(pShbInstance, fTimeOut);
    }

    pShbMemInst->m_pfnSigHndlrJobReady = NULL;
    pShbMemInst->m_fJobReadyThreadStarted = FALSE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief Generate CRC32 generation table

This function generates a CRC32 generation table.
*/
//------------------------------------------------------------------------------
void ShbIpcCrc32GenTable(void)
{
    UINT        uiCrc, uiPoly;
    int         iIndexI, iIndexJ;

    uiPoly = 0xEDB88320L;
    for (iIndexI = 0; iIndexI < 256; iIndexI++)
    {
        uiCrc = iIndexI;
        for (iIndexJ = 8; iIndexJ > 0; iIndexJ--)
        {
            if (uiCrc & 1)
                uiCrc = (uiCrc >> 1) ^ uiPoly;
            else
                uiCrc >>= 1;
        }
        aulCrcTable_g[iIndexI] = uiCrc;
    }
}

//------------------------------------------------------------------------------
/**
\brief Create CRC32 checksum

This function generates a CRC32 checksum over the specified string.

\param  pcString        String for checksum calculation

\return Returns the CRC32 checksum
*/
//------------------------------------------------------------------------------
ULONG ShbIpcCrc32GetCrc(const char *pcString)
{
    UINT      uiCrc;
    UINT      uiIndex;

    uiCrc = 0xFFFFFFFF;
    for (uiIndex = 0; uiIndex < strlen(pcString); uiIndex++)
    {
        uiCrc = ((uiCrc >> 8) & 0x00FFFFFF) ^ aulCrcTable_g[(uiCrc ^ pcString[uiIndex]) & 0xFF];
    }
    return( uiCrc ^ 0xFFFFFFFF );
}

//------------------------------------------------------------------------------
/**
\brief Add two timespec values

This function adds a second timespec value to the first one.

\param  time1_p         First timespec value.
\param  time2_p         Second timespec value to be added to the first one.

\return Returns the CRC32 checksum
*/
//------------------------------------------------------------------------------
static void timespecadd(struct timespec *time1_p, struct timespec *time2_p)
{
    time1_p->tv_sec += time2_p->tv_sec;
    time1_p->tv_nsec += time2_p->tv_nsec;
    if (time1_p->tv_nsec >= 1000000000)
    {
        time1_p->tv_sec++;
        time1_p->tv_nsec -= 1000000000;
    }
}

///\}
