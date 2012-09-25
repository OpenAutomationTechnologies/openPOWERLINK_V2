/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  Implementation of platform specific part for the
                shared buffer
                (Implementation for Linux Userspace)

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

  2006/06/28 -rs:   V 1.00 (initaial version)

****************************************************************************/

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






/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

#define MAX_LEN_BUFFER_ID               256

#define IDX_EVENT_NEW_DATA              0
#define IDX_EVENT_TERM_REQU             1
#define IDX_EVENT_TERM_RESP             2

#define TIMEOUT_ENTER_ATOMIC            1000        // (ms) for debgging: INFINITE
#define TIMEOUT_ENTER_ATOMIC_KERNEL     1000
#define TIMEOUT_TERM_THREAD             2
#define TIMEOUT_TERM_THREAD_KERNEL      5
#define INFINITE                        3600

#define SBI_MAGIC_ID                    0x5342492B  // magic ID ("SBI+")
#define SBH_MAGIC_ID                    0x5342482A  // magic ID ("SBH*")

#define INVALID_ID                      -1
#define MEM_DEV_NAME                    "/dev/ShbMemDev0"
#define SHARED_BUF1                     "GlobalMemsharedbuf29"
//#define SHARED_BUF1                     "sharedbuf29"

#define IOCTL_BUFFER_ALLREADY_CREATED   0
#define IOCTL_GET_MEM_POINTER           1
#define IOCTL_SET_MEM_POINTER           2
#define IOCTL_CREATE_SEMS               3
#define IOCTL_SET_SEM                   4
#define IOCTL_SET_BUFFER_ID             5
#define IOCTL_GET_SEM                   6
#define IOCTL_PUT_SEM                   7
#define IOCTL_GET_SEMS                  8
#define IOCTL_GET_SEM_TIMEOUT           9
#define IOCTL_FREE_MEM                  10

#define DEBUG_IOCTL_ALOC_MEM            100

//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------

struct sMappedBuffers
{
    int                     m_iBufferId;
    struct sMappedBuffers   *m_psNextElement;
};

struct sMappedBuffers *psMappedBuffersElementFirst_g=NULL;

//This structure is for the communication between the usperspace and
//the kernelspace modul
typedef struct
{
    int                 m_iBufferId;
    void                *m_pBuffer;
    void                *m_pSemBuffAccess;
    void                *m_pSemNewData;
    void                *m_pSemJobReady;
    void                *m_pSemStopSignalingNewData;
    void                *m_pSemGeneric;
    int                 m_iSemValue;
    unsigned long       m_ulTimeOut;
} tShbMemKernelUser;


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
    int                 m_iUserSpaceMem;           //0 for userspace mem   !=0 kernelspace mem

    #ifndef NDEBUG
        unsigned long   m_ulOwnerProcID;
    #endif

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
    int                 m_iSemBuffAccessId;
    int                 m_iSemNewDataId;
    int                 m_iSemStopSignalingNewDataId;
    int                 m_iSemJobReadyId;
    int                 m_iThreadTermFlag;
    int                 m_iUserSpaceMem;           //0 for userspace mem   !=0 kernelspace mem
    pthread_t           m_tThreadNewDataId;
    pthread_t           m_tThreadJobReadyId;
    tSigHndlrNewData    m_pfnSigHndlrNewData;
    unsigned long       m_ulTimeOutMsJobReady;
    tSigHndlrJobReady   m_pfnSigHndlrJobReady;
    tShbMemHeader*      m_pShbMemHeader;
    tShbMemKernelUser*  m_pShbMemKernelUser;
    void                *m_pSemBuffAccess;
    void                *m_pSemNewData;
    void                *m_pSemJobReady;
    void                *m_pSemStopSignalingNewData;


    #ifndef NDEBUG
        unsigned long   m_ulThreadIDNewData;
        unsigned long   m_ulThreadIDJobReady;
    #endif

} tShbMemInst;




//---------------------------------------------------------------------------
//  Prototypes of internal functions
//---------------------------------------------------------------------------

tShbMemInst*    ShbIpcGetShbMemInst        (tShbInstance pShbInstance_p);
tShbMemHeader*  ShbIpcGetShbMemHeader      (tShbInstance pShbInstance_p);
void*           ShbIpcAllocPrivateMem      (unsigned long ulMemSize_p,int iBufferKey,int *iBufferId_p);
void            ShbIpcReleasePrivateMem    (int uiBufferId);
const char*     ShbIpcGetUniformObjectName (const char* pszEventJobName_p, const char* pszBufferID_p, BOOL fGlobalObject_p);
void            *ShbIpcThreadSignalNewData  (void* pvThreadParam_p);
void  		    *ShbIpcThreadSignalJobReady (void* pvThreadParam_p);
unsigned char   ShbIpcMakeHex(unsigned char uca,unsigned char ucb);
void            ShbIpcAppendListElementMappedBuffers(struct sMappedBuffers *psNewMappedBuffersElement);
int             ShbIpcFindListElementMappedBuffers (int iBufferId);


void ShbIpcCrc32GenTable(unsigned long aulCrcTable[]);
unsigned long ShbIpcCrc32GetCrc(char *pcString,unsigned long aulCrcTable[]);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Initialize IPC for Shared Buffer Module
//---------------------------------------------------------------------------

tShbError  ShbIpcInit (void)
{

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
int                     iBufferId;
int                     iIndex;
unsigned long           ulCrc32=0;
int                     iBufferKey;
int                     iSemBuffAccessId;
int                     iSemNewDataId;
int                     iSemStopSignalingNewDataId;
int                     iSemJobReadyId;
int                     iSemBuffAccessKey;
int                     iSemNewDataKey;
int                     iSemStopSignalingNewDataKey;
int                     iSemJobReadyKey;
unsigned int            uiFirstProcess=0;
int                     iBufferId_p=0;
int                     iUserSpaceMem;
int                     iFdMemDevice;
int                     iRetValue;
void                    *pBufferKey=&iBufferKey;
struct sMappedBuffers   *psNewMappedBuffersElement;
unsigned long           ulShMemSize;
tShbMemHeader*          pShbMemHeader;
tShbMemInst*            pShbMemInst;
tShbInstance            pShbInstance;
unsigned int            fShMemNewCreated=FALSE;
void                    *pSharedMem=NULL;
unsigned long           aulCrcTable[256] ;
tShbMemKernelUser       *pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));

    ulShMemSize      = ulBufferSize_p + sizeof(tShbMemHeader);
    TRACE("ulShMemSize  %u\n",ulShMemSize);


    //check if user- or kernelspace mem
    if (strstr(pszBufferID_p,"GlobalMem")==NULL)
    {
        //userspace mem
        iUserSpaceMem=0;
    }
    else
    {
        //kernspace mem
        iUserSpaceMem=1;
    }

    //create Buffer Key
    ShbIpcCrc32GenTable(aulCrcTable);
    ulCrc32=ShbIpcCrc32GetCrc(pszBufferID_p,aulCrcTable);
    iBufferKey=(int)ulCrc32;
    TRACE("ShbIpcAllocBuffer BufferSize:%d sizeof(tShb..):%d\n",ulBufferSize_p,sizeof(tShbMemHeader));
    TRACE("ShbIpcAllocBuffer BufferId:%d MemSize:%d\n",iBufferKey,ulShMemSize);

    if (iUserSpaceMem==0)
    {
        TRACE("UserSpaceMem\n");
        //UserSpaceMem


        //---------------------------------------------------------------
        // (1) open an existing or create a new shared memory
        //---------------------------------------------------------------
        // try to create an shared memory

        iBufferId=shmget(iBufferKey,ulShMemSize,0666|IPC_CREAT|IPC_EXCL);
        TRACE("iBufferId:%d iBufferKey:%d\n",iBufferId,iBufferKey);
        if (iBufferId==-1)
        {
            // a shared memory already exis, get buffer id
            iBufferId=shmget(iBufferKey,ulShMemSize,0666);
            uiFirstProcess=1;
            if (iBufferId==-1)
            {
                //unable to create mem
                ShbError = kShbOutOfMem;
                goto ExitUserSpaceMem;
            }
            fShMemNewCreated=FALSE;
        }
        else
        {
            //shared memory is new created
            fShMemNewCreated=TRUE;
        }
        //attach shared buf
        pSharedMem=shmat(iBufferId,0,0);
        if (pSharedMem==-1)
        {
            //unable to create mem
            ShbError = kShbOutOfMem;
            goto ExitUserSpaceMem;
        }
        //update header
        pShbMemHeader = (tShbMemHeader*)pSharedMem;
        // allocate a memory block from process specific mempool to save
        // process local information to administrate/manage the shared buffer
        pShbMemInst = (tShbMemInst*) ShbIpcAllocPrivateMem (sizeof(tShbMemInst),iBufferKey,&iBufferId_p);
        if (pShbMemInst == NULL)
        {
            ShbError = kShbOutOfMem;
            goto ExitUserSpaceMem;
        }

        // reset complete header to default values
        pShbMemInst->m_iSharedMemId                             = iBufferId;
        pShbMemInst->m_iSemBuffAccessId                         = INVALID_ID;
        pShbMemInst->m_iSemNewDataId                            = INVALID_ID;
        pShbMemInst->m_iSemJobReadyId                           = INVALID_ID;
        pShbMemInst->m_iSemStopSignalingNewDataId               = INVALID_ID;
        pShbMemInst->m_tThreadNewDataId                         = INVALID_ID;
        pShbMemInst->m_tThreadJobReadyId                        = INVALID_ID;
        pShbMemInst->m_pfnSigHndlrNewData                       = NULL;
        pShbMemInst->m_ulTimeOutMsJobReady                        = 0;
        pShbMemInst->m_pfnSigHndlrJobReady                      = NULL;
        pShbMemInst->m_pShbMemHeader                            = pShbMemHeader;
        pShbMemInst->m_iUserSpaceMem                            = 0;
        pShbMemInst->m_iThreadTermFlag                          = 0;


        //create semaphore for buffer access and signal new data
        //set sem keys
        iSemBuffAccessKey=iBufferKey+4081;
        iSemNewDataKey=iBufferKey+4082;
        iSemStopSignalingNewDataKey=iBufferKey+4083;
        iSemJobReadyKey=iBufferKey+4084;
        //get semaphores
        iSemBuffAccessId=semget(iSemBuffAccessKey,1,IPC_CREAT|0666);
        iSemNewDataId=semget(iSemNewDataKey,1,IPC_CREAT|0666);
        iSemStopSignalingNewDataId=semget(iSemStopSignalingNewDataKey,1,IPC_CREAT|0666);
        iSemJobReadyId=semget(iSemJobReadyKey,1,IPC_CREAT|0666);
        //if first process init semaphores
        if (uiFirstProcess==0)
        {
            semctl(iSemBuffAccessId,0,SETVAL,1);
            semctl(iSemNewDataId,0,SETVAL,0);
            semctl(iSemStopSignalingNewDataId,0,SETVAL,0);
            semctl(iSemJobReadyId,0,SETVAL,0);
        }
        //save semaphores in mem inst
        pShbMemInst->m_iSemBuffAccessId=iSemBuffAccessId;
        pShbMemInst->m_iSemNewDataId=iSemNewDataId;
        pShbMemInst->m_iSemStopSignalingNewDataId=iSemStopSignalingNewDataId;
        pShbMemInst->m_iSemJobReadyId=iSemJobReadyId;
        pShbMemInst->m_iUserSpaceMem=0;
        ShbError         = kShbOk;

        if ( fShMemNewCreated )
        {
            // this process was the first who wanted to use the shared memory,
            // so a new shared memory was created
            // -> setup new header information inside the shared memory region
            //    itself
            pShbMemHeader->m_ulShMemSize = ulShMemSize;
            pShbMemHeader->m_ulRefCount  = 1;
            pShbMemHeader->m_iBufferId=iBufferId_p;
        }
        else
        {
            // any other process has created the shared memory and this
            // process has only attached to it
            // -> check and update existing header information inside the
            //    shared memory region itself
            TRACE("MEM %d %d \n",pShbMemHeader->m_ulShMemSize, ulShMemSize);
            if (pShbMemHeader->m_ulShMemSize != ulShMemSize)
            {
                ShbError = kShbOpenMismatch;
                goto ExitUserSpaceMem;
            }
            pShbMemHeader->m_ulRefCount++;
        }

        ExitUserSpaceMem:
        pShbInstance = (tShbInstance*)pShbMemInst;
        *pfShbNewCreated_p = fShMemNewCreated;
        *ppShbInstance_p   = pShbInstance;

        return (ShbError);
    }//end UserSpaceMem
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpaceMem
        TRACE("KernelSpaceMem\n");
        //---------------------------------------------------------------
        // (1) open an existing or create a new shared memory
        //---------------------------------------------------------------
        // try to create an shared memory

        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("ShbIpcAllocBuffer open error\n");
            perror("ShbIpcAllocBuffer\n");
            ShbError = kShbUnableOpenMemDevice;
            goto ExitKernelSpaceMem;
        }
        //check if buffer allready exist
        pShbMemKernelUser->m_iBufferId=iBufferKey;
        iRetValue=ioctl(iFdMemDevice,IOCTL_BUFFER_ALLREADY_CREATED,pShbMemKernelUser);
        if (iRetValue==0)
        {
            // a shared memory already exist
            TRACE("ShbIpcAllocBuffer a shared memory already exist\n");
            pShbMemKernelUser->m_iBufferId=iBufferKey;
            //check if buffer also allready mapped
            if (ShbIpcFindListElementMappedBuffers(iBufferKey)==0)
            {
                //Buffer is allready mapped
                //get pointer to the shared mem
                TRACE("ShbIpcAllocBuffer Buffer is allready mapped\n");
                pShbMemKernelUser->m_iBufferId=iBufferKey;
                ioctl(iFdMemDevice,IOCTL_GET_MEM_POINTER,pShbMemKernelUser);
                pSharedMem=pShbMemKernelUser->m_pBuffer;
                uiFirstProcess=1;
            }
            else
            {
                //Buffer has to map
                TRACE("ShbIpcAllocBuffer Buffer has to map\n");
                //map to the shared mem
                //pSharedMem=mmap(0,ulShMemSize, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED,iFdMemDevice,(64*1024));
                pSharedMem=mmap(0,ulShMemSize, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED,iFdMemDevice,0);
                TRACE("ShbIpcAllocBuffer mmaped at: %p \n",pSharedMem);
                psNewMappedBuffersElement=malloc(sizeof(struct sMappedBuffers));
                psNewMappedBuffersElement->m_iBufferId=iBufferKey;
                //Append buffer element to list
                ShbIpcAppendListElementMappedBuffers(psNewMappedBuffersElement);
                TRACE("ShbIpcAllocBuffer Buffer appended to list\n");
                uiFirstProcess=2;
            }

            if (iBufferId==-1)
            {
                //unable to create mem
                ShbError = kShbOutOfMem;
                goto ExitKernelSpaceMem;
            }
            fShMemNewCreated=FALSE;
        }

        if (iRetValue==1)
        {
            TRACE("ShbIpcAllocBuffer a shared memory is new created\n");
            //shared memory is new created
            uiFirstProcess=0;
            fShMemNewCreated=TRUE;
            pShbMemKernelUser->m_iBufferId=iBufferKey;
            //map to the shared mem
            pSharedMem=mmap(0,ulShMemSize, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED,iFdMemDevice,0);
            if (pSharedMem==-1)
            {
                //unable to set map
                TRACE("ShbIpcAllocBuffer mmap error\n");
                perror("ShbIpcAllocBuffer mmap\n");
                ShbError = kShbOutOfMem;
                goto ExitKernelSpaceMem;
            }
            TRACE("ShbIpcAllocBuffer mmaped at: %p \n",pSharedMem);
            //Append buffer element to list
            psNewMappedBuffersElement=malloc(sizeof(struct sMappedBuffers));
            psNewMappedBuffersElement->m_iBufferId=iBufferKey;
            ShbIpcAppendListElementMappedBuffers(psNewMappedBuffersElement);
            pShbMemKernelUser->m_pBuffer=pSharedMem;
        }

        if (pSharedMem==NULL)
        {
            //unable to create mem
            TRACE("ShbIpcAllocBuffer unable to create mem\n");
            ShbError = kShbOutOfMem;
            goto ExitKernelSpaceMem;
        }
        //update header
        pShbMemHeader = (tShbMemHeader*)pSharedMem;
        TRACE("ShbIpcAllocBuffer Mem Size: %d %d \n",pShbMemHeader->m_ulShMemSize, ulShMemSize);
        // allocate a memory block from process specific mempool to save
        // process local information to administrate/manage the shared buffer
        pShbMemInst = (tShbMemInst*) ShbIpcAllocPrivateMem (sizeof(tShbMemInst),iBufferKey,&iBufferId_p);
        if (pShbMemInst == NULL)
        {
            ShbError = kShbOutOfMem;
            goto ExitKernelSpaceMem;
        }

        // reset complete header to default values
        pShbMemInst->m_iSharedMemId                             = iBufferKey;
        pShbMemInst->m_tThreadNewDataId                         = INVALID_ID;
        pShbMemInst->m_tThreadJobReadyId                        = INVALID_ID;
        pShbMemInst->m_pfnSigHndlrNewData                       = NULL;
        pShbMemInst->m_ulTimeOutMsJobReady                        = 0;
        pShbMemInst->m_pfnSigHndlrJobReady                      = NULL;
        pShbMemInst->m_pShbMemHeader                            = pShbMemHeader;
        pShbMemInst->m_iUserSpaceMem                            = 0;
        pShbMemInst->m_pShbMemKernelUser                        = pShbMemKernelUser;
        pShbMemInst->m_pSemBuffAccess                           = NULL;
        pShbMemInst->m_pSemNewData                              = NULL;
        pShbMemInst->m_pSemJobReady                             = NULL;
        pShbMemInst->m_pSemStopSignalingNewData                 = NULL;
        pShbMemInst->m_iThreadTermFlag                          = 0;

        if (uiFirstProcess==0)
        {
            //first process created mem
            //create semaphores
            ioctl(iFdMemDevice,IOCTL_CREATE_SEMS,pShbMemKernelUser);
            //set semaphores
            pShbMemKernelUser->m_iSemValue=1;
            pShbMemKernelUser->m_pSemGeneric=pShbMemKernelUser->m_pSemBuffAccess;
            ioctl(iFdMemDevice,IOCTL_SET_SEM,pShbMemKernelUser);
            pShbMemKernelUser->m_iSemValue=0;
            pShbMemKernelUser->m_pSemGeneric=pShbMemKernelUser->m_pSemNewData;
            ioctl(iFdMemDevice,IOCTL_SET_SEM,pShbMemKernelUser);
            pShbMemKernelUser->m_iSemValue=0;
            pShbMemKernelUser->m_pSemGeneric=pShbMemKernelUser->m_pSemStopSignalingNewData;
            ioctl(iFdMemDevice,IOCTL_SET_SEM,pShbMemKernelUser);
            pShbMemKernelUser->m_iSemValue=0;
            pShbMemKernelUser->m_pSemGeneric=pShbMemKernelUser->m_pSemJobReady;
            ioctl(iFdMemDevice,IOCTL_SET_SEM,pShbMemKernelUser);
        }

        if (uiFirstProcess==2)
        {
            //first process in instance mapped mem
        }

        //save semaphores to instance
        ioctl(iFdMemDevice,IOCTL_GET_SEMS,pShbMemKernelUser);
        pShbMemInst->m_pSemBuffAccess                           = pShbMemKernelUser->m_pSemBuffAccess;
        pShbMemInst->m_pSemNewData                              = pShbMemKernelUser->m_pSemNewData;
        pShbMemInst->m_pSemJobReady                             = pShbMemKernelUser->m_pSemJobReady;
        pShbMemInst->m_pSemStopSignalingNewData                 = pShbMemKernelUser->m_pSemStopSignalingNewData;
        TRACE("SEMBUFFACCESS %p\n",pShbMemInst->m_pSemBuffAccess);
        pShbMemInst->m_iUserSpaceMem=1;
        ShbError         = kShbOk;

        if ( fShMemNewCreated )
        {
            // this process was the first who wanted to use the shared memory,
            // so a new shared memory was created
            // -> setup new header information inside the shared memory region
            //    itself
            pShbMemHeader->m_ulShMemSize = ulShMemSize;
            pShbMemHeader->m_ulRefCount  = 1;
            pShbMemHeader->m_iBufferId=iBufferId_p;
            TRACE("ShbIpcAllocBuffer Mem Size New Created: %d %d \n",pShbMemHeader->m_ulShMemSize, ulShMemSize);
        }
        else
        {
            // any other process has created the shared memory and this
            // process has only attached to it
            // -> check and update existing header information inside the
            //    shared memory region itself
            TRACE("ShbIpcAllocBuffer MEM %d %d \n",pShbMemHeader->m_ulShMemSize, ulShMemSize);
            if (pShbMemHeader->m_ulShMemSize < ulShMemSize)
            {
                ShbError = kShbOpenMismatch;
                goto ExitUserSpaceMem;
            }
            pShbMemHeader->m_ulRefCount++;
        }
        ExitKernelSpaceMem:
        pShbInstance = (tShbInstance*)pShbMemInst;
        *pfShbNewCreated_p = fShMemNewCreated;
        *ppShbInstance_p   = pShbInstance;
        close(iFdMemDevice);
        TRACE("ShbIpcAllocBuffer exit\n");
        return (ShbError);
    }//end KernelSpaceMem
}



//---------------------------------------------------------------------------
//  Release Shared Buffer
//---------------------------------------------------------------------------

tShbError  ShbIpcReleaseBuffer (tShbInstance pShbInstance_p)
{
tShbMemInst*        pShbMemInst;
tShbMemHeader*      pShbMemHeader;
tShbError           ShbError;
tShbError           ShbError2;
int                 iSemId=INVALID_ID;
int                 iBufferId=INVALID_ID;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;

    TRACE("ShbIpcReleaseBuffer \n");
    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    if ( !--pShbMemHeader->m_ulRefCount )
    {
        ShbError = kShbOk;

    }
    else
    {
        ShbError = kShbMemUsedByOtherProcs;
    }

    if(pShbMemHeader->m_ulRefCount==0)
    {
        TRACE("----->ShbIpcReleaseBuffer ShbIpcStopSignalingNewData\n");
        //ShbError2 = ShbIpcStopSignalingNewData (pShbInstance_p);

    }
    ShbError2 = ShbIpcStopSignalingNewData (pShbInstance_p);
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem

        iBufferId=pShbMemInst->m_iSharedMemId;
        //detach shared buf
        shmdt(iBufferId);

        if(pShbMemHeader->m_ulRefCount==0)
        {
            TRACE("ShbIpcReleaseBuffer destroy shared mem\n");
            //destroy Buffer
            shmctl(iBufferId,IPC_RMID,0);
            //delete semaphores
            iSemId=pShbMemInst->m_iSemBuffAccessId;
            iRetVal=semctl(iSemId,SETVAL,IPC_RMID);
            iSemId=pShbMemInst->m_iSemNewDataId;
            iRetVal=semctl(iSemId,SETVAL,IPC_RMID);
            iSemId=pShbMemInst->m_iSemStopSignalingNewDataId;
            iRetVal=semctl(iSemId,SETVAL,IPC_RMID);
            iSemId=pShbMemInst->m_iSemJobReadyId;
            iRetVal=semctl(iSemId,SETVAL,IPC_RMID);
        }
        //delete privat mem
        ShbIpcReleasePrivateMem (iBufferId+4800);
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("ShbIpcReleaseBuffer open error\n");
            perror("ShbIpcReleaseBuffer\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }
        //ShbError2 = ShbIpcStopSignalingNewData (pShbInstance_p);

        if (pShbMemHeader->m_ulRefCount==0)
        {
            //free shared mem
            pShbMemKernelUser->m_iBufferId=pShbMemInst->m_iSharedMemId;
            iRetVal=ioctl(iFdMemDevice,IOCTL_FREE_MEM,pShbMemKernelUser);
        }
        //delete privat mem
        ShbIpcReleasePrivateMem (iBufferId+4800);
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return (ShbError);
}



//---------------------------------------------------------------------------
//  Process function (only used for implementations without threads)
//---------------------------------------------------------------------------

tShbError  ShbIpcProcess (void)
{
tShbError       ShbError = kShbOk;

    return ShbError;
}


//---------------------------------------------------------------------------
//  Enter atomic section for Shared Buffer access
//---------------------------------------------------------------------------

tShbError  ShbIpcEnterAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbMemInst         *pShbMemInst;
int                 iSemBuffAccessId;
int                 iRetVal=-1;
int                 iTimeOut=0;
int                 iFdMemDevice;
tShbError           ShbError;
struct timespec     sDelay;
tShbMemKernelUser   *pShbMemKernelUser;
struct sembuf       sSemBuf[1];

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    iSemBuffAccessId= pShbMemInst->m_iSemBuffAccessId;

    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        sSemBuf[0].sem_num=0;
        sSemBuf[0].sem_op=-1;
        sSemBuf[0].sem_flg=IPC_NOWAIT;

        sDelay.tv_sec=0;
        sDelay.tv_nsec=1000000; //1 mili sec

        while ((iTimeOut<TIMEOUT_ENTER_ATOMIC)&&(iRetVal<0))
        {
            iRetVal=semop(iSemBuffAccessId,sSemBuf,1);
            if (iRetVal<0)
            {
                if (errno==EAGAIN)
                {
                    nanosleep(&sDelay,NULL);
                    iTimeOut++;
                    TRACE("---->nanosleep<----\n");
                }
            }
        }


        if (iRetVal==0)
        {
            TRACE("ShbIpcEnterAtomicSection Enter Atomic ok\n");
            ShbError = kShbOk;
        }
        else
        {
            TRACE("ShbIpcEnterAtomicSection Enter Atomic not ok %d\n",iRetVal);
            perror("perror: ");
            ShbError = kShbBufferInvalid;
        }

        TRACE("ShbIpcEnterAtomicSection Leave Atomic\n");
        return (kShbOk);
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("ShbIpcEnterAtomicSection: Enter Atomic Kernel\n");
        //open mem device
        iFdMemDevice=open(MEM_DEV_NAME, O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("ShbIpcEnterAtomicSection open error\n");
            perror("ShbIpcEnterAtomicSectionr\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }
        pShbMemKernelUser->m_ulTimeOut=TIMEOUT_ENTER_ATOMIC_KERNEL;
        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemBuffAccess;
        //try to get buff access semaphore
        iRetVal=ioctl(iFdMemDevice,IOCTL_GET_SEM,pShbMemKernelUser);

        if (iRetVal==0)
        {
            TRACE("ShbIpcEnterAtomicSection Got Sem Kernel\n");
            ShbError = kShbOk;
        }
        else
        {
            TRACE("ShbIpcEnterAtomicSection Enter Atomic not ok Kernel\n");
            ShbError = kShbBufferInvalid;
        }
        close(iFdMemDevice);
        free(pShbMemKernelUser);
        return ShbError;

    }
}



//---------------------------------------------------------------------------
//  Leave atomic section for Shared Buffer access
//---------------------------------------------------------------------------

tShbError  ShbIpcLeaveAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbMemInst*        pShbMemInst;
unsigned int        iSemBuffAccessId=INVALID_ID;
tShbError           ShbError;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;
struct sembuf       sSemBuf[1];

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        sSemBuf[0].sem_num=0;
        sSemBuf[0].sem_op=1;
        sSemBuf[0].sem_flg=SEM_UNDO;
        iSemBuffAccessId= pShbMemInst->m_iSemBuffAccessId;
        ShbError = kShbOk;
        TRACE("Leave Atomic User\n");
        if (iSemBuffAccessId != -1)
        {
            semctl(iSemBuffAccessId,0,SETVAL,1);
            //semop(iSemBuffAccessId,sSemBuf,1);
        }
        else
        {
            ShbError = kShbBufferInvalid;
        }
        return (ShbError);
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("ShbIpcEnterAtomicSection: Leave Atomic Kernel\n");
        //open mem device
        iFdMemDevice=open(MEM_DEV_NAME, O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("ShbIpcLeaveAtomicSection open error\n");
            perror("ShbIpcLeaveAtomicSectionr\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }

        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemBuffAccess;
        iRetVal=ioctl(iFdMemDevice,IOCTL_PUT_SEM,pShbMemKernelUser);

        if (iRetVal==0)
        {
            TRACE("ShbIpcLeaveAtomicSection Put Sem ok\n");
            ShbError = kShbOk;
        }
        else
        {
            TRACE("ShbIpcLeaveAtomicSection  not ok %d\n",iRetVal);
            ShbError = kShbBufferInvalid;
        }
        close(iFdMemDevice);
        free(pShbMemKernelUser);
        return (ShbError);
    }
}



//---------------------------------------------------------------------------
//  Start signaling of new data (called from reading process)
//---------------------------------------------------------------------------

tShbError  ShbIpcStartSignalingNewData (
    tShbInstance pShbInstance_p,
    tSigHndlrNewData pfnSignalHandlerNewData_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;


    if ((pShbInstance_p == NULL) || (pfnSignalHandlerNewData_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    ShbError = kShbOk;

    if ((pShbMemInst->m_tThreadNewDataId != -1)||(pShbMemInst->m_pfnSigHndlrNewData!= NULL))
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }

    pShbMemInst->m_pfnSigHndlrNewData = pfnSignalHandlerNewData_p;

    //create thread for signalling new data
    pthread_create(&pShbMemInst->m_tThreadNewDataId,NULL,&ShbIpcThreadSignalNewData,pShbInstance_p);


    Exit:
    return (ShbError);
}



//---------------------------------------------------------------------------
//  Stop signaling of new data (called from reading process)
//---------------------------------------------------------------------------

tShbError  ShbIpcStopSignalingNewData (
    tShbInstance pShbInstance_p)
{
tShbMemInst         *pShbMemInst;
tShbMemHeader       *pShbMemHeader;
int                 iSemNewDataId;
int                 iTimeOut;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;
int                 iSemStopSignalingNewDataId;
struct sembuf       sSemBuf[1];
struct sembuf       sSemBufNewData[1];
struct timespec     sDelay;
tShbError           ShbError;

    if (pShbInstance_p == NULL)
    {
            return (kShbInvalidArg);
    }
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    ShbError = kShbOk;
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        sSemBuf[0].sem_num=0;
        sSemBuf[0].sem_op=-1;
        sSemBuf[0].sem_flg=IPC_NOWAIT;
        sSemBufNewData[0].sem_num=0;
        sSemBufNewData[0].sem_op=1;
        sSemBufNewData[0].sem_flg=SEM_UNDO;
        sDelay.tv_sec=0;
        sDelay.tv_nsec=1000000;
        TRACE("------->ShbIpcStopSignalingNewData\n");

        pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
        iSemNewDataId=pShbMemInst->m_iSemNewDataId;
        iSemStopSignalingNewDataId=pShbMemInst->m_iSemStopSignalingNewDataId;
        //set termination flag
        pShbMemInst->m_iThreadTermFlag =1;
        //set semaphore new data
        semctl(iSemNewDataId,0,SETVAL,1);
        //semop(iSemNewDataId,sSemBufNewData,1);
        //wait for thread terminated sem
        while ((iTimeOut<TIMEOUT_ENTER_ATOMIC)&&(iRetVal<0))
        {
            iRetVal=semop(iSemStopSignalingNewDataId,sSemBuf,1);
            if (iRetVal<0)
            {
                if (errno==EAGAIN)
                {
                    nanosleep(&sDelay,NULL);
                    iTimeOut++;
                    //TRACE("---->nanosleep<----\n");
                }
            }
        }

        if (iRetVal==0)
        {
            TRACE("Stop Sem recived\n");
        }
        else
        {
            TRACE("Stop Sem TIMEOUT %d\n",iRetVal);
            perror("perror: ");
        }
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));

        TRACE("Stop signaling of new data Kernel\n");
        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("Stop signaling of new data open error\n");
            perror("Stop signaling of new data\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }
        TRACE("------->ShbIpcStopSignalingNewData Kernel\n");
        //set termination flag
        pShbMemInst->m_iThreadTermFlag =1;
        //set semaphore new data
        pShbMemKernelUser->m_iSemValue=1;
        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemNewData;
        ioctl(iFdMemDevice,IOCTL_SET_SEM,pShbMemKernelUser);
        //wait for sem timeout
        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemStopSignalingNewData;
        pShbMemKernelUser->m_ulTimeOut=TIMEOUT_TERM_THREAD_KERNEL;
        iRetVal=ioctl(iFdMemDevice,IOCTL_GET_SEM_TIMEOUT,pShbMemKernelUser);
        if (iRetVal==0)
        {
            TRACE("Stop Sem recived\n");
        }
        else
        {
            TRACE("Stop Sem TIMEOUT %d\n",iRetVal);
        }
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return (ShbError);
}



//---------------------------------------------------------------------------
//  Signal new data (called from writing process)
//---------------------------------------------------------------------------

tShbError  ShbIpcSignalNewData (
    tShbInstance pShbInstance_p)
{
tShbMemInst*        pShbMemInst;
int                 iSemNewDataId=-1;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;
tShbError           ShbError;
struct sembuf       sSemBuf[1];

    sSemBuf[0].sem_num=0;
    sSemBuf[0].sem_op=1;
    sSemBuf[0].sem_flg=SEM_UNDO;
    TRACE("ShbIpcSignalNewData\n");
    if (pShbInstance_p == NULL)
    {
            return (kShbInvalidArg);
    }
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    ShbError = kShbOk;
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        iSemNewDataId=pShbMemInst->m_iSemNewDataId;

        if (iSemNewDataId!=-1)
        {
            //set semaphore
            TRACE("ShbIpcSignalNewData set Sem -> New Data 1 \n");
            semctl(iSemNewDataId,0,SETVAL,1);
            //semop(iSemNewDataId,sSemBuf,1);
        }
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("Signal new data Kernel\n");
        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("Signal new data open error\n");
            perror("Signal new data data\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }

        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemNewData;
        iRetVal=ioctl(iFdMemDevice,IOCTL_PUT_SEM,pShbMemKernelUser);
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return ShbError;
}



//---------------------------------------------------------------------------
//  Start signaling for job ready (called from waiting process)
//---------------------------------------------------------------------------

tShbError  ShbIpcStartSignalingJobReady (
    tShbInstance pShbInstance_p,
    unsigned long ulTimeOut_p,
    tSigHndlrJobReady pfnSignalHandlerJobReady_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerJobReady_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);
    ShbError = kShbOk;

    if ((pShbMemInst->m_tThreadJobReadyId != -1)||(pShbMemInst->m_pfnSigHndlrJobReady!= NULL))
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }

    pShbMemInst->m_ulTimeOutMsJobReady = ulTimeOut_p;
    pShbMemInst->m_pfnSigHndlrJobReady = pfnSignalHandlerJobReady_p;

    //create thread for signalling new data
    pthread_create(&pShbMemInst->m_tThreadJobReadyId,NULL,&ShbIpcThreadSignalJobReady,pShbInstance_p);
    Exit:
    return 0;
}



//---------------------------------------------------------------------------
//  Signal job ready (called from executing process)
//---------------------------------------------------------------------------

tShbError  ShbIpcSignalJobReady (
    tShbInstance pShbInstance_p)
{
tShbMemInst*        pShbMemInst;
int                 iSemJobReadyId=-1;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;
tShbError           ShbError;
struct sembuf       sSemBuf[1];

    sSemBuf[0].sem_num=0;
    sSemBuf[0].sem_op=1;
    sSemBuf[0].sem_flg=SEM_UNDO;
    TRACE("ShbIpcSignalJobReady\n");
    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    ShbError = kShbOk;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        iSemJobReadyId=pShbMemInst->m_iSemJobReadyId;

        if (iSemJobReadyId!=-1)
        {
            //set semaphore
            TRACE("ShbIpcSignalJobReady set Sem -> Job Ready  \n");
            semctl(iSemJobReadyId,0,SETVAL,1);
            //semop(iSemJobReadyId,sSemBuf,1);
        }
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("Signal new data Kernel\n");
        //open mem device
        iFdMemDevice=open(MEM_DEV_NAME, O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("Signal new data open error\n");
            perror("Signal new data data\n");
            ShbError = kShbUnableOpenMemDevice;
            return ShbError;
        }

        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemJobReady;
        iRetVal=ioctl(iFdMemDevice,IOCTL_PUT_SEM,pShbMemKernelUser);
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return ShbError;
}



//---------------------------------------------------------------------------
//  Get pointer to common used share memory area
//---------------------------------------------------------------------------

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
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Get pointer to process local information structure
//---------------------------------------------------------------------------

tShbMemInst*  ShbIpcGetShbMemInst (
    tShbInstance pShbInstance_p)
{
tShbMemInst*  pShbMemInst;

    pShbMemInst = (tShbMemInst*)pShbInstance_p;
    return (pShbMemInst);
}



//---------------------------------------------------------------------------
//  Get pointer to shared memory header
//---------------------------------------------------------------------------

tShbMemHeader*  ShbIpcGetShbMemHeader (
    tShbInstance pShbInstance_p)
{

tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;

    pShbMemInst   = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = pShbMemInst->m_pShbMemHeader;
    return (pShbMemHeader);
}



//---------------------------------------------------------------------------
//  Allocate a memory block from process specific mempool
//---------------------------------------------------------------------------

void*  ShbIpcAllocPrivateMem (unsigned long ulMemSize_p,int iBufferKey,int *iBufferId_p)
{
tShbError       ShbError;
int             iBufferId;
void*           pMem;

    //set new buffer id
    iBufferKey+=4080;
    //get private mem
    iBufferId=shmget(IPC_PRIVATE,ulMemSize_p,0666|IPC_CREAT);

    if (iBufferId==-1)
    {
        //unable to create mem
        ShbError = kShbOutOfMem;
        goto Exit;
    }
    //attach shared buf
    pMem=shmat(iBufferId,0,0);
    if (pMem==-1)
    {
        //unable to create mem
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    Exit:
    *iBufferId_p=iBufferId;
    return (pMem);
}



//---------------------------------------------------------------------------
//  Release a memory block from process specific mempool
//---------------------------------------------------------------------------

void  ShbIpcReleasePrivateMem (int iBufferId)
{
    //detach shared buf
    shmdt(iBufferId);
    //destroy Buffer
    shmctl(iBufferId,IPC_RMID,0);
    return;
}



//---------------------------------------------------------------------------
//  Thread for new data signaling
//---------------------------------------------------------------------------

void *ShbIpcThreadSignalNewData (void *pvThreadParam_p)
{
tShbInstance        pShbInstance;
tShbMemInst*        pShbMemInst;
tShbMemHeader*      pShbMemHeader;
int                 fTermRequ;
int                 iSemNewDataId;
int                 iSemStopSignalingNewDataId;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iRetVal=-1;
int                 iFdMemDevice;
struct sembuf       sSemBuf[1];
struct sembuf       sSemBufStopSignaling[1];

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance);
    //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        iSemNewDataId = pShbMemInst->m_iSemNewDataId;
        iSemStopSignalingNewDataId=pShbMemInst->m_iSemStopSignalingNewDataId;
        fTermRequ    = FALSE;
        sSemBuf[0].sem_num=0;
        sSemBuf[0].sem_op=-1;
        sSemBuf[0].sem_flg=SEM_UNDO;
        sSemBufStopSignaling[0].sem_num=0;
        sSemBufStopSignaling[0].sem_op=1;
        sSemBufStopSignaling[0].sem_flg=SEM_UNDO;

        do
        {
            TRACE("ShbIpcThreadSignalNewData wait for New Data Sem: \n");
            semctl(iSemNewDataId,0,GETVAL,0);
            //wait for new data semaphore
            semop(iSemNewDataId,sSemBuf,1);
            TRACE("ShbIpcThreadSignalNewData New Data Sem\n");
            //check terminate flag
            if (pShbMemInst->m_iThreadTermFlag==0)
            {
                //call Rx Handler
                TRACE("ShbIpcThreadSignalNewData call Rx Handler\n");
                pShbMemInst->m_pfnSigHndlrNewData(pShbInstance);
            }
            else
            {
                //terminate thread
                TRACE("Terminate ShbIpcThreadSignalNewData\n");

            }
        }while(pShbMemInst->m_iThreadTermFlag==0);
        //set sem thread terminated
        semctl(iSemStopSignalingNewDataId,0,SETVAL,1);
        //semop(iSemStopSignalingNewDataId,sSemBufStopSignaling,1);
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("Signal new data Kernel\n");
        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("Signal new data open error\n");
            perror("Signal new data data\n");
        }
        do
        {
            TRACE("ShbIpcThreadSignalNewData wait for New Data Sem Kernel\n");
            //wait for new data semaphore
            pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemNewData;
            pShbMemKernelUser->m_ulTimeOut=TIMEOUT_ENTER_ATOMIC_KERNEL;
            iRetVal=ioctl(iFdMemDevice,IOCTL_GET_SEM_TIMEOUT,pShbMemKernelUser);
            TRACE("ShbIpcThreadSignalNewData Got New Data Sem Kernel iRetVal: %d\n",iRetVal);
            //check terminate flag
            if (pShbMemInst->m_iThreadTermFlag==0)
            {
                if (iRetVal==0)
                {
                    //call Handler
                    pShbMemInst->m_pfnSigHndlrNewData(pShbInstance);
                }
            }
            else
            {
                //terminate thread
                TRACE("Terminate ShbIpcThreadSignalNewData Kernel\n");
            }
        }while(pShbMemInst->m_iThreadTermFlag==0);
        //set sem thread terminated
        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemStopSignalingNewData;
        iRetVal=ioctl(iFdMemDevice,IOCTL_PUT_SEM,pShbMemKernelUser);
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return NULL;
}



//---------------------------------------------------------------------------
//  Thread for new data Job Ready signaling
//---------------------------------------------------------------------------

void *ShbIpcThreadSignalJobReady (void *pvThreadParam_p)
{
tShbInstance        pShbInstance;
tShbMemInst         *pShbMemInst;
DWORD               ulTimeOut;
int                 iSemJobReadyId;
int                 iRetVal=-1;
int                 iTimeOut;
unsigned int        fTimeOut=FALSE;
tShbMemKernelUser   *pShbMemKernelUser;
int                 iFdMemDevice;
struct sembuf       sSemBuf[1];
struct timespec     sDelay;

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);
    iSemJobReadyId = pShbMemInst->m_iSemJobReadyId;
    ulTimeOut = pShbMemInst->m_ulTimeOutMsJobReady;

   //check if user or kernelspace mem
    if (pShbMemInst->m_iUserSpaceMem==0)
    {
        //UserSpace mem
        sDelay.tv_sec=ulTimeOut;
        sDelay.tv_nsec=0;
        sSemBuf[0].sem_num=0;
        sSemBuf[0].sem_op=-1;
        sSemBuf[0].sem_flg=IPC_NOWAIT;
        TRACE("ShbIpcThreadSignalJobReady wait for job ready Sem \n");
        if (pShbMemInst->m_ulTimeOutMsJobReady != 0)
        {
            ulTimeOut = pShbMemInst->m_ulTimeOutMsJobReady;
        }
        else
        {
            ulTimeOut = INFINITE;
        }

        //wait for job ready semaphore
        while ((iTimeOut<TIMEOUT_ENTER_ATOMIC)&&(iRetVal<0))
        {
            iRetVal=semop(iSemJobReadyId,sSemBuf,1);
            if (iRetVal<0)
            {
                if (errno==EAGAIN)
                {
                    nanosleep(&sDelay,NULL);
                    iTimeOut++;
                    //TRACE("---->nanosleep<----\n");
                }
            }
        }
        if (iRetVal==0)
        {
            fTimeOut=FALSE;
        }
        else
        {
            fTimeOut=TRUE;
        }

        if (pShbMemInst->m_pfnSigHndlrJobReady!=NULL)
        {
            //call Handler
            pShbMemInst->m_pfnSigHndlrJobReady(pShbInstance,fTimeOut);
        }
        pShbMemInst->m_pfnSigHndlrJobReady=NULL;
    }
//#######################################################################
// KernelUserSpaceBorder
//#######################################################################
    else
    {
        //KernelSpace Mem
        pShbMemKernelUser = malloc(sizeof(tShbMemKernelUser));
        TRACE("Signal new data Kernel\n");
        //open mem device
        iFdMemDevice=open("/dev/ShbMemDev0", O_RDWR);
        if (iFdMemDevice<0)
        {
            TRACE("Signal new data open error\n");
            perror("Signal new data data\n");
        }
        //wait for job ready semaphore
        pShbMemKernelUser->m_pSemGeneric=pShbMemInst->m_pSemJobReady;
        pShbMemKernelUser->m_ulTimeOut=TIMEOUT_TERM_THREAD_KERNEL;
        iRetVal=ioctl(iFdMemDevice,IOCTL_GET_SEM_TIMEOUT,pShbMemKernelUser);
        if (iRetVal==0)
        {
            fTimeOut=FALSE;
        }
        else
        {
            fTimeOut=TRUE;
        }

        if (pShbMemInst->m_pfnSigHndlrJobReady!=NULL)
        {
            //call Rx Handler
            pShbMemInst->m_pfnSigHndlrJobReady(pShbInstance,fTimeOut);
        }

        pShbMemInst->m_pfnSigHndlrJobReady=NULL;
        close(iFdMemDevice);
        free(pShbMemKernelUser);
    }
    return NULL;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


unsigned char ShbIpcMakeHex(unsigned char uca,unsigned char ucb)
{
    return uca*0x10+ucb;
}


//Build the crc table
void ShbIpcCrc32GenTable(unsigned long aulCrcTable[])
{
    unsigned long   ulCrc,ulPoly;
    int             iIndexI,iIndexJ;

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
unsigned long ShbIpcCrc32GetCrc(char *pcString,unsigned long aulCrcTable[])
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


void ShbIpcAppendListElementMappedBuffers(struct sMappedBuffers *psNewMappedBuffersElement)
{
    struct sMappedBuffers *psMappedBuffersElement=psMappedBuffersElementFirst_g;
    psNewMappedBuffersElement->m_psNextElement=NULL;

    if (psMappedBuffersElementFirst_g!= NULL )
    {      /* sind Elemente vorhanden */
       while (psMappedBuffersElement->m_psNextElement != NULL )
       {    /* suche das letzte Element */
           psMappedBuffersElement=psMappedBuffersElement->m_psNextElement;
       }
       psMappedBuffersElement->m_psNextElement=psNewMappedBuffersElement;              /*  Haenge das Element hinten an */
    }
    else
    {                           /* wenn die liste leer ist, bin ich das erste Element */
        psMappedBuffersElementFirst_g=psNewMappedBuffersElement;
    }
}


int ShbIpcFindListElementMappedBuffers (int iBufferId)
{
    struct sMappedBuffers *psMappedBuffersElement=psMappedBuffersElementFirst_g;
    while (psMappedBuffersElement!=NULL)
    {
        if(psMappedBuffersElement->m_iBufferId==iBufferId)
        {
             return 0;
        }
        psMappedBuffersElement=psMappedBuffersElement->m_psNextElement;
    }
    return -1;
}


 