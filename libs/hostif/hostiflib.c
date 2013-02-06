/**
********************************************************************************
\file   hostiflib.c

\brief  Host Interface Library - High Level Driver Header

The Host Interface Library High Level Driver provides a software library for the
host interface IP-Core.
The hostiflib provides several features like queues and linear memory modules.

\ingroup module_hostiflib
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include "hostiflib_l.h"
#include "hostiflib.h"
#include "hostiflib_target.h"

#include "lfqueue.h"
#include "linmem.h"

#include <stdlib.h>
#include <string.h>


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

#define HOSTIF_MAGIC              0x504C4B00  ///< Host Interface Magic

#define HOSTIF_INSTANCE_COUNT     2   ///< number of supported instances
#define HOSTIF_QUEUE_COUNT        16  ///< number of supported queues

/**
\brief Dynamic Buffer offsets for host side
*/
const UINT8 *apDynBuf[] =
{
    (UINT8*)HOSTIF_BASE_DYNBUF0,
    (UINT8*)HOSTIF_BASE_DYNBUF1,
    NULL
};

/**
\brief Linear memory configuration of TPDO

Stores the default configuration settings of the linear memory for Pcp and Host.
*/
const tLimConfig LimConfigTpdo =
{
    FALSE, (UINT8*)HOSTIF_BASE_TPDO, HOSTIF_SIZE_TPDO
};

/**
\brief Linear memory configuration of RPDO

Stores the default configuration settings of the linear memory for Pcp and Host.
*/
const tLimConfig LimConfigRpdo =
{
    FALSE, (UINT8*)HOSTIF_BASE_RPDO, HOSTIF_SIZE_RPDO
};

/**
\brief Linear memory configuration of Error Counters

Stores the default configuration settings of the linear memory for Pcp and Host.
*/
const tLimConfig LimConfigErrCount =
{
    FALSE, (UINT8*)HOSTIF_BASE_ERRORCOUNTER, HOSTIF_SIZE_ERRORCOUNTER
};

/**
\brief Queue configuration for Tx Nmt

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigTxNmt =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_TXNMTQ, HOSTIF_SIZE_TXNMTQ
};

/**
\brief Queue configuration for Tx Generic

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigTxGen =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_TXGENQ, HOSTIF_SIZE_TXGENQ
};

/**
\brief Queue configuration for Tx Sync

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigTxSync =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_TXSYNCQ, HOSTIF_SIZE_TXSYNCQ
};

/**
\brief Queue configuration for Tx Virtual Ethernet

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigTxVeth =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_TXVETHQ, HOSTIF_SIZE_TXVETHQ
};

/**
\brief Queue configuration for Rx Virtual Ethernet

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigRxVeth =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_RXVETHQ, HOSTIF_SIZE_RXVETHQ
};

/**
\brief Queue configuration for Kernel-to-User

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigK2U =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_K2UQ, HOSTIF_SIZE_K2UQ
};

/**
\brief Queue configuration for User-to-Kernel

Stores the default configuration settings for the queue instance.
*/
const tQueueConfig QueueConfigU2K =
{
    kQueueBoth, FALSE, (UINT8*)HOSTIF_BASE_U2KQ, HOSTIF_SIZE_U2KQ
};

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Function type to set the address of a dynamic buffer

This function type enables to set the corresponding dynamic buffer address
register. The address of the status/control registers is given with
pHostifScBase_p and the address itself with dwAddr_p.
*/
typedef void (*tSetDynBuf) (UINT8 *pHostifScBase_p, UINT32 addr_p);

/**
\brief Function type to get the address of a dynamic buffer

This function type enables to get the address set in the dynamic buffer address
register. The address of the statuc/control registers is given with
pHostifScBase_p.
*/
typedef UINT32 (*tGetDynBuf) (UINT8 *pHostifScBase_p);

/**
\brief Structure for host interface dynamic buffer

This structure defines for each dynamic buffer instance the set and get
functions. Additionally the base and span is provided.
*/
typedef struct sHostifDynBuf
{
    tSetDynBuf  pfnSetDynBuf; ///< this function sets the dynamic buffer base to hardware
    tGetDynBuf  pfnGetDynBuf; ///< this function gets the dynamic buffer base to hardware
    UINT8*      pBase;        ///< base of the dynamic buffer
    UINT16      span;        ///< span of the dynamic buffer
} tHostifDynBufConfig;

/**
\brief Dynamic buffer configuration for Pcp

Stores the default configuration settings for the dynamic buffers
*/
const tHostifDynBufConfig aDynBufInitPcp[kHostifInstIdLast] =
{
    {   /* kHostifInstIdErrCount */
        hostif_writeDynBufPcpErrCnt, hostif_readDynBufPcpErrCnt,
        (UINT8*)HOSTIF_BASE_ERRORCOUNTER, HOSTIF_SIZE_ERRORCOUNTER
    },
    {   /* kHostifInstIdTxNmtQueue */
        hostif_writeDynBufPcpTxNmtQ, hostif_readDynBufPcpTxNmtQ,
        (UINT8*)HOSTIF_BASE_TXNMTQ, HOSTIF_SIZE_TXNMTQ
    },
    {   /* kHostifInstIdTxGenQueue */
        hostif_writeDynBufPcpTxGenQ, hostif_readDynBufPcpTxGenQ,
        (UINT8*)HOSTIF_BASE_TXGENQ, HOSTIF_SIZE_TXGENQ
    },
    {   /* kHostifInstIdTxSyncQueue */
        hostif_writeDynBufPcpTxSyncQ, hostif_readDynBufPcpTxSyncQ,
        (UINT8*)HOSTIF_BASE_TXSYNCQ, HOSTIF_SIZE_TXSYNCQ
    },
    {   /* kHostifInstIdTxVethQueue */
        hostif_writeDynBufPcpTxVethQ, hostif_readDynBufPcpTxVethQ,
        (UINT8*)HOSTIF_BASE_TXVETHQ, HOSTIF_SIZE_TXVETHQ
    },
    {   /* kHostifInstIdRxVethQueue */
        hostif_writeDynBufPcpRxVethQ, hostif_readDynBufPcpRxVethQ,
        (UINT8*)HOSTIF_BASE_RXVETHQ, HOSTIF_SIZE_RXVETHQ
    },
    {   /* kHostifInstIdK2UQueue */
        hostif_writeDynBufPcpK2UQ, hostif_readDynBufPcpK2UQ,
        (UINT8*)HOSTIF_BASE_K2UQ, HOSTIF_SIZE_K2UQ
    },
    {   /* kHostifInstIdU2KQueue */
        hostif_writeDynBufPcpU2KQ, hostif_readDynBufPcpU2KQ,
        (UINT8*)HOSTIF_BASE_U2KQ, HOSTIF_SIZE_U2KQ
    },
    {   /* kHostifInstIdTpdo */
        hostif_writeDynBufPcpTpdo, hostif_readDynBufPcpTpdo,
        (UINT8*)HOSTIF_BASE_TPDO, HOSTIF_SIZE_TPDO
    },
    {   /* kHostifInstIdRpdo */
        hostif_writeDynBufPcpRpdo, hostif_readDynBufPcpRpdo,
        (UINT8*)HOSTIF_BASE_RPDO, HOSTIF_SIZE_RPDO
    }
};

/**
\brief Version field in Status/Control - Information

Used to obtain hardware/software mismatch
*/
typedef struct sHostifVersion
{
    volatile UINT8      cnt;        ///< Counting field
    volatile UINT8      revision;   ///< Revision field
    volatile UINT8      minor;      ///< Minor field
    volatile UINT8      major;      ///< Major field
} tHostifVersion;

/**
\brief Queue process information

This structure holds the queue instance and the corresponding callback and
argument for a certain queue resource.
*/
typedef struct sQueueProcess
{
    tHostifQueueInstance pInstance;    ///< the queue instance
    tQueueCb            pfnCallback;     ///< instance callback
    void                *pArg;        ///< instance argument

} tQueueProcess;

/**
\brief Host Interface Instance

Holds the configuration passed to the instance at creation.
*/
typedef struct sHostif
{
    tHostifConfig       config;       ///< copy of configuration
    UINT8               *pBase;       ///< base address of host interface

    int                 iDynBufEntries; ///< number of dynamic buffers (Pcp/Host)
    tHostifDynBufConfig* pDynBufTbl;  ///< dynamic buffer table (Pcp/Host)
    UINT8*              apDynBufHost[HOSTIF_DYNBUF_COUNT]; ///< DynBuf acquired by Host

    tQueueProcess       aQueueProcessTable[HOSTIF_QUEUE_COUNT]; ///< queue process table, processed by hostif_process()
    int                 iQueueProcessEntries; ///< number of entries in aQueueProcessTable

    tHostifIrqCb        apfnIrqCb[kHostifIrqSrcLast];

} tHostif;

/**
\brief Host Interface Queue Instance

Holds the Host interface and queue instance
*/
typedef struct sHostifQueue
{
    tHostif*            pHostif;      ///< reference to hostif instance
    tQueueInstance      pQueueInstance; ///< reference to queue instance
} tHostifQueue;

/**
\brief Host Interface Lim Instance

Holds the Host interface and lim instance
*/
typedef struct sHostifLim
{
    tHostif*            pHostif;      ///< reference to hostif instance
    tLimInstance        pLimInstance; ///< reference to lim instance
} tHostifLim;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/**
\brief Instance array

This array holds all Host Interface instances available.
*/
static tHostif *paHostifInstance[HOSTIF_INSTANCE_COUNT] =
{
    NULL
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void hostifIrqHandler (void *pArg_p);
static void freePtr(void *p);
static tHostifReturn checkMagic (tHostif *pHostif_p);
static tHostifReturn checkVersion (tHostif *pHostif_p);
static tHostifReturn allocateDynBuffers (tHostif *pHostif_p);
static tHostifReturn freeDynBuffers (tHostif *pHostif_p);
static tHostifReturn setDynBuffers (tHostif *pHostif_p);
static tHostifReturn controlBridge (tHostif *pHostif_p, BOOL fEnable_p);
static BOOL getBridgeEnabled (tHostif *pHostif_p);
static tHostifReturn controlIrqMaster (tHostif *pHostif_p, BOOL fEnable_p);

static tHostifReturn queueConfig (tHostif *pHostif_p,
        tHostifInstanceId InstanceId_p, tQueueConfig *pQueueConfig_p);

static tHostifReturn addQueueProcess (tHostif *pHostif_p,
        tQueueProcess QueueProcess_p);
static tHostifReturn removeQueueProcess (tHostif *pHostif_p,
        tQueueProcess QueueProcess_p);

static tHostifReturn limConfig (tHostif *pHostif_p,
        tHostifInstanceId InstanceId_p, tLimConfig *pLimConfig_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a host interface instance

This function creates a host interface instance, and initializes it depending
on the pConfig_p parameters.

\param  pConfig_p               The caller provides the configuration
                                parameters with this pointer.
\param  ppInstance_p            The function returns with this double-pointer
                                the created instance pointer. (return)

\return tHostifReturn
\retval kHostifSuccessful       The host interface is configured successfully
                                with the provided parameters.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       Heap allocation was impossible or to many
                                instances are present.
\retval kHostifWrongMagic       Can't find a valid host interface (invalid
                                magic).
\retval kHostifWrongVersion     The version fields in hardware mismatches those
                                in software.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_create (tHostifConfig *pConfig_p, tHostifInstance *ppInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = NULL;
    int i;

    if(pConfig_p == NULL || ppInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    // check config
    switch(pConfig_p->ProcInstance)
    {
        case kHostifProcHost:
        case kHostifProcPcp:
            break;
        default:
            Ret = kHostifInvalidParameter;
            goto Exit;
    }

    // create instance
    pHostif = (tHostif*)malloc(sizeof(tHostif));

    if(pHostif == NULL)
    {
        Ret = kHostifNoResource;
        goto Exit;
    }

    memset(pHostif, 0, sizeof(tHostif));

    // initialize instance
    pHostif->config = *pConfig_p;

    if(pHostif->config.ProcInstance == kHostifProcPcp)
    {
        pHostif->iDynBufEntries = kHostifInstIdLast;
        pHostif->pDynBufTbl = (tHostifDynBufConfig*)aDynBufInitPcp;
    }

    // store hostif base
    if(pHostif->config.ProcInstance == kHostifProcHost)
    {
        pHostif->pBase = (UINT8*)HOSTIF_MAKE_NONCACHEABLE(HOSTIF_HOST_BASE);
    }
    else
    {
        pHostif->pBase = (UINT8*)HOSTIF_MAKE_NONCACHEABLE(HOSTIF_PCP_BASE);
    }

    // check magic
    Ret = checkMagic(pHostif);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

    // check version
    Ret = checkVersion(pHostif);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

    if(pHostif->config.ProcInstance == kHostifProcPcp)
    {
        // allocate buffers
        Ret = allocateDynBuffers(pHostif);

        if(Ret != kHostifSuccessful)
        {
            goto Exit;
        }
    }

    // store instance in array
    for(i=0; i<HOSTIF_INSTANCE_COUNT; i++)
    {
        if(paHostifInstance[i] == NULL)
        {
            // free entry found
            paHostifInstance[i] = pHostif;

            break;
        }
    }

    if(i == HOSTIF_INSTANCE_COUNT)
    {
        Ret = kHostifNoResource;
        goto Exit;
    }

    if(pHostif->config.ProcInstance == kHostifProcPcp)
    {
        // since everything is fine, activate bridge
        Ret = controlBridge(pHostif, TRUE);

        if(Ret != kHostifSuccessful)
        {
            goto Exit;
        }
    }

    if(pHostif->config.ProcInstance == kHostifProcHost)
    {
        // register isr in system
        if(HOSTIF_IRQ_REG(hostifIrqHandler, (void*)pHostif))
        {
            Ret = kHostifNoResource;
            goto Exit;
        }

        // enable system irq
        HOSTIF_IRQ_ENABLE();
    }

    // return instance pointer
    *ppInstance_p = pHostif;

Exit:
    if(Ret != kHostifSuccessful)
    {
        hostif_delete(pHostif);
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete host interface instance

This function deletes a host interface instance.

\param  pInstance_p             The host interface instance that should be
                                deleted

\return tHostifReturn
\retval kHostifSuccessful       The host interface is deleted successfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifHwWriteError     Deactivation of hardware is faulty.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_delete (tHostifInstance pInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;
    int i;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance == kHostifProcHost)
    {

        // enable system irq (ignore ret)
        HOSTIF_IRQ_DISABLE();

        // degister isr in system (ignore ret)
        HOSTIF_IRQ_REG(NULL, NULL);
    }

    // delete instance in instance array
    for(i=0; i<HOSTIF_INSTANCE_COUNT; i++)
    {
        if(pHostif == paHostifInstance[i])
        {
            paHostifInstance[i] = NULL;

            break;
        }
    }

    if(pHostif->config.ProcInstance == kHostifProcPcp)
    {
        // deactivate bridge (ignore ret)
        Ret = controlBridge(pHostif, FALSE);

        Ret = freeDynBuffers(pHostif);
    }

    freePtr(pHostif);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Returns the instance of the given processor instance

If the instance is not found NULL is returned

\param  Instance_p              Processor instance

\return tHostifInstance
\retval NULL                    host interface instance not found

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifInstance hostif_getInstance (tHostifProcInstance Instance_p)
{
    tHostifInstance pHostif = NULL;
    int i;

    // search through array and return the matching one
    for(i=0; i<HOSTIF_INSTANCE_COUNT; i++)
    {
        if(paHostifInstance[i]->config.ProcInstance == Instance_p)
        {
            pHostif = (tHostifInstance)paHostifInstance[i];

            break;
        }
    }

    return pHostif;
}

//------------------------------------------------------------------------------
/**
\brief  This function processes all resources of the host interface

This function processes the configured resources of the host interface like
the queues.

\param  pInstance_p             host interface instance

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_process (tHostifInstance pInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tQueueReturn qRet;
    tHostif *pHostif = (tHostif*)pInstance_p;
    tQueueProcess *pQueueProcess;
    int i;
    int iProcessed;
    BOOL fQueueEmpty;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    // set process pointer to very first entry
    pQueueProcess = pHostif->aQueueProcessTable;

    for(i = 0, iProcessed = 0; i < HOSTIF_QUEUE_COUNT; i++)
    {
        if(pQueueProcess->pInstance == NULL)
            continue;

        qRet = lfq_checkEmpty(pQueueProcess->pInstance, &fQueueEmpty);

        if(qRet != kQueueSuccessful)
        {
            Ret = kHostifInvalidParameter;
            goto Exit;
        }

        if(fQueueEmpty == FALSE && pQueueProcess->pfnCallback != NULL)
        {
            pQueueProcess->pfnCallback(pQueueProcess->pArg);
        }

        if(++iProcessed >= pHostif->iQueueProcessEntries)
            break;

        pQueueProcess++;
    }

    // add other resources to be processed here

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function adds an irq handler for the corresponding irq source

This function adds an irq handler function for the corresponding irq source.
Note: The provided callback is invoked within the interrupt context!
If the provided callback is NULL, then the irq source is disabled.

\param  pInstance_p             host interface instance
\param  irqSrc_p                irq source that should invoke the callback
\param  callback_p              callback that is invoked

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    Only the host may call this function.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqRegHdl (tHostifInstance pInstance_p,
        tHostifIrqSrc irqSrc_p, tHostifIrqCb pfnCb_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;
    UINT16 irqEnableVal;

    if(pInstance_p == NULL || irqSrc_p >= kHostifIrqSrcLast)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance != kHostifProcHost)
    {
        Ret = kHostifWrongProcInst;
        goto Exit;
    }

    // get irq source enable from hw
    irqEnableVal = hostif_readIrqEnable(pHostif->pBase);

    // enable irq source if callback is not NULL
    if(pfnCb_p != NULL)
        irqEnableVal |= (1 << irqSrc_p);
    else
        irqEnableVal &= ~(1 << irqSrc_p);

    // store callback in instance
    pHostif->apfnIrqCb[irqSrc_p] = pfnCb_p;

    // write irq source enable back to hw
    hostif_writeIrqEnable(pHostif->pBase, irqEnableVal);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function enables an irq source

This function enables an irq source from the Pcp side.

\param  pInstance_p             host interface instance
\param  irqSrc_p                irq source to be controlled
\param  fEnable_p               enable the irq source (TRUE)

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    Only the host may call this function.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqSourceEnable (tHostifInstance pInstance_p,
        tHostifIrqSrc irqSrc_p, BOOL fEnable_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;
    UINT16 irqEnableVal;

    if(pInstance_p == NULL || irqSrc_p >= kHostifIrqSrcLast)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance != kHostifProcPcp)
    {
        Ret = kHostifWrongProcInst;
        goto Exit;
    }

    // get irq source enable from hw
    irqEnableVal = hostif_readIrqEnable(pHostif->pBase);

    if(fEnable_p != FALSE)
        irqEnableVal |= (1 << irqSrc_p);
    else
        irqEnableVal &= ~(1 << irqSrc_p);

    // write irq source enable back to hw
    hostif_writeIrqEnable(pHostif->pBase, irqEnableVal);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function controls the master irq enable

This function allows the host to enable or disable all irq sources from the
host interface.

\param  pInstance_p             host interface instance
\param  fEnable_p               enable the master irq (TRUE)

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    Only the host may call this function.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_irqMasterEnable (tHostifInstance pInstance_p,
        BOOL fEnable_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance == kHostifProcHost)
    {
        // activte master irq enable
        Ret = controlIrqMaster(pHostif, fEnable_p);

        if(Ret != kHostifSuccessful)
        {
            goto Exit;
        }
    }
    else
        Ret = kHostifWrongProcInst;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets a boot base to the host interface

\param  pInstance_p             host interface instance
\param  base_p                  base address to boot memory

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setBootBase (tHostifInstance pInstance_p, UINT32 base_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeBootBase(pHostif->pBase, base_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the boot base from the host interface

\param  pInstance_p             host interface instance
\param  pBase_p                 pointer to boot base

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getBootBase (tHostifInstance pInstance_p, UINT32 *pBase_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pBase_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pBase_p = hostif_readBootBase(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets an init base to the host interface

\param  pInstance_p             host interface instance
\param  base_p                  base address to init memory

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setInitBase (tHostifInstance pInstance_p, UINT32 base_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeInitBase(pHostif->pBase, base_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the init base from the host interface

\param  pInstance_p             host interface instance
\param  pBase_p                 pointer to init base

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getInitBase (tHostifInstance pInstance_p, UINT32 *pBase_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pBase_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pBase_p = hostif_readInitBase(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets a command to the host interface

\param  pInstance_p             host interface instance
\param  cmd_p                   command

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setCommand (tHostifInstance pInstance_p, tHostifCommand cmd_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    hostif_writeCommand(pHostif->pBase, cmd_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets a command from the host interface

\param  pInstance_p             host interface instance
\param  pCmd_p                  command

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getCommand (tHostifInstance pInstance_p, tHostifCommand *pCmd_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pCmd_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pCmd_p = hostif_readCommand(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets a state to the host interface

Note that only the Pcp is allowed to write to this register!

\param  pInstance_p             host interface instance
\param  sta_p                   state

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    The caller processor instance is not allowed.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setState (tHostifInstance pInstance_p, tHostifState sta_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance != kHostifProcPcp)
    {
        // host can't set state field
        Ret = kHostifWrongProcInst;
        goto Exit;
    }

    hostif_writeState(pHostif->pBase, sta_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the state from the host interface

\param  pInstance_p             host interface instance
\param  pSta_p                  state

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getState (tHostifInstance pInstance_p, tHostifState *pSta_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pSta_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pSta_p = hostif_readState(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets an error/return to the host interface

Note that only the Pcp is allowed to write to this register!

\param  pInstance_p             host interface instance
\param  err_p                   error/return code

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    The caller processor instance is not allowed.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setError (tHostifInstance pInstance_p, tHostifError err_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance != kHostifProcPcp)
    {
        // host can't set error field
        Ret = kHostifWrongProcInst;
        goto Exit;
    }

    hostif_writeReturn(pHostif->pBase, err_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the error/return from the host interface

\param  pInstance_p             host interface instance
\param  pErr_p                  error/return

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getError (tHostifInstance pInstance_p, tHostifError *pErr_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pErr_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pErr_p = hostif_readReturn(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function sets the heart beat value to the host interface

Note that only the Pcp is allowed to write to this register!

\param  pInstance_p             host interface instance
\param  heartbeat_p             heart beat value

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifWrongProcInst    The caller processor instance is not allowed.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_setHeartbeat (tHostifInstance pInstance_p, UINT16 heartbeat_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(pHostif->config.ProcInstance != kHostifProcPcp)
    {
        // host can't set heart beat field
        Ret = kHostifWrongProcInst;
        goto Exit;
    }

    hostif_writeHeartbeat(pHostif->pBase, heartbeat_p);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function gets the heart beat value from the host interface

\param  pInstance_p             host interface instance
\param  pHeartbeat_p            heart beat value

\return tHostifReturn
\retval kHostifSuccessful       The process function exit without errors.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_getHeartbeat (tHostifInstance pInstance_p, UINT16 *pHeartbeat_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostif *pHostif = (tHostif*)pInstance_p;

    if(pInstance_p == NULL || pHeartbeat_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    *pHeartbeat_p = hostif_readHeartbeat(pHostif->pBase);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function acquires a dynamic buffer for the host

\param  pinstance_p             host interface instance
\param  pcpBaseAddr_p           address in pcp memory space
\param  ppDynBufBase_p          returns base address in host memory space

\return tHostifReturn
\retval kHostifSuccessful       Dynamic buffer acquired ppDynBufBase_p valid.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge is disabled.
\retval kHostifNoResource       No dynamic buffer is available

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_dynBufAcquire (tHostifInstance pInstance_p,
        UINT32 pcpBaseAddr_p, UINT8 **ppDynBufBase_p)
{
    tHostifReturn Ret;
    tHostif *pHostif = (tHostif*)pInstance_p;
    int i;

    if(pInstance_p == NULL || ppDynBufBase_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    Ret = kHostifNoResource;

    for(i=0; i<HOSTIF_DYNBUF_COUNT; i++)
    {
        if(pHostif->apDynBufHost[i] == 0)
        {
            // handle base address in pcp memory space
            pHostif->apDynBufHost[i] = (UINT8*)pcpBaseAddr_p;

            hostif_writeDynBufHost(pHostif->pBase, (UINT8)i, pcpBaseAddr_p);

            // return base address in host memory space
            *ppDynBufBase_p = (UINT8*)((UINT32)apDynBuf[i] +
                    (UINT32)pHostif->pBase);

            Ret = kHostifSuccessful;
            break;
        }
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  This function frees a dynamic buffer acquired by the host

\param  pInstance_p             host interface instance
\param  pcpBaseAddr_p           address in pcp memory space

\return tHostifReturn
\retval kHostifSuccessful       Dynamic buffer freed
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       No dynamic buffer is available to be freed

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_dynBufFree (tHostifInstance pInstance_p, UINT32 pcpBaseAddr_p)
{
    tHostifReturn Ret;
    tHostif *pHostif = (tHostif*)pInstance_p;
    int i;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    Ret = kHostifNoResource;

    for(i=0; i<HOSTIF_DYNBUF_COUNT; i++)
    {
        if((UINT32)pHostif->apDynBufHost[i] == pcpBaseAddr_p)
        {
            // free dynamic buffer
            pHostif->apDynBufHost[i] = 0;

            hostif_writeDynBufHost(pHostif->pBase, (UINT8)i, 0);

            Ret = kHostifSuccessful;
            break;
        }
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create a queue instance for the given host interface instance

This function creates queue instance for the given host instance.

\param  pInstance_p             host interface instance
\param  InstanceId_p            resource instance id (Queue)
\param  ppQueueInstance_p       double-pointer to return created queue instance

\return tHostifReturn
\retval kHostifSuccessful       The queue is created succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       The queue can't be created

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueCreate (tHostifInstance pInstance_p,
        tHostifInstanceId InstanceId_p, tHostifQueueInstance *ppQueueInstance_p)

{
    tHostifReturn Ret = kHostifSuccessful;
    tQueueReturn qRet;
    tHostifQueue *pHostifQueue;
    tQueueConfig QueueConfig;

    if(pInstance_p == NULL || ppQueueInstance_p == NULL ||
            InstanceId_p >= kHostifInstIdLast)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    // create hostif queue instance
    pHostifQueue = (tHostifQueue*)malloc(sizeof(tHostifQueue));

    if(pHostifQueue == NULL)
    {
        Ret = kHostifNoResource;
        goto Exit;
    }

    pHostifQueue->pHostif = (tHostif*)pInstance_p;

    Ret = queueConfig(pHostifQueue->pHostif, InstanceId_p, &QueueConfig);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

    qRet = lfq_create(&QueueConfig, &pHostifQueue->pQueueInstance);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueAlignment:
            Ret = kHostifBufferError;
            goto Exit;
        case kQueueHwError:
            Ret = kHostifHwWriteError;
            goto Exit;
        case kQueueNoResource:
        default:
            Ret = kHostifNoResource;
            goto Exit;
    }

    *ppQueueInstance_p = pHostifQueue;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a queue instance

This function deletes the queue instance

\param  pQueueInstance_p        queue instance that has to be deleted

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifUnspecError      Unspecified error.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueDelete (tHostifQueueInstance pQueueInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tQueueReturn qRet;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueProcess ProcessParam;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    // remove queue from processing
    // note: If queue is already removed or was never processed then
    //          it doesn't matter!
    ProcessParam.pInstance = pHostifQueue->pQueueInstance;

    removeQueueProcess(pHostifQueue->pHostif, ProcessParam);

    qRet = lfq_delete(pHostifQueue->pQueueInstance);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueInvalidParameter:
            Ret = kHostifInvalidParameter;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

    // delete hostif queue instance
    free(pHostifQueue);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register a queue callback

This function adds a queue callback for a specific queue instance into the
queue process array.

\param  pQueueInstance_p        queue instance
\param  pfnQueueCb_p            call back for queue instance
\param  pArg_p                  argument of call back

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       The queue can't be processed.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueCallback (tHostifQueueInstance pQueueInstance_p,
        tQueueCb pfnQueueCb_p, void *pArg_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueProcess ProcessParam;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    ProcessParam.pfnCallback = pfnQueueCb_p;
    ProcessParam.pInstance = pHostifQueue->pQueueInstance;
    ProcessParam.pArg = pArg_p;

    if(ProcessParam.pfnCallback == NULL)
    {
        Ret = removeQueueProcess(pHostifQueue->pHostif, ProcessParam);
    }
    else
    {
        Ret = addQueueProcess(pHostifQueue->pHostif, ProcessParam);
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset queue

This function triggers a queue reset with a given timeout

\param  pQueueInstance_p        queue instance

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted successfülly.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueReset (tHostifQueueInstance pQueueInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueReturn qRet;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifQueue->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    qRet = lfq_reset(pHostifQueue->pQueueInstance);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueInvalidParameter:
            Ret = kHostifInvalidParameter;
            goto Exit;
        case kQueueWrongCaller:
            Ret = kHostifWrongProcInst;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get queue entry count

This function returns the number of entries in the given queue

\param  pQueueInstance_p        queue instance
\param  pwEntryCount_p          returns the number of entries in the queue

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted successfülly.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueGetEntryCount (tHostifQueueInstance pQueueInstance_p,
        UINT16 *pEntryCount_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueReturn qRet;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifQueue->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    qRet = lfq_getEntryCount(pHostifQueue->pQueueInstance, pEntryCount_p);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueInvalidParameter:
            Ret = kHostifInvalidParameter;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Insert a queue entry

This function inserts the given queue entry into a specified queue.

\param  pQueueInstance_p        queue instance
\param  pData_p                 data to be inserted
\param  size_p                  size of data to be inserted

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted successfülly.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.
\retval kHostifBufferOverflow   The queue is full.
\retval kHostifUnspecError      Unknown error

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueInsert (tHostifQueueInstance pQueueInstance_p,
        UINT8 *pData_p, UINT16 size_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueReturn qRet;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifQueue->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    qRet = lfq_entryEnqueue(pHostifQueue->pQueueInstance, pData_p, size_p);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueFull:
            Ret = kHostifBufferOverflow;
            goto Exit;
        case kQueueAlignment:
        case kQueueInvalidParameter:
            Ret = kHostifInvalidParameter;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Extract queue entry

This function extracts an entry from the specified queue.

\param  pQueueInstance_p        queue instance
\param  pData_p                 buffer provided by the caller
\param  pSize_p                 buffer size, returns actual size

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted successfülly.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.
\retval kHostifBufferEmpty      The queue is empty.
\retval kHostifUnspecError      Unknown error

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_queueExtract (tHostifQueueInstance pQueueInstance_p,
        UINT8 *pData_p, UINT16 *pSize_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tHostifQueue *pHostifQueue = (tHostifQueue*)pQueueInstance_p;
    tQueueReturn qRet;

    if(pQueueInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifQueue->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    qRet = lfq_entryDequeue(pHostifQueue->pQueueInstance, pData_p, pSize_p);

    switch(qRet)
    {
        case kQueueSuccessful:
            break;
        case kQueueEmpty:
            Ret = kHostifBufferEmpty;
            goto Exit;
        case kQueueAlignment:
        case kQueueInvalidParameter:
        case kQueueNoResource:
            Ret = kHostifInvalidParameter;
            goto Exit;
        case kQueueInvalidEntry:
            Ret = kHostifBufferError;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create a linear memory instance for the given host interface instance

This function creates linear memory instance for the given host instance.

\param  pInstance_p             host interface instance
\param  InstanceId_p            resource instance id (Queue)
\param  ppQueueInstance_p       double-pointer to return created linear memory
                                instance

\return tHostifReturn
\retval kHostifSuccessful       The lim is created successfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifNoResource       The lim can't be created
\retval kHostifBufferError      The linear memory buffer is not UINT32-aligned.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_limCreate (tHostifInstance pInstance_p,
        tHostifInstanceId InstanceId_p, tHostifLimInstance *ppLimInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tLimReturn lRet = kLimSuccessful;
    tHostifLim *pHostifLim;
    tLimConfig LimConfig;

    if(pInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    // create hostif queue instance
    pHostifLim = (tHostifLim*)malloc(sizeof(tHostifLim));

    if(pHostifLim == NULL)
    {
        Ret = kHostifNoResource;
        goto Exit;
    }

    pHostifLim->pHostif = (tHostif*)pInstance_p;

    Ret = limConfig(pHostifLim->pHostif, InstanceId_p, &LimConfig);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

    lRet = lim_create(&LimConfig, &pHostifLim->pLimInstance);

    switch(lRet)
    {
        case kLimSuccessful:
            break;
        case kLimInvalidParameter:
            Ret = kHostifInvalidParameter;
            goto Exit;
        case kLimNoResource:
            Ret = kHostifNoResource;
            goto Exit;
        case kLimAlignment:
            Ret = kHostifBufferError;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

    *ppLimInstance_p = pHostifLim;

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a linear memory instance

This function deletes the linear memory instance

\param  pLimInstance_p          linear memory instance that has to be deleted

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifUnspecError      Unspecified error.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_limDelete (tHostifLimInstance pLimInstance_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tLimReturn lRet;
    tHostifLim *pHostifLim = (tHostifLim*)pLimInstance_p;

    if(pLimInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    lRet = lim_delete(pHostifLim->pLimInstance);

    switch(lRet)
    {
        case kLimSuccessful:
            break;
        case kLimInvalidInstance:
            Ret = kHostifInvalidParameter;
            goto Exit;
        default:
            Ret = kHostifUnspecError;
            goto Exit;
    }

    free(pHostifLim);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write to a linear memory instance

This function writes to the linear memory instance

\param  pLimInstance_p          linear memory instance to be written to
\param  offset_p                write at byte offset (UINT32-aligned!)
\param  pSrc_p                  pointer to data to be written (UINT32-aligned!)
\param  size_p                  size of data to be written

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.
\retval kHostifBufferOverflow   The write exceeds the linear memory buffer.
\retval kHostifUnspecError      Unspecified error.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_limWrite (tHostifLimInstance pLimInstance_p,
        UINT16 offset_p, UINT8 *pSrc_p, UINT16 size_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tLimReturn lRet;
    tHostifLim *pHostifLim = (tHostifLim*)pLimInstance_p;

    if(pLimInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifLim->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    lRet = lim_write(pHostifLim->pLimInstance, offset_p, pSrc_p, size_p);

    switch(lRet)
    {
        case kLimSuccessful:
            break;
        case kLimInvalidParameter:
        case kLimInvalidInstance:
            Ret = kHostifInvalidParameter;
            goto Exit;
        case kLimOverflow:
            Ret = kHostifBufferOverflow;
            goto Exit;
        case kLimAlignment:
        default:
            Ret = kHostifBufferError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read from a linear memory instance

This function reads from the linear memory instance

\param  pLimInstance_p          linear memory instance to be written to
\param  pDst_p                  pointer to buffer to be written to
                                (UINT32-aligned!)
\param  offset_p                write at byte offset (UINT32-aligned!)
\param  size_p                  size of buffer

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.
\retval kHostifBridgeDisabled   The bridge logic of the hostif is disabled.
\retval kHostifBufferOverflow   The read exceeds the linear memory buffer.
\retval kHostifUnspecError      Unspecified error.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_limRead (tHostifLimInstance pLimInstance_p,
        UINT8 *pDst_p, UINT16 offset_p, UINT16 size_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tLimReturn lRet;
    tHostifLim *pHostifLim = (tHostifLim*)pLimInstance_p;

    if(pLimInstance_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    if(getBridgeEnabled(pHostifLim->pHostif) == FALSE)
    {
        Ret = kHostifBridgeDisabled;
        goto Exit;
    }

    lRet = lim_read(pHostifLim->pLimInstance, offset_p, pDst_p, size_p);

    switch(lRet)
    {
        case kLimSuccessful:
            break;
        case kLimInvalidParameter:
        case kLimInvalidInstance:
            Ret = kHostifInvalidParameter;
            goto Exit;
        case kLimOverflow:
            Ret = kHostifBufferOverflow;
            goto Exit;
        case kLimAlignment:
        default:
            Ret = kHostifBufferError;
            goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get the linear memory buffer

This function provides the base address and size of the linear memory buffer of
the specified instance.

\param  pLimInstance_p          linear memory instance of interest
\param  ppBase_p                With this pointer the base address is returned.
\param  pSpan_p                 With this pointer the size is returned.

\return tHostifReturn
\retval kHostifSuccessful       The queue is deleted succesfully.
\retval kHostifInvalidParameter The caller has provided incorrect parameters.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_limGetBuffer (tHostifLimInstance pLimInstance_p,
        UINT8 **ppBase_p, UINT16 *pSpan_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    tLimReturn lRet;
    tHostifLim *pHostifLim = (tHostifLim*)pLimInstance_p;

    if(pLimInstance_p == NULL || ppBase_p == NULL || pSpan_p == NULL)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    lRet = lim_getBase(pHostifLim->pLimInstance, ppBase_p);

    if(lRet != kLimSuccessful)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

    lRet = lim_getSpan(pHostifLim->pLimInstance, pSpan_p);

    if(lRet != kLimSuccessful)
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

Exit:
    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Host Interrupt Handler

This is the host interrupt handler which should by called by the system if the
irq signal is asserted by the ipcore. This handler acknowledges the processed
interrupt sources and calls the corresponding callbacks registered with
hostif_irqRegHdl().

\param  pArg_p                  The system caller should provide the host
                                interface instance with this parameter.
*/
//------------------------------------------------------------------------------
static void hostifIrqHandler (void *pArg_p)
{
    tHostif *pHostif = (tHostif*)pArg_p;
    UINT16 pendings;
    UINT16 mask;
    int i;

    if(pArg_p == NULL)
    {
        goto Exit;
    }

    pendings = hostif_readIrqPending(pHostif->pBase);

    for(i=0; i<kHostifIrqSrcLast; i++)
    {
        mask = 1 << i;

        //ack irq source first
        if(pendings & mask)
            hostif_ackIrq(pHostif->pBase, mask);

        //then try to execute the callback
        if(pHostif->apfnIrqCb[i] != NULL)
            pHostif->apfnIrqCb[i](pArg_p);
    }

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Free pointers which are not NULL

\param  p                       Pointer to be freed
*/
//------------------------------------------------------------------------------
static void freePtr(void *p)
{
    if(p != NULL)
        free(p);
}

//------------------------------------------------------------------------------
/**
\brief  Check magic word of ipcore

This function reads and verifies the magic word from the host interface.

\param  pHostif_p               Host interface instance

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn checkMagic(tHostif *pHostif_p)
{
    if(hostif_readMagic(pHostif_p->pBase) == HOSTIF_MAGIC)
        return kHostifSuccessful;
    else
        return kHostifWrongMagic;
}

//------------------------------------------------------------------------------
/**
\brief  Check version of ipcore

This function reads and verifies the version from the host interface.

\param  pHostif_p               Host interface instance

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn checkVersion(tHostif *pHostif_p)
{
    UINT32 version = hostif_readVersion(pHostif_p->pBase);
    tHostifVersion *pVersion = (tHostifVersion*)&version;

    if( (HOSTIF_VERSION_COUNT == pVersion->cnt)
     && (HOSTIF_VERSION_REVISION == pVersion->revision)
     && (HOSTIF_VERSION_MINOR == pVersion->minor)
     && (HOSTIF_VERSION_MAJOR == pVersion->major)
      )
        return kHostifSuccessful;
    else
        return kHostifWrongVersion;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate the dynamic buffers in heap memory

This function allocates the memory necessary for the dynamic buffers in the heap
on Pcp side. Note that this function does not allocate memory for buffers that
can be set from host side, instead buffers like K2U-Queue or Error Counters.

\param  pHostif_p               Host interface instance

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn allocateDynBuffers (tHostif *pHostif_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    int i;
    UINT8 *pBase;
    UINT16 span;

    for(i=0; i<pHostif_p->iDynBufEntries; i++)
    {
        pBase = NULL;
        span = pHostif_p->pDynBufTbl[i].span;

        pBase = (UINT8*)HOSTIF_UNCACHED_MALLOC(span);

        if(pBase == NULL)
        {
            Ret = kHostifNoResource;
            goto Exit;
        }

        memset(pBase, 0, span);

        pHostif_p->pDynBufTbl[i].pBase = pBase;
    }

    Ret = setDynBuffers(pHostif_p);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

Exit:
    if(Ret != kHostifSuccessful)
    {
        // i points to failed one
        while(i != 0)
        {
            freePtr(pHostif_p->pDynBufTbl[--i].pBase);

            pHostif_p->pDynBufTbl[i].pBase = NULL;
        }
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the dynamic buffers in heap memory

This function frees the memory necessary for the dynamic buffers in the heap
on Pcp side. Note that this function does not free memory for buffers that
can be set from host side, instead buffers like K2U-Queue or Error Counters.

\param  pHostif_p               Host interface instance

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn freeDynBuffers (tHostif *pHostif_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    int i;

    for(i=0; i<pHostif_p->iDynBufEntries; i++)
    {
        HOSTIF_UNCACHED_FREE(pHostif_p->pDynBufTbl[i].pBase);

        pHostif_p->pDynBufTbl[i].pBase = NULL;
    }

    Ret = setDynBuffers(pHostif_p);

    if(Ret != kHostifSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set the address to the dynamic buffers

This function sets the base addresses of the dynamic buffers in Pcp environment
to the host interface bridge logic.

\param  pHostif_p               Host interface instance

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn setDynBuffers (tHostif *pHostif_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    int i;
    UINT8 *pAddr;

    for(i=0; i<pHostif_p->iDynBufEntries; i++)
    {
        pAddr = pHostif_p->pDynBufTbl[i].pBase;

        pHostif_p->pDynBufTbl[i].pfnSetDynBuf(pHostif_p->pBase, (UINT32)pAddr);
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Turn on/off the host interface bridge

This function turns on or off the bridge logic from Pcp side.
This refuses the host accessing the Pcp-memory space in case of an uninitialized
host interface.
The function writes the specific enable pattern to hardware and reads it back
again.

\param  pHostif_p               Host interface instance
\param  fEnable_p               Enable the bridge with TRUE

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn controlBridge (tHostif *pHostif_p, BOOL fEnable_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    UINT16 dst = 0;
    UINT16 src;

    if(fEnable_p != FALSE)
    {
        dst = HOSTIF_BRIDGE_ENABLE;
    }

    // set value to hw
    hostif_writeBridgeEnable(pHostif_p->pBase, dst);

    // read back value from hw and check if write was successful
    src = hostif_readBridgeEnable(pHostif_p->pBase);

    if((src & HOSTIF_BRIDGE_ENABLE) != dst)
    {
        Ret = kHostifHwWriteError;
        goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get bridge turned on/off

This getter returns whether the bridge is turned on or off.

\param  pHostif_p               Host interface instance

\return The function returns TRUE if the bridge is turned on, otherwise FALSE.
*/
//------------------------------------------------------------------------------
static BOOL getBridgeEnabled (tHostif *pHostif_p)
{
    UINT16 val;

    val = hostif_readBridgeEnable(pHostif_p->pBase);

    if(val & HOSTIF_BRIDGE_ENABLE)
        return TRUE;
    else
        return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Turn on/off the host interface interrupt master

This function turns on or off the interrupt master from host side.
The function writes the specific enable pattern to hardware and reads it back
again.

\param  pHostif_p               Host interface instance
\param  fEnable_p               Enable interrupt master with TRUE

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn controlIrqMaster (tHostif *pHostif_p, BOOL fEnable_p)
{
    tHostifReturn Ret = kHostifSuccessful;
    UINT16 dst = 0;
    UINT16 src;

    if(fEnable_p != FALSE)
    {
        dst = HOSTIF_IRQ_MASTER_ENABLE;
    }

    // set value to hw
    hostif_writeIrqMasterEnable(pHostif_p->pBase, dst);

    // read back value from hw and check if write was successful
    src = hostif_readIrqMasterEnable(pHostif_p->pBase);

    if((src & HOSTIF_IRQ_MASTER_ENABLE) != dst)
    {
        Ret = kHostifHwWriteError;
        goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure queue instances

This function configures the queue instances depending on the processor instance
(Pcp or host).

\param  pHostif_p               Host interface instance
\param  InstanceId_p            Instance to be configured
\param  pQueueConfig_p          Reference to queue configuration structure

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn queueConfig (tHostif *pHostif_p,
        tHostifInstanceId InstanceId_p, tQueueConfig *pQueueConfig_p)
{
    tHostifReturn Ret = kHostifSuccessful;

    // initialize queue config from constants
    switch(InstanceId_p)
    {
        case kHostifInstIdTxNmtQueue:
            *pQueueConfig_p = QueueConfigTxNmt;
            break;
        case kHostifInstIdTxGenQueue:
            *pQueueConfig_p = QueueConfigTxGen;
            break;
        case kHostifInstIdTxSyncQueue:
            *pQueueConfig_p = QueueConfigTxSync;
            break;
        case kHostifInstIdTxVethQueue:
            *pQueueConfig_p = QueueConfigTxVeth;
            break;
        case kHostifInstIdRxVethQueue:
            *pQueueConfig_p = QueueConfigRxVeth;
            break;
        case kHostifInstIdK2UQueue:
            *pQueueConfig_p = QueueConfigK2U;
            break;
        case kHostifInstIdU2KQueue:
            *pQueueConfig_p = QueueConfigU2K;
            break;
        default:
            Ret = kHostifInvalidParameter;
            goto Exit;
    }

    // set producer or consumer depending on host/pcp
    switch(InstanceId_p)
    {
        case kHostifInstIdTxNmtQueue:
        case kHostifInstIdTxGenQueue:
        case kHostifInstIdTxSyncQueue:
        case kHostifInstIdTxVethQueue:
        case kHostifInstIdU2KQueue:
            if(pHostif_p->config.ProcInstance == kHostifProcHost)
            {
                pQueueConfig_p->queueRole = kQueueProducer;
            }
            else
            {
                pQueueConfig_p->queueRole = kQueueConsumer;
            }
            break;
        case kHostifInstIdRxVethQueue:
        case kHostifInstIdK2UQueue:
            if(pHostif_p->config.ProcInstance == kHostifProcPcp)
            {
                pQueueConfig_p->queueRole = kQueueProducer;
            }
            else
            {
                pQueueConfig_p->queueRole = kQueueConsumer;
            }
            break;
        default:
            Ret = kHostifInvalidParameter;
            goto Exit;
    }

    // buffer allocation (malloc is done in hostif_create!)
    pQueueConfig_p->fAllocHeap = FALSE;
    if(pHostif_p->config.ProcInstance == kHostifProcPcp)
    {
        // get buffer in heap
        pQueueConfig_p->pBase =
                (UINT8*)pHostif_p->pDynBufTbl[InstanceId_p].pBase;
    }
    else if(pHostif_p->config.ProcInstance == kHostifProcHost)
    {
        // add hostif offset
        pQueueConfig_p->pBase = (UINT8*)((UINT32)pQueueConfig_p->pBase +
                (UINT32)pHostif_p->pBase);
    }
    else
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add queue instance to processing list

This function adds a queue instance to the processing table of the host
interface instance.
If the function hostif_process() is called, the queue process table is
processed.

\param  pHostif_p               Host interface instance
\param  QueueProcess_p          Entry to be added to queue process table

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn addQueueProcess (tHostif *pHostif_p,
        tQueueProcess QueueProcess_p)
{
    tHostifReturn Ret = kHostifNoResource;
    int i;

    for(i=0; i<HOSTIF_QUEUE_COUNT; i++)
    {
        if(pHostif_p->aQueueProcessTable[i].pInstance == NULL)
        {
            pHostif_p->aQueueProcessTable[i] = QueueProcess_p;
            pHostif_p->iQueueProcessEntries++;
            Ret = kHostifSuccessful;
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Remove queue instance from processing list

This function removes a queue instance from the processing table of the host
interface instance.

\param  pHostif_p               Host interface instance
\param  QueueProcess_p          Entry to be removed from queue process table

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn removeQueueProcess (tHostif *pHostif_p,
        tQueueProcess QueueProcess_p)
{
    tHostifReturn Ret = kHostifNoResource;
    int i;

    for(i=0; i<HOSTIF_QUEUE_COUNT; i++)
    {
        if(pHostif_p->aQueueProcessTable[i].pInstance ==
                QueueProcess_p.pInstance)
        {
            memset(&pHostif_p->aQueueProcessTable[i], 0, sizeof(tQueueProcess));
            pHostif_p->iQueueProcessEntries--;
            Ret = kHostifSuccessful;
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure linear memory instances

This function configures the linear memory instances depending on the processor
instance (Pcp or host).

\param  pHostif_p               Host interface instance
\param  InstanceId_p            Instance to be configured
\param  pLimConfig_p            Reference to linear memory configuration
                                structure

\return The function returns tHostifReturn error code.
*/
//------------------------------------------------------------------------------
static tHostifReturn limConfig (tHostif *pHostif_p,
        tHostifInstanceId InstanceId_p, tLimConfig *pLimConfig_p)
{
    tHostifReturn Ret = kHostifSuccessful;

    // initialize linear memory config from constants
    switch(InstanceId_p)
    {
        case kHostifInstIdErrCount:
            *pLimConfig_p = LimConfigErrCount;
            break;
        case kHostifInstIdRpdo:
            *pLimConfig_p = LimConfigRpdo;
            break;
        case kHostifInstIdTpdo:
            *pLimConfig_p = LimConfigTpdo;
            break;
        default:
            Ret = kHostifInvalidParameter;
            goto Exit;
    }

    // buffer allocation (malloc is done in hostif_create!)
    pLimConfig_p->fAllocHeap = FALSE;
    if(pHostif_p->config.ProcInstance == kHostifProcPcp)
    {
        // get buffer in heap
        pLimConfig_p->pBase =
                (UINT8*)pHostif_p->pDynBufTbl[InstanceId_p].pBase;
    }
    else if(pHostif_p->config.ProcInstance == kHostifProcHost)
    {
        // add hostif offset
        pLimConfig_p->pBase = (UINT8*)((UINT32)pLimConfig_p->pBase +
                (UINT32)pHostif_p->pBase);
    }
    else
    {
        Ret = kHostifInvalidParameter;
        goto Exit;
    }

Exit:
    return Ret;
}
