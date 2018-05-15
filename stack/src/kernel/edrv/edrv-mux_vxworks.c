/**
********************************************************************************
\file   edrv-mux_vxworks.c

\brief  Implementation of VxWorks MUX Ethernet driver

This file contains the implementation of the VxWorks MUX Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <kernel/edrv.h>

#include <unistd.h>
#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <muxLib.h>
#include <netBufLib.h>
#include <semLib.h>
#include <netLib.h>
#include <taskLib.h>
#include <logLib.h>
#include <sysLib.h>

#include <end.h>
#include <netinet/in.h>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <net/if.h>

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
#include "hrtimerLib.h"
#endif

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
#define EDRV_MAX_FRAME_SIZE     0x0600

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                          ///< Init parameters
    tEdrvTxBuffer*      pTransmittedTxBufferLastEntry;      ///< Pointer to the last entry of the transmitted TX buffer
    tEdrvTxBuffer*      pTransmittedTxBufferFirstEntry;     ///< Pointer to the first entry of the transmitted Tx buffer
    SEM_ID              mutex;                              ///< Mutex for locking of critical sections
    SEM_ID              syncSem;                            ///< Semaphore for thread synchronization

    NET_POOL_ID         dataPoolId;                         ///< Data pool to allocate packet buffers
    PROTO_COOKIE        pCookie;                            ///< Handle of the MUX interface used for POWERLINK
    int                 txTaskId;                           ///< ID of the TX handler task
    SEM_ID              txWakeupSem;                        ///< Semaphore to wake up the TX sender task
    BOOL                fStopTxTask;                        ///< Flag indicating whether the TX sender task shall be stopped

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    struct timespec     txSendTime;                         ///< Time of issuing a TX frame
    struct timespec     txCbTime;                           ///< Time of entering the TX sender task
    struct timespec     maxTxLatency;                       ///< Max time difference between issuing a frame and entering the sender task
#endif

} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static BOOL   packetHandler(void* pCookie_p, LONG type_p, M_BLK_ID pPkt_p,
                            LL_HDR_INFO* pLLHInfo_p, void* pNetCallbackId_p);
static STATUS muxShutdown(void* pCookie_p, void* pNetCallbackId_p);
static STATUS muxRestart(void* pEnd_p, void* pNetCallbackId_p);
static void   muxError(END_OBJ* pEnd_p, END_ERR* pError_p, void* pNetCallbackId_p);
static int    txTask(int iArg_p);
static void   getMacAddr(PROTO_COOKIE pCookie_p, const char* pIfName, UINT8* pMacAddr_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(const tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError      ret = kErrorOk;
    NETBUF_CFG      bufCfgData;
    NETBUF_CL_DESC  clDescTblData;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    if (pEdrvInitParam_p->pDevName == NULL)
    {
        ret = kErrorEdrvInit;
        goto Exit;
    }

    /* create memory pool */
    clDescTblData.clNum = 65536;
    clDescTblData.clSize = 2048;

    bufCfgData.pName = "data";
    bufCfgData.attributes = ATTR_AI_SH_ISR;
    bufCfgData.pDomain = NULL;
    bufCfgData.bMemExtraSize = 0;
    bufCfgData.ctrlPartId = NULL;
    bufCfgData.bMemPartId = NULL;
    bufCfgData.ctrlNumber = 65536;

    bufCfgData.clDescTblNumEnt = 1;
    bufCfgData.pClDescTbl = &clDescTblData;

    if ((edrvInstance_l.dataPoolId = netPoolCreate(&bufCfgData, NULL)) == NULL)
    {
        ret = kErrorEdrvInit;
        goto Exit;
    }

    DEBUG_LVL_EDRV_TRACE("%s() Using interface %s%d\n", __func__,
                           pEdrvInitParam_p->pDevName,
                           pEdrvInitParam_p->devNum);
    /* Binding to Mux Device */
    if ((edrvInstance_l.pCookie =
               muxBind(pEdrvInitParam_p->pDevName,
                       pEdrvInitParam_p->devNum,
                       packetHandler, muxShutdown, muxRestart,
                       muxError,
                       MUX_PROTO_PROMISC, "POWERLINK", &edrvInstance_l)) == NULL)
    {
        ret = kErrorEdrvInit;
        netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);
        goto Exit;
    }

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    /* if no MAC address was specified read MAC address of used
     * Ethernet interface
     */
    if ((edrvInstance_l.initParam.aMacAddr[0] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[1] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[2] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[3] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[4] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[5] == 0))
    {   // read MAC address from controller
        getMacAddr(edrvInstance_l.pCookie,
                   edrvInstance_l.initParam.pDevName,
                   edrvInstance_l.initParam.aMacAddr);
    }

    if ((edrvInstance_l.mutex =
                    semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init mutex\n", __func__);
        ret = kErrorEdrvInit;
        muxUnbind(edrvInstance_l.pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)packetHandler);
        netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);
        goto Exit;
    }

    if ((edrvInstance_l.syncSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        ret = kErrorEdrvInit;
        muxUnbind(edrvInstance_l.pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)packetHandler);
        netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(edrvInstance_l.mutex);
        goto Exit;
    }

    if ((edrvInstance_l.txWakeupSem = semCCreate(SEM_Q_FIFO, 0)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        ret = kErrorEdrvInit;
        muxUnbind(edrvInstance_l.pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)packetHandler);
        netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(edrvInstance_l.mutex);
        semDelete(edrvInstance_l.syncSem);
        goto Exit;
    }

    /* create TX handler task */
    edrvInstance_l.fStopTxTask = FALSE;
    if ((edrvInstance_l.txTaskId = taskSpawn("tTxHandler",
                                             EPL_TASK_PRIORITY_NETTX,
                                             0,
                                             EPL_TASK_STACK_SIZE,
                                             txTask,
                                             (int)&edrvInstance_l,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0))
                                == ERROR)
    {
        DEBUG_LVL_ERROR_TRACE ("%s() Couldn't create TX handler task!\n",
                                 __func__);
        muxUnbind(edrvInstance_l.pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)packetHandler);
        netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(edrvInstance_l.mutex);
        semDelete(edrvInstance_l.syncSem);
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down Ethernet driver

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_exit(void)
{
    // signal shutdown to the thread
    muxUnbind(edrvInstance_l.pCookie, MUX_PROTO_PROMISC,
              (FUNCPTR)packetHandler);

    /* stop TX handler task */
    edrvInstance_l.fStopTxTask = TRUE;
    semGive(edrvInstance_l.txWakeupSem);
    taskDelay(sysClkRateGet() / 10);

    netPoolRelease(edrvInstance_l.dataPoolId, NET_REL_IN_CONTEXT);

    semDelete(edrvInstance_l.mutex);
    semDelete(edrvInstance_l.syncSem);
    semDelete(edrvInstance_l.txWakeupSem);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    return kErrorOk; //assuming no problems with closing the handle
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address

This function returns the MAC address of the Ethernet controller

\return The function returns a pointer to the MAC address.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
const UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError  ret = kErrorOk;
    int         muxRet;
    M_BLK_ID    pPacket;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
    {
        ret = kErrorInvalidOperation;
        goto Exit;
    }

    semTake(edrvInstance_l.mutex, WAIT_FOREVER);
    if (edrvInstance_l.pTransmittedTxBufferLastEntry == NULL)
    {
        edrvInstance_l.pTransmittedTxBufferLastEntry =
            edrvInstance_l.pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        edrvInstance_l.pTransmittedTxBufferLastEntry->txBufferNumber.pArg =
                                                                     pBuffer_p;
        edrvInstance_l.pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    semGive(edrvInstance_l.mutex);

    /* generate packet */
    if ((pPacket = netTupleGet(edrvInstance_l.dataPoolId, EDRV_MAX_MTU,
                               M_WAIT, MT_HEADER, TRUE)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't get tuple!\n", __func__);
        goto Exit;
    }
    pPacket->mBlkHdr.reserved = htons(0x88ab);
    OPLK_MEMCPY(pPacket->mBlkHdr.mData, pBuffer_p->pBuffer, pBuffer_p->txFrameSize);
    pPacket->mBlkHdr.mLen = pBuffer_p->txFrameSize;
    /* send packet out */
    if ((muxRet = muxSend(edrvInstance_l.pCookie, pPacket)) != OK)
    {
        DEBUG_LVL_ERROR_TRACE("%s() muxSend returned %d\n", __func__, muxRet);
        /* as muxSend was not successful we still own the packet and have to
         * free it! */
        netMblkClChainFree(pPacket);
        ret = kErrorInvalidOperation;
    }

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
     hrtimer_clock_gettime(0, &edrvInstance_l.txSendTime);
#endif

    /* wakeup TX Send Task */
    semGive(edrvInstance_l.txWakeupSem);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc
    pBuffer_p->pBuffer = OPLK_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->txBufferNumber.pArg = NULL;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    void*   pBuffer;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    pBuffer = pBuffer_p->pBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->pBuffer = NULL;

    OPLK_FREE(pBuffer);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                               UINT count_p,
                               UINT entryChanged_p,
                               UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    if (muxMCastAddrDel(edrvInstance_l.pCookie, (char*)pMacAddr_p) != OK)
    {
        DEBUG_LVL_EDRV_TRACE("error clearing multicast addresses\n");
        return kErrorEdrvInit;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    if (muxMCastAddrAdd(edrvInstance_l.pCookie, (char*)pMacAddr_p) != OK)
    {
        DEBUG_LVL_EDRV_TRACE("error adding multicast addresses\n");
        return kErrorEdrvInit;
    }
    return kErrorOk;
}

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Show Ethernet driver information

This function shows the Ethernet driver information.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
void edrv_showDiagnose(void)
{
    printf("Max Tx Callback Latency: %ld ns\n",
            edrvInstance_l.maxTxLatency.tv_nsec);
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Edrv packet handler

This function is the packet handler forwarding the frames to the dllk.

\param[in,out]  pCookie_p           Pointer to MUX interface
\param[in]      type_p              Network service type of the packet
\param[in]      pPkt_p              M_BLK tuple chain of the packet
\param[in,out]  pLLHInfo_p          Pointer to link level header structure
\param[in,out]  pNetCallbackId_p    Handle installed at bind time

\return The function returns a BOOL.
*/
//------------------------------------------------------------------------------
static BOOL packetHandler(void* pCookie_p,
                          LONG type_p,
                          M_BLK_ID pPkt_p,
                          LL_HDR_INFO* pLLHInfo_p,
                          void* pNetCallbackId_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pNetCallbackId_p;
    tEdrvRxBuffer   rxBuffer;
    char            aBuffer[EDRV_MAX_MTU];

    UNUSED_PARAMETER(pCookie_p);
    UNUSED_PARAMETER(type_p);

    if (OPLK_MEMCMP(pPkt_p->mBlkHdr.mData + pLLHInfo_p->srcAddrOffset,
                    pInstance->initParam.aMacAddr, 6) != 0)
    {
        rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
        rxBuffer.rxFrameSize = netMblkToBufCopy(pPkt_p, aBuffer, NULL);
        rxBuffer.pBuffer = (void*)aBuffer;

        pInstance->initParam.pfnRxHandler(&rxBuffer);
        netMblkClChainFree(pPkt_p);
    }
    else
    {   // self generated traffic
        DEBUG_LVL_EDRV_TRACE ("%s() self generated traffic!\n", __func__);
    }
    return TRUE;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown Edrv MUX

This function shuts down the Edrv MUX.

\param[in,out]  pCookie_p           Pointer to MUX interface
\param[in,out]  pNetCallbackId_p    Handle installed at bind time

\return The function returns a STATUS error code.
*/
//------------------------------------------------------------------------------
static STATUS muxShutdown(void* pCookie_p, void* pNetCallbackId_p)
{
    UNUSED_PARAMETER(pCookie_p);
    UNUSED_PARAMETER(pNetCallbackId_p);

    DEBUG_LVL_EDRV_TRACE("muxShutdown()\n");
    return OK;
}

//------------------------------------------------------------------------------
/**
\brief  Restart Edrv MUX

This function restarts the Edrv MUX.

\param[in,out]  pEnd_p              END_OBJ passed to the MUX by the driver
\param[in,out]  pNetCallbackId_p    Handle installed at bind time

\return The function returns a STATUS error code.
*/
//------------------------------------------------------------------------------
static STATUS muxRestart(void* pEnd_p, void* pNetCallbackId_p)
{
    UNUSED_PARAMETER(pEnd_p);
    UNUSED_PARAMETER(pNetCallbackId_p);

    DEBUG_LVL_EDRV_TRACE("muxRestart()\n");
    return OK;
}

//------------------------------------------------------------------------------
/**
\brief  Edrv MUX error callback

This is the error callback function of the Edrv MUX.

\param[in,out]  pEnd_p              END_OBJ passed to the MUX by the driver
\param[in,out]  pError_p            Pointer to structure containing the error
\param[in,out]  pNetCallbackId_p    Handle installed at bind time
*/
//------------------------------------------------------------------------------
static void muxError(END_OBJ* pEnd_p, END_ERR* pError_p, void* pNetCallbackId_p)
{
    UNUSED_PARAMETER(pEnd_p);
    UNUSED_PARAMETER(pError_p);
    UNUSED_PARAMETER(pNetCallbackId_p);

    DEBUG_LVL_EDRV_TRACE("muxError()\n");
}

//------------------------------------------------------------------------------
/**
\brief  Transmit callback task

This is transmit callback task.

\param[in]      arg_p               Task argument contains the instance pointer

\return The function returns 0.
*/
//------------------------------------------------------------------------------
static int txTask(int arg_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)arg_p;
    STATUS          result;
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    struct timespec txLatency;
#endif

    while (TRUE)
    {
        result = semTake(pInstance->txWakeupSem, WAIT_FOREVER);
        if (pInstance->fStopTxTask)
        {
            break;
        }

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
        hrtimer_clock_gettime(0, &edrvInstance_l.txCbTime);
        txLatency = hrtimer_subTimespec(edrvInstance_l.txCbTime,
                                        edrvInstance_l.txSendTime);
        if (hrtimer_compareTimespec(&txLatency, &edrvInstance_l.maxTxLatency) > 0)
        {
            edrvInstance_l.maxTxLatency = txLatency;
        }
#endif

        if (result == OK)
        {
            if (pInstance->pTransmittedTxBufferFirstEntry != NULL)
            {
                tEdrvTxBuffer* pTxBuffer = pInstance->pTransmittedTxBufferFirstEntry;

                if (pTxBuffer->pBuffer != NULL)
                {
                    semTake(pInstance->mutex, WAIT_FOREVER);
                    pInstance->pTransmittedTxBufferFirstEntry =
                        pInstance->pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->pTransmittedTxBufferLastEntry = NULL;
                    }
                    semGive(pInstance->mutex);

                    pTxBuffer->txBufferNumber.pArg = NULL;

                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    logMsg("%s() pTxBuffer->pBuffer == NULL\n",
                           (int)__func__, 0, 0, 0, 0, 0);
                }
            }
            else
            {
                logMsg("%s() pTransmittedTxBufferFirstEntry == NULL\n",
                       (int)__func__, 0, 0, 0, 0, 0);
            }
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get Edrv MAC address

This function gets the interface's MAC address.

\param[in]      pCookie_p           Pointer to MUX interface
\param[in]      pIfName_p           Ethernet interface device name
\param[out]     pMacAddr_p          Pointer to store MAC address
*/
//------------------------------------------------------------------------------
static void getMacAddr(PROTO_COOKIE pCookie_p, const char* pIfName_p, UINT8* pMacAddr_p)
{
    char aData[6];

    UNUSED_PARAMETER(pIfName_p);

    muxIoctl(pCookie_p, EIOCGADDR, aData);
    OPLK_MEMCPY(pMacAddr_p, aData, 6);
}

/// \}
