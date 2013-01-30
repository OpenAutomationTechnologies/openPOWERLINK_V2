/****************************************************************************
  File:         EdrvMuxVxWorks.c

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  VxWorks MUX implementation of openPOWERLINK Edrv module

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holders nor the names of
       its contributors may be used to endorse or promote products derived
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

****************************************************************************/

#include "edrv.h"

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

#if EDRV_USE_DIAGNOSTICS != FALSE
#include "hrtimerLib.h"
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
// module global types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
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
#define EDRV_MAX_FRAME_SIZE     0x600

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{
    tEdrvInitParam      m_initParam;
    tEdrvTxBuffer*      m_pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*      m_pTransmittedTxBufferFirstEntry;
    SEM_ID              m_mutex;
    SEM_ID              m_syncSem;

    NET_POOL_ID         m_dataPoolId;
    PROTO_COOKIE        m_pCookie;
    int                 m_txTaskId;
    SEM_ID              m_txWakeupSem;
    BOOL                m_fStopTxTask;

#if EDRV_USE_DIAGNOSTICS != FALSE
    struct timespec     m_txSendTime;
    struct timespec     m_txCbTime;
    struct timespec     m_maxTxLatency;
#endif

} tEdrvInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------
static tEdrvInstance EdrvInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static BOOL EdrvPacketHandler (void * pCookie, long  type, M_BLK_ID pPkt,
                               LL_HDR_INFO *pLLHInfo, void *netCallbackId);
static int EdrvTxTask (int iArg_p);


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EdrvPacketHandler
//
// Description: Receive Callback function of MUX interface
//
// Parameters:  pCookie     =
//              type        =
//              pPkt        =
//              pLLHinfo    =
//              netCallbackId =
//
// Returns:     BOOL
//---------------------------------------------------------------------------
static BOOL EdrvPacketHandler (void * pCookie, long  type, M_BLK_ID pPkt,
                               LL_HDR_INFO *pLLHInfo, void *netCallbackId)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*) netCallbackId;
    tEdrvRxBuffer   RxBuffer;
    char            pBuf[MAX_ETH_DATA_SIZE];

    if (memcmp(pPkt->mBlkHdr.mData + pLLHInfo->srcAddrOffset,
               pInstance->m_initParam.m_abMyMacAddr, 6 ) != 0)
    {
        RxBuffer.m_BufferInFrame    = kEdrvBufferLastInFrame;
        RxBuffer.m_uiRxMsgLen = netMblkToBufCopy(pPkt, pBuf, NULL);
        RxBuffer.m_pbBuffer = (unsigned char*)pBuf;

        pInstance->m_initParam.m_pfnRxHandler(&RxBuffer);
        netMblkClChainFree(pPkt);
    }
    else
    {   // self generated traffic
        EPL_DBGLVL_EDRV_TRACE ("%s() self generated traffic!\n", __func__);
    }
    return TRUE;
}


//---------------------------------------------------------------------------
// Function:            EdrvMuxShutdown
//
// Description:         Shutdown callback function of MUX interface
//
// Parameters:
//
// Returns:             STATUS
//---------------------------------------------------------------------------
static STATUS EdrvMuxShutdown (void *pCookie, void *netCallbackId)
{
    EPL_DBGLVL_EDRV_TRACE("EdrvMuxShutdown()\n");
    return OK;
}

//---------------------------------------------------------------------------
// Function:            EdrvMuxRestart
//
// Description:         Restart callback function of MUX interface
//
// Parameters:
//
// Returns:             STATUS
//---------------------------------------------------------------------------
static STATUS EdrvMuxRestart (void *pEnd, void *netCallbackId)
{
    EPL_DBGLVL_EDRV_TRACE("EdrvMuxRestart()\n");
    return OK;
}

//---------------------------------------------------------------------------
// Function:            EdrvMuxError
//
// Description:         Error callback function of MUX interface
//
// Parameters:
//
// Returns:             void
//---------------------------------------------------------------------------
static void EdrvMuxError(END_OBJ *pEnd, END_ERR *pError, void * netCallbackId)
{
    EPL_DBGLVL_EDRV_TRACE("EdrvMuxError()\n");
}

//---------------------------------------------------------------------------
//
// Function:    EdrvTxTask
//
// Description: Transmit callback task.
//
// Parameters:  iArg_p          = Task argument. Contains instance pointer.
//
// Returns:     int
//---------------------------------------------------------------------------
static int EdrvTxTask (int iArg_p)
{
    tEdrvInstance*      pInstance = (tEdrvInstance*) iArg_p;
    STATUS              result;
#if EDRV_USE_DIAGNOSTICS != FALSE
    struct timespec      txLatency;
#endif

    while (TRUE)
    {
        result = semTake(pInstance->m_txWakeupSem, WAIT_FOREVER);
        if (pInstance->m_fStopTxTask)
        {
            break;
        }

#if EDRV_USE_DIAGNOSTICS != FALSE
        hrtimer_clock_gettime(0, &EdrvInstance_l.m_txCbTime);
        txLatency = hrtimer_subTimespec(EdrvInstance_l.m_txCbTime,
                                         EdrvInstance_l.m_txSendTime);
        if (hrtimer_compareTimespec(&txLatency, &EdrvInstance_l.m_maxTxLatency) > 0)
        {
            EdrvInstance_l.m_maxTxLatency = txLatency;
        }
#endif

        if (result == OK)
        {
            if (pInstance->m_pTransmittedTxBufferFirstEntry != NULL)
            {
                tEdrvTxBuffer* pTxBuffer = pInstance->m_pTransmittedTxBufferFirstEntry;

                if (pTxBuffer->m_pbBuffer != NULL)
                {
                    semTake(pInstance->m_mutex, WAIT_FOREVER);
                    pInstance->m_pTransmittedTxBufferFirstEntry =
                        pInstance->m_pTransmittedTxBufferFirstEntry->m_BufferNumber.m_pVal;
                    if (pInstance->m_pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->m_pTransmittedTxBufferLastEntry = NULL;
                    }
                    semGive(pInstance->m_mutex);

                    pTxBuffer->m_BufferNumber.m_pVal = NULL;

                    if (pTxBuffer->m_pfnTxHandler != NULL)
                    {
                        pTxBuffer->m_pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    logMsg("%s() pTxBuffer->m_pbBuffer == NULL\n",
                           (int)__func__, 0, 0, 0, 0, 0);
                }
            }
            else
            {
                logMsg("%s() m_pTransmittedTxBufferFirstEntry == NULL\n",
                       (int)__func__, 0, 0, 0, 0, 0);
            }
        }
    }

    return 0;
}

//---------------------------------------------------------------------------
// Function:            EdrvGetMacAdrs
//
// Description:         get mac address of interface
//
// Parameters:          pCookie = Pointer to MUX interface
//                      ifName = Device name of ethernet interface
//                      macAdrs = Pointer to store MAC address
//
// Returns:             void
//---------------------------------------------------------------------------
static void EdrvGetMacAdrs(PROTO_COOKIE pCookie, char *ifName, BYTE *macAdrs)
{
    char      data[6];

    muxIoctl(pCookie, EIOCGADDR, data);
    EPL_MEMCPY(macAdrs, data, 6);
}


//=========================================================================//
//                                                                         //
//          P U B L I C    F U N C T I O N S                               //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EdrvInit
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//---------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam *pEdrvInitParam_p)
{
    tEplKernel                  Ret;
    NETBUF_CFG                    bufCfgData;
    NETBUF_CL_DESC                clDescTblData;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    if (pEdrvInitParam_p->m_HwParam.m_pszDevName == NULL)
    {
        Ret = kEplEdrvInitError;
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

    if ((EdrvInstance_l.m_dataPoolId = netPoolCreate(&bufCfgData, NULL)) == NULL)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    EPL_DBGLVL_EDRV_TRACE("%s() Using interface %s%d\n", __func__,
                           pEdrvInitParam_p->m_HwParam.m_pszDevName,
                           pEdrvInitParam_p->m_HwParam.m_uiDevNumber);
    /* Binding to Mux Device */
    if ((EdrvInstance_l.m_pCookie =
               muxBind(pEdrvInitParam_p->m_HwParam.m_pszDevName,
                       pEdrvInitParam_p->m_HwParam.m_uiDevNumber,
                       EdrvPacketHandler, EdrvMuxShutdown, EdrvMuxRestart,
                       EdrvMuxError,
                       MUX_PROTO_PROMISC, "POWERLINK", &EdrvInstance_l)) == NULL)
    {
        Ret = kEplEdrvInitError;
        netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);
        goto Exit;
    }

    /* if no MAC address was specified read MAC address of used
     * ethernet interface
     */
    if ((pEdrvInitParam_p->m_abMyMacAddr[0] == 0) &&
        (pEdrvInitParam_p->m_abMyMacAddr[1] == 0) &&
        (pEdrvInitParam_p->m_abMyMacAddr[2] == 0) &&
        (pEdrvInitParam_p->m_abMyMacAddr[3] == 0) &&
        (pEdrvInitParam_p->m_abMyMacAddr[4] == 0) &&
        (pEdrvInitParam_p->m_abMyMacAddr[5] == 0)  )
    {   // read MAC address from controller
        EdrvGetMacAdrs(EdrvInstance_l.m_pCookie,
                       pEdrvInitParam_p->m_HwParam.m_pszDevName,
                       pEdrvInitParam_p->m_abMyMacAddr);
    }

    // save the init data (with updated MAC address)
    EdrvInstance_l.m_initParam = *pEdrvInitParam_p;


    if ((EdrvInstance_l.m_mutex =
                    semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE)) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't init mutex\n", __func__);
        Ret = kEplEdrvInitError;
        muxUnbind(EdrvInstance_l.m_pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)EdrvPacketHandler);
        netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);
        goto Exit;
    }

    if ((EdrvInstance_l.m_syncSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY)) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        Ret = kEplEdrvInitError;
        muxUnbind(EdrvInstance_l.m_pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)EdrvPacketHandler);
        netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(EdrvInstance_l.m_mutex);
        goto Exit;
    }

    if ((EdrvInstance_l.m_txWakeupSem = semCCreate(SEM_Q_FIFO, 0)) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        Ret = kEplEdrvInitError;
        muxUnbind(EdrvInstance_l.m_pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)EdrvPacketHandler);
        netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(EdrvInstance_l.m_mutex);
        semDelete(EdrvInstance_l.m_syncSem);
        goto Exit;
    }

    /* create TX handler task */
    EdrvInstance_l.m_fStopTxTask = FALSE;
    if ((EdrvInstance_l.m_txTaskId = taskSpawn("tTxHandler",
                                             EPL_TASK_PRIORITY_NETTX,
                                             0,
                                             EPL_TASK_STACK_SIZE,
                                             EdrvTxTask,
                                             (int)&EdrvInstance_l,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0))
                                == ERROR)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() Couldn't create TX handler task!\n",
                                 __func__);
        muxUnbind(EdrvInstance_l.m_pCookie, MUX_PROTO_PROMISC,
                  (FUNCPTR)EdrvPacketHandler);
        netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);
        semDelete(EdrvInstance_l.m_mutex);
        semDelete(EdrvInstance_l.m_syncSem);
        goto Exit;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EdrvShutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown( void )
{
    // signal shutdown to the thread
    muxUnbind(EdrvInstance_l.m_pCookie, MUX_PROTO_PROMISC,
              (FUNCPTR)EdrvPacketHandler);

    /* stop TX handler task */
    EdrvInstance_l.m_fStopTxTask = TRUE;
    semGive (EdrvInstance_l.m_txWakeupSem);
    taskDelay (sysClkRateGet() / 10);

    netPoolRelease(EdrvInstance_l.m_dataPoolId, NET_REL_IN_CONTEXT);

    semDelete(EdrvInstance_l.m_mutex);
    semDelete(EdrvInstance_l.m_syncSem);
    semDelete(EdrvInstance_l.m_txWakeupSem);

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    return kEplSuccessful; //assuming no problems with closing the handle
}

//---------------------------------------------------------------------------
// Function:    EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg(tEdrvTxBuffer *pBuffer_p)
{
    tEplKernel          Ret = kEplSuccessful;
    INT                 iRet;
    M_BLK_ID            pPacket;

    if (pBuffer_p->m_BufferNumber.m_pVal != NULL)
    {
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    semTake(EdrvInstance_l.m_mutex, WAIT_FOREVER);
    if (EdrvInstance_l.m_pTransmittedTxBufferLastEntry == NULL)
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry =
            EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry->m_BufferNumber.m_pVal =
                                                                     pBuffer_p;
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    semGive(EdrvInstance_l.m_mutex);

    /* generate packet */
    if ((pPacket = netTupleGet (EdrvInstance_l.m_dataPoolId, MAX_ETH_DATA_SIZE,
                                M_WAIT, MT_HEADER, TRUE)) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Couldn't get tuple!\n", __func__);
        goto Exit;
    }
    pPacket->mBlkHdr.reserved = htons(0x88ab);
    memcpy(pPacket->mBlkHdr.mData, pBuffer_p->m_pbBuffer, pBuffer_p->m_uiTxMsgLen);
    pPacket->mBlkHdr.mLen = pBuffer_p->m_uiTxMsgLen;
    /* send packet out */
    if ((iRet = muxSend(EdrvInstance_l.m_pCookie, pPacket)) != OK)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() muxSend returned %d\n", __func__, iRet);
        /* as muxSend was not successfull we still own the packet and have to
         * free it! */
        netMblkClChainFree(pPacket);
        Ret = kEplInvalidOperation;
    }

#if EDRV_USE_DIAGNOSTICS != FALSE
     hrtimer_clock_gettime(0, &EdrvInstance_l.m_txSendTime);
#endif

    /* wakeup TX Send Task */
    semGive(EdrvInstance_l.m_txWakeupSem);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc
    pBuffer_p->m_pbBuffer = EPL_MALLOC(pBuffer_p->m_uiMaxBufferLen);
    if (pBuffer_p->m_pbBuffer == NULL)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->m_BufferNumber.m_pVal = NULL;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
    BYTE*   pbBuffer = pBuffer_p->m_pbBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->m_pbBuffer = NULL;

    EPL_FREE(pbBuffer);

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
// Function:    EdrvChangeFilter
//
// Description: Change all rx-filters or one specific rx-filter
//              of the openMAC
//
// Parameters:  pFilter_p           = pointer to array of filter entries
//              uiCount_p           = number of filters in array
//              uiEntryChanged_p    = selects one specific filter which is
//                                    to be changed. If value is equal to
//                                    or larger than uiCount_p, all entries
//                                    are selected.
//              uiChangeFlags_p     = If one specific entry is selected,
//                                    these flag bits show which filter
//                                    properties have been changed.
//                                    available flags:
//                                      EDRV_FILTER_CHANGE_MASK
//                                      EDRV_FILTER_CHANGE_VALUE
//                                      EDRV_FILTER_CHANGE_STATE
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE
//                                    if auto-response delay is supported:
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplEdrvInvalidParam
//---------------------------------------------------------------------------
tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p __attribute__((unused)),
                            unsigned int    uiCount_p __attribute__((unused)),
                            unsigned int    uiEntryChanged_p __attribute__((unused)),
                            unsigned int    uiChangeFlags_p __attribute__((unused)))
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
    if (muxMCastAddrDel(EdrvInstance_l.m_pCookie, (char *)pbMacAddr_p) != OK)
    {
        EPL_DBGLVL_EDRV_TRACE("error adding multicast addresses\n");
        return kEplEdrvInitError;
    }
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
    if (muxMCastAddrAdd(EdrvInstance_l.m_pCookie, (char *)pbMacAddr_p) != OK)
    {
        EPL_DBGLVL_EDRV_TRACE("error adding multicast addresses\n");
        return kEplEdrvInitError;
    }
    return kEplSuccessful;
}

#if EDRV_USE_DIAGNOSTICS != FALSE
//---------------------------------------------------------------------------
//
// Function:    EdrvShow
//
// Description: Show Edrv information
//
// Parameters:  NONE
//
// Returns:     void
//---------------------------------------------------------------------------
void EdrvShow(void)
{
    printf ("Max Tx Callback Latency: %ld ns\n",
            EdrvInstance_l.m_maxTxLatency.tv_nsec);
}
#endif
