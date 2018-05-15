/**
********************************************************************************
\file   sdoseq.c

\brief  Implementation of SDO Sequence Layer

This file contains the implementation of the SDO Sequence Layer

\ingroup module_sdo_seq
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/sdoseq.h>
#include <user/sdoal.h>
#include <user/sdoasnd.h>
#include <user/sdoudp.h>
#include <user/timeru.h>
#include <common/ami.h>

#if (!defined(CONFIG_INCLUDE_SDO_UDP) && !defined(CONFIG_INCLUDE_SDO_ASND))
#error "ERROR: sdoseq.c - At least UDP or ASND module needed!"
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SDO_HISTORY_SIZE                5

#ifndef CONFIG_SDO_MAX_CONNECTION_SEQ
#define CONFIG_SDO_MAX_CONNECTION_SEQ   5
#endif

#define SDO_SEQ_RETRY_COUNT             2                       // number of ack requests before close (final timeout)
#define SDO_SEQ_CMDL_INACTIVE_THLD      2                       // number of seq. layer sub timeouts before close if command layer is not active
#define SDO_SEQ_NUM_THRESHOLD           100                     // threshold which distinguishes between old and new sequence numbers
#define SDO_SEQ_FRAME_SIZE              24                      // frame with size of Asnd-Header-, SDO Sequence header size, SDO Command header and Ethernet-header size
#define SDO_SEQ_HEADER_SIZE             4                       // size of the header of the SDO Sequence layer
#define SDO_SEQ_TX_HISTORY_FRAME_SIZE   SDO_MAX_TX_FRAME_SIZE   // buffersize for one frame in history
#define SDO_CON_MASK                    0x03                    // mask to get scon and rcon

#define SEQ_NUM_MASK                    0xFC

static const UINT32 SDO_SEQU_MAX_TIMEOUT_MS = 86400000UL;       // [ms], 86400000 ms = 1 day

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
\brief  SDO sequence layer events

This enumeration defines all valid event which could be processed by the
SDO sequence layer.
*/
typedef enum
{
    kSdoSeqEventNoEvent   = 0x00,       ///< No Event
    kSdoSeqEventInitCon   = 0x01,       ///< Init connection
    kSdoSeqEventFrameRec  = 0x02,       ///< Frame received
    kSdoSeqEventFrameSend = 0x03,       ///< Frame to send
    kSdoSeqEventTimeout   = 0x04,       ///< Timeout for connection
    kSdoSeqEventCloseCon  = 0x05        ///< Higher layer closed connection
} eSdoSeqEvent;

/**
\brief SDO sequence layer event data type

Data type for the enumerator \ref eSdoSeqEvent.
*/
typedef UINT32 tSdoSeqEvent;

/**
\brief  SDO sequence layer connection history

This structure defines the SDO sequence layer connection history buffer.
*/
typedef struct
{
    UINT8   freeEntries;    ///< Number of free history entries
    UINT8   writeIndex;     ///< Index of the next free buffer entry
    UINT8   ackIndex;       ///< Index of the next message which should become acknowledged
    UINT8   readIndex;      ///< Index between ackIndex and writeIndex to the next message for retransmission
    UINT8   aHistoryFrame[SDO_HISTORY_SIZE][SDO_SEQ_TX_HISTORY_FRAME_SIZE];    ///< Array of the history frames
    size_t  aFrameSize[SDO_HISTORY_SIZE];           ///< Array of sizes of the history frames
    BOOL    afFrameFirstTxFailed[SDO_HISTORY_SIZE]; ///< Array of flags tagging frame as unsent
                                                    /**< Array of flags indicating that the first attempt to
                                                         forward a frame to a lower layer send function failed
                                                         due to buffer overflow e.g. and should be repeated later */
} tSdoSeqConHistory;

/**
\brief  SDO sequence layer states

This enumeration defines all valid states for the SDO sequence layer state
machine.
*/
typedef enum
{
    kSdoSeqStateIdle        = 0x00, ///< SDO connection is idle (closed)
    kSdoSeqStateInit1       = 0x01, ///< SDO init 1: scon=1, rcon=0
    kSdoSeqStateInit2       = 0x02, ///< SDO init 2: scon=1, rcon=1
    kSdoSeqStateInit3       = 0x03, ///< SDO init 3: scon=2, rcon=1
    kSdoSeqStateConnected   = 0x04, ///< SDO connection is established
    kSdoSeqStateWaitAck     = 0x05, ///< SDO connection is waiting for an acknowledgment
} eSdoSeqState;

/**
\brief SDO sequence layer state data type

Data type for the enumerator \ref eSdoSeqState.
*/
typedef UINT32 tSdoSeqState;

/**
\brief  SDO sequence layer connection control structure

This structure is used by the SDO sequence layer to save connection information.
*/
typedef struct
{
    tSdoConHdl              conHandle;              ///< Connection handle
    tSdoSeqState            sdoSeqState;            ///< State of the connection
    UINT8                   recvSeqNum;             ///< Receive sequence number
                                                    /**< Expected receive sequence number (acknowledge) of other node,
                                                         updated on every segment Tx with new command data layer data */
    UINT8                   sendSeqNum;             ///< Send sequence number of communication partner, updated on Rx
    tSdoSeqConHistory       sdoSeqConHistory;       ///< Connection history buffer
    tTimerHdl               timerHandle;            ///< Timer handle
    UINT                    retryCount;             ///< Retry counter
    UINT                    useCount;               ///< One sequence layer connection may be used by multiple command layer connections
    BOOL                    fForceFlowControl;      ///< If enabled, Rx sequences will not be forwarded to command layer
    UINT                    countCmdLayerInactive;  ///< Counter of an inactive command layer using timeout events
} tSdoSeqCon;

/**
\brief  SDO sequence layer instance structure

This structure defines a SDO sequence layer instance.
*/
typedef struct
{
    tSdoSeqCon              aSdoSeqCon[CONFIG_SDO_MAX_CONNECTION_SEQ];  ///< Array of sequence layer connections
    tSdoComReceiveCb        pfnSdoComRecvCb;                            ///< Pointer to receive callback function
    tSdoComConCb            pfnSdoComConCb;                             ///< Pointer to connection callback function
    UINT32                  sdoSeqTimeout;                              ///< Configured Sequence layer sub-timeout

#if (defined(WIN32) || defined(_WIN32))
    LPCRITICAL_SECTION      pCriticalSection;
    CRITICAL_SECTION        criticalSection;

    LPCRITICAL_SECTION      pCriticalSectionReceive;
    CRITICAL_SECTION        criticalSectionReceive;
#endif
} tSdoSeqInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoSeqInstance   sdoSeqInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processState(UINT handle_p,
                               size_t dataSize_p,
                               tPlkFrame* pData_p,
                               const tAsySdoSeq* pRecvFrame_p,
                               tSdoSeqEvent event_p);
static tOplkError processStateIdle(tSdoSeqCon* pSdoSeqCon_p,
                                   tSdoSeqConHdl sdoSeqConHdl_p,
                                   tSdoSeqEvent event_p,
                                   const tAsySdoSeq* pRecvFrame_p);
static tOplkError processStateInit1(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p);
static tOplkError processStateInit2(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p);
static tOplkError processStateInit3(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p);
static tOplkError processStateConnected(tSdoSeqCon* pSdoSeqCon_p,
                                        tSdoSeqConHdl sdoSeqConHdl_p,
                                        tSdoSeqEvent event_p,
                                        const tAsySdoSeq* pRecvFrame_p,
                                        size_t dataSize_p,
                                        tPlkFrame* pData_p);
static tOplkError processStateWaitAck(tSdoSeqCon* pSdoSeqCon_p,
                                      tSdoSeqConHdl sdoSeqConHdl_p,
                                      tSdoSeqEvent event_p,
                                      const tAsySdoSeq* pRecvFrame_p,
                                      size_t dataSize_p);
static tOplkError sendFrame(tSdoSeqCon* pSdoSeqCon_p,
                            size_t dataSize_p,
                            tPlkFrame* pData_p,
                            BOOL fFrameInHistory_p);
static tOplkError sendToLowerLayer(const tSdoSeqCon* pSdoSeqCon_p,
                                   size_t dataSize_p,
                                   tPlkFrame* pFrame_p);
static tOplkError receiveCb(tSdoConHdl conHdl_p,
                            const tAsySdoSeq* pSdoSeqData_p,
                            size_t dataSize_p);
static tOplkError initHistory(tSdoSeqCon* pSdoSeqCon_p);
static tOplkError addFrameToHistory(tSdoSeqCon* pSdoSeqCon_p,
                                    const tPlkFrame* pFrame_p,
                                    size_t size_p,
                                    BOOL fTxFailed_p);
static tOplkError sendAllTxHistory(tSdoSeqCon* pSdoSeqCon_p);
static tOplkError deleteAckedFrameFromHistory(tSdoSeqCon* pSdoSeqCon_p,
                                              UINT8 recvSeqNumber_p);
static tOplkError readFromHistory(tSdoSeqCon* pSdoSeqCon_p,
                                  tPlkFrame** ppFrame_p,
                                  size_t* pSize_p,
                                  BOOL fInitRead_p);
static UINT8      getFreeHistoryEntries(const tSdoSeqCon* pSdoSeqCon_p);
static tOplkError setTimer(tSdoSeqCon* pSdoSeqCon_p, ULONG timeout_p);
static void       processFinalTimeout(tSdoSeqCon* pSdoSeqCon_p,
                                      tSdoSeqConHdl sdoSeqConHdl_p);
static tOplkError processSubTimeout(tSdoSeqCon* pSdoSeqCon_p);
static tOplkError processTimeoutEvent(tSdoSeqCon* pSdoSeqCon_p,
                                      tSdoSeqConHdl sdoSeqConHdl_p);
static tOplkError deleteLowLayerConnection(tSdoSeqCon* pSdoSeqCon_p);
static BOOL       checkHistoryAcked(const tSdoSeqCon* pSdoSeqCon_p,
                                    UINT8 recvSeqNumber_p);
static BOOL       checkConnectionAckValid(const tSdoSeqCon* pSdoSeqCon_p,
                                          UINT8 recvSeqNumber_p);
static tOplkError sendHistoryOldestSegm(tSdoSeqCon* pSdoSeqCon_p,
                                        UINT8 recvSeqNumber_p);
static void       forceRetransmissionRequest(tSdoSeqCon* pSdoSeqCon_p,
                                             BOOL fEnable_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO sequence layer

The function initializes the SDO sequence layer

\param[in]      pfnSdoComRecvCb_p   Pointer to callback function that informs command
                                    layer about new frames.
\param[in]      pfnSdoComConCb_p    Pointer to callback function that informs command
                                    layer about connection state.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_init(tSdoComReceiveCb pfnSdoComRecvCb_p,
                       tSdoComConCb pfnSdoComConCb_p)
{
    tOplkError  ret = kErrorOk;

    if (pfnSdoComRecvCb_p == NULL)
        return kErrorSdoSeqMissCb;
    else
        sdoSeqInstance_l.pfnSdoComRecvCb = pfnSdoComRecvCb_p;

    if (pfnSdoComConCb_p == NULL)
        return kErrorSdoSeqMissCb;
    else
        sdoSeqInstance_l.pfnSdoComConCb = pfnSdoComConCb_p;

    OPLK_MEMSET(&sdoSeqInstance_l.aSdoSeqCon[0], 0x00, sizeof(sdoSeqInstance_l.aSdoSeqCon));

#if (defined(WIN32) || defined(_WIN32))
    // create critical section for process function
    sdoSeqInstance_l.pCriticalSection = &sdoSeqInstance_l.criticalSection;
    InitializeCriticalSection(sdoSeqInstance_l.pCriticalSection);

    // create critical section for receive cb function
    sdoSeqInstance_l.pCriticalSectionReceive = &sdoSeqInstance_l.criticalSectionReceive;
    InitializeCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

    // init lower layers
#if defined(CONFIG_INCLUDE_SDO_UDP)
    ret = sdoudp_init(receiveCb);
    if (ret != kErrorOk)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_SDO_ASND)
    ret = sdoasnd_init(receiveCb);
    if (ret != kErrorOk)
        return ret;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down SDO sequence layer

The function shuts down the SDO sequence layer.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_exit(void)
{
    tOplkError  ret = kErrorOk;
    UINT        count;
    tSdoSeqCon* pSdoSeqCon;

    // delete timer of open connections
    count = 0;
    pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[0];
    while (count < CONFIG_SDO_MAX_CONNECTION_SEQ)
    {
        if (pSdoSeqCon->conHandle != 0)
            timeru_deleteTimer(&pSdoSeqCon->timerHandle);

        count++;
        pSdoSeqCon++;
    }

#if (defined(WIN32) || defined(_WIN32))
    // delete critical section for process function
    DeleteCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif
    OPLK_MEMSET(&sdoSeqInstance_l, 0x00, sizeof(sdoSeqInstance_l));

#if defined(CONFIG_INCLUDE_SDO_UDP)
    ret = sdoudp_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("sdoudp_exit():  0x%X\n", ret);
    }
#endif
#if defined(CONFIG_INCLUDE_SDO_ASND)
    ret = sdoasnd_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("sdoasnd_exit(): 0x%X\n", ret);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a SDO sequence layer connection

The function initializes a SDO sequence layer connection. It tries to reuse an
existing connection to the specified node.

\param[out]     pSdoSeqConHdl_p     Pointer to store the sequence layer connection
                                    handle.
\param[in]      nodeId_p            Node ID of the target to connect to.
\param[in]      sdoType_p           Type of the SDO connection.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_initCon(tSdoSeqConHdl* pSdoSeqConHdl_p,
                          UINT nodeId_p,
                          tSdoType sdoType_p)
{
    tOplkError  ret = kErrorOk;
    UINT        count;
    UINT        freeCon;
    tSdoConHdl  conHandle = (tSdoConHdl)~0U;
    tSdoSeqCon* pSdoSeqCon;

    // Check parameter validity
    ASSERT(pSdoSeqConHdl_p != NULL);

    // check SdoType
    // call init function of the protocol abstraction layer
    // which tries to find an existing connection to the same node
    switch (sdoType_p)
    {
        case kSdoTypeUdp:
#if defined(CONFIG_INCLUDE_SDO_UDP)
            ret = sdoudp_initCon(&conHandle, nodeId_p);
            if (ret != kErrorOk)
                return ret;
            break;
#else
            return kErrorSdoSeqUnsupportedProt;
#endif

        case kSdoTypeAsnd:
#if defined(CONFIG_INCLUDE_SDO_ASND)
            ret = sdoasnd_initCon(&conHandle, nodeId_p);
            if (ret != kErrorOk)
                return ret;
#else
            return kErrorSdoSeqUnsupportedProt;
#endif
            break;

        // unsupported protocols -> auto should be replaced by command layer
        case kSdoTypeAuto:
        case kSdoTypePdo:
        default:
            return kErrorSdoSeqUnsupportedProt;
    }

    // find existing connection to the same node or find empty entry for connection
    count = 0;
    freeCon = CONFIG_SDO_MAX_CONNECTION_SEQ;
    pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[0];

    while (count < CONFIG_SDO_MAX_CONNECTION_SEQ)
    {
        if (pSdoSeqCon->conHandle == conHandle)
            break;

        if (pSdoSeqCon->conHandle == 0)
            freeCon = count;

        count++;
        pSdoSeqCon++;
    }

    if (count == CONFIG_SDO_MAX_CONNECTION_SEQ)
    {
        if (freeCon == CONFIG_SDO_MAX_CONNECTION_SEQ)
        {   // no free entry found
            switch (sdoType_p)
            {
                case kSdoTypeUdp:
#if defined(CONFIG_INCLUDE_SDO_UDP)
                    ret = sdoudp_delConnection(conHandle);
                    if (ret != kErrorOk)
                        return ret;
#endif
                    break;

                case kSdoTypeAsnd:
#if defined(CONFIG_INCLUDE_SDO_ASND)
                    ret = sdoasnd_deleteCon(conHandle);
                    if (ret != kErrorOk)
                        return ret;
#endif
                    break;

                // unsupported protocols -> auto should be replaced by command layer
                case kSdoTypeAuto:
                case kSdoTypePdo:
                default:
                    return kErrorSdoSeqUnsupportedProt;
            }
            return kErrorSdoSeqNoFreeHandle;
        }
        else
        {   // free entry found
            pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[freeCon];
            pSdoSeqCon->conHandle = conHandle;
            pSdoSeqCon->useCount++;     // increment use counter
            count = freeCon;
        }
    }

    *pSdoSeqConHdl_p = (tSdoSeqConHdl)(count | SDO_ASY_HANDLE); // set handle

    ret = processState(count, 0, NULL, NULL, kSdoSeqEventInitCon);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send data via a sequence layer connection

The function sends data via an existing sequence layer connection.

\param[in]      sdoSeqConHdl_p      Sequence layer connection handle to use for
                                    transfer.
\param[in]      dataSize_p          Size of sequence layer frame (without higher
                                    layer headers)
\param[in,out]  pData_p             Pointer to the data to send.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_sendData(tSdoSeqConHdl sdoSeqConHdl_p,
                           size_t dataSize_p,
                           tPlkFrame* pData_p)
{
    tOplkError  ret;
    UINT        handle;

    handle = ((UINT)sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

    // check if connection ready
    if (sdoSeqInstance_l.aSdoSeqCon[handle].sdoSeqState == kSdoSeqStateIdle)
    {
        // no connection with this handle
        return kErrorSdoSeqInvalidHdl;
    }
    else
    {
        if (sdoSeqInstance_l.aSdoSeqCon[handle].sdoSeqState != kSdoSeqStateConnected)
            return kErrorSdoSeqConnectionBusy;
    }

    // calling send function from application counts as reset of flow control
    forceRetransmissionRequest(&sdoSeqInstance_l.aSdoSeqCon[handle], FALSE);

    ret = processState(handle, dataSize_p, pData_p, NULL, kSdoSeqEventFrameSend);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO event

The function processes SDO events.

\param[in]      pEvent_p            Event to process.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_processEvent(const tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tTimerEventArg*     pTimerEventArg;
    tSdoSeqCon*         pSdoSeqCon;
    tTimerHdl           timerHdl;
    UINT                count;

    if (pEvent_p == NULL)
        return kErrorSdoSeqInvalidEvent;

    if (pEvent_p->eventType != kEventTypeTimer)
        return kErrorSdoSeqInvalidEvent;

    // get timer handle
    pTimerEventArg = (tTimerEventArg*)pEvent_p->eventArg.pEventArg;
    timerHdl = pTimerEventArg->timerHdl.handle;
    // get pointer to intern control structure of connection
    if (pTimerEventArg->argument.pValue == NULL)
        return ret;

    pSdoSeqCon = (tSdoSeqCon*)pTimerEventArg->argument.pValue;
    // check if time is current
    if (timerHdl != pSdoSeqCon->timerHandle)
    {
        timeru_deleteTimer(&timerHdl);
        return ret;
    }
    timeru_deleteTimer(&pSdoSeqCon->timerHandle);

    // get index number of control structure
    count = 0;
    while (&sdoSeqInstance_l.aSdoSeqCon[count] != pSdoSeqCon)
    {
        count++;
        if (count > CONFIG_SDO_MAX_CONNECTION_SEQ)
            return ret;
    }

    // process event and call process function if needed
    ret = processState(count, 0, NULL, NULL, kSdoSeqEventTimeout);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a sequence layer connection

The function closes and deletes an existing sequence layer connection.

\param[in]      sdoSeqConHdl_p      Connection handle of sequence layer connection
                                    to delete.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_deleteCon(tSdoSeqConHdl sdoSeqConHdl_p)
{
    tOplkError      ret = kErrorOk;
    UINT            handle;
    tSdoSeqCon*     pSdoSeqCon;

    handle = ((UINT)sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

    // check if handle invalid
    if (handle >= CONFIG_SDO_MAX_CONNECTION_SEQ)
        return kErrorSdoSeqInvalidHdl;

    pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[handle];    // get pointer to connection

    // Check if connection is already closed
    if (pSdoSeqCon->useCount == 0)
        return ret;

    pSdoSeqCon->useCount--;
    if (pSdoSeqCon->useCount == 0)
    {
        // process close in process function
        ret = processState(handle, 0, NULL, NULL, kSdoSeqEventCloseCon);

        deleteLowLayerConnection(pSdoSeqCon);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set sequence layer timeout

The function sets the sequence layer timeout.

\param[in]      timeout_p           Timeout to set in milliseconds.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_setTimeout(UINT32 timeout_p)
{
    // Adopt new SDO sequence layer timeout (truncated to an upper bound)
    sdoSeqInstance_l.sdoSeqTimeout = min(timeout_p, SDO_SEQU_MAX_TIMEOUT_MS) /
                                         (SDO_SEQ_RETRY_COUNT + 1);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process Idle state

The function processes the sequence layer state: kSdoSeqStateIdle
Server: Listens for incoming connection requests (InitReq)
Client: Sends a connection request (InitReq)

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateIdle(tSdoSeqCon* pSdoSeqCon_p,
                                   tSdoSeqConHdl sdoSeqConHdl_p,
                                   tSdoSeqEvent event_p,
                                   const tAsySdoSeq* pRecvFrame_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(sdoSeqConHdl_p);

    switch (event_p)
    {
        // new connection -> send init frame and change to kSdoSeqStateInit1
        case kSdoSeqEventInitCon:
            pSdoSeqCon_p->recvSeqNum = 0x01;    // set sending scon to 1
            pSdoSeqCon_p->sendSeqNum = 0x00;    // set set send rcon to 0
            ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
            if (ret != kErrorOk)
                return ret;

            forceRetransmissionRequest(pSdoSeqCon_p, FALSE);
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit1; // change state
            ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            break;

        // init con from extern, check rcon and scon -> send answer (InitAck)
        case kSdoSeqEventFrameRec:
            if (pRecvFrame_p == NULL)
                return kErrorInvalidOperation;

            DEBUG_LVL_SDO_TRACE("%s scon=%u rcon=%u\n",
                                __func__,
                                pRecvFrame_p->sendSeqNumCon,
                                pRecvFrame_p->recvSeqNumCon);

            // check if scon == 1 and rcon == 0
            if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x00) &&
                ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {
                // save sequence numbers
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                // create answer and send answer, set rcon to 1 (in send direction own scon)
                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return kErrorOk;

                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit2; // change state to kSdoSeqStateInit2
                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            }
            else
            {   // otherwise, this is not an init from extern, therefore silently ignore it
                // and just delete the connection internally
                deleteLowLayerConnection(pSdoSeqCon_p);
            }

            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state Init1

The function processes the sequence layer state: kSdoSeqStateInit1
Client: Listens for init request confirmation from server (InitAck)
        and confirms with an init response frame (InitResp)
Server: Changes role from client to server, if already in client mode
        but receiving an initialization (InitReq) request from another client

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit1(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p)
{
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_SDO_TRACE("sdoseq: %s()\n", __func__);

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
            if (pRecvFrame_p == NULL)
                return kErrorInvalidOperation;

            // check scon == 1 and rcon == 1
            if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x01) &&
                ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {   // create answer own scon = 2 - save sequence numbers
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);

                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;

                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit3;
                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            }
            // check if scon == 1 and rcon == 0, i.e. other side wants me to be server
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x00) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {
                // save sequence numbers
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                // create answer and send answer - set rcon to 1 (in send direction own scon)
                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;

                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit2;
                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            }
            else
            {   // error -> Close
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;

                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);

                if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) != 0x00) ||
                    ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) != 0x00))
                {   // d.k. only answer with close message if the message sent was not a close message
                    // save sequence numbers
                    pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                    pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                    // set rcon and scon to 0
                    pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
                    pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

                    sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                }

                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            }
            break;

        // timeout
        case kSdoSeqEventTimeout:
            // error -> Close
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;

            // set rcon and scon to 0
            pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
            pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

            sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            break;

        default:
            // d.k. do nothing
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state Init2

The function processes the sequence layer state: kSdoSeqStateInit2
Server: Listens for init ack confirmation from client (InitResp)
        and announces a valid connection (Valid)
Client: Changes role from server to client, if already in server mode, but
        receiving an initialization confirmation (InitAck) from another server

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit2(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p)
{
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_SDO_TRACE("sdoseq: %s()\n", __func__);

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
            if (pRecvFrame_p == NULL)
                return kErrorInvalidOperation;

            // check scon == 2 and rcon == 1
            if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x01) &&
                ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x02))
            {   // create answer own rcon = 2 - save sequence numbers
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);

                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;

                // change state to kSdoSeqStateConnected
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;
                ret = initHistory(pSdoSeqCon_p);
                if (ret != kErrorOk)
                    return ret;

                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateConnected);
            }
            // check scon == 1 and rcon == 1, i.e. other side wants me to initiate the connection
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x01) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                // create answer and send answer - set rcon to 1 (in send direction own scon)
                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;

                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit3;
            }
            // check scon == 1 and rcon == 0 (InitReq from client)
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x00) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;   // return to idle
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTransferAbort);
                // restart immediately with initialization request
                DEBUG_LVL_SDO_TRACE("sdoseq: Reinit immediately\n");
                ret = kErrorRetry;
                break;
            }
            else
            {   // error -> Close
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);

                if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) != 0x00) ||
                    ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) != 0x00))
                {   // d.k. only answer with close message if the message sent was not a close message
                    // save sequence numbers
                    pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                    pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                    // set rcon and scon to 0
                    pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
                    pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

                    sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                }

                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            }
            break;

        // timeout
        case kSdoSeqEventTimeout:
            // error -> Close
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
            // set rcon and scon to 0
            pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
            pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;
            sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            break;

        default:
            break;
    } // end of switch (event_p)

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state Init3

The function processes the sequence layer state: kSdoSeqStateInit3
Client: Listens for valid connection announcement from server (Valid)
        and confirms with a valid connection frame (Valid)
Server: Never reaches this state

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit3(tSdoSeqCon* pSdoSeqCon_p,
                                    tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p,
                                    const tAsySdoSeq* pRecvFrame_p)
{
    tOplkError  ret = kErrorOk;

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
            if (pRecvFrame_p == NULL)
                return kErrorInvalidOperation;

            // check scon == 2 and rcon == 2
            if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x02) &&
                ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x02))
            {
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;

                ret = initHistory(pSdoSeqCon_p);
                if (ret != kErrorOk)
                    return ret;

                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateConnected);
            }
            // check scon == 1 and rcon == 1
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x01) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {   // server repeated an InitAck frame
                // allow this until timeout -> do nothing and wait for correct frame
                break;
            }
            // check scon == 2 and rcon == 1
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x01) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x02))
            {   // create answer own rcon = 2 - save sequence numbers
                pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);

                pSdoSeqCon_p->recvSeqNum++;
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;

                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;
                ret = initHistory(pSdoSeqCon_p);
                if (ret != kErrorOk)
                    return ret;

                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateConnected);
            }
            // check scon == 1 and rcon == 0 (InitReq from client)
            else if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) == 0x00) &&
                     ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) == 0x01))
            {
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;   // return to idle
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTransferAbort);
                // restart immediately with initialization request
                DEBUG_LVL_SDO_TRACE("sdoseq: Reinit immediately\n");
                ret = kErrorRetry;
                break;
            }
            else
            {   // error -> Close
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;

                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) != 0x00) ||
                    ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) != 0x00))
                {   // d.k. only answer with close message if the message sent was not a close message
                    // save sequence numbers
                    pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                    pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                    // set rcon and scon to 0
                    pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
                    pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

                    sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                }

                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            }
            break;

        // timeout
        case kSdoSeqEventTimeout:
            // error -> Close
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
            // set rcon and scon to 0
            pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
            pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

            sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateInitError);
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state Connected

The function processes the sequence layer state: kSdoSeqStateConnected

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame (can be NULL).
\param[in]      dataSize_p          Size of sequence layer frame (without higher layer
                                    headers).
\param[in,out]  pData_p             Pointer to frame to be sent (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateConnected(tSdoSeqCon* pSdoSeqCon_p,
                                        tSdoSeqConHdl sdoSeqConHdl_p,
                                        tSdoSeqEvent event_p,
                                        const tAsySdoSeq* pRecvFrame_p,
                                        size_t dataSize_p,
                                        tPlkFrame* pData_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       sendSeqNumCon;
    UINT8       recvSeqNumCon;

    switch (event_p)
    {
        // frame to send
        case kSdoSeqEventFrameSend:
            pSdoSeqCon_p->countCmdLayerInactive = 0;

            // set timer
            ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            if (ret != kErrorOk)
                return ret;

            // check if data frame or ack
            if (pData_p == NULL)
            {   // send ack, increment scon
                ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                if (ret != kErrorOk)
                    return ret;
            }
            else
            {   // send dataframe, increment send sequence number
                pSdoSeqCon_p->recvSeqNum += 4;
                ret = sendFrame(pSdoSeqCon_p, dataSize_p, pData_p, TRUE);
                if (ret == kErrorSdoSeqRequestAckNeeded)
                {
                    // successful, but Tx history buffer is reaching its limits
                    // request ack, change state to wait ack
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateWaitAck;

                    ret = sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameSent);
                    if (ret == kErrorSdoComHandleBusy)
                    {
                        ret = kErrorOk;
                        forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                    }
                    else
                        forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                }
                else if (ret != kErrorOk)
                    return ret;
                else
                {
                    pSdoSeqCon_p->countCmdLayerInactive = 0;
                    ret = sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameSent);
                    if (ret == kErrorSdoComHandleBusy)
                    {
                        ret = kErrorOk;
                        forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                    }
                    else
                        forceRetransmissionRequest(pSdoSeqCon_p, FALSE);
                }
            }
            break;

        // frame received
        case kSdoSeqEventFrameRec:
            if (pRecvFrame_p == NULL)
                return kErrorInvalidOperation;

            sendSeqNumCon = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
            recvSeqNumCon = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);

            switch (sendSeqNumCon & SDO_CON_MASK)
            {
                // close from other node
                case 0:
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;   // return to idle
                    timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                    sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateConClosed);
                    break;

                // init
                case 1:
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;   // return to idle
                    timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                    sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTransferAbort);
                    // restart immediately with initialization request
                    DEBUG_LVL_SDO_TRACE("sdoseq: Reinit immediately\n");
                    ret = kErrorRetry;
                    break;

                // acknowledge request (signaling lost ack) - possibly contains data
                case 3:
                // connection valid
                case 2:
                    if (checkConnectionAckValid(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK) ||
                        checkHistoryAcked(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK))
                    {
                        ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                        if (ret != kErrorOk)
                            return ret;

                        // reset timeout counter
                        pSdoSeqCon_p->retryCount = 0;
                    }

                    deleteAckedFrameFromHistory(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK);

                    if ((recvSeqNumCon & SDO_CON_MASK) == 3)
                    {   // retransmission request (signaling lost segments)

                        // TRACE("sdoseq: error response received\n");
                        ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                        if (ret != kErrorOk)
                            return ret;

                        // reset timeout counter
                        pSdoSeqCon_p->retryCount = 0;
                        ret = sendAllTxHistory(pSdoSeqCon_p);
                        if (ret != kErrorOk)
                            return ret;
                    }

                    // trigger segmented Tx before timeout does (speed-up transmission)
                    ret = sendHistoryOldestSegm(pSdoSeqCon_p, recvSeqNumCon);
                    if (ret != kErrorOk)
                        return ret;

                    if (((pSdoSeqCon_p->sendSeqNum + 4) & SEQ_NUM_MASK) == (sendSeqNumCon & SEQ_NUM_MASK))
                    {   // next frame of sequence received (new command layer data)
                        pSdoSeqCon_p->countCmdLayerInactive = 0;

                        if (!pSdoSeqCon_p->fForceFlowControl)
                        {
                            // save send sequence number (without ack request)
                            pSdoSeqCon_p->sendSeqNum = sendSeqNumCon & ~0x01;
                            // forward Rx frame to command layer
                            ret = sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                (dataSize_p - SDO_SEQ_HEADER_SIZE));
                            if (ret == kErrorSdoComHandleBusy)
                            {
                                ret = kErrorOk;
                                forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                            }
                            else
                                forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameReceived);
                        }
                    }
                    else if (((sendSeqNumCon - pSdoSeqCon_p->sendSeqNum - 4) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
                    {   // frame of sequence was lost,
                        // because difference of received and old value
                        // is less then halve of the values range.
                        // send error frame with own rcon = 3 (error response)
                        pSdoSeqCon_p->sendSeqNum |= 0x03;
                        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                        // restore rcon = 2
                        pSdoSeqCon_p->sendSeqNum--;
                        if (ret != kErrorOk)
                            return ret;

                        // break here, because a requested acknowledge was sent implicitly above
                        break;
                    }

                    // else, ignore repeated frame
                    if ((sendSeqNumCon & SDO_CON_MASK) == 3)
                    {   // ack request received
                        if (pSdoSeqCon_p->fForceFlowControl)
                        {   // other node is about to close the connection
                            // -> reset timer of other node with
                            // retransmission request
                            pSdoSeqCon_p->sendSeqNum |= 0x03;
                        }
                        else
                        {   // create ack with own scon = 2
                            pSdoSeqCon_p->sendSeqNum |= 0x03;
                            pSdoSeqCon_p->sendSeqNum--;
                        }
                        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                        if (ret != kErrorOk)
                            return ret;
                    }
                    break;
            }
            break;

        //close event from higher layer
        case kSdoSeqEventCloseCon:
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
            // set rcon and scon to 0
            pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
            pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;
            sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

            break;

        case kSdoSeqEventTimeout:
            processTimeoutEvent(pSdoSeqCon_p, sdoSeqConHdl_p);
            if (ret != kErrorOk)
                return ret;
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state WaitAck

The function processes the sequence layer state: kSdoSeqStateWaitAck

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
\param[in]      event_p             Event to be processed.
\param[in]      pRecvFrame_p        Pointer to received frame.
\param[in]      dataSize_p          Size of sequence layer frame (without higher layer
                                    headers).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateWaitAck(tSdoSeqCon* pSdoSeqCon_p,
                                      tSdoSeqConHdl sdoSeqConHdl_p,
                                      tSdoSeqEvent event_p,
                                      const tAsySdoSeq* pRecvFrame_p,
                                      size_t dataSize_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       sendSeqNumCon;
    UINT8       recvSeqNumCon;

    DEBUG_LVL_SDO_TRACE("sdoseq: %s()\n", __func__);

    // set scon = 3 -> acknowledge request
    pSdoSeqCon_p->recvSeqNum |= 0x03;

    //TODO: retry of acknowledge
    if (event_p == kSdoSeqEventFrameRec)
    {
        if (pRecvFrame_p == NULL)
            return kErrorInvalidOperation;

        sendSeqNumCon = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
        recvSeqNumCon = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);

        if ((sendSeqNumCon & SDO_CON_MASK) == 1)
        {   // reinit from other node -> return to idle
            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
            timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTransferAbort);
            // restart immediately with initialization request
            return kErrorRetry;
        }

        // check rcon
        switch (pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK)
        {
            // close from other node
            case 0:
                // return to idle
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateConClosed);
                break;

            // normal frame
            case 2:
                if (checkHistoryAcked(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK))
                {   // we came here only due to a full history buffer
                    // and one element is now acknowledged

                    ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
                    if (ret != kErrorOk)
                        return ret;

                    // reset timeout counter
                    pSdoSeqCon_p->retryCount = 0;

                    deleteAckedFrameFromHistory(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK);
                    // reset own scon to 2 (valid connection)
                    pSdoSeqCon_p->recvSeqNum--;

                    // change state
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;
                    ret = sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateAckReceived);
                    if (ret == kErrorSdoComHandleBusy)
                    {
                        ret = kErrorOk;
                        forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                    }
                    else
                        forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                    if (ret != kErrorOk)
                        return ret;
                }

                // send data to higher layer if needed
                if (((pSdoSeqCon_p->sendSeqNum + 4) & SEQ_NUM_MASK) == (sendSeqNumCon & SEQ_NUM_MASK))
                {   // next frame of sequence received (new command layer data)
                    pSdoSeqCon_p->countCmdLayerInactive = 0;

                    if (!pSdoSeqCon_p->fForceFlowControl)
                    {
                        // save send sequence number (without ack request)
                        pSdoSeqCon_p->sendSeqNum = sendSeqNumCon & ~0x01;

                        // forward Rx frame to command layer
                        ret = sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                               ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                               (dataSize_p - SDO_SEQ_HEADER_SIZE));
                        if (ret == kErrorSdoComHandleBusy)
                        {
                            ret = kErrorOk;
                            forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                        }
                        else
                            forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                        // Note: Posting "kAsySdoConStateFrameReceived" to higher layer not necessary
                        //       here, because if we are server it is an initial transfer and will
                        //       not be processed. If we are client, it doesn't happen since the other
                        //       node would need to be a client also.

                        if (ret != kErrorOk)
                            return ret;
                    }
                }

                // trigger segmented Tx before timeout does (speed-up transmission)
                ret = sendHistoryOldestSegm(pSdoSeqCon_p, recvSeqNumCon);
                if (ret != kErrorOk)
                    return ret;
                break;

            // retransmission request (error response)
            case 3:
                if (checkHistoryAcked(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK))
                {   // we came here only due to a full history buffer
                    // and one element is now acknowledged
                    deleteAckedFrameFromHistory(pSdoSeqCon_p, recvSeqNumCon & SEQ_NUM_MASK);

                    // reset own scon to 2 (valid connection)
                    pSdoSeqCon_p->recvSeqNum--;

                    // change state
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;
                    ret = sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateAckReceived);
                    if (ret == kErrorSdoComHandleBusy)
                    {
                        ret = kErrorOk;
                        forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                    }
                    else
                        forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                    if (ret != kErrorOk)
                        return ret;
                }

                // reset timeout counter
                pSdoSeqCon_p->retryCount = 0;
                // retransmission request resets the timer without real acknowledge
                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);

                if ((recvSeqNumCon & SEQ_NUM_MASK) == (pSdoSeqCon_p->recvSeqNum & SEQ_NUM_MASK))
                {   // latest sent frame acked, history buffer is empty
                    // -> send only an empty ack frame, or process new command layer if it exists

                    // change state
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;

                    // send data to higher layer if needed
                    if (((pSdoSeqCon_p->sendSeqNum + 4) & SEQ_NUM_MASK) == (sendSeqNumCon & SEQ_NUM_MASK))
                    {   // next frame of sequence received (new command layer data)
                        pSdoSeqCon_p->countCmdLayerInactive = 0;

                        if (!pSdoSeqCon_p->fForceFlowControl)
                        {
                            // save send sequence number (without ack request)
                            pSdoSeqCon_p->sendSeqNum = sendSeqNumCon & ~0x01;

                            // forward Rx frame to command layer
                            ret = sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                                   ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                                   (dataSize_p - SDO_SEQ_HEADER_SIZE));
                            if (ret == kErrorSdoComHandleBusy)
                            {
                                ret = kErrorOk;
                                forceRetransmissionRequest(pSdoSeqCon_p, TRUE);
                            }
                            else
                                forceRetransmissionRequest(pSdoSeqCon_p, FALSE);

                            // Note: Posting "kAsySdoConStateFrameReceived" to higher layer not necessary
                            //       here, because if we are server it is an initial transfer and will
                            //       not be processed. If we are client, it doesn't happen since the other
                            //       node would need to be a client also.
                        }
                    }
                    else
                    {   // send empty ack
                        // create answer own rcon = 2, since history is empty
                        pSdoSeqCon_p->recvSeqNum = recvSeqNumCon | 0x03;
                        pSdoSeqCon_p->recvSeqNum--;
                        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                        if (ret != kErrorOk)
                            return ret;
                    }
                }
                else
                {   // retransmit all frames from history
                    ret= sendAllTxHistory(pSdoSeqCon_p);
                    if (ret != kErrorOk)
                        return ret;
                }
                break;
        }

    }
    else if (event_p == kSdoSeqEventTimeout)
    {
        processTimeoutEvent(pSdoSeqCon_p, sdoSeqConHdl_p);
        if (ret != kErrorOk)
            return ret;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process SDO states

The function processes the internal SDO sequence layer state machine.

\param[in]      handle_p            Index of the control structure of the connection
\param[in]      dataSize_p          Size of sequence layer frame (without higher layer
                                    headers).
\param[in,out]  pData_p             Pointer to frame to be sent (can be NULL).
\param[in]      pRecvFrame_p        Pointer to received frame.
\param[in]      event_p             Event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processState(UINT handle_p,
                               size_t dataSize_p,
                               tPlkFrame* pData_p,
                               const tAsySdoSeq* pRecvFrame_p,
                               tSdoSeqEvent event_p)
{
    tOplkError      ret = kErrorOk;
    tSdoSeqCon*     pSdoSeqCon;
    tSdoSeqConHdl   sdoSeqConHdl;

#if (defined(WIN32) || defined(_WIN32))
    EnterCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    // get handle for higher layer
    sdoSeqConHdl = (tSdoSeqConHdl)(handle_p | SDO_ASY_HANDLE);

    // check if handle invalid
    if ((sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
        return kErrorSdoSeqInvalidHdl;

    // get pointer to connection
    pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[handle_p];

    // check size
    if ((pData_p == NULL) && (pRecvFrame_p == NULL) && (dataSize_p != 0))
        return kErrorSdoSeqInvalidFrame;

    // check state
    switch (pSdoSeqCon->sdoSeqState)
    {
        // idle state
        case kSdoSeqStateIdle:
            ret = processStateIdle(pSdoSeqCon,
                                   sdoSeqConHdl,
                                   event_p,
                                   pRecvFrame_p);
            break;

        // init connection step 1 - wait for frame with scon = 1 and rcon = 1
        case kSdoSeqStateInit1:
            ret = processStateInit1(pSdoSeqCon,
                                    sdoSeqConHdl,
                                    event_p,
                                    pRecvFrame_p);
            break;

        // init connection step 2
        case kSdoSeqStateInit2:
            ret = processStateInit2(pSdoSeqCon,
                                    sdoSeqConHdl,
                                    event_p,
                                    pRecvFrame_p);
            break;

        // init connection step 3
        case kSdoSeqStateInit3:
            ret = processStateInit3(pSdoSeqCon,
                                    sdoSeqConHdl,
                                    event_p,
                                    pRecvFrame_p);
            break;

        // connection established
        case kSdoSeqStateConnected:
            ret = processStateConnected(pSdoSeqCon,
                                        sdoSeqConHdl,
                                        event_p,
                                        pRecvFrame_p,
                                        dataSize_p,
                                        pData_p);
            break;

        // wait for acknowledge (history buffer full)
        case kSdoSeqStateWaitAck:
            ret = processStateWaitAck(pSdoSeqCon,
                                      sdoSeqConHdl,
                                      event_p,
                                      pRecvFrame_p,
                                      dataSize_p);
            break;

        // unknown state
        default:
            DEBUG_LVL_SDO_TRACE("Error: Unknown State in %s()\n", __func__);
            break;
    } // end of switch (pSdoSeqCon_p->sdoSeqState)

#if (defined(WIN32) || defined(_WIN32))
    LeaveCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an SDO frame

The function creates and sends a frame. If dataSize_p is 0 it creates a frame
with information from pSdoSeqCon_p.

\param[in,out]  pSdoSeqCon_p        Pointer to control structure of connection.
\param[in]      dataSize_p          Size of sequence layer frame (without higher layer
                                    headers).
\param[in,out]  pData_p             Pointer to frame to be sent (can be NULL).
\param[in]      fFrameInHistory_p   If TRUE, the frame is saved into the history buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendFrame(tSdoSeqCon* pSdoSeqCon_p,
                            size_t dataSize_p,
                            tPlkFrame* pData_p,
                            BOOL fFrameInHistory_p)
{
    tOplkError  ret = kErrorOk;
    tOplkError  retReplace = kErrorOk;
    UINT8       aFrame[SDO_SEQ_FRAME_SIZE];
    tPlkFrame*  pFrame;
    tPlkFrame*  pFrameResend;
    size_t      frameSizeResend;
    UINT8       freeEntries = 0;

    if (pData_p == NULL)
    {   // set pointer to own frame
        OPLK_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));
        pFrame = (tPlkFrame*)&aFrame[0];
    }
    else
    {
        pFrame = pData_p;
    }

    // filling header informations
    ami_setUint8Le(&pFrame->data.asnd.serviceId, 0x05);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.aReserved, 0x00);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon, pSdoSeqCon_p->sendSeqNum);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon, pSdoSeqCon_p->recvSeqNum);
    dataSize_p += SDO_SEQ_HEADER_SIZE;

    if (fFrameInHistory_p != FALSE)
    {
        // save frame to history
        ret = addFrameToHistory(pSdoSeqCon_p, pFrame, dataSize_p, TRUE);
        if (ret != kErrorOk)
            goto Exit;

        // check if only one free entry in history buffer
        freeEntries = getFreeHistoryEntries(pSdoSeqCon_p);
        if (freeEntries <= 1)
        {   // change state
            retReplace = kErrorSdoSeqRequestAckNeeded;
        }

        // send unsent frames from history first to prevent retransmission request
        // caused by newer frames "overtaking" unsent frames internally
        ret = readFromHistory(pSdoSeqCon_p, &pFrameResend, &frameSizeResend, TRUE);
        while ((pFrameResend != NULL) && (frameSizeResend != 0))
        {
            if (ret == kErrorRetry)
            { // resend unsent frame
                ret = sendToLowerLayer(pSdoSeqCon_p, frameSizeResend, pFrameResend);
                if (ret == kErrorDllAsyncTxBufferFull)
                {
                    ret = kErrorOk; // ignore unsent frames
                    break;          // stop sending old frames
                }
                if (ret != kErrorOk)
                    goto Exit;
            }
            // read next frame
            ret = readFromHistory(pSdoSeqCon_p, &pFrameResend, &frameSizeResend, FALSE);
        }
    }
    else
    {   // frame not stored to history
        ret = sendToLowerLayer(pSdoSeqCon_p, dataSize_p, pFrame);
    }

Exit:
    if (ret == kErrorOk)
        ret = retReplace;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send frame to lower layer

The function sends an already created frame to the lower layer.

\param[in]      pSdoSeqCon_p        Pointer to control structure of connection.
\param[in]      dataSize_p          Size of sequence layer frame (without higher layer
                                    headers).
\param[in,out]  pFrame_p            Pointer to frame to be sent (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendToLowerLayer(const tSdoSeqCon* pSdoSeqCon_p,
                                   size_t dataSize_p,
                                   tPlkFrame* pFrame_p)
{
    tOplkError  ret = kErrorOk;
    tSdoConHdl  handle;

    handle = pSdoSeqCon_p->conHandle & SDO_ASY_HANDLE_MASK;
    switch (handle)
    {
        case SDO_UDP_HANDLE:
#if defined(CONFIG_INCLUDE_SDO_UDP)
            ret = sdoudp_sendData(pSdoSeqCon_p->conHandle, pFrame_p, dataSize_p);
#else
            ret = kErrorSdoSeqUnsupportedProt;
#endif
            break;

        case SDO_ASND_HANDLE:
#if defined(CONFIG_INCLUDE_SDO_ASND)
            ret = sdoasnd_sendData(pSdoSeqCon_p->conHandle, pFrame_p, dataSize_p);
#else
            ret = kErrorSdoSeqUnsupportedProt;
#endif
            break;

        default:
            ret =  kErrorSdoSeqInvalidHdl;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Receive callback function

The function implements the receive callback function which is call for received
frames from the lower layer.

\param[in]      conHdl_p            SDO connection handle of the connection.
\param[in]      pSdoSeqData_p       Pointer to information structure of received frame.
\param[in]      dataSize_p          Size of sequence layer payload (without any
                                    headers).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveCb(tSdoConHdl conHdl_p,
                            const tAsySdoSeq* pSdoSeqData_p,
                            size_t dataSize_p)
{
    tOplkError  ret = kErrorOk;
    UINT        count;
    UINT        freeEntry;
    tSdoSeqCon* pSdoSeqCon;

    do
    {
        count = 0;
        freeEntry = CONFIG_SDO_MAX_CONNECTION_SEQ;

#if (defined(WIN32) || defined(_WIN32))
        EnterCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

        DEBUG_LVL_SDO_TRACE("Handle: 0x%x , First data Byte 0x%x\n",
                            conHdl_p,
                            ((const UINT8*)pSdoSeqData_p)[0]);

        // search control structure for this connection
        pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[count];
        while (count < CONFIG_SDO_MAX_CONNECTION_SEQ)
        {
            if (pSdoSeqCon->conHandle == conHdl_p)
                break;
            else if ((pSdoSeqCon->conHandle == 0) && (freeEntry == CONFIG_SDO_MAX_CONNECTION_SEQ))
                freeEntry = count;   // free entry

            count++;
            pSdoSeqCon++;
        }

        if (count == CONFIG_SDO_MAX_CONNECTION_SEQ)
        {   // new connection
            if (freeEntry == CONFIG_SDO_MAX_CONNECTION_SEQ)
            {
                ret = kErrorSdoSeqNoFreeHandle;
#if (defined(WIN32) || defined(_WIN32))
                LeaveCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif
                return ret;
            }
            else
            {
                pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[freeEntry];
                pSdoSeqCon->conHandle = conHdl_p;    // save handle from lower layer
                pSdoSeqCon->useCount++;
                count = freeEntry;
            }
        }

#if (defined(WIN32) || defined(_WIN32))
        LeaveCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

        // call process function with pointer of frame and event kSdoSeqEventFrameRec
        ret = processState(count, dataSize_p, NULL, pSdoSeqData_p, kSdoSeqEventFrameRec);
    } while (ret == kErrorRetry);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize history buffer

The function initializes the history buffer of a SDO connection.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initHistory(tSdoSeqCon* pSdoSeqCon_p)
{
    pSdoSeqCon_p->sdoSeqConHistory.freeEntries = SDO_HISTORY_SIZE;
    pSdoSeqCon_p->sdoSeqConHistory.ackIndex = 0;
    pSdoSeqCon_p->sdoSeqConHistory.writeIndex = 0;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Add frame to the history buffer

The function adds a frame to the history buffer.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      pFrame_p            Pointer to frame to be stored in history buffer.
\param[in]      size_p              Size of sequence layer frame
\param[in]      fTxFailed_p         Flag indicating that lower layer send function failed
                                    e.g. due to buffer overflow and should be repeated later

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError addFrameToHistory(tSdoSeqCon* pSdoSeqCon_p,
                                    const tPlkFrame* pFrame_p,
                                    size_t size_p,
                                    BOOL fTxFailed_p)
{
    tOplkError          ret = kErrorOk;
    tSdoSeqConHistory*  pHistory;
    tPlkFrame*          pHistoryFrame;

    // add frame to history buffer
    // check size - SDO_SEQ_HISTORY_FRAME_SIZE includes the header size, but size_p does not!
    if ((size_p + ASND_HEADER_SIZE) > SDO_SEQ_TX_HISTORY_FRAME_SIZE)
        return kErrorSdoSeqFrameSizeError;

    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;      // save pointer to history

    // check if a free entry is available
    if (pHistory->freeEntries > 0)
    {   // write message in free entry
        pHistoryFrame = (tPlkFrame*)pHistory->aHistoryFrame[pHistory->writeIndex];

        OPLK_MEMCPY(&pHistoryFrame->messageType,
                    &pFrame_p->messageType,
                    size_p + ASND_HEADER_SIZE);
        pHistory->aFrameSize[pHistory->writeIndex] = size_p;
        pHistory->afFrameFirstTxFailed[pHistory->writeIndex] = fTxFailed_p;
        pHistory->freeEntries--;
        pHistory->writeIndex++;
        if (pHistory->writeIndex == SDO_HISTORY_SIZE)    // check if write-index ran over array-border
            pHistory->writeIndex = 0;
    }
    else
        ret = kErrorSdoSeqNoFreeHistory;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send the whole history buffer

The function sends all frames stored in the Tx history buffer to lower layer.

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendAllTxHistory(tSdoSeqCon* pSdoSeqCon_p)
{
    tOplkError  ret = kErrorOk;
    size_t      frameSize;
    tPlkFrame*  pFrame;

    ret = readFromHistory(pSdoSeqCon_p, &pFrame, &frameSize, TRUE);
    if (ret == kErrorRetry)
        ret = kErrorOk; // ignore unsent frames info

    if (ret != kErrorOk)
        return ret;

    while ((pFrame != NULL) && (frameSize != 0))
    {
        ret = sendToLowerLayer(pSdoSeqCon_p, frameSize, pFrame);
        if (ret == kErrorDllAsyncTxBufferFull)
        {
            ret = kErrorOk; // ignore unsent frames but stop sending since
            break;          // following tries will also fail
        }
        if (ret != kErrorOk)
            return ret;

        ret = readFromHistory(pSdoSeqCon_p, &pFrame, &frameSize, FALSE);
        if (ret == kErrorRetry)
            ret = kErrorOk; // ignore unsent frames info

        if (ret != kErrorOk)
            return ret;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete acknowledged frame from the history buffer

The function deletes an acknowledged frame from the history buffer.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      recvSeqNumber_p     Receive sequence number of frame to delete.

\return The function returns kErrorOk.
*/
//------------------------------------------------------------------------------
static tOplkError deleteAckedFrameFromHistory(tSdoSeqCon* pSdoSeqCon_p,
                                              UINT8 recvSeqNumber_p)
{
    tOplkError          ret = kErrorOk;
    tSdoSeqConHistory*  pHistory;
    UINT8               ackIndex;
    UINT8               currentSeqNum;
    tPlkFrame*          pHistoryFrame;

    // get pointer to history buffer
    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;

    // release all acknowledged frames from history buffer

    // check if there are entries in history
    if (pHistory->freeEntries < SDO_HISTORY_SIZE)
    {
        ackIndex = pHistory->ackIndex;
        do
        {
            pHistoryFrame = (tPlkFrame*)pHistory->aHistoryFrame[ackIndex];

            currentSeqNum = (pHistoryFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon & SEQ_NUM_MASK);
            if (((recvSeqNumber_p - currentSeqNum) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
            {
                pHistory->aFrameSize[ackIndex] = 0;
                pHistory->afFrameFirstTxFailed[ackIndex] = FALSE;
                ackIndex++;
                pHistory->freeEntries++;
                if (ackIndex == SDO_HISTORY_SIZE)
                    ackIndex = 0;
            }
            else
            {   // nothing to do anymore, because any further frame in history
                // has larger sequence number than the acknowledge
                return ret;
            }
        } while ((((recvSeqNumber_p - 1 - currentSeqNum) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD) &&
                 (pHistory->writeIndex != ackIndex));

        // store local read-index to global var
        pHistory->ackIndex = ackIndex;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read a frame from the history buffer

The function reads a frame from the history buffer.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[out]     ppFrame_p           Pointer to store the pointer of the frame.
\param[out]     pSize_p             Pointer to store the size of the frame
\param[in]      fInitRead_p         Indicates the start of a retransmission. If TRUE,
                                    it returns the last, not acknowledged frame.

\return The function returns a tOplkError error code.
\retval kErrorOk            No error
\retval kErrorRetry         If the frame has never been sent successfully
*/
//------------------------------------------------------------------------------
static tOplkError readFromHistory(tSdoSeqCon* pSdoSeqCon_p,
                                  tPlkFrame** ppFrame_p,
                                  size_t* pSize_p,
                                  BOOL fInitRead_p)
{
    tOplkError          ret = kErrorOk;
    tSdoSeqConHistory*  pHistory;

    // read one message from History

    // get pointer to history buffer
    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;

    // check if init
    if (fInitRead_p)
    {   // initialize read index to the index which shall be acknowledged next
        pHistory->readIndex = pHistory->ackIndex;
    }

    // history buffer not empty and end of read iteration not yet reached
    if ((pHistory->freeEntries < SDO_HISTORY_SIZE) &&
        ((pHistory->writeIndex != pHistory->readIndex) ||
         ((pHistory->freeEntries == 0) && fInitRead_p)))
    {
        // inform caller about unsent frame
        if (pHistory->afFrameFirstTxFailed[pHistory->readIndex])
        {
            // signal caller, that this frame has not been sent successfully yet
            ret = kErrorRetry;
        }

        DEBUG_LVL_SDO_TRACE("%s(): init = %d, read = %u, write = %u, ack = %u",
                            __func__,
                            (int)fInitRead_p,
                            (UINT16)pHistory->readIndex,
                            (UINT16)pHistory->writeIndex,
                            (UINT16)pHistory->ackIndex);
        DEBUG_LVL_SDO_TRACE(", free entries = %u, next frame size = %u\n",
                            (UINT16)pHistory->freeEntries,
                            pHistory->aFrameSize[pHistory->readIndex]);

        // return pointer to stored frame
        *ppFrame_p = (tPlkFrame*)pHistory->aHistoryFrame[pHistory->readIndex];
        *pSize_p = pHistory->aFrameSize[pHistory->readIndex];   // save size
        pHistory->readIndex++;
        if (pHistory->readIndex == SDO_HISTORY_SIZE)
            pHistory->readIndex = 0;
    }
    else
    {
        DEBUG_LVL_SDO_TRACE("%s(): read = %u, write = %u, ack = %u, free entries = %u, no frame\n",
                            __func__,
                            (UINT16)pHistory->readIndex,
                            (UINT16)pHistory->writeIndex,
                            (UINT16)pHistory->ackIndex,
                            (UINT16)pHistory->freeEntries);

        // no more frames to send - return null pointer
        *ppFrame_p = NULL;
        *pSize_p = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get number of free history entries

The function returns the number of free history entries.

\param[in]      pSdoSeqCon_p        Pointer to connection control structure.

\return The function returns the number of free history entries.
*/
//------------------------------------------------------------------------------
static UINT8 getFreeHistoryEntries(const tSdoSeqCon* pSdoSeqCon_p)
{
    return pSdoSeqCon_p->sdoSeqConHistory.freeEntries;
}

//------------------------------------------------------------------------------
/**
\brief  Set a timer

The function sets up a timer with the specified timeout for the connection.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      timeout_p           Timeout to set in milliseconds.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setTimer(tSdoSeqCon* pSdoSeqCon_p, ULONG timeout_p)
{
    tOplkError  ret = kErrorOk;
    tTimerArg   timerArg;

    timerArg.eventSink = kEventSinkSdoAsySeq;
    timerArg.argument.pValue = pSdoSeqCon_p;

    if (pSdoSeqCon_p->timerHandle == 0)
    {   // create new timer
        ret = timeru_setTimer(&pSdoSeqCon_p->timerHandle, timeout_p, &timerArg);
    }
    else
    {   // modify existing timer
        ret = timeru_modifyTimer(&pSdoSeqCon_p->timerHandle, timeout_p, &timerArg);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes final sequence layer timeout

The function processes and signals the final sequence layer timeout, causing the
connection to be closed.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.
*/
//------------------------------------------------------------------------------
static void processFinalTimeout(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p)
{
    DEBUG_LVL_SDO_TRACE("SDO final timeout!\n");

    // timeout, because of no traffic -> Close
    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
    // set rcon and scon to 0
    pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
    pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

    sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

    sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTimeout);
}

//------------------------------------------------------------------------------
/**
\brief  Processes a sequence layer sub timeout

The function processes a sequence layer sub timeout and sends the oldest
frame of the history buffer. This timeout does not cause the connection to
be closed immediately, only if it occurred a multiple times without any
acknowledge from the other node.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSubTimeout(tSdoSeqCon* pSdoSeqCon_p)
{
    tOplkError  ret = kErrorOk;
    size_t      frameSize;
    tPlkFrame*  pFrame;
    UINT8       recvSeqNumCon;

    DEBUG_LVL_SDO_TRACE("SDO temporary timeout!\n");

    if (pSdoSeqCon_p->fForceFlowControl)
    {   // other node is about to close the connection
        // -> reset timer of other node with
        // retransmission request
        pSdoSeqCon_p->sendSeqNum |= 0x03;
    }
    else
    {   // create ack with own scon = 2
        pSdoSeqCon_p->sendSeqNum |= 0x03;
        pSdoSeqCon_p->sendSeqNum--;
    }

    // resend data with acknowledge request
    pSdoSeqCon_p->retryCount++;
    ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
    if (ret != kErrorOk)
        return ret;

    // read first frame from history
    ret = readFromHistory(pSdoSeqCon_p, &pFrame, &frameSize, TRUE);
    if (ret == kErrorRetry)
        ret = kErrorOk; // ignore unsent frames info
    if (ret != kErrorOk)
        return ret;

    if ((pFrame != NULL) && (frameSize != 0))
    {
        // set ack request in scon
        ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon,
                       ami_getUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon) | 0x03);

        if (pSdoSeqCon_p->fForceFlowControl)
        {   // other node is about to close the connection
            // -> reset timer of other node with retransmission request in rcon
            ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon,
                           ami_getUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon) | 0x03);
        }
        else
        {   // create ack with own scon = 2
            ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon,
                           (ami_getUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon) & SEQ_NUM_MASK) |
                            0x02);
        }

        ret = sendToLowerLayer(pSdoSeqCon_p, frameSize, pFrame);
        if (ret == kErrorDllAsyncTxBufferFull)
            ret = kErrorOk; // ignore unsent frames
        if (ret != kErrorOk)
            return ret;
    }
    else
    {   // send empty frame with ack request in own scon
        recvSeqNumCon = pSdoSeqCon_p->recvSeqNum; // save sequence number
        pSdoSeqCon_p->recvSeqNum |= 0x03;
        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
        pSdoSeqCon_p->recvSeqNum = recvSeqNumCon; // restore sequence number
        if (ret != kErrorOk)
            return ret;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes a sequence layer timeout event

The function processes and signals a sequence layer timeout event.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      sdoSeqConHdl_p      Handle of sequence layer connection.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processTimeoutEvent(tSdoSeqCon* pSdoSeqCon_p,
                                      tSdoSeqConHdl sdoSeqConHdl_p)
{
    tOplkError  ret = kErrorOk;

    // monitor inactive command layer
    if (!pSdoSeqCon_p->fForceFlowControl)
    {
        if (pSdoSeqCon_p->countCmdLayerInactive < SDO_SEQ_CMDL_INACTIVE_THLD)
            pSdoSeqCon_p->countCmdLayerInactive++;
        else
        {   // sequence layer connection might be valid, but no command layer
            // activity happens => Close connection to save bandwidth.
            processFinalTimeout(pSdoSeqCon_p, sdoSeqConHdl_p);
            return ret;
        }
    }

    // sequence layer timeout
    if (pSdoSeqCon_p->retryCount < SDO_SEQ_RETRY_COUNT)
    {   // retry counter not exceeded
        ret = processSubTimeout(pSdoSeqCon_p);
        if (ret != kErrorOk)
            return ret;
    }
    else
        processFinalTimeout(pSdoSeqCon_p, sdoSeqConHdl_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete SDO sequence connection to lower layer

The function deletes the sequence layer connection and the connection to the
lower layer as well as the related timer.

\param[in,out]  pSdoSeqCon_p        Pointer to sequence layer connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError deleteLowLayerConnection(tSdoSeqCon* pSdoSeqCon_p)
{
    tOplkError ret = kErrorOk;

    if (pSdoSeqCon_p == NULL)
    {
        ret = kErrorSdoSeqInvalidHdl;
        goto Exit;
    }

    //check protocol
    if ((pSdoSeqCon_p->conHandle & SDO_ASY_HANDLE_MASK) == SDO_UDP_HANDLE)
    {
#if defined(CONFIG_INCLUDE_SDO_UDP)
        sdoudp_delConnection(pSdoSeqCon_p->conHandle);
#endif
    }
    else
    {
#if defined(CONFIG_INCLUDE_SDO_ASND)
        sdoasnd_deleteCon(pSdoSeqCon_p->conHandle);
#endif
    }
    timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);

    // cleanup control structure
    OPLK_MEMSET(pSdoSeqCon_p, 0x00, sizeof(tSdoSeqCon));
    pSdoSeqCon_p->sdoSeqConHistory.freeEntries = SDO_HISTORY_SIZE;

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief  Check if a history frame was acknowledged by the receiver

The function checks if the receiver sent an acknowledge for a frame existing
in the (Tx) history buffer.

\param[in]      pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      recvSeqNumber_p     Receive sequence number of frame to delete.

\return The function returns TRUE if the receiver sent a ack. Otherwise FALSE.
*/
//------------------------------------------------------------------------------
static BOOL checkHistoryAcked(const tSdoSeqCon* pSdoSeqCon_p,
                              UINT8 recvSeqNumber_p)
{
    const tSdoSeqConHistory*    pHistory;
    UINT8                       currentSeqNum;
    const tPlkFrame*            pHistoryFrame;

    // get pointer to history buffer
    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;
    pHistoryFrame = (const tPlkFrame*)pHistory->aHistoryFrame[pHistory->ackIndex];
    currentSeqNum = (pHistoryFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon & SEQ_NUM_MASK);
    if (((recvSeqNumber_p - currentSeqNum) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
    {   // acknowledges at least the oldest history frame
        return TRUE;
    }
    else
        return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Check if a received acknowledge indicates a valid connection

The function checks if the receiver confirms the reception of a sequence frame
sent by this node (sender). This ack can be old, but needs to be in a certain
range due to an overflowing sequence number counter.

\param[in]      pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      recvSeqNumber_p     Receive sequence number of frame to delete.

\return The function returns TRUE if the receiver sent an ack. Otherwise FALSE.
*/
//------------------------------------------------------------------------------
static BOOL checkConnectionAckValid(const tSdoSeqCon* pSdoSeqCon_p,
                                    UINT8 recvSeqNumber_p)
{
    UINT8   currentSeqNum;

    // get expected receive sequence number
    currentSeqNum = pSdoSeqCon_p->recvSeqNum & SEQ_NUM_MASK;

    if (((currentSeqNum - recvSeqNumber_p) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
    {   // acknowledges a sequence frame within a valid range (considering overflow)
        return TRUE;
    }
    else
        return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Send one frame (oldest) of the Tx history buffer

This function transmits the oldest frame of the Tx history buffer, if
it exists. If the history buffer is empty, nothing happens.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      recvSeqNumber_p     Receive sequence number of frame to delete.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendHistoryOldestSegm(tSdoSeqCon* pSdoSeqCon_p,
                                        UINT8 recvSeqNumber_p)
{
    tOplkError  ret = kErrorOk;
    size_t      frameSize;
    tPlkFrame*  pFrame;

    // transmission on server for last segments
    if (((recvSeqNumber_p & SEQ_NUM_MASK) != (pSdoSeqCon_p->recvSeqNum & SEQ_NUM_MASK)) &&
        ((recvSeqNumber_p & SDO_CON_MASK) == 2))
    {   // old acknowledge of receiver
        // Use this as trigger for last segments, since they
        // don't get a trigger otherwise, except a timeout.

        // send oldest history frame
        ret = readFromHistory(pSdoSeqCon_p, &pFrame, &frameSize, TRUE);
        if (ret == kErrorRetry)
            ret = kErrorOk; // ignore unsent frames info
        if (ret != kErrorOk)
            return ret;

        if ((pFrame != NULL) && (frameSize != 0))
        {
            ret = sendToLowerLayer(pSdoSeqCon_p, frameSize, pFrame);
            if (ret == kErrorDllAsyncTxBufferFull)
                ret = kErrorOk; // ignore unsent frame

            if (ret != kErrorOk)
                return ret;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Use retransmission request to control flow and keep connection alive

Flow control possibility for application by dropping newly received sequences
in the sequence layer and not forwarding them to the command layer until
the command layer sends a "delayed" response.
The sequence layer will keep the connection alive (resetting timeout of other
node) by responding on an acknowledge request with a retransmission request.
The own timeout will occur, if the command layer does not respond.

\param[in,out]  pSdoSeqCon_p        Pointer to connection control structure.
\param[in]      fEnable_p           TRUE: enable manipulation.
                                    FALSE: disable manipulation.

*/
//------------------------------------------------------------------------------
static void forceRetransmissionRequest(tSdoSeqCon* pSdoSeqCon_p, BOOL fEnable_p)
{
    DEBUG_LVL_SDO_TRACE("%s(): %d\n", __func__, fEnable_p);

    pSdoSeqCon_p->fForceFlowControl = fEnable_p;
}

/// \}
