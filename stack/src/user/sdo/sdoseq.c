/**
********************************************************************************
\file   sdoseq.c

\brief  Implementation of SDO Sequence Layer

This file contains the implementation of the SDO Sequence Layer

\ingroup module_sdo_seq
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#if !defined(CONFIG_INCLUDE_SDO_UDP) && !defined(CONFIG_INCLUDE_SDO_ASND)
#error 'ERROR: sdoseq.c - At least UDP or ASND module needed!'
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SDO_HISTORY_SIZE            5

#ifndef CONFIG_SDO_MAX_CONNECTION_SEQ
#define CONFIG_SDO_MAX_CONNECTION_SEQ             5
#endif

#define SDO_SEQ_DEFAULT_TIMEOUT     5000                        // in [ms] => 5 sec
#define SDO_SEQ_RETRY_COUNT         5                           // => max. Timeout 30 sec
#define SDO_SEQ_NUM_THRESHOLD       100                         // threshold which distinguishes between old and new sequence numbers
#define SDO_SEQ_FRAME_SIZE          24                          // frame with size of Asnd-Header-, SDO Sequence header size, SDO Command header and Ethernet-header size
#define SDO_SEQ_HEADER_SIZE         4                           // size of the header of the SDO Sequence layer
#define SDO_SEQ_TX_HISTORY_FRAME_SIZE  SDO_MAX_TX_FRAME_SIZE    // buffersize for one frame in history
#define SDO_CON_MASK                0x03                        // mask to get scon and rcon

#define SEQ_NUM_MASK                0xFC

const UINT32 SDO_SEQU_MAX_TIMEOUT_MS = (UINT32)86400000UL; // [ms], 86400000 ms = 1 day

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
    kSdoSeqEventNoEvent  =   0x00,   ///< No Event
    kSdoSeqEventInitCon  =   0x01,   ///< Init connection
    kSdoSeqEventFrameRec =   0x02,   ///< Frame received
    kSdoSeqEventFrameSend=   0x03,   ///< Frame to send
    kSdoSeqEventTimeout  =   0x04,   ///< Timeout for connection
    kSdoSeqEventCloseCon =   0x05    ///< Higher layer closed connection
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
    UINT    aFrameSize[SDO_HISTORY_SIZE];           ///< Array of sizes of the history frames
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
    kSdoSeqStateIdle        = 0x00,             ///< SDO connection is idle (closed)
    kSdoSeqStateInit1       = 0x01,             ///< SDO init 1: scon=1, rcon=0
    kSdoSeqStateInit2       = 0x02,             ///< SDO init 2: scon=1, rcon=1
    kSdoSeqStateInit3       = 0x03,             ///< SDO init 3: scon=2, rcon=1
    kSdoSeqStateConnected   = 0x04,             ///< SDO connection is established
    kSdoSeqStateWaitAck     = 0x05              ///< SDO connection is waiting for an acknowledgement
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
    tSdoConHdl              conHandle;          ///< Connection handle
    tSdoSeqState            sdoSeqState;        ///< State of the connection
    UINT8                   recvSeqNum;         ///< Receive sequence number
                                                /**< Expected receive sequence number (acknowledge) of other node,
                                                     updated on every segment Tx with new command data layer data */
    UINT8                   sendSeqNum;         ///< Send sequence number of communication partner, updated on Rx
    tSdoSeqConHistory       sdoSeqConHistory;   ///< Connection history buffer
    tTimerHdl               timerHandle;        ///< Timer handle
    UINT                    retryCount;         ///< Retry counter
    UINT                    useCount;           ///< One sequence layer connection may be used by multiple command layer connections
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
    UINT32                  sdoSeqTimeout;                              ///< Configured Sequence layer timeout

#if defined(WIN32) || defined(_WIN32)
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
static tOplkError processState(UINT handle_p, UINT dataSize_p, tPlkFrame* pData_p,
                               tAsySdoSeq* pRecvFrame_p, tSdoSeqEvent event_p);

static tOplkError processStateIdle(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                   tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p);

static tOplkError processStateInit1(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p);

static tOplkError processStateInit2(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p);

static tOplkError processStateInit3(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p);

static tOplkError processStateConnected(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                        tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p,
                                        UINT dataSize_p, tPlkFrame* pData_p);

static tOplkError processStateWaitAck(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                      tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p,
                                      UINT dataSize_p);

static tOplkError sendFrame(tSdoSeqCon* pSdoSeqCon_p, UINT dataSize_p,
                            tPlkFrame* pData_p, BOOL fFrameInHistory_p);

static tOplkError sendToLowerLayer(tSdoSeqCon* pSdoSeqCon_p, UINT dataSize_p, tPlkFrame* pFrame_p);

static tOplkError receiveCb(tSdoConHdl conHdl_p, tAsySdoSeq* pSdoSeqData_p, UINT dataSize_p);

static tOplkError initHistory(tSdoSeqCon* pSdoSeqCon_p);

static tOplkError addFrameToHistory(tSdoSeqCon* pSdoSeqCon_p, tPlkFrame* pFrame_p,
                                    UINT size_p, BOOL fTxFailed_p);

static tOplkError sendAllTxHistory(tSdoSeqCon* pSdoSeqCon_p);

static tOplkError deleteAckedFrameFromHistory(tSdoSeqCon* pSdoSeqCon_p, UINT8 recvSeqNumber_p);

static tOplkError readFromHistory(tSdoSeqCon* pSdoSeqCon_p, tPlkFrame** ppFrame_p,
                                  UINT* pSize_p, BOOL fInitRead_p);

static UINT       getFreeHistoryEntries(tSdoSeqCon* pSdoSeqCon_p);

static tOplkError setTimer(tSdoSeqCon* pSdoSeqCon_p, ULONG timeout_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO sequence layer

The function initializes the SDO sequence layer

\param  pfnSdoComRecvCb_p       Pointer to callback function that informs command
                                layer about new frames.
\param  pfnSdoComConCb_p        Pointer to callback function that informs command
                                layer about connection state.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_init(tSdoComReceiveCb pfnSdoComRecvCb_p, tSdoComConCb pfnSdoComConCb_p)
{
    tOplkError      ret = kErrorOk;

    if (pfnSdoComRecvCb_p == NULL)
    {
        return kErrorSdoSeqMissCb;
    }
    else
    {
        sdoSeqInstance_l.pfnSdoComRecvCb = pfnSdoComRecvCb_p;
    }

    if (pfnSdoComConCb_p == NULL)
    {
        return kErrorSdoSeqMissCb;
    }
    else
    {
        sdoSeqInstance_l.pfnSdoComConCb = pfnSdoComConCb_p;
    }

    OPLK_MEMSET(&sdoSeqInstance_l.aSdoSeqCon[0], 0x00, sizeof(sdoSeqInstance_l.aSdoSeqCon));

#if defined(WIN32) || defined(_WIN32)
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
    sdoSeqInstance_l.sdoSeqTimeout = SDO_SEQ_DEFAULT_TIMEOUT;
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
    tOplkError          ret = kErrorOk;
    UINT                count;
    tSdoSeqCon*         pSdoSeqCon;

    // delete timer of open connections
    count = 0;
    pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[0];
    while (count < CONFIG_SDO_MAX_CONNECTION_SEQ)
    {
        if (pSdoSeqCon->conHandle != 0)
        {
            timeru_deleteTimer(&pSdoSeqCon->timerHandle);
        }
        count++;
        pSdoSeqCon++;
    }

#if defined(WIN32) || defined(_WIN32)
    // delete critical section for process function
    DeleteCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif
    OPLK_MEMSET(&sdoSeqInstance_l, 0x00, sizeof(sdoSeqInstance_l));

#if defined(CONFIG_INCLUDE_SDO_UDP)
    ret = sdoudp_exit();
#endif
#if defined(CONFIG_INCLUDE_SDO_ASND)
    ret = sdoasnd_exit();
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a SDO sequence layer connection

The function initializes a SDO sequence layer connection. It tries to reuse an
existing connection to the specified node.

\param  pSdoSeqConHdl_p         Pointer to store the sequence layer connection
                                handle.
\param  nodeId_p                Node ID of the target to connect to.
\param  sdoType_p               Type of the SDO connection.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_initCon(tSdoSeqConHdl* pSdoSeqConHdl_p, UINT nodeId_p, tSdoType sdoType_p)
{
    tOplkError          ret = kErrorOk;
    UINT                count;
    UINT                freeCon;
    tSdoConHdl          conHandle = ~0U;
    tSdoSeqCon*         pSdoSeqCon;

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
#else
            return kErrorSdoSeqUnsupportedProt;
#endif
            break;

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

    *pSdoSeqConHdl_p = (count | SDO_ASY_HANDLE);    // set handle

    ret = processState(count, 0, NULL, NULL, kSdoSeqEventInitCon);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send data via a sequence layer connection

The function sends data via an existing sequence layer connection.

\param  sdoSeqConHdl_p          Sequence layer connection handle to use for
                                transfer.
\param  dataSize_p              Size of sequence layer frame (without higher
                                layer headers)
\param  pData_p                 Pointer to the data to send.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_sendData(tSdoSeqConHdl sdoSeqConHdl_p, UINT dataSize_p, tPlkFrame* pData_p)
{
    tOplkError      ret;
    UINT            handle;

    handle = (sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

    // check if connection ready
    if (sdoSeqInstance_l.aSdoSeqCon[handle].sdoSeqState == kSdoSeqStateIdle)
    {
        // no connection with this handle
        return kErrorSdoSeqInvalidHdl;
    }
    else
    {
        if (sdoSeqInstance_l.aSdoSeqCon[handle].sdoSeqState != kSdoSeqStateConnected)
        {
            return kErrorSdoSeqConnectionBusy;
        }
    }
    ret = processState(handle, dataSize_p, pData_p, NULL, kSdoSeqEventFrameSend);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO event

The function processes SDO events.

\param  pEvent_p                Event to process.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_processEvent(tEvent* pEvent_p)
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
    while ((&sdoSeqInstance_l.aSdoSeqCon[count]) != pSdoSeqCon)
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

\param  sdoSeqConHdl_p          Connection handle of sequence layer connection
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

    handle = (sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

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

        //check protocol
        if ((pSdoSeqCon->conHandle & SDO_ASY_HANDLE_MASK) == SDO_UDP_HANDLE)
        {
#if defined(CONFIG_INCLUDE_SDO_UDP)
            sdoudp_delConnection(pSdoSeqCon->conHandle);
#endif
        }
        else
        {
#if defined(CONFIG_INCLUDE_SDO_ASND)
            sdoasnd_deleteCon(pSdoSeqCon->conHandle);
#endif
        }
        timeru_deleteTimer(&pSdoSeqCon->timerHandle);

        // cleanup control structure
        OPLK_MEMSET(pSdoSeqCon, 0x00, sizeof(tSdoSeqCon));
        pSdoSeqCon->sdoSeqConHistory.freeEntries = SDO_HISTORY_SIZE;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set sequence layer timeout

The function sets the sequence layer timeout.

\param  timeout_p           Timeout to set in milliseconds.

\return The function returns a tOplkError error code.

\ingroup module_sdo_seq
*/
//------------------------------------------------------------------------------
tOplkError sdoseq_setTimeout(UINT32 timeout_p)
{
    // Adopt new SDO sequence layer timeout (truncated to an upper bound)
    sdoSeqInstance_l.sdoSeqTimeout = min(timeout_p, SDO_SEQU_MAX_TIMEOUT_MS);
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

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateIdle(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                   tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p)
{
    tOplkError      ret = kErrorOk;

    switch (event_p)
    {
        // new connection -> send init frame and change to kSdoSeqStateInit1
        case kSdoSeqEventInitCon:
            pSdoSeqCon_p->recvSeqNum = 0x01;    // set sending scon to 1
            pSdoSeqCon_p->sendSeqNum = 0x00;    // set set send rcon to 0
            ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
            if (ret != kErrorOk)
                return ret;

            pSdoSeqCon_p->sdoSeqState = kSdoSeqStateInit1; // change state
            ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
            break;

        // init con from extern, check rcon and scon -> send answer
        case kSdoSeqEventFrameRec:
            DEBUG_LVL_SDO_TRACE("%s scon=%u rcon=%u\n", __func__,
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
            {   // error -> close - delete timer
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);

                if (((pRecvFrame_p->recvSeqNumCon & SDO_CON_MASK) != 0x00) ||
                    ((pRecvFrame_p->sendSeqNumCon & SDO_CON_MASK) != 0x00))
                {   // d.k. only answer with close message if the message sent was not a close message
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

        default:
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state Init1

The function processes the sequence layer state: kSdoSeqStateInit1

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit1(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p)
{
    tOplkError              ret = kErrorOk;

    // TRACE("sdoseq: processStateInit1\n");

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
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

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit2(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p)
{
    tOplkError          ret = kErrorOk;

    // TRACE("sdoseq: processStateInit2\n");

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
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

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateInit3(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                    tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p)
{
    tOplkError          ret = kErrorOk;

    switch (event_p)
    {
        // frame received
        case kSdoSeqEventFrameRec:
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

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame (can be NULL).
\param  dataSize_p          Size of sequence layer frame (without higher layer
                            headers).
\param  pData_p             Pointer to frame to be sent (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateConnected(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                        tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p,
                                        UINT dataSize_p, tPlkFrame* pData_p)
{
    tOplkError          ret = kErrorOk;
    UINT8               sendSeqNumCon;
    UINT                frameSize;
    tPlkFrame*          pFrame;
    UINT                freeEntries;

    switch (event_p)
    {
        // frame to send
        case kSdoSeqEventFrameSend:
            // set timer
            ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
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
                {   // request ack, change state to wait ack
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateWaitAck;
                    // set ret to kErrorOk, because no error
                    // for higher layer
                    ret = kErrorOk;
                }
                else if (ret != kErrorOk)
                {
                    return ret;
                }
                else
                {
                    sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameSent);
                }
            }
            break;

        // frame received
        case kSdoSeqEventFrameRec:
            sendSeqNumCon = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);

            ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);

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
                    if ((ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon) & SDO_CON_MASK) == 3)
                    {   // retransmission request (signaling lost segments)

                        // TRACE("sdoseq: error response received\n");
                        ret= sendAllTxHistory(pSdoSeqCon_p);
                        if (ret != kErrorOk)
                            return ret;
                    }

                    if (((pSdoSeqCon_p->sendSeqNum + 4) & SEQ_NUM_MASK) == (sendSeqNumCon & SEQ_NUM_MASK))
                    {   // next frame of sequence received (new command layer data)

                        // save send sequence number (without ack request)
                        pSdoSeqCon_p->sendSeqNum = sendSeqNumCon & ~0x01;

                        // check if ack or data-frame, ignore ack -> already processed
                        if (dataSize_p > SDO_SEQ_HEADER_SIZE)
                        {   // forward Rx frame to command layer
                            sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                (dataSize_p - SDO_SEQ_HEADER_SIZE));
                            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameReceived);
                        }
                        else
                        {
                            sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateAckReceived);
                        }
                    }
                    else if (((sendSeqNumCon - pSdoSeqCon_p->sendSeqNum - 4) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
                    {   // frame of sequence was lost,
                        // because difference of received and old value
                        // is less then halve of the values range.
                        // send error frame with own rcon = 3
                        pSdoSeqCon_p->sendSeqNum |= 0x03;
                        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                        // restore send sequence number
                        pSdoSeqCon_p->sendSeqNum = (pSdoSeqCon_p->sendSeqNum & SEQ_NUM_MASK) | 0x02;
                        if (ret != kErrorOk)
                            return ret;

                        // break here, because a requested acknowledge was sent implicitly above
                        break;
                    }
                    // else, ignore repeated frame

                    if ((sendSeqNumCon & SDO_CON_MASK) == 3)
                    {   // ack request received
                        // create ack with own scon = 2
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

            timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
            // call Command Layer Cb is not necessary, because the event came from there
            // sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl, kAsySdoConStateInitError);
            break;

        // timeout
        case kSdoSeqEventTimeout:
            freeEntries = getFreeHistoryEntries(pSdoSeqCon_p);
            if ((freeEntries < SDO_HISTORY_SIZE) &&
                (pSdoSeqCon_p->retryCount < SDO_SEQ_RETRY_COUNT))
            {   // unacknowledged frames in history and retry counter not exceeded
                // resend data with acknowledge request
                pSdoSeqCon_p->retryCount++;
                ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);
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

                    ret = sendToLowerLayer(pSdoSeqCon_p, frameSize, pFrame);
                    if (ret == kErrorDllAsyncTxBufferFull)
                        ret = kErrorOk; // ignore unsent frames
                    if (ret != kErrorOk)
                        return ret;
                }
            }
            else
            {
                // timeout, because of no traffic -> Close
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
                // set rcon and scon to 0
                pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
                pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;

                sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);

                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTimeout);
            }
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

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.
\param  sdoSeqConHdl_p      Handle of sequence layer connection.
\param  event_p             Event to be processed.
\param  pRecvFrame_p        Pointer to received frame.
\param  dataSize_p          Size of sequence layer frame (without higher layer
                            headers).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateWaitAck(tSdoSeqCon* pSdoSeqCon_p, tSdoSeqConHdl sdoSeqConHdl_p,
                                      tSdoSeqEvent event_p, tAsySdoSeq* pRecvFrame_p,
                                      UINT dataSize_p)
{
    tOplkError          ret = kErrorOk;

    DEBUG_LVL_SDO_TRACE("sdoseq: processStateWaitAck\n");

    ret = setTimer(pSdoSeqCon_p, sdoSeqInstance_l.sdoSeqTimeout);

    //TODO: retry of acknowledge
    if (event_p == kSdoSeqEventFrameRec)
    {
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

            // reinit from other node
            case 1:
                // return to idle
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
                timeru_deleteTimer(&pSdoSeqCon_p->timerHandle);
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTransferAbort);
                // restart immediately with initialization request
                ret = kErrorRetry;
                break;

            // normal frame
            case 2:
                // should be ack -> change to state kSdoSeqStateConnected
                pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;
                sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateAckReceived);
                // send data to higher layer if needed
                if (dataSize_p > SDO_SEQ_HEADER_SIZE)
                {   // forward Rx frame to command layer
                    sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                     ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                     (dataSize_p - SDO_SEQ_HEADER_SIZE));
                }
                break;

            // retransmission request (error response)
            case 3:
                if (pRecvFrame_p->recvSeqNumCon == pSdoSeqCon_p->recvSeqNum)
                {   // previously, an ack request was sent, and also now a
                    // retransmission request is received, but now new data
                    // is requested (all send segments are already acknowledged)
                    // -> change to state kSdoSeqStateConnected
                    pSdoSeqCon_p->sdoSeqState = kSdoSeqStateConnected;

                    // send ack
                    // save sequence numbers
                    pSdoSeqCon_p->recvSeqNum = ami_getUint8Le(&pRecvFrame_p->recvSeqNumCon);
                    pSdoSeqCon_p->sendSeqNum = ami_getUint8Le(&pRecvFrame_p->sendSeqNumCon);
                    // create answer own rcon = 2
                    pSdoSeqCon_p->recvSeqNum--;
                    // check if ack or data-frame
                    if (dataSize_p > SDO_SEQ_HEADER_SIZE)
                    {   // forward Rx frame to command layer
                        // TODO: why not checking for new send sequence number?
                        sdoSeqInstance_l.pfnSdoComRecvCb(sdoSeqConHdl_p,
                                                         ((tAsySdoCom*)&pRecvFrame_p->sdoSeqPayload),
                                                         (dataSize_p - SDO_SEQ_HEADER_SIZE));
                        sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateFrameReceived);
                    }
                    else
                    {
                        ret = sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
                        if (ret != kErrorOk)
                            return ret;
                    }
                }
                else
                {
                    // retransmit all frames from history
                    ret = sendAllTxHistory(pSdoSeqCon_p);
                }
                break;
        }

    }
    else if (event_p == kSdoSeqEventTimeout)
    {   // error -> Close
        pSdoSeqCon_p->sdoSeqState = kSdoSeqStateIdle;
        // set rcon and scon to 0
        pSdoSeqCon_p->sendSeqNum &= SEQ_NUM_MASK;
        pSdoSeqCon_p->recvSeqNum &= SEQ_NUM_MASK;
        sendFrame(pSdoSeqCon_p, 0, NULL, FALSE);
        sdoSeqInstance_l.pfnSdoComConCb(sdoSeqConHdl_p, kAsySdoConStateTimeout);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process SDO states

The function processes the internal SDO sequence layer state machine.

\param  handle_p            Index of the control structure of the connection
\param  dataSize_p          Size of sequence layer frame (without higher layer
                            headers).
\param  pData_p             Pointer to frame to be sent (can be NULL).
\param  pRecvFrame_p        Pointer to received frame.
\param  event_p             Event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processState(UINT handle_p, UINT dataSize_p, tPlkFrame* pData_p,
                               tAsySdoSeq* pRecvFrame_p, tSdoSeqEvent event_p)
{
    tOplkError          ret = kErrorOk;
    tSdoSeqCon*         pSdoSeqCon;
    tSdoSeqConHdl       sdoSeqConHdl;

#if defined(WIN32) || defined(_WIN32)
    EnterCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    // get handle for higher layer
    sdoSeqConHdl = handle_p | SDO_ASY_HANDLE;

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
            ret = processStateIdle(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p);
            break;

        // init connection step 1 - wait for frame with scon = 1 and rcon = 1
        case kSdoSeqStateInit1:
            ret = processStateInit1(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p);
            break;

        // init connection step 2
        case kSdoSeqStateInit2:
            ret = processStateInit2(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p);
            break;

        // init connection step 3
        case kSdoSeqStateInit3:
            ret = processStateInit3(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p);
            break;

        // connection established
        case kSdoSeqStateConnected:
            ret = processStateConnected(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p,
                                        dataSize_p, pData_p);
            break;

        // wait for Acknowledge (history buffer full)
        case kSdoSeqStateWaitAck:
            ret = processStateWaitAck(pSdoSeqCon, sdoSeqConHdl, event_p, pRecvFrame_p,
                                      dataSize_p);
            break;

        // unknown state
        default:
            DEBUG_LVL_SDO_TRACE("Error: Unknown State in processState\n");
            break;
    } // end of switch (pSdoSeqCon_p->sdoSeqState)

#if defined(WIN32) || defined(_WIN32)
    LeaveCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an SDO frame

The function creates and sends a frame. If dataSize_p is 0 it creates a frame
with information from pSdoSeqCon_p.

\param  pSdoSeqCon_p        Pointer to control structure of connection.
\param  dataSize_p          Size of sequence layer frame (without higher layer
                            headers).
\param  pData_p             Pointer to frame to be sent (can be NULL).
\param  fFrameInHistory_p   If TRUE, the frame is saved into the history buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendFrame(tSdoSeqCon* pSdoSeqCon_p, UINT dataSize_p,
                            tPlkFrame* pData_p, BOOL fFrameInHistory_p)
{
    tOplkError      ret;
    UINT8           aFrame[SDO_SEQ_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tPlkFrame*      pFrameResend;
    UINT            frameSizeResend;
    UINT            freeEntries = 0;

    if (pData_p == NULL)
    {   // set pointer to own frame
        OPLK_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));
        pFrame = (tPlkFrame*)&aFrame[0];
    }
    else
    {
        pFrame = pData_p;
    }

    if (fFrameInHistory_p != FALSE)
    {
        // check if only one free entry in history buffer
        freeEntries = getFreeHistoryEntries(pSdoSeqCon_p);
        if (freeEntries <= 1)
        {   // request an acknowledge in dataframe - own scon = 3
            pSdoSeqCon_p->recvSeqNum |= 0x03;
        }
    }

    // filling header informations
    ami_setUint8Le(&pFrame->data.asnd.serviceId, 0x05);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.aReserved, 0x00);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.recvSeqNumCon, pSdoSeqCon_p->sendSeqNum);
    ami_setUint8Le(&pFrame->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon, pSdoSeqCon_p->recvSeqNum);
    dataSize_p += SDO_SEQ_HEADER_SIZE;

    if (fFrameInHistory_p != FALSE)
    {
        // send unsent frames from history first to prevent retransmission request
        // caused by newer frames "overtaking" unsent frames
        ret = readFromHistory(pSdoSeqCon_p, &pFrameResend, &frameSizeResend, TRUE);
        while ((pFrameResend != NULL) && (frameSizeResend != 0))
        {
            if ((ret == kErrorRetry))
            { // resend unsent frame
                ret = sendToLowerLayer(pSdoSeqCon_p, frameSizeResend, pFrameResend);
                if (ret == kErrorDllAsyncTxBufferFull)
                {
                    ret = kErrorOk; // ignore unsent frames
                    break;          // stop sending old frames
                }
                if (ret != kErrorOk)
                    return ret;
            }
            // read next frame
            ret = readFromHistory(pSdoSeqCon_p, &pFrameResend, &frameSizeResend, FALSE);
        }
    }

    // calling send function is necessary for size check before storing it
    // to the history buffer, even if lower layer Tx buffer is full
    ret = sendToLowerLayer(pSdoSeqCon_p, dataSize_p, pFrame);
    if (((ret == kErrorOk) || (ret == kErrorDllAsyncTxBufferFull)) &&
        (fFrameInHistory_p != FALSE)                                  )
    {
        // set own scon to 2 if needed
        if ((pSdoSeqCon_p->recvSeqNum & 0x03) == 0x03)
        {
            pSdoSeqCon_p->recvSeqNum--;
        }

        // save frame to history
        ret = addFrameToHistory(pSdoSeqCon_p, pFrame, dataSize_p,
                                (ret == kErrorDllAsyncTxBufferFull));
        if ((ret == kErrorSdoSeqNoFreeHistory) || (freeEntries <= 1))
        {
            ret = kErrorSdoSeqRequestAckNeeded;       // request Ack needed
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send frame to lower layer

The function sends an already created fram to the lower layer.

\param  pSdoSeqCon_p        Pointer to control structure of connection.
\param  dataSize_p          Size of sequence layer frame (without higher layer
                            headers).
\param  pFrame_p            Pointer to frame to be sent (can be NULL).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendToLowerLayer(tSdoSeqCon* pSdoSeqCon_p, UINT dataSize_p,
                                   tPlkFrame* pFrame_p)
{
    tOplkError      ret;
    tSdoConHdl      handle;

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

\param  conHdl_p            SDO connection handle of the connection.
\param  pSdoSeqData_p       Pointer to information structure of received frame.
\param  dataSize_p          Size of sequence layer payload (without any
                            headers).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveCb(tSdoConHdl conHdl_p, tAsySdoSeq* pSdoSeqData_p,
                            UINT dataSize_p)
{
    tOplkError          ret;
    UINT                count;
    UINT                freeEntry;
    tSdoSeqCon*         pSdoSeqCon;

    do
    {
        count = 0;
        freeEntry = CONFIG_SDO_MAX_CONNECTION_SEQ;

#if defined(WIN32) || defined(_WIN32)
        EnterCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

        DEBUG_LVL_SDO_TRACE("Handle: 0x%x , First Databyte 0x%x\n", conHdl_p, ((BYTE*)pSdoSeqData_p)[0]);

        // search control structure for this connection
        pSdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[count];
        while (count < CONFIG_SDO_MAX_CONNECTION_SEQ)
        {
            if (pSdoSeqCon->conHandle == conHdl_p)
            {
                break;
            }
            else if ((pSdoSeqCon->conHandle == 0) && (freeEntry == CONFIG_SDO_MAX_CONNECTION_SEQ))
            {
                freeEntry = count;   // free entry
            }
            count++;
            pSdoSeqCon++;
        }

        if (count == CONFIG_SDO_MAX_CONNECTION_SEQ)
        {   // new connection
            if (freeEntry == CONFIG_SDO_MAX_CONNECTION_SEQ)
            {
                ret = kErrorSdoSeqNoFreeHandle;

#if defined(WIN32) || defined(_WIN32)
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

        // call history ack function
        ret = deleteAckedFrameFromHistory(pSdoSeqCon, (ami_getUint8Le(&pSdoSeqData_p->recvSeqNumCon) & SEQ_NUM_MASK));

#if defined(WIN32) || defined(_WIN32)
        LeaveCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif
        if (ret != kErrorOk)
            return ret;

        // call process function with pointer of frame and event kSdoSeqEventFrameRec
        ret = processState(count, dataSize_p, NULL, pSdoSeqData_p, kSdoSeqEventFrameRec);
    } while (ret == kErrorRetry);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize history buffer

The function initializes the history buffer of a SDO connection.

\param  pSdoSeqCon_p        Pointer to connection control structure.

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

\param  pSdoSeqCon_p        Pointer to connection control structure.
\param  pFrame_p            Pointer to frame to be stored in history buffer.
\param  size_p              Size of frame (without headers).
\param  fTxFailed_p         Flag indicating that lower layer send function failed
                            e.g. due to buffer overflow and should be repeated later

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError addFrameToHistory(tSdoSeqCon* pSdoSeqCon_p, tPlkFrame* pFrame_p,
                                    UINT size_p, BOOL fTxFailed_p)
{
    tOplkError              ret = kErrorOk;
    tSdoSeqConHistory*      pHistory;

    // add frame to history buffer
    // check size - SDO_SEQ_HISTORY_FRAME_SIZE includes the header size, but size_p does not!
    if (size_p > SDO_SEQ_TX_HISTORY_FRAME_SIZE)
        return kErrorSdoSeqFrameSizeError;

    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;      // save pointer to history

    // check if a free entry is available
    if (pHistory->freeEntries > 0)
    {   // write message in free entry
        OPLK_MEMCPY(&((tPlkFrame*)pHistory->aHistoryFrame[pHistory->writeIndex])->messageType,
                    &pFrame_p->messageType, size_p + ASND_HEADER_SIZE);
        pHistory->aFrameSize[pHistory->writeIndex] = size_p;
        pHistory->afFrameFirstTxFailed[pHistory->writeIndex] = fTxFailed_p;
        pHistory->freeEntries--;
        pHistory->writeIndex++;
        if (pHistory->writeIndex == SDO_HISTORY_SIZE)    // check if write-index ran over array-border
        {
            pHistory->writeIndex = 0;
        }
    }
    else
    {
        ret = kErrorSdoSeqNoFreeHistory;
    }


    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send the whole history buffer

The function send all frames stored in the Tx history buffer to lower layer.

\param  pSdoSeqCon_p        Pointer to sequence layer connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendAllTxHistory(tSdoSeqCon* pSdoSeqCon_p)
{
    tOplkError          ret = kErrorOk;
    UINT                frameSize;
    tPlkFrame*          pFrame;

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

\param  pSdoSeqCon_p        Pointer to connection control structure.
\param  recvSeqNumber_p     Receive sequence number of frame to delete.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError deleteAckedFrameFromHistory(tSdoSeqCon* pSdoSeqCon_p, UINT8 recvSeqNumber_p)
{
    tOplkError              ret = kErrorOk;
    tSdoSeqConHistory*      pHistory;
    UINT8                   ackIndex;
    UINT8                   currentSeqNum;

    // get pointer to history buffer
    pHistory = &pSdoSeqCon_p->sdoSeqConHistory;

    // release all acknowledged frames from history buffer

    // check if there are entries in history
    if (pHistory->freeEntries < SDO_HISTORY_SIZE)
    {
        ackIndex = pHistory->ackIndex;
        do
        {
            currentSeqNum = (((tPlkFrame*)pHistory->aHistoryFrame[ackIndex])->data.asnd.payload.sdoSequenceFrame.sendSeqNumCon & SEQ_NUM_MASK);
            if (((recvSeqNumber_p - currentSeqNum) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
            {
                pHistory->aFrameSize[ackIndex] = 0;
                pHistory->afFrameFirstTxFailed[ackIndex] = FALSE;
                ackIndex++;
                pHistory->freeEntries++;
                if (ackIndex == SDO_HISTORY_SIZE)
                {
                    ackIndex = 0;
                }
            }
            else
            {   // nothing to do anymore, because any further frame in history
                // has larger sequence number than the acknowledge
                return ret;
            }
        }

        while ((((recvSeqNumber_p - 1 - currentSeqNum) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD) &&
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

\param  pSdoSeqCon_p        Pointer to connection control structure.
\param  ppFrame_p           Pointer to store the pointer of the frame.
\param  pSize_p             Pointer to store the size of the frame
\param  fInitRead_p         Indicates the start of a retransmission. If TRUE,
                            it returns the last, not acknowledged frame.

\return The function returns a tOplkError error code.
\retval kErrorOk            No error
\retval kErrorRetry         If the frame has never been sent successfully
*/
//------------------------------------------------------------------------------
static tOplkError readFromHistory(tSdoSeqCon* pSdoSeqCon_p, tPlkFrame** ppFrame_p,
                                  UINT* pSize_p, BOOL fInitRead_p)
{
    tOplkError              ret = kErrorOk;
    tSdoSeqConHistory*      pHistory;

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

        DEBUG_LVL_SDO_TRACE("readFromHistory(): init = %d, read = %u, write = %u, ack = %u",
                             (int)fInitRead_p, (WORD)pHistory->readIndex,
                             (WORD)pHistory->writeIndex, (WORD)pHistory->ackIndex);
        DEBUG_LVL_SDO_TRACE(", free entries = %u, next frame size = %u\n",
                             (WORD)pHistory->freeEntries, pHistory->aFrameSize[pHistory->readIndex]);

        // return pointer to stored frame
        *ppFrame_p = (tPlkFrame*)pHistory->aHistoryFrame[pHistory->readIndex];
        *pSize_p = pHistory->aFrameSize[pHistory->readIndex];   // save size
        pHistory->readIndex++;
        if (pHistory->readIndex == SDO_HISTORY_SIZE)
        {
            pHistory->readIndex = 0;
        }
    }
    else
    {
        DEBUG_LVL_SDO_TRACE("readFromHistory(): read = %u, write = %u, ack = %u, free entries = %u, no frame\n",
                            (WORD)pHistory->readIndex, (WORD)pHistory->writeIndex, (WORD)pHistory->ackIndex, (WORD)pHistory->freeEntries);

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

\param  pSdoSeqCon_p        Pointer to connection control structure.

\return The function returns the number of free history entries.
*/
//------------------------------------------------------------------------------
static UINT getFreeHistoryEntries(tSdoSeqCon* pSdoSeqCon_p)
{
    UINT freeEntries;

    freeEntries = (UINT)pSdoSeqCon_p->sdoSeqConHistory.freeEntries;
    return freeEntries;
}

//------------------------------------------------------------------------------
/**
\brief  Set a timer

The function sets up a timer with the specified timeout for the connection.

\param  pSdoSeqCon_p        Pointer to connection control structure.
\param  timeout_p           Timeout to set in milliseconds.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setTimer(tSdoSeqCon* pSdoSeqCon_p, ULONG timeout_p)
{
    tOplkError          ret;
    tTimerArg           timerArg;

    timerArg.eventSink = kEventSinkSdoAsySeq;
    timerArg.argument.pValue = pSdoSeqCon_p;

    if (pSdoSeqCon_p->timerHandle == 0)
    {   // create new timer
        ret = timeru_setTimer(&pSdoSeqCon_p->timerHandle, timeout_p, timerArg);
    }
    else
    {   // modify existing timer
        ret = timeru_modifyTimer(&pSdoSeqCon_p->timerHandle, timeout_p, timerArg);
    }
    return ret;
}

/// \}
