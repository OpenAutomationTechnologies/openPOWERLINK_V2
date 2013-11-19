/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for asynchronous SDO Sequence Layer module

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

  2006/06/26 k.t.:   start of the implementation

****************************************************************************/

#include <user/sdoseq.h>

#if ((((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) == 0) &&\
     (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) == 0)   )

    #error 'ERROR: At least UDP or Asnd module needed!'

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

#define SDO_HISTORY_SIZE            5

#ifndef MAX_SDO_SEQ_CON
#define MAX_SDO_SEQ_CON             5
#endif

#define SDO_SEQ_DEFAULT_TIMEOUT     5000                    // in [ms] => 5 sec
#define SDO_SEQ_RETRY_COUNT         5                       // => max. Timeout 30 sec
#define SDO_SEQ_NUM_THRESHOLD       100                     // threshold which distinguishes between old and new sequence numbers
#define SDO_SEQ_FRAME_SIZE          24                      // frame with size of Asnd-Header-, SDO Sequence header size, SDO Command header and Ethernet-header size
#define SDO_SEQ_HEADER_SIZE         4                       // size of the header of the SDO Sequence layer
#define SDO_SEQ_HISTROY_FRAME_SIZE  SDO_MAX_FRAME_SIZE      // buffersize for one frame in history
#define SDO_CON_MASK                0x03                    // mask to get scon and rcon

const UINT32 SDO_SEQU_MAX_TIMEOUT_MS = (UINT32) 86400000UL; // [ms], 86400000 ms = 1 day

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

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
}tSdoSeqEvent;

/**
\brief  SDO sequence layer connection history

This structure defines the SDO sequence layer connection history buffer.
*/
typedef struct
{
    UINT8           freeEntries;    ///< Number of free history entries
    UINT8           writeIndex;     ///< Index of the next free buffer entry
    UINT8           ackIndex;       ///< Index of the next message which should become acknowledged
    UINT8           readIndex;      ///< Index between ackIndex and writeIndex to the next message for retransmission
    UINT8           aHistoryFrame[SDO_HISTORY_SIZE][SDO_SEQ_HISTROY_FRAME_SIZE];
    UINT            aFrameSize[SDO_HISTORY_SIZE];
}tSdoSeqConHistory;

/**
\brief  SDO sequence layer states

This enumeration defines all valid states for the SDO sequence layer state
machine.
*/
typedef enum
{
    kSdoSeqStateIdle        = 0x00,
    kSdoSeqStateInit1       = 0x01,
    kSdoSeqStateInit2       = 0x02,
    kSdoSeqStateInit3       = 0x03,
    kSdoSeqStateConnected   = 0x04,
    kSdoSeqStateWaitAck     = 0x05
}tSdoSeqState;

/**
\brief  SDO sequence layer connection control structure

This structure is used by the SDO sequence layer to save connection information.
*/
typedef struct
{
    tSdoConHdl              conHandle;          ///< Connection handle
    tSdoSeqState            sdoSeqState;        ///< State of the connection
    UINT8                   recvSeqNum;         ///< Receive sequence number
    UINT8                   sendSeqNum;         ///< Send sequence number
    tSdoSeqConHistory       sdoSeqConHistory;   ///< Connection history buffer
    tEplTimerHdl            timerHandle;        ///< Timer handle
    UINT                    retryCount;         ///< Retry counter
    UINT                    useCount;           ///< One sequence layer connection may be used by multiple command layer connections
}tSdoSeqCon;

/**
\brief  SDO sequence layer instance structure

This structure defines a SDO sequence layer instance.
*/
typedef struct
{
    tSdoSeqCon              aSdoSeqCon[MAX_SDO_SEQ_CON];    ///< Array of sequence layer connections
    tSdoComReceiveCb        pfnSdoComRecvCb;                ///< Pointer to receive callback function
    tSdoComConCb            pfnSdoComConCb;                 ///< Pointer to connection callback function
    UINT32                  sdoSeqTimeout;                  ///< Configured Sequence layer timeout

#if defined(WIN32) || defined(_WIN32)
    LPCRITICAL_SECTION      pCriticalSection;
    CRITICAL_SECTION        criticalSection;

    LPCRITICAL_SECTION      pCriticalSectionReceive;
    CRITICAL_SECTION        criticalSectionReceive;
#endif
}tSdoSeqInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tSdoSeqInstance   sdoSeqInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel processState(UINT handle_p, UINT dataSize_p, tEplFrame* pData_p,
                               tAsySdoSeq* pRecFrame_p, tSdoSeqEvent event_p);

static tEplKernel sendFrame(tSdoSeqCon* pAsySdoSeqCon_p, UINT dataSize_p,
                            tEplFrame* pData_p, BOOL fFrameInHistory_p);

static tEplKernel sendToLowerLayer(tSdoSeqCon* pAsySdoSeqCon_p, UINT dataSize_p, tEplFrame* pFrame_p);

static tEplKernel receiveCb(tSdoConHdl conHdl_p, tAsySdoSeq* pSdoSeqData_p, UINT dataSize_p);

static tEplKernel initHistory(tSdoSeqCon* pAsySdoSeqCon_p);

static tEplKernel addFrameToHistory(tSdoSeqCon* pAsySdoSeqCon_p, tEplFrame* pFrame_p, UINT size_p);

static tEplKernel deleteAckedFrameFromHistory(tSdoSeqCon* pAsySdoSeqCon_p, UINT8 recvSeqNumber_p);

static tEplKernel readFromHistory(tSdoSeqCon* pAsySdoSeqCon_p, tEplFrame** ppFrame_p,
                                  UINT* pSize_p, BOOL fInitRead_p);

static UINT       getFreeHistoryEntries(tSdoSeqCon* pAsySdoSeqCon_p);

static tEplKernel setTimer(tSdoSeqCon* pAsySdoSeqCon_p, ULONG timeout_p);

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <EPL asynchronous SDO Sequence layer>                 */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: this module contains the asynchronous SDO Sequence Layer for
//              the EPL SDO service
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    sdoseq_init
//
// Description: init first instance
//
//
//
// Parameters:  fpSdoComCb_p    = callback function to inform Command layer
//                                about new frames
//              fpSdoComConCb_p = callback function to inform command layer
//                                about connection state
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_init(tSdoComReceiveCb pfnSdoComRecvCb_p, tSdoComConCb pfnSdoComConCb_p)
{
tEplKernel  Ret;


    Ret = sdoseq_addInstance(pfnSdoComRecvCb_p, pfnSdoComConCb_p);

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    sdoseq_addInstance
//
// Description: init following instances
//
//
//
// Parameters:  fpSdoComCb_p    = callback function to inform Command layer
//                                about new frames
//              fpSdoComConCb_p = callback function to inform command layer
//                                about connection state
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_addInstance(tSdoComReceiveCb pfnSdoComRecvCb_p, tSdoComConCb pfnSdoComConCb_p)
{
    tEplKernel      Ret;

    Ret = kEplSuccessful;

    // check function pointer
    if(pfnSdoComRecvCb_p == NULL)
    {
        Ret = kEplSdoSeqMissCb;
        goto Exit;
    }
    else
    {
        sdoSeqInstance_l.pfnSdoComRecvCb = pfnSdoComRecvCb_p;
    }

    // check function pointer
    if(pfnSdoComConCb_p == NULL)
    {
        Ret = kEplSdoSeqMissCb;
        goto Exit;
    }
    else
    {
        sdoSeqInstance_l.pfnSdoComConCb = pfnSdoComConCb_p;
    }

    // set control structure to 0
    EPL_MEMSET(&sdoSeqInstance_l.aSdoSeqCon[0], 0x00, sizeof(sdoSeqInstance_l.aSdoSeqCon));

#if defined(WIN32) || defined(_WIN32)
    // create critical section for process function
    sdoSeqInstance_l.pCriticalSection = &sdoSeqInstance_l.criticalSection;
    InitializeCriticalSection(sdoSeqInstance_l.pCriticalSection);

    // init critical section for receive cb function
    sdoSeqInstance_l.pCriticalSectionReceive = &sdoSeqInstance_l.criticalSectionReceive;
    InitializeCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
    // init lower layer
    Ret = sdoudp_addInstance(receiveCb);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
    // init lower layer
    Ret = sdoasnd_addInstance(receiveCb);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    sdoSeqInstance_l.sdoSeqTimeout   = SDO_SEQ_DEFAULT_TIMEOUT;

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    sdoseq_delInstance
//
// Description: delete instances
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_delInstance(void)
{
tEplKernel  Ret;
unsigned int        uiCount;
tSdoSeqCon*   pAsySdoSeqCon;

    Ret = kEplSuccessful;

    // delete timer of open connections
    uiCount = 0;
    pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[0];
    while(uiCount < MAX_SDO_SEQ_CON)
    {
        if (pAsySdoSeqCon->conHandle != 0)
        {
            EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
        }
        uiCount++;
        pAsySdoSeqCon++;
    }


#if defined(WIN32) || defined(_WIN32)
    // delete critical section for process function
    DeleteCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    // set instance-table to 0
    EPL_MEMSET(&sdoSeqInstance_l, 0x00, sizeof(sdoSeqInstance_l));

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
    // delete lower layer
    Ret = sdoudp_delInstance();
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
    // delete lower layer
    Ret = sdoasnd_delInstance();
#endif

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    sdoseq_initCon
//
// Description: start initialization of a sequence layer connection.
//              It tries to reuse an existing connection to the same node.
//
//
// Parameters:  pSdoSeqConHdl_p = pointer to the variable for the connection handle
//              uiNodeId_p      = Node Id of the target
//              SdoType          = Type of the SDO connection
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_initCon(tSdoSeqConHdl* pSdoSeqConHdl_p, UINT nodeId_p, tSdoType sdoType_p)
{
tEplKernel          Ret;
unsigned int        uiCount;
unsigned int        uiFreeCon;
tSdoConHdl       ConHandle = ~0U;
tSdoSeqCon*   pAsySdoSeqCon;
    Ret = kEplSuccessful;

    // check SdoType
    // call init function of the protocol abstraction layer
    // which tries to find an existing connection to the same node
    switch (sdoType_p)
    {
        // SDO over UDP
        case kSdoTypeUdp:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
            Ret = sdoudp_initCon(&ConHandle,
                                    nodeId_p);
            if(Ret != kEplSuccessful)
            {
                goto Exit;
            }
#else
            Ret = kEplSdoSeqUnsupportedProt;
            goto Exit;
#endif
            break;
        }

        // SDO over Asnd
        case kSdoTypeAsnd:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
            Ret = sdoasnd_initCon(&ConHandle,
                                    nodeId_p);
            if(Ret != kEplSuccessful)
            {
                goto Exit;
            }
#else
            Ret = kEplSdoSeqUnsupportedProt;
            goto Exit;
#endif
            break;
        }

        // unsupported protocols
        // -> auto should be replaced by command layer
        case kSdoTypeAuto:
        case kSdoTypePdo:
        default:
        {
            Ret = kEplSdoSeqUnsupportedProt;
            goto Exit;
        }

    }// end of switch(SdoType)


    // find existing connection to the same node or find empty entry for connection
    uiCount = 0;
    uiFreeCon = MAX_SDO_SEQ_CON;
    pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[0];

    while (uiCount < MAX_SDO_SEQ_CON)
    {
        if (pAsySdoSeqCon->conHandle == ConHandle)
        {   // existing connection found
            break;
        }
        if (pAsySdoSeqCon->conHandle == 0)
        {
            uiFreeCon = uiCount;
        }
        uiCount++;
        pAsySdoSeqCon++;
    }

    if (uiCount == MAX_SDO_SEQ_CON)
    {
        if (uiFreeCon == MAX_SDO_SEQ_CON)
        {   // no free entry found
            switch (sdoType_p)
            {
                // SDO over UDP
                case kSdoTypeUdp:
                {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
                    Ret = sdoudp_delConnection(ConHandle);
                    if(Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
#endif
                    break;
                }

                // SDO over Asnd
                case kSdoTypeAsnd:
                {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
                    Ret = sdoasnd_deleteCon(ConHandle);
                    if(Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
#endif
                    break;
                }

                // unsupported protocols
                // -> auto should be replaced by command layer
                case kSdoTypeAuto:
                case kSdoTypePdo:
                default:
                {
                    Ret = kEplSdoSeqUnsupportedProt;
                    goto Exit;
                }

            }// end of switch(SdoType)

            Ret = kEplSdoSeqNoFreeHandle;
            goto Exit;
        }
        else
        {   // free entry found
            pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[uiFreeCon];
            pAsySdoSeqCon->conHandle = ConHandle;
            // increment use counter
            pAsySdoSeqCon->useCount++;

            uiCount = uiFreeCon;
        }
    }

    // set handle
    *pSdoSeqConHdl_p = (uiCount | SDO_ASY_HANDLE);

    // call intern process function
    Ret = processState(uiCount,
                                0,
                                NULL,
                                NULL,
                                kSdoSeqEventInitCon);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    sdoseq_sendData
//
// Description: send data using an established connection
//
//
//
// Parameters:  pSdoSeqConHdl_p = connection handle
//              uiDataSize_p    = Size of Frame to send
//                                  -> without SDO sequence layer header, Asnd header
//                                     and ethernet
//                                  ==> SDO Sequence layer payload
//              SdoType          = Type of the SDO connection
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_sendData(tSdoSeqConHdl sdoSeqConHdl_p, UINT dataSize_p, tEplFrame* pData_p )
{
tEplKernel      Ret;
unsigned int    uiHandle;



    uiHandle = (sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

    // check if connection ready
    if(sdoSeqInstance_l.aSdoSeqCon[uiHandle].sdoSeqState == kSdoSeqStateIdle )
    {
        // no connection with this handle
        Ret = kEplSdoSeqInvalidHdl;
        goto Exit;
    }
    else if(sdoSeqInstance_l.aSdoSeqCon[uiHandle].sdoSeqState != kSdoSeqStateConnected)
    {
        Ret = kEplSdoSeqConnectionBusy;
        goto Exit;
    }

    Ret = processState(uiHandle,
                                dataSize_p,
                                pData_p,
                                NULL,
                                kSdoSeqEventFrameSend);
Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    sdoseq_processEvent
//
// Description: function processes external events
//              -> later needed for timeout control with timer-module
//
//
//
// Parameters:  pEvent_p = pointer to event
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_processEvent(tEplEvent* pEvent_p)
{
tEplKernel          Ret;
tEplTimerEventArg*  pTimerEventArg;
tSdoSeqCon*   pAsySdoSeqCon;
tEplTimerHdl        EplTimerHdl;
unsigned int        uiCount;

    Ret = kEplSuccessful;
    // check parameter
    if(pEvent_p == NULL)
    {
        Ret = kEplSdoSeqInvalidEvent;
        goto Exit;
    }

    if(pEvent_p->m_EventType != kEplEventTypeTimer)
    {
        Ret = kEplSdoSeqInvalidEvent;
        goto Exit;
    }

    // get timerhdl
    pTimerEventArg = (tEplTimerEventArg*)pEvent_p->m_pArg;
    EplTimerHdl = pTimerEventArg->m_TimerHdl;

    // get pointer to intern control structure of connection
    if (pTimerEventArg->m_Arg.m_pVal == NULL)
    {
        goto Exit;
    }
    pAsySdoSeqCon = (tSdoSeqCon*)pTimerEventArg->m_Arg.m_pVal;

    // check if time is current
    if(EplTimerHdl != pAsySdoSeqCon->timerHandle)
    {
        // delete timer
        EplTimeruDeleteTimer(&EplTimerHdl);
        goto Exit;
    }

    // delete timer
    EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);

    // get indexnumber of control structure
    uiCount = 0;
    while((&sdoSeqInstance_l.aSdoSeqCon[uiCount]) != pAsySdoSeqCon)
    {
        uiCount++;
        if(uiCount > MAX_SDO_SEQ_CON)
        {
            goto Exit;
        }
    }


    // process event and call process function if needed
    Ret = processState(uiCount,
                                0,
                                NULL,
                                NULL,
                                kSdoSeqEventTimeout);

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    sdoseq_deleteCon
//
// Description: del and close one connection
//
//
//
// Parameters:  SdoSeqConHdl_p = handle of connection
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_deleteCon(tSdoSeqConHdl sdoSeqConHdl_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiHandle;
tSdoSeqCon*   pAsySdoSeqCon;

    uiHandle = (sdoSeqConHdl_p & ~SDO_SEQ_HANDLE_MASK);

    // check if handle invalid
    if(uiHandle >= MAX_SDO_SEQ_CON)
    {
        Ret = kEplSdoSeqInvalidHdl;
        goto Exit;
    }

    // get pointer to connection
    pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[uiHandle];

    // decrement use counter
    pAsySdoSeqCon->useCount--;

    if (pAsySdoSeqCon->useCount == 0)
    {
        // process close in process function
        Ret = processState(uiHandle,
                                    0,
                                    NULL,
                                    NULL,
                                    kSdoSeqEventCloseCon);

        //check protocol
        if((pAsySdoSeqCon->conHandle & SDO_ASY_HANDLE_MASK) == SDO_UDP_HANDLE)
        {
        #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
            // call close function of lower layer
            sdoudp_delConnection(pAsySdoSeqCon->conHandle);
        #endif// end of #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
        }
        else
        {
        #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
            // call close function of lower layer
            sdoasnd_deleteCon(pAsySdoSeqCon->conHandle);
        #endif// end of #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
        }

        // delete timer
        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);

        // clean control structure
        EPL_MEMSET(pAsySdoSeqCon, 0x00, sizeof(tSdoSeqCon));
        pAsySdoSeqCon->sdoSeqConHistory.freeEntries = SDO_HISTORY_SIZE;
    }

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    sdoseq_setTimeout
//
// Description: Set new SDO sequence layer timeout
//
// Parameters:  Timeout_p ... New timeout [ms]
//
// Returns:     tEplKernel = errorcode
//
//---------------------------------------------------------------------------
tEplKernel sdoseq_setTimeout(UINT32 timeout_p)
{
    // Adopt new SDO sequence layer timeout (truncated to an upper bound)
    sdoSeqInstance_l.sdoSeqTimeout   = min(timeout_p, SDO_SEQU_MAX_TIMEOUT_MS);

    return  kEplSuccessful;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    processState
//
// Description: intern function to process the asynchronous SDO Sequence Layer
//              state machine
//
//
//
// Parameters:  uiHandle_p      = index of the control structure of the connection
//              uiDataSize_p    = size of data frame to process (can be 0)
//                                  -> without size of sequence header and Asnd header!!!
//
//              pData_p         = pointer to frame to send (can be NULL)
//              pRecFrame_p     = pointer to received frame (can be NULL)
//              Event_p         = Event to process
//
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel processState(UINT handle_p, UINT dataSize_p, tEplFrame* pData_p,
                               tAsySdoSeq* pRecFrame_p, tSdoSeqEvent Event_p)

{
tEplKernel          Ret;
unsigned int        uiFrameSize;
tEplFrame*          pEplFrame;
tSdoSeqCon*   pAsySdoSeqCon;
tSdoSeqConHdl    SdoSeqConHdl;
unsigned int        uiFreeEntries;

#if defined(WIN32) || defined(_WIN32)
    // enter  critical section for process function
    EnterCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif

    Ret = kEplSuccessful;

    // get handle for hinger layer
    SdoSeqConHdl = handle_p | SDO_ASY_HANDLE;

    // check if handle invalid
    if((SdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
    {
        Ret = kEplSdoSeqInvalidHdl;
        goto Exit;
    }

    // get pointer to connection
    pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[handle_p];

    // check size
    if((pData_p == NULL)&& (pRecFrame_p == NULL) && (dataSize_p != 0))
    {
        Ret = kEplSdoSeqInvalidFrame;
        goto Exit;
    }

    // check state
    switch(pAsySdoSeqCon->sdoSeqState)
    {
        // idle state
        case kSdoSeqStateIdle:
        {
            // check event
            switch(Event_p)
            {
                // new connection
                // -> send init frame and change to
                // kSdoSeqStateInit1
                case kSdoSeqEventInitCon:
                {
                    // set sending scon to 1
                    pAsySdoSeqCon->recvSeqNum = 0x01;
                    // set set send rcon to 0
                    pAsySdoSeqCon->sendSeqNum = 0x00;

                    Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                    if(Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    // change state
                    pAsySdoSeqCon->sdoSeqState = kSdoSeqStateInit1;

                    // set timer
                    Ret = setTimer(pAsySdoSeqCon,
                            sdoSeqInstance_l.sdoSeqTimeout);

                    break;
                }

                // init con from extern
                // check rcon and scon
                // -> send answer
                case kSdoSeqEventFrameRec:
                {

                    DEBUG_LVL_25_TRACE("%s scon=%u rcon=%u\n",
                            __func__,
                            pRecFrame_p->m_le_bSendSeqNumCon,
                            pRecFrame_p->m_le_bRecSeqNumCon);

                    // check if scon == 1 and rcon == 0
                    if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x00)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x01))
                    {
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                        // create answer and send answer
                        // set rcon to 1 (in send direction own scon)
                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // change state to kSdoSeqStateInit2
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateInit2;

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);
                    }
                    else
                    {   // error -> close
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        if (((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) != 0x00)
                            || ((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) != 0x00))
                        {   // d.k. only answer with close message if the message sent was not a close message
                            // save sequence numbers
                            pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                            pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                            // set rcon and scon to 0
                            pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                            pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                            // send frame
                            sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                        }

                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateInitError);
                    }
                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            }// end of switch(Event_p)
            break;
        }

        // init connection step 1
        // wait for frame with scon = 1
        // and rcon = 1
        case kSdoSeqStateInit1:
        {
//            PRINTF("EplSdoAsySequ: StateInit1\n");

            // check event
            switch(Event_p)
            {
                // frame received
                case kSdoSeqEventFrameRec:
                {
                    // check scon == 1 and rcon == 1
                    if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x01)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x01))
                    {   // create answer own scon = 2
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // change state to kSdoSeqStateInit3
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateInit3;

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);

                    }
                    // check if scon == 1 and rcon == 0, i.e. other side wants me to be server
                    else if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x00)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x01))
                    {
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                        // create answer and send answer
                        // set rcon to 1 (in send direction own scon)
                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // change state to kSdoSeqStateInit2
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateInit2;

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);
                    }
                    else
                    {   // error -> Close
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        if (((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) != 0x00)
                            || ((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) != 0x00))
                        {   // d.k. only answer with close message if the message sent was not a close message
                            // save sequence numbers
                            pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                            pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                            // set rcon and scon to 0
                            pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                            pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                            // send frame
                            sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                        }
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateInitError);
                    }
                    break;
                }

                // timeout
                case kSdoSeqEventTimeout:
                {   // error -> Close
                    pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;

                    // set rcon and scon to 0
                    pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                    pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                    // send frame
                    sendFrame(pAsySdoSeqCon,
                                                0,
                                                NULL,
                                                FALSE);
                    // call Command Layer Cb
                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateInitError);
                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            }// end of switch(Event_p)
            break;
        }

        // init connection step 2
        case kSdoSeqStateInit2:
        {
//            PRINTF("EplSdoAsySequ: StateInit2\n");

            // check event
            switch(Event_p)
            {
                // frame received
                case kSdoSeqEventFrameRec:
                {
                    // check scon == 2 and rcon == 1
                    if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x01)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x02))
                    {   // create answer own rcon = 2
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // change state to kSdoSeqStateConnected
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateConnected;

                        // init History
                        Ret = initHistory(pAsySdoSeqCon);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);

                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateConnected);

                    }
                    // check scon == 1 and rcon == 1, i.e. other side wants me to initiate the connection
                    else if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x01)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x01))
                    {
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                        // create answer and send answer
                        // set rcon to 1 (in send direction own scon)
                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);
                        // change state to kSdoSeqStateInit3
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateInit3;

                    }
                    else
                    {   // error -> Close
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        if (((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) != 0x00)
                            || ((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) != 0x00))
                        {   // d.k. only answer with close message if the message sent was not a close message
                            // save sequence numbers
                            pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                            pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                            // set rcon and scon to 0
                            pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                            pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                            // send frame
                            sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                        }
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateInitError);
                    }
                    break;
                }

                // timeout
                case kSdoSeqEventTimeout:
                {   // error -> Close
                    pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                    // set rcon and scon to 0
                    pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                    pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                    // send frame
                    sendFrame(pAsySdoSeqCon,
                                                0,
                                                NULL,
                                                FALSE);

                    // call Command Layer Cb
                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateInitError);
                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            }// end of switch(Event_p)
            break;
        }

        // init connection step 3
        case kSdoSeqStateInit3:
        {
            // check event
            switch(Event_p)
            {
                // frame received
                case kSdoSeqEventFrameRec:
                {
                    // check scon == 2 and rcon == 2
                    if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x02)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x02))
                    {
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                        // change state to kSdoSeqStateConnected
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateConnected;

                        // init History
                        Ret = initHistory(pAsySdoSeqCon);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateConnected);

                    }
                    // check scon == 2 and rcon == 1
                    else if(((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) == 0x01)
                        &&((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) == 0x02))
                    {   // create answer own rcon = 2
                        // save sequence numbers
                        pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                        pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                        pAsySdoSeqCon->recvSeqNum++;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        // change state to kSdoSeqStateConnected
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateConnected;

                        // init History
                        Ret = initHistory(pAsySdoSeqCon);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);

                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateConnected);

                    }
                    else
                    {   // error -> Close
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        if (((pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK) != 0x00)
                            || ((pRecFrame_p->m_le_bSendSeqNumCon & SDO_CON_MASK) != 0x00))
                        {   // d.k. only answer with close message if the message sent was not a close message
                            // save sequence numbers
                            pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                            pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);
                            // set rcon and scon to 0
                            pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                            pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                            // send frame
                            sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                        }
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateInitError);
                    }
                    break;
                }

                // timeout
                case kSdoSeqEventTimeout:
                {   // error -> Close
                    pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                    // set rcon and scon to 0
                    pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                    pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                    // send frame
                    sendFrame(pAsySdoSeqCon,
                                                0,
                                                NULL,
                                                FALSE);

                    // call Command Layer Cb
                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateInitError);
                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            }// end of switch(Event_p)
            break;
        }

        // connection established
        case kSdoSeqStateConnected:
        {
            // check event
            switch(Event_p)
            {

                // frame to send
                case kSdoSeqEventFrameSend:
                {
                    // set timer
                    Ret = setTimer(pAsySdoSeqCon,
                            sdoSeqInstance_l.sdoSeqTimeout);
                    // check if data frame or ack
                    if(pData_p == NULL)
                    {   // send ack
                        // inc scon
                        //pAsySdoSeqCon->recvSeqNum += 4;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);
                        if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }
                    else
                    {   // send dataframe
                        // increment send sequence number
                        pAsySdoSeqCon->recvSeqNum += 4;
                        Ret = sendFrame(pAsySdoSeqCon,
                                                 dataSize_p,
                                                 pData_p,
                                                 TRUE);
                        if(Ret == kEplSdoSeqRequestAckNeeded)
                        { // request ack
                            // change state to wait ack
                            pAsySdoSeqCon->sdoSeqState = kSdoSeqStateWaitAck;
                            // set Ret to kEplSuccessful, because no error
                            // for higher layer
                            Ret = kEplSuccessful;

                        }
                        else if(Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                        else
                        {
                            // call Command Layer Cb
                            sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateFrameSended);
                        }
                    }
                    break;
                }// end of case kSdoSeqEventFrameSend

                // frame received
                case kSdoSeqEventFrameRec:
                {
                BYTE bSendSeqNumCon = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                    // set timer
                    Ret = setTimer(pAsySdoSeqCon,
                            sdoSeqInstance_l.sdoSeqTimeout);
                    // check scon
                    switch (bSendSeqNumCon & SDO_CON_MASK)
                    {
                        // close from other node
                        case 0:
                        {
                            // return to idle
                            pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                            // delete timer
                            EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                            // call Command Layer Cb
                            sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                    kAsySdoConStateConClosed);

                            break;
                        }

                        case 1:
                        {
                            // return to idle
                            pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                            // delete timer
                            EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                            // call Command Layer Cb
                            sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                    kAsySdoConStateTransferAbort);

                            // restart immediately with initialization request
                            DEBUG_LVL_25_TRACE("EplSdoAsySequ: Reinit immediately\n");
                            Ret = kEplRetry;
                            break;
                        }

                        // Request Ack or Error Ack
                        // possible contain data
                        case 3:
                        // normal frame
                        case 2:
                        {
                            if ((AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon) & SDO_CON_MASK) == 3)
                            {
//                                PRINTF("EplSdoAsySequ: error response received\n");

                                // error response (retransmission request)
                                // resend frames from history

                                // read frame from history
                                Ret = readFromHistory(pAsySdoSeqCon,
                                                                &pEplFrame,
                                                                &uiFrameSize,
                                                                TRUE);
                                if (Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }

                                while ((pEplFrame != NULL)
                                       && (uiFrameSize != 0))
                                {
                                    // send frame
                                    Ret = sendToLowerLayer(pAsySdoSeqCon,
                                                        uiFrameSize,
                                                        pEplFrame);
                                    if(Ret != kEplSuccessful)
                                    {
                                        goto Exit;
                                    }

                                    // read next frame from history
                                    Ret = readFromHistory(pAsySdoSeqCon,
                                                                    &pEplFrame,
                                                                    &uiFrameSize,
                                                                    FALSE);
                                    if(Ret != kEplSuccessful)
                                    {
                                        goto Exit;
                                    }
                                } // end of while((pabFrame != NULL)
                            }   // end of if (error response)

                            if (((pAsySdoSeqCon->sendSeqNum + 4) & SEQ_NUM_MASK) == (bSendSeqNumCon & SEQ_NUM_MASK))
                            {   // next frame of sequence received
                                // save send sequence number (without ack request)
                                pAsySdoSeqCon->sendSeqNum = bSendSeqNumCon & ~0x01;

                                // check if ack or data-frame
                                //ignore ack -> already processed
                                if(dataSize_p > SDO_SEQ_HEADER_SIZE)
                                {
                                    sdoSeqInstance_l.pfnSdoComRecvCb(
                                                        SdoSeqConHdl,
                                                        ((tAsySdoCom*) &pRecFrame_p->m_le_abSdoSeqPayload),
                                                        (dataSize_p - SDO_SEQ_HEADER_SIZE));
                                    // call Command Layer Cb
                                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                                kAsySdoConStateFrameSended);


                                }
                                else
                                {
                                    // call Command Layer Cb
                                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                        kAsySdoConStateAckReceived);
                                }
                            }
                            else if (((bSendSeqNumCon - pAsySdoSeqCon->sendSeqNum - 4) & SEQ_NUM_MASK) < SDO_SEQ_NUM_THRESHOLD)
                            {   // frame of sequence was lost,
                                // because difference of received and old value
                                // is less then halve of the values range.

                                // send error frame with own rcon = 3
                                pAsySdoSeqCon->sendSeqNum |= 0x03;
                                Ret = sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                                // restore send sequence number
                                pAsySdoSeqCon->sendSeqNum = (pAsySdoSeqCon->sendSeqNum & SEQ_NUM_MASK) | 0x02;
                                if(Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }

                                // break here, because a requested acknowledge
                                // was sent implicitly above
                                break;
                            }
                            // else, ignore repeated frame

                            if ((bSendSeqNumCon & SDO_CON_MASK) == 3)
                            {   // ack request received

                                // create ack with own scon = 2
                                Ret = sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                                if(Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }
                            }

                            break;
                        }

                    } // switch(pAsySdoSeqCon->sendSeqNum & SDO_CON_MASK)
                    break;
                } // end of case kSdoSeqEventFrameRec:


                //close event from higher layer
                case kSdoSeqEventCloseCon:
                {
                    pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                    // set rcon and scon to 0
                    pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                    pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                    // send frame
                    sendFrame(pAsySdoSeqCon,
                                                 0,
                                                 NULL,
                                                 FALSE);

                    // delete timer
                    EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                    // call Command Layer Cb is not necessary, because the event came from there
//                    sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
//                                                            kAsySdoConStateInitError);
                    break;
                }

                // timeout
                case kSdoSeqEventTimeout:
                {

                    uiFreeEntries = getFreeHistoryEntries(pAsySdoSeqCon);
                    if ((uiFreeEntries < SDO_HISTORY_SIZE)
                        && (pAsySdoSeqCon->retryCount < SDO_SEQ_RETRY_COUNT))
                    {   // unacknowledged frames in history
                        // and retry counter not exceeded

                        // resend data with acknowledge request

                        // increment retry counter
                        pAsySdoSeqCon->retryCount++;

                        // set timer
                        Ret = setTimer(pAsySdoSeqCon,
                                sdoSeqInstance_l.sdoSeqTimeout);

                        // read first frame from history
                        Ret = readFromHistory(pAsySdoSeqCon,
                                                        &pEplFrame,
                                                        &uiFrameSize,
                                                        TRUE);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        if ((pEplFrame != NULL)
                               && (uiFrameSize != 0))
                        {

                            // set ack request in scon
                            AmiSetByteToLe( &pEplFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_bSendSeqNumCon,
                                    AmiGetByteFromLe( &pEplFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_bSendSeqNumCon) | 0x03);

                            // send frame
                            Ret = sendToLowerLayer(pAsySdoSeqCon,
                                                uiFrameSize,
                                                pEplFrame);
                            if(Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                        }
                    }
                    else
                    {
                        // timeout, because of no traffic -> Close
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // set rcon and scon to 0
                        pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                        pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                        // send frame
                        sendFrame(pAsySdoSeqCon,
                                                    0,
                                                    NULL,
                                                    FALSE);

                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                                kAsySdoConStateTimeout);
                    }

                    break;
                }

                default:
                    // d.k. do nothing
                    break;

            }// end of switch(Event_p)
            break;
        }

        // wait for Acknowledge (history buffer full)
        case kSdoSeqStateWaitAck:
        {
            DEBUG_LVL_25_TRACE("EplSdoAsySequ: StateWaitAck\n");

            // set timer
            Ret = setTimer(pAsySdoSeqCon,
                    sdoSeqInstance_l.sdoSeqTimeout);

            //TODO: retry of acknowledge
            if(Event_p == kSdoSeqEventFrameRec)
            {
                // check rcon
                switch (pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK)
                {
                    // close from other node
                    case 0:
                    {
                        // return to idle
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                kAsySdoConStateConClosed);

                        break;
                    }

                    // reinit from other node
                    case 1:
                    {
                        // return to idle
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                        // delete timer
                        EplTimeruDeleteTimer(&pAsySdoSeqCon->timerHandle);
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                kAsySdoConStateTransferAbort);

                        // restart immediately with initialization request
                        Ret = kEplRetry;
                        break;
                    }

                    // normal frame
                    case 2:
                    {
                        // should be ack
                        // -> change to state kSdoSeqStateConnected
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateConnected;
                        // call Command Layer Cb
                        sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                                kAsySdoConStateAckReceived);
                        // send data to higher layer if needed
                        if(dataSize_p > SDO_SEQ_HEADER_SIZE)
                        {
                            sdoSeqInstance_l.pfnSdoComRecvCb(
                                                SdoSeqConHdl,
                                                ((tAsySdoCom*) &pRecFrame_p->m_le_abSdoSeqPayload),
                                                (dataSize_p - SDO_SEQ_HEADER_SIZE));
                        }
                        break;
                    }

                    // Request Ack or Error Ack
                    case 3:
                    {
                        // -> change to state kSdoSeqStateConnected
                        pAsySdoSeqCon->sdoSeqState = kSdoSeqStateConnected;

                        if(pRecFrame_p->m_le_bRecSeqNumCon == pAsySdoSeqCon->recvSeqNum )
                        {   // ack request
                            // -> send ack
                            // save sequence numbers
                            pAsySdoSeqCon->recvSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bRecSeqNumCon);
                            pAsySdoSeqCon->sendSeqNum = AmiGetByteFromLe(&pRecFrame_p->m_le_bSendSeqNumCon);

                            // create answer own rcon = 2
                            pAsySdoSeqCon->recvSeqNum--;

                            // check if ack or data-frame
                            if(dataSize_p > SDO_SEQ_HEADER_SIZE)
                            {
                                sdoSeqInstance_l.pfnSdoComRecvCb(
                                                    SdoSeqConHdl,
                                                    ((tAsySdoCom*) &pRecFrame_p->m_le_abSdoSeqPayload),
                                                    (dataSize_p - SDO_SEQ_HEADER_SIZE));
                                // call Command Layer Cb
                                sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                            kAsySdoConStateFrameSended);


                            }
                            else
                            {
                                Ret = sendFrame(pAsySdoSeqCon,
                                                        0,
                                                        NULL,
                                                        FALSE);
                                if(Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }
                            }

                        }
                        else
                        {
                            // error ack
                            // resend frames from history

                            // read frame from history
                            Ret = readFromHistory(pAsySdoSeqCon,
                                                            &pEplFrame,
                                                            &uiFrameSize,
                                                            TRUE);
                            while ((pEplFrame != NULL)
                                    && (uiFrameSize != 0))
                            {
                                // send frame
                                Ret = sendToLowerLayer(pAsySdoSeqCon,
                                                    uiFrameSize,
                                                    pEplFrame);
                                if(Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }
                                // read next frame

                                // read frame from history
                                Ret = readFromHistory(pAsySdoSeqCon,
                                                                &pEplFrame,
                                                                &uiFrameSize,
                                                                FALSE);
                            } // end of while((pabFrame != NULL)
                        }
                        break;
                    }
                }// end of switch(pRecFrame_p->m_le_bRecSeqNumCon & SDO_CON_MASK)

            }
            else if(Event_p == kSdoSeqEventTimeout)
            {   // error -> Close
                pAsySdoSeqCon->sdoSeqState = kSdoSeqStateIdle;
                // set rcon and scon to 0
                pAsySdoSeqCon->sendSeqNum &= SEQ_NUM_MASK;
                pAsySdoSeqCon->recvSeqNum &= SEQ_NUM_MASK;
                // send frame
                sendFrame(pAsySdoSeqCon,
                                            0,
                                            NULL,
                                            FALSE);

                // call Command Layer Cb
                sdoSeqInstance_l.pfnSdoComConCb(SdoSeqConHdl,
                                                        kAsySdoConStateTimeout);
            }

            break;
        }

        // unknown state
        default:
        {
            EPL_DBGLVL_SDO_TRACE("Error: Unknown State in processState\n");

        }
    }// end of switch(pAsySdoSeqCon->sdoSeqState)



Exit:

#if defined(WIN32) || defined(_WIN32)
    // leave critical section for process function
    LeaveCriticalSection(sdoSeqInstance_l.pCriticalSection);
#endif
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    sendFrame
//
// Description: intern function to create and send a frame
//              -> if uiDataSize_p == 0 create a frame with infos from
//                 pAsySdoSeqCon_p
//
//
//
// Parameters:  pAsySdoSeqCon_p = pointer to control structure of the connection
//              uiDataSize_p    = size of data frame to process (can be 0)
//                                  -> without size of sequence header and Asnd header!!!
//              pData_p         = pointer to frame to process (can be NULL)
//              fFrameInHistory = if TRUE frame is saved to history else not
//
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel sendFrame(tSdoSeqCon* pAsySdoSeqCon_p, UINT dataSize_p,
                            tEplFrame* pData_p, BOOL fFrameInHistory_p)
{
tEplKernel      Ret;
BYTE            abFrame[SDO_SEQ_FRAME_SIZE];
tEplFrame*      pEplFrame;
unsigned int    uiFreeEntries = 0;

    if (pData_p == NULL)
    {   // set pointer to own frame
        EPL_MEMSET(&abFrame[0], 0x00, sizeof(abFrame));
        pEplFrame = (tEplFrame*)&abFrame[0];
    }
    else
    {   // set pointer to frame from calling function
        pEplFrame = pData_p;
    }

    if (fFrameInHistory_p != FALSE)
    {
        // check if only one free entry in history buffer
        uiFreeEntries = getFreeHistoryEntries(pAsySdoSeqCon_p);
        if (uiFreeEntries <= 1)
        {   // request an acknowledge in dataframe
            // own scon = 3
            pAsySdoSeqCon_p->recvSeqNum |= 0x03;
        }
    }

    // filling header informations
    // set service id sdo
    AmiSetByteToLe( &pEplFrame->m_Data.m_Asnd.m_le_bServiceId, 0x05);
    AmiSetByteToLe( &pEplFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_abReserved,0x00);
    // set receive sequence number and rcon
    AmiSetByteToLe( &pEplFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_bRecSeqNumCon, pAsySdoSeqCon_p->sendSeqNum);
    // set send sequence number and scon
    AmiSetByteToLe( &pEplFrame->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_bSendSeqNumCon, pAsySdoSeqCon_p->recvSeqNum);

    // add size
    dataSize_p += SDO_SEQ_HEADER_SIZE;


    // forward frame to appropriate lower layer
    Ret = sendToLowerLayer(pAsySdoSeqCon_p,
                                     dataSize_p,
                                     pEplFrame);    // pointer to frame

    // check if all alright
    if ((Ret == kEplSuccessful)
        && (fFrameInHistory_p != FALSE))
    {
        // set own scon to 2 if needed
        if ((pAsySdoSeqCon_p->recvSeqNum & 0x03) == 0x03)
        {
            pAsySdoSeqCon_p->recvSeqNum--;
        }

        // save frame to history
        Ret = addFrameToHistory(pAsySdoSeqCon_p,
                                            pEplFrame,
                                            dataSize_p);
        if ((Ret == kEplSdoSeqNoFreeHistory) || (uiFreeEntries <= 1))
        {   // request Ack needed
            Ret = kEplSdoSeqRequestAckNeeded;
        }

    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    sendToLowerLayer
//
// Description: intern function to send a previously created frame to lower layer
//
// Parameters:  pAsySdoSeqCon_p = pointer to control structure of the connection
//              uiDataSize_p    = size of data frame to process (can be 0)
//                                  -> without size of Asnd header!!!
//              pData_p         = pointer to frame to process (can be NULL)
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel sendToLowerLayer(tSdoSeqCon* pAsySdoSeqCon_p, UINT dataSize_p,
                                   tEplFrame* pFrame_p)
{
tEplKernel      Ret;

    // call send-function
    // check handle for UDP or Asnd
    if ((pAsySdoSeqCon_p->conHandle & SDO_ASY_HANDLE_MASK) == SDO_UDP_HANDLE)
    {   // send over UDP
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
        Ret = sdoudp_sendData(pAsySdoSeqCon_p->conHandle,
                                    pFrame_p,      // pointer to frame
                                    dataSize_p);
#else
        Ret = kEplSdoSeqUnsupportedProt;
#endif

    }
    else if ((pAsySdoSeqCon_p->conHandle & SDO_ASY_HANDLE_MASK) == SDO_ASND_HANDLE)
    {   // ASND
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_ASND)) != 0)
        Ret = sdoasnd_sendData(pAsySdoSeqCon_p->conHandle,
                                    pFrame_p,      // pointer to frame
                                    dataSize_p);
#else
        Ret = kEplSdoSeqUnsupportedProt;
#endif
    }
    else
    {   // error
        Ret =  kEplSdoSeqInvalidHdl;
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        receiveCb
//
// Description:     callback-function for received frames from lower layer
//
//
//
// Parameters:      ConHdl_p        = handle of the connection
//                  pSdoSeqData_p   = pointer to frame
//                  uiDataSize_p    = size of frame
//
//
// Returns:         tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel receiveCb(tSdoConHdl conHdl_p, tAsySdoSeq* pSdoSeqData_p,
                            UINT dataSize_p)
{
tEplKernel          Ret;
unsigned int        uiCount;
unsigned int        uiFreeEntry;
tSdoSeqCon*   pAsySdoSeqCon;

    do
    {
        uiCount = 0;
        uiFreeEntry = MAX_SDO_SEQ_CON;

#if defined(WIN32) || defined(_WIN32)
        // enter  critical section
        EnterCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

        EPL_DBGLVL_SDO_TRACE("Handle: 0x%x , First Databyte 0x%x\n", ConHdl_p,((BYTE*)pSdoSeqData_p)[0]);

        // search control structure for this connection
        pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[uiCount];
        while (uiCount < MAX_SDO_SEQ_CON)
        {
            if (pAsySdoSeqCon->conHandle == conHdl_p)
            {
                break;
            }
            else if ((pAsySdoSeqCon->conHandle == 0)
                && (uiFreeEntry == MAX_SDO_SEQ_CON))
            {
                // free entry
                uiFreeEntry = uiCount;
            }
            uiCount++;
            pAsySdoSeqCon++;
        }

        if (uiCount == MAX_SDO_SEQ_CON)
        {   // new connection
            if (uiFreeEntry == MAX_SDO_SEQ_CON)
            {
                Ret = kEplSdoSeqNoFreeHandle;

#if defined(WIN32) || defined(_WIN32)
                // leave critical section
                LeaveCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

                goto Exit;
            }
            else
            {
                pAsySdoSeqCon = &sdoSeqInstance_l.aSdoSeqCon[uiFreeEntry];
                // save handle from lower layer
                pAsySdoSeqCon->conHandle = conHdl_p;
                // increment use counter
                pAsySdoSeqCon->useCount++;
                uiCount = uiFreeEntry;
            }
        }

        // call history ack function
        Ret = deleteAckedFrameFromHistory(pAsySdoSeqCon,
            (AmiGetByteFromLe(&pSdoSeqData_p->m_le_bRecSeqNumCon)& SEQ_NUM_MASK));

#if defined(WIN32) || defined(_WIN32)
        // leave critical section
        LeaveCriticalSection(sdoSeqInstance_l.pCriticalSectionReceive);
#endif

        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // call process function with pointer of frame and event kSdoSeqEventFrameRec
        Ret = processState(uiCount,
                                    dataSize_p,
                                    NULL,
                                    pSdoSeqData_p,
                                    kSdoSeqEventFrameRec);

    } while (Ret == kEplRetry);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        initHistory
//
// Description:     init function for history buffer
//
//
//
// Parameters:
//
//
// Returns:         tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel initHistory(tSdoSeqCon* pAsySdoSeqCon_p)
{
tEplKernel      Ret;

    Ret = kEplSuccessful;

    pAsySdoSeqCon_p->sdoSeqConHistory.freeEntries = SDO_HISTORY_SIZE;
    pAsySdoSeqCon_p->sdoSeqConHistory.ackIndex = 0;
    pAsySdoSeqCon_p->sdoSeqConHistory.writeIndex = 0;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:        addFrameToHistory
//
// Description:     function to add a frame to the history buffer
//
//
//
// Parameters:      pAsySdoSeqCon_p = pointer to control structure of this connection
//                  pFrame_p        = pointer to frame
//                  uiSize_p        = size of the frame
//                                     -> without size of the ethernet header
//                                        and the asnd header
//
// Returns:         tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel addFrameToHistory(tSdoSeqCon* pAsySdoSeqCon_p, tEplFrame* pFrame_p,
                                    UINT size_p)
{
tEplKernel              Ret;
tSdoSeqConHistory*   pHistory;

    Ret = kEplSuccessful;

    // add frame to history buffer

    // check size
    // $$$ d.k. EPL_SEQ_HISTORY_FRAME_SIZE includes the header size, but uiSize_p does not!!!
    if(size_p > SDO_SEQ_HISTROY_FRAME_SIZE)
    {
        Ret = kEplSdoSeqFrameSizeError;
        goto Exit;
    }

    // save pointer to history
    pHistory = &pAsySdoSeqCon_p->sdoSeqConHistory;


    // check if a free entry is available
    if(pHistory->freeEntries > 0)
    {   // write message in free entry
        EPL_MEMCPY(&((tEplFrame*)pHistory->aHistoryFrame[pHistory->writeIndex])->m_le_bMessageType,
                &pFrame_p->m_le_bMessageType,
                size_p + ASND_HEADER_SIZE);
        // store size
        pHistory->aFrameSize[pHistory->writeIndex] = size_p;

        // decrement number of free buffer entries
        pHistory->freeEntries--;

        // increment write index
        pHistory->writeIndex++;

        // check if write-index ran over array-border
        if(pHistory->writeIndex == SDO_HISTORY_SIZE)
        {
            pHistory->writeIndex = 0;
        }

    }
    else
    {   // no free entry
        Ret = kEplSdoSeqNoFreeHistory;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:        deleteAckedFrameFromHistory
//
// Description:     function to delete acknowledged frames from history buffer
//
//
//
// Parameters:      pAsySdoSeqCon_p = pointer to control structure of this connection
//                  bRecSeqNumber_p = receive sequence number of the received frame
//
//
// Returns:         tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel deleteAckedFrameFromHistory(tSdoSeqCon* pAsySdoSeqCon_p,
                                              UINT8 recvSeqNumber_p)
{
tEplKernel              Ret;
tSdoSeqConHistory*   pHistory;
BYTE                    bAckIndex;
BYTE                    bCurrentSeqNum;

    Ret = kEplSuccessful;

    // get pointer to history buffer
    pHistory = &pAsySdoSeqCon_p->sdoSeqConHistory;

    // release all acknowledged frames from history buffer

    // check if there are entries in history
    if (pHistory->freeEntries < SDO_HISTORY_SIZE)
    {
        bAckIndex = pHistory->ackIndex;
        do
        {
            bCurrentSeqNum = (((tEplFrame*)pHistory->aHistoryFrame[bAckIndex])->m_Data.m_Asnd.m_Payload.m_SdoSequenceFrame.m_le_bSendSeqNumCon & SEQ_NUM_MASK);
            if (((recvSeqNumber_p - bCurrentSeqNum) & SEQ_NUM_MASK)
                    < SDO_SEQ_NUM_THRESHOLD)
            {
                pHistory->aFrameSize[bAckIndex] = 0;
                bAckIndex++;
                pHistory->freeEntries++;
                if (bAckIndex == SDO_HISTORY_SIZE)
                {   // read index run over array-boarder
                    bAckIndex = 0;
                }
            }
            else
            {   // nothing to do anymore,
                // because any further frame in history has larger sequence
                // number than the acknowledge
                goto Exit;
            }
        }
        while ((((recvSeqNumber_p - 1 - bCurrentSeqNum) & SEQ_NUM_MASK)
                    < SDO_SEQ_NUM_THRESHOLD)
               && (pHistory->writeIndex != bAckIndex));

        // store local read-index to global var
        pHistory->ackIndex = bAckIndex;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        readFromHistory
//
// Description:     function to one frame from history
//
//
//
// Parameters:      pAsySdoSeqCon_p = pointer to control structure of this connection
//                  ppFrame_p       = pointer to pointer to the buffer of the stored frame
//                  puiSize_p       = OUT: size of the frame
//                  fInitRead       = bool which indicate a start of retransmission
//                                      -> return last not acknowledged message if TRUE
//
//
// Returns:         tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel readFromHistory(tSdoSeqCon* pAsySdoSeqCon_p, tEplFrame** ppFrame_p,
                                  UINT* pSize_p, BOOL fInitRead_p)
{
tEplKernel              Ret;
tSdoSeqConHistory*   pHistory;

    Ret = kEplSuccessful;

    // read one message from History

    // get pointer to history buffer
    pHistory = &pAsySdoSeqCon_p->sdoSeqConHistory;

    // check if init
    if (fInitRead_p != FALSE)
    {   // initialize read index to the index which shall be acknowledged next
        pHistory->readIndex = pHistory->ackIndex;
    }

    // check if entries are available for reading
    if ((pHistory->freeEntries < SDO_HISTORY_SIZE)
        && (pHistory->writeIndex != pHistory->readIndex))
    {
//        PRINTF("readFromHistory(): init = %d, read = %u, write = %u, ack = %u", (int) fInitRead_p, (WORD)pHistory->readIndex, (WORD)pHistory->writeIndex, (WORD)pHistory->ackIndex);
//        PRINTF(", free entries = %u, next frame size = %u\n", (WORD)pHistory->freeEntries, pHistory->aFrameSize[pHistory->readIndex]);

        // return pointer to stored frame
        *ppFrame_p = (tEplFrame*)pHistory->aHistoryFrame[pHistory->readIndex];

        // save size
        *pSize_p = pHistory->aFrameSize[pHistory->readIndex];

        pHistory->readIndex++;
        if(pHistory->readIndex == SDO_HISTORY_SIZE)
        {
            pHistory->readIndex = 0;
        }

    }
    else
    {
//        PRINTF("readFromHistory(): read = %u, ack = %u, free entries = %u, no frame\n", (WORD)pHistory->readIndex, (WORD)pHistory->ackIndex, (WORD)pHistory->freeEntries);

        // no more frames to send
        // return null pointer
        *ppFrame_p = NULL;

        *pSize_p = 0;
    }

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:        getFreeHistoryEntries
//
// Description:     function returns the number of free history entries
//
//
//
// Parameters:      pAsySdoSeqCon_p = pointer to control structure of this connection
//
//
// Returns:         unsigned int    = number of free entries
//
//
// State:
//
//---------------------------------------------------------------------------
static unsigned int getFreeHistoryEntries(tSdoSeqCon* pAsySdoSeqCon_p)
{
unsigned int uiFreeEntries;

    uiFreeEntries = (unsigned int)pAsySdoSeqCon_p->sdoSeqConHistory.freeEntries;

    return uiFreeEntries;
}

//---------------------------------------------------------------------------
//
// Function:        setTimer
//
// Description:     function sets or modify timer in timermodule
//
//
//
// Parameters:      pAsySdoSeqCon_p = pointer to control structure of this connection
//                  ulTimeout       = timeout in ms
//
//
// Returns:         unsigned int    = number of free entries
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel setTimer(tSdoSeqCon* pAsySdoSeqCon_p, ULONG timeout)
{
tEplKernel      Ret;
tEplTimerArg    TimerArg;

    TimerArg.m_EventSink = kEplEventSinkSdoAsySeq;
    TimerArg.m_Arg.m_pVal = pAsySdoSeqCon_p;

    if(pAsySdoSeqCon_p->timerHandle == 0)
    {   // create new timer
        Ret = EplTimeruSetTimerMs(&pAsySdoSeqCon_p->timerHandle,
                                    timeout,
                                    TimerArg);
    }
    else
    {   // modify existing timer
        Ret = EplTimeruModifyTimerMs(&pAsySdoSeqCon_p->timerHandle,
                                    timeout,
                                    TimerArg);

    }


    return Ret;
}

// EOF

