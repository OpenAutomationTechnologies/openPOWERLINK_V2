/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Epl-Kernelspace-Event-Modul

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

  2006/06/20 k.t.:   start of the implementation

****************************************************************************/

#include "kernel/EplEventk.h"
#include "kernel/EplNmtk.h"
#include "kernel/EplDllk.h"
#include "kernel/EplDllkCal.h"
#include "kernel/EplErrorHandlerk.h"
#include "Benchmark.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
#include "kernel/EplPdok.h"
#include "kernel/EplPdokCal.h"
#endif

#if EPL_USE_SHAREDBUFF == FALSE
    #include "user/EplEventu.h"
#else
    #include "SharedBuff.h"
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

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
#if EPL_USE_SHAREDBUFF != FALSE
    tShbInstance    m_pShbKernelToUserInstance;
#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    tShbInstance    m_pShbUserToKernelInstance;
    tShbInstance    m_pShbKernelInternalInstance;
    BYTE            m_abRxBuffer[sizeof(tEplEvent) + EPL_MAX_EVENT_ARG_SIZE];
#endif
#endif

#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)
    unsigned int    m_uiUserToKernelFullCount;
#endif

} tEplEventkInstance;


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEplEventkInstance EplEventkInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// callback function for incoming events
#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)

static void  EplEventkRxSignalHandlerCb (
    tShbInstance pShbRxInstance_p,
    unsigned long ulDataSize_p);

#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl-Kernelspace-Event>                             */
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
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplEventkInit
//
// Description: function initializes the first instance
//
// Parameters:  void
//
// Returns:     tEpKernel           = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkInit(void)
{
tEplKernel Ret;

    Ret = EplEventkAddInstance();

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplEventkAddInstance
//
// Description: function adds one more instance
//
// Parameters:  void
//
// Returns:     tEpKernel           = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkAddInstance(void)
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
unsigned int    fShbNewCreated;
#endif

    Ret = kEplSuccessful;

    // init instance structure
#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    EplEventkInstance_g.m_uiUserToKernelFullCount = 0;
#endif

#if EPL_USE_SHAREDBUFF != FALSE
    // init shared loop buffer
    // kernel -> user
    ShbError = ShbCirAllocBuffer (EPL_EVENT_SIZE_SHB_KERNEL_TO_USER,
                                  EPL_EVENT_NAME_SHB_KERNEL_TO_USER,
                                  &EplEventkInstance_g.m_pShbKernelToUserInstance,
                                  &fShbNewCreated);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkAddInstance(): ShbCirAllocBuffer(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }

#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    // user -> kernel
    ShbError = ShbCirAllocBuffer (EPL_EVENT_SIZE_SHB_USER_TO_KERNEL,
                                  EPL_EVENT_NAME_SHB_USER_TO_KERNEL,
                                  &EplEventkInstance_g.m_pShbUserToKernelInstance,
                                  &fShbNewCreated);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkAddInstance(): ShbCirAllocBuffer(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }

    ShbError = ShbCirAllocBuffer (EPL_EVENT_SIZE_SHB_KERNEL_INTERNAL,
                                  EPL_EVENT_NAME_SHB_KERNEL_INTERNAL,
                                  &EplEventkInstance_g.m_pShbKernelInternalInstance,
                                  &fShbNewCreated);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkAddInstance(): ShbCirAllocBuffer(Kint) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }

    // register eventhandler
    ShbError = ShbCirSetSignalHandlerNewData (EplEventkInstance_g.m_pShbKernelInternalInstance,
                                    EplEventkRxSignalHandlerCb,
                                    kShbPriorityHigh);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkAddInstance(): ShbCirSetSignalHandlerNewData(Kint) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }

    ShbError = ShbCirConnectMaster (EplEventkInstance_g.m_pShbUserToKernelInstance,
                                    EplEventkRxSignalHandlerCb,
                                    EplEventkInstance_g.m_pShbKernelInternalInstance,
                                    kShbPriorityNormal);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkAddInstance(): ShbCirConnectMaster(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
        goto Exit;
    }
#endif

Exit:
#endif

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplEventkDelInstance
//
// Description: function deletes instance and frees the buffers
//
// Parameters:  void
//
// Returns:     tEpKernel   = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkDelInstance()
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
#endif

    Ret = kEplSuccessful;

#if EPL_USE_SHAREDBUFF != FALSE
#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
    // set eventhandler to NULL
    ShbError = ShbCirConnectMaster (EplEventkInstance_g.m_pShbUserToKernelInstance,
                                    NULL,
                                    EplEventkInstance_g.m_pShbKernelInternalInstance,
                                    kShbPriorityNormal);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkDelInstance(): ShbCirConnectMaster(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }

    ShbError = ShbCirSetSignalHandlerNewData (EplEventkInstance_g.m_pShbKernelInternalInstance,
                                    NULL,
                                    kShbPriorityNormal);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkDelInstance(): ShbCirSetSignalHandlerNewData(Kint) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }

    // free buffer User -> Kernel
    ShbError = ShbCirReleaseBuffer (EplEventkInstance_g.m_pShbUserToKernelInstance);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkDelInstance(): ShbCirReleaseBuffer(U2K) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }
    else
    {
        EplEventkInstance_g.m_pShbUserToKernelInstance = NULL;
    }

    // free buffer Kernel internal
    ShbError = ShbCirReleaseBuffer (EplEventkInstance_g.m_pShbKernelInternalInstance);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkDelInstance(): ShbCirReleaseBuffer(Kint) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }
    else
    {
        EplEventkInstance_g.m_pShbKernelInternalInstance = NULL;
    }
#endif

    // free buffer  Kernel -> User
    ShbError = ShbCirReleaseBuffer (EplEventkInstance_g.m_pShbKernelToUserInstance);
    if(ShbError != kShbOk)
    {
        EPL_DBGLVL_EVENTK_TRACE("EplEventkDelInstance(): ShbCirReleaseBuffer(K2U) -> 0x%X\n", ShbError);
        Ret = kEplNoResource;
    }
    else
    {
        EplEventkInstance_g.m_pShbKernelToUserInstance = NULL;
    }
#endif

return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplEventkProcess
//
// Description: Kernelthread that dispatches events in kernel part
//
// Parameters:  pEvent_p    = pointer to event-structure from buffer
//
// Returns:     tEpKernel   = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkProcess(tEplEvent* pEvent_p)
{
tEplKernel              Ret;
tEplEventSource         EventSource;

    Ret = kEplSuccessful;

#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)
    // error handling if event queue is full
    if (EplEventkInstance_g.m_uiUserToKernelFullCount > 0)
    {   // UserToKernel event queue has run out of space -> kEplNmtEventInternComError
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)
        tEplEvent   Event;
        tEplNmtEvent NmtEvent;
#endif
        tShbError   ShbError;

        // directly call NMTk process function, because event queue is full
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)
        NmtEvent = kEplNmtEventInternComError;
        Event.m_EventSink = kEplEventSinkNmtk;
        Event.m_NetTime.m_dwNanoSec = 0;
        Event.m_NetTime.m_dwSec = 0;
        Event.m_EventType = kEplEventTypeNmtEvent;
        Event.m_pArg = &NmtEvent;
        Event.m_uiSize = sizeof(NmtEvent);
        Ret = EplNmtkProcess(&Event);
#endif

        // NMT state machine changed to reset (i.e. NMT_GS_RESET_COMMUNICATION)
        // now, it is safe to reset the counter and empty the event queue
        ShbError = ShbCirResetBuffer (EplEventkInstance_g.m_pShbUserToKernelInstance, 1000, NULL);

        EplEventkInstance_g.m_uiUserToKernelFullCount = 0;
        TGT_DBG_SIGNAL_TRACE_POINT(22);

        // also discard the current event (it doesn't matter if we lose another event)
        goto Exit;
    }
#endif

    // check m_EventSink
    switch(pEvent_p->m_EventSink)
    {
        // NMT-Kernel-Modul
        case kEplEventSinkNmtk:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTK)) != 0)
            Ret = EplNmtkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceNmtk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            BENCHMARK_MOD_27_RESET(0);
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            if ((pEvent_p->m_EventType == kEplEventTypeNmtEvent)
                && (*((tEplNmtEvent*)pEvent_p->m_pArg) == kEplNmtEventDllCeSoa))
            {

                BENCHMARK_MOD_27_SET(0);

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
                // forward SoA event to DLLk module for cycle preprocessing
                Ret = EplDllkProcess(pEvent_p);
                if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
                {
                    EventSource = kEplEventSourceDllk;

                    // Error event for API layer
                    EplEventkPostError(kEplEventSourceEventk,
                                    Ret,
                                    sizeof(EventSource),
                                    &EventSource);
                }
#endif
                BENCHMARK_MOD_27_RESET(0);

            }
#endif
            break;
        }

        // events for Dllk module
        case kEplEventSinkDllk:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            Ret = EplDllkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

        // events for DllkCal module
        case kEplEventSinkDllkCal:
        {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
            Ret = EplDllkCalProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceDllk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
#endif
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
        // events for PDO CAL module
        case kEplEventSinkPdokCal:
        {
            Ret = EplPdokCalProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourcePdok;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
        // events for Error handler module
        case kEplEventSinkErrk:
        {
            // only call error handler if DLL is present
            Ret = EplErrorHandlerkProcess(pEvent_p);
            if ((Ret != kEplSuccessful) && (Ret != kEplShutdown))
            {
                EventSource = kEplEventSourceErrk;

                // Error event for API layer
                EplEventkPostError(kEplEventSourceEventk,
                                Ret,
                                sizeof(EventSource),
                                &EventSource);
            }
            break;
        }
#endif

        // unknown sink
        default:
        {
            Ret = kEplEventUnknownSink;

            // Error event for API layer
            EplEventkPostError(kEplEventSourceEventk,
                            Ret,
                            sizeof(pEvent_p->m_EventSink),
                            &pEvent_p->m_EventSink);
        }

    } // end of switch(pEvent_p->m_EventSink)

#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)
Exit:
#endif
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplEventkPost
//
// Description: post events from kernel part
//
// Parameters:  pEvent_p    = pointer to event-structure from buffer
//
// Returns:     tEpKernel   = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkPost(tEplEvent * pEvent_p)
{
tEplKernel      Ret;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError       ShbError;
tShbCirChunk    ShbCirChunk;
unsigned long   ulDataSize;
unsigned int    fBufferCompleted;
#endif

    Ret = kEplSuccessful;


    // the event must be posted by using the abBuffer
    // it is neede because the Argument must by copied
    // to the buffer too and not only the pointer

#if EPL_USE_SHAREDBUFF != FALSE
    // 2006/08/03 d.k.: Event and argument are posted as separate chunks to the event queue.
    ulDataSize = sizeof(tEplEvent) + ((pEvent_p->m_pArg != NULL) ? pEvent_p->m_uiSize : 0);
#endif

    // decide in which buffer the event have to write
    switch(pEvent_p->m_EventSink)
    {
        // kernelspace modules
        case kEplEventSinkSync:
        case kEplEventSinkNmtk:
        case kEplEventSinkDllk:
        case kEplEventSinkDllkCal:
        case kEplEventSinkPdok:
        case kEplEventSinkPdokCal:
        case kEplEventSinkErrk:
        {
            BENCHMARK_MOD_27_SET(2);
#if (EPL_USE_SHAREDBUFF != FALSE) \
    && (EPL_EVENT_USE_KERNEL_QUEUE != FALSE)
            // post message
            ShbError = ShbCirAllocDataBlock (EplEventkInstance_g.m_pShbKernelInternalInstance,
                                   &ShbCirChunk,
                                   ulDataSize);
            switch (ShbError)
            {
                case kShbOk:
                    break;

                case kShbBufferFull:
                {
                    EplEventkInstance_g.m_uiUserToKernelFullCount++;
                    Ret = kEplEventPostError;
                    goto Exit;
                }

                default:
                {
                    EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirAllocDataBlock(U2K) -> 0x%X\n", ShbError);
                    Ret = kEplEventPostError;
                    goto Exit;
                }
            }
            ShbError = ShbCirWriteDataChunk (EplEventkInstance_g.m_pShbKernelInternalInstance,
                                   &ShbCirChunk,
                                   pEvent_p,
                                   sizeof (tEplEvent),
                                   &fBufferCompleted);
            if (ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirWriteDataChunk(U2K) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            if (fBufferCompleted == FALSE)
            {
                ShbError = ShbCirWriteDataChunk (EplEventkInstance_g.m_pShbKernelInternalInstance,
                                       &ShbCirChunk,
                                       pEvent_p->m_pArg,
                                       (unsigned long) pEvent_p->m_uiSize,
                                       &fBufferCompleted);
                if ((ShbError != kShbOk) || (fBufferCompleted == FALSE))
                {
                    EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirWriteDataChunk2(U2K) -> 0x%X\n", ShbError);
                    Ret = kEplEventPostError;
                    goto Exit;
                }
            }

#else
            #if EPL_EVENT_USE_KERNEL_QUEUE == FALSE
                EplTgtEnableGlobalInterrupt(FALSE);
            #endif
            Ret = EplEventkProcess(pEvent_p);
            #if EPL_EVENT_USE_KERNEL_QUEUE == FALSE
                EplTgtEnableGlobalInterrupt(TRUE);
            #endif
#endif
            BENCHMARK_MOD_27_RESET(2);

            break;
        }

        // userspace modules
        case kEplEventSinkNmtu:
        case kEplEventSinkNmtMnu:
        case kEplEventSinkSdoAsySeq:
        case kEplEventSinkApi:
        case kEplEventSinkDlluCal:
        case kEplEventSinkErru:
        {
#if EPL_USE_SHAREDBUFF != FALSE
            // post message
//            BENCHMARK_MOD_27_SET(3);    // 74 µs until reset
            ShbError = ShbCirAllocDataBlock (EplEventkInstance_g.m_pShbKernelToUserInstance,
                                   &ShbCirChunk,
                                   ulDataSize);
            if(ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirAllocDataBlock(K2U) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            ShbError = ShbCirWriteDataChunk (EplEventkInstance_g.m_pShbKernelToUserInstance,
                                   &ShbCirChunk,
                                   pEvent_p,
                                   sizeof (tEplEvent),
                                   &fBufferCompleted);
            if(ShbError != kShbOk)
            {
                EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirWriteDataChunk(K2U) -> 0x%X\n", ShbError);
                Ret = kEplEventPostError;
                goto Exit;
            }
            if (fBufferCompleted == FALSE)
            {
                ShbError = ShbCirWriteDataChunk (EplEventkInstance_g.m_pShbKernelToUserInstance,
                                       &ShbCirChunk,
                                       pEvent_p->m_pArg,
                                       (unsigned long) pEvent_p->m_uiSize,
                                       &fBufferCompleted);
                if ((ShbError != kShbOk) || (fBufferCompleted == FALSE))
                {
                    EPL_DBGLVL_EVENTK_TRACE("EplEventkPost(): ShbCirWriteDataChunk2(K2U) -> 0x%X\n", ShbError);
                    Ret = kEplEventPostError;
                    goto Exit;
                }
            }
//            BENCHMARK_MOD_27_RESET(3);  // 82 µs until ShbCirGetReadDataSize() in EplEventu

#else
            Ret = EplEventuProcess(pEvent_p);
#endif

            break;
        }

        default:
        {
            Ret = kEplEventUnknownSink;
        }


    }// end of switch(pEvent_p->m_EventSink)

#if EPL_USE_SHAREDBUFF != FALSE
Exit:
#endif
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplEventkPostError
//
// Description: post error event from kernel part to API layer
//
// Parameters:  EventSource_p   = source-module of the error event
//              EplError_p      = code of occurred error
//              ArgSize_p       = size of the argument
//              pArg_p          = pointer to the argument
//
// Returns:     tEpKernel       = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplEventkPostError(tEplEventSource EventSource_p,
                                     tEplKernel      EplError_p,
                                     unsigned int    uiArgSize_p,
                                     void*           pArg_p)
{
tEplKernel      Ret;
tEplEventError  EventError;
tEplEvent       EplEvent;

    Ret = kEplSuccessful;

    // create argument
    EventError.m_EventSource = EventSource_p;
    EventError.m_EplError = EplError_p;
    uiArgSize_p = (unsigned int) min ((size_t) uiArgSize_p, sizeof (EventError.m_Arg));
    EPL_MEMCPY(&EventError.m_Arg, pArg_p, uiArgSize_p);

    // create event
    EplEvent.m_EventType = kEplEventTypeError;
    EplEvent.m_EventSink = kEplEventSinkApi;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(EplEvent.m_NetTime));
    EplEvent.m_uiSize = (memberoffs (tEplEventError, m_Arg) + uiArgSize_p);
    EplEvent.m_pArg = &EventError;

    // post errorevent
    Ret = EplEventkPost(&EplEvent);

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplEventkRxSignalHandlerCb()
//
// Description: Callback-function for events from user and kernel part
//
// Parameters:  pShbRxInstance_p    = Instance-pointer of buffer
//              ulDataSize_p        = size of data
//
// Returns: void
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_USE_SHAREDBUFF != FALSE
#if EPL_EVENT_USE_KERNEL_QUEUE != FALSE
static void  EplEventkRxSignalHandlerCb (
                tShbInstance pShbRxInstance_p,
                unsigned long ulDataSize_p)
{
tEplEvent      *pEplEvent;
tShbError       ShbError;
BYTE*           pabDataBuffer;

    TGT_DBG_SIGNAL_TRACE_POINT(20);

    pabDataBuffer = &EplEventkInstance_g.m_abRxBuffer[0];
    BENCHMARK_MOD_27_RESET(0);
    // copy data from event queue
    ShbError = ShbCirReadDataBlock (pShbRxInstance_p,
                            pabDataBuffer,
                            sizeof(EplEventkInstance_g.m_abRxBuffer),
                            &ulDataSize_p);
    if(ShbError != kShbOk)
    {
        EplEventkPostError(kEplEventSourceEventk, kEplEventReadError, sizeof (ShbError), &ShbError);
        // error goto exit
        goto Exit;
    }

    // resolve the pointer to the event structure
    pEplEvent = (tEplEvent *) pabDataBuffer;
    // set Datasize
    pEplEvent->m_uiSize = (ulDataSize_p - sizeof(tEplEvent));
    if(pEplEvent->m_uiSize > 0)
    {
        // set pointer to argument
        pEplEvent->m_pArg = &pabDataBuffer[sizeof(tEplEvent)];
    }
    else
    {
        //set pointer to NULL
        pEplEvent->m_pArg = NULL;
    }

    BENCHMARK_MOD_27_SET(0);
    // call processfunction
    EplEventkProcess(pEplEvent);

Exit:
    return;
}
#endif
#endif

// EOF

