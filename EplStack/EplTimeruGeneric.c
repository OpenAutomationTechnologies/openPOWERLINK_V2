/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Epl Userspace-Timermodule
                Implementation for use without any operating system

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
                    Altera Nios2 GCC V3.4.6
                    Microsoft Visual C 2005/2008

  -------------------------------------------------------------------------

  Revision History:

  2009/08/31 m.u.:   start of the implementation

****************************************************************************/

#include "user/EplTimeru.h"
#include "EplTarget.h"


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

#define TIMERU_TIMER_LIST   0
#define TIMERU_FREE_LIST    1

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    #define TIMERU_EVENT_SHUTDOWN   0   // smaller index has higher priority
    #define TIMERU_EVENT_WAKEUP     1
#endif

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct _tTimerEntry
{
    struct _tTimerEntry* m_pNext;       // this must be the first element
    DWORD           m_dwTimeoutMs;      // timeout in [ms]
    tEplTimerArg    m_TimerArg;

} tTimerEntry;

typedef struct
{
    tTimerEntry*        m_pEntries;         // pointer to array of all timer entries
    tTimerEntry*        m_pTimerListFirst;
    tTimerEntry*        m_pFreeListFirst;
    DWORD               m_dwStartTimeMs;    // start time when the first timeout in list is based on

    unsigned int        m_uiFreeEntries;
    unsigned int        m_uiMinFreeEntries; // minimum number of free entries
                                            // used to check if EPL_TIMERU_MAX_ENTRIES is large enough 
#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    CRITICAL_SECTION    m_aCriticalSections[2];
    HANDLE              m_hProcessThread;
    HANDLE              m_ahEvents[2];      // WakeUp and ShutDown event handles
#endif

} tEplTimeruInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEplTimeruInstance EplTimeruInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void  EplTimeruEnterCriticalSection (int nType_p);
static void  EplTimeruLeaveCriticalSection (int nType_p);
static DWORD EplTimeruGetTickCountMs (void);

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
static DWORD PUBLIC EplTimeruProcessThread (LPVOID lpParameter);
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl Userspace-Timermodule NoOS>                    */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: Epl Userspace-Timermodule Implementation for use without
//              any operating system
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplTimeruInit
//
// Description: function init first instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruInit()
{
tEplKernel  Ret;

    Ret = EplTimeruAddInstance();

return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruAddInstance
//
// Description: function init additional instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruAddInstance(void)
{
int nIdx;
tEplKernel Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplTimeruInstance_g, 0, sizeof (EplTimeruInstance_g));

    EplTimeruInstance_g.m_pEntries = EPL_MALLOC(sizeof (tTimerEntry) * EPL_TIMERU_MAX_ENTRIES);
    if (EplTimeruInstance_g.m_pEntries == NULL)
    {   // allocating of timer entries failed
        Ret = kEplNoResource;
        goto Exit;
    }

    EplTimeruInstance_g.m_pTimerListFirst = NULL;

    // fill free timer list
    for (nIdx = 0; nIdx < EPL_TIMERU_MAX_ENTRIES-1; nIdx++)
    {
        EplTimeruInstance_g.m_pEntries[nIdx].m_pNext = &EplTimeruInstance_g.m_pEntries[nIdx+1];
    }
    EplTimeruInstance_g.m_pEntries[EPL_TIMERU_MAX_ENTRIES-1].m_pNext = NULL;

    EplTimeruInstance_g.m_pFreeListFirst = EplTimeruInstance_g.m_pEntries;
    EplTimeruInstance_g.m_uiFreeEntries = EPL_TIMERU_MAX_ENTRIES;
    EplTimeruInstance_g.m_uiMinFreeEntries = EPL_TIMERU_MAX_ENTRIES;

    // set start time to a value which is in any case less or equal than EplTimeruGetTickCountMs()
    // -> the only solution = 0
    EplTimeruInstance_g.m_dwStartTimeMs = 0;

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    InitializeCriticalSection(&EplTimeruInstance_g.m_aCriticalSections[TIMERU_TIMER_LIST]);
    InitializeCriticalSection(&EplTimeruInstance_g.m_aCriticalSections[TIMERU_FREE_LIST]);

    EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_WAKEUP]   = CreateEvent(NULL, FALSE, FALSE, NULL);
    EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_SHUTDOWN] = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_WAKEUP] == NULL
        || EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_SHUTDOWN] == NULL)
    {
        Ret = kEplTimerThreadError;
        goto Exit;
    }

    EplTimeruInstance_g.m_hProcessThread = CreateThread(NULL, 0,
                                                        EplTimeruProcessThread,
                                                        NULL, 0, NULL);
    if (EplTimeruInstance_g.m_hProcessThread == NULL)
    {
        Ret = kEplTimerThreadError;
        goto Exit;
    }
#endif

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruDelInstance
//
// Description: function deletes instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruDelInstance(void)
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    SetEvent( EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_SHUTDOWN] );

    WaitForSingleObject( EplTimeruInstance_g.m_hProcessThread, INFINITE );

    CloseHandle( EplTimeruInstance_g.m_hProcessThread );
    CloseHandle( EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_SHUTDOWN] );
    CloseHandle( EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_WAKEUP] );

    DeleteCriticalSection( &EplTimeruInstance_g.m_aCriticalSections[TIMERU_TIMER_LIST] );
    DeleteCriticalSection( &EplTimeruInstance_g.m_aCriticalSections[TIMERU_FREE_LIST] );
#endif

    EPL_FREE(EplTimeruInstance_g.m_pEntries);

    EplTimeruInstance_g.m_pEntries = NULL;
    EplTimeruInstance_g.m_pFreeListFirst = NULL;
    EplTimeruInstance_g.m_pTimerListFirst = NULL;
    EplTimeruInstance_g.m_uiFreeEntries = 0;

    return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruProcess
//
// Description: This function is called repeatedly from within the main
//              loop of the application. It checks whether the first timer
//              entry has been elapsed.
//
// Parameters:  none
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruProcess(void)
{
tTimerEntry*        pTimerEntry;
DWORD               dwTimeoutMs;
tEplEvent           EplEvent;
tEplTimerEventArg   TimerEventArg;
tEplKernel          Ret;


    Ret      = kEplSuccessful;

    EplTimeruEnterCriticalSection(TIMERU_TIMER_LIST);
    // calculate elapsed time since start time
    dwTimeoutMs = EplTimeruGetTickCountMs() - EplTimeruInstance_g.m_dwStartTimeMs;

    // observe first timer entry in timer list
    pTimerEntry = EplTimeruInstance_g.m_pTimerListFirst;
    if (pTimerEntry != NULL)
    {
        if (dwTimeoutMs >= pTimerEntry->m_dwTimeoutMs)
        {   // timeout elapsed
            // remove entry from timer list
            EplTimeruInstance_g.m_pTimerListFirst = pTimerEntry->m_pNext;

            // adjust start time
            EplTimeruInstance_g.m_dwStartTimeMs += pTimerEntry->m_dwTimeoutMs;
        }
        else
        {
            pTimerEntry = NULL;
        }
    }
    EplTimeruLeaveCriticalSection(TIMERU_TIMER_LIST);

    if (pTimerEntry != NULL)
    {
        // call event function
        TimerEventArg.m_TimerHdl = (tEplTimerHdl) pTimerEntry;
        EPL_MEMCPY(&TimerEventArg.m_Arg, &pTimerEntry->m_TimerArg.m_Arg, sizeof (TimerEventArg.m_Arg));
    
        EplEvent.m_EventSink = pTimerEntry->m_TimerArg.m_EventSink;
        EplEvent.m_EventType = kEplEventTypeTimer;
        EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(tEplNetTime));
        EplEvent.m_pArg = &TimerEventArg;
        EplEvent.m_uiSize = sizeof(TimerEventArg);

        Ret = EplEventuPost(&EplEvent);
    }

    return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruSetTimerMs
//
// Description: function creates a timer and returns a handle to the pointer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTimeMs_p  = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruSetTimerMs(  tEplTimerHdl*   pTimerHdl_p,
                                        unsigned long   ulTimeMs_p,
                                        tEplTimerArg    Argument_p)
{
tTimerEntry*    pNewEntry;
tTimerEntry**   ppEntry;
tEplKernel      Ret;


    Ret = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // fetch entry from free timer list
    EplTimeruEnterCriticalSection(TIMERU_FREE_LIST);
    pNewEntry = EplTimeruInstance_g.m_pFreeListFirst;
    if (pNewEntry != NULL)
    {
        EplTimeruInstance_g.m_pFreeListFirst = pNewEntry->m_pNext;
        EplTimeruInstance_g.m_uiFreeEntries--;
        if (EplTimeruInstance_g.m_uiMinFreeEntries > EplTimeruInstance_g.m_uiFreeEntries)
        {
            EplTimeruInstance_g.m_uiMinFreeEntries = EplTimeruInstance_g.m_uiFreeEntries;
        }
    }
    EplTimeruLeaveCriticalSection(TIMERU_FREE_LIST);

    if (pNewEntry == NULL)
    {   // sorry, no free entry
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    *pTimerHdl_p = (tEplTimerHdl) pNewEntry;
    EPL_MEMCPY(&pNewEntry->m_TimerArg, &Argument_p, sizeof(tEplTimerArg));

    // insert timer entry in timer list
    EplTimeruEnterCriticalSection(TIMERU_TIMER_LIST);

    // calculate timeout based on start time
    pNewEntry->m_dwTimeoutMs = (EplTimeruGetTickCountMs() - EplTimeruInstance_g.m_dwStartTimeMs) + ulTimeMs_p;

    ppEntry = &EplTimeruInstance_g.m_pTimerListFirst;
    while (*ppEntry != NULL)
    {
        if ((*ppEntry)->m_dwTimeoutMs > pNewEntry->m_dwTimeoutMs)
        {
            (*ppEntry)->m_dwTimeoutMs -= pNewEntry->m_dwTimeoutMs;
            break;
        }
        pNewEntry->m_dwTimeoutMs -= (*ppEntry)->m_dwTimeoutMs;
        ppEntry = &(*ppEntry)->m_pNext;
    }
    // insert before **ppEntry
    pNewEntry->m_pNext = *ppEntry;
    *ppEntry = pNewEntry;
    EplTimeruLeaveCriticalSection(TIMERU_TIMER_LIST);

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
	if (ppEntry == &EplTimeruInstance_g.m_pTimerListFirst)
    {
        SetEvent( EplTimeruInstance_g.m_ahEvents[TIMERU_EVENT_WAKEUP] );
    }
#endif

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruModifyTimerMs
//
// Description: function changes a timer and returns the corresponding handle
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTime_p    = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruModifyTimerMs(tEplTimerHdl*    pTimerHdl_p,
                                        unsigned long     ulTimeMs_p,
                                        tEplTimerArg      Argument_p)
{
tEplKernel      Ret;

    Ret = EplTimeruDeleteTimer(pTimerHdl_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplTimeruSetTimerMs(pTimerHdl_p, ulTimeMs_p, Argument_p);

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruDeleteTimer
//
// Description: function deletes a timer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruDeleteTimer(tEplTimerHdl*     pTimerHdl_p)
{
tTimerEntry*    pTimerEntry;
tTimerEntry**   ppEntry;
tEplKernel      Ret;


    Ret         = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        Ret = kEplSuccessful;
        goto Exit;
    }

    pTimerEntry = (tTimerEntry*) *pTimerHdl_p;

    // remove timer entry from timer list
    EplTimeruEnterCriticalSection(TIMERU_TIMER_LIST);
    ppEntry = &EplTimeruInstance_g.m_pTimerListFirst;
    while (*ppEntry != NULL)
    {
        if (*ppEntry == pTimerEntry)
        {
            *ppEntry = pTimerEntry->m_pNext;
            if (*ppEntry != NULL)
            {
                (*ppEntry)->m_dwTimeoutMs += pTimerEntry->m_dwTimeoutMs;
            }
            break;
        }
            
        ppEntry = &(*ppEntry)->m_pNext;
    }
    EplTimeruLeaveCriticalSection(TIMERU_TIMER_LIST);

    // insert in free list
    EplTimeruEnterCriticalSection(TIMERU_FREE_LIST);
    pTimerEntry->m_pNext = EplTimeruInstance_g.m_pFreeListFirst;
    EplTimeruInstance_g.m_pFreeListFirst = pTimerEntry;
    EplTimeruInstance_g.m_uiFreeEntries++;
    EplTimeruLeaveCriticalSection(TIMERU_FREE_LIST);

    // set handle invalid
    *pTimerHdl_p = 0;
    
Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimeruGetMinFreeEntries
//
// Description: returns the minimum number of free entries
//              since this instance has been added
//
// Parameters:  none
//
// Returns:     unsigned int            = minimum number of free timer entries
//
// State:
//
//---------------------------------------------------------------------------

unsigned int PUBLIC EplTimeruGetMinFreeEntries(void)
{
    return (EplTimeruInstance_g.m_uiMinFreeEntries);
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  EplTimeruGetTickCountMs
//---------------------------------------------------------------------------

static DWORD EplTimeruGetTickCountMs (void)
{
DWORD	TickCountMs;

#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
	TickCountMs = GetTickCount();
#else
    TickCountMs = EplTgtGetTickCountMs();
#endif

	return TickCountMs;
}


//---------------------------------------------------------------------------
//  EplTimeruEnterCriticalSection
//---------------------------------------------------------------------------

static void EplTimeruEnterCriticalSection (int nType_p)
{
#if (TARGET_SYSTEM == _NO_OS_)
    #if EPL_USE_SHAREDBUFF == FALSE
        EplTgtEnableGlobalInterrupt(FALSE);
    #endif
#elif (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    EnterCriticalSection(&EplTimeruInstance_g.m_aCriticalSections[nType_p]);
#endif
}


//---------------------------------------------------------------------------
//  EplTimeruLeaveCriticalSection
//---------------------------------------------------------------------------

static void EplTimeruLeaveCriticalSection (int nType_p)
{
#if (TARGET_SYSTEM == _NO_OS_)
    #if EPL_USE_SHAREDBUFF == FALSE
        EplTgtEnableGlobalInterrupt(TRUE);
    #endif
#elif (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )
    LeaveCriticalSection(&EplTimeruInstance_g.m_aCriticalSections[nType_p]);
#endif
}


#if (TARGET_SYSTEM == _WIN32_ || TARGET_SYSTEM == _WINCE_ )

//---------------------------------------------------------------------------
//  Thread for processing the timer list when using an operating system
//---------------------------------------------------------------------------

static DWORD PUBLIC EplTimeruProcessThread (LPVOID lpParameter)
{
tTimerEntry*	pTimerEntry;
DWORD			dwTimeoutMs;
DWORD           dwWaitResult;
tEplKernel		Ret;

    UNUSED_PARAMETER(lpParameter);
    for (;;)
    {
        Ret = EplTimeruProcess();
        if (Ret != kEplSuccessful)
        {
            // Error
        }
        
        // calculate time until the next timer event
        EplTimeruEnterCriticalSection(TIMERU_TIMER_LIST);
        pTimerEntry = EplTimeruInstance_g.m_pTimerListFirst;
        if (pTimerEntry == NULL)
        {   // timer list is empty
            dwTimeoutMs = INFINITE;
        }
        else
        {
            dwTimeoutMs = EplTimeruGetTickCountMs() - EplTimeruInstance_g.m_dwStartTimeMs;
            if (dwTimeoutMs > pTimerEntry->m_dwTimeoutMs)
            {   // timeout elapsed
                dwTimeoutMs = 0;
            }
            else
            {   // adjust timeout with elapsed time since start time
                dwTimeoutMs = pTimerEntry->m_dwTimeoutMs - dwTimeoutMs;
            }
        }
        EplTimeruLeaveCriticalSection(TIMERU_TIMER_LIST);
        
        dwWaitResult = WaitForMultipleObjects(2, EplTimeruInstance_g.m_ahEvents, FALSE, dwTimeoutMs);
        switch (dwWaitResult)
        {
            case (WAIT_OBJECT_0 + TIMERU_EVENT_SHUTDOWN):
            {
                goto Exit;
            }
            case (WAIT_OBJECT_0 + TIMERU_EVENT_WAKEUP):
            case WAIT_TIMEOUT:
            {
                break;
            }
            default:
            {
                // Error
            }
        }
    }

Exit:
    return 0;
}

#endif


// EOF

