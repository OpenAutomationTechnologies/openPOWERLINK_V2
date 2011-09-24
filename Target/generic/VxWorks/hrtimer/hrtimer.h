/*******************************************************************************
  File:         hrtimer.h

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    Header file with definitions for high-resolution timer library

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
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

*******************************************************************************/

#ifndef __INC_HRTIMER_H
#define __INC_HRTIMER_H

//============================================================================//
// Includes                                                                   //
//============================================================================//
#include "hrtimerLib.h"

#include <stdio.h>

//============================================================================//
// Definitions                                                                //
//============================================================================//

/* some definitions for time conversion */
#define FSEC_PER_NSEC                   1000000LL
#define FSEC_PER_10PSEC                 10000LL
#define FSEC_PER_SEC                    1000000000000000LL
#define NSEC_PER_SEC                    1000000000LL
#define NSEC_PER_10PSEC                 100LL

#define TIMESPEC_TO_NSEC(ts) ((ts.tv_sec * NSEC_PER_SEC) + ts.tv_nsec)
#define NSEC_TO_TIMESPEC(nsec,tp) \
                (tp)->tv_sec = (nsec) / NSEC_PER_SEC; \
                (tp)->tv_nsec = (nsec) % NSEC_PER_SEC;

//------------------------------------------------------------------------------
// tHrtimer specifies the data structure of a timer used by the high-
// resolution timer library.
//------------------------------------------------------------------------------
typedef struct tHrtimer
{
    // Contains the ID of the timer
    timer_t m_iTimerId;
    // Pointer to previous timer in timer list
    struct tHrtimer* m_pPrev;
    // Pointer to the next timer in the timer list
    struct tHrtimer* m_pNext;
    // Pointer to the previous timer in the armed timer list
    struct tHrtimer* m_pPrevArmed;
    // Pointer to the next timer in the armed timer list
    struct tHrtimer* m_pNextArmed;
    // Pointer to next timer in callback list
    struct tHrtimer* m_pNextCallback;
    // Specifies how the timer will be signalled
    tHrtimerSig m_signalling;
    // Contains the timeout information of the timer
    struct itimerspec m_timeout;
    // The next timeout when the timer expires
    struct timespec m_nextTimeout;
    // m_state specifies the state of the timer
    eTimerState m_state;
}  tHrtimer;

//------------------------------------------------------------------------------
// The tHrtimerData structure implements the main data structure used for
// the high-resolution timer library hrtimer.
//------------------------------------------------------------------------------
typedef struct
{
    // Flag determines if timer library is initialized
    BOOL m_fInitialized;
    // Flag is used to determine if timer task should be stopped.
    BOOL m_fStopTimerTask;
    // Task ID of the timer task
    int m_timerTask;
    // Pointer to the double linked list of armed timers
    tHrtimer* m_pArmedTimers;
    // Pointer to the first timer in the callback list
    tHrtimer* m_pFirstCallbackTimer;
    // Pointer to the last timer in the callback list
    tHrtimer* m_pLastCallbackTimer;
    // Pointer to the first timer in the list of created timers
    tHrtimer* m_pFirstTimer;
    // Pointer to the last timer in the list of created timers
    tHrtimer* m_pLastTimer;
    // Semaphore for callback signaling
    SEM_ID m_signalSem;
}  tHrtimerData;


// Main data structure of hrtimer library
extern tHrtimerData timerData_g;

//============================================================================//
// Function declarations                                                      //
//============================================================================//
void addToList(tHrtimer* pTimer_p, timer_t* pTimerId_p);
void removeFromList(tHrtimer* pTimer_p);
void insertInArmedList(tHrtimer* pInsertionPoint_p, tHrtimer* pTimer_p);
void removeFromArmedList(tHrtimer* pTimer_p);
void searchArmedPosition(struct timespec * pTimeout, tHrtimer** pInsertionPoint_p);
int handleCallback(int iArg_p);
void handleTimerEvent(void);
BOOL disarmTimer(tHrtimer * pTimer_p);
int armTimerList(void);

#endif /* ifndef __INC_HRTIMER_H */
