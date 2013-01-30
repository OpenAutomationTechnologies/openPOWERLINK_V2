/*******************************************************************************
  File:         hrtimerLib.h

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    Header file defines the external interface to the high-resolution timer
    library.

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

#ifndef __INC_HRTIMERLIB_H
#define __INC_HRTIMERLIB_H

//============================================================================//
// Includes                                                                   //
//============================================================================//
#include "EplInc.h"

#include <semLib.h>
#include <timers.h>
#include <logLib.h>
#include <intLib.h>
#include <msgQLib.h>

//============================================================================//
// Defines                                                                    //
//============================================================================//
/* definitions needed for correct timer device implementation */
#define    TIMER_DEV_HPET

/* determines if show routines are included */
#define    INCLUDE_HRTIMER_SHOW

//============================================================================//
// Type definitions                                                           //
//============================================================================//

// type definition for signaling callback function
typedef  int (*tHrTimerCbFuncPtr)(void *);

//------------------------------------------------------------------------------
// eHrTimerSigType specifies the valid signaling types for the timers.
//------------------------------------------------------------------------------
typedef enum
{
    // Timer signalling using a semaphore. The timer event can be handled by a
    // custom user task which waits on the semaphore.
    kHrtimerSigSem = 0,
    // Timer signalling using callback function. The callback function will be
    // executed from the timer task.
    kHrtimerSigCallback = 1,
    // Signalling timer events using callbacks within the interrupt context
    kHrtimerSigIntCb = 2,
    // Signalling timer events using message queues
    kHrtimerSigMsgQueue = 3
} eHrtimerSigType;

//------------------------------------------------------------------------------
// eTimerState defines an enumeration with valid timer states.
//------------------------------------------------------------------------------
typedef enum
{
    // The timer is inactive (not armed or scheduled)
    kHrtimerStateInactive = 0,
    // The timer is armed
    kHrtimerStateActive = 1,
    // The timer is armed and scheduled
    kHrtimerStateScheduled = 2
} eTimerState;

//------------------------------------------------------------------------------
// The following struct contains signaling parameters used for signaling
// timer events through a callback function.
//------------------------------------------------------------------------------
typedef struct
{
    // Pointer to callback function which will be called when timer expires
    tHrTimerCbFuncPtr m_pfnCallback;
    // Argument for callback function
    void * m_arg;
}  tHrtimerSigCb;

//------------------------------------------------------------------------------
// The following struct contains the signaling parameters used for signaling
// hrtimer events through a semaphore.
//------------------------------------------------------------------------------
typedef struct
{
    // Semaphore used for timer signaling
    SEM_ID m_semaphore;
}  tHrtimerSigSem;

//------------------------------------------------------------------------------
// The following struct contains the signaling parameters used for signaling
// hrtimer events through a message queue.
//------------------------------------------------------------------------------
typedef struct
{
    // Message queue used for timer signaling
    MSG_Q_ID m_msgQueue;
    // Data to be sent in message queue
    unsigned long m_sigData;
}  tHrtimerSigMq;

//------------------------------------------------------------------------------
// This union contains the signaling parameters for the different signaling
// types.
//------------------------------------------------------------------------------
typedef union
{
    // Signaling parameters for callback signaling
    tHrtimerSigCb m_signalCallback;
    // Signaling parameters for signaling using a semaphore
    tHrtimerSigSem m_signalSem;
    // Signaling parameters for signaling using a message queue
    tHrtimerSigMq m_signalMq;
}  tHrtimerSigparam;

//------------------------------------------------------------------------------
// sigType specifies the timer signaling type to be used by a high-
// resolution timer.
//------------------------------------------------------------------------------
typedef struct
{
    // The signaling type to be used.
    eHrtimerSigType sigType;
    // The parameters for this signaling type
    tHrtimerSigparam sigParam;
}  tHrtimerSig;

//------------------------------------------------------------------------------
// eHrtimerReturn specifies valid return values for the high-resolution
// timer functions.
//------------------------------------------------------------------------------
typedef enum
{

    // Call was successfull
    kHrtimerReturnOk = 0,
    // Returned with general error
    kHrtimerReturnError = -1,
    // High-resolution timer library is not initilized
    kHrtimerReturnNotInit = -2,
    // Invalid Timeout value
    kHrtimerReturnInvalidTimeout = -3
} eHrtimerReturn;

//============================================================================//
// Function declarations                                                      //
//============================================================================//
int hrtimer_create(clockid_t clockid, tHrtimerSig* sigevent,
                   timer_t * pTimerId_p);
int hrtimer_delete(timer_t timerid);
int hrtimer_clock_gettime(clockid_t clk_id, struct timespec* tp);
int hrtimer_clock_getres(clockid_t clk_id, struct timespec* res);
int hrtimer_gettime(timer_t timerid, struct itimerspec* curr_value);
int hrtimer_init(int iCbPrio_p, int iCbStackSize_p);
void hrtimer_nanosleep(struct timespec* req);
int hrtimer_settime(timer_t timerid, int flags, struct itimerspec* new_value,
                    struct itimerspec* old_value);

void hrtimer_shutdown(void);
int hrtimer_setCallback(timer_t timerid, tHrTimerCbFuncPtr fpCallback_p,
                        void * pArg_p);
void hrtimer_setTimespec(struct timespec * ts_p, time_t sec_p,
                         long long nsec_p);
struct timespec hrtimer_subTimespec(struct timespec value1_p,
                                    struct timespec value2_p);
struct timespec hrtimer_addTimespec(struct timespec value1_p,
                                    struct timespec value2_p);
int hrtimer_compareTimespec(const struct timespec * pValue1_p,
                            const struct timespec * pValue2_p);

#ifdef    INCLUDE_HRTIMER_SHOW
void hrtimer_show(void);
void hrtimer_showArmed(void);
#endif

#endif /* ifndef __INC_HRTIMERLIB_H */
