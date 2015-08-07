/**
********************************************************************************
\file   system-linux.c

\brief  System specific functions for Linux

The file implements the system specific functions for Linux used by the
openPOWERLINK demo applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>

#include "system.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SET_CPU_AFFINITY
#define MAIN_THREAD_PRIORITY            20

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static BOOL    fTermSignalReceived_g = FALSE;

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
#if defined(CONFIG_USE_SYNCTHREAD)
typedef struct
{
    tSyncCb         pfnSyncCb;
    BOOL            fTerminate;
} tSyncThreadInstance;
#endif

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
#if defined(CONFIG_USE_SYNCTHREAD)
static pthread_t                syncThreadId_l;
static tSyncThreadInstance      syncThreadInstance_l;
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void handleTermSignal(int signum);

#if defined(CONFIG_USE_SYNCTHREAD)
static void* powerlinkSyncThread(void* arg);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize system

The function initializes important stuff on the system for openPOWERLINK to
work correctly.

\return The function returns 0 if the initialization has been successful,
        otherwise -1.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int system_init(void)
{
    struct sched_param          schedParam;

    /* adjust process priority */
    if (nice(-20) == -1)         // push nice level in case we have no RTPreempt
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set nice value! (%s)\n", __func__, strerror(errno));
    }
    schedParam.sched_priority = MAIN_THREAD_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters! %d\n",
                              __func__, schedParam.sched_priority);
    }

    // Register termination handler for signals with termination semantics
    struct sigaction new_action;

    new_action.sa_handler = handleTermSignal;
    (void)sigemptyset(&new_action.sa_mask);
    new_action.sa_flags = 0;

    (void)sigaction(SIGINT,  &new_action, NULL);    // Sent via CTRL-C
    (void)sigaction(SIGTERM, &new_action, NULL);    // Generic signal used to cause program termination.
    (void)sigaction(SIGQUIT, &new_action, NULL);    // Terminate because of abnormal condition

#ifdef SET_CPU_AFFINITY
    {
        /* binds all openPOWERLINK threads to the second CPU core */
        cpu_set_t                   affinity;

        CPU_ZERO(&affinity);
        CPU_SET(1, &affinity);
        sched_setaffinity(0, sizeof(cpu_set_t), &affinity);
    }
#endif

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown system

The function shuts down the system.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_exit(void)
{

}

//------------------------------------------------------------------------------
/**
\brief  Determines whether a termination signal has been received

The function can be used by the application to react on termination request.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
BOOL system_getTermSignalState(void)
{
    return fTermSignalReceived_g;
}


//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds has elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_msleep(unsigned int milliSeconds_p)
{
    struct  timeval timeout;
    fd_set          readFds;
    int             maxFd;
    int             selectRetVal;
    unsigned int    seconds;
    unsigned int    microSeconds;

    // initialize file descriptor set
    maxFd = 0 + 1;
    FD_ZERO(&readFds);

    // Calculate timeout values
    seconds = milliSeconds_p / 1000;
    microSeconds = (milliSeconds_p - (seconds * 1000)) * 1000;

    // initialize timeout value
    timeout.tv_sec = seconds;
    timeout.tv_usec = microSeconds;

    selectRetVal = select(maxFd, &readFds, NULL, NULL, &timeout);
    switch (selectRetVal)
    {
        case 0:     // select timeout occurred, no packet received
            break;

        case -1:    // select error occurred
            break;

        default:    // packet available for receive
            break;
    }
}

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Start synchronous data thread

The function starts the thread used for synchronous data handling.

\param  pfnSync_p           Pointer to sync callback function

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_startSyncThread(tSyncCb pfnSync_p)
{
    syncThreadInstance_l.pfnSyncCb = pfnSync_p;
    syncThreadInstance_l.fTerminate = FALSE;

    // create sync thread
    if (pthread_create(&syncThreadId_l, NULL, &powerlinkSyncThread,
                       &syncThreadInstance_l) != 0)
    {
        return;
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(syncThreadId_l, "oplkdemo-sync");
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Stop synchronous data thread

The function stops the thread used for synchronous data handling.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_stopSyncThread(void)
{
    syncThreadInstance_l.fTerminate = TRUE;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Handle termination requests

This functions can be used to react on signals with termination semantics,
and remembers in a flag that the user or the system asked the program to shut down.
The application can than check this flag.
*/
//------------------------------------------------------------------------------
static void handleTermSignal(int signum)
{
    switch (signum)
    {
        case SIGINT:    // Signals with termination semantics
        case SIGTERM:   // trigger a flag change
        case SIGQUIT:
            fTermSignalReceived_g = TRUE;
            break;

        default:        // All other signals are ignored by this handler
            break;
    }
}

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Synchronous application thread

This function implements the synchronous application thread.

\param  arg             Needed for thread interface not used
*/
//------------------------------------------------------------------------------
static void* powerlinkSyncThread(void* arg)
{
    tSyncThreadInstance*     pSyncThreadInstance = (tSyncThreadInstance*)arg;

    printf("Synchronous data thread is starting...\n");
    while (!pSyncThreadInstance->fTerminate)
    {
        pSyncThreadInstance->pfnSyncCb();
    }
    printf("Synchronous data thread is terminating...\n");

    return NULL;
}
#endif

/// \}
