/**
********************************************************************************
\file   daemon.c

\brief  openPOWERLINK kernel stack daemon running in Linux userspace

This file contains the implementation of a openPOWERLINK kernel stack daemon
which runs in Linux userspace.

\ingroup    module_driver_linux
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.
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
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sched.h>

#include <oplk/oplkinc.h>
#include <common/oplkinc.h>
#include <kernel/ctrlk.h>
#include <console/console.h>
#include <common/target.h>

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static char* pLogFile_g = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Main function

This is the main function of the openPOWERLINK Linux kernel stack daemon
running in Linux userspace.

\param  argc                    Number of arguments
\param  argv                    Pointer to argument strings

\return Returns an exit code

\ingroup module_driver_linux
*/
//------------------------------------------------------------------------------
int  main (int argc, char** argv)
{
    tOplkError                  ret = kErrorOk;
    char                        cKey = 0;
    BOOL                        fExit;

    struct sched_param          schedParam;
    int                         opt;

    /* get command line parameters */
    while ((opt = getopt(argc, argv, "l:")) != -1)
    {
        switch (opt)
        {
        case 'l':
            pLogFile_g = optarg;
            break;

        default: /* '?' */
            fprintf(stderr, "Usage: %s [-l LOGFILE]\n", argv[0]);
            goto Exit;
        }
    }

    /* adjust process priority */
    if (nice(-20) == -1)          // push nice level in case we have no RTPreempt
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set nice value! (%s)\n", __func__, strerror(errno));
    }
    schedParam.sched_priority = MAIN_THREAD_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters! %d\n",
                              __func__, schedParam.sched_priority);
    }

#ifdef SET_CPU_AFFINITY
    {
        /* binds all openPOWERLINK threads to the first CPU core */
        cpu_set_t   affinity;

        CPU_ZERO(&affinity);
        CPU_SET(0, &affinity);
        sched_setaffinity(0, sizeof(cpu_set_t), &affinity);
    }
#endif

    /* Initialize target specific stuff */
    target_init();

    PRINTF("----------------------------------------------------\n");
    PRINTF("openPOWERLINK kernel stack daemon\n");
    PRINTF("using openPOWERLINK Stack: %s\n", PLK_DEFINED_STRING_VERSION);
    PRINTF("----------------------------------------------------\n");

    ret = ctrlk_init();
    if (ret != kErrorOk)
    {
        TRACE("Could not initialize control module\n");
        goto Exit;
    }

    // initialize POWERLINK stack
    PRINTF("Running...\n");

    fExit = FALSE;
    while (!fExit)
    {
        target_msleep(1);
        if (console_kbhit())
        {
            cKey = (BYTE)console_getch();
            if (cKey == 0x1B)
                fExit = TRUE;
        }
        else
        {
            ctrlk_updateHeartbeat();
            fExit = ctrlk_process();
        }
    }

    printf("\nShutdown openPOWERLINK kernel daemon...\n");
    ctrlk_exit();

Exit:
    PRINTF("Exiting\n");
    return ret;
}
