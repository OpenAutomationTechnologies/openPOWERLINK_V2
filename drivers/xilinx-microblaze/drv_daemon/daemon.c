/**
********************************************************************************
\file   daemon.c

\brief  POWERLINK FPGA Master daemon for Pcp (kernel part)

This is the daemon for the Pcp (kernel part) of the Xilinx Microblaze POWERLINK
master demo application.

\ingroup module_daemon
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2014, Kalycito Infotech Private Limited.
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

#include <unistd.h>
#include <mb_interface.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <kernel/ctrlk.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   initPowerlink(void);
static void         shutdownPowerlink(void);
static void         backgroundProcess(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    main function

Calls the POWERLINK initialization and background task

\return 0

\ingroup module_daemon
*/
//------------------------------------------------------------------------------
INT main(void)
{
    tOplkError    ret;

    ret = target_init();
    if (kErrorOk != ret)
    {
        PRINTF("Target Initialization Failed\n\n");
        return ret;
    }

    while (1)
    {
        PRINTF("\n");

        ret = initPowerlink();

        PRINTF("Initialization returned with \"%s\" (0x%X)\n",
               debugstr_getRetValStr(ret), ret);

        if (ret != kErrorOk)
            break;

        backgroundProcess();

        PRINTF("Background loop stopped.\nShutdown Kernel Stack\n");

        shutdownPowerlink();

        usleep(1000000U);
    }

    PRINTF("halt terminal\n%c", 4);

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack initialization

This function initializes the communication stack and configures objects.

\return This function returns tOplkError error codes.
*/
//------------------------------------------------------------------------------
static tOplkError initPowerlink(void)
{
    tOplkError    ret;

    ret = ctrlk_init();

    if (ret != kErrorOk)
    {
        printf("Could not initialize control module\n");
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack shutdown

This function initiates shutdown of the stack by freeing the allocated
resources and suspending all processes

\ingroup module_daemon
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(void)
{
    ctrlk_exit();
}

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack background tasks

Routine to handle background tasks of POWERLINK such as
- Event handling
- Packet processing
- Status and command exchange

\ingroup module_daemon
*/
//------------------------------------------------------------------------------
static void backgroundProcess(void)
{
    BOOL    fExit = FALSE;

    while (1)
    {
        ctrlk_updateHeartbeat();
        fExit = ctrlk_process();

        if (fExit != FALSE)
            break;
    }
}
