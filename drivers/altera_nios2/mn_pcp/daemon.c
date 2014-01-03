/**
********************************************************************************
\file   daemon.c

\brief  POWERLINK FPGA Master daemon for Pcp (kernel part)

This is the daemon for the Pcp (kernel part) of the Altera Nios II POWERLINK
master demo application.

\ingroup module_daemon
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <system.h>
#include <sys/alt_cache.h>
#include <unistd.h>
#include <altera_avalon_pio_regs.h>

#include <Epl.h>
#include <EplErrStr.h>
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
static tEplKernel initPlk (void);
static void shtdPlk (void);
static void bgtPlk (void);

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
int main (void)
{
    tEplKernel Ret;

    alt_icache_flush_all();
    alt_dcache_flush_all();

    PRINTF("CPU NIOS II /%s (%s)\n", ALT_CPU_CPU_IMPLEMENTATION, ALT_CPU_NAME);
    PRINTF("FREQ = %d MHZ\n", ALT_CPU_CPU_FREQ/1000000U);
    PRINTF("DCACHE = %d BYTE\n", ALT_CPU_DCACHE_SIZE);
    PRINTF("ICACHE = %d BYTE\n", ALT_CPU_ICACHE_SIZE);

    while(1)
    {
        PRINTF("\n");

        Ret = initPlk();

        PRINTF("Initialization returned with \"%s\" (0x%X)\n",
                EplErrStrGetMessage(Ret), Ret);

        if(Ret != kEplSuccessful)
            break;

        bgtPlk();

        PRINTF("Background loop stopped.\nShutdown Kernel Stack\n");

        shtdPlk();

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

\return This function returns tEplKernel error codes.
*/
//------------------------------------------------------------------------------
static tEplKernel initPlk (void)
{
    tEplKernel Ret;

    Ret = ctrlk_init();

    if (Ret != kEplSuccessful)
    {
        printf ("Could not initialize control module\n");
        goto Exit;
    }

Exit:
    return Ret;
}

static void shtdPlk (void)
{
    ctrlk_exit();
}

//------------------------------------------------------------------------------
/**
\brief    openPOWERLINK stack background tasks

This function runs the background tasks
*/
//------------------------------------------------------------------------------
static void bgtPlk (void)
{
    BOOL fExit = FALSE;

    while(1)
    {
        ctrlk_updateHeartbeat();
        fExit = ctrlk_process();

        if(fExit != FALSE)
            break;
    }
}
