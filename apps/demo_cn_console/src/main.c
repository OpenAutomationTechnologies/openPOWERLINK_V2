/**
********************************************************************************
\file   main.c

\brief  Main file of console CN demo application

This file contains the main file of the openPOWERLINK CN console demo
application.

\ingroup module_demo_cn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


/***************************************************************************/
#if (TARGET_SYSTEM == _WIN32_)
#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#endif  // (TARGET_SYSTEM == _WIN32_)

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/Epl.h>
#include <stdio.h>
#include <limits.h>

#include <app.h>
#include <system/system.h>
#include <getopt/getopt.h>
#include <console/console.h>
#include <oplk/EplTarget.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN   50000
#define NODEID      1                   // could be changed by command param
#define IP_ADDR     0xc0a86401          // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00          // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
CONST BYTE aMacAddr_l[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static BOOL fGsOff_l;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_USE_PCAP)
tEplKernel selectPcapDevice(char *pDevName_p);
#endif
void initEvents (BOOL* pfGsOff_p);
tEplKernel processEvents(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    UINT32      nodeId;
    char*       pLogFile;
} tOptions;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int getOptions(int argc_p, char **argv_p, tOptions* pOpts_p);
static tEplKernel initPowerlink(UINT32 cycleLen_p, const BYTE* macAddr_p, UINT32 nodeId_p);
static void loopMain(void);
static void shutdownPowerlink(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  main function

This is the main function of the openPOWERLINK console MN demo application.

\param  argc                    Number of arguments
\param  argv                    Pointer to argument strings

\return Returns an exit code

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
int main (int argc, char **argv)
{
    tEplKernel                  ret = kEplSuccessful;
    tOptions                    opts;

    getOptions (argc, argv, &opts);

    if (initSystem() < 0)
    {
        printf ("Error initializing system!");
        return 0;
    }

    initEvents(&fGsOff_l);

    printf("----------------------------------------------------\n");
    printf("openPOWERLINK console CN DEMO application\n");
    printf("----------------------------------------------------\n");

    if ((ret = initPowerlink(CYCLE_LEN, aMacAddr_l, opts.nodeId))
        != kEplSuccessful)
        goto Exit;

    if((ret = initApp()) != kEplSuccessful)
        goto Exit;

    loopMain();

Exit:
    shutdownPowerlink();
    shutdownApp();
    shutdownSystem();

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack.

\param  cycleLen_p              Length of POWERLINK cycle.
\param  macAddr_p               MAC address to use for POWERLINK interface.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initPowerlink(UINT32 cycleLen_p, const BYTE* macAddr_p, UINT32 nodeId_p)
{
    tEplKernel                  ret = kEplSuccessful;
    static tEplApiInitParam     initParam;
    char*                       sHostname = HOSTNAME;
    static char                 devName[128];

    printf ("Initializing openPOWERLINK stack...\n");

#if defined(CONFIG_USE_PCAP)
    selectPcapDevice(devName);
#endif

    EPL_MEMSET(&initParam, 0, sizeof (initParam));
    initParam.m_uiSizeOfStruct = sizeof (initParam);

    // pass selected device name to Edrv
    initParam.m_HwParam.m_pszDevName = devName;
    initParam.m_uiNodeId = nodeId_p;
    initParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | initParam.m_uiNodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    EPL_MEMCPY(initParam.m_abMacAddress, macAddr_p, sizeof (initParam.m_abMacAddress));

    initParam.m_fAsyncOnly = FALSE;

    initParam.m_dwFeatureFlags            = UINT_MAX;
    initParam.m_dwCycleLen                = cycleLen_p;       // required for error detection
    initParam.m_uiIsochrTxMaxPayload      = 40;               // const
    initParam.m_uiIsochrRxMaxPayload      = 36;               // const
    initParam.m_dwPresMaxLatency          = 50000;            // const; only required for IdentRes
    initParam.m_uiPreqActPayloadLimit     = 36;               // required for initialisation (+28 bytes)
    initParam.m_uiPresActPayloadLimit     = 40;               // required for initialisation of Pres frame (+28 bytes)
    initParam.m_dwAsndMaxLatency          = 150000;           // const; only required for IdentRes
    initParam.m_uiMultiplCycleCnt         = 0;                // required for error detection
    initParam.m_uiAsyncMtu                = 1500;             // required to set up max frame size
    initParam.m_uiPrescaler               = 2;                // required for sync
    initParam.m_dwLossOfFrameTolerance    = 500000;
    initParam.m_dwAsyncSlotTimeout        = 3000000;
    initParam.m_dwWaitSocPreq             = 150000;
    initParam.m_dwDeviceType              = UINT_MAX;               // NMT_DeviceType_U32
    initParam.m_dwVendorId                = UINT_MAX;               // NMT_IdentityObject_REC.VendorId_U32
    initParam.m_dwProductCode             = UINT_MAX;               // NMT_IdentityObject_REC.ProductCode_U32
    initParam.m_dwRevisionNumber          = UINT_MAX;               // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.m_dwSerialNumber            = UINT_MAX;               // NMT_IdentityObject_REC.SerialNo_U32
    initParam.m_dwApplicationSwDate       = 0;
    initParam.m_dwApplicationSwTime       = 0;

    initParam.m_dwSubnetMask              = SUBNET_MASK;
    initParam.m_dwDefaultGateway          = 0;
    EPL_MEMCPY(initParam.m_sHostname, sHostname, sizeof(initParam.m_sHostname));
    initParam.m_uiSyncNodeId              = EPL_C_ADR_SYNC_ON_SOA;
    initParam.m_fSyncOnPrcNode            = FALSE;

    // set callback functions
    initParam.m_pfnCbEvent = processEvents;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.m_pfnCbSync  = processSync;
#else
    initParam.m_pfnCbSync = NULL;
#endif

    // initialize POWERLINK stack
    ret = oplk_init(&initParam);
    if(ret != kEplSuccessful)
    {
        printf("oplk_init() failed (Error:0x%x!\n", ret);
        return ret;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Main loop of demo application

This function implements the main loop of the demo application.
- It creates the sync thread which is responsible for the synchronous data
  application.
- It sends a NMT command to start the stack
- It loops and reacts on commands from the command line.
*/
//------------------------------------------------------------------------------
static void loopMain(void)
{
    tEplKernel              ret = kEplSuccessful;
    char                    cKey = 0;
    BOOL                    fExit = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

#if defined(CONFIG_USE_SYNCTHREAD)
    startSyncThread(processSync);
#endif

#endif

    // start processing
    ret = oplk_execNmtCommand(kNmtEventSwReset);
    if (ret != kEplSuccessful)
    {
        return;
    }

    printf("start POWERLINK Stack... ok\n");
    printf("Digital I/O interface with openPOWERLINK is ready!\n");
    printf("\n-------------------------------\n");
    printf("Press Esc to leave the programm\n");
    printf("Press r to reset the node\n");
    printf("Press i to increase digital input\n");
    printf("Press d to decrease digital input\n");
    printf("Press p to print digital outputs\n");
    printf("-------------------------------\n\n");

    setupInputs();

    // wait for key hit
    while (!fExit)
    {
        if(console_kbhit())
        {
            cKey    = (BYTE) console_getch();
            switch (cKey)
            {
                case 'r':
                {
                    ret = oplk_execNmtCommand(kNmtEventSwReset);
                    if (ret != kEplSuccessful)
                    {
                        fExit = TRUE;
                    }
                    break;
                }
                case 'i':
                    increaseInputs();
                    break;

                case 'd':
                    decreaseInputs();
                    break;

                case 'p':
                    printOutputs();
                    break;

                case 0x1B:
                    fExit = TRUE;
                    break;

                default:
                {
                    break;
                }
            }
        }

        if( system_getTermSignalState() == TRUE )
        {
            fExit = TRUE;
            PRINTF("Received termination signal, exiting...\n");
        }

        if (oplk_checkKernelStack() == FALSE)
        {
            fExit = TRUE;
            PRINTF("Kernel stack has gone! Exiting...\n");
        }

#if defined(CONFIG_USE_SYNCTHREAD) || defined(CONFIG_KERNELSTACK_DIRECTLINK)
        msleep(100);
#else
        processSync();
#endif
    }

#if (TARGET_SYSTEM == _WIN32_)
    PRINTF("Press Enter to quit!\n");
    console_getch();
#endif

    return;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the demo application

The function shut's down the demo application.
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(void)
{
    UINT                i;

    fGsOff_l = FALSE;

    // halt the NMT state machine so the processing of POWERLINK frames stops
    oplk_execNmtCommand(kNmtEventSwitchOff);

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++)
    {
        if (fGsOff_l)
            break;
    }

    printf ("Stack in State off ... Shutdown\n");

    oplk_shutdown();

}

//------------------------------------------------------------------------------
/**
\brief  Get command line parameters

The function parses the supplied command line parameters and stores the
options at pOpts_p.

\param  argc_p                  Argument count.
\param  argc_p                  Pointer to arguments.
\param  pOpts_p                 Pointer to store options

\return The function returns the parsing status.
\retval 0           Successfully parsed
\retval -1          Parsing error
*/
//------------------------------------------------------------------------------
static int getOptions(int argc_p, char **argv_p, tOptions* pOpts_p)
{
    int                         opt;

    /* setup default parameters */
    pOpts_p->nodeId = NODEID;
    pOpts_p->pLogFile = NULL;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "n:l:")) != -1)
    {
        switch (opt)
        {
        case 'n':
            pOpts_p->nodeId = strtoul(optarg, NULL, 10);
            break;

        case 'l':
            pOpts_p->pLogFile = optarg;
            break;

        default: /* '?' */
            fprintf (stderr, "Usage: %s [-n NODE_ID] [-l LOGFILE]\n", argv_p[0]);
            return -1;
        }
    }
    return 0;
}

///\}

