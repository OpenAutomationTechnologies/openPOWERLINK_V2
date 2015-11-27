/**
********************************************************************************
\file   main.c

\brief  Main file of console MN demo application

This file contains the main file of the openPOWERLINK MN console demo
application.

\ingroup module_demo_mn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <limits.h>
#include <string.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <system/system.h>
#include <getopt/getopt.h>
#include <console/console.h>

#include <eventlog/eventlog.h>

#if defined(CONFIG_USE_PCAP)
#include <pcap/pcap-console.h>
#endif

#include "app.h"
#include "event.h"


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN         UINT_MAX
#define NODEID            0xF0                //=> MN
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
const BYTE aMacAddr_g[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static BOOL fGsOff_l;

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
typedef struct
{
    char            cdcFile[256];
    char*           pLogFile;
    tEventlogFormat logFormat;
    UINT32          logLevel;
    UINT32          logCategory;
    char            devName[128];
} tOptions;

typedef struct
{
    tNmtState       nodeState[254];
} tDemoNodeInfo;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int getOptions(int argc_p, char** argv_p, tOptions* pOpts_p);
static tOplkError initPowerlink(UINT32 cycleLen_p, char* pszCdcFileName_p,
                                char* devName_p, const BYTE* macAddr_p);
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
int main(int argc, char** argv)
{
    tOplkError                  ret = kErrorOk;
    tOptions                    opts;

    if (getOptions(argc, argv, &opts) < 0)
        return 0;

    if (system_init() != 0)
    {
        fprintf(stderr, "Error initializing system!");
        return 0;
    }

    eventlog_init(opts.logFormat, opts.logLevel, opts.logCategory, (tEventlogOutputCb)console_printlogadd);

    initEvents(&fGsOff_l);

    printf("----------------------------------------------------\n");
    printf("openPOWERLINK console MN DEMO application\n");
    printf("using openPOWERLINK Stack: %s\n", oplk_getVersionString());
    printf("----------------------------------------------------\n");

    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryGeneric,
                          "demo_mn_console: Stack Version:%s Stack Configuration:0x%08X",
                          oplk_getVersionString(), oplk_getStackConfiguration());

    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryGeneric,
                         "Using CDC file: %s", opts.cdcFile);

    if ((ret = initPowerlink(CYCLE_LEN, opts.cdcFile, opts.devName, aMacAddr_g)) != kErrorOk)
        goto Exit;

    if ((ret = initApp()) != kErrorOk)
        goto Exit;

    loopMain();

Exit:
    shutdownApp();
    shutdownPowerlink();
    system_exit();

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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initPowerlink(UINT32 cycleLen_p, char* pszCdcFileName_p,
                                char* devName_p, const BYTE* macAddr_p)
{
    tOplkError                  ret = kErrorOk;
    static tOplkApiInitParam    initParam;
    static char                 devName[128];

    printf("Initializing openPOWERLINK stack...\n");
    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryControl,
                          "Initializing openPOWERLINK stack");

#if defined(CONFIG_USE_PCAP)
    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryGeneric,
                         "Using libpcap for network access");
    if (devName_p[0] == '\0')
    {
        if (selectPcapDevice(devName) < 0)
            return kErrorIllegalInstance;
    }
    else
    {
        strncpy(devName, devName_p, 128);
    }
#else
    UNUSED_PARAMETER(devName_p);
#endif

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = NODEID;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;       // required for error detection
    initParam.isochrTxMaxPayload      = 256;              // const
    initParam.isochrRxMaxPayload      = 256;              // const
    initParam.presMaxLatency          = 50000;            // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 36;               // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 36;               // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 150000;           // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                // required for error detection
    initParam.asyncMtu                = 1500;             // required to set up max frame size
    initParam.prescaler               = 2;                // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;         // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;         // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;         // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;         // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = processEvents;
#if defined(CONFIG_INCLUDE_SDOS)
    initParam.pfnSdoSrvProcessObdWrite = obdal_proccessWrite;
    initParam.pfnSdoSrvProcessObdRead = obdal_proccessRead;
#endif

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.pfnCbSync  = processSync;
#else
    initParam.pfnCbSync  = NULL;
#endif

    // initialize POWERLINK stack
    ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_initialize() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        eventlog_printMessage(kEventlogLevelFatal, kEventlogCategoryControl, "oplk_init() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    ret = oplk_create(&initParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_create() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    ret = oplk_setCdcFilename(pszCdcFileName_p);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_setCdcFilename() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        eventlog_printMessage(kEventlogLevelFatal, kEventlogCategoryControl, "oplk_setCdcFilename() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    return kErrorOk;
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
    tOplkError              ret = kErrorOk;
    char                    cKey = 0;
    BOOL                    fExit = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

#if defined(CONFIG_USE_SYNCTHREAD)
    system_startSyncThread(processSync);
#endif

#endif

    // start stack processing by sending a NMT reset command
    ret = oplk_execNmtCommand(kNmtEventSwReset);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret), ret);
        return;
    }

    printf("\n-------------------------------\n");
    printf("Press Esc to leave the program\n");
    printf("Press r to reset the node\n");
    printf("-------------------------------\n\n");

    while (!fExit)
    {
        if (console_kbhit())
        {
            cKey = (char)console_getch();
            switch (cKey)
            {
                case 'r':
                    ret = oplk_execNmtCommand(kNmtEventSwReset);
                    if (ret != kErrorOk)
                    {
                        fprintf(stderr, "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret), ret);
                        fExit = TRUE;
                    }
                    break;

                case 'c':
                    ret = oplk_execNmtCommand(kNmtEventNmtCycleError);
                    if (ret != kErrorOk)
                    {
                        fprintf(stderr, "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret), ret);
                        fExit = TRUE;
                    }
                    break;

                case 0x1B:
                    fExit = TRUE;
                    break;

                default:
                    break;
            }
        }

        if (system_getTermSignalState() == TRUE)
        {
            fExit = TRUE;
            printf("Received termination signal, exiting...\n");
            eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryControl, "Received termination signal, exiting...");
        }

        if (oplk_checkKernelStack() == FALSE)
        {
            fExit = TRUE;
            fprintf(stderr, "Kernel stack has gone! Exiting...\n");
            eventlog_printMessage(kEventlogLevelFatal, kEventlogCategoryControl, "Kernel stack has gone! Exiting...");
        }

#if defined(CONFIG_USE_SYNCTHREAD) || defined(CONFIG_KERNELSTACK_DIRECTLINK)
        system_msleep(100);
#else
        processSync();
#endif

    }

#if (TARGET_SYSTEM == _WIN32_)
    printf("Press Enter to quit!\n");
    console_getch();
#endif

}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the demo application

The function shuts down the demo application.
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(void)
{
    UINT                i;
    tOplkError          ret = kErrorOk;

    // NMT_GS_OFF state has not yet been reached
    fGsOff_l = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK) && defined(CONFIG_USE_SYNCTHREAD)
    system_stopSyncThread();
    system_msleep(100);
#endif

    // halt the NMT state machine so the processing of POWERLINK frames stops
    ret = oplk_execNmtCommand(kNmtEventSwitchOff);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret), ret);
    }

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++)
    {
        if (fGsOff_l)
            break;
    }

    printf("Stack is in state off ... Shutdown\n");
    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryControl,
                          "Stack is in state off ... Shutdown openPOWERLINK");

    oplk_destroy();
    oplk_exit();
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
static int getOptions(int argc_p, char** argv_p, tOptions* pOpts_p)
{
    int                         opt;

    /* setup default parameters */
    strncpy(pOpts_p->cdcFile, "mnobd.cdc", 256);
    strncpy(pOpts_p->devName, "\0", 128);
    pOpts_p->pLogFile = NULL;
    pOpts_p->logFormat = kEventlogFormatReadable;
    pOpts_p->logCategory = 0xffffffff;
    pOpts_p->logLevel = 0xffffffff;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "c:l:pv:t:d:")) != -1)
    {
        switch (opt)
        {
            case 'c':
                strncpy(pOpts_p->cdcFile, optarg, 256);
                break;

            case 'd':
                strncpy(pOpts_p->devName, optarg, 128);
                break;

            case 'p':
                pOpts_p->logFormat = kEventlogFormatParsable;
                break;

           case 'v':
                pOpts_p->logLevel = strtoul(optarg, NULL, 16);
                break;

           case 't':
                pOpts_p->logCategory = strtoul(optarg, NULL, 16);
                break;

            default: /* '?' */
                printf("Usage: %s [-c CDC-FILE] [-d DEV_NAME] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]\n", argv_p[0]);
                printf(" -p: Use parsable log format\n");
                printf(" -d DEV_NAME: Ethernet device name to use e.g. eth1 (only used for pcap, not used for kernel driver!)\n");
                printf("              If option is skipped the program prompts for the interface.\n");
                printf(" -v LOGLEVEL: A bit mask with log levels to be printed in the event logger\n");
                printf(" -t LOGCATEGORY: A bit mask with log categories to be printed in the event logger\n");
                return -1;
        }
    }
    return 0;
}

/// \}
