/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  openPOWERLINK CN console demo application

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holders nor the names of its
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

****************************************************************************/


/***************************************************************************/
#if (TARGET_SYSTEM == _WIN32_)
#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#endif  // (TARGET_SYSTEM == _WIN32_)

/* includes */
#include "Epl.h"
#include <stddef.h>

#if (TARGET_SYSTEM == _LINUX_)
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/resource.h>
#include <errno.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>

#ifndef CONFIG_POWERLINK_USERSTACK
#include <pthread.h>
#else
#include <pcap.h>
#endif

#elif (TARGET_SYSTEM == _WIN32_)
#include <pcap.h>
#endif  // (TARGET_SYSTEM == _WIN32_)

#include "EplTgtConio.h"

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
#if (TARGET_SYSTEM == _LINUX_)

#define SET_CPU_AFFINITY
#define MAIN_THREAD_PRIORITY            20

#elif (TARGET_SYSTEM == _WIN32_)

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#endif // (TARGET_SYSTEM == _WIN32_)

#define IP_ADDR     0xc0a86401          // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00          // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "
//#define IF_ETH      EPL_VETH_NAME


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------
/* structure for input process image */
typedef struct
{
   BYTE    digitalIn;
} PI_IN;

/* structure for output process image */
typedef struct
{
   BYTE    digitalOut;
} PI_OUT;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static unsigned int uiNodeId_g = 1;         // used NODE ID
static unsigned int uiCycleLen_g = 0;
static char *pLogFile_g = NULL;

/* process image */
static PI_IN AppProcessImageIn_g;           // input process image
static PI_OUT AppProcessImageOut_g;         // output process image
static tEplApiProcessImageCopyJob AppProcessImageCopyJob_g;

/* application variables */
BYTE    digitalIn_g;                        // 8 bit digital input
BYTE    digitalOut_g;                       // 8 bit digital output

#ifndef CONFIG_POWERLINK_USERSTACK
static pthread_t eventThreadId;
static pthread_t syncThreadId;
#endif

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
// this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.

tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);
tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p);
tEplKernel PUBLIC AppCbSync(void);
static void printOutputs(void);
static void printInputs(void);

#ifndef CONFIG_POWERLINK_USERSTACK
static void *powerlinkEventThread(void * arg);
static void *powerlinkSyncThread(void * arg);
#endif

//---------------------------------------------------------------------------
// Function:            printlog
//
// Description:         print logging entry
//
// Parameters:          fmt             format string
//                      ...             arguments to print
//
// Returns:             N/A
//---------------------------------------------------------------------------
static void printlog(char *fmt, ...)
{
    va_list             arglist;
    time_t              timeStamp;
    struct tm           timeVal;
    char                timeStr[20];

    time(&timeStamp);

#if (TARGET_SYSTEM == _LINUX_)
    localtime_r(&timeStamp, &timeVal);
    strftime(timeStr, 20, "%Y/%m/%d %H:%M:%S", &timeVal);
#else
    {
    struct tm           *p_timeVal;

    p_timeVal = localtime(&timeStamp);
    strftime(timeStr, 20, "%Y/%m/%d %H:%M:%S", p_timeVal);
    }
#endif

    fprintf (stderr, "%s - ", timeStr);
    va_start(arglist, fmt);
    vfprintf(stderr, fmt, arglist);
    va_end(arglist);
}


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:            main
//
// Description:         main function of demo application
//
// Parameters:
//
// Returns:
//---------------------------------------------------------------------------
int  main (int argc, char **argv)
{
    tEplKernel                  EplRet = kEplSuccessful;
    static tEplApiInitParam     EplApiInitParam;
    char*                       sHostname = HOSTNAME;
    char                        cKey = 0;
    tEplObdSize                 ObdSize;
    unsigned int                uiVarEntries;

#ifdef CONFIG_POWERLINK_USERSTACK
#if (TARGET_SYSTEM == _LINUX_)
    struct sched_param          schedParam;
#endif

    // variables for Pcap
    char                        sErr_Msg[ PCAP_ERRBUF_SIZE ];
    char                        devName[128];
    pcap_if_t *                 alldevs;
    pcap_if_t *                 seldev;
    int                         i = 0;
    int                         inum;
#endif


#if (TARGET_SYSTEM == _LINUX_)
    {
    int                         opt;

    /* get command line parameters */
    while ((opt = getopt(argc, argv, "n:l:")) != -1)
    {
        switch (opt)
        {
        case 'n':
            uiNodeId_g = strtoul(optarg, NULL, 10);
            break;

        case 'l':
            pLogFile_g = optarg;
            break;

        default: /* '?' */
            fprintf (stderr, "Usage: %s [-c CYCLE_TIME] [-l LOGFILE]\n", argv[0]);
            goto Exit;
        }
    }
    }
#endif

#ifdef CONFIG_POWERLINK_USERSTACK

#if (TARGET_SYSTEM == _LINUX_)
    /* adjust process priority */
    if (nice (-20) == -1)         // push nice level in case we have no RTPreempt
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't set nice value! (%s)\n", __func__, strerror(errno));
    }
    schedParam.__sched_priority = MAIN_THREAD_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters! %d\n",
                __func__, schedParam.__sched_priority);
    }

#ifdef SET_CPU_AFFINITY
    {
        /* binds all openPOWERLINK threads to the first CPU core */
        cpu_set_t                   affinity;

        CPU_ZERO(&affinity);
        CPU_SET(0, &affinity);
        sched_setaffinity(0, sizeof(cpu_set_t), &affinity);
    }
#endif

    /* Initialize target specific stuff */
    EplTgtInit();

#elif (TARGET_SYSTEM == _WIN32_)

    // activate realtime priority class
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    // lower the priority of this thread
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

#endif // (TARGET_SYSTEM == _WIN32_)

#endif // CONFIG_POWERLINK_USERSTACK

    /* Enabling ftrace for debugging */
    FTRACE_OPEN();
    FTRACE_ENABLE(TRUE);

    /*
    EPL_DBGLVL_ALWAYS_TRACE("%s(): Main Thread Id:%ld\n", __func__,
                             syscall(SYS_gettid));
                             */
    printf("----------------------------------------------------\n");
    printf("openPOWERLINK console CN DEMO application\n");
    printf("----------------------------------------------------\n");

    EPL_MEMSET(&EplApiInitParam, 0, sizeof (EplApiInitParam));
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);

#ifdef CONFIG_POWERLINK_USERSTACK

    /* Retrieve the device list on the local machine */
    if (pcap_findalldevs(&alldevs, sErr_Msg) == -1)
    {
        fprintf(stderr, "Error in pcap_findalldevs: %s\n", sErr_Msg);
        EplRet = kEplNoResource;
        goto Exit;
    }

    PRINTF("--------------------------------------------------\n");
    PRINTF("List of Ethernet Cards Found in this System: \n");
    PRINTF("--------------------------------------------------\n");

    /* Print the list */
    for (seldev = alldevs; seldev != NULL; seldev = seldev->next)
    {
        PRINTF("%d. ", ++i);

        if (seldev->description)
        {
            PRINTF("%s\n      %s\n", seldev->description, seldev->name);
        }
        else
        {
            PRINTF("%s\n", seldev->name);
        }
    }

    if (i == 0)
    {
        PRINTF("\nNo interfaces found! Make sure WinPcap is installed.\n");
        EplRet = kEplNoResource;
        goto Exit;
    }

    PRINTF("--------------------------------------------------\n");
    PRINTF("Select the interface to be used for POWERLINK (1-%d):",i);
    if (scanf("%d", &inum) == EOF)
    {
        pcap_freealldevs(alldevs);
        EplRet = kEplNoResource;
        goto Exit;
    }
    PRINTF("--------------------------------------------------\n");
    if ((inum < 1) || (inum > i))
    {
        PRINTF("\nInterface number out of range.\n");
        /* Free the device list */
        pcap_freealldevs(alldevs);
        EplRet = kEplNoResource;
        goto Exit;
    }

    /* Jump to the selected adapter */
    for (seldev = alldevs, i = 0;
         i < (inum - 1);
         seldev = seldev->next, i++)
    {   // do nothing
    }

    strncpy(devName, seldev->name, 127);
    // pass selected device name to Edrv
    EplApiInitParam.m_HwParam.m_pszDevName = devName;

#endif

    EplApiInitParam.m_uiNodeId = uiNodeId_g;
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_dwFeatureFlags            = -1;
    EplApiInitParam.m_dwCycleLen                = uiCycleLen_g;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload      = 36;               // const
    EplApiInitParam.m_uiIsochrRxMaxPayload      = 36;               // const
    EplApiInitParam.m_dwPresMaxLatency          = 50000;            // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit     = 36;               // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit     = 36;               // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency          = 150000;           // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt         = 0;                // required for error detection
    EplApiInitParam.m_uiAsyncMtu                = 1500;             // required to set up max frame size
    EplApiInitParam.m_uiPrescaler               = 2;                // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance    = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout        = 3000000;
    EplApiInitParam.m_dwWaitSocPreq             = 150000;
    EplApiInitParam.m_dwDeviceType              = -1;               // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId                = -1;               // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode             = -1;               // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber          = -1;               // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber            = -1;               // NMT_IdentityObject_REC.SerialNo_U32
    EplApiInitParam.m_dwApplicationSwDate       = 0;
    EplApiInitParam.m_dwApplicationSwTime       = 0;

    EplApiInitParam.m_dwSubnetMask              = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway          = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));


    EplApiInitParam.m_uiSyncNodeId              = EPL_C_ADR_SYNC_ON_SOA;
    EplApiInitParam.m_fSyncOnPrcNode            = FALSE;

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = AppCbEvent;

#ifdef CONFIG_POWERLINK_USERSTACK
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
#else
    EplApiInitParam.m_pfnCbSync = NULL;
#endif


    printf("\n\nHello, I'm a POWERLINK node running as %s!\n  (build: %s / %s)\n\n",
            (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID ?
                "Managing Node" : "Controlled Node"),
            __DATE__, __TIME__);

    // initialize POWERLINK stack
    printf ("Initializing openPOWERLINK stack...\n");
    EplRet = EplApiInitialize(&EplApiInitParam);
    if(EplRet != kEplSuccessful)
    {
        printf("EplApiInitialize() failed (Error:0x%x!\n", EplRet);
        goto Exit;
    }

    // initialize application
    printf ("Initializing openPOWERLINK application...\n");

#ifdef CONFIG_POWERLINK_USERSTACK
    /* At this point, we don't need any more the device list. Free it */
    pcap_freealldevs(alldevs);

#else
    // create event thread
    if (pthread_create(&eventThreadId, NULL,
                   &powerlinkEventThread, NULL) != 0)
    {
        goto Exit;
    }

    // create sync thread
    if (pthread_create(&syncThreadId, NULL,
                   &powerlinkSyncThread, NULL) != 0)
    {
        goto Exit;
    }
#endif

    /**********************************************************/
    /* Allocate process image */
    printf("Initializing process image...\n");
    printf("Size of input process image: %ld\n", sizeof(AppProcessImageIn_g));
    printf("Size of output process image: %ld\n", sizeof (AppProcessImageOut_g));
    AppProcessImageCopyJob_g.m_fNonBlocking = FALSE;
    AppProcessImageCopyJob_g.m_uiPriority = 0;
    AppProcessImageCopyJob_g.m_In.m_pPart = &AppProcessImageIn_g;
    AppProcessImageCopyJob_g.m_In.m_uiOffset = 0;
    AppProcessImageCopyJob_g.m_In.m_uiSize = sizeof (AppProcessImageIn_g);
    AppProcessImageCopyJob_g.m_Out.m_pPart = &AppProcessImageOut_g;
    AppProcessImageCopyJob_g.m_Out.m_uiOffset = 0;
    AppProcessImageCopyJob_g.m_Out.m_uiSize = sizeof (AppProcessImageOut_g);

    EplRet = EplApiProcessImageAlloc(sizeof (AppProcessImageIn_g), sizeof (AppProcessImageOut_g), 2, 2);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    /**********************************************************/
    /* link process variables used by CN to object dictionary */
    printf("linking process image vars:\n");

    ObdSize = sizeof(AppProcessImageIn_g.digitalIn);
    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x6000, 0x01,
             offsetof(PI_IN, digitalIn), FALSE, ObdSize, &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars ... error %04x\n\n", EplRet);
        goto ExitShutdown;
    }

    ObdSize = sizeof(AppProcessImageOut_g.digitalOut);
    uiVarEntries = 1;
    EplRet = EplApiProcessImageLinkObject(0x6200, 0x01,
             offsetof(PI_OUT, digitalOut), TRUE, ObdSize, &uiVarEntries);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars ... error %04x\n\n", EplRet);
        goto ExitShutdown;
    }

    printf("linking process vars... ok\n\n");

    // start processing
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
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

    digitalIn_g = 1;

    // wait for key hit
    while (cKey != 0x1B)
    {
        if( EplTgtKbhit() )
        {
            cKey    = (BYTE) EplTgtGetch();

            switch (cKey)
            {
                case 'r':
                {
                    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
                    if (EplRet != kEplSuccessful)
                    {
                        goto ExitShutdown;
                    }
                    break;
                }
                case 'i':
                    if (digitalIn_g == 128)
                        digitalIn_g = 1;
                    else
                        digitalIn_g = digitalIn_g << 1;
                    printf ("\b \b");
                    printInputs();
                    break;

                case 'd':
                    if (digitalIn_g == 1)
                        digitalIn_g = 128;
                    else
                        digitalIn_g = digitalIn_g >> 1;
                    printf ("\b \b");
                    printInputs();
                    break;

                case 'p':
                    printf ("\b \b");
                    printOutputs();
                    break;

                default:
                {
                    break;
                }
            }
        }

        EplTgtMilliSleep (200);
    }

ExitShutdown:
    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // delete instance for all modules
    EplRet = EplApiShutdown();

Exit:
    PRINTF("main(): returns 0x%X\n", EplRet);

#if (TARGET_SYSTEM == _WIN32_)
    PRINTF("Press Enter to quit!\n");
    EplTgtGetch();
#endif

    return EplRet;
}

//---------------------------------------------------------------------------
//
// Function:    AppCbEvent
//
// Description: event callback function called by EPL API layer within
//              user part (low priority).
//
// Parameters:  EventType_p     = event type
//              pEventArg_p     = pointer to union, which describes
//                                the event in detail
//              pUserArg_p      = user specific argument
//
// Returns:     tEplKernel      = error code,
//                                kEplSuccessful = no error
//                                kEplReject = reject further processing
//                                otherwise = post error event to API layer
//---------------------------------------------------------------------------
tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p)    //__attribute((unused))

{
//UNUSED_PARAMETER(pUserArg_p);

    tEplKernel          EplRet = kEplSuccessful;

    // check if NMT_GS_OFF is reached
    switch (EventType_p)
    {
        case kEplApiEventNmtStateChange:
        {
            switch (pEventArg_p->m_NmtStateChange.m_NewNmtState)
            {
                case kEplNmtGsOff:
                {   // NMT state machine was shut down,
                    // because of user signal (CTRL-C) or critical EPL stack error
                    // -> also shut down EplApiProcess() and main()
                    EplRet = kEplShutdown;

                    printlog("Event:kEplNmtGsOff originating event = 0x%X (%s)\n", pEventArg_p->m_NmtStateChange.m_NmtEvent,
                             EplGetNmtEventStr(pEventArg_p->m_NmtStateChange.m_NmtEvent));
                    break;
                }

                case kEplNmtGsInitialising:
                case kEplNmtGsResetApplication:
                case kEplNmtGsResetConfiguration:
                case kEplNmtCsPreOperational1:
                case kEplNmtCsBasicEthernet:
                case kEplNmtMsBasicEthernet:
                case kEplNmtGsResetCommunication:
                    PRINTF("%s(0x%X) originating event = 0x%X\n",
                            __func__,
                            pEventArg_p->m_NmtStateChange.m_NewNmtState,
                            pEventArg_p->m_NmtStateChange.m_NmtEvent);
                    break;

                case kEplNmtMsNotActive:
                case kEplNmtCsNotActive:
                case kEplNmtCsOperational:
                    break;

                default:
                {
                    break;
                }
            }

            break;
        }

        case kEplApiEventCriticalError:
        case kEplApiEventWarning:
        {   // error or warning occurred within the stack or the application
            // on error the API layer stops the NMT state machine

            printlog( "%s(Err/Warn): Source = %s (%02X) EplError = %s (0x%03X)\n",
                        __func__,
                        EplGetEventSourceStr(pEventArg_p->m_InternalError.m_EventSource),
                        pEventArg_p->m_InternalError.m_EventSource,
                        EplGetEplKernelStr(pEventArg_p->m_InternalError.m_EplError),
                        pEventArg_p->m_InternalError.m_EplError
                        );
            FTRACE_MARKER("%s(Err/Warn): Source = %s (%02X) EplError = %s (0x%03X)\n",
                        __func__,
                        EplGetEventSourceStr(pEventArg_p->m_InternalError.m_EventSource),
                        pEventArg_p->m_InternalError.m_EventSource,
                        EplGetEplKernelStr(pEventArg_p->m_InternalError.m_EplError),
                        pEventArg_p->m_InternalError.m_EplError
                        );
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {   // error occurred within event processing
                    // either in kernel or in user part
                    printlog(" OrgSource = %s %02X\n",  EplGetEventSourceStr(pEventArg_p->m_InternalError.m_Arg.m_EventSource),
                                                        pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    FTRACE_MARKER(" OrgSource = %s %02X\n",     EplGetEventSourceStr(pEventArg_p->m_InternalError.m_Arg.m_EventSource),
                                                                pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occurred within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    printlog(" val = %X\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    FTRACE_MARKER(" val = %X\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    break;
                }

                default:
                {
                    printlog("\n");
                    break;
                }
            }
            break;
        }

        default:
            break;
    }

    return EplRet;
}

//---------------------------------------------------------------------------
//
// Function:    AppCbSync
//
// Description: sync event callback function called by event module within
//              kernel part (high priority).
//              This function sets the outputs, reads the inputs and runs
//              the control loop.
//
// Parameters:  void
//
// Returns:     tEplKernel      = error code,
//                                kEplSuccessful = no error
//                                otherwise = post error event to API layer
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC AppCbSync(void)
{
    tEplKernel      EplRet = kEplSuccessful;

    EplRet = EplApiProcessImageExchange(&AppProcessImageCopyJob_g);
    if (EplRet != kEplSuccessful)
    {
        return EplRet;
    }

    /* read input image - digital outputs */
    digitalOut_g = AppProcessImageOut_g.digitalOut;

    /* setup output image - digital inputs */
    AppProcessImageIn_g.digitalIn = digitalIn_g;

    return EplRet;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    printOutputs
//
// Description: print output signals on console
//
// Parameters:  void
//
// Returns:     void
//---------------------------------------------------------------------------
static void printOutputs(void)
{
    int i;

    printf ("Digital Outputs: ");
    for (i = 0; i < 8; i++)
    {
        if (((digitalOut_g >> i) & 1) == 1)
            printf ("*");
        else
            printf ("-");
    }
    printf ("\n");
}

//---------------------------------------------------------------------------
//
// Function:    printInputs
//
// Description: print input signals
//
// Parameters:  void
//
// Returns:     void
//---------------------------------------------------------------------------
static void  printInputs(void)
{
    int i;

    printf ("Digital Inputs: ");
    for (i = 0; i < 8; i++)
    {
        if (((digitalIn_g >> i) & 1) == 1)
            printf ("*");
        else
            printf ("-");
    }
    printf ("\n");
}


#ifndef CONFIG_POWERLINK_USERSTACK
//---------------------------------------------------------------------------
//
// Function:    powerlinkEventThread
//
// Description: main function of event thread.
//
// Parameters:  arg             = thread parameters (unused!)
//
// Returns:     NULL, always
//---------------------------------------------------------------------------
static void *powerlinkEventThread(void * arg __attribute__((unused)))
{
    EplApiProcess();

    return NULL;
}

//---------------------------------------------------------------------------
//
// Function:    powerlinkSyncThread
//
// Description: main function of sync thread.
//
// Parameters:  arg             = thread parameters (unused!)
//
// Returns:     NULL, always
//---------------------------------------------------------------------------
static void *powerlinkSyncThread(void * arg __attribute__((unused)))
{
    while (1)
    {
        AppCbSync();
    }
    return NULL;
}

#endif // #ifndef CONFIG_POWERLINK_USERSTACK

// EOF

