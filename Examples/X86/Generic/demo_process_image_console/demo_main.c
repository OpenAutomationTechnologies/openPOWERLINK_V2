/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  openPOWERLINK process image console demo application

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
/* includes */
#if defined __linux__

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
#endif

#elif defined WIN32

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h

#endif  // WIN32

#include "Epl.h"
#include <pcap.h>
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
#if defined __linux__

#define SET_CPU_AFFINITY
#define MAIN_THREAD_PRIORITY            20

#elif defined WIN32

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#endif // WIN32

#define NODEID      0xF0                //=> MN
#define IP_ADDR     0xc0a86401          // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00          // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "
//#define IF_ETH      EPL_VETH_NAME


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static unsigned int uiNodeId_g = EPL_C_ADR_INVALID;
static unsigned int uiCycleLen_g = 0;
static unsigned int uiCurCycleLen_g = 0;
static char *pLogFile_g = NULL;

/* process image stuff */
#include "xap.h"



static PI_IN AppProcessImageIn_g;
static PI_OUT AppProcessImageOut_g;
static tEplApiProcessImageCopyJob AppProcessImageCopyJob_g;

#ifdef CONFIG_POWERLINK_USERSTACK

static char* pszCdcFilename_g = "mnobd.cdc";

#else

static pthread_t eventThreadId;
static pthread_t syncThreadId;

void *powerlinkEventThread(void * arg);
void *powerlinkSyncThread(void * arg);

#endif

/*----------------------------------------------------------------------------*/
/* application defines and variables */
#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))
#define MAX_NODES               255

typedef struct
{
    unsigned int            m_uiLeds;
    unsigned int            m_uiLedsOld;
    unsigned int            m_uiInput;
    unsigned int            m_uiInputOld;
    unsigned int            m_uiPeriod;
    int                     m_iToggle;
} APP_NODE_VAR_T;

static int                  iUsedNodeIds_g[] = {1, 32, 110, 0};
static unsigned int         uiCnt_g;
static APP_NODE_VAR_T       nodeVar_g[MAX_NODES];
/*----------------------------------------------------------------------------*/

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
tEplKernel PUBLIC AppInit(void);

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
	struct tm           *p_timeVal;
    char                timeStr[20];

    time(&timeStamp);

#if defined __linux__
    localtime_r(&timeStamp, &timeVal);
	strftime(timeStr, 20, "%Y/%m/%d %H:%M:%S", &timeVal);
#else
	p_timeVal = localtime(&timeStamp);
	strftime(timeStr, 20, "%Y/%m/%d %H:%M:%S", p_timeVal);
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

#ifdef CONFIG_POWERLINK_USERSTACK
#ifdef __linux__
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

    int                         opt;

#ifdef __linux__
    /* get command line parameters */
    while ((opt = getopt(argc, argv, "c:l:")) != -1)
    {
        switch (opt)
        {
        case 'c':
            uiCycleLen_g = strtoul(optarg, NULL, 10);
            break;

        case 'l':
            pLogFile_g = optarg;
            break;

        default: /* '?' */
            fprintf (stderr, "Usage: %s [-c CYCLE_TIME] [-l LOGFILE]\n", argv[0]);
            goto Exit;
        }
    }
#endif

#ifdef CONFIG_POWERLINK_USERSTACK

#if defined __linux__
	/* adjust process priority */
    if (nice (-20) == -1)         // push nice level in case we have no RTPreempt
    {
        EPL_DBGLVL_ERROR_TRACE2("%s() couldn't set nice value! (%s)\n", __func__, strerror(errno));
    }
    schedParam.__sched_priority = MAIN_THREAD_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s() couldn't set thread scheduling parameters! %d\n",
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

#elif defined WIN32

	// activate realtime priority class
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    // lower the priority of this thread
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

#endif // WIN32
	
#endif // CONFIG_POWERLINK_USERSTACK

    /* Enabling ftrace for debugging */
    FTRACE_OPEN();
    FTRACE_ENABLE(TRUE);

    /*
    EPL_DBGLVL_ALWAYS_TRACE2("%s(): Main Thread Id:%ld\n", __func__,
                             syscall(SYS_gettid));
                             */
    printf("----------------------------------------------------\n");
    printf("openPOWERLINK console process image DEMO application\n");
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

	PRINTF0("--------------------------------------------------\n");
    PRINTF0("List of Ethernet Cards Found in this System: \n");
    PRINTF0("--------------------------------------------------\n");
	
	 /* Print the list */
    for (seldev = alldevs; seldev != NULL; seldev = seldev->next)
    {
        PRINTF1("%d. ", ++i);

        if (seldev->description)
        {
            PRINTF2("%s\n      %s\n", seldev->description, seldev->name);
        }
        else
        {
            PRINTF1("%s\n", seldev->name);
        }
    }
	
	if (i == 0)
    {
        PRINTF0("\nNo interfaces found! Make sure WinPcap is installed.\n");
        EplRet = kEplNoResource;
        goto Exit;
    }

	PRINTF0("--------------------------------------------------\n");
    PRINTF1("Select the interface to be used for POWERLINK (1-%d):",i);
	if (scanf("%d", &inum) == EOF)
    {
        pcap_freealldevs(alldevs);
        EplRet = kEplNoResource;
        goto Exit;
    }
    PRINTF0("--------------------------------------------------\n");
    if ((inum < 1) || (inum > i))
    {
        PRINTF0("\nInterface number out of range.\n");
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

    EplApiInitParam.m_uiNodeId = uiNodeId_g = NODEID;
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_dwFeatureFlags = -1;
    EplApiInitParam.m_dwCycleLen = uiCycleLen_g;        // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 256;       // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 256;       // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;         // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36;       // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36;       // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;        // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;            // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;                // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;                  // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
    EplApiInitParam.m_dwWaitSocPreq = 150000;
    EplApiInitParam.m_dwDeviceType = -1;      // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = -1;        // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = -1;     // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = -1;  // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = -1;    // NMT_IdentityObject_REC.SerialNo_U32

    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    EplApiInitParam.m_uiSyncNodeId = EPL_C_ADR_SYNC_ON_SOA;
    EplApiInitParam.m_fSyncOnPrcNode = FALSE;

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = AppCbEvent;

#ifdef CONFIG_POWERLINK_USERSTACK
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
#else
    EplApiInitParam.m_pfnCbSync = NULL;
#endif


    printf("\n\nHello, I'm a Userspace POWERLINK node running as %s!\n  (build: %s / %s)\n\n",
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
    EplRet = AppInit();
    if(EplRet != kEplSuccessful)
    {
        printf("ApiInit() failed!\n");
        goto Exit;
    }


#ifdef CONFIG_POWERLINK_USERSTACK
    /* At this point, we don't need any more the device list. Free it */
    pcap_freealldevs(alldevs);

    EplRet = EplApiSetCdcFilename(pszCdcFilename_g);
    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }
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

    EplRet = EplApiProcessImageSetup();
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // start processing
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    printf("\n-------------------------------\n");
    printf("Press Esc to leave the programm\n");
    printf("Press r to reset the node\n");
    printf("-------------------------------\n\n");
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

                case 'c':
                {
                    EplRet = EplApiExecNmtCommand(kEplNmtEventNmtCycleError);
                    if (EplRet != kEplSuccessful)
                    {
                        goto ExitShutdown;
                    }
                    break;
                }

                default:
                {
                    break;
                }
            }
        }

        EplTgtMilliSleep( 1500 );
    }
	
    FTRACE_ENABLE(FALSE);
	
ExitShutdown:
    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);
	 
	// delete process image
    EplRet = EplApiProcessImageFree();
	
    // delete instance for all modules
    EplRet = EplApiShutdown();

Exit:
    PRINTF1("main(): returns 0x%X\n", EplRet);

#ifdef WIN32
    PRINTF0("Press Enter to quit!\n");
    EplTgtGetch();
#endif

    return EplRet;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


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
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p)    //__attribute((unused))

{
    tEplKernel          EplRet = kEplSuccessful;
    UINT                uiVarLen;

	UNUSED_PARAMETER(pUserArg_p);

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

                case kEplNmtGsResetCommunication:
                {
                    // continue
                }

                case kEplNmtGsResetConfiguration:
                {
                    if (uiCycleLen_g != 0)
                    {
                        EplRet = EplApiWriteLocalObject(0x1006, 0x00, &uiCycleLen_g, sizeof (uiCycleLen_g));
                        uiCurCycleLen_g = uiCycleLen_g;
                    }
                    else
                    {
                        uiVarLen = sizeof(uiCurCycleLen_g);
                        EplApiReadLocalObject(0x1006, 0x00, &uiCurCycleLen_g, &uiVarLen);
                    }
                    // continue
                }

                case kEplNmtMsPreOperational1:
                {
                    printlog("AppCbEvent(0x%X) originating event = 0x%X (%s)\n",
                           pEventArg_p->m_NmtStateChange.m_NewNmtState,
                           pEventArg_p->m_NmtStateChange.m_NmtEvent,
                           EplGetNmtEventStr(pEventArg_p->m_NmtStateChange.m_NmtEvent));

                    // continue
                }

                case kEplNmtGsInitialising:
                case kEplNmtGsResetApplication:
                case kEplNmtMsNotActive:
                case kEplNmtCsNotActive:
                case kEplNmtCsPreOperational1:
                {
                    break;
                }

                case kEplNmtCsOperational:
                case kEplNmtMsOperational:
                {
                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }

        case kEplApiEventCriticalError:
        case kEplApiEventWarning:
        {   // error or warning occured within the stack or the application
            // on error the API layer stops the NMT state machine

            printlog("%s(Err/Warn): Source=%02X EplError=0x%03X",
                                __func__,
                                pEventArg_p->m_InternalError.m_EventSource,
                                pEventArg_p->m_InternalError.m_EplError);
            FTRACE_MARKER("%s(Err/Warn): Source=%02X EplError=0x%03X",
                                __func__,
                                pEventArg_p->m_InternalError.m_EventSource,
                                pEventArg_p->m_InternalError.m_EplError);
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {   // error occured within event processing
                    // either in kernel or in user part
                    printlog(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    FTRACE_MARKER(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occured within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    printlog(" val=%X\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    FTRACE_MARKER(" val=%X\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
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

        case kEplApiEventHistoryEntry:
        {   // new history entry

            printlog("%s(HistoryEntry): Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
                    __func__,
                    pEventArg_p->m_ErrHistoryEntry.m_wEntryType,
                    pEventArg_p->m_ErrHistoryEntry.m_wErrorCode,
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[0],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[1],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[2],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[3],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[4],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[5],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[6],
                    (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[7]);
            FTRACE_MARKER("%s(HistoryEntry): Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
                                __func__,
                                pEventArg_p->m_ErrHistoryEntry.m_wEntryType,
                                pEventArg_p->m_ErrHistoryEntry.m_wErrorCode,
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[0],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[1],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[2],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[3],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[4],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[5],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[6],
                                (WORD) pEventArg_p->m_ErrHistoryEntry.m_abAddInfo[7]);
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplApiEventNode:
        {
            // check additional argument
            switch (pEventArg_p->m_Node.m_NodeEvent)
            {
                case kEplNmtNodeEventCheckConf:
                {
                    printlog("%s(Node=0x%X, CheckConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventUpdateConf:
                {
                    printlog("%s(Node=0x%X, UpdateConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventNmtState:
                {
                    printlog("%s(Node=0x%X, NmtState=%s)\n", __func__, pEventArg_p->m_Node.m_uiNodeId, EplGetNmtStateStr(pEventArg_p->m_Node.m_NmtState));

                    break;
                }

                case kEplNmtNodeEventError:
                {
                    printlog("%s(Node=0x%X, Error=0x%X)\n", __func__, pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_wErrorCode);

                    break;
                }

                case kEplNmtNodeEventFound:
                {
                    printlog("%s(Node=0x%X, Found)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);

                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
       case kEplApiEventCfmProgress:
        {
            printlog("%s(Node=0x%X, CFM-Progress: Object 0x%X/%u, ", __func__, pEventArg_p->m_CfmProgress.m_uiNodeId, pEventArg_p->m_CfmProgress.m_uiObjectIndex, pEventArg_p->m_CfmProgress.m_uiObjectSubIndex);
            printlog("%lu/%lu Bytes", (ULONG) pEventArg_p->m_CfmProgress.m_dwBytesDownloaded, (ULONG) pEventArg_p->m_CfmProgress.m_dwTotalNumberOfBytes);
            if ((pEventArg_p->m_CfmProgress.m_dwSdoAbortCode != 0)
                || (pEventArg_p->m_CfmProgress.m_EplError != kEplSuccessful))
            {
                printlog(" -> SDO Abort=0x%lX, Error=0x%X)\n", (unsigned long) pEventArg_p->m_CfmProgress.m_dwSdoAbortCode,
                                                              pEventArg_p->m_CfmProgress.m_EplError);
            }
            else
            {
                printlog(")\n");
            }
            break;
        }

        case kEplApiEventCfmResult:
        {
            switch (pEventArg_p->m_CfmResult.m_NodeCommand)
            {
                case kEplNmtNodeCommandConfOk:
                {
                    printlog("%s(Node=0x%X, ConfOk)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfErr:
                {
                    printlog("%s(Node=0x%X, ConfErr)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfReset:
                {
                    printlog("%s(Node=0x%X, ConfReset)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfRestored:
                {
                    printlog("%s(Node=0x%X, ConfRestored)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                default:
                {
                    printlog("%s(Node=0x%X, CfmResult=0x%X)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId, pEventArg_p->m_CfmResult.m_NodeCommand);
                    break;
                }
            }
            break;
        }
#endif

        default:
            break;
    }

    return EplRet;
}

//---------------------------------------------------------------------------
//
// Function:    AppInit
//
// Description: initialize application
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
tEplKernel PUBLIC AppInit(void)
{
    tEplKernel EplRet = kEplSuccessful;
    int        i;

    uiCnt_g = 0;

    for (i = 0; (i < MAX_NODES) && (iUsedNodeIds_g[i] != 0); i++)
    {
        nodeVar_g[i].m_uiLeds = 0;
        nodeVar_g[i].m_uiLedsOld = 0;
        nodeVar_g[i].m_uiInput = 0;
        nodeVar_g[i].m_uiInputOld = 0;
        nodeVar_g[i].m_iToggle = 0;
        nodeVar_g[i].m_uiPeriod = 0;
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
    tEplKernel          EplRet;
    int                 i;

    EplRet = EplApiProcessImageExchange(&AppProcessImageCopyJob_g);
    if (EplRet != kEplSuccessful)
    {
        return EplRet;
    }

    uiCnt_g++;

    nodeVar_g[0].m_uiInput = AppProcessImageOut_g.CN1_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[1].m_uiInput = AppProcessImageOut_g.CN32_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[2].m_uiInput = AppProcessImageOut_g.CN110_M00_Digital_Input_8_Bit_Byte_1;

    for (i = 0; (i < MAX_NODES) && (iUsedNodeIds_g[i] != 0); i++)
    {
        /* Running Leds */
        /* period for LED flashing determined by inputs */
        nodeVar_g[i].m_uiPeriod = (nodeVar_g[i].m_uiInput == 0) ? 1 : (nodeVar_g[i].m_uiInput * 20);
        if (uiCnt_g % nodeVar_g[i].m_uiPeriod == 0)
        {
            if (nodeVar_g[i].m_uiLeds == 0x00)
            {
                nodeVar_g[i].m_uiLeds = 0x1;
                nodeVar_g[i].m_iToggle = 1;
            }
            else
            {
                if (nodeVar_g[i].m_iToggle)
                {
                    nodeVar_g[i].m_uiLeds <<= 1;
                    if (nodeVar_g[i].m_uiLeds == APP_LED_MASK_1)
                    {
                        nodeVar_g[i].m_iToggle = 0;
                    }
                }
                else
                {
                    nodeVar_g[i].m_uiLeds >>= 1;
                    if (nodeVar_g[i].m_uiLeds == 0x01)
                    {
                        nodeVar_g[i].m_iToggle = 1;
                    }
                }
            }
        }

        if (nodeVar_g[i].m_uiInput != nodeVar_g[i].m_uiInputOld)
        {
            nodeVar_g[i].m_uiInputOld = nodeVar_g[i].m_uiInput;
        }

        if (nodeVar_g[i].m_uiLeds != nodeVar_g[i].m_uiLedsOld)
        {
            nodeVar_g[i].m_uiLedsOld = nodeVar_g[i].m_uiLeds;
        }
    }

    AppProcessImageIn_g.CN1_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[0].m_uiLeds;
    AppProcessImageIn_g.CN32_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[1].m_uiLeds;
    AppProcessImageIn_g.CN110_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[2].m_uiLeds;

    return EplRet;
}

#ifndef CONFIG_POWERLINK_USERSTACK

void *powerlinkEventThread(void * arg __attribute__((unused)))
{
    EplApiProcess();

    return NULL;
}

void *powerlinkSyncThread(void * arg __attribute__((unused)))
{
    while (1)
    {
        AppCbSync();
    }
    return NULL;
}


#endif

// EOF

