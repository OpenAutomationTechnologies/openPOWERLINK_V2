/*******************************************************************************
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  demo application for openPOWERLINK MN with CFM
                under VxWorks on X86

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

#include "Epl.h"
#include "hrtimerLib.h"

#include <taskLib.h>
#include <sysLib.h>


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


// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#define NODEID      0xF0 //=> MN
#define CYCLE_LEN   5000 // [us]
#define IP_ADDR     0xc0a86401  // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "


// LIGHT EFFECT
#define DEFAULT_MAX_CYCLE_COUNT 20  // 6 is very fast
#define APP_DEFAULT_MODE        0x01
#define APP_LED_COUNT           5       // number of LEDs in one row
#define APP_LED_MASK            ((1 << APP_LED_COUNT) - 1)
#define APP_DOUBLE_LED_MASK     ((1 << (APP_LED_COUNT * 2)) - 1)
#define APP_MODE_COUNT          5
#define APP_MODE_MASK           ((1 << APP_MODE_COUNT) - 1)


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) == 0)
static DWORD    dw_le_CycleLen_g;
#endif

static UINT uiNodeId_g = EPL_C_ADR_INVALID;
static char* pszCdcFilename_g = EPL_OBD_DEF_CONCISEDCF_FILENAME;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
static UINT uiCycleLen_g = 0;
#else
static UINT uiCycleLen_g = CYCLE_LEN;
#endif


/* process image stuff */
#include "xap.h"

static PI_IN AppProcessImageIn_g;
static PI_OUT AppProcessImageOut_g;
static tEplApiProcessImageCopyJob AppProcessImageCopyJob_g;

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

BOOL                        fOpenPowerlinkIsRunning_g;
/*----------------------------------------------------------------------------*/


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to
// define this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.

tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);

tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p);

tEplKernel PUBLIC AppCbSync(void);
tEplKernel PUBLIC AppInit(void);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    openPowerlinkInit
//
// Description:
//    Initialize and start the openPOWERLINK demo application
//
// Parameters:
//    pszEthName = pointer to string with name of network interface to use
//
// Returns: int
//
//---------------------------------------------------------------------------
int openPowerlinkInit (char *pszEthName, unsigned int uiDevNumber)
{
    tEplKernel          EplRet;
    static tEplApiInitParam EplApiInitParam = {0};
    char*               sHostname = HOSTNAME;
    BOOL                fApiInit = FALSE;
    int                 tid;

    // Adjust task priorities of system tasks

    /* shell task is normally set to priority 1 which could disturb
     * openPOWERLINK if long-running commands are entered. Therefore
     * we lower the performance to 20
     */
    tid = taskIdSelf();
    printf ("Task ID: %d\n", tid);
    taskPrioritySet(tid, 20);

    /* The priority of the network stack task has to be increased (default:50)
     * in order to get network packets in time!
     */
    if ((tid = taskNameToId("tNet0")) != -1 )
    {
        taskPrioritySet(tid, 5);
    }

    // Initialize high-resolution timer library
    hrtimer_init(EPL_TASK_PRIORITY_HRTIMER, EPL_TASK_STACK_SIZE);

    // get node ID from insmod command line
    EplApiInitParam.m_uiNodeId = uiNodeId_g;

    if (EplApiInitParam.m_uiNodeId == EPL_C_ADR_INVALID)
    {   // invalid node ID set
        // set default node ID
        EplApiInitParam.m_uiNodeId = NODEID;
    }

    uiNodeId_g = EplApiInitParam.m_uiNodeId;

    // calculate IP address
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) |
                                    EplApiInitParam.m_uiNodeId;

    EplApiInitParam.m_fAsyncOnly = FALSE;

    // store the specified device name
    EplApiInitParam.m_HwParam.m_pszDevName = pszEthName;
    EplApiInitParam.m_HwParam.m_uiDevNumber = uiDevNumber;

    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr,
               sizeof (EplApiInitParam.m_abMacAddress));
    EplApiInitParam.m_dwFeatureFlags = (DWORD) ~0UL;
    EplApiInitParam.m_dwCycleLen = uiCycleLen_g;  // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 100; // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 100; // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;   // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36; // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36; // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;  // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;      // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;          // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;            // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
    EplApiInitParam.m_dwWaitSocPreq = 0;
    EplApiInitParam.m_dwDeviceType = (DWORD) ~0UL;      // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = (DWORD) ~0UL;        // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = (DWORD) ~0UL;     // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = (DWORD) ~0UL;  // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = (DWORD) ~0UL;    // NMT_IdentityObject_REC.SerialNo_U32
    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    EplApiInitParam.m_uiSyncNodeId = EPL_C_ADR_SYNC_ON_SOA; // for fSyncOnPrcNode==TRUE, this means last PRC node
    EplApiInitParam.m_fSyncOnPrcNode = TRUE;

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = AppCbEvent;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;

    printf("\n\n Hello, I'm a VxWorks POWERLINK node running as %s!\n"
           "(build: %s / %s)\n\n",
            (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID ?
                "Managing Node" : "Controlled Node"),
            __DATE__, __TIME__);

    // initialize POWERLINK stack
    EplRet = EplApiInitialize(&EplApiInitParam);
    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }
    fApiInit = TRUE;

    // set CDC filename
    EplRet = EplApiSetCdcFilename(pszCdcFilename_g);
    if(EplRet != kEplSuccessful)
    {
    	printf ("Error set cdc filename!\n");
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

    // initialize process image
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

    EplRet = EplApiProcessImageAlloc(sizeof (AppProcessImageIn_g),
                                     sizeof (AppProcessImageOut_g), 2, 2);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    EplRet = EplApiProcessImageSetup();
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // start the NMT state machine
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);

Exit:
    PRINTF("%s(): returns 0x%X\n", __func__, EplRet);
    if (EplRet != kEplSuccessful)
    {
        if (fApiInit != FALSE)
        {
            EplApiShutdown();
        }

        // Shutdown high-resolution timer library
        hrtimer_shutdown();
        fOpenPowerlinkIsRunning_g = FALSE;
        return -ENODEV;
    }
    else
    {
        fOpenPowerlinkIsRunning_g = TRUE;
        return 0;
    }
}

//---------------------------------------------------------------------------
//
// Function:    openPowerlinkExit
//
// Description:
//    Shutdown and cleanup openPOWERLINK demo application
//
// Parameters:
//    none
//
// Returns:
//    void
//---------------------------------------------------------------------------
void openPowerlinkExit (void)
{
	tEplKernel          EplRet;

    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // wait until NMT state machine is shut down
    taskDelay (sysClkRateGet());
    fOpenPowerlinkIsRunning_g = FALSE;

    EplApiProcessImageFree();

    // delete instance for all modules
    EplRet = EplApiShutdown();
    PRINTF("EplApiShutdown():  0x%X\n", EplRet);

    hrtimer_shutdown();

}

//---------------------------------------------------------------------------
//
// Function:    openPowerlinkSendNmt
//
// Description:
//    Send a NMT command to the stack
//
// Parameters:
//    nmtCmd_p = NMT command to send
//
// Returns:
//    void
//---------------------------------------------------------------------------
void openPowerlinkSendNmt(tEplNmtEvent nmtCmd_p)
{
    tEplKernel          EplRet;

    EplRet = EplApiExecNmtCommand(nmtCmd_p);
    if (EplRet != kEplSuccessful)
    {
        PRINTF("Error sending NMT command %d\n", nmtCmd_p);
    }
    else
    {
        PRINTF("NMT command %d successfully sent\n", nmtCmd_p);
    }
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
//---------------------------------------------------------------------------

tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p)
{
tEplKernel          EplRet = kEplSuccessful;

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

                    PRINTF("%s:kEplNmtGsOff originating event = 0x%X (%s)\n",
                            __func__, pEventArg_p->m_NmtStateChange.m_NmtEvent,
                            EplGetNmtEventStr(pEventArg_p->m_NmtStateChange.m_NmtEvent));
                    // wake up EplLinExit()
                    /*atomic_set(&AtomicShutdown_g, TRUE);
                    wake_up_interruptible(&WaitQueueShutdown_g);*/
                    break;
                }

                case kEplNmtGsResetCommunication:
                {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) == 0)
                DWORD   dwNodeAssignment;

                    // configure OD for MN in state ResetComm after reseting the OD
                    // TODO: setup your own network configuration here
                    dwNodeAssignment = (EPL_NODEASSIGN_NODE_IS_CN |
                                        EPL_NODEASSIGN_NODE_EXISTS); // 0x00000003L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x01, &dwNodeAssignment,
                                                    sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x20, &dwNodeAssignment,
                                                    sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwNodeAssignment,
                                                    sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xFE, &dwNodeAssignment,
                                                    sizeof (dwNodeAssignment));

                    // dwNodeAssignment |= EPL_NODEASSIGN_MANDATORY_CN;    // 0x0000000BL
                    // EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    dwNodeAssignment = (EPL_NODEASSIGN_MN_PRES |
                                    EPL_NODEASSIGN_NODE_EXISTS);       // 0x00010001L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xF0, &dwNodeAssignment,
                                                    sizeof (dwNodeAssignment));
#endif
                }

                case kEplNmtGsResetConfiguration:
                {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
                    if (uiCycleLen_g != 0)
                    {
                        EplRet = EplApiWriteLocalObject(0x1006, 0x00,
                                                        &uiCycleLen_g,
                                                        sizeof (uiCycleLen_g));
                    }
#else
                    unsigned int uiSize;

                    // fetch object 0x1006 NMT_CycleLen_U32 from local OD
                    // (in little endian byte order) for configuration of
                    // remote CN
                    uiSize = 4;
                    EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00,
                                              &dw_le_CycleLen_g, &uiSize,
                                              kEplSdoTypeAsnd, NULL);
                    if (EplRet != kEplSuccessful)
                    {   // local OD access failed
                        break;
                    }
#endif
                }

                case kEplNmtMsPreOperational1:
                {
                    PRINTF("%s(0x%X) originating event = 0x%X (%s)\n",
                            __func__,
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
        {   // error or warning occurred within the stack or the application
            // on error the API layer stops the NMT state machine

            PRINTF("%s(Err/Warn): Source=%02X EplError=0x%03X",
                    __func__,
                    pEventArg_p->m_InternalError.m_EventSource,
                    pEventArg_p->m_InternalError.m_EplError);
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {   // error occurred within event processing
                    // either in kernel or in user part
                    PRINTF(" OrgSource=%02X\n",
                            pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occurred within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    PRINTF(" val=%lX\n",
                            (ULONG) pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    break;
                }

                case kEplEventSourceObdk:
                case kEplEventSourceObdu:
                {   // error occurred within OBD module
                    // either in kernel or in user part
                    PRINTF(" Object=0x%04X/%u\n",
                            pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiIndex,
                            pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiSubIndex);
                    break;
                }

                default:
                {
                    PRINTF("\n");
                    break;
                }
            }
            break;
        }

        case kEplApiEventHistoryEntry:
        {   // new history entry
            PRINTF("%s(HistoryEntry): Type=0x%04X Code=0x%04X (0x%02X %02X %02X %02X %02X %02X %02X %02X)\n",
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

        case kEplApiEventNode:
        {
            // check additional argument
            switch (pEventArg_p->m_Node.m_NodeEvent)
            {
                case kEplNmtNodeEventCheckConf:
                {
                    PRINTF("%s(Node=0x%X, CheckConf)\n", __func__,
                            pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventUpdateConf:
                {
                    PRINTF("%s(Node=0x%X, UpdateConf)\n", __func__,
                            pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventNmtState:
                {
                    PRINTF("%s(Node=0x%X, NmtState=0x%X)\n", __func__,
                            pEventArg_p->m_Node.m_uiNodeId,
                            pEventArg_p->m_Node.m_NmtState);
                    break;
                }

                case kEplNmtNodeEventError:
                {
                    PRINTF("%s(Node=0x%X, Error=0x%X)\n", __func__,
                            pEventArg_p->m_Node.m_uiNodeId,
                            pEventArg_p->m_Node.m_wErrorCode);
                    break;
                }

                case kEplNmtNodeEventFound:
                {
                    PRINTF("%s(Node=0x%X, Found)\n", __func__,
                            pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        case kEplApiEventCfmProgress:
        {
            PRINTF("%s(Node=0x%X, CFM-Progress: Object 0x%X/%u, ",
                    __func__, pEventArg_p->m_CfmProgress.m_uiNodeId,
                    pEventArg_p->m_CfmProgress.m_uiObjectIndex,
                    pEventArg_p->m_CfmProgress.m_uiObjectSubIndex);
            PRINTF("%lu/%lu Bytes",
                    (ULONG) pEventArg_p->m_CfmProgress.m_dwBytesDownloaded,
                    (ULONG) pEventArg_p->m_CfmProgress.m_dwTotalNumberOfBytes);
            if ((pEventArg_p->m_CfmProgress.m_dwSdoAbortCode != 0)
                || (pEventArg_p->m_CfmProgress.m_EplError != kEplSuccessful))
            {
                PRINTF(" -> SDO Abort=0x%lX, Error=0x%X)\n",
                        (unsigned long) pEventArg_p->m_CfmProgress.m_dwSdoAbortCode,
                        pEventArg_p->m_CfmProgress.m_EplError);
            }
            else
            {
                PRINTF(")\n");
            }
            break;
        }

        case kEplApiEventCfmResult:
        {
            switch (pEventArg_p->m_CfmResult.m_NodeCommand)
            {
                case kEplNmtNodeCommandConfOk:
                {
                    PRINTF("%s(Node=0x%X, ConfOk)\n", __func__,
                            pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfErr:
                {
                    PRINTF("%s(Node=0x%X, ConfErr)\n", __func__,
                            pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfReset:
                {
                    PRINTF("%s(Node=0x%X, ConfReset)\n", __func__,
                            pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfRestored:
                {
                    PRINTF("%s(Node=0x%X, ConfRestored)\n", __func__,
                            pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                default:
                {
                    PRINTF("%s(Node=0x%X, CfmResult=0x%X)\n", __func__,
                            pEventArg_p->m_CfmResult.m_uiNodeId,
                            pEventArg_p->m_CfmResult.m_NodeCommand);
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


