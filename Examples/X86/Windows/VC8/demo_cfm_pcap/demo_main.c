/****************************************************************************

  (c) Kalycito Infotech Private Limited
  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK - Demo main written originally by Kalycito

  Description:


  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Kalycito Infotech Private Limited nor the names of
       its contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@kalycito.com.

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

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#include "Epl.h"
#include <conio.h>
#include "pcap.h"
#include "xap.h"


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
#define IP_ADDR     0xc0a86401  // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0
#define HOSTNAME    "EPL Stack"



//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static DWORD uiNodeId_g = EPL_C_ADR_INVALID;


static PI_IN AppProcessImageIn_g;
static PI_OUT AppProcessImageOut_g;
static tEplApiProcessImageCopyJob AppProcessImageCopyJob_g;

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



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------

int main (void)
{
tEplKernel          EplRet;
static tEplApiInitParam EplApiInitParam = {0};
char*               sHostname = HOSTNAME;

char                cKey = 0;
// variables for Pcap
char sErr_Msg[ PCAP_ERRBUF_SIZE ];
pcap_if_t *alldevs;
pcap_if_t *seldev;
int i = 0;
int inum;


    // activate realtime priority class
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    // lower the priority of this thread
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

////////////////////////////////////////////////////////////////////////////////
						// PCAP: List ETH CARDS //
////////////////////////////////////////////////////////////////////////////////

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
    PRINTF1("Enter the interface number (1-%d):",i);
    scanf_s("%d", &inum);
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
////////////////////////////////////////////////////////////////////////////////
	// Setup EplApiInitParam (some of them can be removed as we have obd?) //
////////////////////////////////////////////////////////////////////////////////

    // pass selected device name to Edrv
    EplApiInitParam.m_HwParam.m_pszDevName = seldev->name;

    // get node ID from insmod command line
    EplApiInitParam.m_uiNodeId = uiNodeId_g;

    if (EplApiInitParam.m_uiNodeId == EPL_C_ADR_INVALID)
    {   // invalid node ID set
        // set default node ID
        EplApiInitParam.m_uiNodeId = NODEID;
    }

    uiNodeId_g = EplApiInitParam.m_uiNodeId;

    // calculate IP address
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));
    EplApiInitParam.m_dwFeatureFlags = ~0UL;
    EplApiInitParam.m_dwCycleLen = 10000;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 100; // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 100; // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;  // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36; // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36; // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;   // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;  // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;         // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;         // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 900000000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 10000000;
    EplApiInitParam.m_dwWaitSocPreq = 0;
    EplApiInitParam.m_dwDeviceType = ~0UL;              // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = ~0UL;                // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = ~0UL;             // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = ~0UL;          // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = ~0UL;            // NMT_IdentityObject_REC.SerialNo_U32
    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    EplApiInitParam.m_uiSyncNodeId = EPL_C_ADR_SYNC_ON_SOA;
    EplApiInitParam.m_fSyncOnPrcNode = FALSE;

    // currently unset parameters left at default value 0
    //EplApiInitParam.m_qwVendorSpecificExt1;
    //EplApiInitParam.m_dwVerifyConfigurationDate; // CFM_VerifyConfiguration_REC.ConfDate_U32
    //EplApiInitParam.m_dwVerifyConfigurationTime; // CFM_VerifyConfiguration_REC.ConfTime_U32
    //EplApiInitParam.m_dwApplicationSwDate;       // PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    //EplApiInitParam.m_dwApplicationSwTime;       // PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    //EplApiInitParam.m_abVendorSpecificExt2[48];

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = AppCbEvent;
    EplApiInitParam.m_pfnCbSync = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
				// Initialize Powerlink Stack //
////////////////////////////////////////////////////////////////////////////////
    PRINTF3("\n\n Powerlink %s running.\n  (build: %s / %s)\n\n",
            (NODEID == EPL_C_ADR_MN_DEF_NODE_ID ?
                "Managing Node" : "Controlled Node"),
            __DATE__, __TIME__);

    // initialize POWERLINK stack
    EplRet = EplApiInitialize(&EplApiInitParam);

    /* At this point, we don't need any more the device list. Free it */
    pcap_freealldevs(alldevs);

    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }

	PRINTF1("PI_IN Size %d\n", sizeof (AppProcessImageIn_g));
	PRINTF1("PI_OUT Size %d\n", sizeof (AppProcessImageOut_g));

    AppProcessImageCopyJob_g.m_fNonBlocking = FALSE;
    AppProcessImageCopyJob_g.m_uiPriority = 0;
    AppProcessImageCopyJob_g.m_In.m_pPart = &AppProcessImageIn_g;
    AppProcessImageCopyJob_g.m_In.m_uiOffset = 0;
    AppProcessImageCopyJob_g.m_In.m_uiSize = sizeof (AppProcessImageIn_g);
    AppProcessImageCopyJob_g.m_Out.m_pPart = &AppProcessImageOut_g;
    AppProcessImageCopyJob_g.m_Out.m_uiOffset = 0;
    AppProcessImageCopyJob_g.m_Out.m_uiSize = sizeof (AppProcessImageOut_g);

    EplRet = EplApiProcessImageAlloc(sizeof (AppProcessImageIn_g), sizeof (AppProcessImageOut_g), 2, 2);
	PRINTF1("PI Alloc returns %x\n", EplRet);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    EplRet = EplApiProcessImageSetup();
	PRINTF1("PI Set up returns %x\n", EplRet);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    // start processing
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    PRINTF0("Press Esc to leave the programm\n");
    // wait for key hit
    while (cKey != 0x1B)
    {
        if (_kbhit())
        {
            cKey = (BYTE)_getch() ;
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
        Sleep(1500);

    }
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
				// Stop the stack //
////////////////////////////////////////////////////////////////////////////////
ExitShutdown:
    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // delete process image
    EplRet = EplApiProcessImageFree();

    // delete instance for all modules
    EplRet = EplApiShutdown();
    PRINTF1("EplApiShutdown():  0x%X\n", EplRet);
////////////////////////////////////////////////////////////////////////////////

Exit:
    PRINTF1("main(): returns 0x%X\n", EplRet);
    PRINTF0("Press Enter to quit!\n");
    _getch();
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

                    PRINTF2("%s(kEplNmtGsOff) originating event = 0x%X\n", __func__, pEventArg_p->m_NmtStateChange.m_NmtEvent);

                    break;
                }

                case kEplNmtGsResetCommunication:
                {
					// continue
                }

                case kEplNmtGsResetConfiguration:
                {
                    // continue
                }

                case kEplNmtMsPreOperational1:
                {
                    PRINTF3("%s(0x%X) originating event = 0x%X\n",
                            __func__,
                           pEventArg_p->m_NmtStateChange.m_NewNmtState,
                           pEventArg_p->m_NmtStateChange.m_NmtEvent);

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

            PRINTF3("%s(Err/Warn): Source=%02X EplError=0x%03X",
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
                    PRINTF1(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occurred within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    PRINTF1(" val=%lX\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    break;
                }

                case kEplEventSourceObdk:
                case kEplEventSourceObdu:
                {   // error occurred within OBD module
                    // either in kernel or in user part
                    PRINTF2(" Object=0x%04X/%u\n", pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiIndex, pEventArg_p->m_InternalError.m_Arg.m_ObdError.m_uiSubIndex);
                    break;
                }

                default:
                {
                    PRINTF0("\n");
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
                    PRINTF2("%s(Node=0x%X, CheckConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventUpdateConf:
                {
                    PRINTF2("%s(Node=0x%X, UpdateConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeEventNmtState:
                {
                    PRINTF3("%s(Node=0x%X, NmtState=0x%X)\n", __func__, pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_NmtState);
                    break;
                }

                case kEplNmtNodeEventError:
                {
                    PRINTF3("%s(Node=0x%X, Error=0x%X)\n", __func__, pEventArg_p->m_Node.m_uiNodeId, pEventArg_p->m_Node.m_wErrorCode);
                    break;
                }

                case kEplNmtNodeEventFound:
                {
                    PRINTF2("%s(Node=0x%X, Found)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);
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
            PRINTF4("%s(Node=0x%X, CFM-Progress: Object 0x%X/%u, ", __func__, pEventArg_p->m_CfmProgress.m_uiNodeId, pEventArg_p->m_CfmProgress.m_uiObjectIndex, pEventArg_p->m_CfmProgress.m_uiObjectSubIndex);
            PRINTF2("%u/%u Bytes", pEventArg_p->m_CfmProgress.m_dwBytesDownloaded, pEventArg_p->m_CfmProgress.m_dwTotalNumberOfBytes);
            if ((pEventArg_p->m_CfmProgress.m_dwSdoAbortCode != 0)
                || (pEventArg_p->m_CfmProgress.m_EplError != kEplSuccessful))
            {
                PRINTF2(" -> SDO Abort=0x%lX, Error=0x%X)\n", pEventArg_p->m_CfmProgress.m_dwSdoAbortCode, pEventArg_p->m_CfmProgress.m_EplError);
            }
            else
            {
                PRINTF0(")\n");
            }
            break;
        }

        case kEplApiEventCfmResult:
        {
            switch (pEventArg_p->m_CfmResult.m_NodeCommand)
            {
                case kEplNmtNodeCommandConfOk:
                {
                    PRINTF2("%s(Node=0x%X, ConfOk)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfErr:
                {
                    PRINTF2("%s(Node=0x%X, ConfErr)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfReset:
                {
                    PRINTF2("%s(Node=0x%X, ConfReset)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                case kEplNmtNodeCommandConfRestored:
                {
                    PRINTF2("%s(Node=0x%X, ConfRestored)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId);
                    break;
                }

                default:
                {
                    PRINTF3("%s(Node=0x%X, CfmResult=0x%X)\n", __func__, pEventArg_p->m_CfmResult.m_uiNodeId, pEventArg_p->m_CfmResult.m_NodeCommand);
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
tEplKernel          EplRet = kEplSuccessful;
static unsigned int uiDigitalModData = 1;

    EplRet = EplApiProcessImageExchange(&AppProcessImageCopyJob_g);
	
	/*
	uiDigitalModData++;
	if (uiDigitalModData % 20 == 0)
	{
		AppProcessImageIn_g.CN1_M02_X20DO9322_DigitalOutput01 = (uiDigitalModData % 100 == 0);
		AppProcessImageIn_g.CN2_M02_X20DO9322_DigitalOutput01 = (uiDigitalModData % 100 == 0);
		AppProcessImageIn_g.CN3_M04_X20DO9322_DigitalOutput01 = (uiDigitalModData % 100 == 0);
	}
    */
	
    return EplRet;
}


// EOF

