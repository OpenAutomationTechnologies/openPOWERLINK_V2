/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  demoapplication for EPL MN (with SDO over ASnd)
                under Windows with WinPcap driver

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
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

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                MS Visual C 8.0 / 9.0

  -------------------------------------------------------------------------

  Revision History:

  2006/09/01 d.k.:   start of implementation

****************************************************************************/

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#include <conio.h>
#include "Epl.h"
#include "pcap.h"

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
#define HOSTNAME    "SYS TEC electronic EPL Stack    "
#define IF_ETH      EPL_VETH_NAME


// LIGHT EFFECT
#define DEFAULT_MAX_CYCLE_COUNT 20  // 6 is very fast
#define DEFAULT_MODE            0x01
#define LED_COUNT               5       // number of LEDs in one row
#define LED_MASK                ((1 << LED_COUNT) - 1)
#define DOUBLE_LED_MASK         ((1 << (LED_COUNT * 2)) - 1)
#define MODE_COUNT              4
#define MODE_MASK               ((1 << MODE_COUNT) - 1)


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

BYTE    bVarIn1_l;
BYTE    bVarOut1_l;
BYTE    bVarOut1Old_l;
BYTE    bModeSelect_l;      // state of the pushbuttons to select the mode
BYTE    bSpeedSelect_l;     // state of the pushbuttons to increase/decrease the speed
BYTE    bSpeedSelectOld_l;  // old state of the pushbuttons
DWORD   dwLeds_l;           // current state of all LEDs
BYTE    bLedsRow1_l;        // current state of the LEDs in row 1
BYTE    bLedsRow2_l;        // current state of the LEDs in row 2
BYTE    abSelect_l[3];      // pushbuttons from CNs

DWORD   dwMode_l;           // current mode
int     iCurCycleCount_l;   // current cycle count
int     iMaxCycleCount_l;   // maximum cycle count (i.e. number of cycles until next light movement step)
int     iToggle;            // indicates the light movement direction

BYTE    abDomain_l[3000];

static DWORD    dw_le_CycleLen_g;           // object 0x1006
static DWORD    dw_le_LossOfSocTolerance_g; // object 0x1C14

static DWORD uiNodeId_g = EPL_C_ADR_INVALID;

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
unsigned int        uiVarEntries;
tEplObdSize         ObdSize;
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
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;


    PRINTF3("\n\n Powerlink %s running.!\n  (build: %s / %s)\n\n",
            (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID ?
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

    // link process variables used by CN to object dictionary
    ObdSize = sizeof(bVarIn1_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6000, &bVarIn1_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    ObdSize = sizeof(bVarOut1_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6200, &bVarOut1_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    // link process variables used by MN to object dictionary
    ObdSize = sizeof(bLedsRow1_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bLedsRow1_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    ObdSize = sizeof(bLedsRow2_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bLedsRow2_l, &uiVarEntries, &ObdSize, 0x02);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    ObdSize = sizeof(abSelect_l[0]);
    uiVarEntries = sizeof(abSelect_l);
    EplRet = EplApiLinkObject(0x2200, &abSelect_l[0], &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    // link a DOMAIN to object 0x6100, but do not exit, if it is missing
    ObdSize = sizeof(abDomain_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6100, &abDomain_l, &uiVarEntries, &ObdSize, 0x00);
    if (EplRet != kEplSuccessful)
    {
        PRINTF1("EplApiLinkObject(0x6100): returns 0x%X\n", EplRet);
    }

    // reset old process variables
    bVarOut1Old_l = 0;
    bSpeedSelectOld_l = 0;
    dwMode_l = DEFAULT_MODE;
    iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;

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

ExitShutdown:
    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // delete instance for all modules
    EplRet = EplApiShutdown();
    PRINTF1("EplApiShutdown():  0x%X\n", EplRet);

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
                DWORD   dwNodeAssignment;
                WORD    wPresPayloadLimit;
                DWORD   dwCycTimeExceedThreshold = 15;

                    // configure OD for MN in state ResetComm after reseting the OD
                    // TODO: setup your own network configuration here
                    dwNodeAssignment = (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS);    // 0x00000003L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x01, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x02, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x03, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x04, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x20, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    dwNodeAssignment = (EPL_NODEASSIGN_MN_PRES | EPL_NODEASSIGN_NODE_EXISTS);       // 0x00010001L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xF0, &dwNodeAssignment, sizeof (dwNodeAssignment));

                    wPresPayloadLimit = 50;
                    EplRet = EplApiWriteLocalObject(0x1F8D, 0x01, &wPresPayloadLimit, sizeof (wPresPayloadLimit));
                    EplRet = EplApiWriteLocalObject(0x1F8D, 0x02, &wPresPayloadLimit, sizeof (wPresPayloadLimit));
                    EplRet = EplApiWriteLocalObject(0x1F8D, 0x03, &wPresPayloadLimit, sizeof (wPresPayloadLimit));
                    EplRet = EplApiWriteLocalObject(0x1F8D, 0x04, &wPresPayloadLimit, sizeof (wPresPayloadLimit));
                    EplRet = EplApiWriteLocalObject(0x1F8D, 0x20, &wPresPayloadLimit, sizeof (wPresPayloadLimit));

                    EplRet = EplApiWriteLocalObject(0x1C02, 0x03, &dwCycTimeExceedThreshold, sizeof (dwCycTimeExceedThreshold));
                    // continue
                }

                case kEplNmtGsResetConfiguration:
                {
                unsigned int uiSize;

                    // fetch object 0x1006 NMT_CycleLen_U32 from local OD (in little endian byte order)
                    // for configuration of remote CN
                    uiSize = 4;
                    EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00, &dw_le_CycleLen_g, &uiSize, kEplSdoTypeAsnd, NULL);
                    if (EplRet != kEplSuccessful)
                    {   // local OD access failed
                        break;
                    }

                    uiSize = 4;
                    EplRet = EplApiReadObject(NULL, 0, 0x1C14, 0x00, &dw_le_LossOfSocTolerance_g, &uiSize, kEplSdoTypeAsnd, NULL);
                    if (EplRet != kEplSuccessful)
                    {   // local OD access failed
                        break;
                    }

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

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplApiEventNode:
        {
            // check additional argument
            switch (pEventArg_p->m_Node.m_NodeEvent)
            {
                case kEplNmtNodeEventCheckConf:
                {
                tEplSdoComConHdl SdoComConHdl;

                    PRINTF2("%s(Node=0x%X, CheckConf)\n", __func__, pEventArg_p->m_Node.m_uiNodeId);

                    // update object 0x1006 on CN
                    EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId, 0x1006, 0x00, &dw_le_CycleLen_g, 4, kEplSdoTypeAsnd, NULL);
                    if (EplRet == kEplApiTaskDeferred)
                    {   // SDO transfer started
                        EplRet = kEplReject;
                    }
                    else if (EplRet == kEplSuccessful)
                    {   // local OD access (should not occur)
                        PRINTF1("%s(Node) write to local OD\n", __func__);
                    }
                    else
                    {   // error occurred
                        TGT_DBG_SIGNAL_TRACE_POINT(1);

                        EplRet = EplApiFreeSdoChannel(SdoComConHdl);
                        SdoComConHdl = 0;

                        EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId, 0x1006, 0x00, &dw_le_CycleLen_g, 4, kEplSdoTypeAsnd, NULL);
                        if (EplRet == kEplApiTaskDeferred)
                        {   // SDO transfer started
                            EplRet = kEplReject;
                        }
                        else
                        {
                            PRINTF2("%s(Node): EplApiWriteObject() returned 0x%02X\n", __func__, EplRet);
                        }
                    }

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

        case kEplApiEventSdo:
        {   // SDO transfer finished
            if (pEventArg_p->m_Sdo.m_pUserArg == NULL)
            {   // object 0x1006 written
                if (pEventArg_p->m_Sdo.m_SdoComConState == kEplSdoComTransferFinished)
                {   // ... successfully
                tEplSdoComConHdl SdoComConHdl;

                    PRINTF2("%s(Node=0x%X) first SDO finished\n", __func__, pEventArg_p->m_Sdo.m_uiNodeId);

                    // update object 0x1C14 on CN
                    EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Sdo.m_uiNodeId, 0x1C14, 0x00, &dw_le_LossOfSocTolerance_g, 4, kEplSdoTypeAsnd, (void*)1);
                    if (EplRet == kEplApiTaskDeferred)
                    {   // SDO transfer started
                        EplRet = kEplSuccessful;
                        break;
                    }
                    else if (EplRet == kEplSuccessful)
                    {   // local OD access (should not occur)
                        PRINTF1("%s(Node) write to local OD\n", __func__);
                    }
                    else
                    {   // error occurred
                        PRINTF2("%s(Node): EplApiWriteObject() returned 0x%02X\n", __func__, EplRet);
                    }
                }
                else
                {   // indicate configuration error CN
                    PRINTF3("%s(Node=0x%X) first SDO failed with abort code 0x%lX\n", __func__, pEventArg_p->m_Sdo.m_uiNodeId, pEventArg_p->m_Sdo.m_dwAbortCode);
                    EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId, kEplNmtNodeCommandConfErr);
                }
            }
            else
            {   // object 0x1C14 written
                EplRet = EplApiFreeSdoChannel(pEventArg_p->m_Sdo.m_SdoComConHdl);
                if (EplRet != kEplSuccessful)
                {
                    break;
                }

                if (pEventArg_p->m_Sdo.m_SdoComConState == kEplSdoComTransferFinished)
                {   // continue boot-up of CN with NMT command Reset Configuration
                    PRINTF2("%s(Node=0x%X) second SDO finished -> ConfReset\n", __func__, pEventArg_p->m_Sdo.m_uiNodeId);
                    EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId, kEplNmtNodeCommandConfReset);
                }
                else
                {   // indicate configuration error CN
                    PRINTF3("%s(Node=0x%X) second SDO failed with abort code 0x%lX\n", __func__, pEventArg_p->m_Sdo.m_uiNodeId, pEventArg_p->m_Sdo.m_dwAbortCode);
                    EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId, kEplNmtNodeCommandConfErr);
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

    if (bVarOut1Old_l != bVarOut1_l)
    {   // output variable has changed
        bVarOut1Old_l = bVarOut1_l;
        // set LEDs

//        printk("bVarIn = 0x%02X bVarOut = 0x%02X\n", (WORD) bVarIn_l, (WORD) bVarOut_l);
    }
//    if (uiNodeId_g != EPL_C_ADR_MN_DEF_NODE_ID)
    {
        bVarIn1_l++;
    }

    if (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID)
    {   // we are the master and must run the control loop

        // collect inputs from CNs and own input
        bSpeedSelect_l = /*bVarIn1_l |*/ abSelect_l[0];

        bModeSelect_l = abSelect_l[1] | abSelect_l[2];

        if ((bModeSelect_l & MODE_MASK) != 0)
        {
            dwMode_l = bModeSelect_l & MODE_MASK;
        }

        iCurCycleCount_l--;

        if (iCurCycleCount_l <= 0)
        {
            if ((dwMode_l & 0x01) != 0)
            {   // fill-up
                if (iToggle)
                {
                    if ((dwLeds_l & DOUBLE_LED_MASK) == 0x00)
                    {
                        dwLeds_l = 0x01;
                    }
                    else
                    {
                        dwLeds_l <<= 1;
                        dwLeds_l++;
                        if (dwLeds_l >= DOUBLE_LED_MASK)
                        {
                            iToggle = 0;
                        }
                    }
                }
                else
                {
                    dwLeds_l <<= 1;
                    if ((dwLeds_l & DOUBLE_LED_MASK) == 0x00)
                    {
                        iToggle = 1;
                    }
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x02) != 0)
            {   // running light forward
                dwLeds_l <<= 1;
                if ((dwLeds_l > DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 0x01;
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x04) != 0)
            {   // running light backward
                dwLeds_l >>= 1;
                if ((dwLeds_l > DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 1 << (LED_COUNT * 2);
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> LED_COUNT) & LED_MASK);
            }

            else if ((dwMode_l & 0x08) != 0)
            {   // Knightrider
                if (bLedsRow1_l == 0x00)
                {
                    bLedsRow1_l = 0x01;
                }
                else if (iToggle)
                {
                    bLedsRow1_l <<= 1;
                    if( bLedsRow1_l >= 0x10 )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    bLedsRow1_l >>= 1;
                    if( bLedsRow1_l <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
                bLedsRow2_l = bLedsRow1_l;
            }

            // set own output
            bVarOut1_l = bLedsRow1_l;
//            bVarOut1_l = (bLedsRow1_l & 0x03) | (bLedsRow2_l << 2);

            // restart cycle counter
            iCurCycleCount_l = iMaxCycleCount_l;
        }

        if (bSpeedSelectOld_l == 0)
        {
            if ((bSpeedSelect_l & 0x01) != 0)
            {
                if (iMaxCycleCount_l < 200)
                {
                    iMaxCycleCount_l++;
                }
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
            else if ((bSpeedSelect_l & 0x02) != 0)
            {
                if (iMaxCycleCount_l > 1)
                {
                    iMaxCycleCount_l--;
                }
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
            else if ((bSpeedSelect_l & 0x04) != 0)
            {
                iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;
                bSpeedSelectOld_l = bSpeedSelect_l;
            }
        }
        else if (bSpeedSelect_l == 0)
        {
            bSpeedSelectOld_l = 0;
        }
    }

    TGT_DBG_SIGNAL_TRACE_POINT(1);

    return EplRet;
}



// EOF

