/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  demoapplication for EPL CN (with SDO over UDP)
                under Linux on Freescale Coldfire MCF5484

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
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2006/09/01 d.k.:   start of implementation

****************************************************************************/


#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_COLDFIRE
    #include <asm/coldfire.h>
    #include <asm/mcfsim.h>
    #include <asm/m5485gpio.h>
#endif

#include "Epl.h"
#include "proc_fs.h"
#ifdef CONFIG_COLDFIRE
    #include "cf54drv.h"
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    // remove ("make invisible") obsolete symbols for kernel versions 2.6
    // and higher
    #define MOD_INC_USE_COUNT
    #define MOD_DEC_USE_COUNT
    #define EXPORT_NO_SYMBOLS
#else
    #error "This driver needs a 2.6.x kernel or higher"
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

// Metainformation
MODULE_LICENSE("Dual BSD/GPL");
#ifdef MODULE_AUTHOR
    MODULE_AUTHOR("Daniel.Krueger@SYSTEC-electronic.com");
    MODULE_DESCRIPTION("EPL CN demo");
#endif

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


#define CF54DRV

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif


// POWERLINK default parameters
#define NODEID      0x1 //0x6E
#define CYCLE_LEN   3000 // [us]
#define IP_ADDR     0xc0a86401  // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0
#define HOSTNAME    "SYS TEC electronic openPOWERLINK"
#define IF_ETH      EPL_VETH_NAME


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

CONST BYTE abMacAddr[] = {0x00, 0x03, 0xc0, 0xa8, 0x64, 0xf1};


// process variables
BYTE    bVarIn1_l;
BYTE    bVarOut1_l;
BYTE    bVarOut1Old_l;

BYTE    abDomain_l[3000];


// Linux wait queue for shutdown
static wait_queue_head_t    WaitQueueShutdown_g; // wait queue for tEplNmtEventSwitchOff
static atomic_t             AtomicShutdown_g = ATOMIC_INIT(FALSE);


// module parameters like node-ID and cycle length
static uint uiNodeId_g = EPL_C_ADR_INVALID;
module_param_named(nodeid, uiNodeId_g, uint, 0);

static uint uiCycleLen_g = CYCLE_LEN;
module_param_named(cyclelen, uiCycleLen_g, uint, 0);


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

#ifdef CF54DRV
extern int      PLCcoreCF54DrvCmdInitialize      (WORD* pwDrvVer_p);
extern int      PLCcoreCF54DrvCmdShutdown        (void);
extern int      PLCcoreCF54DrvCmdResetTarget     (void);
extern int      PLCcoreCF54DrvCmdEnableWatchdog  (void);
extern int      PLCcoreCF54DrvCmdServiveWatchdog (void);
//extern int      PLCcoreCF54DrvCmdGetHardwareInfo (tCF54HwInfo* pCF54HwInfo_p);
extern int      PLCcoreCF54DrvCmdSetRunLED       (BYTE bState_p);
extern int      PLCcoreCF54DrvCmdSetErrLED       (BYTE bState_p);
extern int      PLCcoreCF54DrvCmdGetRSMSwitch    (BYTE* pbRSMSwitch_p);
extern int      PLCcoreCF54DrvCmdGetHexSwitch    (BYTE* pbHexSwitch_p);
extern int      PLCcoreCF54DrvCmdGetDipSwitch    (BYTE* pbDipSwitch_p);
extern int      PLCcoreCF54DrvCmdGetDigiIn       (tCF54DigiIn* pDiData_p);
extern int      PLCcoreCF54DrvCmdSetDigiOut      (tCF54DigiOut* pDoData_p);
#endif

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

static int  __init  EplLinInit (void);
static void __exit  EplLinExit (void);

//---------------------------------------------------------------------------
//  Kernel Module specific Data Structures
//---------------------------------------------------------------------------

EXPORT_NO_SYMBOLS;


module_init(EplLinInit);
module_exit(EplLinExit);


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplLinInit
//
// Description: initialization of EPL module
//
// Parameters:  void
//
// Returns:     int                 = error code
//
// State:
//
//---------------------------------------------------------------------------

static  int  __init  EplLinInit (void)
{
tEplKernel          EplRet;
int                 iRet;
static tEplApiInitParam EplApiInitParam = {0};
char*               sHostname = HOSTNAME;
WORD                wDrvVersion;
BYTE                bDipSwitch;
char*               argv[4], *envp[3];
char                sBuffer[16];
unsigned int        uiVarEntries;
tEplObdSize         ObdSize;

    atomic_set(&AtomicShutdown_g, TRUE);

    // get node ID from insmod command line
    EplApiInitParam.m_uiNodeId = uiNodeId_g;

#ifdef CF54DRV
    iRet = PLCcoreCF54DrvCmdInitialize(&wDrvVersion);

    if (EplApiInitParam.m_uiNodeId == EPL_C_ADR_INVALID)
    {   // invalid node ID set

        // get node ID from hex switch
        iRet = PLCcoreCF54DrvCmdGetHexSwitch(&bDipSwitch);
        EplApiInitParam.m_uiNodeId = bDipSwitch;
    }
#endif

    if (EplApiInitParam.m_uiNodeId == EPL_C_ADR_INVALID)
    {   // invalid node ID set
        // set default node ID
        EplApiInitParam.m_uiNodeId = NODEID;
    }

    uiNodeId_g = EplApiInitParam.m_uiNodeId;

    // calculate IP address
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;

    // configure CN to async-only according to DIP switch
#ifdef CF54DRV
    iRet = PLCcoreCF54DrvCmdGetDipSwitch(&bDipSwitch);
#else
    bDipSwitch = 0;
#endif

    if ((bDipSwitch & 0x01) != 0)
    {   // DIP 1 is on
        EplApiInitParam.m_fAsyncOnly = TRUE;
    }
    else
    {
        EplApiInitParam.m_fAsyncOnly = FALSE;
    }

    // set up initialization parameters (e.g. cycle length, MAC address)
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));
    EplApiInitParam.m_abMacAddress[5] = (BYTE) EplApiInitParam.m_uiNodeId;
    EplApiInitParam.m_dwFeatureFlags = -1;  // determined by stack itself
    EplApiInitParam.m_dwCycleLen = uiCycleLen_g;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 100; // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 100; // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;  // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36; // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36; // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;   // in [ns] const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;// required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;    // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;      // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;  // in [ns]; required to detect loss of SoC
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;     // in [ns]; MN AsyncSlotTimeout_U21 in [ns]
    EplApiInitParam.m_dwWaitSocPreq = 150000;           // in [ns]; MN WaitSocPreq_U32 in [ns]
    EplApiInitParam.m_dwDeviceType = -1;              // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = -1;                // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = -1;             // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = -1;          // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = -1;            // NMT_IdentityObject_REC.SerialNo_U32
    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));

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


    printk("\n\n Hello, I'm a simple POWERLINK node running as %s!\n  (build: %s / %s)\n\n",
            (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID ?
                "Managing Node" : "Controlled Node"),
            __DATE__, __TIME__);

    // initialize the Linux a wait queue for shutdown of this module
    init_waitqueue_head(&WaitQueueShutdown_g);

    // initialize the procfs device
    EplRet = EplLinProcInit();
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // initialize POWERLINK stack
    EplRet = EplApiInitialize(&EplApiInitParam);
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
        goto Exit;
    }

    ObdSize = sizeof(bVarOut1_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6200, &bVarOut1_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // link a DOMAIN to object 0x6100, but do not exit, if it is missing
    ObdSize = sizeof(abDomain_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6100, &abDomain_l, &uiVarEntries, &ObdSize, 0x00);
    if (EplRet != kEplSuccessful)
    {
        printk("EplApiLinkObject(0x6100): returns 0x%X\n", EplRet);
    }

    // reset old process variables
    bVarOut1Old_l = 0;


    // configure IP address of virtual network interface
    // for TCP/IP communication over the POWERLINK network
    sprintf(sBuffer, "%lu.%lu.%lu.%lu", (EplApiInitParam.m_dwIpAddress >> 24), ((EplApiInitParam.m_dwIpAddress >> 16) & 0xFF), ((EplApiInitParam.m_dwIpAddress >> 8) & 0xFF), (EplApiInitParam.m_dwIpAddress & 0xFF));
    /* set up a minimal environment */
    iRet = 0;
    envp[iRet++] = "HOME=/";
    envp[iRet++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
    envp[iRet] = NULL;

    /* set up the argument list */
    iRet = 0;
    argv[iRet++] = "/sbin/ifconfig";
    argv[iRet++] = IF_ETH;
    argv[iRet++] = sBuffer;
    argv[iRet] = NULL;

    /* call ifconfig to configure the virtual network interface */
    iRet = call_usermodehelper(argv[0], argv, envp, 1);
    printk("ifconfig %s %s returned %d\n", argv[1], argv[2], iRet);

    // start the NMT state machine
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    atomic_set(&AtomicShutdown_g, FALSE);

Exit:
    printk("EplLinInit(): returns 0x%X\n", EplRet);
    if (EplRet != kEplSuccessful)
    {
        return -ENODEV;
    }
    else
    {
        return 0;
    }
}


//---------------------------------------------------------------------------
//
// Function:    EplLinExit
//
// Description: shutdown of EPL module
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------

static  void  __exit  EplLinExit (void)
{
tEplKernel          EplRet;
#ifdef CF54DRV
int                 iRet;
#endif

    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // wait until NMT state machine is shut down
    wait_event_interruptible(WaitQueueShutdown_g,
                                    (atomic_read(&AtomicShutdown_g) == TRUE));

    // delete instance for all modules
    EplRet = EplApiShutdown();
    printk("EplApiShutdown():  0x%X\n", EplRet);

    // deinitialize proc fs
    EplRet = EplLinProcFree();
    printk("EplLinProcFree():        0x%X\n", EplRet);

#ifdef CF54DRV
    iRet = PLCcoreCF54DrvCmdShutdown();
#endif
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

                    printk("AppCbEvent(kEplNmtGsOff) originating event = 0x%X\n", pEventArg_p->m_NmtStateChange.m_NmtEvent);

                    // wake up EplLinExit()
                    atomic_set(&AtomicShutdown_g, TRUE);
                    wake_up_interruptible(&WaitQueueShutdown_g);
                    break;
                }

                case kEplNmtGsResetCommunication:
                case kEplNmtGsResetConfiguration:
                case kEplNmtMsPreOperational1:
                {
                    printk("AppCbEvent(0x%X) originating event = 0x%X\n",
                           pEventArg_p->m_NmtStateChange.m_NewNmtState,
                           pEventArg_p->m_NmtStateChange.m_NmtEvent);

                    // continue
                }

                case kEplNmtGsInitialising:
                case kEplNmtGsResetApplication:
                case kEplNmtMsNotActive:
                case kEplNmtCsNotActive:
                case kEplNmtCsPreOperational1:
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

            printk("AppCbEvent(Err/Warn): Source=%02X EplError=0x%03X", pEventArg_p->m_InternalError.m_EventSource, pEventArg_p->m_InternalError.m_EplError);
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {   // error occurred within event processing
                    // either in kernel or in user part
                    printk(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occurred within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    printk(" val=%lX\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    break;
                }

                default:
                {
                    printk("\n");
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

#ifdef CF54DRV
        case kEplApiEventLed:
        {   // status or error LED shall be changed

            switch (pEventArg_p->m_Led.m_LedType)
            {
                case kEplLedTypeStatus:
                {
                    PLCcoreCF54DrvCmdSetRunLED(pEventArg_p->m_Led.m_fOn);
                    break;
                }

                case kEplLedTypeError:
                {
                    PLCcoreCF54DrvCmdSetErrLED(pEventArg_p->m_Led.m_fOn);
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
#if defined(CF54DRV)
tCF54DigiIn         CF54DigiIn;
#if (BENCHMARK_MODULES == 0)
tCF54DigiOut        CF54DigiOut;
#endif
#endif

    // set outputs
    if (bVarOut1Old_l != bVarOut1_l)
    {   // output variable has changed
        bVarOut1Old_l = bVarOut1_l;
        // set LEDs
#if defined(CF54DRV) && (BENCHMARK_MODULES == 0)
        CF54DigiOut.m_bDoByte0 = bVarOut1_l;
        PLCcoreCF54DrvCmdSetDigiOut(&CF54DigiOut);

#else
//        printk("bVarIn = 0x%02X bVarOut = 0x%02X\n", (WORD) bVarIn1_l, (WORD) bVarOut1_l);
#endif

    }

    // read inputs
#ifdef CF54DRV
    PLCcoreCF54DrvCmdGetDigiIn(&CF54DigiIn);
    bVarIn1_l = CF54DigiIn.m_bDiByte0;

#else
    bVarIn1_l++;
#endif

    TGT_DBG_SIGNAL_TRACE_POINT(1);

    return EplRet;
}



// EOF

