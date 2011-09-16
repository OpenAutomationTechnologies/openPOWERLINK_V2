/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  demoapplication for EPL MN with CFM (with SDO over UDP)
                under Linux on X86 with RTL8139 Ethernet controller

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

                $Revision$

                $State$

                Build Environment:
                GCC

  -------------------------------------------------------------------------

  Revision History:

  2006/09/01 d.k.:   start of implementation

****************************************************************************/


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


#include "Epl.h"
#include "proc_fs.h"
#include "PosixFileLinuxKernel.h"


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

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    MODULE_DESCRIPTION("openPOWERLINK MN demo with CFM");
#else
    MODULE_DESCRIPTION("openPOWERLINK MN demo");
#endif
#endif

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

static wait_queue_head_t    WaitQueueShutdown_g; // wait queue for tEplNmtEventSwitchOff
static atomic_t             AtomicShutdown_g = ATOMIC_INIT(FALSE);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) == 0)
static DWORD    dw_le_CycleLen_g;
#endif

static uint uiNodeId_g = EPL_C_ADR_INVALID;
module_param_named(nodeid, uiNodeId_g, uint, 0);
MODULE_PARM_DESC(nodeid, "Local Node-ID of this POWERLINK node (0x01 - 0xEF -> CNs, 0xF0 -> MN");

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
static uint uiCycleLen_g = 0;
#else
static uint uiCycleLen_g = CYCLE_LEN;
#endif

#ifdef CONFIG_OPENCONFIGURATOR_MAPPING

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
/*----------------------------------------------------------------------------*/

#endif


module_param_named(cyclelen, uiCycleLen_g, uint, 0);
MODULE_PARM_DESC(cyclelen, "Cyclelength in [µs] (it is stored in object 0x1006)");

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
static char* pszCdcFilename_g = EPL_OBD_DEF_CONCISEDCF_FILENAME;
module_param_named(cdc, pszCdcFilename_g, charp, 0);
MODULE_PARM_DESC(cdc, "Full path to ConciseDCF (CDC file) which is imported into the local object dictionary");
#endif

static FD_TYPE hAppFdTracingEnabled_g;


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
static  int  __init  EplLinInit (void)
{
tEplKernel          EplRet;
static tEplApiInitParam EplApiInitParam = {0};
char*               sHostname = HOSTNAME;

#ifndef CONFIG_OPENCONFIGURATOR_MAPPING
unsigned int        uiVarEntries;
tEplObdSize         ObdSize;
#endif

BOOL                fApiInit = FALSE;
BOOL                fLinProcInit =FALSE;

    atomic_set(&AtomicShutdown_g, TRUE);

    // open character device from debugfs to disable tracing when necessary
    hAppFdTracingEnabled_g = open("/sys/kernel/debug/tracing/tracing_enabled", O_WRONLY, 0666);

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
    EplApiInitParam.m_dwFeatureFlags = (DWORD) ~0UL;
    EplApiInitParam.m_dwCycleLen = uiCycleLen_g;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 100; // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 100; // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;  // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36; // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36; // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;   // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;  // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;         // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;         // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    EplApiInitParam.m_dwWaitSocPreq = 0;
#else
    EplApiInitParam.m_dwWaitSocPreq = 150000;
#endif
    EplApiInitParam.m_dwDeviceType = (DWORD) ~0UL;      // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = (DWORD) ~0UL;        // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = (DWORD) ~0UL;     // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = (DWORD) ~0UL;  // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = (DWORD) ~0UL;    // NMT_IdentityObject_REC.SerialNo_U32
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
        PRINTF0("EplLinProcInit failed!\n");
        goto Exit;
    }
    fLinProcInit = TRUE;

    // initialize POWERLINK stack
    EplRet = EplApiInitialize(&EplApiInitParam);
    if(EplRet != kEplSuccessful)
    {
        PRINTF0("EplApiInitialize failed!\n");
        goto Exit;
    }
    fApiInit = TRUE;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    EplRet = EplApiSetCdcFilename(pszCdcFilename_g);
    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#ifdef CONFIG_OPENCONFIGURATOR_MAPPING
    PRINTF0("Initializing process image...\n");
    PRINTF1("Size of input process image: %ld\n", sizeof(AppProcessImageIn_g));
    PRINTF1("Size of output process image: %ld\n", sizeof (AppProcessImageOut_g));

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
#else
    PRINTF0("Linking objects ...\n");

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

    // link process variables used by MN to object dictionary
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    ObdSize = sizeof(bLedsRow1_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bLedsRow1_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    ObdSize = sizeof(bLedsRow2_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bLedsRow2_l, &uiVarEntries, &ObdSize, 0x02);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    ObdSize = sizeof(bSpeedSelect_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bSpeedSelect_l, &uiVarEntries, &ObdSize, 0x03);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    ObdSize = sizeof(bSpeedSelectOld_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x2000, &bSpeedSelectOld_l, &uiVarEntries, &ObdSize, 0x04);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    ObdSize = sizeof(abSelect_l[0]);
    uiVarEntries = sizeof(abSelect_l);
    EplRet = EplApiLinkObject(0x2200, &abSelect_l[0], &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }
#endif

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
    dwMode_l = APP_DEFAULT_MODE;
    iMaxCycleCount_l = DEFAULT_MAX_CYCLE_COUNT;
#endif


    // start the NMT state machine
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    atomic_set(&AtomicShutdown_g, FALSE);

Exit:
    PRINTF1("EplLinInit(): returns 0x%X\n", EplRet);
    if (EplRet != kEplSuccessful)
    {
        if (fApiInit != FALSE)
        {
            EplApiShutdown();
        }
        if (fLinProcInit != FALSE)
        {
            EplLinProcFree();
        }

        return -ENODEV;
    }
    else
    {
        return 0;
    }
}

static  void  __exit  EplLinExit (void)
{
tEplKernel          EplRet;

    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    // wait until NMT state machine is shut down
    wait_event_interruptible(WaitQueueShutdown_g,
                                    (atomic_read(&AtomicShutdown_g) == TRUE));
/*    if ((iErr != 0) || (atomic_read(&AtomicShutdown_g) == EVENT_STATE_IOCTL))
    {   // waiting was interrupted by signal or application called wrong function
        EplRet = kEplShutdown;
    }*/

#ifdef CONFIG_OPENCONFIGURATOR_MAPPING
    // Free resources used by the process image API
    EplRet = EplApiProcessImageFree();
#endif

    // delete instance for all modules
    EplRet = EplApiShutdown();
    PRINTF1("EplApiShutdown():  0x%X\n", EplRet);

    // deinitialize proc fs
    EplRet = EplLinProcFree();
    PRINTF1("EplLinProcFree():        0x%X\n", EplRet);

    if (IS_FD_VALID(hAppFdTracingEnabled_g))
    {
        close(hAppFdTracingEnabled_g);
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

                    // wake up EplLinExit()
                    atomic_set(&AtomicShutdown_g, TRUE);
                    wake_up_interruptible(&WaitQueueShutdown_g);
                    break;
                }

                case kEplNmtGsResetCommunication:
                {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
                DWORD   dwNodeAssignment;

                    // configure OD for MN in state ResetComm after reseting the OD
                    // TODO: setup your own network configuration here
                    dwNodeAssignment = (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS);    // 0x00000003L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x01, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x02, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x03, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x04, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x05, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x06, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x07, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x08, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x20, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xFE, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwNodeAssignment, sizeof (dwNodeAssignment));

//                    dwNodeAssignment |= EPL_NODEASSIGN_MANDATORY_CN;    // 0x0000000BL
//                    EplRet = EplApiWriteLocalObject(0x1F81, 0x6E, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    dwNodeAssignment = (EPL_NODEASSIGN_MN_PRES | EPL_NODEASSIGN_NODE_EXISTS);       // 0x00010001L
                    EplRet = EplApiWriteLocalObject(0x1F81, 0xF0, &dwNodeAssignment, sizeof (dwNodeAssignment));
#endif
                    // continue
                }

                case kEplNmtGsResetConfiguration:
                {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
                    if (uiCycleLen_g != 0)
                    {
                        EplRet = EplApiWriteLocalObject(0x1006, 0x00, &uiCycleLen_g, sizeof (uiCycleLen_g));
                    }
#else
                    unsigned int uiSize;

                    // fetch object 0x1006 NMT_CycleLen_U32 from local OD (in little endian byte order)
                    // for configuration of remote CN
                    uiSize = 4;
                    EplRet = EplApiReadObject(NULL, 0, 0x1006, 0x00, &dw_le_CycleLen_g, &uiSize, kEplSdoTypeAsnd, NULL);
                    if (EplRet != kEplSuccessful)
                    {   // local OD access failed
                        break;
                    }
#endif
                    // continue
                }

                case kEplNmtMsPreOperational1:
                {
                    PRINTF4("%s(0x%X -> 0x%X) originating event = 0x%X\n",
                            __func__,
                           pEventArg_p->m_NmtStateChange.m_OldNmtState,
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
        {   // error or warning occured within the stack or the application
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
                {   // error occured within event processing
                    // either in kernel or in user part
                    PRINTF1(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occured within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    PRINTF1(" val=%lX\n", (ULONG) pEventArg_p->m_InternalError.m_Arg.m_dwArg);
                    break;
                }

                case kEplEventSourceObdk:
                case kEplEventSourceObdu:
                {   // error occured within OBD module
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

            if ((pEventArg_p->m_ErrHistoryEntry.m_wErrorCode == EPL_E_DLL_CYCLE_EXCEED_TH)
                && (IS_FD_VALID(hAppFdTracingEnabled_g)))
            {
            mm_segment_t    old_fs;
            loff_t          pos;
            ssize_t         iRet;

                old_fs = get_fs();
                set_fs(KERNEL_DS);

                pos = hAppFdTracingEnabled_g->f_pos;
                iRet = vfs_write(hAppFdTracingEnabled_g, "0", 1, &pos);
                hAppFdTracingEnabled_g->f_pos = pos;
            }

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
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) == 0)
                 tEplSdoComConHdl SdoComConHdl;
                    // update object 0x1006 on CN
                    EplRet = EplApiWriteObject(&SdoComConHdl, pEventArg_p->m_Node.m_uiNodeId, 0x1006, 0x00, &dw_le_CycleLen_g, 4, kEplSdoTypeAsnd, NULL);
                    if (EplRet == kEplApiTaskDeferred)
                    {   // SDO transfer started
                        EplRet = kEplReject;
                    }
                    else if (EplRet == kEplSuccessful)
                    {   // local OD access (should not occur)
                        printk("AppCbEvent(Node) write to local OD\n");
                    }
                    else
                    {   // error occured
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
                            printk("AppCbEvent(Node): EplApiWriteObject() returned 0x%02X\n", EplRet);
                        }
                    }
#endif
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

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) == 0)
        case kEplApiEventSdo:
        {   // SDO transfer finished
            EplRet = EplApiFreeSdoChannel(pEventArg_p->m_Sdo.m_SdoComConHdl);
            if (EplRet != kEplSuccessful)
            {
                break;
            }
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            if (pEventArg_p->m_Sdo.m_SdoComConState == kEplSdoComTransferFinished)
            {   // continue boot-up of CN with NMT command Reset Configuration
                EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId, kEplNmtNodeCommandConfReset);
            }
            else
            {   // indicate configuration error CN
                EplRet = EplApiMnTriggerStateChange(pEventArg_p->m_Sdo.m_uiNodeId, kEplNmtNodeCommandConfErr);
            }
#endif

            break;
        }
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        case kEplApiEventCfmProgress:
        {
            PRINTF4("%s(Node=0x%X, CFM-Progress: Object 0x%X/%u, ", __func__, pEventArg_p->m_CfmProgress.m_uiNodeId, pEventArg_p->m_CfmProgress.m_uiObjectIndex, pEventArg_p->m_CfmProgress.m_uiObjectSubIndex);
            PRINTF2("%lu/%lu Bytes", (ULONG) pEventArg_p->m_CfmProgress.m_dwBytesDownloaded, (ULONG) pEventArg_p->m_CfmProgress.m_dwTotalNumberOfBytes);
            if ((pEventArg_p->m_CfmProgress.m_dwSdoAbortCode != 0)
                || (pEventArg_p->m_CfmProgress.m_EplError != kEplSuccessful))
            {
                PRINTF2(" -> SDO Abort=0x%lX, Error=0x%X)\n", (unsigned long) pEventArg_p->m_CfmProgress.m_dwSdoAbortCode,
                                                              pEventArg_p->m_CfmProgress.m_EplError);
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

#ifdef CONFIG_OPENCONFIGURATOR_MAPPING
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

#else
    if (bVarOut1Old_l != bVarOut1_l)
    {   // output variable has changed
        bVarOut1Old_l = bVarOut1_l;
        // set LEDs

//        printk("bVarIn = 0x%02X bVarOut = 0x%02X\n", (WORD) bVarIn_l, (WORD) bVarOut_l);
    }
    bVarIn1_l++;

    if (uiNodeId_g == EPL_C_ADR_MN_DEF_NODE_ID)
    {   // we are the master and must run the control loop

        // collect inputs from CNs and own input
        bSpeedSelect_l = abSelect_l[0];

        bModeSelect_l = abSelect_l[1] | abSelect_l[2];

        if ((bModeSelect_l & APP_MODE_MASK) != 0)
        {
            dwMode_l = bModeSelect_l & APP_MODE_MASK;
        }

        iCurCycleCount_l--;

        if (iCurCycleCount_l <= 0)
        {
            if ((dwMode_l & 0x01) != 0)
            {   // fill-up
                if (iToggle)
                {
                    if ((dwLeds_l & APP_DOUBLE_LED_MASK) == 0x00)
                    {
                        dwLeds_l = 0x01;
                    }
                    else
                    {
                        dwLeds_l <<= 1;
                        dwLeds_l++;
                        if (dwLeds_l >= APP_DOUBLE_LED_MASK)
                        {
                            iToggle = 0;
                        }
                    }
                }
                else
                {
                    dwLeds_l <<= 1;
                    if ((dwLeds_l & APP_DOUBLE_LED_MASK) == 0x00)
                    {
                        iToggle = 1;
                    }
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & APP_LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x02) != 0)
            {   // running light forward
                dwLeds_l <<= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 0x01;
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & APP_LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x04) != 0)
            {   // running light backward
                dwLeds_l >>= 1;
                if ((dwLeds_l > APP_DOUBLE_LED_MASK) || (dwLeds_l == 0x00000000L))
                {
                    dwLeds_l = 1 << (APP_LED_COUNT * 2);
                }
                bLedsRow1_l = (unsigned char) (dwLeds_l & APP_LED_MASK);
                bLedsRow2_l = (unsigned char) ((dwLeds_l >> APP_LED_COUNT) & APP_LED_MASK);
            }

            else if ((dwMode_l & 0x08) != 0)
            {   // Knightrider
                if (bLedsRow1_l == 0x00)
                {
                    bLedsRow1_l = 0x01;
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    bLedsRow1_l <<= 1;
                    if ( bLedsRow1_l >= (1 << (APP_LED_COUNT - 1)) )
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

            else if ((dwMode_l & 0x10) != 0)
            {   // Knightrider
                if ((bLedsRow1_l == 0x00)
                    || (bLedsRow2_l == 0x00)
                    || ((bLedsRow2_l & ~APP_LED_MASK) != 0))
                {
                    bLedsRow1_l = 0x01;
                    bLedsRow2_l = (1 << (APP_LED_COUNT - 1));
                    iToggle = 1;
                }
                else if (iToggle)
                {
                    bLedsRow1_l <<= 1;
                    bLedsRow2_l >>= 1;
                    if ( bLedsRow1_l >= (1 << (APP_LED_COUNT - 1)) )
                    {
                        iToggle = 0;
                    }
                }
                else
                {
                    bLedsRow1_l >>= 1;
                    bLedsRow2_l <<= 1;
                    if ( bLedsRow1_l <= 0x01 )
                    {
                        iToggle = 1;
                    }
                }
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
#endif

    return EplRet;
}



// EOF
