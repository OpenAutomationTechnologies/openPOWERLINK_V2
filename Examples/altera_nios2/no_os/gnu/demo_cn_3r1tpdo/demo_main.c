
#include "Epl.h"

#include "system.h"
#ifdef NODESWITCH_SPI_BASE
#include "altera_avalon_spi_regs.h"
#endif
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include <sys/alt_cache.h>

#define NODEID      0x01 // should be NOT 0xF0 (=MN) in case of CN
#define CYCLE_LEN   1000 // [us]
#define MAC_ADDR	0x00, 0x12, 0x34, 0x56, 0x78, 0x9A
#define IP_ADDR     0xc0a86401  // 192.168.100.1 // don't care the last byte!
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0


// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
// this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.

tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);


tEplKernel PUBLIC AppCbSync(void);
tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p);

// 50 byte input process vars
static BYTE     bButtonInputs_l         = 0;
static BYTE     abVirtualInputs_l[17];
static DWORD    adwVirtualInputs_l[8];

// 50 byte output process vars
static WORD     wDigitalOutputs_l       = 0;
static BYTE     bLedOutputs_l           = 0;
static BYTE     abVirtualOutputs_l[15];
static DWORD    adwVirtualOutputs_l[8];


static BOOL     fShutdown_l = FALSE;

int openPowerlink(void);

int main(void) {
    int i=0;

    alt_icache_flush_all();
    alt_dcache_flush_all();

    printf("NIOS II is running...\n");
    printf("starting openPowerlink application...\n\n");
    while (1) {
        if (openPowerlink() != 0) {
            printf("openPowerlink was shut down because of an error\n");
            break;
        } else {
            printf("openPowerlink was shut down, restart...\n\n");
        }
        for (i=0; i<1000000; i++);
    }
    printf("shut down NIOS II...\n%c", 4);

    return 0;
}


int openPowerlink(void)
{
    const BYTE              abMacAddr[] = {MAC_ADDR};
    static tEplApiInitParam EplApiInitParam = {0};
	// needed for process var
	tEplObdSize         	ObdSize;
	tEplKernel 				EplRet;
	unsigned int			uiVarEntries;

    fShutdown_l = FALSE;

	////////////////////////
	// setup th EPL Stack //
	////////////////////////

	// set EPL init parameters
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);

#ifdef NODESWITCH_SPI_BASE
    // read node-ID from hex switch on baseboard, which is connected via SPI shift register
    IOWR_ALTERA_AVALON_SPI_TXDATA(NODESWITCH_SPI_BASE, 0xFF);   // generate pulse for latching inputs
    while ((IORD_ALTERA_AVALON_SPI_STATUS(NODESWITCH_SPI_BASE) & ALTERA_AVALON_SPI_STATUS_RRDY_MSK) == 0)
    {   // wait
    }
    EplApiInitParam.m_uiNodeId = IORD_ALTERA_AVALON_SPI_RXDATA(NODESWITCH_SPI_BASE);
#endif
#ifdef NODESWITCH_PIO_BASE
    EplApiInitParam.m_uiNodeId = IORD_ALTERA_AVALON_PIO_DATA(NODESWITCH_PIO_BASE);
#endif

    if (EplApiInitParam.m_uiNodeId == EPL_C_ADR_INVALID)
    {
        EplApiInitParam.m_uiNodeId = NODEID; // defined at the top of this file!
    }

    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof(EplApiInitParam.m_abMacAddress));
    EplApiInitParam.m_abMacAddress[5] = (BYTE) EplApiInitParam.m_uiNodeId;

    // calculate IP address
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;
	EplApiInitParam.m_uiIsochrTxMaxPayload = 256;
	EplApiInitParam.m_uiIsochrRxMaxPayload = 256;
	EplApiInitParam.m_dwPresMaxLatency = 2000;
	EplApiInitParam.m_dwAsndMaxLatency = 2000;
	EplApiInitParam.m_fAsyncOnly = FALSE;
	EplApiInitParam.m_dwFeatureFlags = -1;
	EplApiInitParam.m_dwCycleLen = CYCLE_LEN;
	EplApiInitParam.m_uiPreqActPayloadLimit = 36;
	EplApiInitParam.m_uiPresActPayloadLimit = 36;
	EplApiInitParam.m_uiMultiplCycleCnt = 0;
	EplApiInitParam.m_uiAsyncMtu = 1500;
	EplApiInitParam.m_uiPrescaler = 2;
	EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
	EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
	EplApiInitParam.m_dwWaitSocPreq = 0;
	EplApiInitParam.m_dwDeviceType = -1;
	EplApiInitParam.m_dwVendorId = -1;
	EplApiInitParam.m_dwProductCode = -1;
	EplApiInitParam.m_dwRevisionNumber = -1;
	EplApiInitParam.m_dwSerialNumber = -1;
	EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
	EplApiInitParam.m_dwDefaultGateway = 0;
    EplApiInitParam.m_dwApplicationSwDate = 1;       // PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
	EplApiInitParam.m_pfnCbEvent = AppCbEvent;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
    EplApiInitParam.m_dwSyncResLatency = EPL_C_DLL_T_IFG;

	// initialize EPL stack
    printf("init EPL Stack with node-ID 0x%02X:\n", EplApiInitParam.m_uiNodeId);
	EplRet = EplApiInitialize(&EplApiInitParam);
	if(EplRet != kEplSuccessful) {
        printf("init EPL Stack... error %X\n\n", EplRet);
		goto Exit;
    }
    printf("init EPL Stack...ok\n\n");

	// link process variables used by CN to object dictionary
    printf("linking process vars:\n");
    ObdSize = sizeof(bButtonInputs_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6000, &bButtonInputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(abVirtualInputs_l[0]);
    uiVarEntries = 17;
    EplRet = EplApiLinkObject(0x2000, abVirtualInputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(adwVirtualInputs_l[0]);
    uiVarEntries = 8;
    EplRet = EplApiLinkObject(0x2001, adwVirtualInputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(wDigitalOutputs_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6300, &wDigitalOutputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(bLedOutputs_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6200, &bLedOutputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(abVirtualOutputs_l[0]);
    uiVarEntries = 15;
    EplRet = EplApiLinkObject(0x2200, abVirtualOutputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(adwVirtualOutputs_l[0]);
    uiVarEntries = 8;
    EplRet = EplApiLinkObject(0x2201, adwVirtualOutputs_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }
	printf("linking process vars... ok\n\n");

	// start the EPL stack
    printf("start EPL Stack...\n");
	EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful) {
        printf("start EPL Stack... error\n\n");
        goto ExitShutdown;
    }
    printf("start EPL Stack... ok\n\n");

    printf("NIOS II with openPowerlink is ready!\n\n");

#ifdef LED_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_BASE, 0xFF);
#endif

    while(1)
    {
        EplApiProcess();
        if (fShutdown_l == TRUE)
            break;
    }

ExitShutdown:
    printf("Shutdown EPL Stack\n");
    EplApiShutdown(); //shutdown node

Exit:
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
//
// State:
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
                    // because of critical EPL stack error
                    // -> also shut down EplApiProcess() and main()
                    EplRet = kEplShutdown;
                    fShutdown_l = TRUE;

                    PRINTF2("%s(kEplNmtGsOff) originating event = 0x%X\n", __func__, pEventArg_p->m_NmtStateChange.m_NmtEvent);
                    break;
                }

                case kEplNmtGsInitialising:
                case kEplNmtGsResetApplication:
                case kEplNmtGsResetConfiguration:
                case kEplNmtCsPreOperational1:
                case kEplNmtCsBasicEthernet:
                case kEplNmtMsBasicEthernet:
                {
                    PRINTF3("%s(0x%X) originating event = 0x%X\n",
                            __func__,
                            pEventArg_p->m_NmtStateChange.m_NewNmtState,
                            pEventArg_p->m_NmtStateChange.m_NmtEvent);
                    break;
                }

                case kEplNmtGsResetCommunication:
                {
                BYTE    bNodeId = 0xF0;
                DWORD   dwNodeAssignment = EPL_NODEASSIGN_NODE_EXISTS;
                WORD    wPresPayloadLimit = 256;

                    PRINTF3("%s(0x%X) originating event = 0x%X\n",
                            __func__,
                            pEventArg_p->m_NmtStateChange.m_NewNmtState,
                            pEventArg_p->m_NmtStateChange.m_NmtEvent);

                    EplRet = EplApiWriteLocalObject(0x1F81, bNodeId, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    if (EplRet != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    bNodeId = 0x04;
                    EplRet = EplApiWriteLocalObject(0x1F81, bNodeId, &dwNodeAssignment, sizeof (dwNodeAssignment));
                    if (EplRet != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    EplRet = EplApiWriteLocalObject(0x1F8D, bNodeId, &wPresPayloadLimit, sizeof (wPresPayloadLimit));
                    if (EplRet != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    break;
                }

                case kEplNmtMsNotActive:
                    break;
                case kEplNmtCsNotActive:
                    break;
                    break;

                case kEplNmtCsOperational:
                    break;
                case kEplNmtMsOperational:
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

        case kEplApiEventLed:
        {   // status or error LED shall be changed

            switch (pEventArg_p->m_Led.m_LedType)
            {
#ifdef LED_STATUS_PIO_BASE
                case kEplLedTypeStatus:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        IOWR_ALTERA_AVALON_PIO_DATA(LED_STATUS_PIO_BASE, 1);
                    }
                    else
                    {
                        IOWR_ALTERA_AVALON_PIO_DATA(LED_STATUS_PIO_BASE, 0);
                    }
                    break;
                }
#endif
#ifdef LED_ERROR_PIO_BASE
                case kEplLedTypeError:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        IOWR_ALTERA_AVALON_PIO_DATA(LED_ERROR_PIO_BASE, 1);
                    }
                    else
                    {
                        IOWR_ALTERA_AVALON_PIO_DATA(LED_ERROR_PIO_BASE, 0);
                    }
                    break;
                }
#endif
                default:
                    break;
            }
            break;
        }

        default:
            break;
    }

Exit:
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
	tEplKernel EplRet = kEplSuccessful;
    int        nIdx;

#ifdef DIN_PIO_BASE
    bButtonInputs_l = IORD_ALTERA_AVALON_PIO_DATA(DIN_PIO_BASE);
#else
	bButtonInputs_l++;
#endif

#ifdef DOUT_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(DOUT_PIO_BASE, bLedOutputs_l);
#endif

#ifdef LED_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_BASE, ~wDigitalOutputs_l);
#endif

    for (nIdx = 0; nIdx < 15; nIdx++)
    {
        abVirtualInputs_l[nIdx] = (~abVirtualOutputs_l[nIdx])-1;
    }
    abVirtualInputs_l[15] = (~bButtonInputs_l)-1;
    abVirtualInputs_l[16] = (~bLedOutputs_l)-1;

    for (nIdx = 0; nIdx < 8; nIdx++)
    {
        adwVirtualInputs_l[nIdx] = (~adwVirtualOutputs_l[nIdx])-1;
    }

    return EplRet;
}
