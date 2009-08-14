#include "Epl.h"
#include "alt_types.h"
#include <sys/alt_cache.h>
#define     u8      alt_u8
#define     u16     alt_u16
#define     u32     alt_u32

#define NODEID      0x01 // should be NOT 0xF0 (=MN) in case of CN
#define CYCLE_LEN   1000 // [us]
#define MAC_ADDR	0x00, 0x12, 0x34, 0x56, 0x78, 0x9A
#define IP_ADDR     0xc0a86401  // 192.168.100.1 // don't care the last byte!
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0

tEplKernel PUBLIC AppCbSync(void);
tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p);

	u8  					VarIn1 = 0; // process var for test purpose
    
    BOOL                    shutdown = FALSE;

int openPowerlink(void);

int main(void) {
    int i=0;
    
    alt_icache_flush_all();
    alt_dcache_flush_all();
    
    printf("NIOS II is running...\n");
    printf("starting openPowerlink application...\n\n");
    while(1) {
        if(openPowerlink() != 0) {
            printf("openPowerlink was shut down because of an error\n");
            break;
        } else {
            printf("openPowerlink was shut down, restart...\n\n");
        }
        for(i=0; i<1000000; i++);
    }
    printf("shut down NIOS II...\n%c", 4);
    
    return 0;
}


int openPowerlink(void) {
	u32		 				ip = IP_ADDR, // ip address
                            i = 0;
	
	const u8 				abMacAddr[] = {MAC_ADDR};
	static tEplApiInitParam EplApiInitParam; //epl init parameter
	// needed for process var
	tEplObdSize         	ObdSize;
	tEplKernel 				EplRet;
	unsigned int			uiVarEntries;
    
    shutdown = FALSE;
	
	////////////////////////
	// setup th EPL Stack //
	////////////////////////
	
	// calc the IP address with the nodeid
	ip &= 0xFFFFFF00; //dump the last byte
	ip |= NODEID; // and mask it with the node id

	// set EPL init parameters
	EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);
	EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof(EplApiInitParam.m_abMacAddress));
	EplApiInitParam.m_uiNodeId = NODEID; // defined at the top of this file!
	EplApiInitParam.m_dwIpAddress = ip;
	EplApiInitParam.m_uiIsochrTxMaxPayload = 100;
	EplApiInitParam.m_uiIsochrRxMaxPayload = 100;
	EplApiInitParam.m_dwPresMaxLatency = 50000;
	EplApiInitParam.m_dwAsndMaxLatency = 150000;
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
	EplApiInitParam.m_dwWaitSocPreq = 150000;
	EplApiInitParam.m_dwDeviceType = -1;
	EplApiInitParam.m_dwVendorId = -1;
	EplApiInitParam.m_dwProductCode = -1;
	EplApiInitParam.m_dwRevisionNumber = -1;
	EplApiInitParam.m_dwSerialNumber = -1;
	EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
	EplApiInitParam.m_dwDefaultGateway = 0;
	EplApiInitParam.m_pfnCbEvent = AppCbEvent;	EplApiInitParam.m_pfnCbSyncProcess = AppCbSync;
    
    // the ethernet driver needs to know the CN's node id
    //  so let him know!!!
    set_NodeID(NODEID);

	// initialize EPL stack
    printf("init EPL Stack:\n");
	EplRet = EplApiInitialize(&EplApiInitParam);
	if(EplRet != kEplSuccessful) {
        printf("init EPL Stack... error\n\n");
		return 10;
    }
    printf("init EPL Stack...ok\n\n");

	// link process variables used by CN to object dictionary
    printf("linkin process vars:\n");
    ObdSize = sizeof(VarIn1);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6000, &VarIn1, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful) {
        printf("linkin process vars... error\n\n");
	    return 20;
    }
	printf("linkin process vars... ok\n\n");
    
	// start the EPL stack
    printf("start EPL Stack...\n");
	EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful) {
        printf("start EPL Stack... error\n\n");
	    return 30;
    }
    printf("start EPL Stack... ok\n\n");
    
    printf("NIOS II with openPowerlink is ready!\n\n");
    
	while(1) {
        asyncCall();
        if(shutdown == TRUE)
            break;
    }
    printf("Shutdown EPL Stack\n");
    EplApiShutdown(); //shutdown node
	
	return 0;
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
                    // because of user signal (CTRL-C) or critical EPL stack error
                    // -> also shut down EplApiProcess() and main()
                    EplRet = kEplShutdown;
                    shutdown = TRUE;
                    break;
                }

                case kEplNmtGsResetCommunication:
                case kEplNmtGsResetConfiguration:
                case kEplNmtMsPreOperational1:
                    break;

                case kEplNmtGsInitialising:
                    break;
                case kEplNmtGsResetApplication:
                    break;
                case kEplNmtMsNotActive:
                    break;
                case kEplNmtCsNotActive:
                    break;
                case kEplNmtCsPreOperational1:
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
        {   // error or warning occured within the stack or the application
            // on error the API layer stops the NMT state machine
            printf("CriticalError/Warning ");
            // check additional argument
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {   // error occured within event processing
                    // either in kernel or in user part
                    printf("in Event Processing (kernel / user)\n");
                    break;
                }

                case kEplEventSourceDllk:
                {   // error occured within the data link layer (e.g. interrupt processing)
                    // the DWORD argument contains the DLL state and the NMT event
                    printf("in DLL kernel\n");
                    break;
                }

                default:
                {
                    printf("\n");
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
	tEplKernel          EplRet = kEplSuccessful;

	VarIn1++;

    return EplRet;
}
