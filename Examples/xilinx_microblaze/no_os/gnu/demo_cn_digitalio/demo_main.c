/****************************************************************************

Copyright (c) 2009, B&R
Copyright (c) 2009, SYSTEC electronic GmbH
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer
      in the documentation and/or other materials provided with the distribution.
    * Neither the name of the B&R nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  -------------------------------------------------------------------------

                $RCSfile: demo_main.c,v $

                $Author: Josef Baumgartner $

                $Revision: 1.2 $  $Date: 2009/09/28 14:38:37 $

                $State: Exp $

                Build Environment:
                    ...

  -------------------------------------------------------------------------

  Revision History:

  2006/06/06    k.t.: Start of Implementation

****************************************************************************/
/* includes */
#include "Epl.h"

#include "xparameters.h"
#include "xgpio_l.h"
#include "IRQ_Microblaze.h"
#include "Benchmark.h"

/******************************************************************************/
/* defines */

#define NODEID      0x01 // should be NOT 0xF0 (=MN) in case of CN
#define CYCLE_LEN   1000 // [us]
#define MAC_ADDR	0x00, 0x12, 0x34, 0x56, 0x78, NODEID
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

BYTE		digitalIn[4];
BYTE		digitalOut[4];

static BOOL     fShutdown_l = FALSE;

int openPowerlink(void);

int main(void) 
{
	int iCnt=0;
	
	#if XPAR_MICROBLAZE_0_USE_ICACHE
		#warning "ICACHE is used!"
		microblaze_init_icache_range(0, XPAR_MICROBLAZE_0_CACHE_BYTE_SIZE);
		microblaze_enable_icache();
	#endif
	
	#if XPAR_MICROBLAZE_0_USE_DCACHE
		#warning "DCACHE is used!"
		microblaze_init_dcache_range(0, XPAR_MICROBLAZE_0_DCACHE_BYTE_SIZE);
		microblaze_enable_dcache();
	#endif
	
	PRINTF0("MICROBLAZE is running...\n");
	
	PRINTF0("starting openPowerlink application...\n");
	while(1) {
		if(openPowerlink() != 0) {
			PRINTF0("openPowerlink was shut down because of an error\n");
			break;
		} else {
			PRINTF0("openPowerlink was shut down, restart...\n\n");
		}
			for(iCnt=0; iCnt<1000000; iCnt++)
				asm("NOP ;");
		}
		PRINTF1("shut down MICROBLAZE...\n%c", 4);
	
	#if XPAR_MICROBLAZE_0_USE_DCACHE
		microblaze_disable_dcache();
		microblaze_init_dcache_range(0, XPAR_MICROBLAZE_0_DCACHE_BYTE_SIZE);
	#endif
	
	#if XPAR_MICROBLAZE_0_USE_ICACHE
		microblaze_disable_icache();
		microblaze_init_icache_range(0, XPAR_MICROBLAZE_0_CACHE_BYTE_SIZE);
	#endif
	
	return 0;
}


int openPowerlink(void)
{
	DWORD		 				ip = IP_ADDR; // ip address

	const BYTE 				abMacAddr[] = {MAC_ADDR};
	static tEplApiInitParam EplApiInitParam; //epl init parameter
	// needed for process var
	tEplObdSize         	ObdSize;
	tEplKernel 				EplRet;
	unsigned int			uiVarEntries;

    fShutdown_l = FALSE;
    
    //Init Interrupt Controller
    {
    	XIntc *Intc = getIntc(); //get Interrupt Controller Instance
    	XStatus Status = 0;
    	PRINTF0("Init Xilinx Interrupt Controller:\n");
    	//init device
		Status = XIntc_Initialize(Intc, XPAR_XPS_INTC_0_DEVICE_ID);
		if (Status != XST_SUCCESS)
			return -1;
		
		//set options
		Status = XIntc_SetOptions(Intc, XIN_SVC_ALL_ISRS_OPTION);
		if (Status != XST_SUCCESS)
			return -1;
		PRINTF0("done!\n");
    }
    
#ifdef XPAR_LEDS_8BIT_BASEADDR
    //Set USER LEDs (8bit)
    PRINTF0("set User LEDs (8bit)\n");
    XGpio_WriteReg(XPAR_LEDS_8BIT_BASEADDR, XGPIO_TRI_OFFSET, 0);
#endif

#ifdef XPAR_POWERLINK_LED_BASEADDR
    XGpio_WriteReg(XPAR_POWERLINK_LED_BASEADDR, XGPIO_TRI_OFFSET, 0);
#endif

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
	EplApiInitParam.m_dwLossOfFrameTolerance = 5000000;
	EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
	EplApiInitParam.m_dwWaitSocPreq = 0;
	EplApiInitParam.m_dwDeviceType = -1;
	EplApiInitParam.m_dwVendorId = -1;
	EplApiInitParam.m_dwProductCode = -1;
	EplApiInitParam.m_dwRevisionNumber = -1;
	EplApiInitParam.m_dwSerialNumber = -1;
	EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
	EplApiInitParam.m_dwDefaultGateway = 0;
	EplApiInitParam.m_pfnCbEvent = AppCbEvent;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;

	// initialize EPL stack
    PRINTF0("init EPL Stack:\n");
	EplRet = EplApiInitialize(&EplApiInitParam);
	if(EplRet != kEplSuccessful) {
        PRINTF1("init EPL Stack... error %X\n\n", EplRet);
		goto Exit;
    }
    PRINTF0("init EPL Stack...ok\n\n");

	// link process variables used by CN to object dictionary
    PRINTF0("linking process vars:\n");
    ObdSize = sizeof(digitalIn[0]);
    uiVarEntries = 4;
    EplRet = EplApiLinkObject(0x6000, digitalIn, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

    ObdSize = sizeof(digitalOut[0]);
    uiVarEntries = 4;
    EplRet = EplApiLinkObject(0x6200, digitalOut, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        printf("linking process vars... error\n\n");
        goto ExitShutdown;
    }

	PRINTF0("linking process vars... ok\n\n");
	

	// start the EPL stack
    PRINTF0("start EPL Stack...\n");
	EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful) {
        PRINTF0("start EPL Stack... error\n\n");
        goto ExitShutdown;
    }
    PRINTF0("start EPL Stack... ok\n\n");

    PRINTF0("MICROBLAZE with openPowerlink is ready!\n\n");

    while(1)
    {
        EplApiProcess();
        if (fShutdown_l == TRUE)
            break;
    }

ExitShutdown:
    PRINTF0("Shutdown EPL Stack\n");
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
	static DWORD eplLeds = 0;

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
				
				case kEplNmtCsReadyToOperate:
					PRINTF0("enable operate\n");
					break;
				
                case kEplNmtCsOperational:
                	PRINTF0("ready 2 operate\n");
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
#ifdef XPAR_POWERLINK_LED_BASEADDR
                case kEplLedTypeStatus:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        eplLeds |= 2;
                        XGpio_WriteReg(XPAR_POWERLINK_LED_BASEADDR, XGPIO_DATA_OFFSET, eplLeds);
                    }
                    else
                    {
                        eplLeds &= ~2;
                        XGpio_WriteReg(XPAR_POWERLINK_LED_BASEADDR, XGPIO_DATA_OFFSET, eplLeds);
                    }
                    break;
                }
                case kEplLedTypeError:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        eplLeds |= 1;
                        XGpio_WriteReg(XPAR_POWERLINK_LED_BASEADDR, XGPIO_DATA_OFFSET, eplLeds);
                    }
                    else
                    {
                        eplLeds &= ~1;
                        XGpio_WriteReg(XPAR_POWERLINK_LED_BASEADDR, XGPIO_DATA_OFFSET, eplLeds);
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

#ifdef XPAR_PUSH_BUTTONS_3BIT_BASEADDR
    digitalIn[0] = XGpio_ReadReg(XPAR_PUSH_BUTTONS_3BIT_BASEADDR, XGPIO_DATA_OFFSET);
#endif

#ifdef XPAR_LEDS_OUTPUT_BASEADDR
    XGpio_WriteReg(XPAR_LEDS_OUTPUT_BASEADDR, XGPIO_DATA_OFFSET, digitalOut[0]);
#endif

    return EplRet;
}
