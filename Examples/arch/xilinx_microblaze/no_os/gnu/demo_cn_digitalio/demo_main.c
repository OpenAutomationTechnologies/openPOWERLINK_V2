/**
********************************************************************************
\file       demo_main.c

\brief      Main module of the directIO user example

Application of the directIO example which starts the openPOWERLINK stack and
implements AppCbSync and AppCbEvent.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include "Epl.h"

#ifdef __NIOS2__
#include <unistd.h>
#elif defined(__MICROBLAZE__)
#include "xilinx_usleep.h"
#endif

#include "systemComponents.h"

#ifdef LCD_BASE
#include "Cmp_Lcd.h"
#endif


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
// this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.
tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);

tEplKernel PUBLIC AppCbSync(void);
tEplKernel PUBLIC AppCbEvent(
    tEplApiEventType        EventType_p,
    tEplApiEventArg*        pEventArg_p,
    void GENERIC*           pUserArg_p);

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NODEID      0x01    ///< This node id is overwritten when the dip switches are != 0!
                            ///< Additionally this should be NOT 0xF0 (=MN) in case of CN

#define CYCLE_LEN   1000        ///< lenght of the cycle [us]
#define MAC_ADDR    0x00, 0x12, 0x34, 0x56, 0x78, NODEID  ///< MAC address of the CN
#define IP_ADDR     0xc0a86401  ///< IP-Address 192.168.100.1 (don't care the last byte!)
#define SUBNET_MASK 0xFFFFFF00  ///< The subnet mask (255.255.255.0)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static BYTE        portIsOutput[4];
static BYTE        digitalIn[4];
static BYTE        digitalOut[4];
static BOOL        fShutdown_l = FALSE;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static int openPowerlink(BYTE bNodeId_p);
static void InitPortConfiguration (BYTE *p_portIsOutput);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief               Entry point of the program

main function of the directIO example which inits the peripheral reads the node
ID and calls openPowerlink.

\return              exist status
\retval              0                           returns 0 after successfull exit
*/
//------------------------------------------------------------------------------
int main (void)
{
    WORD    bNodeId;        ///< nodeID of the CN

    SysComp_initPeripheral();


#ifdef LCD_BASE
    SysComp_LcdTest();
#endif

    PRINTF("\n\nDigital I/O interface is running...\n");
    PRINTF("starting openPowerlink...\n\n");

    if((bNodeId = SysComp_getNodeId()) == 0)
    {
        bNodeId = NODEID;
    }

#ifdef LCD_BASE
    SysComp_LcdPrintNodeInfo(bNodeId);
#endif

    while (1)
    {
        if (openPowerlink(bNodeId) != 0)
        {
            PRINTF("openPowerlink was shut down because of an error\n");
            break;
        } else
        {
            PRINTF("openPowerlink was shut down, restart...\n\n");
        }
        /* wait some time until we restart the stack */
        usleep(1000000);
    }

    PRINTF("shut down processor...\n%c", 4);

    SysComp_freeProcessorCache();

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief               AppCbEvent

event callback function called by EPL API layer within user part (low priority).

\param               EventType_p                  event type
\param               pEventArg_p                  pointer to union, which describes
                                                  the event in detail
\param               pUserArg_p                   user specific argument

\return              tEplKernel
\retval              kEplSuccessful               no error
\retval              kEplReject                   reject further processing
\retval              otherwise                    post error event to API layer
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC AppCbEvent(tEplApiEventType EventType_p,
                             tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p)
{
    tEplKernel          EplRet = kEplSuccessful;
    BYTE                bPwlState;      ///< state of the CN (operational = high)

    /* check if NMT_GS_OFF is reached */
    switch (EventType_p)
    {
        case kEplApiEventNmtStateChange:
        {
#ifdef LCD_BASE
            SysComp_LcdPrintState(pEventArg_p->m_NmtStateChange.m_NewNmtState);
#endif

#ifdef LATCHED_IOPORT_CFG
            if (pEventArg_p->m_NmtStateChange.m_NewNmtState != kEplNmtCsOperational)
            {
                bPwlState = 0x0;
                memcpy(LATCHED_IOPORT_CFG+3,(BYTE *)&bPwlState,1);    // Set PortIO operational pin to low
            } else
            {
                /* reached operational state */
                bPwlState = 0x80;
                memcpy(LATCHED_IOPORT_CFG+3,(BYTE *)&bPwlState,1);    // Set PortIO operational pin to high
            }
#endif //LATCHED_IOPORT_CFG

            switch (pEventArg_p->m_NmtStateChange.m_NewNmtState)
            {
                case kEplNmtGsOff:
                {
                    /* NMT state machine was shut down,
                       because of critical EPL stack error
                       -> also shut down EplApiProcess() and main() */
                    EplRet = kEplShutdown;
                    fShutdown_l = TRUE;

                    PRINTF("%s(kEplNmtGsOff) originating event = 0x%X\n", __func__,
                            pEventArg_p->m_NmtStateChange.m_NmtEvent);
                    break;
                }

                case kEplNmtGsInitialising:
                case kEplNmtGsResetApplication:
                case kEplNmtGsResetConfiguration:
                case kEplNmtCsPreOperational1:
                case kEplNmtCsBasicEthernet:
                case kEplNmtMsBasicEthernet:
                case kEplNmtGsResetCommunication:
                {
                    PRINTF("%s(0x%X) originating event = 0x%X\n",
                            __func__,
                            pEventArg_p->m_NmtStateChange.m_NewNmtState,
                            pEventArg_p->m_NmtStateChange.m_NmtEvent);
                    break;
                }

                case kEplNmtMsNotActive:
                    break;
                case kEplNmtCsNotActive:
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
        {
            /* set error LED */
#ifdef STATUS_LEDS_BASE
            SysComp_setPowerlinkStatus(0x2);
#endif
            /* fall through */
        }
        case kEplApiEventWarning:
        {
            /* error or warning occurred within the stack or the application
               on error the API layer stops the NMT state machine */
            PRINTF("%s(Err/Warn): Source=%02X EplError=0x%03X",
                    __func__,
                    pEventArg_p->m_InternalError.m_EventSource,
                    pEventArg_p->m_InternalError.m_EplError);
            /* check additional argument */
            switch (pEventArg_p->m_InternalError.m_EventSource)
            {
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                {
                    /* error occurred within event processing
                       either in kernel or in user part */
                    PRINTF(" OrgSource=%02X\n", pEventArg_p->m_InternalError.m_Arg.m_EventSource);
                    break;
                }

                case kEplEventSourceDllk:
                {
                    /* error occurred within the data link layer (e.g. interrupt processing)
                       the DWORD argument contains the DLL state and the NMT event */
                    PRINTF(" val=%lX\n", pEventArg_p->m_InternalError.m_Arg.m_dwArg);
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
        {
            /* new history entry */
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
        {
            /* status or error LED shall be changed */
#ifdef STATUS_LEDS_BASE
            switch (pEventArg_p->m_Led.m_LedType)
            {
                case kEplLedTypeStatus:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        SysComp_resetPowerlinkStatus(0x1);
                    }
                    else
                    {
                        SysComp_setPowerlinkStatus(0x1);
                    }
                    break;
                }
                case kEplLedTypeError:
                {
                    if (pEventArg_p->m_Led.m_fOn != FALSE)
                    {
                        SysComp_resetPowerlinkStatus(0x2);
                    }
                    else
                    {
                        SysComp_setPowerlinkStatus(0x2);
                    }
                    break;
                }
                default:
                    break;
            }
#endif
            break;
        }

        case kEplApiEventUserDef:
        {
            break;
        }

        default:
            break;
    }

    return EplRet;
}

//------------------------------------------------------------------------------
/**
\brief               AppCbSync

sync event callback function called by event module within kernel part
(high priority).
This function sets the outputs, reads the inputs and runs the control loop.

\return              tEplKernel
\retval              kEplSuccessful              no error
\retval              otherwise                   post error event to API layer
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC AppCbSync(void)
{
    tEplKernel        EplRet = kEplSuccessful;
    register int      iCnt;
    DWORD             ports; ///< 4 byte input or output ports
    DWORD*            ulDigInputs = LATCHED_IOPORT_BASE;
    DWORD*            ulDigOutputs = LATCHED_IOPORT_BASE;

    /* read digital input ports */
    ports = AmiGetDwordFromLe((BYTE*) ulDigInputs);;

    for (iCnt = 0; iCnt <= 3; iCnt++)
    {
        if (portIsOutput[iCnt])
        {
            /* configured as output -> overwrite invalid input values with RPDO mapped variables */
            ports = (ports & ~(0xff << (iCnt * 8))) | (digitalOut[iCnt] << (iCnt * 8));
        }
        else
        {
            /* configured as input -> store in TPDO mapped variable */
            digitalIn[iCnt] = (ports >> (iCnt * 8)) & 0xff;
        }
    }

    /* write digital output ports */
    AmiSetDwordToLe((BYTE*)ulDigOutputs, ports);

    return EplRet;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief               openPOWERLINK function

Init the openPOWERLINK stack, link digitalIn/Out objects and perform a reset
communcation command.

\param               bNodeId_p                   NodeID of the CN

\return              tEplKernel
\retval              kEplSuccessful              successfull exit
\retval              otherwise                   post error event and exit
*/
//------------------------------------------------------------------------------
static int openPowerlink(BYTE bNodeId_p)
{
    DWORD                       ip = IP_ADDR;          ///< ip address
    const BYTE                  abMacAddr[] = {MAC_ADDR};
    static tEplApiInitParam     EplApiInitParam;       ///< epl init parameter
    tEplObdSize                 ObdSize;               ///< needed for process var
    tEplKernel                  EplRet;
    unsigned int                uiVarEntries;

    fShutdown_l = FALSE;

    /* initialize port configuration */
    InitPortConfiguration(portIsOutput);

    /* setup the POWERLINK stack */

    /* calc the IP address with the nodeid */
    ip &= 0xFFFFFF00; //dump the last byte
    ip |= bNodeId_p; // and mask it with the node id

    /* set EPL init parameters */
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof(EplApiInitParam.m_abMacAddress));
    EplApiInitParam.m_abMacAddress[5] = bNodeId_p;
    EplApiInitParam.m_uiNodeId = bNodeId_p;
    EplApiInitParam.m_dwIpAddress = ip;
    EplApiInitParam.m_uiIsochrTxMaxPayload = 36;
    EplApiInitParam.m_uiIsochrRxMaxPayload = 1490;
    EplApiInitParam.m_dwPresMaxLatency = 2000;
    EplApiInitParam.m_dwAsndMaxLatency = 2000;
    EplApiInitParam.m_fAsyncOnly = FALSE;
    EplApiInitParam.m_dwFeatureFlags = -1;
    EplApiInitParam.m_dwCycleLen = CYCLE_LEN;
    EplApiInitParam.m_uiPreqActPayloadLimit = 36;
    EplApiInitParam.m_uiPresActPayloadLimit = 36;
    EplApiInitParam.m_uiMultiplCycleCnt = 0;
    EplApiInitParam.m_uiAsyncMtu = 300;
    EplApiInitParam.m_uiPrescaler = 2;
    EplApiInitParam.m_dwLossOfFrameTolerance = 100000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
    EplApiInitParam.m_dwWaitSocPreq = 0;
    EplApiInitParam.m_dwDeviceType = -1;
    EplApiInitParam.m_dwVendorId = -1;
    EplApiInitParam.m_dwProductCode = -1;
    EplApiInitParam.m_dwRevisionNumber = -1;
    EplApiInitParam.m_dwSerialNumber = -1;
    EplApiInitParam.m_dwApplicationSwDate = 0;
    EplApiInitParam.m_dwApplicationSwTime = 0;
    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EplApiInitParam.m_pfnCbEvent = AppCbEvent;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;

    PRINTF("\nNode ID is set to: %d\n", EplApiInitParam.m_uiNodeId);

    /* initialize POWERLINK stack */
    PRINTF("init POWERLINK stack:\n");
    EplRet = EplApiInitialize(&EplApiInitParam);
    if(EplRet != kEplSuccessful)
    {
        PRINTF("init POWERLINK Stack... error 0x%X\n\n", EplRet);
        goto Exit;
    }
    PRINTF("init POWERLINK Stack...ok\n\n");

    /* link process variables used by CN to object dictionary */
    PRINTF("linking process vars:\n");

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

    PRINTF("linking process vars... ok\n\n");

    /* start the POWERLINK stack */
    PRINTF("start EPL Stack...\n");
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);
    if (EplRet != kEplSuccessful)
    {
        PRINTF("start EPL Stack... error\n\n");
        goto ExitShutdown;
    }

    /* Start POWERLINK Stack */
    PRINTF("start POWERLINK Stack... ok\n\n");

    PRINTF("Digital I/O interface with openPowerlink is ready!\n\n");

#ifdef STATUS_LEDS_BASE
    SysComp_setPowerlinkStatus(0xff);
#endif

    SysComp_enableInterrupts();

    while(1)
    {
        EplApiProcess();
        if (fShutdown_l == TRUE)
        {
            break;
        }
    }

ExitShutdown:
    PRINTF("Shutdown EPL Stack\n");
    EplApiShutdown();       // shutdown node

Exit:
    return EplRet;
}

//------------------------------------------------------------------------------
/**
\brief               openPOWERLINK function

InitPortConfiguration() reads the port configuration inputs. The port configuration
inputs are connected to general purpose I/O pins IO3V3[16..12].
The read port configuration if stored at the port configuration outputs to set up
the input/output selection logic.

\param               p_portIsOutput              direction of port IO
*/
//------------------------------------------------------------------------------
static void InitPortConfiguration (BYTE *p_portIsOutput)
{
    register int     iCnt;
    volatile BYTE    portconf;          ///< direction of each byte in portio
    unsigned int     direction = 0;

    /* read port configuration input pins */
    memcpy((BYTE *) &portconf, LATCHED_IOPORT_CFG, 1);
    portconf = (~portconf) & 0x0f;

    PRINTF("\nPort configuration register value = 0x%1X\n", portconf);

    for (iCnt = 0; iCnt <= 3; iCnt++)
    {
        if (portconf & (1 << iCnt))
        {
            direction |= 0xff << (iCnt * 8);
            p_portIsOutput[iCnt] = TRUE;
        }
        else
        {
            direction &= ~(0xff << (iCnt * 8));
            p_portIsOutput[iCnt] = FALSE;
        }
    }
}

