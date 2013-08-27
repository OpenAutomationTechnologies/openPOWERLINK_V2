/**
********************************************************************************
\file   demo_main.c

\brief  Demo application for Altera FPGA MN

This is a demo application for Altera FPGA MN.

\ingroup module_demo
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <sys/alt_cache.h>

#include <Epl.h>

#include "xap.h"

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
void initEvents (BOOL* pfGsOff_p);
tEplKernel PUBLIC processEvents(tEplApiEventType EventType_p, tEplApiEventArg* pEventArg_p, void GENERIC* pUserArg_p);

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NODEID      0xF0                //=> MN
#define CYCLE_LEN   UINT_MAX
#define IP_ADDR     0xc0a86401          // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00          // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "
#define CHECK_KERNEL_TIMEOUT    100000
const BYTE abMacAddr[] = {0x00, 0x12, 0x34, 0x56, 0x78, NODEID};

#define DEFAULT_MAX_CYCLE_COUNT 20      // 6 is very fast
#define APP_LED_COUNT_1         8       // number of LEDs for CN1
#define APP_LED_MASK_1          (1 << (APP_LED_COUNT_1 - 1))
#define MAX_NODES               255

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static unsigned int uiNodeId_g = EPL_C_ADR_INVALID;
static unsigned int uiCycleLen_g = CYCLE_LEN;
static BOOL fShutdown = FALSE;

static PI_IN* pProcessImageIn_l;
static PI_OUT* pProcessImageOut_l;

static unsigned char aCdcBuffer[] =
{
    #include "mnobd.txt"
};

typedef struct
{
    unsigned int            m_uiLeds;
    unsigned int            m_uiLedsOld;
    unsigned int            m_uiInput;
    unsigned int            m_uiInputOld;
    unsigned int            m_uiPeriod;
    int                     m_iToggle;
} APP_NODE_VAR_T;

static int                  iUsedNodeIds_g[] =
{
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12,
        0

};
static unsigned int         uiCnt_g;
static APP_NODE_VAR_T       nodeVar_g[MAX_NODES];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);
tEplKernel PUBLIC AppCbSync(void);
tEplKernel PUBLIC AppInit(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Main function

Calls the initializes and run POWERLINK and application

\return 0

\ingroup module_demo
*/
//------------------------------------------------------------------------------
int  main (void)
{
    tEplKernel                  EplRet = kEplSuccessful;
    static tEplApiInitParam     EplApiInitParam;
    char*                       sHostname = HOSTNAME;
    int                         checkStack = 0;

    alt_icache_flush_all();
    alt_dcache_flush_all();

    printf("----------------------------------------------------\n");
    printf("openPOWERLINK FPGA MN DEMO application\n");
    printf("----------------------------------------------------\n");

    EPL_MEMSET(&EplApiInitParam, 0, sizeof (EplApiInitParam));
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);

    EplApiInitParam.m_uiNodeId = uiNodeId_g = NODEID;
    EplApiInitParam.m_dwIpAddress = (0xFFFFFF00 & IP_ADDR) | EplApiInitParam.m_uiNodeId;

    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_dwFeatureFlags            = -1;
    EplApiInitParam.m_dwCycleLen                = uiCycleLen_g;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload      = 256;              // const
    EplApiInitParam.m_uiIsochrRxMaxPayload      = 256;              // const
    EplApiInitParam.m_dwPresMaxLatency          = 50000;            // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit     = 36;               // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit     = 36;               // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency          = 150000;           // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt         = 0;                // required for error detection
    EplApiInitParam.m_uiAsyncMtu                = 1500;             // required to set up max frame size
    EplApiInitParam.m_uiPrescaler               = 2;                // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance    = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout        = 3000000;
    EplApiInitParam.m_dwWaitSocPreq             = -1;
    EplApiInitParam.m_dwDeviceType              = -1;               // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId                = -1;               // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode             = -1;               // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber          = -1;               // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber            = -1;               // NMT_IdentityObject_REC.SerialNo_U32

    EplApiInitParam.m_dwSubnetMask              = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway          = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    EplApiInitParam.m_uiSyncNodeId              = EPL_C_ADR_SYNC_ON_SOA;
    EplApiInitParam.m_fSyncOnPrcNode            = FALSE;

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = processEvents;
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
    EplApiInitParam.m_pfnCbSync  = AppCbSync;


    printf("\n\nHello, I'm a POWERLINK node running as %s!\n  (build: %s / %s)\n\n",
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

    EplRet = EplApiSetCdcBuffer(aCdcBuffer, sizeof(aCdcBuffer));
    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    printf("Initializing process image...\n");
    printf("Size of input process image: %ld\n", sizeof(PI_IN));
    printf("Size of output process image: %ld\n", sizeof (PI_OUT));
    EplRet = api_processImageAlloc(sizeof(PI_IN), sizeof(PI_OUT));
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    pProcessImageIn_l = api_processImageGetInputImage();
    pProcessImageOut_l = api_processImageGetOutputImage();

    EplRet = EplApiProcessImageSetup();
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // start processing
    EplRet = EplApiExecNmtCommand(kNmtEventSwReset);
    if (EplRet != kEplSuccessful)
    {
        goto ExitShutdown;
    }

    initEvents(&fShutdown);

    while(!fShutdown)
    {
        if((EplRet = EplApiProcess()) != kEplSuccessful)
            goto ExitShutdown;

        if(checkStack++ >= CHECK_KERNEL_TIMEOUT)
        {
            checkStack = 0;
            if(!api_checkKernelStack())
            {
                printf("Kernel is dead!\n");
                fShutdown = TRUE;
            }
        }
    }

ExitShutdown:
    // halt the NMT state machine
    // so the processing of POWERLINK frames stops
    EplRet = EplApiExecNmtCommand(kNmtEventSwitchOff);

    // delete process image
    EplRet = api_processImageFree();

    // delete instance for all modules
    EplRet = EplApiShutdown();

Exit:
    PRINTF("main(): returns 0x%X\n", EplRet);

    return 0;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//------------------------------------------------------------------------------
/**
\brief    Application initialization

This function is called to initialize the application.

\return The function returns a tEplKernel error code.

\ingroup module_demo
*/
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
/**
\brief    Application synchronization callback

This function is called in case of a synchronization event. It sets the outputs,
reads the inputs and runs the control loop.

\return The function returns a tEplKernel error code.

\ingroup module_demo
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC AppCbSync(void)
{
    tEplKernel          EplRet;
    int                 i;

    EplRet = api_processImageExchangeOut();
    if (EplRet != kEplSuccessful)
    {
        return EplRet;
    }

    uiCnt_g++;

    nodeVar_g[0].m_uiInput = pProcessImageOut_l->CN1_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[1].m_uiInput = pProcessImageOut_l->CN2_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[2].m_uiInput = pProcessImageOut_l->CN3_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[3].m_uiInput = pProcessImageOut_l->CN4_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[4].m_uiInput = pProcessImageOut_l->CN5_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[5].m_uiInput = pProcessImageOut_l->CN6_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[6].m_uiInput = pProcessImageOut_l->CN7_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[7].m_uiInput = pProcessImageOut_l->CN8_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[8].m_uiInput = pProcessImageOut_l->CN9_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[9].m_uiInput = pProcessImageOut_l->CN10_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[10].m_uiInput = pProcessImageOut_l->CN11_M00_Digital_Input_8_Bit_Byte_1;
    nodeVar_g[11].m_uiInput = pProcessImageOut_l->CN12_M00_Digital_Input_8_Bit_Byte_1;

    for (i = 0; (i < MAX_NODES) && (iUsedNodeIds_g[i] != 0); i++)
    {
        /* Running Leds */
        /* period for LED flashing determined by inputs */
        nodeVar_g[i].m_uiPeriod = (nodeVar_g[i].m_uiInput == 0) ? 20 : (nodeVar_g[i].m_uiInput * 20);
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

    pProcessImageIn_l->CN1_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[0].m_uiLeds;
    pProcessImageIn_l->CN2_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[1].m_uiLeds;
    pProcessImageIn_l->CN3_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[2].m_uiLeds;
    pProcessImageIn_l->CN4_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[3].m_uiLeds;
    pProcessImageIn_l->CN5_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[4].m_uiLeds;
    pProcessImageIn_l->CN6_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[5].m_uiLeds;
    pProcessImageIn_l->CN7_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[6].m_uiLeds;
    pProcessImageIn_l->CN8_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[7].m_uiLeds;
    pProcessImageIn_l->CN9_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[8].m_uiLeds;
    pProcessImageIn_l->CN10_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[9].m_uiLeds;
    pProcessImageIn_l->CN11_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[10].m_uiLeds;
    pProcessImageIn_l->CN12_M00_Digital_Ouput_8_Bit_Byte_1 = nodeVar_g[11].m_uiLeds;

    EplRet = api_processImageExchangeIn();

    return EplRet;
}
