/**
********************************************************************************
\file   main.c

\brief  Main file of console MN demo application

This file contains the main file of the openPOWERLINK MN console demo
application.

\ingroup module_demo_mn_embedded
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2015, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <gpio/gpio.h>
#include <lcd/lcd.h>
#include <arp/arp.h>
#include <system/system.h>

#include "app.h"
#include "event.h"

#if (CONFIG_CDC_ON_SD != FALSE)
#include <sdcard/sdcard.h>
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN           -1
#define NODEID              0xF0                    //=> MN
#define IP_ADDR             0xc0a86401              // 192.168.100.1
#define SUBNET_MASK         0xFFFFFF00              // 255.255.255.0
#define DEFAULT_GATEWAY     0xC0A864FE              // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define MAC_ADDR            0x00, 0x12, 0x34, 0x56, 0x78, NODEID

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (CONFIG_CDC_ON_SD != FALSE)
static char*            pszCdcFilename_g = "mnobd.cdc";

#else
const unsigned char     aCdcBuffer[] =
{
    #include "mnobd_char.txt"
};

#endif
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct sInstace
{
    UINT8           aMacAddr[6];    ///< Mac address
    UINT8           nodeId;         ///< Node ID
    UINT32          cycleLen;       ///< Cycle length
    BOOL            fShutdown;      ///< User flag to shutdown the stack
    BOOL            fGsOff;         ///< NMT State GsOff reached
    unsigned char*  pCdcBuffer;     ///< Pointer to CDC buffer
    UINT            cdcBufferSize;  ///< Size of CDC buffer
} tInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   initPowerlink(tInstance* pInstance_p);
static tOplkError   loopMain(tInstance* pInstance_p);
static void         shutdownPowerlink(tInstance* pInstance_p);
static tOplkError   eventCbPowerlink(tOplkApiEventType eventType_p,
                                     tOplkApiEventArg* pEventArg_p, void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  main function

This is the main function of the openPOWERLINK console MN demo application.

\return Returns an exit code

\ingroup module_demo_mn_embedded
*/
//------------------------------------------------------------------------------
int main(void)
{
    tOplkError      ret = kErrorOk;
    const UINT8     aMacAddr[] = {MAC_ADDR};
    UINT8           nodeid;

#if (CONFIG_CDC_ON_SD != FALSE)
    tCdcBuffInfo    cdcBuffInfo;
#endif

    // Initialize helper modules
    system_init();
    gpio_init();
    lcd_init();

    // get node ID from input
    nodeid = gpio_getNodeid();

    // initialize instance
    memset(&instance_l, 0, sizeof(instance_l));

    instance_l.cycleLen         = CYCLE_LEN;
    instance_l.nodeId           = (nodeid != 0) ? nodeid : NODEID;
    instance_l.fShutdown        = FALSE;
    instance_l.fGsOff           = FALSE;
#if (CONFIG_CDC_ON_SD != FALSE)
    if (sdcard_getCdcOnSd(pszCdcFilename_g, &cdcBuffInfo) != 0)
    {
        goto Exit;
    }
    instance_l.pCdcBuffer       = (unsigned char*)cdcBuffInfo.pCdcBuffer;
    instance_l.cdcBufferSize    = cdcBuffInfo.cdcSize;
#else
    instance_l.pCdcBuffer       = (unsigned char*)aCdcBuffer;
    instance_l.cdcBufferSize    = sizeof(aCdcBuffer);
#endif
    // set mac address (last byte is set to node ID)
    memcpy(instance_l.aMacAddr, aMacAddr, sizeof(aMacAddr));
    instance_l.aMacAddr[5] = instance_l.nodeId;

    initEvents(&instance_l.fGsOff, &eventCbPowerlink);
    arp_init((UINT8)instance_l.nodeId);

    PRINTF("----------------------------------------------------\n");
    PRINTF("openPOWERLINK embedded MN DEMO application\n");
    PRINTF("using openPOWERLINK Stack: %s\n", oplk_getVersionString());
    PRINTF("----------------------------------------------------\n");

    PRINTF("NODEID=0x%02X\n", instance_l.nodeId);
    lcd_printNodeId(instance_l.nodeId);

    if ((ret = initPowerlink(&instance_l)) != kErrorOk)
        goto Exit;

    if ((ret = initApp()) != kErrorOk)
        goto Exit;

    if ((ret = oplk_setNonPlkForward(TRUE)) != kErrorOk)
    {
        PRINTF("WARNING: oplk_setNonPlkForward() failed with \"%s\"\n(Error:0x%x!)\n",
               debugstr_getRetValStr(ret), ret);
    }

    loopMain(&instance_l);

Exit:
#if (CONFIG_CDC_ON_SD != FALSE)
    sdcard_freeCdcBuffer(&cdcBuffInfo);
#endif
    arp_exit();
    shutdownApp();
    shutdownPowerlink(&instance_l);

    // Shutdown helper modules
    lcd_exit();
    gpio_exit();
    system_exit();

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack.

\param  pInstance_p             Pointer to demo instance

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initPowerlink(tInstance* pInstance_p)
{
    tOplkError                  ret = kErrorOk;
    static tOplkApiInitParam    initParam;

    PRINTF("Initializing openPOWERLINK stack...\n");

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    initParam.nodeId = pInstance_p->nodeId;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    memcpy(initParam.aMacAddress, pInstance_p->aMacAddr, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = -1;
    initParam.cycleLen                = pInstance_p->cycleLen;  // required for error detection
    initParam.isochrTxMaxPayload      = 256;                    // const
    initParam.isochrRxMaxPayload      = 256;                    // const
    initParam.presMaxLatency          = 50000;                  // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 36;                     // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 36;                     // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 150000;                 // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                      // required for error detection
    initParam.asyncMtu                = 1500;                   // required to set up max frame size
    initParam.prescaler               = 2;                      // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = -1;
    initParam.deviceType              = -1;                     // NMT_DeviceType_U32
    initParam.vendorId                = -1;                     // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = -1;                     // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = -1;                     // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = -1;                     // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOC;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = processEvents;
    initParam.pfnCbSync  = processSync;
#if defined(CONFIG_INCLUDE_SDOS)
    initParam.pfnSdoSrvProcessObdWrite = obdal_processWrite;
    initParam.pfnSdoSrvProcessObdRead = obdal_processRead;
#endif

    // initialize POWERLINK stack
    ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        PRINTF("oplk_initialize() failed with \"%s\"\n(Error:0x%x!)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    ret = oplk_create(&initParam);
    if (ret != kErrorOk)
    {
        PRINTF("oplk_create() failed with \"%s\"\n(Error:0x%x!)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    ret = oplk_setCdcBuffer(pInstance_p->pCdcBuffer, pInstance_p->cdcBufferSize);
    if (ret != kErrorOk)
    {
        PRINTF("oplk_setCdcBuffer() failed with \"%s\"\n(Error:0x%x!)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    // Set real MAC address to ARP module
    oplk_getEthMacAddr(initParam.aMacAddress);
    arp_setMacAddr(initParam.aMacAddress);

    // Set IP address to ARP module
    arp_setIpAddr(initParam.ipAddress);

    // Set default gateway to ARP module
    arp_setDefGateway(initParam.defaultGateway);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Main loop of demo application

This function implements the main loop of the demo application.
- It sends an NMT command to start the stack

\param  pInstance_p             Pointer to demo instance

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loopMain(tInstance* pInstance_p)
{
    tOplkError    ret = kErrorOk;

    // start processing
    if ((ret = oplk_execNmtCommand(kNmtEventSwReset)) != kErrorOk)
        return ret;

    while (1)
    {
        // do background tasks
        if ((ret = oplk_process()) != kErrorOk)
            break;

        if (oplk_checkKernelStack() == FALSE)
        {
            PRINTF("Kernel stack has gone! Exiting...\n");
            instance_l.fShutdown = TRUE;
        }

        // trigger switch off
        if (pInstance_p->fShutdown != FALSE)
        {
            oplk_execNmtCommand(kNmtEventSwitchOff);

            // reset shutdown flag to generate only one switch off command
            pInstance_p->fShutdown = FALSE;
        }

        // exit loop if NMT is in off state
        if (pInstance_p->fGsOff != FALSE)
            break;

        switch (gpio_getAppInput())
        {
            case 0x01:
                PRINTF("KEY0: SwReset\n");
                lcd_printText("KEY0: SwReset", 2);
                ret = oplk_execNmtCommand(kNmtEventSwReset);
                break;

            case 0x02:
                PRINTF("KEY1: SwitchOff\n");
                lcd_printText("KEY1: SwitchOff", 2);
                ret = oplk_execNmtCommand(kNmtEventSwitchOff);
                break;

            default:
                break;
        }

        while (gpio_getAppInput() != 0);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the demo application

The function shuts down the demo application.

\param  pInstance_p             Pointer to demo instance
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(tInstance* pInstance_p)
{
    UNUSED_PARAMETER(pInstance_p);

    PRINTF("Shut down DEMO\n");

    oplk_destroy();
    oplk_exit();
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK event callback

The function implements the applications stack event handler.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError eventCbPowerlink(tOplkApiEventType eventType_p,
                                   tOplkApiEventArg* pEventArg_p, void* pUserArg_p)
{
    tOplkError ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    // Handle ARP demo:
    // - If a node is found (e.g. plug in a CN), then an ARP request is sent
    //   to this node. There is no timeout handling done.
    // - If an Ethernet frame is received, it is forwarded to process reply.
    //   The function prints the ARP reply information.
    if (eventType_p == kOplkApiEventNode)
    {
        tOplkApiEventNode* pNode = &pEventArg_p->nodeEvent;

        // Note: This is a demonstration to generate non-POWERLINK frames.
        if (pNode->nodeEvent == kNmtNodeEventFound)
        {
            arp_sendRequest((0xFFFFFF00 & IP_ADDR) | pNode->nodeId);
        }
    }
    else if (eventType_p == kOplkApiEventReceivedNonPlk)
    {
        tOplkApiEventReceivedNonPlk* pFrameInfo = &pEventArg_p->receivedEth;

        // Note: This is a demonstration how to forward Ethernet frames to upper
        //       layers. Instead you can insert an IP stack and forward the
        //       pFrameInfo content to it. Frames that are sent from the IP
        //       stack to lower layers must be forwarded with the function
        //       \ref oplk_sendEthFrame.

        // Forward received frame to ARP processing
        if (arp_processReceive(pFrameInfo->pFrame, pFrameInfo->frameSize) == 0)
            return kErrorOk;

        // If you get here, the received Ethernet frame is no ARP frame.
        // Here you can call other protocol stacks for processing.

        ret = kErrorOk; // Frame wasn't processed, so simply dump it.
    }
    else if (eventType_p == kOplkApiEventDefaultGwChange)
    {
        // ARP demo: Set default gateway and send request
        arp_setDefGateway(pEventArg_p->defaultGwChange.defaultGateway);

        arp_sendRequest(pEventArg_p->defaultGwChange.defaultGateway);
    }

    return ret;
}

/// \}
