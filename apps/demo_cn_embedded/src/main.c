/**
********************************************************************************
\file   main.c

\brief  Main file of embedded CN demo application

This file contains the main file of the openPOWERLINK CN embedded demo
application.

\ingroup module_demo_cn_embedded
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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

#include <gpio.h>
#include <lcd.h>
#include <arp.h>

#include "app.h"
#include "event.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN         1000        ///< lenght of the cycle [us]
#define NODEID            1           ///< This node id is overwritten when the dip switches are != 0!
#define IP_ADDR           0xc0a86401  ///< 192.168.100.1
#define SUBNET_MASK       0xFFFFFF00  ///< 255.255.255.0
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define MAC_ADDR          0x00, 0x12, 0x34, 0x56, 0x78, NODEID

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct sInstace
{
    UINT8   aMacAddr[6];    ///< Mac address
    UINT8   nodeId;         ///< Node ID
    UINT32  cycleLen;       ///< Cycle length
    BOOL    fShutdown;      ///< User flag to shutdown the stack
    BOOL    fGsOff;         ///< NMT State GsOff reached
} tInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initPowerlink(tInstance* pInstance_p);
static tOplkError loopMain(tInstance* pInstance_p);
static void shutdownPowerlink(tInstance* pInstance_p);
static tOplkError eventCbPowerlink(tOplkApiEventType eventType_p, tOplkApiEventArg* pEventArg_p, void* pUserArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  main function

This is the main function of the openPOWERLINK embedded CN demo application.

\return Returns an exit code

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
int main(void)
{
    tOplkError  ret = kErrorOk;
    const UINT8 aMacAddr[] = {MAC_ADDR};
    UINT8       nodeid;

    // Initialize helper modules
    gpio_init();
    lcd_init();

    // get node ID from input
    nodeid = gpio_getNodeid();

    // initialize instance
    memset(&instance_l, 0, sizeof(instance_l));

    instance_l.cycleLen     = CYCLE_LEN;
    instance_l.nodeId       = (nodeid != 0) ? nodeid : NODEID;
    instance_l.fShutdown    = FALSE;
    instance_l.fGsOff       = FALSE;

    // set mac address (last byte is set to node ID)
    memcpy(instance_l.aMacAddr, aMacAddr, sizeof(aMacAddr));
    instance_l.aMacAddr[5]  = instance_l.nodeId;

    initEvents(&eventCbPowerlink);
    arp_init((UINT8)instance_l.nodeId);

    PRINTF("----------------------------------------------------\n");
    PRINTF("openPOWERLINK embedded CN DEMO application\n");
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
    arp_exit();
    shutdownApp();
    shutdownPowerlink(&instance_l);

    // Shutdown helper modules
    lcd_exit();
    gpio_exit();

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
    initParam.isochrTxMaxPayload      = 36;                     // const
    initParam.isochrRxMaxPayload      = 36;                     // const
    initParam.presMaxLatency          = 2000;                   // const; only required for IdentRes
    initParam.asndMaxLatency          = 2000;                   // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 36;                     // required for initialization (+28 bytes)
    initParam.presActPayloadLimit     = 36;                     // required for initialization of Pres frame (+28 bytes)
    initParam.multiplCylceCnt         = 0;                      // required for error detection
    initParam.asyncMtu                = 300;                    // required to set up max frame size
    initParam.prescaler               = 2;                      // required for sync
    initParam.lossOfFrameTolerance    = 100000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 0;
    initParam.deviceType              = -1;               // NMT_DeviceType_U32
    initParam.vendorId                = -1;               // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = -1;               // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = -1;               // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = -1;               // NMT_IdentityObject_REC.SerialNo_U32
    initParam.applicationSwDate       = 0;
    initParam.applicationSwTime       = 0;
    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOC;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = processEvents;
    initParam.pfnCbSync  = processSync;
    initParam.pfnSdoSrvProcessObdWrite = obdal_processWrite;
    initParam.pfnSdoSrvProcessObdRead = obdal_processRead;

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
    tOplkError ret = kErrorOk;

    // start processing
    if ((ret = oplk_execNmtCommand(kNmtEventSwReset)) != kErrorOk)
        return ret;

    while (1)
    {
        // do background tasks
        if ((ret = oplk_process()) != kErrorOk)
            break;

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

\ingroup module_demo_cn_embedded
*/
//------------------------------------------------------------------------------
static tOplkError eventCbPowerlink(tOplkApiEventType eventType_p,
                                   tOplkApiEventArg* pEventArg_p, void* pUserArg_p)
{
    tOplkError                      ret = kErrorOk;
    tOplkApiEventReceivedNonPlk*    pFrameInfo = &pEventArg_p->receivedEth;

    UNUSED_PARAMETER(pUserArg_p);

    switch (eventType_p)
    {
        case kOplkApiEventNmtStateChange:
            lcd_printNmtState(pEventArg_p->nmtStateChange.newNmtState);

            switch (pEventArg_p->nmtStateChange.newNmtState)
            {
                case kNmtGsOff:
                    // NMT state machine was shut down
                    ret = kErrorShutdown;

                    // NMT off state is reached
                    instance_l.fGsOff = TRUE;
                    break;

                case kNmtCsBasicEthernet:
                    // ARP demo: Send request to MN
                    arp_sendRequest((0xFFFFFF00 & IP_ADDR) | C_ADR_MN_DEF_NODE_ID);
                    break;

                case kNmtCsPreOperational2:
                     // automatic change to kEplNmtCsReadyToOperate can be
                     // prevented with:
                     // ret = kErrorReject;
                     // As soon as application is ready for OPERATIONAL state,
                     // execute the following function call:
                     // oplk_execNmtCommand(kNmtEventEnterReadyToOperate);
                     break;

                default:
                    break;
            }
            break;

        case kOplkApiEventReceivedNonPlk:
            if (arp_processReceive(pFrameInfo->pFrame, pFrameInfo->frameSize) == 0)
                return kErrorOk;

            // If you get here, the received Ethernet frame is no ARP frame.
            // Here you can call other protocol stacks for processing.

            ret = kErrorOk; // Frame wasn't processed, so simply dump it.
            break;

        case kOplkApiEventDefaultGwChange:
            // ARP demo: Set default gateway and send request
            arp_setDefGateway(pEventArg_p->defaultGwChange.defaultGateway);

            arp_sendRequest(pEventArg_p->defaultGwChange.defaultGateway);
            break;

        default:
            break;
    }
    return ret;
}

/// \}
