/**
********************************************************************************
\file   Api.cpp

\brief  openPOWERLINK API class

This file implements the openPOWERLINK API class.
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <QWidget>
#include <QThread>
#include <QString>
#include <QMessageBox>

#include "Api.h"
#include "State.h"
#include "Input.h"
#include "Output.h"
#include "MainWindow.h"

#include <stdio.h>
#include <string.h>
#include <limits.h>

#if (TARGET_SYSTEM == _LINUX_)
    #include <netinet/in.h>
    #include <net/if.h>
    #include <sys/socket.h>
    #include <sys/ioctl.h>
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

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NODEID            0xF0                // MN
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define IF_ETH            PLK_VETH_NAME

#define CYCLE_LEN   5000                /* org val 5000 */

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char*    pszCdcFilename_g = (char*)"mnobd.cdc";
static char     devName_g[256];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a POWERLINK Api object.

\param      pMainWindow_p           Pointer to main window
\param      nodeId_p                Node ID of the POWERLINK node
\param      devName_p               Device name of the network interface
*/
//------------------------------------------------------------------------------
Api::Api(MainWindow* pMainWindow_p, UINT nodeId_p, QString devName_p)
{
    tOplkError          ret;
    State*              pState;
    Output*             pOutput;
    Input*              pInput;
    CnState*            pCnState;

    pState = pMainWindow_p->getStateWidget();
    pOutput = pMainWindow_p->getOutputWidget();
    pInput = pMainWindow_p->getInputWidget();
    pCnState = pMainWindow_p->getCnStateWidget();

    pProcessThread = new ProcessThread(pMainWindow_p);
    QObject::connect(pProcessThread, SIGNAL(oplkStatusChanged(int)),
                     pState, SLOT(setStatusLed(int)));
    QObject::connect(pProcessThread, SIGNAL(nmtStateChanged(const QString&)),
                     pState, SLOT(setNmtStateText(const QString &)));

    QObject::connect(pProcessThread, SIGNAL(nodeAppeared(int)),
                     pInput, SLOT(addNode(int)));
    QObject::connect(pProcessThread, SIGNAL(allNodesRemoved()),
                     pInput, SLOT(removeAllNodes()));
    QObject::connect(pProcessThread, SIGNAL(nodeDisappeared(int)),
                     pInput, SLOT(removeNode(int)));

    QObject::connect(pProcessThread, SIGNAL(nodeAppeared(int)),
                     pOutput, SLOT(addNode(int)));
    QObject::connect(pProcessThread, SIGNAL(nodeDisappeared(int)),
                     pOutput, SLOT(removeNode(int)));
    QObject::connect(pProcessThread, SIGNAL(allNodesRemoved()),
                     pOutput, SLOT(removeAllNodes()));

    QObject::connect(pProcessThread, SIGNAL(nodeAppeared(int)),
                     pCnState, SLOT(addNode(int)));
    QObject::connect(pProcessThread, SIGNAL(nodeDisappeared(int)),
                     pCnState, SLOT(removeNode(int)));
    QObject::connect(pProcessThread, SIGNAL(allNodesRemoved()),
                     pCnState, SLOT(removeAllNodes()));

    QObject::connect(pProcessThread, SIGNAL(nodeStatusChanged(int, int)),
                     pCnState, SLOT(setState(int, int)));

    QObject::connect(pProcessThread, SIGNAL(printLog(const QString&)),
                     pMainWindow_p, SLOT(printlog(const QString&)));


    pDataInOutThread = new DataInOutThread;
    QObject::connect(pDataInOutThread, SIGNAL(processImageOutChanged(int, int)),
                     pOutput, SLOT(setValue(int, int)));
    QObject::connect(pDataInOutThread, SIGNAL(processImageInChanged(int, int)),
                     pInput, SLOT(setLeds(int, int)));

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    initParam.nodeId = nodeId_p;
    initParam.ipAddress = (IP_ADDR & 0xFFFFFF00) | initParam.nodeId;

    initParam.fAsyncOnly = FALSE;
    initParam.featureFlags = UINT_MAX;
    initParam.cycleLen = CYCLE_LEN;           // required for error detection
    initParam.isochrTxMaxPayload = 256;       // const
    initParam.isochrRxMaxPayload = 256;       // const
    initParam.presMaxLatency = 50000;         // const; only required for IdentRes
    initParam.preqActPayloadLimit = 36;       // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit = 36;       // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency = 150000;        // const; only required for IdentRes
    initParam.multiplCylceCnt = 0;            // required for error detection
    initParam.asyncMtu = 1500;                // required to set up max frame size
    initParam.prescaler = 2;                  // required for sync
    initParam.lossOfFrameTolerance = 500000;
    initParam.asyncSlotTimeout = 3000000;
    initParam.waitSocPreq = 150000;
    initParam.deviceType = UINT_MAX;          // NMT_DeviceType_U32
    initParam.vendorId = UINT_MAX;            // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber = UINT_MAX;      // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber = UINT_MAX;        // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask = SUBNET_MASK;
    initParam.defaultGateway = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode = FALSE;

    // set callback functions
    initParam.pfnCbEvent = pProcessThread->getEventCbFunc();

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, abMacAddr, sizeof(initParam.aMacAddress));

    // Copy the selected interface string to a local variable
    strcpy(devName_g, devName_p.toStdString().c_str());
    initParam.hwParam.pDevName = devName_g;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.pfnCbSync = pDataInOutThread->getSyncCbFunc();
#else
    initParam.pfnCbSync  =    NULL;
#endif

    // init POWERLINK
    ret = oplk_init(&initParam);
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0, "POWERLINK demo",
                              QString("Initialization of openPOWERLINK Stack failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) +
                                      "\nThe most common error source are an unsupported Ethernet controller or the kernel module is not loaded."
                                      "\nFor further information please consult the manual.");
        goto Exit;
    }

    ret = oplk_setCdcFilename(pszCdcFilename_g);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    ret = pDataInOutThread->setupProcessImage();
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0, "POWERLINK demo",
                              QString("Initialization of process image failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16));
        goto Exit;
    }
    // start the openPOWERLINK stack
    ret = oplk_execNmtCommand(kNmtEventSwReset);

    // start process thread
    pProcessThread->start();

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)
    // start data in out thread
    pDataInOutThread->start();
#endif

Exit:
    return;

}

/**
********************************************************************************
\brief  Destructor

Destructs a POWERLINK object.
*******************************************************************************/
Api::~Api()
{
    tOplkError          ret;

    ret = oplk_execNmtCommand(kNmtEventSwitchOff);
    pProcessThread->waitForNmtStateOff();
    ret = oplk_freeProcessImage();
    ret = oplk_shutdown();
}

/**
********************************************************************************
\brief  Get default node ID

Returns the default Node ID.

\return         Default node ID
*******************************************************************************/
UINT Api::defaultNodeId()
{
    return NODEID;
}

