/**
********************************************************************************

  \file           EplApi.cpp

  \brief          Implementation of the EplApi class

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  (c) Bernecker + Rainer Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/
#define _WINSOCKAPI_ // prevent windows.h from including winsock.h

/******************************************************************************/
/* includes */
#include <QWidget>
#include <QThread>
#include <QString>
#include <QMessageBox>

#include "EplApi.h"
#include "EplState.h"
#include "EplInput.h"
#include "EplOutput.h"
#include "MainWindow.h"

#include <pcap.h>
#include <stdio.h>
#include <string.h>

#if (LINUX)
	#include <netinet/in.h>
	#include <net/if.h>
	#include <sys/socket.h>
	#include <sys/ioctl.h>
#endif

/******************************************************************************/
/* definitions */
#define NODEID      0xF0                // MN
#define IP_ADDR     0xc0a86401          // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00          // 255.255.255.0
#define HOSTNAME    "openPOWERLINK Stack    "
#define IF_ETH      EPL_VETH_NAME

#define CYCLE_LEN   5000                /* org val 5000 */

/******************************************************************************/
/* global variables */
CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char * pszCdcFilename_g = "mnobd.cdc";

/******************************************************************************/
/* class declarations */

/******************************************************************************/
/* function declarations */

/*
This function is the entry point for your object dictionary. It is defined
in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
this function prototype here. If you want to use more than one Epl
instances then the function name of each object dictionary has to differ.
*/
extern "C" tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);

/**
********************************************************************************
\brief	Constructor

Constructs a POWERLINK object.

\param		pMainWindow_p	        pointer to main window
\param          uiNodeId_p              node ID of the POWERLINK node
\param          devName_p               device name of the network interface
*******************************************************************************/
EplApi::EplApi(MainWindow *pMainWindow_p, unsigned int uiNodeId_p, QString devName_p)
{
    const char*         sHostname = HOSTNAME;
    tEplKernel          EplRet;
    EplState*           pEplState;
    EplOutput*          pEplOutput;
    EplInput*           pEplInput;
    EplCnState*         pEplCnState;
    char                devName[256];

    pEplState = pMainWindow_p->getEplStateWidget();
    pEplOutput = pMainWindow_p->getEplOutputWidget();
    pEplInput = pMainWindow_p->getEplInputWidget();
    pEplCnState = pMainWindow_p->getEplCnStateWidget();

    pEplProcessThread = new EplProcessThread;
    QObject::connect(pEplProcessThread, SIGNAL(eplStatusChanged(int)),
                     pEplState, SLOT(setEplStatusLed(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nmtStateChanged(const QString&)),
                     pEplState, SLOT(setNmtStateText(const QString &)));

    QObject::connect(pEplProcessThread, SIGNAL(nodeAppeared(int)),
                     pEplInput, SLOT(addNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(allNodesRemoved()),
                     pEplInput, SLOT(removeAllNodes()));
    QObject::connect(pEplProcessThread, SIGNAL(nodeDisappeared(int)),
                     pEplInput, SLOT(removeNode(int)));

    QObject::connect(pEplProcessThread, SIGNAL(nodeAppeared(int)),
                     pEplOutput, SLOT(addNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nodeDisappeared(int)),
                     pEplOutput, SLOT(removeNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(allNodesRemoved()),
                     pEplOutput, SLOT(removeAllNodes()));

    QObject::connect(pEplProcessThread, SIGNAL(nodeAppeared(int)),
                     pEplCnState, SLOT(addNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nodeDisappeared(int)),
                     pEplCnState, SLOT(removeNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(allNodesRemoved()),
                     pEplCnState, SLOT(removeAllNodes()));

    QObject::connect(pEplProcessThread, SIGNAL(nodeStatusChanged(int, int)),
                     pEplCnState, SLOT(setState(int, int)));

    pEplDataInOutThread = new EplDataInOutThread;
    QObject::connect(pEplDataInOutThread, SIGNAL(processImageOutChanged(unsigned int, unsigned int)),
                     pEplOutput, SLOT(setValue(unsigned int, unsigned int)));
    QObject::connect(pEplDataInOutThread, SIGNAL(processImageInChanged(unsigned int, unsigned int)),
                     pEplInput, SLOT(setLeds(unsigned int, unsigned int)));

    EPL_MEMSET(&EplApiInitParam, 0, sizeof (EplApiInitParam));
    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);


    EplApiInitParam.m_uiNodeId = uiNodeId_p;
    EplApiInitParam.m_dwIpAddress = (IP_ADDR & 0xFFFFFF00) | EplApiInitParam.m_uiNodeId;

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_dwFeatureFlags = -1;
    EplApiInitParam.m_dwCycleLen = CYCLE_LEN;           // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 256;       // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 256;       // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;         // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36;       // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36;       // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;        // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;            // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;                // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;                  // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
    EplApiInitParam.m_dwWaitSocPreq = 150000;
    EplApiInitParam.m_dwDeviceType = -1;                // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = -1;                  // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = -1;               // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = -1;            // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = -1;              // NMT_IdentityObject_REC.SerialNo_U32

    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    EplApiInitParam.m_uiSyncNodeId = EPL_C_ADR_SYNC_ON_SOA;
    EplApiInitParam.m_fSyncOnPrcNode = FALSE;

    // set callback functions
    EplApiInitParam.m_pfnCbEvent = pEplProcessThread->getEventCbFunc();

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));

#ifdef CONFIG_POWERLINK_USERSTACK
    EplApiInitParam.m_HwParam.m_pszDevName = devName_p.toAscii().data();
    EplApiInitParam.m_pfnObdInitRam = EplObdInitRam;
    EplApiInitParam.m_pfnCbSync = pEplDataInOutThread->getSyncCbFunc();
#else
    // Sync call back function not required for init from user space
    EplApiInitParam.m_pfnCbSync = NULL;
#endif

    // init EPL
    EplRet = EplApiInitialize(&EplApiInitParam);
    if(EplRet != kEplSuccessful)
    {
        printf("%s: EplApiInitialize() failed\n", __FUNCTION__);

        QMessageBox::critical(0, "POWERLINK demo",
                              QString("Initialization of openPOWERLINK Stack failed.\n") +
                              "Error code: 0x"+ QString::number(EplRet, 16) +
                              "\nThe most common error source are an unsupported Ethernet controller or the kernel module is not loaded."
                              "\nFor further information please consult the manual.");
        goto Exit;
    }

#ifdef CONFIG_POWERLINK_USERSTACK
    EplRet = EplApiSetCdcFilename(pszCdcFilename_g);
    if(EplRet != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    EplRet = pEplDataInOutThread->SetupProcessImage();
    if (EplRet != kEplSuccessful)
    {
        printf("%s: pEplDataInOutThread->SetupProcessImage() failed\n", __FUNCTION__);
        goto Exit;
    }
    printf("Setup Process Image Successfull\n");

    // start the EPL stack
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);

    // start process thread
    pEplProcessThread->start();

#ifndef CONFIG_POWERLINK_USERSTACK
    // start data in out thread
    pEplDataInOutThread->start();
#endif


Exit:
    printf("%s: returns 0x%X\n", __FUNCTION__, EplRet);

}

/**
********************************************************************************
\brief	Destructor

Destructs a POWERLINK object.
*******************************************************************************/
EplApi::~EplApi()
{
    tEplKernel          EplRet;

    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);
    pEplProcessThread->waitForNmtStateOff();
    EplRet = EplApiProcessImageFree();
    EplRet = EplApiShutdown();
}

/**
********************************************************************************
\brief  Get default node ID

Returns the default Node ID.

\return         Default node ID
*******************************************************************************/
unsigned int EplApi::defaultNodeId()

{
    return NODEID;
}

