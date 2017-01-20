/**
********************************************************************************
\file   Api.cpp

\brief  openPOWERLINK API class

This file implements the openPOWERLINK API class.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#define _WINSOCKAPI_        // prevent windows.h from including winsock.h

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <Api.h>

#include <QMetaType>
#include <QMessageBox>

#include <MainWindow.h>
#include <EventLog.h>
#include <EventHandler.h>
#include <SyncEventHandler.h>
#include <NmtStateWidget.h>
#include <IoWidget.h>
#include <CnListWidget.h>

#include <oplk/debugstr.h>
#include <obdcreate/obdcreate.h>

#include <cstdio>
#include <cstring>

#if (TARGET_SYSTEM == _LINUX_)
#include <netinet/in.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#endif

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
const UINT32 Api::IP_ADDR = 0xc0a86400;         // 192.168.100.0
const UINT32 Api::SUBNET_MASK = 0xFFFFFF00;     // 255.255.255.0
const UINT32 Api::DEFAULT_GATEWAY = 0xC0A864FE; // 192.168.100.C_ADR_RT1_DEF_NODE_ID
const UINT32 Api::CYCLE_LEN = 5000;             // default cycle time
const UINT8  Api::aMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
Q_DECLARE_METATYPE(tSdoComFinished)
Q_DECLARE_METATYPE(tNmtState)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            S T A T I C   M E M B E R   F U N C T I O N S                   //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief   Get the openPOWERLINK stack version

\return  Returns the openPOWERLINK stack version
*/
//------------------------------------------------------------------------------
UINT32 Api::getVersion()
{
    return oplk_getVersion();
}

//------------------------------------------------------------------------------
/**
\brief   Execute an NMT command

\param[in]      nmtEvent_p          NMT event
*/
//------------------------------------------------------------------------------
void Api::execNmtCommand(tNmtEvent nmtEvent_p)
{
    oplk_execNmtCommand(nmtEvent_p);
}

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Constructor

Constructs a POWERLINK Api object.

\param[in,out]  pMainWindow_p       Pointer to main window
*/
//------------------------------------------------------------------------------
Api::Api(MainWindow* pMainWindow_p) :
    pCdcFilename("mnobd.cdc")
{
    IoWidget*   pOutput = pMainWindow_p->getOutputWidget();
    IoWidget*   pInput = pMainWindow_p->getInputWidget();

    // Register types in Qt
    qRegisterMetaType<tNmtState>("tNmtState");
    qRegisterMetaType<tSdoComFinished>("tSdoComFinished");

    // Event logger
    this->pEventLog = new EventLog();
    QObject::connect(this->pEventLog,
                     SIGNAL(printLog(const QString&)),
                     pMainWindow_p,
                     SLOT(printLogMessage(const QString&)));

    // Connect process thread
    this->pEventHandler = new EventHandler(this->pEventLog);
    QObject::connect(this->pEventHandler,
                     SIGNAL(nmtStateChanged(tNmtState)),
                     pMainWindow_p,
                     SLOT(nmtStateChanged(tNmtState)));
    QObject::connect(this->pEventHandler,
                     SIGNAL(nodeStatusChanged(unsigned int, tNmtState)),
                     pMainWindow_p,
                     SLOT(nodeNmtStateChanged(unsigned int, tNmtState)));

    // Connect other events
    QObject::connect(this->pEventHandler,
                     SIGNAL(userDefEvent(void*)),
                     this,
                     SIGNAL(userDefEvent(void*)),
                     Qt::DirectConnection);
    QObject::connect(this->pEventHandler,
                     SIGNAL(sdoFinished(tSdoComFinished)),
                     this,
                     SIGNAL(sdoFinished(tSdoComFinished)));

    // Connect sync event handler
    this->pSyncEventHandler = &SyncEventHandler::getInstance();
    QObject::connect(this->pSyncEventHandler,
                     SIGNAL(processImageOutChanged(unsigned int, unsigned int)),
                     pOutput,
                     SLOT(setValue(unsigned int, unsigned int)));
    QObject::connect(this->pSyncEventHandler,
                     SIGNAL(processImageInChanged(unsigned int, unsigned int)),
                     pInput,
                     SLOT(setValue(unsigned int, unsigned int)));
    QObject::connect(this->pSyncEventHandler,
                     SIGNAL(disableOutputs(unsigned int)),
                     pOutput,
                     SLOT(disableNode(unsigned int)));
    QObject::connect(this->pEventHandler,
                     SIGNAL(isMnActive(bool)),
                     this->pSyncEventHandler,
                     SLOT(setOperational(bool)));

    // Initialize the stack
    tOplkError ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("Initialization of openPOWERLINK stack failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"\n" +
                                      "For further information please consult the manual.");
    }
}

//------------------------------------------------------------------------------
/**
\brief  Destructor

Destructs a POWERLINK Api object.
*/
//------------------------------------------------------------------------------
Api::~Api()
{
    // Exit the stack API
    oplk_exit();

    // Cleanup
    delete this->pEventHandler;
    delete this->pEventLog;
}

//------------------------------------------------------------------------------
/**
\brief  Start stack

Starts the openPOWERLINK stack.

\param[in]      nodeId_p            Node ID of the POWERLINK node
\param[in]      devName_p           Reference to the device name of the network interface
*/
//------------------------------------------------------------------------------
void Api::start(unsigned int nodeId_p,
                const QString& devName_p)
{
    tOplkError  ret;

    std::memset(&this->initParam, 0, sizeof(this->initParam));
    this->initParam.sizeOfInitParam = sizeof(this->initParam);
    this->initParam.fAsyncOnly = FALSE;
    this->initParam.nodeId = nodeId_p;
    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    std::memcpy(this->initParam.aMacAddress, Api::aMacAddr, sizeof(this->initParam.aMacAddress));
    this->initParam.featureFlags = UINT_MAX;
    this->initParam.cycleLen = Api::CYCLE_LEN;      // required for error detection
    this->initParam.isochrTxMaxPayload = 1490;      // const
    this->initParam.isochrRxMaxPayload = 1490;      // const
    this->initParam.presMaxLatency = 150000;        // const; only required for IdentRes
    this->initParam.preqActPayloadLimit = 36;       // required for initialization (+28 bytes)
    this->initParam.presActPayloadLimit = 36;       // required for initialization of Pres frame (+28 bytes)
    this->initParam.asndMaxLatency = 150000;        // const; only required for IdentRes
    this->initParam.multiplCylceCnt = 0;            // required for error detection
    this->initParam.asyncMtu = 1500;                // required to set up max frame size
    this->initParam.prescaler = 2;                  // required for sync
    this->initParam.lossOfFrameTolerance = 500000;
    this->initParam.waitSocPreq = 1000;
    this->initParam.asyncSlotTimeout = 3000000;
    this->initParam.deviceType = UINT_MAX;          // NMT_DeviceType_U32
    this->initParam.vendorId = UINT_MAX;            // NMT_IdentityObject_REC.VendorId_U32
    this->initParam.productCode = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    this->initParam.revisionNumber = UINT_MAX;      // NMT_IdentityObject_REC.RevisionNo_U32
    this->initParam.serialNumber = UINT_MAX;        // NMT_IdentityObject_REC.SerialNo_U32

    this->initParam.ipAddress = (Api::IP_ADDR & Api::SUBNET_MASK) | this->initParam.nodeId;
    this->initParam.subnetMask = Api::SUBNET_MASK;
    this->initParam.defaultGateway = Api::DEFAULT_GATEWAY;
    std::sprintf((char*)this->initParam.sHostname,
                 "%02x-%08x",
                 this->initParam.nodeId,
                 this->initParam.vendorId);

    // set event and sync callback functions
    this->initParam.pfnCbEvent = EventHandler::appCbEvent;
    this->initParam.pEventUserArg = this->pEventHandler;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    this->initParam.pfnCbSync = SyncEventHandler::appCbSync;
#else
    this->initParam.pfnCbSync = NULL;
#endif

    // Copy the selected interface string to a local variable
    std::strcpy(this->devName, devName_p.toStdString().c_str());
    this->initParam.hwParam.pDevName = this->devName;

    this->initParam.syncNodeId = C_ADR_SYNC_ON_SOA;
    this->initParam.fSyncOnPrcNode = FALSE;

    // Initialize object dictionary
    ret = obdcreate_initObd(&this->initParam.obdInitParam);
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("Initialization of openPOWERLINK stack failed.\n") +
                                      "Error code: 0x" + QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"\n" +
                                      "For further information please consult the manual.");
        goto Exit;
    }

    ret = oplk_create(&this->initParam);
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("Creation of openPOWERLINK stack failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"\n" +
                                      "The most common error source are an unsupported Ethernet controller " +
                                      "or the kernel module is not loaded.\n" +
                                      "For further information please consult the manual.");
        goto Exit;
    }

    ret = oplk_setCdcFilename(Api::pCdcFilename);
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("oplk_setCdcFilename() failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"");
        goto Exit;
    }

    ret = this->pSyncEventHandler->setupProcessImage();
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("setupProcessImage() failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"");
        goto Exit;
    }

    // start the openPOWERLINK stack
    ret = oplk_execNmtCommand(kNmtEventSwReset);
    if (ret != kErrorOk)
    {
        QMessageBox::critical(0,
                              "POWERLINK demo",
                              QString("oplk_execNmtCommand() failed.\n") +
                                      "Error code: 0x"+ QString::number(ret, 16) + "\n" +
                                      "\"" + debugstr_getRetValStr(ret) + "\"");
        goto Exit;
    }

    // Start synchronous data handler
    this->pSyncEventHandler->setMinSyncPeriod(1000);
    this->pSyncEventHandler->start();

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Stop stack

Stops the openPOWERLINK stack.
*/
//------------------------------------------------------------------------------
void Api::stop()
{
    // Stop the sync event handler
    if (this->pSyncEventHandler->isRunning())
    {
        this->pSyncEventHandler->requestInterruption();
        this->pSyncEventHandler->wait(100);          // wait until thread terminates (max 100ms)
    }

    // Signal the stack to switch off
    Api::execNmtCommand(kNmtEventSwitchOff);
    // And wait until the stack has really shut down (reached state "NMT_GS_OFF")
    this->pEventHandler->awaitNmtGsOff();

    oplk_freeProcessImage();
    oplk_destroy();
}
