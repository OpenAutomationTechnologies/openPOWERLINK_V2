/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.de

  Project:      EPL Messe Demo

  Description:  demoapplication for EPL

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2008/04/11 m.u.:   start of implementation

****************************************************************************/

#include <QWidget>
#include <QThread>
#include <QString>
#include <QMessageBox>

#include "EplApi.h"
#include "EplState.h"
#include "MainWindow.h"

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define NODEID      0xF0    // MN
#define IP_ADDR     0xc0a86401  // 192.168.100.1
#define SUBNET_MASK 0xFFFFFF00  // 255.255.255.0
#define HOSTNAME    "SYS TEC electronic EPL Stack    "
#define IF_ETH      EPL_VETH_NAME

#define CYCLE_LEN   5000 /* org val 5000 */

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------

CONST BYTE abMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          E p l A p i   C L A S S                                        //
//                                                                         //
//=========================================================================//

EplApi::EplApi(MainWindow *pMainWindow_p, unsigned int uiNodeId_p)
{
const char*         sHostname = HOSTNAME;
tEplKernel          EplRet;
//tEplObdSize         ObdSize;
//unsigned int        uiVarEntries;
EplState*           pEplState;
Slides*             pSlides;

    pEplState = pMainWindow_p->getEplStateWidget();
    pSlides   = pMainWindow_p->getSlidesWidget();

    pEplProcessThread = new EplProcessThread;
    QObject::connect(pEplProcessThread, SIGNAL(eplStatusChanged(int)),
                     pEplState, SLOT(setEplStatusLed(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nmtStateChanged(const QString&)),
                     pEplState, SLOT(setNmtStateText(const QString &)));
    QObject::connect(pEplProcessThread, SIGNAL(nodeAppeared(int)),
                     pEplState, SLOT(addNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nodeDisappeared(int)),
                     pEplState, SLOT(removeNode(int)));
    QObject::connect(pEplProcessThread, SIGNAL(nodeStatusChanged(int, int)),
                     pEplState, SLOT(setNodeStatus(int, int)));
    QObject::connect(pEplProcessThread, SIGNAL(allNodesRemoved()),
                     pEplState, SLOT(removeAllNodes()));

    pEplDataInOutThread = new EplDataInOutThread;
    QObject::connect(pEplDataInOutThread, SIGNAL(processImageOutChanged(unsigned int)),
                     pMainWindow_p, SLOT(setLcd(unsigned int)));
    QObject::connect(pEplDataInOutThread, SIGNAL(processImageInChanged(unsigned int)),
                     pEplState, SLOT(setLeds(unsigned int)));

    EPL_MEMSET(&EplApiInitParam, 0, sizeof (EplApiInitParam));

    EplApiInitParam.m_uiNodeId = uiNodeId_p;
    EplApiInitParam.m_dwIpAddress = (IP_ADDR & 0xFFFFFF00) | EplApiInitParam.m_uiNodeId;

    EplApiInitParam.m_fAsyncOnly = FALSE;

    EplApiInitParam.m_uiSizeOfStruct = sizeof (EplApiInitParam);

    EPL_MEMCPY(EplApiInitParam.m_abMacAddress, abMacAddr, sizeof (EplApiInitParam.m_abMacAddress));

//    EplApiInitParam.m_abMacAddress[5] = (BYTE) EplApiInitParam.m_uiNodeId;
    EplApiInitParam.m_dwFeatureFlags = -1;
    EplApiInitParam.m_dwCycleLen = CYCLE_LEN;     // required for error detection
    EplApiInitParam.m_uiIsochrTxMaxPayload = 100; // const
    EplApiInitParam.m_uiIsochrRxMaxPayload = 100; // const
    EplApiInitParam.m_dwPresMaxLatency = 50000;  // const; only required for IdentRes
    EplApiInitParam.m_uiPreqActPayloadLimit = 36; // required for initialisation (+28 bytes)
    EplApiInitParam.m_uiPresActPayloadLimit = 36; // required for initialisation of Pres frame (+28 bytes)
    EplApiInitParam.m_dwAsndMaxLatency = 150000;   // const; only required for IdentRes
    EplApiInitParam.m_uiMultiplCycleCnt = 0;  // required for error detection
    EplApiInitParam.m_uiAsyncMtu = 1500;         // required to set up max frame size
    EplApiInitParam.m_uiPrescaler = 2;         // required for sync
    EplApiInitParam.m_dwLossOfFrameTolerance = 500000;
    EplApiInitParam.m_dwAsyncSlotTimeout = 3000000;
    EplApiInitParam.m_dwWaitSocPreq = 150000;
    EplApiInitParam.m_dwDeviceType = -1;              // NMT_DeviceType_U32
    EplApiInitParam.m_dwVendorId = -1;                // NMT_IdentityObject_REC.VendorId_U32
    EplApiInitParam.m_dwProductCode = -1;             // NMT_IdentityObject_REC.ProductCode_U32
    EplApiInitParam.m_dwRevisionNumber = -1;          // NMT_IdentityObject_REC.RevisionNo_U32
    EplApiInitParam.m_dwSerialNumber = -1;            // NMT_IdentityObject_REC.SerialNo_U32
    //EplApiInitParam.m_qwVendorSpecificExt1;
    //EplApiInitParam.m_dwVerifyConfigurationDate; // CFM_VerifyConfiguration_REC.ConfDate_U32
    //EplApiInitParam.m_dwVerifyConfigurationTime; // CFM_VerifyConfiguration_REC.ConfTime_U32
    //EplApiInitParam.m_dwApplicationSwDate;       // PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    //EplApiInitParam.m_dwApplicationSwTime;       // PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    EplApiInitParam.m_dwSubnetMask = SUBNET_MASK;
    EplApiInitParam.m_dwDefaultGateway = 0;
    EPL_MEMCPY(EplApiInitParam.m_sHostname, sHostname, sizeof(EplApiInitParam.m_sHostname));
    //EplApiInitParam.m_abVendorSpecificExt2[48];

    EplApiInitParam.m_pfnCbEvent = pEplProcessThread->getEventCbFunc();
    // Sync call back function not required for init from user space
    EplApiInitParam.m_pfnCbSync = NULL;

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

    EplRet = EplApiProcessImageSetup();
    if (EplRet != kEplSuccessful)
    {
        printf("%s: EplApiProcessImageSetup() failed\n", __FUNCTION__);
        goto Exit;
    }
    printf("Setup Process Image Successfull\n");

/*
    // link process variables to OD
    ObdSize = sizeof(bVarIn_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6000, &bVarIn_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    ObdSize = sizeof(bVarOut_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6200, &bVarOut_l, &uiVarEntries, &ObdSize, 0x01);
    if (EplRet != kEplSuccessful)
    {
        goto Exit;
    }

    // link a DOMAIN to object 0x6100, but do not exit, if it is missing
    ObdSize = sizeof(abDomain_l);
    uiVarEntries = 1;
    EplRet = EplApiLinkObject(0x6100, &abDomain_l, &uiVarEntries, &ObdSize, 0x00);
    if (EplRet != kEplSuccessful)
    {
        printf("EplApiDefineObject(0x6100): returns 0x%X\n", EplRet);
    }

    // reset old process variables
    bVarInOld_l = 0;
    bVarOutOld_l = 0;
*/
    // start the EPL stack
    EplRet = EplApiExecNmtCommand(kEplNmtEventSwReset);

    // start process thread
    pEplProcessThread->start();

    // start data in out thread
    pEplDataInOutThread->start();


Exit:
    printf("%s: returns 0x%X\n", __FUNCTION__, EplRet);

}

EplApi::~EplApi()
{
tEplKernel          EplRet;

    EplRet = EplApiExecNmtCommand(kEplNmtEventSwitchOff);

    printf("wait for Off...\n");
    pEplProcessThread->waitForNmtStateOff();
    printf("reached Off\n");

    EplRet = EplApiShutdown();
    printf("EplApiShutdown():  0x%X\n", EplRet);

}


unsigned int EplApi::defaultNodeId()
{
    return NODEID;
}


