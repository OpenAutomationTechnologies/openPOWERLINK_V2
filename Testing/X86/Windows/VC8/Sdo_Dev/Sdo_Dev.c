/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      EPL

  Description:  source file for development of the SDO-Module

  -------------------------------------------------------------------------

                $RCSfile: Sdo_Dev.c,v $

                $Author: D.Krueger $

                $Revision: 1.10 $  $Date: 2010-06-11 14:06:19 $

                $State: Exp $

                Build Environment:
                KEIL uVision 2

  -------------------------------------------------------------------------

  Revision History:

  2006/06/26 k.t.:   start of the implementation

****************************************************************************/

#define _CRT_SECURE_NO_WARNINGS

#include <conio.h>
#include "EplSdo.h"
#include "EplAmi.h"
#include "kernel/EplObdk.h"
#include "kernel/EplNmtk.h"
#include "kernel/EplDllk.h"
#include "kernel/EplDllkCal.h"
#include "kernel/EplPdokCal.h"
#include "user/EplDlluCal.h"
#include "user/EplDllu.h"
#include "user/EplNmtCnu.h"
#include "user/EplSdoComu.h"
#include "user/EplTimeru.h"

#include <ctype.h>


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

#define NODEID  0xF0 //=> MN
// ip-address of the interface to use for the SDO Transfer
#define EPL_UDP_IP_ADDR         "0.0.0.0"   // was "192.168.100.233"
#define EPL_UDP_PORT            0           // was 2000
// node id of the target node for sdo client-test
#define EPL_UDP_TARGET_NODEID   1

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static CONST BYTE abMacAddr[] = {0x00, 0x02, 0xc0, 0xa8, 0x64, 0xf0}; // of Snd Shuttle Board

// variables for OD entry 0x2000 and 0x2200
static BYTE    bVarIn_l;
static WORD    wVarOut_l;

// HANDLE for event
static HANDLE  SdoReadyHdl_l;

// flag
static BOOL    fSdoSuccessful_l;

// strints to print out states of sdo transfer
static char aszSdoStates_l[6][40]={"Connection not active\n",
                                "Transfer running\n",
                                "TX Abort\n",
                                "RX Abort\n",
                                "Sdo Tranfer finished\n",
                                "Sdo Transfer aborted by lower layer\n"};

static BYTE    abTest_l[3000];

static BYTE    abDomain_l[2000];

static BYTE    bTargetNodeId_l;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
// This function is the entry point for your object dictionary. It is defined
// in OBJDICT.C by define EPL_OBD_INIT_RAM_NAME. Use this function name to define
// this function prototype here. If you want to use more than one Epl
// instances then the function name of each object dictionary has to differ.

tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);

static tEplKernel PUBLIC EplAppSdoConnectionCb(tEplSdoComFinished* pSdoComFinished_p);

static void EplAppPrintMenue(void);

static void EplAppDumpData(void*         pData_p,
                           unsigned long ulDataSize_p);

static tEplKernel PUBLIC EplAppCbAccessFinished(tEplObdParam* pObdParam_p);

static tEplKernel PUBLIC EplAppProcessEvent(
            tEplEvent* pEplEvent_p);

static tEplKernel PUBLIC EplAppCbDefaultObdAccess(tEplObdParam MEM* pParam_p);


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <xxxxx>                                             */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
int main(void)
{
tEplKernel                  Ret;
tEplObdInitParam            ObdInitParam;
tEplVarParam                VarParam;
int                         iChar;
tEplObdSize                 ObdSize;
char                        cBuffer[32];
tEplObdParam                ObdParam;

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)

unsigned long               ulIpAddr;

#endif


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)

tEplSdoComConHdl            SdoComConHdl = 0;
unsigned long               ulSize;
unsigned int                uiBuffer;
tEplSdoComTransParamByIndex TransParamByIndex;
unsigned int                uiCount;
BYTE                        bBuffer;

#endif


    //---------------------------------------------------------------------------
    // init of epl stack

    // init OD
    Ret = EplObdInitRam(&ObdInitParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // initialize EplObd-Module
    Ret = EplObdInit(&ObdInitParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplObdSetDefaultObdCallback(EplAppCbDefaultObdAccess);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // link process variables to OD
//    VarParam.m_pArg = NULL;
    VarParam.m_pData = &bVarIn_l;
    VarParam.m_Size = sizeof(bVarIn_l);
    VarParam.m_uiIndex = 0x2000;
    VarParam.m_uiSubindex = 0x01;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObdDefineVar(&VarParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

//    VarParam.m_pArg = NULL;
    VarParam.m_pData = &wVarOut_l;
    VarParam.m_Size = sizeof(wVarOut_l);
    VarParam.m_uiIndex = 0x2200;
    VarParam.m_uiSubindex = 0x01;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObdDefineVar(&VarParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // initialize EplEventu-Module
    Ret = EplEventuInit(EplAppProcessEvent);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // init EplTimeru-Module
    Ret = EplTimeruInit();
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // init SDO-Module
    // init sdo command layer
    Ret = EplSdoComInit();
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
    // config
    ulIpAddr = htonl(inet_addr(EPL_UDP_IP_ADDR));
    Ret = EplSdoUdpuConfig(ulIpAddr, EPL_UDP_PORT);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // init daomain entry
//    VarParam.m_pArg = NULL;
    VarParam.m_pData = &abDomain_l[0];
    VarParam.m_Size = sizeof(abDomain_l);
    VarParam.m_uiIndex = 0x2100;
    VarParam.m_uiSubindex = 0x00;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObduDefineVar(&VarParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //---------------------------------------------------------------------------
    // end of stack init


    //------------------------------------------------------------------------------
    // start testprogramm

    // init event
    SdoReadyHdl_l = CreateEvent(NULL,
                                FALSE,
                                FALSE,
                                NULL);
    if(SdoReadyHdl_l == NULL)
    {
        printf("Error during CreateEvent()\n");
        goto ShutDown;
    }

    fSdoSuccessful_l = FALSE;
    bTargetNodeId_l = EPL_UDP_TARGET_NODEID;

    // print out menue
    EplAppPrintMenue();
    // main loop for processing
    for (;;)
    {
        // check key hit
        if (_kbhit())
        {
            iChar = _getch();
        }
        else
        {
            iChar = 0;
        }

        switch(iChar)
        {
            // ESC
            case 0x1B:
            {
                printf("ShutDown\n\n");
                goto ShutDown;
            }
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
            // 1
            case '1':
            {
                // init command layer connection
                Ret = EplSdoComDefineCon(&SdoComConHdl,
                                            bTargetNodeId_l,  // target node id
                                            kEplSdoTypeUdp);    // udp-connection
                if(Ret != kEplSuccessful)
                {
                    printf("Define of SDO via UDP Connection failed: 0x%03X\n", Ret);
                }
                else
                {
                    printf("SDO via UDP Connection defined successfully\n");
                }
                break;
            }

            // 0
            case '0':
            {
                // close connection
                Ret = EplSdoComUndefineCon(SdoComConHdl);
                if(Ret != kEplSuccessful)
                {
                    printf("Close of SDO via UDP Connection failed: 0x%03X\n", Ret);
                }
                else
                {
                    printf("SDO via UDP Connection closed successfully\n");
                }

                break;
            }

            // 2
            case '2':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iVal3 = 0;
            int iVal4 = 0;
            DWORD dwNetAddress = 0;
            int iNumber;

                printf("\nNetwork address (e.g. 192.168.100.0): ");
                iNumber = scanf("%i.%i.%i.%i", &iVal1, &iVal2, &iVal3, &iVal4);
                if (iNumber < 2)
                {
                    printf("\nInvalid address specified!\n");
                    break;
                }

                dwNetAddress = iVal1;
                dwNetAddress <<= 8;
                dwNetAddress |= iVal2;
                dwNetAddress <<= 8;
                dwNetAddress |= iVal3;
                dwNetAddress <<= 8;
                dwNetAddress |= iVal4;

                Ret = EplSdoUdpuSetNetAddress(dwNetAddress);
                if(Ret != kEplSuccessful)
                {
                    printf("Setting of new network address failed: 0x%03X\n", Ret);
                }
                else
                {
                    printf("Network address %lX set successfully\n", (unsigned long) dwNetAddress);
                }
                break;
            }

            // 3
            case '3':
            {
            int iVal1 = 0;
            int iNumber;

                printf("\nRemote node-ID (e.g. 0x21): ");
                iNumber = scanf("%i", &iVal1);
                if (iNumber != 1)
                {
                    printf("\nInvalid node-ID specified!\n");
                    break;
                }

                bTargetNodeId_l = (BYTE) iVal1;
                break;
            }

            // a
            case 'a':
            {
                // read object 0x1000
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(uiBuffer);
                TransParamByIndex.m_uiIndex = 0x1000;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Read of object 0x1000 started\n");
                }

                break;
            }

            // b
            case 'b':
            {   // read object 0x1001
                TransParamByIndex.m_pData = &bBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(bBuffer);
                TransParamByIndex.m_uiIndex = 0x1001;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Read of object 0x1001 started\n");
                }

                break;
            }

            //  c
            case 'c':
            {   // read object 0x1008
                // init buffer
                EPL_MEMSET(&cBuffer[0], 0x00, sizeof(cBuffer));

                TransParamByIndex.m_pData = &cBuffer[0];
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(cBuffer);
                TransParamByIndex.m_uiIndex = 0x1008;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Read of object 0x1008 started\n");
                }
                break;
            }

            // d
            case 'd':
            {   // write 1000 to object 0x1006
                ulSize = sizeof(uiBuffer);
                uiBuffer = 1000;
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = ulSize;
                TransParamByIndex.m_uiIndex = 0x1006;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("write of object 0x1006 started\n");
                }
                break;
            }

            // e
            case 'e':
            {   // read object 0x1006
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(uiBuffer);
                TransParamByIndex.m_uiIndex = 0x1006;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Read of object 0x1006 started\n");
                }

                break;
            }

            // f
            case 'f':
            {   // write 2000 byte to object 0x2100

                // init buffer
                uiCount = sizeof(abTest_l) / sizeof(abTest_l[0]);
                while(uiCount > 0)
                {
                    abTest_l[uiCount - 1] = (BYTE)uiCount;
                    uiCount--;
                }
                ulSize = sizeof(abTest_l);
                TransParamByIndex.m_pData = &abTest_l[0];
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = ulSize;
                TransParamByIndex.m_uiIndex = 0x2100;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {
                    printf("write of object 0x2100 started\n");
                }
                break;
            }

            // g
            case 'g':
            {   // read object 0x2100
                // init data of domain 0x2100
                ulSize = sizeof(abTest_l);
                EPL_MEMSET(&abTest_l[0], 0x00, ulSize);
                TransParamByIndex.m_pData = &abTest_l[0];
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = ulSize;
                TransParamByIndex.m_uiIndex = 0x2100;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Read of object 0x2100 started\n");
                }

                break;
            }

            // h
            case 'h':
            {   // write 0xff to object 0x1000
                uiBuffer = 0xff;
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(uiBuffer);
                TransParamByIndex.m_uiIndex = 0x1000;
                TransParamByIndex.m_uiSubindex = 0;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Write of object 0x1000 started\n");
                }
                break;
            }

            // i
            case 'i':
            {   // write 0xff object 0x1030 subindex 1
                uiBuffer = 0xff;
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(uiBuffer);
                TransParamByIndex.m_uiIndex = 0x1030;
                TransParamByIndex.m_uiSubindex = 1;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Write of object 0x1030 subindex 1 started\n");
                }
                break;
            }

            // j
            case 'j':
            {   // write 0xFFFF to object 0x1030 subindex 8
                uiBuffer = 0xffff;
                TransParamByIndex.m_pData = &uiBuffer;
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiDataSize = sizeof(uiBuffer);
                TransParamByIndex.m_uiIndex = 0x1030;
                TransParamByIndex.m_uiSubindex = 8;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                if(Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplSdoComInitTransferByIndex\n", Ret);
                }
                else
                {   // wait for end of transfer
                    printf("Write of object 0x1030 subindex 8 started\n");
                }

                break;
            }

            // r
            case 'r':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;

                // read arbitrary remote object
                TransParamByIndex.m_pData = &abTest_l[0];
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;
                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    TransParamByIndex.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        TransParamByIndex.m_uiSubindex = (unsigned int) iVal2;
                    }
                    else
                    {
                        TransParamByIndex.m_uiSubindex = 0;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                printf("\nSegment Size (e.g. 4): ");
                fflush(stdin);
                iNumber = scanf("%i", &iVal1);
                if (iNumber >= 1)
                {
                    TransParamByIndex.m_uiDataSize = iVal1;
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                if (TransParamByIndex.m_uiDataSize > sizeof (abTest_l))
                {
                    TransParamByIndex.m_uiDataSize = sizeof (abTest_l);
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = 0xDEADBEEF;
                    uiCount--;
                }

                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                printf("Read of remote object 0x%X/%u with transfer size %u returned\n   code 0x%X",
                    TransParamByIndex.m_uiIndex, TransParamByIndex.m_uiSubindex,
                    TransParamByIndex.m_uiDataSize,
                    Ret);
                if (Ret != kEplSuccessful)
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf("   Waiting for finish\n");
                }

                break;
            }

            // s
            case 's':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;

                // write string to arbitrary remote object
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    TransParamByIndex.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        TransParamByIndex.m_uiSubindex = (unsigned int) iVal2;
                    }
                    else
                    {
                        TransParamByIndex.m_uiSubindex = 0;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = 0xDEADBEEF;
                    uiCount--;
                }

                printf("\nString to be written to object: ");
                fflush(stdin);
                TransParamByIndex.m_pData = fgets((char*) &abTest_l[0], sizeof (abTest_l), stdin);
                if (TransParamByIndex.m_pData == NULL)
                {
                    printf("\nWrong input!\n");
                    break;
                }

                TransParamByIndex.m_uiDataSize = strlen((const char*) abTest_l) - 1;

                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;
                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                printf("Write of remote object 0x%X/%u with transfer size %u returned\n   code 0x%X",
                    TransParamByIndex.m_uiIndex, TransParamByIndex.m_uiSubindex,
                    TransParamByIndex.m_uiDataSize,
                    Ret);
                if (Ret != kEplSuccessful)
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf("   Waiting for finish\n");
                }

                break;
            }

            // w
            case 'w':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;
            char* pszToken;
            char* pszRemainder;

                // write bytes to arbitrary remote object
                TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
                TransParamByIndex.m_SdoComConHdl = SdoComConHdl;
                TransParamByIndex.m_uiTimeout = 0;
                TransParamByIndex.m_pfnSdoFinishedCb = EplAppSdoConnectionCb;

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    TransParamByIndex.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        TransParamByIndex.m_uiSubindex = (unsigned int) iVal2;
                    }
                    else
                    {
                        TransParamByIndex.m_uiSubindex = 0;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                printf("\nSegment Size (0 if you want to specify the actual byte string): ");
                fflush(stdin);
                iNumber = scanf("%i", &iVal1);
                if (iNumber == 1)
                {
                    TransParamByIndex.m_uiDataSize = iVal1;
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = uiCount;
                    uiCount--;
                }

                if (TransParamByIndex.m_uiDataSize == 0)
                {
                    printf("\nBytes to be written to object in LE (eg. 12 AB 3F): ");
                    fflush(stdin);
                    TransParamByIndex.m_pData = fgets((char*) &abTest_l[0], sizeof (abTest_l), stdin);
                    if (TransParamByIndex.m_pData == NULL)
                    {
                        printf("\nWrong input!\n");
                        break;
                    }

                    pszRemainder = (char*) &abTest_l[0];
                    TransParamByIndex.m_uiDataSize = 0;
                    do
                    {
                        pszToken = pszRemainder;
                        abTest_l[TransParamByIndex.m_uiDataSize] = (BYTE) strtol(pszToken, &pszRemainder, 16);
                        if (pszToken == pszRemainder)
                        {
                            break;
                        }
                        TransParamByIndex.m_uiDataSize++;
                    }
                    while (*pszRemainder != '\0');
                }
                else
                {
                    TransParamByIndex.m_pData = &abTest_l[0];
                }

                TransParamByIndex.m_pUserArg = TransParamByIndex.m_pData;
                Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
                printf("Write of remote object 0x%X/%u with transfer size %u returned\n   code 0x%X",
                    TransParamByIndex.m_uiIndex, TransParamByIndex.m_uiSubindex,
                    TransParamByIndex.m_uiDataSize,
                    Ret);
                if (Ret != kEplSuccessful)
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf("   Waiting for finish\n");
                }

                break;
            }

#endif

            // A
            case 'A':
            {
                // read object 0x1000
                ObdSize = 4;
                Ret = EplObdReadEntry(0x1000, 0x00, cBuffer, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdReadEntry\n", Ret);
                }
                else
                {
                    printf("Read of local object 0x1000 returned 0x%lX and size=%u\n", (unsigned long) *((DWORD*)cBuffer), ObdSize);
                }

                break;
            }

            // B
            case 'B':
            {
                // read object 0x1001
                ObdSize = 1;
                Ret = EplObdReadEntry(0x1001, 0x00, cBuffer, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdReadEntry\n", Ret);
                }
                else
                {
                    printf("Read of local object 0x1001 returned 0x%X and size=%u\n", (unsigned int) *((BYTE*)&cBuffer[0]), ObdSize);
                }

                break;
            }

            // C
            case 'C':
            {
                // read object 0x1008
                ObdSize = sizeof (cBuffer);
                Ret = EplObdReadEntry(0x1008, 0x00, cBuffer, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdReadEntry\n", Ret);
                }
                else
                {
                    printf("Read of local object 0x1008 returned \"%s\" and size=%u\n", cBuffer, ObdSize);
                }

                break;
            }

            // D
            case 'D':
            {
                // write object 0x1006
                ObdSize = 4;
                *((DWORD*)cBuffer) = 1000;
                Ret = EplObdWriteEntry(0x1006, 0x00, cBuffer, ObdSize);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdReadEntry\n", Ret);
                }
                else
                {
                    printf("Write of value 1000 to local object 0x1006 succeeded\n");
                }

                break;
            }

            // E
            case 'E':
            {
                // read object 0x1006
                ObdSize = 4;
                Ret = EplObdReadEntry(0x1006, 0x00, cBuffer, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdReadEntry\n", Ret);
                }
                else
                {
                    printf("Read of local object 0x1006 returned 0x%lX and size=%u\n", (unsigned long) *((DWORD*)cBuffer), ObdSize);
                }

                break;
            }

            // F
            case 'F':
            {
                // write object 0x2100
                EPL_MEMSET(&ObdParam, 0, sizeof (ObdParam));
                ObdParam.m_pData = abTest_l;
                ObdParam.m_SegmentSize = 2000;
                ObdParam.m_TransferSize = ObdParam.m_SegmentSize;
                ObdParam.m_uiIndex = 0x2100;
                ObdParam.m_uiSubIndex = 0x0;

                // init buffer
                uiCount = sizeof(abTest_l) / sizeof(abTest_l[0]);
                while (uiCount > 0)
                {
                    abTest_l[uiCount - 1] = (BYTE)uiCount;
                    uiCount--;
                }

                Ret = EplObdWriteEntryFromLe(&ObdParam);
                if (Ret != kEplSuccessful)
                {
                    printf("Error = 0x%04X in function EplObdWriteEntryFromLe\n", Ret);
                }
                else
                {
                    printf("Write of local object 0x2100 returned size=%u\n", ObdParam.m_SegmentSize);
                }

                break;
            }

            // G
            case 'G':
            {
                // read object 0x2100
                EPL_MEMSET(&ObdParam, 0, sizeof (ObdParam));
                ObdParam.m_pData = abTest_l;
                ObdParam.m_SegmentSize = 1000; //sizeof (abTest_l);
                ObdParam.m_TransferSize = ObdParam.m_SegmentSize;
                ObdParam.m_uiIndex = 0x2100;
                ObdParam.m_uiSubIndex = 0x0;
                Ret = EplObdReadEntryToLe(&ObdParam);
                if ((Ret != kEplSuccessful) && (Ret != kEplObdSegmentReturned))
                {
                    printf("Error = 0x%04X in function EplObdReadEntryToLe\n", Ret);
                }
                else
                {
                    printf("Read of local object 0x2100 returned size=%u\n", ObdParam.m_SegmentSize);
                    EplAppDumpData(ObdParam.m_pData, min (ObdParam.m_SegmentSize, 256));
                }

                break;
            }

            // R
            case 'R':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;

                // read arbitrary local object
                EPL_MEMSET(&ObdParam, 0, sizeof (ObdParam));

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    ObdParam.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        ObdParam.m_uiSubIndex = (unsigned int) iVal2;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                printf("\nSegment Size [and Segment Offset] (e.g. 4/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    ObdParam.m_SegmentSize = iVal1;

                    if (iNumber == 2)
                    {
                        ObdParam.m_SegmentOffset = iVal2;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                if (ObdParam.m_SegmentSize > sizeof (abTest_l))
                {
                    ObdParam.m_SegmentSize = sizeof (abTest_l);
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = 0xDEADBEEF;
                    uiCount--;
                }

                ObdParam.m_pData = abTest_l;
                ObdParam.m_TransferSize = ObdParam.m_SegmentSize;
                ObdParam.m_pHandle = (void*)0xBEEFDEAD;
                ObdParam.m_pfnAccessFinished = EplAppCbAccessFinished;
                Ret = EplObdReadEntryToLe(&ObdParam);
                printf("Read of local object 0x%X/%u with object size %u returned\n   code 0x%X, size=%u, offset=%u",
                    ObdParam.m_uiIndex, ObdParam.m_uiSubIndex,
                    ObdParam.m_ObjSize,
                    Ret, ObdParam.m_SegmentSize, ObdParam.m_SegmentOffset);
                if ((Ret != kEplSuccessful) && (Ret != kEplObdSegmentReturned))
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf(" and data[%u]=%02X data[%u]=\n",
                        min(ObdParam.m_SegmentSize, sizeof (abTest_l) - 1),
                        (WORD) ((BYTE*)ObdParam.m_pData)[min(ObdParam.m_SegmentSize, sizeof (abTest_l) - 1)],
                        ObdParam.m_SegmentOffset);
                    EplAppDumpData(ObdParam.m_pData, min (ObdParam.m_SegmentSize, 256));
                }

                break;
            }

            // S
            case 'S':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;

                // write string to arbitrary local object
                EPL_MEMSET(&ObdParam, 0, sizeof (ObdParam));

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    ObdParam.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        ObdParam.m_uiSubIndex = (unsigned int) iVal2;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                printf("\nSegment Offset (e.g. 0): ");
                fflush(stdin);
                iNumber = scanf("%i", &iVal1);
                if (iNumber == 1)
                {
                    ObdParam.m_SegmentOffset = iVal1;
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = 0xDEADBEEF;
                    uiCount--;
                }

                printf("\nString to be written to object: ");
                fflush(stdin);
                ObdParam.m_pData = fgets((char*) &abTest_l[0], sizeof (abTest_l), stdin);
                if (ObdParam.m_pData == NULL)
                {
                    printf("\nWrong input!\n");
                    break;
                }

                ObdParam.m_SegmentSize = strlen((const char*) abTest_l) - 1;

                ObdParam.m_TransferSize = ObdParam.m_SegmentSize;
                ObdParam.m_pHandle = (void*)0xBEEFDEAD;
                ObdParam.m_pfnAccessFinished = EplAppCbAccessFinished;
                Ret = EplObdWriteEntryFromLe(&ObdParam);
                printf("Write to local object 0x%X/%u with object size %u returned\n   code 0x%X, size=%u, offset=%u",
                    ObdParam.m_uiIndex, ObdParam.m_uiSubIndex,
                    ObdParam.m_ObjSize,
                    Ret, ObdParam.m_SegmentSize, ObdParam.m_SegmentOffset);
                if (Ret != kEplSuccessful)
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf("   successfully\n");
                }

                break;
            }

            // W
            case 'W':
            {
            int iVal1 = 0;
            int iVal2 = 0;
            int iNumber;
            char* pszToken;
            char* pszRemainder;

                // write bytes to arbitrary local object
                EPL_MEMSET(&ObdParam, 0, sizeof (ObdParam));

                printf("\nObject index[/sub-index] (e.g. 0x1000/0): ");
                fflush(stdin);
                iNumber = scanf("%i/%i", &iVal1, &iVal2);
                if (iNumber >= 1)
                {
                    ObdParam.m_uiIndex = (unsigned int) iVal1;

                    if (iNumber == 2)
                    {
                        ObdParam.m_uiSubIndex = (unsigned int) iVal2;
                    }
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                printf("\nSegment Offset (e.g. 0): ");
                fflush(stdin);
                iNumber = scanf("%i", &iVal1);
                if (iNumber == 1)
                {
                    ObdParam.m_SegmentOffset = iVal1;
                }
                else
                {
                    printf("\nWrong input!\n");
                    break;
                }

                // init buffer
                uiCount = sizeof(abTest_l) / 4;
                while (uiCount > 0)
                {
                    ((DWORD*)abTest_l)[uiCount - 1] = 0xDEADBEEF;
                    uiCount--;
                }

                printf("\nBytes to be written to object in LE (eg. 12 AB 3F): ");
                fflush(stdin);
                ObdParam.m_pData = fgets((char*) &abTest_l[0], sizeof (abTest_l), stdin);
                if (ObdParam.m_pData == NULL)
                {
                    printf("\nWrong input!\n");
                    break;
                }

                pszRemainder = (char*) &abTest_l[0];
                ObdParam.m_SegmentSize = 0;
                do
                {
                    pszToken = pszRemainder;
                    abTest_l[ObdParam.m_SegmentSize] = (BYTE) strtol(pszToken, &pszRemainder, 16);
                    if (pszToken == pszRemainder)
                    {
                        break;
                    }
                    ObdParam.m_SegmentSize++;
                }
                while (*pszRemainder != '\0');

                ObdParam.m_TransferSize = ObdParam.m_SegmentSize;
                ObdParam.m_pHandle = (void*)0xBEEFDEAD;
                ObdParam.m_pfnAccessFinished = EplAppCbAccessFinished;
                Ret = EplObdWriteEntryFromLe(&ObdParam);
                printf("Write to local object 0x%X/%u with object size %u returned\n   code 0x%X, size=%u, offset=%u",
                    ObdParam.m_uiIndex, ObdParam.m_uiSubIndex,
                    ObdParam.m_ObjSize,
                    Ret, ObdParam.m_SegmentSize, ObdParam.m_SegmentOffset);
                if (Ret != kEplSuccessful)
                {
                    printf("   Error!\n");
                }
                else
                {
                    printf("   successfully\n");
                }

                break;
            }

            // ?
            case '?':
            {
                EplAppPrintMenue();
                break;
            }

            default:
            {
                Sleep(10);
            }

        } // end of switch(iChar)

    } // end of while(1)


ShutDown:


    // delete instance for all modules
    EplSdoComDelInstance();

    // delete instance of eventu-module
    EplEventuDelInstance();

    // delete instance of Timeru-module
    EplTimeruDelInstance();

    // delete obd instnace
    EplObdDeleteInstance();

    // close handle
    CloseHandle(SdoReadyHdl_l);

Exit:
    return Ret;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplApiProcessEvent
//
// Description: processes events from event queue and forwards these to
//              the application's event callback function
//
// Parameters:  pEplEvent_p =   pointer to event
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplAppProcessEvent(
            tEplEvent* pEplEvent_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplTimerEventArg*  pTimerEventArg;
tEplObdParam*       pObdParam;
BYTE                abData[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};

    Ret = kEplSuccessful;
    // check parameter
    if (pEplEvent_p == NULL)
    {
        Ret = kEplSdoSeqInvalidEvent;
        goto Exit;
    }

    if (pEplEvent_p->m_EventType != kEplEventTypeTimer)
    {
        Ret = kEplSdoSeqInvalidEvent;
        goto Exit;
    }

    // get timerhdl
    pTimerEventArg = (tEplTimerEventArg*)pEplEvent_p->m_pArg;

    // get pointer to intern control structure of connection
    if (pTimerEventArg->m_Arg.m_pVal == NULL)
    {
        goto Exit;
    }
    pObdParam = (tEplObdParam*)pTimerEventArg->m_Arg.m_pVal;

    pObdParam->m_pData = &abData[0];

    if (pObdParam->m_uiSubIndex == 3)
    {
        pObdParam->m_dwAbortCode = EPL_SDOAC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL;
    }

    printf("EplAppProcessEvent(0x%04X/%u Ev=%X pData=%p Off=%u Size=%u\n"
           "                   ObjSize=%u TransSize=%u Acc=%X Typ=%X)\n",
        pObdParam->m_uiIndex, pObdParam->m_uiSubIndex,
        pObdParam->m_ObdEvent,
        pObdParam->m_pData, pObdParam->m_SegmentOffset, pObdParam->m_SegmentSize,
        pObdParam->m_ObjSize, pObdParam->m_TransferSize, pObdParam->m_Access, pObdParam->m_Type);

    Ret = pObdParam->m_pfnAccessFinished(pObdParam);

    EPL_FREE(pObdParam);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplAppCbAccessFinished();
//
// Description: function is called by the adopter of the OD access
//
// Parameters:  pObdParam_p     = pointer to OBD parameter structure
//
// Returns:     tEplKernel  =  errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplAppCbAccessFinished(tEplObdParam* pObdParam_p)
{
tEplKernel      Ret;

    Ret = kEplSuccessful;

    if (pObdParam_p->m_ObdEvent == kEplObdEvPreRead)
    {
        printf("Local read of object ");
    }
    else
    {
        printf("Local write of object ");
    }
    printf("0x%04X/0x%02X finished %u bytes transferred\n(Abortcode: 0x%04x Handle: 0x%x)\n",
        pObdParam_p->m_uiIndex,
        pObdParam_p->m_uiSubIndex,
        pObdParam_p->m_SegmentSize,
        pObdParam_p->m_dwAbortCode,
        pObdParam_p->m_pHandle);

    switch (pObdParam_p->m_SegmentSize)
    {
        case 0:
            printf("no Bytes transfered\n");
            break;

        case 1:
            printf("BYTE: 0x%02X\n", (WORD)AmiGetByteFromLe(pObdParam_p->m_pData));
            break;

        case 2:
            printf("WORD: 0x%04X\n", AmiGetWordFromLe(pObdParam_p->m_pData));
            break;

        case 3:
            printf("3 BYTEs: 0x%06X\n", AmiGetDword24FromLe(pObdParam_p->m_pData));
            break;

        case 4:
            printf("DWORD: 0x%08X\n", AmiGetDwordFromLe(pObdParam_p->m_pData));
            break;

        default:
            EplAppDumpData(pObdParam_p->m_pData, min (pObdParam_p->m_SegmentSize, 256));
            break;
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:        EplAppSdoConnectionCb
//
// Description:     callback to get informt about the result of the sdo transfer
//
//
//
// Parameters:      SdoComFinished_p = infos abort the sdo transfer
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel PUBLIC  EplAppSdoConnectionCb(tEplSdoComFinished*  pSdoComFinished_p)
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    if (pSdoComFinished_p->m_SdoAccessType == kEplSdoAccessTypeRead)
    {
        printf("SDO read of object ");
    }
    else
    {
        printf("SDO write of object ");
    }
    printf("0x%04X/0x%02X finished %u bytes transferred\n(Abortcode: 0x%04x Handle: 0x%x State: %s)\n",
        pSdoComFinished_p->m_uiTargetIndex,
        pSdoComFinished_p->m_uiTargetSubIndex,
        pSdoComFinished_p->m_uiTransferredByte,
        pSdoComFinished_p->m_dwAbortCode,
        pSdoComFinished_p->m_SdoComConHdl,
        aszSdoStates_l[pSdoComFinished_p->m_SdoComConState]);

    switch (pSdoComFinished_p->m_uiTransferredByte)
    {
        case 0:
            printf("no Bytes transfered\n");
            break;

        case 1:
            printf("BYTE: 0x%02X\n", (WORD)AmiGetByteFromLe(pSdoComFinished_p->m_pUserArg));
            break;

        case 2:
            printf("WORD: 0x%04X\n", AmiGetWordFromLe(pSdoComFinished_p->m_pUserArg));
            break;

        case 3:
            printf("3 BYTEs: 0x%06X\n", AmiGetDword24FromLe(pSdoComFinished_p->m_pUserArg));
            break;

        case 4:
            printf("DWORD: 0x%08X\n", AmiGetDwordFromLe(pSdoComFinished_p->m_pUserArg));
            break;

        default:
            EplAppDumpData(pSdoComFinished_p->m_pUserArg, min (pSdoComFinished_p->m_uiTransferredByte, 256));
            break;
    }

/*    if((pSdoComFinished_p->m_dwAbortCode != 0)
        ||(pSdoComFinished_p->m_SdoComConState != kEplSdoComTransferFinished))
    {
        fSdoSuccessful_l = FALSE;
    }
    else
    {
        fSdoSuccessful_l = TRUE;
    }

    // set event
    SetEvent(SdoReadyHdl_l);
*/

return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplAppCbObdAccess
//
// Description: callback function for OD accesses
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplAppCbObdAccess(tEplObdParam MEM* pParam_p)
{
tEplKernel          Ret = kEplSuccessful;

    printf("EplAppCbObdAccess(0x%04X/%u Ev=%X pData=%p Off=%u Size=%u\n"
           "                  ObjSize=%u TransSize=%u Acc=%X Typ=%X)\n",
        pParam_p->m_uiIndex, pParam_p->m_uiSubIndex,
        pParam_p->m_ObdEvent,
        pParam_p->m_pData, pParam_p->m_SegmentOffset, pParam_p->m_SegmentSize,
        pParam_p->m_ObjSize, pParam_p->m_TransferSize, pParam_p->m_Access, pParam_p->m_Type);

    if ((pParam_p->m_uiSubIndex >= 2)
        && ((pParam_p->m_ObdEvent == kEplObdEvInitWriteLe)
            || (pParam_p->m_ObdEvent == kEplObdEvPreRead)))
    {   // adopt the transfer
    tEplObdParam*   pMyObdParam;
    tEplTimerArg    TimerArg;
    tEplTimerHdl    EplTimerHdl;

        if (pParam_p->m_ObdEvent == kEplObdEvInitWriteLe)
        {
            EplAppDumpData(pParam_p->m_pData, min (pParam_p->m_SegmentSize, 256));
        }

        if (pParam_p->m_pfnAccessFinished == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_DATA_NOT_TRANSF_OR_STORED;
            Ret = kEplObdAccessViolation;
            goto Exit;
        }

        pMyObdParam = EPL_MALLOC(sizeof (*pMyObdParam));
        if (pMyObdParam == NULL)
        {
            Ret = kEplObdOutOfMemory;
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            goto Exit;
        }

        EPL_MEMCPY(pMyObdParam, pParam_p, sizeof (*pMyObdParam));

        TimerArg.m_EventSink = kEplEventSinkApi;
        TimerArg.m_Arg.m_pVal = pMyObdParam;

        Ret = EplTimeruSetTimerMs(&EplTimerHdl,
                                    2000,
                                    TimerArg);

        Ret = kEplObdAccessAdopted;
        printf("  Adopted\n");
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplAppCbDefaultObdAccess
//
// Description: callback function for default OD accesses
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplAppCbDefaultObdAccess(tEplObdParam MEM* pParam_p)
{
tEplKernel          Ret = kEplSuccessful;

    printf("EplAppCbDefaultObdAccess(0x%04X/%u Ev=%X pData=%p Off=%u Size=%u\n"
           "                         ObjSize=%u TransSize=%u Acc=%X Typ=%X)\n",
        pParam_p->m_uiIndex, pParam_p->m_uiSubIndex,
        pParam_p->m_ObdEvent,
        pParam_p->m_pData, pParam_p->m_SegmentOffset, pParam_p->m_SegmentSize,
        pParam_p->m_ObjSize, pParam_p->m_TransferSize, pParam_p->m_Access, pParam_p->m_Type);

    if (pParam_p->m_uiIndex != 0x2500)
    {
        printf("  Object does not exist!\n");
        pParam_p->m_dwAbortCode = EPL_SDOAC_OBJECT_NOT_EXIST;
        Ret = kEplObdIndexNotExist;
        goto Exit;
    }

    if (pParam_p->m_uiSubIndex > 10)
    {
        printf("  Sub-index does not exist!\n");
        pParam_p->m_dwAbortCode = EPL_SDOAC_SUB_INDEX_NOT_EXIST;
        Ret = kEplObdSubindexNotExist;
        goto Exit;
    }

    if ((pParam_p->m_ObdEvent == kEplObdEvInitWriteLe)
        || (pParam_p->m_ObdEvent == kEplObdEvPreRead))
    {   // adopt the transfer
    tEplObdParam*   pMyObdParam;
    tEplTimerArg    TimerArg;
    tEplTimerHdl    EplTimerHdl;

        if (pParam_p->m_ObdEvent == kEplObdEvInitWriteLe)
        {
            EplAppDumpData(pParam_p->m_pData, min (pParam_p->m_SegmentSize, 256));
        }

        if (pParam_p->m_pfnAccessFinished == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_DATA_NOT_TRANSF_OR_STORED;
            Ret = kEplObdAccessViolation;
            goto Exit;
        }

        pMyObdParam = EPL_MALLOC(sizeof (*pMyObdParam));
        if (pMyObdParam == NULL)
        {
            Ret = kEplObdOutOfMemory;
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            goto Exit;
        }

        EPL_MEMCPY(pMyObdParam, pParam_p, sizeof (*pMyObdParam));

        TimerArg.m_EventSink = kEplEventSinkApi;
        TimerArg.m_Arg.m_pVal = pMyObdParam;

        Ret = EplTimeruSetTimerMs(&EplTimerHdl,
                                    2000,
                                    TimerArg);

        Ret = kEplObdAccessAdopted;
        printf("  Adopted\n");
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:        EplAppPrintMenue
//
// Description:     function print out menue
//
//
//
// Parameters:      void
//
//
// Returns:         void
//
//
// State:
//
//---------------------------------------------------------------------------
static void EplAppPrintMenue()
{
    printf("SDO remote node-ID=0x%02X\n", bTargetNodeId_l);

    // print menue
    printf("\nESC \t Leave programm\n");

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
    printf("0 \t Close SDO via UDP connection\n");
    printf("1 \t Define SDO via UDP connection\n");
    printf("2 \t Set network part of IP address\n");
    printf("3 \t Set remote node-ID\n");
    printf("a \t Read remote object 0x1000\n");
    printf("b \t Read remote object 0x1001\n");
    printf("c \t Read remote object 0x1008\n");
    printf("d \t Write 1000 to remote object 0x1006\n");
    printf("e \t Read remote object 0x1006\n");
    printf("f \t Write 2000 Byte to remote object 0x2100\n");
    printf("g \t Read remote object 0x2100\n");
    printf("h \t Write 0xFF to remote object 0x1000\n");
    printf("i \t Write 0xFF to remote object 0x1030 subindex 1\n");
    printf("j \t Write 0xFFFF to remote object 0x1030 subindex 8\n");
    printf("r \t Read arbitrary remote object in LE\n");
    printf("s \t Write string to arbitrary remote object\n");
    printf("w \t Write bytes to arbitrary remote object in LE\n");
#endif

    printf("A \t Read local object 0x1000\n");
    printf("B \t Read local object 0x1001\n");
    printf("C \t Read local object 0x1008\n");
    printf("D \t Write 1000 to local object 0x1006\n");
    printf("E \t Read local object 0x1006\n");
    printf("F \t Write 2000 Byte to local object 0x2100\n");
    printf("G \t Read local object 0x2100\n");
    printf("R \t Read arbitrary local object in LE\n");
    printf("S \t Write string to arbitrary local object\n");
    printf("W \t Write bytes to arbitrary local object in LE\n");

    printf("? \t show help\n");
    printf("\n");
}

//---------------------------------------------------------------------------
//
// Function:        EplAppDumpData
//
// Description:     function print out data in hexadecimal value
//
//
//
// Parameters:      pData_p         = pointer to data
//                  ulDataSize_p    = size of data in byte
//
//
// Returns:         void
//
//
// State:
//
//---------------------------------------------------------------------------
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
static void EplAppDumpData(void*         pData_p,
                           unsigned long ulDataSize_p)
{
unsigned long   ulCount;
unsigned long   ulOffset;
BYTE*           pbData;
char            szBuffer[17];

    if (pData_p == NULL)
    {
        printf("Null pointer!\n");
        return;
    }

    szBuffer[16] = '\0';

    pbData = (BYTE*)pData_p;

    ulCount = 0;
    ulOffset = 0;
    // printout data
    while (ulCount < ulDataSize_p)
    {
        printf(" %02X", (WORD)*pbData);
        if (isgraph(*pbData))
        {
            szBuffer[ulOffset] = *pbData;
        }
        else
        {
            szBuffer[ulOffset] = '.';
        }
        pbData++;
        ulCount++;
        ulOffset++;
        if (ulOffset == 16)
        {
            printf("  %s\n", szBuffer);
            ulOffset = 0;
        }
    }

    if (ulOffset != 0)
    {
        szBuffer[ulOffset] = '\0';
        for (; ulOffset < 16; ulOffset++)
        {
            printf("   ");
        }
        printf("  %s\n", szBuffer);
    }
    else
    {
        printf("\n");
    }
}
#endif

// EOF

