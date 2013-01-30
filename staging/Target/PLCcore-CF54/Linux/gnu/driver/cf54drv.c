/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Board driver for SYSTEC ECUcore-5484

  Description:  User space wrapper for SYSTEC ECUcore-5485 board driver

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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

  -------------------------------------------------------------------------

  2006/09/27 -rs:   Initial Version
  2006/10/01 -rs:   Support for I/O board PCB 4160.0
  2006/10/03 -rs:   Support for I/O board PCB 4158.1
  2007/02/19 -rs:   Support for I/O board PCB 4158.1 with PLD

****************************************************************************/

#include "../../../../../Include/global.h"
#include "cf54drv.h"
#include "cf54def.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Global variables
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Local variables
//---------------------------------------------------------------------------

static  int  hDrvInst_l = -1;           // driver file descriptior



//---------------------------------------------------------------------------
//  Prototypes of internal functions
//---------------------------------------------------------------------------





//=========================================================================//
//                                                                         //
//          C F 5 4   I / O   D R I V E R   S T U B                        //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Initialize hardware
//---------------------------------------------------------------------------

WORD  PUBLIC  CF54Initialize (void)
{

const char*  pszDrvName;                // driver name
WORD         wDrvVer;
int          iRet;


    TRACE("\nIOSTB: + cf54drv#CF54Initialize...\n");


    pszDrvName = SYSTEC_CF54DRV_NAME_DEV;
    hDrvInst_l = -1;

    wDrvVer = 0;
    iRet    = -1;


    // open driver
    TRACE("IOSTB:   Try to open driver '%s'\n", pszDrvName);
    hDrvInst_l = open (pszDrvName, O_RDWR);
    if (hDrvInst_l > -1)
    {
        TRACE("IOSTB:   open '%s' successful -> hDrvInst=%d\n", pszDrvName, hDrvInst_l);
    }
    else
    {
        TRACE("IOSTB:   ERROR: Can't open '%s'\n", pszDrvName);
        goto Exit;
    }


    // initialize hardware
    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_INITIALIZE, (unsigned long)&wDrvVer);


Exit:

    TRACE("IOSTB: - cf54drv#CF54Initialize (iRet=%d, wDrvVer=0x%04X)\n\n", iRet, wDrvVer);
    return (wDrvVer);

}



//---------------------------------------------------------------------------
//  Release hardware
//---------------------------------------------------------------------------

void  PUBLIC  CF54ShutDown (void)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54ShutDown...\n");


    // release hardware
    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_SHUTDOWN, 0);


    // close driver
    if (hDrvInst_l >= 0)
    {
        close (hDrvInst_l);
    }


    TRACE("IOSTB: - cf54drv#CF54ShutDown (iRet=%d)\n\n", iRet);
    return;

}



//---------------------------------------------------------------------------
//  Get hardware info
//---------------------------------------------------------------------------

void  PUBLIC  CF54GetHardwareInfo (
    tCF54HwInfo* pHwInfo_p)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54GetHardwareInfo...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_GET_HARDWARE_INFO, pHwInfo_p);

    TRACE("IOSTB: - cf54drv#CF54GetHardwareInfo (iRet=%d)\n\n", iRet);
    return;

}



//---------------------------------------------------------------------------
//  Set Run LED
//---------------------------------------------------------------------------

void  PUBLIC  CF54SetRunLed (
    BYTE bState_p)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54SetRunLed...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_SET_RUN_LED, bState_p);

    TRACE("IOSTB: - cf54drv#CF54SetRunLed (iRet=%d)\n\n", iRet);
    return;

}



//---------------------------------------------------------------------------
//  Set Error LED
//---------------------------------------------------------------------------

void  PUBLIC  CF54SetErrLed (
    BYTE bState_p)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54SetErrLed...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_SET_ERR_LED, bState_p);

    TRACE("IOSTB: - cf54drv#CF54SetErrLed (iRet=%d)\n\n", iRet);
    return;

}



//---------------------------------------------------------------------------
//  Get Run/Stop/MRes-Switch
//---------------------------------------------------------------------------

BYTE  PUBLIC  CF54GetRSMSwitch (void)
{

BYTE  bRSMSwitch;
int   iRet;


    TRACE("\nIOSTB: + cf54drv#CF54GetRSMSwitch...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_GET_RSM_SWITCH, &bRSMSwitch);

    TRACE("IOSTB: - cf54drv#CF54GetRSMSwitch (iRet=%d, bRSMSwitch=0x%02X)\n\n", iRet, (WORD)bRSMSwitch);
    return (bRSMSwitch);

}



//---------------------------------------------------------------------------
//  Get Hex-Switch
//---------------------------------------------------------------------------

BYTE  PUBLIC  CF54GetHexSwitch (void)
{

BYTE  bHexSwitch;
int   iRet;


    TRACE("\nIOSTB: + cf54drv#CF54GetHexSwitch...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_GET_HEX_SWITCH, &bHexSwitch);

    TRACE("IOSTB: - cf54drv#CF54GetHexSwitch (iRet=%d, bHexSwitch=0x%02X)\n\n", iRet, (WORD)bHexSwitch);
    return (bHexSwitch);

}



//---------------------------------------------------------------------------
//  Get DIP-Switch
//---------------------------------------------------------------------------

BYTE  PUBLIC  CF54GetDipSwitch (void)
{

BYTE  bDipSwitch;
int   iRet;


    TRACE("\nIOSTB: + cf54drv#CF54GetDipSwitch...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_GET_DIP_SWITCH, &bDipSwitch);

    TRACE("IOSTB: - cf54drv#CF54GetDipSwitch (iRet=%d, bDipSwitch=0x%02X)\n\n", iRet, (WORD)bDipSwitch);
    return (bDipSwitch);

}



//---------------------------------------------------------------------------
//  Get digital input lines
//---------------------------------------------------------------------------

void  PUBLIC  CF54GetDigiIn (
    tCF54DigiIn* pDiData_p)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54GetDigiIn...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_GET_DIGI_IN, pDiData_p);

    TRACE("IOSTB: - cf54drv#CF54GetDigiIn (iRet=%d)\n\n", iRet);
    return;

}



//---------------------------------------------------------------------------
//  Set digital output lines
//---------------------------------------------------------------------------

void  PUBLIC  CF54SetDigiOut (
    tCF54DigiOut* pDoData_p)
{

int  iRet;


    TRACE("\nIOSTB: + cf54drv#CF54SetDigiOut...\n");

    iRet = ioctl (hDrvInst_l, CF54DRV_CMD_SET_DIGI_OUT, pDoData_p);

    TRACE("IOSTB: - cf54drv#CF54SetDigiOut (iRet=%d)\n\n", iRet);
    return;

}




// EOF


