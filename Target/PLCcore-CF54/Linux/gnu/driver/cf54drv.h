/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Board driver for SYSTEC ECUcore-5484

  Description:  Declarations for board driver

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

#ifndef _CF54DRV_H_
#define _CF54DRV_H_



//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------

#define SYSTEC_CF54DRV_MAJOR            124         // only used if _CFG_DYNMAJOR_ isn't defined

#define SYSTEC_CF54DRV_NAME_DEV         "/dev/cf54drv"
#define SYSTEC_CF54DRV_NAME_VAR         "/var/cf54drv"



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

#define ON              0x01            // LED on
#define OFF             0x00            // LED off

// RUN/STOP Switch:
#define SWITCH_MRES     0x01            // switch in position MRES
#define SWITCH_STOP     0x02            // switch in position STOP
#define SWITCH_RUN      0x03            // switch in position RUN

// driver configuration (used for tCF54HwInfo.m_wCfgDriver)
#define CFG_4158_DI1_4_VIA_CS3  0x0001  // if set -> JP300 can be opend
#define CFG_4158_DO1_4_VIA_CS3  0x0002  // if set -> JP302 must be set to 1-2



//---------------------------------------------------------------------------
//  Type definitions
//---------------------------------------------------------------------------

// Hardware information structure
typedef struct
{
    WORD    m_wCpuPcbVersion;
    BYTE    m_bCpuPcbRevision;
    BYTE    m_bCpuHwId;
    WORD    m_wCpuPldVersion;
    BYTE    m_bCpuPldRevision;
    BYTE    m_bCpuPldType;
    WORD    m_wIoPcbVersion;
    BYTE    m_bIoPcbRevision;
    BYTE    m_bIoHwId;
    WORD    m_wCfgDriver;

} tCF54HwInfo;


// Digital input structure
typedef struct
{
    BYTE    m_bDiByte0;
    BYTE    m_bDiByte1;
    BYTE    m_bDiByte2;
    BYTE    m_bDiByte3;

} tCF54DigiIn;


// Digital output structure
typedef struct
{
    BYTE    m_bDoByte0;
    BYTE    m_bDoByte1;
    BYTE    m_bDoByte2;
    BYTE    m_bDoByte3;

} tCF54DigiOut;



//---------------------------------------------------------------------------
//  Prototypes of driver functions
//---------------------------------------------------------------------------

WORD  PUBLIC  CF54Initialize      (void);
void  PUBLIC  CF54ShutDown        (void);
void  PUBLIC  CF54GetHardwareInfo (tCF54HwInfo* pHwInfo_p);
void  PUBLIC  CF54SetRunLed       (BYTE bState_p);
void  PUBLIC  CF54SetErrLed       (BYTE bState_p);
BYTE  PUBLIC  CF54GetRSMSwitch    (void);
BYTE  PUBLIC  CF54GetHexSwitch    (void);
BYTE  PUBLIC  CF54GetDipSwitch    (void);
void  PUBLIC  CF54GetDigiIn       (tCF54DigiIn* pDiData_p);
void  PUBLIC  CF54SetDigiOut      (tCF54DigiOut* pDoData_p);




#endif  // #ifndef _CF54DRV_H_

