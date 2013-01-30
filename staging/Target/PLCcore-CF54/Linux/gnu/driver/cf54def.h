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

#ifndef _CF54DEF_H_
#define _CF54DEF_H_



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

#define CF54DRV_RES_OK                  0
#define CF54DRV_RES_ERROR              -1
#define CF54DRV_RES_NOT_IMPLEMENTED    -2



//---------------------------------------------------------------------------
//  Commands for <ioctl>
//---------------------------------------------------------------------------

#define CF54DRV_CMD_INITIALIZE           0
#define CF54DRV_CMD_SHUTDOWN             1
#define CF54DRV_CMD_RESET_TARGET         2
#define CF54DRV_CMD_ENABLE_WATCHDOG      3
#define CF54DRV_CMD_SERVICE_WATCHDOG     4
#define CF54DRV_CMD_GET_HARDWARE_INFO    5
#define CF54DRV_CMD_SET_RUN_LED          6
#define CF54DRV_CMD_SET_ERR_LED          7
#define CF54DRV_CMD_GET_RSM_SWITCH       8
#define CF54DRV_CMD_GET_HEX_SWITCH       9
#define CF54DRV_CMD_GET_DIP_SWITCH      10
#define CF54DRV_CMD_GET_DIGI_IN         11
#define CF54DRV_CMD_SET_DIGI_OUT        12



#endif  // #ifndef _CF54DEF_H_

