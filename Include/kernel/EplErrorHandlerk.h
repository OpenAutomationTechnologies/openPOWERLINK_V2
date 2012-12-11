/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for kernel error handler module

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

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/10/02 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_ERRORHANDLERK_H_
#define _EPL_ERRORHANDLERK_H_

#include "event.h"

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EPL_DLL_ERR_MN_CRC              0x00000001L  ///< object 0x1C00
#define EPL_DLL_ERR_MN_COLLISION        0x00000002L  ///< object 0x1C01
#define EPL_DLL_ERR_MN_CYCTIMEEXCEED    0x00000004L  ///< object 0x1C02
#define EPL_DLL_ERR_MN_LOSS_LINK        0x00000008L  ///< object 0x1C03
#define EPL_DLL_ERR_MN_CN_LATE_PRES     0x00000010L  ///< objects 0x1C04-0x1C06
#define EPL_DLL_ERR_MN_CN_LOSS_PRES     0x00000080L  ///< objects 0x1C07-0x1C09
#define EPL_DLL_ERR_CN_COLLISION        0x00000400L  ///< object 0x1C0A
#define EPL_DLL_ERR_CN_LOSS_SOC         0x00000800L  ///< object 0x1C0B
#define EPL_DLL_ERR_CN_LOSS_SOA         0x00001000L  ///< object 0x1C0C
#define EPL_DLL_ERR_CN_LOSS_PREQ        0x00002000L  ///< object 0x1C0D
#define EPL_DLL_ERR_CN_RECVD_PREQ       0x00004000L  ///< decrement object 0x1C0D/2
#define EPL_DLL_ERR_CN_SOC_JITTER       0x00008000L  ///< object 0x1C0E
#define EPL_DLL_ERR_CN_CRC              0x00010000L  ///< object 0x1C0F
#define EPL_DLL_ERR_CN_LOSS_LINK        0x00020000L  ///< object 0x1C10
#define EPL_DLL_ERR_MN_LOSS_STATRES     0x00040000L  ///< objects 0x1C15-0x1C17 (should be operated by NmtMnu module)
#define EPL_DLL_ERR_BAD_PHYS_MODE       0x00080000L  ///< no object
#define EPL_DLL_ERR_MAC_BUFFER          0x00100000L  ///< no object (NMT_GT6)
#define EPL_DLL_ERR_INVALID_FORMAT      0x00200000L  ///< no object (NMT_GT6)
#define EPL_DLL_ERR_ADDRESS_CONFLICT    0x00400000L  ///< no object (remove CN from configuration)

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

// init function
tEplKernel PUBLIC EplErrorHandlerkInit(void);

// add instance
tEplKernel PUBLIC EplErrorHandlerkAddInstance(void);

// delete instance
tEplKernel PUBLIC EplErrorHandlerkDelInstance(void);

// processes error events
tEplKernel PUBLIC EplErrorHandlerkProcess(tEplEvent * pEvent_p);

// posts error events
tEplKernel PUBLIC EplErrorHandlerkPostError(tEplErrorHandlerkEvent* pDllEvent_p);

// cycle finished (decrement threshold counters)
tEplKernel PUBLIC EplErrorHandlerkCycleFinished(BOOL fMN_p);

// reset error flag for the specified CN
tEplKernel PUBLIC EplErrorHandlerkResetCnError(unsigned int uiNodeId_p);


#endif  // #ifndef _EPL_ERRORHANDLERK_H_


