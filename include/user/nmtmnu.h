/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for NMT-MN-Userspace-Module

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

  2006/06/09 k.t.:   start of the implementation


****************************************************************************/

#ifndef _NMTMNU_H_
#define _NMTMNU_H_

#include "EplNmtu.h"


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------

typedef tEplKernel (PUBLIC * tEplNmtMnuCbNodeEvent) (
    UINT nodeId_p,
    tEplNmtNodeEvent NodeEvent_p,
    tEplNmtState NmtState_p,
    UINT16 wErrorCode_p,
    BOOL fMandatory_p);


typedef tEplKernel (PUBLIC * tEplNmtMnuCbBootEvent) (
    tEplNmtBootEvent BootEvent_p,
    tEplNmtState NmtState_p,
    UINT16 wErrorCode_p);


typedef struct
{
    UINT32   prcPResTimeFirstCorrectionNs;
    UINT32   prcPResTimeFirstNegOffsetNs;
} tEplNmtMnuConfigParam;


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

tEplKernel EplNmtMnuInit(tEplNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                         tEplNmtMnuCbBootEvent pfnCbBootEvent_p);

tEplKernel EplNmtMnuAddInstance(tEplNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                                tEplNmtMnuCbBootEvent pfnCbBootEvent_p);

tEplKernel EplNmtMnuDelInstance(void);

EPLDLLEXPORT tEplKernel PUBLIC EplNmtMnuProcessEvent(
            tEplEvent* pEvent_p);

tEplKernel EplNmtMnuSendNmtCommand(UINT nodeId_p,
                                   tEplNmtCommand  nmtCommand_p);

tEplKernel EplNmtMnuRequestNmtCommand(UINT nodeId_p,
                                    tEplNmtCommand  nmtCommand_p);

tEplKernel EplNmtMnuTriggerStateChange(UINT nodeId_p,
                                       tEplNmtNodeCommand  nodeCommand_p);

tEplKernel PUBLIC EplNmtMnuCbNmtStateChange(tEplEventNmtStateChange nmtStateChange_p);

tEplKernel PUBLIC EplNmtMnuCbCheckEvent(tEplNmtEvent NmtEvent_p);

tEplKernel PUBLIC EplNmtMnuGetDiagnosticInfo(UINT* pMandatorySlaveCount_p,
                                             UINT* pSignalSlaveCount_p,
                                             UINT16* pflags_p);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
tEplKernel PUBLIC EplNmtMnuPrcConfig(tEplNmtMnuConfigParam* pConfigParam_p);
#endif

#endif

#endif  // #ifndef _NMTMNU_H_


