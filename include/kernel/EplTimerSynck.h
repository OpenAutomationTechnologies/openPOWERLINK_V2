/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for EPL timer synchronization module

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

  2009/09/24 m.u.:   start of the implementation


****************************************************************************/

#ifndef _EPLTIMERSYNCK_H_
#define _EPLTIMERSYNCK_H_

#include "EplInc.h"

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


typedef tEplKernel (* tEplTimerSynckCbSync) (void);
typedef tEplKernel (* tEplTimerSynckCbLossOfSync) (void);


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerSynckAddInstance(void);

tEplKernel PUBLIC EplTimerSynckDelInstance(void);

tEplKernel PUBLIC EplTimerSynckRegSyncHandler(tEplTimerSynckCbSync pfnTimerSynckCbSync_p);

tEplKernel PUBLIC EplTimerSynckRegLossOfSyncHandler(tEplTimerSynckCbLossOfSync pfnTimerSynckCbLossOfSync_p);

tEplKernel PUBLIC EplTimerSynckRegLossOfSyncHandler2(tEplTimerSynckCbLossOfSync pfnTimerSynckCbLossOfSync2_p);

tEplKernel PUBLIC EplTimerSynckSetSyncShiftUs(DWORD dwAdvanceShiftUs_p);

tEplKernel PUBLIC EplTimerSynckSetCycleLenUs(DWORD dwCycleLenUs_p);

tEplKernel PUBLIC EplTimerSynckSetLossOfSyncToleranceNs(DWORD dwLossOfSyncToleranceNs_p);

tEplKernel PUBLIC EplTimerSynckSetLossOfSyncTolerance2Ns(DWORD dwLossOfSyncTolerance2Ns_p);

tEplKernel PUBLIC EplTimerSynckTriggerAtTimeStamp(tEplTgtTimeStamp* pTimeStamp_p);

tEplKernel PUBLIC EplTimerSynckStopSync(void);


#endif  // #ifndef _EPLTIMERSYNCK_H_
