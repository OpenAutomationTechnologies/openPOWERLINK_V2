/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com


  Project:      openPOWERLINK

  Description:  Implementiation of target specific time stamp functions

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

****************************************************************************/

#include "EplInc.h"
#include "EplTgtTimeStamp_openMac.h"

#include "omethlib.h"


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

//---------------------------------------------------------------------------
// module global types
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
// Function:    EplTgtTimeStampTimeDiffNs()
//
// Description: calculates the time difference between two time stamps
//              and converts the result to [ns].
//
// Parameters:  pTimeStampPredecessor_p = pointer to time stamp which was
//                                        taken earlier than pTimeStampSuccessor_p
//              pTimeStampSuccessor_p   = pointer to time stamp which was
//                                        taken later than pTimeStampPredecessor_p
//
// Return:      time difference in [ns]
//
// State:       not tested
//
//---------------------------------------------------------------------------

DWORD PUBLIC EplTgtTimeStampTimeDiffNs (tEplTgtTimeStamp* pTimeStampPredecessor_p,
                                        tEplTgtTimeStamp* pTimeStampSuccessor_p)
{
DWORD dwTimeDiffTicks;

    dwTimeDiffTicks = pTimeStampSuccessor_p->m_dwTimeStamp
                      - pTimeStampPredecessor_p->m_dwTimeStamp;

    return OMETH_TICKS_2_NS(dwTimeDiffTicks);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtTimeStampAlloc()
//
// Description: allocates memory for one target specific time stamp
//
// Parameters:  void
//
// Return:      pointer to allocated memory
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplTgtTimeStamp* PUBLIC EplTgtTimeStampAlloc (void)
{
tEplTgtTimeStamp* pTimeStamp;

    pTimeStamp = malloc(sizeof (struct _tEplTgtTimeStamp));

    return pTimeStamp;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtTimeStampFree()
//
// Description: frees memory of one previously allocated
//              target specific time stamp
//
// Parameters:  pTimeStamp_p    = pointer to memory which is to be freed
//
// Return:      void
//
// State:       not tested
//
//---------------------------------------------------------------------------

void PUBLIC EplTgtTimeStampFree (tEplTgtTimeStamp* pTimeStamp_p)
{

    free(pTimeStamp_p);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtTimeStampCopy()
//
// Description: copies one target specific time stamp to another
//
// Parameters:  pTimeStampDest_p    = pointer to destination time stamp where
//                                    the source time stamp is copied to
//              pTimeStampSrc_p     = pointer to source time stamp
//
// Return:      void
//
// State:       not tested
//
//---------------------------------------------------------------------------

void PUBLIC EplTgtTimeStampCopy (tEplTgtTimeStamp* pTimeStampDest_p,
                                 tEplTgtTimeStamp* pTimeStampSrc_p)
{

    memcpy(pTimeStampDest_p, pTimeStampSrc_p, sizeof(struct _tEplTgtTimeStamp));

}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//
