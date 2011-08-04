/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Syncu-Module

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

  2010-05-27 m.u.:   start of the implementation

****************************************************************************/

#if EPL_DLL_PRES_CHAINING_MN != FALSE

#include "user/EplSyncu.h"
#include "user/EplDlluCal.h"

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
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


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

typedef struct
{
    tEplSyncuCbResponse m_apfnCbResponse[254];

} tEplSyncuInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplSyncuInstance   EplSyncuInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplSyncuCbSyncResponse(tEplFrameInfo * pFrameInfo_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplSyncuInit
//
// Description: init first instance of the module
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplSyncuInit()
{
tEplKernel Ret;

    Ret = EplSyncuAddInstance();

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplSyncuAddInstance
//
// Description: init other instances of the module
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplSyncuAddInstance()
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplSyncuInstance_g, 0, sizeof (EplSyncuInstance_g));

    // register SyncResponse callback function
    Ret = EplDlluCalRegAsndService(kEplDllAsndSyncResponse, EplSyncuCbSyncResponse, kEplDllAsndFilterAny);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplSyncuDelInstance
//
// Description: delete instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplSyncuDelInstance()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // deregister SyncResponse callback function
    Ret = EplDlluCalRegAsndService(kEplDllAsndSyncResponse, NULL, kEplDllAsndFilterNone);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplSyncuReset
//
// Description: resets this instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplSyncuReset()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplSyncuInstance_g, 0, sizeof (EplSyncuInstance_g));

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplSyncuRequestSyncResponse
//
// Description: returns the SyncResponse for the specified node.
//
// Parameters:  pfnCbResponse_p     = IN: function pointer to callback function
//                                        which will be called if SyncResponse is received
//              pSyncRequestData_p  = IN: pointer to SyncRequest data structure
//              uiSize_p            = IN: only the first part of the SyncRequest
//                                        data structure until size bytes is used
//
// Return:      tEplKernel                  = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplSyncuRequestSyncResponse(
                                  tEplSyncuCbResponse pfnCbResponse_p,
                                  tEplDllSyncRequest* pSyncRequestData_p,
                                  unsigned int        uiSize_p)
{
tEplKernel      Ret;
unsigned int    uiNodeId;

    Ret = kEplSuccessful;
    uiNodeId = pSyncRequestData_p->m_uiNodeId;

    if (uiNodeId == 0)
    {
        Ret = kEplInvalidNodeId;
        goto Exit;
    }

    // decrement node ID, because array is zero based
    uiNodeId--;
    if (uiNodeId < tabentries (EplSyncuInstance_g.m_apfnCbResponse))
    {
        if (EplSyncuInstance_g.m_apfnCbResponse[uiNodeId] != NULL)
        {   // request already issued (maybe by someone else)
            Ret = kEplNmtSyncReqRejected;
        }
        else
        {
            EplSyncuInstance_g.m_apfnCbResponse[uiNodeId] = pfnCbResponse_p;
            Ret = EplDlluCalIssueSyncRequest(pSyncRequestData_p, uiSize_p);
        }
    }
    else
    {   // invalid node ID specified
        Ret = kEplInvalidNodeId;
    }

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
// Function:    EplSyncuCbSyncResponse
//
// Description: callback funktion for SyncResponse
//
// Parameters:  pFrameInfo_p            = Frame with the SyncResponse
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel PUBLIC EplSyncuCbSyncResponse(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel          Ret;
unsigned int        uiNodeId;
unsigned int        uiIndex;
tEplSyncuCbResponse pfnCbResponse;

    Ret = kEplSuccessful;

    uiNodeId = AmiGetByteFromLe(&pFrameInfo_p->m_pFrame->m_le_bSrcNodeId);
    uiIndex  = uiNodeId - 1;

    if (uiIndex < tabentries (EplSyncuInstance_g.m_apfnCbResponse))
    {
        // memorize pointer to callback function
        pfnCbResponse = EplSyncuInstance_g.m_apfnCbResponse[uiIndex];
        if (pfnCbResponse == NULL)
        {   // response was not requested
            goto Exit;
        }
        // reset callback function pointer so that caller may issue next request
        EplSyncuInstance_g.m_apfnCbResponse[uiIndex] = NULL;

        if (pFrameInfo_p->m_uiFrameSize < EPL_C_DLL_MINSIZE_SYNCRES)
        {   // SyncResponse not received or it has invalid size
            Ret = pfnCbResponse(uiNodeId, NULL);
        }
        else
        {   // SyncResponse received
            Ret = pfnCbResponse(uiNodeId, &pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse);
        }
    }

Exit:
    return Ret;
}

#endif // EPL_DLL_PRES_CHAINING_MN != FALSE

// EOF

