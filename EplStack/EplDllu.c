/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for userspace DLL module for asynchronous communication

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

  2006/06/20 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "user/EplDllu.h"
#include "user/EplDlluCal.h"

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLU)) != 0)
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
/*          C L A S S  EplDllu                                             */
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
    tEplDlluCbAsnd  m_apfnDlluCbAsnd[EPL_DLL_MAX_ASND_SERVICE_ID];

} tEplDlluInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

// if no dynamic memory allocation shall be used
// define structures statically
static tEplDlluInstance     EplDlluInstance_g;

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
// Function:    EplDlluAddInstance()
//
// Description: add and initialize new instance of EPL stack
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDlluAddInstance()
{
tEplKernel      Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplDlluInstance_g, 0, sizeof (EplDlluInstance_g));

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDlluDelInstance()
//
// Description: deletes instance of EPL stack
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDlluDelInstance()
{
tEplKernel      Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplDlluInstance_g, 0, sizeof (EplDlluInstance_g));

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDlluProcess
//
// Description: process the passed asynch frame
//
// Parameters:  pFrameInfo_p            = frame to be processed
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDlluProcess(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplMsgType     MsgType;
unsigned int    uiAsndServiceId;

    MsgType = (tEplMsgType)AmiGetByteFromLe(&pFrameInfo_p->m_pFrame->m_le_bMessageType);
    if (MsgType != kEplMsgTypeAsnd)
    {
        Ret = kEplInvalidOperation; // $$$ kEplDllInvalidFrame
        goto Exit;
    }

    uiAsndServiceId = (unsigned int) AmiGetByteFromLe(&pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_le_bServiceId);
    if (uiAsndServiceId < EPL_DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid
        if (EplDlluInstance_g.m_apfnDlluCbAsnd[uiAsndServiceId] != NULL)
        {   // handler was registered
            Ret = EplDlluInstance_g.m_apfnDlluCbAsnd[uiAsndServiceId](pFrameInfo_p);
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDlluRegAsndService()
//
// Description: registers the specified handler for the specified
//              AsndServiceId with the specified node ID filter.
//
// Parameters:  ServiceId_p             = ASnd Service ID
//              pfnDlluCbAsnd_p         = callback function
//              Filter_p                = node ID filter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDlluRegAsndService(tEplDllAsndServiceId ServiceId_p, tEplDlluCbAsnd pfnDlluCbAsnd_p, tEplDllAsndFilter Filter_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (ServiceId_p < sizeof (EplDlluInstance_g.m_apfnDlluCbAsnd))
    {
        // memorize function pointer
        EplDlluInstance_g.m_apfnDlluCbAsnd[ServiceId_p] = pfnDlluCbAsnd_p;

        if (pfnDlluCbAsnd_p == NULL)
        {   // close filter
            Filter_p = kEplDllAsndFilterNone;
        }

        // set filter in kernelspace DLL module
        Ret = EplDlluCalSetAsndServiceIdFilter(ServiceId_p, Filter_p);

    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDlluAsyncSend()
//
// Description: sends the frame with the specified priority.
//
// Parameters:  pFrameInfo_p            = frame
//                                        m_uiFrameSize does not include the
//                                        ethernet header (14 bytes)
//              Priority_p              = priority
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDlluAsyncSend(tEplFrameInfo * pFrameInfo_p, tEplDllAsyncReqPriority Priority_p)
{
tEplKernel  Ret = kEplSuccessful;

    Ret = EplDlluCalAsyncSend(pFrameInfo_p, Priority_p);

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLU)) != 0)

// EOF

