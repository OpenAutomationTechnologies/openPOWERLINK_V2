/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for kernel PDO Communication Abstraction Layer module

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

  2006/06/27 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "kernel/EplPdokCal.h"
#include "kernel/EplPdok.h"
#include "kernel/EplDllk.h"
#include "kernel/EplEventk.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)


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
/*          C L A S S  EplPdokCal                                          */
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

/*
typedef struct
{

} tEplPdokCalInstance;
*/

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//static tEplPdokCalInstance  EplPdokCalInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplPdokCalCbProcessRpdo(tEplFrameInfo * pFrameInfo_p);


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplPdokCalAddInstance()
//
// Description: add and initialize new instance of EPL stack
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdokCalAddInstance(void)
{
tEplKernel      Ret = kEplSuccessful;

//    EPL_MEMSET(&EplPdokCalInstance_g, 0, sizeof(EplPdokCalInstance_g));

    Ret = EplDllkRegRpdoHandler(EplPdokCalCbProcessRpdo);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplPdokCalDelInstance()
//
// Description: deletes an instance of EPL stack
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdokCalDelInstance(void)
{

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdokCalProcess
//
// Description: This function processes events from PdouCal module.
//
// Parameters:  pEvent_p                = pointer to event structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdokCalProcess(tEplEvent * pEvent_p)
{
tEplKernel      Ret = kEplSuccessful;

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypePdokAlloc:
        {
        tEplPdoAllocationParam* pAllocationParam;

            pAllocationParam = (tEplPdoAllocationParam*) pEvent_p->m_pArg;
            Ret = EplPdokAlloc(pAllocationParam);
            break;
        }

        case kEplEventTypePdokConfig:
        {
        tEplPdoChannelConf* pChannelConf;

            pChannelConf = (tEplPdoChannelConf*) pEvent_p->m_pArg;
            Ret = EplPdokConfigureChannel(pChannelConf);
            break;
        }

        case kEplEventTypePdoRx:  // RPDO received
        {
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
        tEplFrameInfo*  pFrameInfo;

            pFrameInfo = (tEplFrameInfo *) pEvent_p->m_pArg;

            Ret = EplPdokPdoDecode(pFrameInfo->m_pFrame, pFrameInfo->m_uiFrameSize);
#else
        tEplFrame*  pFrame;

            pFrame = (tEplFrame *) pEvent_p->m_pArg;

            Ret = EplPdokPdoDecode(pFrame, pEvent_p->m_uiSize);
#endif

            break;
        }

        default:
        {
            Ret = kEplInvalidEvent;
            break;
        }
    }

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplPdokCalCbProcessRpdo
//
// Description: This function is called by DLL if PRes or PReq frame was
//              received. It posts the frame to the event queue.
//              It is called in states NMT_CS_READY_TO_OPERATE and NMT_CS_OPERATIONAL.
//              The passed PDO needs not to be valid.
//
// Parameters:  pFrameInfo_p            = pointer to frame info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplPdokCalCbProcessRpdo(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplEvent       Event;

    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdoRx;
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
    Event.m_uiSize    = sizeof(tEplFrameInfo);
    Event.m_pArg      = pFrameInfo_p;
#else
    // limit copied data to size of PDO (because from some CNs the frame is larger than necessary)
    Event.m_uiSize = AmiGetWordFromLe(&pFrameInfo_p->m_pFrame->m_Data.m_Pres.m_le_wSize) + EPL_FRAME_OFFSET_PDO_PAYLOAD; // pFrameInfo_p->m_uiFrameSize;
    Event.m_pArg = pFrameInfo_p->m_pFrame;
#endif
    Ret = EplEventkPost(&Event);
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
    if (Ret == kEplSuccessful)
    {
        Ret = kEplReject; // Reject release of rx buffer
    }
#endif

    return Ret;
}


#endif

// EOF

