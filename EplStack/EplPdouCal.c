/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for user PDO Communication Abstraction Layer module

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

  2009/09/03 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "user/EplPdouCal.h"
#include "user/EplEventu.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOU)) != 0)


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
/*          C L A S S  EplPdouCal                                          */
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

} tEplPdouCalInstance;
*/

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//static tEplPdouCalInstance  EplPdouCalInstance_g;

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
// Function:    EplPdouCalAddInstance()
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

tEplKernel EplPdouCalAddInstance(void)
{

//    EPL_MEMSET(&EplPdouCalInstance_g, 0, sizeof(EplPdouCalInstance_g));

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    EplPdouCalDelInstance()
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

tEplKernel EplPdouCalDelInstance(void)
{

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EplPdouCalAlloc()
//
// Description: This function allocates memory for PDOs according to the specified parameter.
//
// Parameters:  pAllocationParam_p      =
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdouCalAlloc(tEplPdoAllocationParam* pAllocationParam_p)
{
tEplKernel  Ret = kEplSuccessful;
tEplEvent   Event;

    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdokAlloc;
    Event.m_pArg = pAllocationParam_p;
    Event.m_uiSize = sizeof (*pAllocationParam_p);

    Ret = EplEventuPost(&Event);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplPdouCalConfigureChannel()
//
// Description: This function configures the specified PDO channel.
//
// Parameters:  pChannelConf_p          = PDO channel configuration
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplPdouCalConfigureChannel(tEplPdoChannelConf* pChannelConf_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplEvent       Event;
unsigned int    uiSize;

    Event.m_EventSink = kEplEventSinkPdokCal;
    Event.m_EventType = kEplEventTypePdokConfig;
    Event.m_pArg = pChannelConf_p;
    uiSize = memberoffs(tEplPdoChannelConf, m_aMappObject)
             + (pChannelConf_p->m_PdoChannel.m_uiMappObjectCount * sizeof (pChannelConf_p->m_aMappObject[0]));
    Event.m_uiSize = uiSize;

    Ret = EplEventuPost(&Event);

    return Ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:
//
// Description:
//
//
//
// Parameters:
//
//
// Returns:
//
//
// State:
//
//---------------------------------------------------------------------------

#endif

// EOF

