/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Part of Ethernet driver to implement time-triggered transmission.
                It is necessary to implement Managing Nodes.

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
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2010/03/18 d.k.:   start of implementation

****************************************************************************/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "kernel/EplTimerHighResk.h"

#if EPL_TIMER_USE_HIGHRES == FALSE
#error "EdrvCyclic needs EPL_TIMER_USE_HIGHRES = TRUE"
#endif



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

typedef struct
{
    tEdrvTxBuffer**     m_paTxBufferList;
    unsigned int        m_uiCurTxBufferCount;
    unsigned int        m_uiMaxTxBufferCount;
    unsigned int        m_uiCurTxBufferEntry;
    DWORD               m_dwCycleLenUs;
    tEplTimerHdl        m_TimerHdlCycle;

} tEdrvCyclicInstance;



//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// module globale vars
//---------------------------------------------------------------------------

static tEdrvCyclicInstance EdrvCyclicInstance_l;




/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <EdrvCyclic>                                        */
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

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------




//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicInit
//
// Description: initialize EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvCyclicInit()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvCyclicInstance_l, 0, sizeof (EdrvCyclicInstance_l));


Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicShutdown
//
// Description: Shutdown EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvCyclicShutdown(void)
{
    if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
    {
        EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
        EdrvCyclicInstance_l.m_paTxBufferList = NULL;
    }

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetMaxTxBufferListSize
//
// Description: Sets the maximum number of TxBuffer list entries.
//
// Parameters:  uiMaxListSize_p = maximum number of TxBuffer list entries.
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvCyclicSetMaxTxBufferListSize(unsigned int uiMaxListSize_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (EdrvCyclicInstance_l.m_uiMaxTxBufferCount != uiMaxListSize_p)
    {
        EdrvCyclicInstance_l.m_uiMaxTxBufferCount = uiMaxListSize_p;
        if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
        {
            EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
            EdrvCyclicInstance_l.m_paTxBufferList = NULL;
        }

        EdrvCyclicInstance_l.m_paTxBufferList = EPL_MALLOC(sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * uiMaxListSize_p * 2);
        if (EdrvCyclicInstance_l.m_paTxBufferList == NULL)
        {
            Ret = kEplEdrvNoFreeBufEntry;
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
// Function:    EdrvCyclicCbTimerCycle()
//
// Description: called by timer module. It starts the next cycle.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;


Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicCbTimerSlot()
//
// Description: called by timer module. It triggers the transmission of the next frame.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EdrvCyclicCbTimerSlot(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;


Exit:
    return Ret;
}


