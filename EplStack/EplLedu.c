/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for LED user part module.

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

#include "EplInc.h"
#include "user/EplLedu.h"
#include "user/EplTimeru.h"

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)

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

#define EPL_LEDU_DURATION_FLICKERING    50      // [ms]
#define EPL_LEDU_DURATION_BLINKING      200     // [ms]
#define EPL_LEDU_DURATION_FLASH_ON      200     // [ms]
#define EPL_LEDU_DURATION_FLASH_OFF     1000    // [ms]

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef enum
{
    kEplLeduModeInit        = 0x00,
    kEplLeduModeOff         = 0x01,
    kEplLeduModeOn          = 0x02,
    kEplLeduModeFlickering  = 0x03,
    kEplLeduModeBlinking    = 0x04,
    kEplLeduModeSingleFlash = 0x05,
    kEplLeduModeDoubleFlash = 0x06,
    kEplLeduModeTripleFlash = 0x07,

} tEplLeduMode;


typedef struct
{
    tEplTimerHdl                m_TimerHdlLedBlink; // timer for LED blinking
    DWORD                       m_dwTimerArg;
    tEplLeduStateChangeCallback m_pfnCbStateChange;
    tEplLeduMode                m_StatusLedMode;
    unsigned int                m_uiStatusLedState;
        // 0 - long off (e.g. 50 ms while flickering, 200 ms while blinking and
        //               1000 ms while flashing)
        // 1 - on (odd number)
        // 2 - short off (even number)

} tEplLeduInstance;


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEplLeduInstance   EplLeduInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplLeduCallStateChanged(
            tEplLedType LedType_p, BOOL fOn_p);

static tEplKernel PUBLIC EplLeduChangeMode(
            tEplLeduMode NewMode_p);



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplLeduInit
//
// Description: init the first instance of the module
//
// Parameters:  pfnCbStateChange_p  = callback function for LED state changes
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplLeduInit(tEplLeduStateChangeCallback pfnCbStateChange_p)
{
tEplKernel Ret;

    Ret = EplLeduAddInstance(pfnCbStateChange_p);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplLeduAddInstance
//
// Description: init the add new instance of the module
//
// Parameters:  pfnCbStateChange_p  = callback function for LED state changes
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplLeduAddInstance(tEplLeduStateChangeCallback pfnCbStateChange_p)
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplLeduInstance_g, 0, sizeof (EplLeduInstance_g));

    // save callback function pointer
    EplLeduInstance_g.m_pfnCbStateChange = pfnCbStateChange_p;



    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplLeduDelInstance
//
// Description: delete instance of the module
//
// Parameters:
//
//
// Returns:     tEplKernel = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplLeduDelInstance(void)
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    Ret = EplTimeruDeleteTimer(&EplLeduInstance_g.m_TimerHdlLedBlink);

    // reset instance structure
    EPL_MEMSET(&EplLeduInstance_g, 0, sizeof (EplLeduInstance_g));

    return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    EplLeduCbNmtStateChange
//
// Description: callback function for NMT state changes
//
// Parameters:  NmtStateChange_p        = NMT state change event
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplLeduCbNmtStateChange(tEplEventNmtStateChange NmtStateChange_p)
{
tEplKernel      Ret = kEplSuccessful;

    // activate status LED according to NMT state
    switch (NmtStateChange_p.m_NewNmtState)
    {
        // status LED off
        case kEplNmtGsOff:
        case kEplNmtGsInitialising:
        case kEplNmtGsResetApplication:
        case kEplNmtGsResetCommunication:
        case kEplNmtGsResetConfiguration:
        case kEplNmtCsNotActive:
        case kEplNmtMsNotActive:
        {
            if (EplLeduInstance_g.m_StatusLedMode != kEplLeduModeOff)
            {   // state changed
                EplLeduInstance_g.m_StatusLedMode = kEplLeduModeOff;
                Ret = EplTimeruDeleteTimer(&EplLeduInstance_g.m_TimerHdlLedBlink);
                Ret = EplLeduCallStateChanged(kEplLedTypeStatus, FALSE);
            }
            break;
        }

        // status LED single flashing
        case kEplNmtCsPreOperational1:
        case kEplNmtMsPreOperational1:
        {
            Ret = EplLeduChangeMode(kEplLeduModeSingleFlash);
            break;
        }

        // status LED double flashing
        case kEplNmtCsPreOperational2:
        case kEplNmtMsPreOperational2:
        {
            Ret = EplLeduChangeMode(kEplLeduModeDoubleFlash);
            break;
        }

        // status LED triple flashing
        case kEplNmtCsReadyToOperate:
        case kEplNmtMsReadyToOperate:
        {
            Ret = EplLeduChangeMode(kEplLeduModeTripleFlash);
            break;
        }

        // status LED on
        case kEplNmtCsOperational:
        case kEplNmtMsOperational:
        {
            if (EplLeduInstance_g.m_StatusLedMode != kEplLeduModeOn)
            {   // state changed
                EplLeduInstance_g.m_StatusLedMode = kEplLeduModeOn;
                Ret = EplTimeruDeleteTimer(&EplLeduInstance_g.m_TimerHdlLedBlink);
                Ret = EplLeduCallStateChanged(kEplLedTypeStatus, TRUE);
            }
            break;
        }

        // status LED blinking
        case kEplNmtCsStopped:
        {
            Ret = EplLeduChangeMode(kEplLeduModeBlinking);
            break;
        }

        // status LED flickering
        case kEplNmtCsBasicEthernet:
        case kEplNmtMsBasicEthernet:
        {
            Ret = EplLeduChangeMode(kEplLeduModeFlickering);
            break;
        }

    }

    // activate error LED according to NMT event
    switch (NmtStateChange_p.m_NmtEvent)
    {
        // error LED off
        case kEplNmtEventSwReset:               // NMT_GT2
        case kEplNmtEventStartNode:             // NMT_CT7
        case kEplNmtEventTimerBasicEthernet:    // NMT_CT3
        case kEplNmtEventEnterMsOperational:    // NMT_MT5
        {
            Ret = EplLeduCallStateChanged(kEplLedTypeError, FALSE);
            break;
        }

        // error LED on
        case kEplNmtEventNmtCycleError:     // NMT_CT11, NMT_MT6
        case kEplNmtEventInternComError:    // NMT_GT6
        {
            Ret = EplLeduCallStateChanged(kEplLedTypeError, TRUE);
            break;
        }

        default:
        {   // do nothing
            break;
        }

    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplLeduProcessEvent
//
// Description: processes events from event queue
//
// Parameters:  pEvent_p        = pointer to event
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplLeduProcessEvent(
            tEplEvent* pEvent_p)
{
tEplKernel      Ret;
tEplTimerArg    TimerArg;
DWORD           dwTimeout = 0;
BOOL            fLedOn = FALSE;

    Ret = kEplSuccessful;

    // process event
    switch(pEvent_p->m_EventType)
    {
        // timer event
        case kEplEventTypeTimer:
        {
        tEplTimerEventArg*  pTimerEventArg = (tEplTimerEventArg*)pEvent_p->m_pArg;

            if (pTimerEventArg->m_Arg.m_dwVal != EplLeduInstance_g.m_dwTimerArg)
            {   // zombie timer
                // ignore it
                break;
            }

            // increment status LED state
            EplLeduInstance_g.m_uiStatusLedState++;

            // select timeout and new LED state corresponding to mode
            switch (EplLeduInstance_g.m_StatusLedMode)
            {
                case kEplLeduModeInit:
                case kEplLeduModeOn:
                case kEplLeduModeOff:
                {   // should not occur
                    goto Exit;
                }

                case kEplLeduModeFlickering:
                {
                    if (EplLeduInstance_g.m_uiStatusLedState >= 2)
                    {   // reset state
                        EplLeduInstance_g.m_uiStatusLedState = 0;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        fLedOn = TRUE;
                    }

                    dwTimeout = EPL_LEDU_DURATION_FLICKERING;
                    break;
                }

                case kEplLeduModeBlinking:
                {
                    if (EplLeduInstance_g.m_uiStatusLedState >= 2)
                    {   // reset state
                        EplLeduInstance_g.m_uiStatusLedState = 0;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        fLedOn = TRUE;
                    }

                    dwTimeout = EPL_LEDU_DURATION_BLINKING;
                    break;
                }

                case kEplLeduModeSingleFlash:
                {
                    if (EplLeduInstance_g.m_uiStatusLedState >= 2)
                    {   // reset state
                        EplLeduInstance_g.m_uiStatusLedState = 0;
                        dwTimeout = EPL_LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        dwTimeout = EPL_LEDU_DURATION_FLASH_ON;
                        fLedOn = ((EplLeduInstance_g.m_uiStatusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }

                    break;
                }

                case kEplLeduModeDoubleFlash:
                {
                    if (EplLeduInstance_g.m_uiStatusLedState >= 4)
                    {   // reset state
                        EplLeduInstance_g.m_uiStatusLedState = 0;
                        dwTimeout = EPL_LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        dwTimeout = EPL_LEDU_DURATION_FLASH_ON;
                        fLedOn = ((EplLeduInstance_g.m_uiStatusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }

                    break;
                }

                case kEplLeduModeTripleFlash:
                {
                    if (EplLeduInstance_g.m_uiStatusLedState >= 6)
                    {   // reset state
                        EplLeduInstance_g.m_uiStatusLedState = 0;
                        dwTimeout = EPL_LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        dwTimeout = EPL_LEDU_DURATION_FLASH_ON;
                        fLedOn = ((EplLeduInstance_g.m_uiStatusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }

                    break;
                }
            }

            // create new timer
            TimerArg.m_EventSink = kEplEventSinkLedu;
            EplLeduInstance_g.m_dwTimerArg++;
            TimerArg.m_Arg.m_dwVal = EplLeduInstance_g.m_dwTimerArg;
            Ret = EplTimeruModifyTimerMs(&EplLeduInstance_g.m_TimerHdlLedBlink,
                                         dwTimeout,
                                         TimerArg);

            // call callback function
            Ret = EplLeduCallStateChanged(kEplLedTypeStatus, fLedOn);

            break;
        }

        default:
        {
            Ret = kEplNmtInvalidEvent;
        }

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
// Function:    EplLeduCallStateChanged
//
// Description: call the registered state change callback function
//
// Parameters:  LedType_p       = type of LED
//              fOn_p           = state of LED
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplLeduCallStateChanged(
            tEplLedType LedType_p, BOOL fOn_p)
{
tEplKernel      Ret;

    Ret = kEplSuccessful;

    if (EplLeduInstance_g.m_pfnCbStateChange != NULL)
    {
        Ret = EplLeduInstance_g.m_pfnCbStateChange(LedType_p, fOn_p);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplLeduChangeMode
//
// Description: call the registered state change callback function
//
// Parameters:  LedType_p       = type of LED
//              fOn_p           = state of LED
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplLeduChangeMode(
            tEplLeduMode NewMode_p)
{
tEplKernel      Ret;
tEplLeduMode    OldMode;
tEplTimerArg    TimerArg;
DWORD           dwTimeout;
BOOL            fLedOn;

    Ret = kEplSuccessful;

    OldMode = EplLeduInstance_g.m_StatusLedMode;

    if (OldMode != NewMode_p)
    {   // state changed
        // save new mode
        EplLeduInstance_g.m_StatusLedMode = NewMode_p;

        // Where are we coming from?
        if (OldMode == kEplLeduModeOff)
        {   // status LED was off

            // switch LED on
            fLedOn = TRUE;
            EplLeduInstance_g.m_uiStatusLedState = 0xFF;
        }
        else if (OldMode == kEplLeduModeOn)
        {   // status LED was on

            // switch LED off
            fLedOn = FALSE;
            EplLeduInstance_g.m_uiStatusLedState = 0;
        }
        else
        {   // timer should be up and running
            goto Exit;
        }

        // select timeout corresponding to mode
        switch (NewMode_p)
        {
            default:
            {   // should not occur
                goto Exit;
            }

            case kEplLeduModeFlickering:
            {
                dwTimeout = EPL_LEDU_DURATION_FLICKERING;
                break;
            }

            case kEplLeduModeBlinking:
            {
                dwTimeout = EPL_LEDU_DURATION_BLINKING;
                break;
            }

            case kEplLeduModeSingleFlash:
            case kEplLeduModeDoubleFlash:
            case kEplLeduModeTripleFlash:
            {
                if (fLedOn == FALSE)
                {
                    dwTimeout = EPL_LEDU_DURATION_FLASH_OFF;
                }
                else
                {
                    dwTimeout = EPL_LEDU_DURATION_FLASH_ON;
                }

                break;
            }

        }

        // create new timer
        TimerArg.m_EventSink = kEplEventSinkLedu;
        EplLeduInstance_g.m_dwTimerArg++;
        TimerArg.m_Arg.m_dwVal = EplLeduInstance_g.m_dwTimerArg;
        Ret = EplTimeruModifyTimerMs(&EplLeduInstance_g.m_TimerHdlLedBlink,
                                     dwTimeout,
                                     TimerArg);

        // call callback function
        Ret = EplLeduCallStateChanged(kEplLedTypeStatus, fLedOn);
    }

Exit:
    return Ret;
}


#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)

// EOF

