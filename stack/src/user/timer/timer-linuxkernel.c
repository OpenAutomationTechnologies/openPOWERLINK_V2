/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for EPL User Timermodule for Linux kernel module

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
                KEIL uVision 2

  -------------------------------------------------------------------------

  Revision History:

  2006/09/12 d.k.:   start of the implementation

****************************************************************************/

#include "user/timeru.h"
#include <linux/timer.h>

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
    struct timer_list   m_Timer;
    tEplTimerArg        TimerArgument;

} tEplTimeruData;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static void PUBLIC EplTimeruCbMs(unsigned long ulParameter_p);

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl Userspace-Timermodule for Linux Kernel>              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: Epl Userspace-Timermodule for Linux Kernel
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    timeru_init
//
// Description: function inits first instance
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_init(void)
{
tEplKernel  Ret;

    Ret = timeru_addInstance();

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_addInstance
//
// Description: function inits additional instance
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_addInstance(void)
{
tEplKernel Ret;

    Ret = kEplSuccessful;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_delInstance
//
// Description: function deletes instance
//              -> under Linux nothing to do
//              -> no instance table needed
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_delInstance(void)
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_process
//
// Description: This function is called repeatedly from within the main
//              loop of the application. It checks whether the first timer
//              entry has been elapsed.
//
// Parameters:  none
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_process(void)
{
    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_setTimer
//
// Description: function creates a timer and returns the corresponding handle
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              timeInMs_p    = time for timer in ms
//              argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_setTimer(tEplTimerHdl* pTimerHdl_p, ULONG timeInMs_p, tEplTimerArg argument_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplTimeruData*     pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pData = (tEplTimeruData*) EPL_MALLOC(sizeof (tEplTimeruData));
    if (pData == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    init_timer(&pData->m_Timer);
    pData->m_Timer.function = EplTimeruCbMs;
    pData->m_Timer.data = (unsigned long) pData;
    pData->m_Timer.expires = jiffies + 1 + ((timeInMs_p * HZ) + 999) / 1000;

    EPL_MEMCPY(&pData->TimerArgument, &argument_p, sizeof(tEplTimerArg));

    add_timer(&pData->m_Timer);

    *pTimerHdl_p = (tEplTimerHdl) pData;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_modifyTimer
//
// Description: function changes a timer and returns the corresponding handle
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              timeInMs_p    = time for timer in ms
//              argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_modifyTimer(tEplTimerHdl* pTimerHdl_p, ULONG timeInMs_p, tEplTimerArg argument_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplTimeruData*     pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        Ret = timeru_setTimer(pTimerHdl_p, timeInMs_p, argument_p);
        goto Exit;
    }
    pData = (tEplTimeruData*) *pTimerHdl_p;
    if ((tEplTimeruData*)pData->m_Timer.data != pData)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    mod_timer(&pData->m_Timer, (jiffies + 1 + ((timeInMs_p * HZ) + 999) / 1000));

    // copy the TimerArg after the timer is restarted,
    // so that a timer occurred immediately before mod_timer
    // won't use the new TimerArg and
    // therefore the old timer cannot be distinguished from the new one.
    // But if the new timer is too fast, it may get lost.
    EPL_MEMCPY(&pData->TimerArgument, &argument_p, sizeof(tEplTimerArg));

    // check if timer is really running
    if (timer_pending(&pData->m_Timer) == 0)
    {   // timer is not running
        // retry starting it
        add_timer(&pData->m_Timer);
    }

    // set handle to pointer of tEplTimeruData
//    *pTimerHdl_p = (tEplTimerHdl) pData;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    timeru_deleteTimer
//
// Description: function deletes a timer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel timeru_deleteTimer(tEplTimerHdl* pTimerHdl_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplTimeruData*     pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        Ret = kEplSuccessful;
        goto Exit;
    }
    pData = (tEplTimeruData*) *pTimerHdl_p;
    if ((tEplTimeruData*)pData->m_Timer.data != pData)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

/*    if (del_timer(&pData->m_Timer) == 1)
    {
        kfree(pData);
    }
*/
    // try to delete the timer
    del_timer(&pData->m_Timer);
    // free memory in any case
    kfree(pData);

    // uninitialize handle
    *pTimerHdl_p = 0;

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    timeru_isActive
//
// Description: checks if the timer referenced by the handle is currently
//              active.
//
// Parameters:  TimerHdl_p  = handle of the timer to check
//
// Returns:     BOOL        = TRUE, if active;
//                            FALSE, otherwise
//
// State:
//
//---------------------------------------------------------------------------

BOOL timeru_isActive(tEplTimerHdl timerHdl_p)
{
BOOL        fActive = FALSE;
tEplTimeruData*     pData;

    // check handle itself, i.e. was the handle initialized before
    if (timerHdl_p == 0)
    {   // timer was not created yet, so it is not active
        goto Exit;
    }
    pData = (tEplTimeruData*) timerHdl_p;
    if ((tEplTimeruData*)pData->m_Timer.data != pData)
    {   // invalid timer
        goto Exit;
    }

    // check if timer is running
    if (timer_pending(&pData->m_Timer) == 0)
    {   // timer is not running
        goto Exit;
    }

    fActive = TRUE;

Exit:
    return fActive;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplTimeruCbMs
//
// Description: function to process timer
//
//
//
// Parameters:  lpParameter = pointer to structur of type tEplTimeruData
//
//
// Returns:     (none)
//
//
// State:
//
//---------------------------------------------------------------------------
static void PUBLIC EplTimeruCbMs(unsigned long ulParameter_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplTimeruData*     pData;
tEplEvent           EplEvent;
tEplTimerEventArg   TimerEventArg;

    pData = (tEplTimeruData*) ulParameter_p;

    // call event function
    TimerEventArg.m_TimerHdl = (tEplTimerHdl)pData;
    EPL_MEMCPY(&TimerEventArg.m_Arg, &pData->TimerArgument.m_Arg, sizeof (TimerEventArg.m_Arg));

    EplEvent.m_EventSink = pData->TimerArgument.m_EventSink;
    EplEvent.m_EventType = kEplEventTypeTimer;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(tEplNetTime));
    EplEvent.m_pArg = &TimerEventArg;
    EplEvent.m_uiSize = sizeof(TimerEventArg);

    Ret = eventu_postEvent(&EplEvent);

    // d.k. do not free memory, user has to call timeru_deleteTimer()
    //kfree(pData);

}


// EOF

