/**
********************************************************************************
\file   kernel/hrestimer.h

\brief  Definitions for high-resolution timer module

This file contains the definitions for the high-resolution timer module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
#ifndef _INC_kernel_hrestimer_H_
#define _INC_kernel_hrestimer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/timer.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Type for timer callback function pointers

This type defines a function pointer to a timer callback function.

\param[in]      pEventArg_p         Pointer to timer event argument

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tTimerkCallback)(const tTimerEventArg* pEventArg_p);

/// Callback function pointer for hres timer callback function
typedef void (*tHresCallback)(tTimerHdl* pTimerHdl_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError hrestimer_init(void);
tOplkError hrestimer_exit(void);
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p,
                                 ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p,
                                 ULONG argument_p,
                                 BOOL fContinue_p) SECTION_HRTIMER_MODTIMER;
tOplkError hrestimer_setAbsoluteTimer(tTimerHdl* pTimerHdl_p,
                                      tTimestamp time_p,
                                      tTimerkCallback pfnCallback_p,
                                      ULONG argument_p) SECTION_HRTIMER_SETTIMER;
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p);
void       hrestimer_controlExtSyncIrq(BOOL fEnable_p);
void       hrestimer_setExtSyncIrqTime(tTimestamp time_p);

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_hrestimer_H_ */
