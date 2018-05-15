/**
********************************************************************************
\file   kernel/synctimer.h

\brief  Definitions for synchronization timer module

This file contains the definitions for the synchronization timer module.

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
#ifndef _INC_kernel_synctimer_H_
#define _INC_kernel_synctimer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef tOplkError (*tSyncTimerCbSync)(void);
typedef tOplkError (*tSyncTimerCbLossOfSync)(void);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError synctimer_init(void);
tOplkError synctimer_exit(void);
tOplkError synctimer_registerHandler(tSyncTimerCbSync pfnTimerSynckCbSync_p);
tOplkError synctimer_registerLossOfSyncHandler(tSyncTimerCbLossOfSync pfnTimerSynckCbLossOfSync_p);
tOplkError synctimer_setSyncShift(UINT32 advanceShift_p);
tOplkError synctimer_setCycleLen(UINT32 cycleLen_p, UINT32 minSyncTime_p);
tOplkError synctimer_setLossOfSyncTolerance(UINT32 lossOfSyncTolerance_p);
tOplkError synctimer_syncTriggerAtTimeStamp(const tTimestamp* pTimeStamp_p);
tOplkError synctimer_stopSync(void);
void       synctimer_controlExtSyncIrq(BOOL fEnable_p);

#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
tOplkError synctimer_registerLossOfSyncHandler2(tSyncTimerCbLossOfSync pfnTimerSynckCbLossOfSync2_p);
tOplkError synctimer_setLossOfSyncTolerance2(UINT32 lossOfSyncTolerance2_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_synctimer_H_ */
