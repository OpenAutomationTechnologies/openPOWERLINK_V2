/**
********************************************************************************
\file   synctimer.h

\brief  Definitions for synchronization timer module

This file contains the definitions for the synchronization timer module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


#ifndef _INC_synctimer_H_
#define _INC_synctimer_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <EplInc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef tEplKernel (*tSyncTimerCbSync) (void);
typedef tEplKernel (*tSyncTimerCbLossOfSync) (void);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tEplKernel synctimer_addInstance(void);
tEplKernel synctimer_delInstance(void);
tEplKernel synctimer_registerHandler(tSyncTimerCbSync pfnTimerSynckCbSync_p);
tEplKernel synctimer_registerLossOfSyncHandler(tSyncTimerCbLossOfSync pfnTimerSynckCbLossOfSync_p);
tEplKernel synctimer_registerLossOfSyncHandler2(tSyncTimerCbLossOfSync pfnTimerSynckCbLossOfSync2_p);
tEplKernel synctimer_setSyncShift(UINT32 advanceShift_p);
tEplKernel synctimer_setCycleLen(UINT32 cycleLen_p);
tEplKernel synctimer_setLossOfSyncTolerance(UINT32 lossOfSyncTolerance_p);
tEplKernel synctimer_setLossOfSyncTolerance2(UINT32 lossOfSyncTolerance2_p);
tEplKernel synctimer_syncTriggerAtTimeStamp(tEplTgtTimeStamp* pTimeStamp_p);
tEplKernel synctimer_stopSync(void);
void       synctimer_enableExtSyncIrq(UINT32 syncIntCycle_p, UINT32 pulseWidth_p);
void       synctimer_disableExtSyncIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_synctimer_H_ */
