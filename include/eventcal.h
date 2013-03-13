/**
********************************************************************************
\file   eventcal.h

\brief  Definitions for event CAL module

The file contains definitions for the event CAL module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_eventcal_H_
#define _INC_eventcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Event CAL function interface structure

This struct defines the function interface of a event CAL module. The functions
are defines as static functions and a variable of this type provides the
function pointers to other modules.
*/
typedef struct
{
    tEplKernel (* pfnAddInstance)(tEventQueueInstPtr *ppEventQueueInst_p,
                                  tEventQueue eventQueue_p);
    tEplKernel (* pfnDelInstance)(tEventQueueInstPtr pEventQueue_p);
    tEplKernel (* pfnPostEvent)(tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p);
} tEventCalFuncIntf;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// event CAL function prototypes of the shared buffer implementation
tEplKernel   eventcalshb_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p, tEplProcessEventCb pfnProcessEventCb,
                              tEplPostErrorEventCb pfnPostErrorEventCb);
tEplKernel   eventcalshb_delInstance (tEventQueueInstPtr pEventQueueInst_p);
tEplKernel   eventcalshb_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p);

// event CAL function prototypes of the host interface implementation
tEplKernel   eventcalhostif_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p, tEplProcessEventCb pfnProcessEventCb,
                              tEplPostErrorEventCb pfnPostErrorEventCb);
tEplKernel   eventcalhostif_delInstance (tEventQueueInstPtr pEventQueueInst_p);
tEplKernel   eventcalhostif_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p) SECTION_EVENTCAL_HOSTIF_POST;

// event CAL functions prototypes of the direct call implementation
tEplKernel   eventcaldirect_addInstance(tEventQueueInstPtr *ppEventQueueInst_p,
                              tEventQueue eventQueue_p, tEplProcessEventCb pfnProcessEventCb,
                              BOOL fProcessThreadSafe_p);
tEplKernel   eventcaldirect_delInstance (tEventQueueInstPtr pEventQueueInst_p);
tEplKernel   eventcaldirect_postEvent (tEventQueueInstPtr pEventQueue_p, tEplEvent *pEvent_p) SECTION_EVENTCAL_DIRECT_POST;

#ifdef __cplusplus
}
#endif

#endif /* _INC_eventcal_H_ */


