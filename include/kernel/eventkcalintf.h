/**
********************************************************************************
\file   eventkcalintf.h

\brief  Include file for kernel event CAL module interfaces

This file contains definitions for the kernel event CAL module. The kernel event
CAL builds the interface between the event and the event queue implementations.

*******************************************************************************/

/*------------------------------------------------------------------------------
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

#ifndef _INC_eventkcalintf_H_
#define _INC_eventkcalintf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <event.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/* host interface buffer event interface */
tEplKernel eventkcal_initQueueHostif(tEventQueue eventQueue_p);
tEplKernel eventkcal_exitQueueHostif (tEventQueue eventQueue_p);
tEplKernel eventkcal_postEventHostif (tEventQueue eventQueue_p, tEplEvent *pEvent_p);
tEplKernel eventkcal_processEventHostif(tEventQueue eventQueue_p) SECTION_EVENTKCAL_HOSTIF_PROCESS;
tEplKernel eventkcal_getEventHostif(tEventQueue eventQueue_p, BYTE* pDataBuffer_p, size_t* pReadSize_p);
UINT       eventkcal_getEventCountHostif(tEventQueue eventQueue_p);
tEplKernel eventkcal_setSignalingHostif(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p);

/* circular buffer event interface */
tEplKernel eventkcal_initQueueCircbuf(tEventQueue eventQueue_p);
tEplKernel eventkcal_exitQueueCircbuf (tEventQueue eventQueue_p);
tEplKernel eventkcal_postEventCircbuf (tEventQueue eventQueue_p, tEplEvent *pEvent_p);
tEplKernel eventkcal_processEventCircbuf(tEventQueue eventQueue_p);
tEplKernel eventkcal_getEventCircbuf(tEventQueue eventQueue_p, BYTE* pDataBuffer_p, size_t* pReadSize_p);
UINT       eventkcal_getEventCountCircbuf(tEventQueue eventQueue_p);
tEplKernel eventkcal_setSignalingCircbuf(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_eventkcalintf_H_ */
