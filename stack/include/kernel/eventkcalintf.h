/**
********************************************************************************
\file   kernel/eventkcalintf.h

\brief  Include file for kernel event CAL module interfaces

This file contains definitions for the kernel event CAL module. The kernel event
CAL builds the interface between the event and the event queue implementations.

*******************************************************************************/

/*------------------------------------------------------------------------------
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
#ifndef _INC_kernel_eventkcalintf_H_
#define _INC_kernel_eventkcalintf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/event.h>

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
extern "C"
{
#endif

/* circular buffer event interface */
tOplkError eventkcal_initQueueCircbuf(tEventQueue eventQueue_p);
tOplkError eventkcal_exitQueueCircbuf(tEventQueue eventQueue_p);
tOplkError eventkcal_postEventCircbuf(tEventQueue eventQueue_p, const tEvent* pEvent_p) SECTION_EVENTKCAL_CIRCBUF_POST;
tOplkError eventkcal_processEventCircbuf(tEventQueue eventQueue_p);
tOplkError eventkcal_getEventCircbuf(tEventQueue eventQueue_p, UINT8* pDataBuffer_p, size_t* pReadSize_p);
UINT       eventkcal_getEventCountCircbuf(tEventQueue eventQueue_p);
tOplkError eventkcal_setSignalingCircbuf(tEventQueue eventQueue_p, VOIDFUNCPTR pfnSignalCb_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_eventkcalintf_H_ */
