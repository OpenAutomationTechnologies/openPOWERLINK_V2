/**
********************************************************************************
\file   kernel/eventkcal.h

\brief  Include file for kernel event CAL module

This file contains definitions for the kernel event CAL module. The kernel event
CAL builds the interface between the user event and the event queue
implementations.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#ifndef _INC_kernel_eventkcal_H_
#define _INC_kernel_eventkcal_H_

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

tOplkError eventkcal_init(void);
tOplkError eventkcal_exit(void);
tOplkError eventkcal_postUserEvent(const tEvent* pEvent_p) SECTION_EVENTKCAL_POST;
tOplkError eventkcal_postKernelEvent(const tEvent* pEvent_p) SECTION_EVENTKCAL_POST;
void       eventkcal_process(void);

#if ((TARGET_SYSTEM == _LINUX_) && defined(__KERNEL__))
/* functions used in eventkcal-linuxkernel.c */
int        eventkcal_postEventFromUser(ULONG arg);
int        eventkcal_getEventForUser(ULONG arg);
#elif ((TARGET_SYSTEM == _WIN32_) && defined(_KERNEL_MODE))
// TODO: Check if they can be revised to merge with Linux APIs
void       eventkcal_postEventFromUser(const void* pEvent_p);
void       eventkcal_getEventForUser(void* pEvent_p, size_t* pSize_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_eventkcal_H_ */
