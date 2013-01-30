/**
********************************************************************************
\file   eventkcal.h

\brief  Include file for kernel event CAL module

This file contains definitions for the kernel event CAL module. The kernel event
CAL builds the interface between the user event and the event queue
implementations.

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

#ifndef _INC_eventkcal_H_
#define _INC_eventkcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <event.h>
#include <eventcal.h>
#include <kernel/eventk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/*
The following macros define the kernel event CAL interface getter functions for
the different event queues depending on the used queue.
*/

/* setup interface getting function for user to kernel queue */
#if (EPL_EVENT_U2K_QUEUE == EPL_QUEUE_DIRECT)
#define GET_EVENTK_U2K_INTERFACE eventkcaldirect_getInterface
#elif (EPL_EVENT_U2K_QUEUE == EPL_QUEUE_SHB)
#define GET_EVENTK_U2K_INTERFACE eventkcalshb_getInterface
#elif (EPL_EVENT_U2K_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_EVENTK_U2K_INTERFACE eventkcalhostif_getInterface
#else
#error "Unsupported user-to-kernel queue"
#endif

/* setup interface getting function for user internal queue */
#if (EPL_EVENT_UINT_QUEUE == EPL_QUEUE_DIRECT)
#define GET_EVENTK_UINT_INTERFACE eventkcaldirect_getInterface
#elif (EPL_EVENT_UINT_QUEUE == EPL_QUEUE_SHB)
#define GET_EVENTK_UINT_INTERFACE eventkcalshb_getInterface
#elif (EPL_EVENT_UINT_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_EVENTK_UINT_INTERFACE eventkcalhostif_getInterface
#else
#error "Unsupported user internal queue"
#endif

/* setup interface getting function for kernel to user queue */
#if (EPL_EVENT_K2U_QUEUE == EPL_QUEUE_DIRECT)
#define GET_EVENTK_K2U_INTERFACE eventkcaldirect_getInterface
#elif (EPL_EVENT_K2U_QUEUE == EPL_QUEUE_SHB)
#define GET_EVENTK_K2U_INTERFACE eventkcalshb_getInterface
#elif (EPL_EVENT_K2U_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_EVENTK_K2U_INTERFACE eventkcalhostif_getInterface
#else
#error "Unsupported kernel-to-user queue"
#endif

/* setup interface getting function for kernel internal queue */
#if (EPL_EVENT_KINT_QUEUE == EPL_QUEUE_DIRECT)
#define GET_EVENTK_KINT_INTERFACE eventkcaldirect_getInterface
#elif (EPL_EVENT_KINT_QUEUE == EPL_QUEUE_SHB)
#define GET_EVENTK_KINT_INTERFACE eventkcalshb_getInterface
#elif (EPL_EVENT_KINT_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_EVENTK_KINT_INTERFACE eventkcalhostif_getInterface
#else
#error "Unsupported kernel internal queue"
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tEplKernel eventkcal_init (void);
tEplKernel eventkcal_exit (void);
tEplKernel eventkcal_postEvent (tEplEvent *pEvent_p);
tEplKernel eventkcal_rxHandler (tEplEvent *pEvent_p);

/* interface getter functions for the different implementations */
tEventCalFuncIntf* eventkcaldirect_getInterface(void);
tEventCalFuncIntf* eventkcalshb_getInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_eventkcal_H_ */
