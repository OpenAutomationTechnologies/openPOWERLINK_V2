/**
********************************************************************************
\file   common/section.h

\brief  Header file for specific function linking

This header file defines macros for stack functions linked to specific memory.

Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
*******************************************************************************/
#ifndef _INC_common_section_H_
#define _INC_common_section_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/targetsystem.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Get target-dependent section defines
#if (DEV_SYSTEM == _DEV_NIOS2_ || \
     DEV_SYSTEM == _DEV_MICROBLAZE_BIG_ || \
     DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_)
#include <targetsection.h>
#endif

// Create empty defines for undefined macros (no special function placement)
#ifndef SECTION_AMI_GETUINT16LE
#define SECTION_AMI_GETUINT16LE
#endif

#ifndef SECTION_AMI_GETUINT16BE
#define SECTION_AMI_GETUINT16BE
#endif

#ifndef SECTION_CIRCBUF_WRITE_DATA
#define SECTION_CIRCBUF_WRITE_DATA
#endif

#ifndef SECTION_CIRCBUF_WRITE_MULT_DATA
#define SECTION_CIRCBUF_WRITE_MULT_DATA
#endif

#ifndef SECTION_CIRCBUF_READ_DATA
#define SECTION_CIRCBUF_READ_DATA
#endif

#ifndef SECTION_CIRCBUF_LOCK
#define SECTION_CIRCBUF_LOCK
#endif

#ifndef SECTION_CIRCBUF_UNLOCK
#define SECTION_CIRCBUF_UNLOCK
#endif

#ifndef SECTION_DLLK_FRAME_RCVD_CB
#define SECTION_DLLK_FRAME_RCVD_CB
#endif

#ifndef SECTION_PDOK_PROCESS_TPDO_CB
#define SECTION_PDOK_PROCESS_TPDO_CB
#endif

#ifndef SECTION_PDOKCAL_READ_TPDO
#define SECTION_PDOKCAL_READ_TPDO
#endif

#ifndef SECTION_PDOK_PROCESS_RPDO
#define SECTION_PDOK_PROCESS_RPDO
#endif

#ifndef SECTION_PDOKCAL_WRITE_RPDO
#define SECTION_PDOKCAL_WRITE_RPDO
#endif

#ifndef SECTION_PDOKCAL_PROCESS
#define SECTION_PDOKCAL_PROCESS
#endif

#ifndef SECTION_PDOKLUT_GETCHANNEL
#define SECTION_PDOKLUT_GETCHANNEL
#endif

#ifndef SECTION_EVENT_GET_HDL_FOR_SINK
#define SECTION_EVENT_GET_HDL_FOR_SINK
#endif

#ifndef SECTION_EVENTK_PROCESS
#define SECTION_EVENTK_PROCESS
#endif

#ifndef SECTION_EVENTK_POST
#define SECTION_EVENTK_POST
#endif

#ifndef SECTION_EVENTKCAL_POST
#define SECTION_EVENTKCAL_POST
#endif

#ifndef SECTION_EVENTKCAL_CIRCBUF_POST
#define SECTION_EVENTKCAL_CIRCBUF_POST
#endif

#ifndef SECTION_EDRVOPENMAC_RX_HOOK
#define SECTION_EDRVOPENMAC_RX_HOOK
#endif

#ifndef SECTION_EDRVOPENMAC_IRQ_HDL
#define SECTION_EDRVOPENMAC_IRQ_HDL
#endif

#ifndef SECTION_EDRVCYC_TIMER_CB
#define SECTION_EDRVCYC_TIMER_CB
#endif

#ifndef SECTION_HRTIMER_IRQ_HDL
#define SECTION_HRTIMER_IRQ_HDL
#endif

#ifndef SECTION_HRTIMER_MODTIMER
#define SECTION_HRTIMER_MODTIMER
#endif

#ifndef SECTION_HRTIMER_SETTIMER
#define SECTION_HRTIMER_SETTIMER
#endif

#ifndef SECTION_SYNCTIMER_FINDTIMER
#define SECTION_SYNCTIMER_FINDTIMER
#endif

#ifndef SECTION_SYNCTIMER_CONFTIMER
#define SECTION_SYNCTIMER_CONFTIMER
#endif

#ifndef SECTION_DLLK_PROCESS
#define SECTION_DLLK_PROCESS
#endif

#ifndef SECTION_DLLK_PROCESS_CYCFIN
#define SECTION_DLLK_PROCESS_CYCFIN
#endif

#ifndef SECTION_DLLK_PROCESS_SYNC
#define SECTION_DLLK_PROCESS_SYNC
#endif

#ifndef SECTION_DLLK_CHANGE_STATE
#define SECTION_DLLK_CHANGE_STATE
#endif

#ifndef SECTION_DLLK_GETNODEINFO
#define SECTION_DLLK_GETNODEINFO
#endif

#ifndef SECTION_DLLKCAL_ASYNCRX
#define SECTION_DLLKCAL_ASYNCRX
#endif

#ifndef SECTION_DLLKCAL_GETSOAREQ
#define SECTION_DLLKCAL_GETSOAREQ
#endif

#ifndef SECTION_DLLKCAL_GETPENREQ
#define SECTION_DLLKCAL_GETPENREQ
#endif

#ifndef SECTION_ERRHNDK_DECRCNTERS
#define SECTION_ERRHNDK_DECRCNTERS
#endif

#ifndef SECTION_ERRHNDKCAL_GETMNCNT
#define SECTION_ERRHNDKCAL_GETMNCNT
#endif

#ifndef SECTION_ERRHNDKCAL_SETMNCNT
#define SECTION_ERRHNDKCAL_SETMNCNT
#endif

#ifndef SECTION_DLLK_FRAME_UPDATE_SOA
#define SECTION_DLLK_FRAME_UPDATE_SOA
#endif

#ifndef SECTION_DLLK_FRAME_ASYNC_NRX
#define SECTION_DLLK_FRAME_ASYNC_NRX
#endif

#ifndef SECTION_DLLK_PROCESS_TX_SOA
#define SECTION_DLLK_PROCESS_TX_SOA
#endif

#ifndef SECTION_DLLK_PROCESS_TX_SOC
#define SECTION_DLLK_PROCESS_TX_SOC
#endif

#ifndef SECTION_DLLK_PROCESS_TX_NMT
#define SECTION_DLLK_PROCESS_TX_NMT
#endif

#ifndef SECTION_DLLK_PROCESS_TX_NPLK
#define SECTION_DLLK_PROCESS_TX_NPLK
#endif

#ifndef SECTION_EDRVCYC_SET_NEXT_TX
#define SECTION_EDRVCYC_SET_NEXT_TX
#endif

#ifndef SECTION_DLLK_MN_SYNC_CB
#define SECTION_DLLK_MN_SYNC_CB
#endif

#ifndef SECTION_TARGET_GLOBAL_INT
#define SECTION_TARGET_GLOBAL_INT
#endif

#ifndef SECTION_TARGET_SET_INTCONT
#define SECTION_TARGET_SET_INTCONT
#endif

#ifndef SECTION_TARGET_GET_INTCONT
#define SECTION_TARGET_GET_INTCONT
#endif

#ifndef SECTION_DUALPROCSHM_ACQUIRE_LOCK
#define SECTION_DUALPROCSHM_ACQUIRE_LOCK
#endif

#ifndef SECTION_DUALPROCSHM_RELEASE_LOCK
#define SECTION_DUALPROCSHM_RELEASE_LOCK
#endif

#ifndef SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#define SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#endif

#ifndef SECTION_DUALPROCSHM_RE_BUFF_LOCK
#define SECTION_DUALPROCSHM_RE_BUFF_LOCK
#endif

#ifndef SECTION_DUALPROCSHM_IRQ_ENABLE
#define SECTION_DUALPROCSHM_IRQ_ENABLE
#endif

#ifndef SECTION_DUALPROCSHM_IRQ_SET
#define SECTION_DUALPROCSHM_IRQ_SET
#endif

#ifndef SECTION_DUALPROCSHM_IRQ_HDL
#define SECTION_DUALPROCSHM_IRQ_HDL
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_common_section_H_ */
