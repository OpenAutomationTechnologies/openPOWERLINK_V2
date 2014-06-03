/**
********************************************************************************
\file   oplk/section-microblaze.h

\brief  Macros for special function linking for Xilinx Microblaze

This header file defines macros for Xilinx Microblaze targets to link specific
functions to local memory.

Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2014, Kalycito Infotech Private Ltd.
Copyright (c) 2012, SYSTEC electronik GmbH

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

#ifndef _INC_oplk_section_microblaze_H_
#define _INC_oplk_section_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifdef NDEBUG
#define XIL_INTERNAL_RAM    __attribute__((section(".local_memory")))
#else
#define XIL_INTERNAL_RAM
#endif

#ifdef CONFIG_MN

    #define SECTION_CIRCBUF_WRITE_DATA          XIL_INTERNAL_RAM
    #define SECTION_CIRCBUF_WRITE_MULT_DATA     XIL_INTERNAL_RAM
    #define SECTION_CIRCBUF_READ_DATA           XIL_INTERNAL_RAM
    #define SECTION_CIRCBUF_LOCK                XIL_INTERNAL_RAM
    #define SECTION_CIRCBUF_UNLOCK              XIL_INTERNAL_RAM
    #define SECTION_EDRVOPENMAC_RX_HOOK         XIL_INTERNAL_RAM
    #define SECTION_EDRVOPENMAC_IRQ_HDL         XIL_INTERNAL_RAM
    #define SECTION_DLLK_FRAME_RCVD_CB          XIL_INTERNAL_RAM
    #define SECTION_EVENTK_PROCESS              XIL_INTERNAL_RAM
    #define SECTION_EVENTK_POST                 XIL_INTERNAL_RAM
    #define SECTION_EVENTKCAL_POST              XIL_INTERNAL_RAM
    #define SECTION_EVENTKCAL_CIRCBUF_POST      XIL_INTERNAL_RAM
    #define SECTION_EVENTKCAL_CIRCBUF_PROCESS   XIL_INTERNAL_RAM
    #define SECTION_PDOK_PROCESS_TPDO_CB        XIL_INTERNAL_RAM
    #define SECTION_PDOKCAL_READ_TPDO           XIL_INTERNAL_RAM
    #define SECTION_PDOK_PROCESS_RPDO           XIL_INTERNAL_RAM
    #define SECTION_PDOKCAL_WRITE_RPDO          XIL_INTERNAL_RAM
    #define SECTION_PDOKCAL_PROCESS             XIL_INTERNAL_RAM
    #define SECTION_EDRVCYC_TIMER_CB            XIL_INTERNAL_RAM
    #define SECTION_HRTIMER_IRQ_HDL             XIL_INTERNAL_RAM
    #define SECTION_HRTIMER_MODTIMER            XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS                XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_CYCFIN         XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_SYNC           XIL_INTERNAL_RAM
    #define SECTION_DLLK_CHANGE_STATE           XIL_INTERNAL_RAM
    #define SECTION_DLLKCAL_ASYNCRX             XIL_INTERNAL_RAM
    #define SECTION_DLLKCAL_GETSOAREQ           XIL_INTERNAL_RAM
    #define SECTION_DLLKCAL_GETPENREQ           XIL_INTERNAL_RAM
    #define SECTION_DLLK_FRAME_UPDATE_SOA       XIL_INTERNAL_RAM
    #define SECTION_DLLK_FRAME_ASYNC_NRX        XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_TX_SOA         XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_TX_SOC         XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_TX_NMT         XIL_INTERNAL_RAM
    #define SECTION_DLLK_PROCESS_TX_NPLK        XIL_INTERNAL_RAM
    #define SECTION_EDRVCYC_SET_NEXT_TX         XIL_INTERNAL_RAM
    #define SECTION_DLLK_MN_SYNC_CB             XIL_INTERNAL_RAM
    #define SECTION_TARGET_GLOBAL_INT           XIL_INTERNAL_RAM
    #define SECTION_ERRHNDK_DECRCNTERS          XIL_INTERNAL_RAM


#endif //CONFIG_MN

#ifdef CONFIG_CN
    /* TODO:
     * Find optimal setting again due to revised stack design!
     */
    #define SECTION_PDOK_PROCESS_TPDO_CB    XIL_INTERNAL_RAM
    #define SECTION_PDOK_COPY_TPDO          XIL_INTERNAL_RAM
    #define SECTION_PDOK_PROCESS_RPDO       XIL_INTERNAL_RAM
    #define SECTION_EVENTK_PROCESS          XIL_INTERNAL_RAM
    #define SECTION_EVENTK_POST             XIL_INTERNAL_RAM
    #define SECTION_OMETHLIB_RX_IRQ_HDL     XIL_INTERNAL_RAM
    #define SECTION_EDRVOPENMAC_RX_HOOK     XIL_INTERNAL_RAM
    #define SECTION_EDRVOPENMAC_IRQ_HDL     XIL_INTERNAL_RAM
    #define SECTION_MAIN_APP_CB_SYNC        XIL_INTERNAL_RAM
    #define SECTION_DLLK_FRAME_RCVD_CB      XIL_INTERNAL_RAM

#endif //CONFIG_CN

// Common Sections in ometh and dualprocshm library
    #define SECTION_DUALPROCSHM_ACQUIRE_LOCK    XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_RELEASE_LOCK    XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_AQ_BUFF_LOCK    XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_RE_BUFF_LOCK    XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_IRQ_ENABLE      XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_IRQ_SET         XIL_INTERNAL_RAM
    #define SECTION_DUALPROCSHM_IRQ_HDL         XIL_INTERNAL_RAM

    #define SECTION_OMETHLIB_RX_IRQ_HDL         XIL_INTERNAL_RAM
    #define SECTION_OMETHLIB_TX_IRQ_HDL         XIL_INTERNAL_RAM
    #define SECTION_OMETHLIB_RXTX_IRQ_MUX       XIL_INTERNAL_RAM
    #define SECTION_OMETHLIB_TX_TIME            XIL_INTERNAL_RAM

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_oplk_section_microblaze_H_ */

