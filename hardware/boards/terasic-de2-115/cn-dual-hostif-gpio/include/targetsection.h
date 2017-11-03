/**
********************************************************************************
\file   targetsection.h

\brief  Special function linking for TERASIC DE2-115 Eval Board CN dual GPIO

This header file defines macros for Altera Nios II targets to link specific
functions to tightly-coupled memory.

Copyright (c) 2015, B&R Industrial Automation GmbH
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

#ifndef _INC_targetsection_H_
#define _INC_targetsection_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef ALT_TCMEM_SIZE
#define ALT_TCMEM_SIZE                      0
#endif

#ifdef NDEBUG
#define ALT_INTERNAL_RAM    __attribute__((section(".tc_mem")))
#else
#define ALT_INTERNAL_RAM
#endif

#if (ALT_TCMEM_SIZE >= 2048)
#define SECTION_EVENTK_POST                 ALT_INTERNAL_RAM
#define SECTION_EVENTK_PROCESS              ALT_INTERNAL_RAM
#define SECTION_EDRVOPENMAC_IRQ_HDL         ALT_INTERNAL_RAM
#define SECTION_SYNCTIMER_FINDTIMER         ALT_INTERNAL_RAM
#define SECTION_SYNCTIMER_CONFTIMER         ALT_INTERNAL_RAM
#define SECTION_OMETHLIB_RX_IRQ_HDL         ALT_INTERNAL_RAM
#define SECTION_OMETHLIB_RXTX_IRQ_MUX       ALT_INTERNAL_RAM
#endif

#if (ALT_TCMEM_SIZE >= 4096)
#define SECTION_EDRVOPENMAC_RX_HOOK         ALT_INTERNAL_RAM
#define SECTION_DLLK_CHANGE_STATE           ALT_INTERNAL_RAM
#endif

#if (ALT_TCMEM_SIZE >= 8192)
#define SECTION_CIRCBUF_WRITE_DATA          ALT_INTERNAL_RAM
#define SECTION_CIRCBUF_WRITE_MULT_DATA     ALT_INTERNAL_RAM
#define SECTION_CIRCBUF_READ_DATA           ALT_INTERNAL_RAM
#define SECTION_CIRCBUF_LOCK                ALT_INTERNAL_RAM
#define SECTION_CIRCBUF_UNLOCK              ALT_INTERNAL_RAM
#define SECTION_OMETHLIB_TX_IRQ_HDL         ALT_INTERNAL_RAM
#define SECTION_DLLK_FRAME_RCVD_CB          ALT_INTERNAL_RAM
#define SECTION_EVENTKCAL_POST              ALT_INTERNAL_RAM
#define SECTION_EVENTKCAL_CIRCBUF_POST      ALT_INTERNAL_RAM
#define SECTION_PDOK_PROCESS_TPDO_CB        ALT_INTERNAL_RAM
#define SECTION_PDOKCAL_READ_TPDO           ALT_INTERNAL_RAM
#define SECTION_PDOK_PROCESS_RPDO           ALT_INTERNAL_RAM
#define SECTION_PDOKCAL_WRITE_RPDO          ALT_INTERNAL_RAM
#define SECTION_PDOKCAL_PROCESS             ALT_INTERNAL_RAM
#define SECTION_PDOKLUT_GETCHANNEL          ALT_INTERNAL_RAM
#define SECTION_DLLK_PROCESS                ALT_INTERNAL_RAM
#define SECTION_DLLK_PROCESS_CYCFIN         ALT_INTERNAL_RAM
#define SECTION_DLLK_PROCESS_SYNC           ALT_INTERNAL_RAM
#define SECTION_DLLKCAL_ASYNCRX             ALT_INTERNAL_RAM
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_targetsection_H_ */
