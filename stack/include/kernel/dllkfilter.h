/**
********************************************************************************
\file   dllkfilter.h

\brief  Definitions of the filters for the dllk module

This file defines which filter is for which frame in the dllk module. This filters
can only be used for MACs which have a frame filter.

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

#ifndef _INC_dllkfilter_H_
#define _INC_dllkfilter_H_

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DLLK_FILTER_PREQ                0
#define DLLK_FILTER_SOA_IDREQ           1
#define DLLK_FILTER_SOA_STATREQ         2
#define DLLK_FILTER_SOA_NMTREQ          3
#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
  #define DLLK_FILTER_SOA_SYNCREQ       4
  #define DLLK_FILTER_SOA_NONPLK        5
#else
  #define DLLK_FILTER_SOA_NONPLK        4
#endif

#define DLLK_FILTER_SOA                 (DLLK_FILTER_SOA_NONPLK + 1)
#define DLLK_FILTER_SOC                 (DLLK_FILTER_SOA + 1)
#define DLLK_FILTER_ASND                (DLLK_FILTER_SOC + 1)
#define DLLK_FILTER_PRES                (DLLK_FILTER_ASND + 1)

#if CONFIG_DLL_PRES_FILTER_COUNT < 0
  #define DLLK_FILTER_COUNT             (DLLK_FILTER_PRES + 1)
#else
  #define DLLK_FILTER_COUNT             (DLLK_FILTER_PRES + CONFIG_DLL_PRES_FILTER_COUNT)
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

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_dllkfilter_H_

