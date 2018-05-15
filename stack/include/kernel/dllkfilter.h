/**
********************************************************************************
\file   kernel/dllkfilter.h

\brief  Definitions of the filters for the dllk module

This file defines which filter is for which frame in the dllk module. This filters
can only be used for MACs which have a frame filter.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_kernel_dllkfilter_H_
#define _INC_kernel_dllkfilter_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/edrv.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DLLK_FILTER_PREQ                0
#define DLLK_FILTER_SOA_IDREQ           1
#define DLLK_FILTER_SOA_STATREQ         2
#define DLLK_FILTER_SOA_NMTREQ          3

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
#define DLLK_FILTER_SOA_SYNCREQ         4
#define DLLK_FILTER_SOA_NONPLK          5
#else /* (CONFIG_DLL_PRES_CHAINING_CN != FALSE) */
#define DLLK_FILTER_SOA_NONPLK          4
#endif /* (CONFIG_DLL_PRES_CHAINING_CN != FALSE) */

#define DLLK_FILTER_SOA                 (DLLK_FILTER_SOA_NONPLK + 1)
#define DLLK_FILTER_SOC                 (DLLK_FILTER_SOA + 1)
#define DLLK_FILTER_ASND                (DLLK_FILTER_SOC + 1)

#if defined(CONFIG_INCLUDE_VETH)
#define DLLK_FILTER_VETH_UNICAST        (DLLK_FILTER_ASND + 1)
#define DLLK_FILTER_VETH_BROADCAST      (DLLK_FILTER_VETH_UNICAST + 1)
#define DLLK_FILTER_PRES                (DLLK_FILTER_VETH_BROADCAST + 1)
#else /* defined(CONFIG_INCLUDE_VETH) */
#define DLLK_FILTER_PRES                (DLLK_FILTER_ASND + 1)
#endif /* defined(CONFIG_INCLUDE_VETH) */

#if (CONFIG_DLL_PRES_FILTER_COUNT < 0)
#define DLLK_FILTER_COUNT               (DLLK_FILTER_PRES + 1)
#else /* (CONFIG_DLL_PRES_FILTER_COUNT < 0) */
#define DLLK_FILTER_COUNT               (DLLK_FILTER_PRES + CONFIG_DLL_PRES_FILTER_COUNT)
#endif /* (CONFIG_DLL_PRES_FILTER_COUNT < 0) */

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

void dllkfilter_setupFilters(void);
void dllkfilter_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p);
void dllkfilter_setupPreqFilter(tEdrvFilter* pFilter_p, UINT8 nodeId_p,
                                tEdrvTxBuffer* pBuffer_p,
                                const UINT8* pMacAdrs_p);

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_dllkfilter_H_ */
