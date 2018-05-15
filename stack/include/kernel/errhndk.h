/**
********************************************************************************
\file   kernel/errhndk.h

\brief  External interface of the error handler kernel module

This header provides the external interface of the error handler kernel module

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
#ifndef _INC_kernel_errhndk_H_
#define _INC_kernel_errhndk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/errhnd.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DLL_ERR_MN_CRC              0x00000001L     ///< object 0x1C00
#define DLL_ERR_MN_COLLISION        0x00000002L     ///< object 0x1C01
#define DLL_ERR_MN_CYCTIMEEXCEED    0x00000004L     ///< object 0x1C02
#define DLL_ERR_MN_LOSS_LINK        0x00000008L     ///< object 0x1C03
#define DLL_ERR_MN_CN_LATE_PRES     0x00000010L     ///< objects 0x1C04-0x1C06
#define DLL_ERR_MN_CN_LOSS_PRES     0x00000080L     ///< objects 0x1C07-0x1C09
#define DLL_ERR_CN_COLLISION        0x00000400L     ///< object 0x1C0A
#define DLL_ERR_CN_LOSS_SOC         0x00000800L     ///< object 0x1C0B
#define DLL_ERR_CN_LOSS_SOA         0x00001000L     ///< object 0x1C0C
#define DLL_ERR_CN_LOSS_PREQ        0x00002000L     ///< object 0x1C0D
#define DLL_ERR_CN_RECVD_PREQ       0x00004000L     ///< decrement object 0x1C0D/2
#define DLL_ERR_CN_SOC_JITTER       0x00008000L     ///< object 0x1C0E
#define DLL_ERR_CN_CRC              0x00010000L     ///< object 0x1C0F
#define DLL_ERR_CN_LOSS_LINK        0x00020000L     ///< object 0x1C10
#define DLL_ERR_MN_LOSS_STATRES     0x00040000L     ///< objects 0x1C15-0x1C17 (should be operated by NmtMnu module)
#define DLL_ERR_BAD_PHYS_MODE       0x00080000L     ///< no object
#define DLL_ERR_MAC_BUFFER          0x00100000L     ///< no object (NMT_GT6)
#define DLL_ERR_INVALID_FORMAT      0x00200000L     ///< no object (NMT_GT6)
#define DLL_ERR_ADDRESS_CONFLICT    0x00400000L     ///< no object (remove CN from configuration)

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

tOplkError      errhndk_init(void);
tOplkError      errhndk_exit(void);
tErrHndObjects* errhndk_getMemPtr(void);
tOplkError      errhndk_process(const tEvent* pEvent_p);
tOplkError      errhndk_postError(const tEventDllError* pDllEvent_p);
tOplkError      errhndk_decrementCounters(BOOL fMN_p) SECTION_ERRHNDK_DECRCNTERS;

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError      errhndk_resetCnError(UINT nodeId_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_errhndk_H_ */
