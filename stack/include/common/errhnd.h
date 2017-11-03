/**
********************************************************************************
\file   common/errhnd.h

\brief  include file for error handler modules

This file contains all definitions and declarations for the error handler
modules.

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
#ifndef _INC_common_errhnd_H_
#define _INC_common_errhnd_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Error object structure

This structure specifies an error object consisting of a cumulative counter, a
threshold counter and a threshold.
*/
typedef struct
{
    UINT32              cumulativeCnt;              ///< Cumulative counter, stored in subindex 1
    UINT32              thresholdCnt;               ///< Threshold counter, stored in subindex 2
    UINT32              threshold;                  ///< Threshold, stored in subindex 3
} tErrorObject;

/**
\brief Error handler object structure

This structure defines all error handler objects. It is represented by
according objects in the object dictionary.
*/
typedef struct
{
    tErrorObject        cnLossSoc;                                  ///< CN: Loss of SoC error object (0x1C0B)
    tErrorObject        cnLossPreq;                                 ///< CN: Loss of PReq error object (0x1C0D)
    tErrorObject        cnCrcErr;                                   ///< CN: CRC error object (0x1C0F)
#if defined(CONFIG_INCLUDE_NMT_MN)
    tErrorObject        mnCrcErr;                                   ///< MN: CRC error object (0x1C00)
    tErrorObject        mnCycTimeExceed;                            ///< MN: Cycle Time Exceeded error object (0x1C02)
    tErrorObject        aMnCnLossPres[NUM_DLL_MNCN_LOSSPRES_OBJS];  ///< MN: CN Loss of PRes error objects (0x1C07, 0x1C08, 0x1C09)
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
} tErrHndObjects;

#endif /* _INC_common_errhnd_H_ */
