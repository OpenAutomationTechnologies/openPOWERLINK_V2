/**
********************************************************************************
\file   user/sdoal.h

\brief  Definitions for SDO protocol abstraction layer modules

This file contains common definitions for the SDO protocol abstraction layer
modules.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_user_sdoal_H_
#define _INC_user_sdoal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/frame.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// handle between protocol abstraction layer and asynchronous SDO Sequence Layer
#define SDO_UDP_HANDLE              0x8000
#define SDO_ASND_HANDLE             0x4000
#define SDO_ASY_HANDLE_MASK         0xC000
#define SDO_ASY_INVALID_HDL         0x3FFF

#define ASND_HEADER_SIZE            4


//------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------

/// Data type for handle between protocol abstraction layer and asynchronous SDO Sequence Layer
typedef UINT tSdoConHdl;

/// Callback function pointer for the protocol abstraction layer to call the asynchronous SDO Sequence Layer
typedef tOplkError (*tSequLayerReceiveCb)(tSdoConHdl conHdl_p,
                                          const tAsySdoSeq* pSdoSeqData_p,
                                          size_t dataSize_p);

#endif /* _INC_user_sdoal_H_ */
