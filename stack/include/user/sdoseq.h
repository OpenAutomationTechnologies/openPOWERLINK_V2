/**
********************************************************************************
\file   user/sdoseq.h

\brief  Definitions for SDO sequence layer module

The file contains definitions for the SDO sequence layer module.
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
#ifndef _INC_user_sdoseq_H_
#define _INC_user_sdoseq_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/sdo.h>
#include <oplk/frame.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// handle between SDO Sequence Layer and SDO Command Layer
#define SDO_ASY_HANDLE              0x8000
#define SDO_PDO_HANDLE              0x4000
#define SDO_SEQ_HANDLE_MASK         0xC000
#define SDO_SEQ_INVALID_HDL         0x3FFF

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Enumeration lists valid SDO connection states

This enumeration lists all valid SDO connection states.
*/
typedef enum
{
    kAsySdoConStateConnected            = 0x00,     ///< An SDO connection is established
    kAsySdoConStateInitError            = 0x01,     ///< An error occurred during initialization
    kAsySdoConStateConClosed            = 0x02,     ///< The SDO connection is closed
    kAsySdoConStateAckReceived          = 0x03,     ///< An acknowledge has been received
    kAsySdoConStateFrameSent            = 0x04,     /**< A frame with command layer data
                                                         has been sent or added to the
                                                         Tx history buffer */
    kAsySdoConStateFrameReceived        = 0x05,     /**< A frame with command layer data
                                                         was received */
    kAsySdoConStateTimeout              = 0x06,     ///< A timeout has occurred
    kAsySdoConStateTransferAbort        = 0x07,     ///< The SDO transfer has been aborted
} eAsySdoConState;

/**
\brief SDO connection state data type

Data type for the enumerator \ref eAsySdoConState.
*/
typedef UINT32 tAsySdoConState;

/// Data type for handle between asynchronous SDO Sequence Layer and SDO Command Layer
typedef UINT tSdoSeqConHdl;

/// Callback function pointer for asynchronous SDO Sequence Layer to call SDO Command Layer for connection status
typedef tOplkError (*tSdoComConCb)(tSdoSeqConHdl sdoSeqConHdl_p,
                                   tAsySdoConState asySdoConState_p);

/// Callback function pointer for asynchronous SDO Sequence Layer to call SDO Command Layer for received data
typedef tOplkError (*tSdoComReceiveCb)(tSdoSeqConHdl sdoSeqConHdl_p,
                                       const tAsySdoCom* pAsySdoCom_p,
                                       size_t dataSize_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError sdoseq_init(tSdoComReceiveCb pfnSdoComRecvCb_p,
                       tSdoComConCb pfnSdoComConCb_p);
tOplkError sdoseq_exit(void);
tOplkError sdoseq_initCon(tSdoSeqConHdl* pSdoSeqConHdl_p,
                          UINT nodeId_p,
                          tSdoType sdoType_p);
tOplkError sdoseq_sendData(tSdoSeqConHdl sdoSeqConHdl_p,
                           size_t dataSize_p,
                           tPlkFrame* pData_p);
tOplkError sdoseq_processEvent(const tEvent* pEvent_p);
tOplkError sdoseq_deleteCon(tSdoSeqConHdl sdoSeqConHdl_p);
tOplkError sdoseq_setTimeout(UINT32 timeout_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_sdoseq_H_ */
