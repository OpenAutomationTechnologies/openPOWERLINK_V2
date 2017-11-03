/**
********************************************************************************
\file   user/sdoudp.h

\brief  Definitions for SDO over UDP protocol abstraction layer

The file contains definitions for the SDO over UDP protocol abstraction layer.
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
#ifndef _INC_user_sdoudp_H_
#define _INC_user_sdoudp_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <user/sdoal.h>
#include <oplk/frame.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// Size of max. UDP payload for a UDP frame transmitted over POWERLINK
// Header for calculation: 20 (IPv4) + 8 (UDP) = 28 bytes
#define SDO_MAX_RX_FRAME_SIZE_UDP       (C_DLL_MAX_ASYNC_MTU - 28)

#define SDOUDP_INADDR_ANY               0

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    UINT32          ipAddr;     /// IP address in network byte order
    UINT16          port;       /// Port in network byte order
} tSdoUdpCon;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#if defined(CONFIG_INCLUDE_SDO_UDP)
tOplkError sdoudp_init(tSequLayerReceiveCb pfnReceiveCb_p);
tOplkError sdoudp_exit(void);
tOplkError sdoudp_config(UINT32 ipAddr_p, UINT16 port_p);
tOplkError sdoudp_initCon(tSdoConHdl* pSdoConHandle_p,
                          UINT targetNodeId_p);
tOplkError sdoudp_sendData(tSdoConHdl sdoConHandle_p,
                           tPlkFrame* pSrcData_p,
                           size_t dataSize_p);
void       sdoudp_receiveData(const tSdoUdpCon* pSdoUdpCon_p,
                              const tAsySdoSeq* pSdoSeqData_p,
                              size_t dataSize_p);
tOplkError sdoudp_delConnection(tSdoConHdl sdoConHandle_p);

tOplkError sdoudp_initSocket(void);
void       sdoudp_exitSocket(void);
tOplkError sdoudp_createSocket(tSdoUdpCon* pSdoUdpCon_p);
tOplkError sdoudp_closeSocket(void);
tOplkError sdoudp_sendToSocket(const tSdoUdpCon* pSdoUdpCon_p,
                               const tPlkFrame* pSrcData_p,
                               size_t dataSize_p);
void       sdoudp_criticalSection(BOOL fEnable_p);
tOplkError sdoudp_arpQuery(UINT32 remoteIpAddr_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_sdoudp_H_ */
