/**
********************************************************************************
\file   hostiflib_l.h

\brief  Host Interface Library - Low Level Driver Header

The Host Interface Low Level Driver provides access to the status control
register structures.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_hostiflib_l_H_
#define _INC_hostiflib_l_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define HOSTIF_BRIDGE_ENABLE      0x0001    ///< enables the bridge
#define HOSTIF_IRQ_MASTER_ENABLE  0x0001    ///< enabled the IRQ master

#define HOSTIF_BUF_COUNT          10        ///< number of supported buffers

#define HOSTIF_STCTRL_SPAN        2048      ///< size of status control register

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

UINT32  hostif_readMagic(const void* pHostifScBase_p);
UINT32  hostif_readVersion(const void* pHostifScBase_p);
UINT32  hostif_readBootBase(const void* pHostifScBase_p);
void    hostif_writeBootBase(void* pHostifScBase_p, UINT32 val_p);
UINT32  hostif_readInitBase(const void* pHostifScBase_p);
void    hostif_writeInitBase(void* pHostifScBase_p, UINT32 val_p);

UINT16  hostif_readBridgeEnable(const void* pHostifScBase_p);
void    hostif_writeBridgeEnable(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readCommand(const void* pHostifScBase_p);
void    hostif_writeCommand(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readState(const void* pHostifScBase_p);
void    hostif_writeState(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readReturn(const void* pHostifScBase_p);
void    hostif_writeReturn(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readHeartbeat(const void* pHostifScBase_p);
void    hostif_writeHeartbeat(void* pHostifScBase_p, UINT16 val_p);

UINT16  hostif_readIrqEnable(const void* pHostifScBase_p);
void    hostif_writeIrqEnable(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readIrqPending(const void* pHostifScBase_p);
UINT16  hostif_readIrqMasterEnable(const void* pHostifScBase_p);
void    hostif_writeIrqMasterEnable(void* pHostifScBase_p, UINT16 val_p);
void    hostif_ackIrq(void* pHostifScBase_p, UINT16 val_p);
void    hostif_setIrq(void* pHostifScBase_p, UINT16 val_p);
UINT16  hostif_readSyncConfig(const void* pHostifScBase_p);
void    hostif_writeSyncConfig(void* pHostifScBase_p, UINT16 val_p);

UINT32  hostif_readDynBufHost(const void* pHostifScBase_p, UINT8 num_p);
void    hostif_writeDynBufHost(void* pHostifScBase_p, UINT8 num_p, UINT32 addr_p);

UINT32  hostif_readBufPcp(const void* pHostifScBase_p, UINT8 num_p);
void    hostif_writeBufPcp(void* pHostifScBase_p, UINT8 num_p, UINT32 addr_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_hostiflib_l_H_ */
