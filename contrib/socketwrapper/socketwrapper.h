/**
********************************************************************************
\file   socketwrapper.h

\brief  Definitions for socketwrapper

The file contains definitions for the socketwrapper.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_socketwrapper_H_
#define _INC_socketwrapper_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define SOCKETWRAPPER_INVALID       NULL    ///< Invalid socket wrapper instance
#define SOCKETWRAPPER_INADDR_ANY    0       ///< Receive any address from socket wrapper

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief  Socket wrapper instance type

This is the socket wrapper instance type.
*/
typedef void* tSocketWrapper;

/**
\brief  Socket wrapper address structure

This structure defines the socket wrapper address connection.
*/
typedef struct
{
    UINT32      ipAddress;      ///< IP address (V4)
    UINT16      port;           ///< Port
    UINT16      reserved;       ///< Reserved
} tSocketWrapperAddress;

/**
\brief  Socket wrapper receive callback

This type defines the socket wrapper receive callback. It is called by the
socket wrapper module if a frame is received from the socket.

\param[in]      pData_p             Pointer to received data
\param[in]      dataSize_p          Size of received data
\param[in]      pRemote_p           Pointer to socket wrapper address structure

*/
typedef void (*tSocketWrapperReceiveCb)(const void* pData_p,
                                        size_t dataSize_p,
                                        const tSocketWrapperAddress* pRemote_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tSocketWrapper  socketwrapper_create(tSocketWrapperReceiveCb pfnReceiveCb_p);
tOplkError      socketwrapper_bind(tSocketWrapper pSocketWrapper_p,
                                   const tSocketWrapperAddress* pSocketAddress_p);
void            socketwrapper_close(tSocketWrapper pSocketWrapper_p);
tOplkError      socketwrapper_send(tSocketWrapper pSocketWrapper_p,
                                   const tSocketWrapperAddress* pRemote_p,
                                   const void* pData_p,
                                   size_t dataSize_p);
void            socketwrapper_criticalSection(BOOL fEnable_p);
tOplkError      socketwrapper_arpQuery(tSocketWrapper pSocketWrapper_p,
                                       UINT32 remoteIpAddress_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_socketwrapper_H_ */
