/**
********************************************************************************
\file   errsigk.h

\brief  External interface of the error handler kernel module

This header provides the external interface of the error handler kernel module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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

#ifndef _INC_errsigk_H_
#define _INC_errsigk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Enumeration for buffer owner

The enumeration is used to describe the owners of error status buffers.
*/
typedef enum
{
    kOwnerDll               = 0,    ///< Dll owns the current buffer
    kOwnerErrSigk,                  ///< Error Signalling module owns the current buffer
    kOwnerReserved                  ///< Current frame's ownership is non-changeable
}tErrBufOwner;

/**
\brief  Status entry buffer

The structure defines the status entry fields of error signalling module and status response frame
*/
typedef struct ErrSigkBuffer
{
    BOOL                    fDataValid;                  ///< Data with in the buffer is valid or invalid
    tErrBufOwner            bufOwner;                    ///< Owner of the buffer
    UINT8                   numberOfHistoryEntries;      ///< Number of Error Entries from Emergency queue in the current buffer
    UINT8                   numberOfStatusEntries;       ///< Number of Error Entries in the current buffer
    struct ErrSigkBuffer*   pNextErrorBuffer;            ///< pointer to Next StatusEntry Buffer structure
    UINT64                  staticError;                 ///< static error bit field
    tErrHistoryEntry*       pErrHistoryEntry;            ///< History entry
    tErrHistoryEntry*       pErrStatusEntry;             ///< Status entry
}PACK_STRUCT tErrSigkBuffer;

/**
\brief Enumeration for Error Signal Buffer Status

The enumeration contains status for an Error signalling buffer.
*/
typedef enum
{
    kNoBuffer = 0,          ///< No ErrorSignalling buffer is initialised
    kBuffersEmpty,          ///< No ErrorSignalling buffer is used
    kBuffersAvailable,      ///< Some buffers excluding the reserved one,
                            /// with ErrorSignaling module are occupied
    kBuffersFull,           ///< All buffers excluding the reserved one,
                            /// with ErrorSignaling module are occupied
    kReservedBufferFull     ///< All buffers including the reserved one,
                            /// with ErrorSignaling module are occupied
}tErrSigkBufferStatus;
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// init function
tOplkError errsigk_init(void);
// delete instance
tOplkError errsigk_exit(void);
// reset status
tOplkError errsigk_reset(void);
// allocate the buffers from dll
tOplkError errsigk_createErrStatusBuffers(tErrSigkBuffer** ppdllErrStatusBuffer_p);
// deallocate buffers from dll
tOplkError errsigk_cleanErrStatusBuffers(tErrSigkBuffer** ppdllErrStatusBuffer_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_errsigk_H_ */
