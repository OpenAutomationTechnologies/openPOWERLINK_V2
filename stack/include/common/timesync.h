/**
********************************************************************************
\file   common/timesync.h

\brief  Include file for timesync module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#ifndef _INC_common_timesync_H_
#define _INC_common_timesync_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMESYNC_SYNC_BSDSEM            "/semTimeSyncSync"
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#define TIMESYNC_TIMESTAMP_SHM          "/shmTimeSyncTimestamp"
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief  SoC time

This structure defines the SoC time information.
*/
typedef struct
{
    tNetTime                netTime;        ///< Net time in IEEE 1588 format
    UINT64                  relTime;        ///< Relative time in us
    UINT8                   fRelTimeValid;  ///< TRUE if relTime is valid
    UINT8                   aReserved[3];   ///< Reserved
    UINT32                  padding1;       ///< Padding to achieve 64 bit alignment
} tTimesyncSocTime;

/**
\brief  Triple buffer for SoC time transfer

This structure defines the triple buffer memory to transfer the SoC time
information from the kernel to the user layer. The triple buffer mechanism
allows the application to access the SoC time information asynchronously.
*/
typedef struct
{
    OPLK_ATOMIC_T           write;          ///< Write buffer (current producer)
    OPLK_ATOMIC_T           read;           ///< Read buffer (current consumer)
    OPLK_ATOMIC_T           clean;          ///< Clean buffer
    UINT8                   newData;        ///< Signalizes new data
    UINT32                  padding1;       ///< Padding to achieve 64 bit alignment
    tTimesyncSocTime        aTripleBuf[3];  ///< Triple buffer
} tTimesyncSocTimeTripleBuf;

/**
\brief  Timesync shared memory

This structure defines the timesync module shared memory. These are used to
transfer SoC time information from the kernel to the user layer and from the
user to the kernel layer.
*/
typedef struct
{
    tTimesyncSocTimeTripleBuf   kernelToUserSocTime;    ///< Buffer to transfer SoC time information from kernel to user.
#if defined(CONFIG_INCLUDE_NMT_MN)
    tTimesyncSocTimeTripleBuf   userToKernelSocTime;    ///< Buffer to transfer SoC time information from user to kernel.
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
} tTimesyncSharedMemory;

#endif /* _INC_common_timesync_H_ */
