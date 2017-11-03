/**
********************************************************************************
\file   common/ctrlcal-mem.h

\brief  Definitions for ctrl CAL module

This file contains the definitions for the ctrl CAL module.

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
#ifndef _INC_common_ctrlcal_mem_H_
#define _INC_common_ctrlcal_mem_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/ctrl.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CTRL_MAGIC          0xA5A5

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Structure for control command

The structure defines how a command looks like.
*/
typedef struct sCtrlCmd
{
    tCtrlCmdType            cmd;            ///< The command to execute
    UINT16                  retVal;         ///< The return value of the command
} tCtrlCmd;

/**
\brief Structure for control buffer

This structure defines the control buffer which is used to exchange control
information between the kernel and the user layer.
*/
typedef struct
{
    UINT16                  magic;          ///< Magic 0xA5A5 identifies valid structure
    tCtrlKernelStatus       status;         ///< Status of the kernel stack
    UINT16                  heartbeat;      ///< Heartbeat counter
    tCtrlCmd                ctrlCmd;        ///< The control command structure
    tCtrlInitParam          initParam;      ///< The initialization parameter structure
    tOplkApiFileChunkDesc   fileChunkDesc;  ///< File chunk descriptor
    UINT8                   aFileChunkBuffer[CONFIG_CTRL_FILE_CHUNK_SIZE];
                                            ///< File chunk transfer buffer
} tCtrlBuf;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif

#endif /* _INC_common_ctrlcal_mem_H_ */
