/**
********************************************************************************
\file   kernel/ctrlk.h

\brief  Definitions for kernel ctrl module

This file contains the definitions for the kernel ctrl module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_kernel_ctrlk_H_
#define _INC_kernel_ctrlk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/ctrl.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Type for ctrlk pre-execute command callback

This type defines a function pointer to the kernel layer driver which is called
if a command is received from the user layer control module.
The callback is invoked before the ctrlk module executes the command.

\param[in]      cmd_p               The command to be executed.
\param[out]     pRet_p              Pointer to store the return value.
\param[out]     pStatus_p           Pointer to store the kernel stack status. (if not NULL)
\param[out]     pfExit_p            Pointer to store the exit flag. (if not NULL)

\return The function returns a BOOL.
\retval TRUE                        Execution completed in callback.
\retval FALSE                       Execution needed in ctrlk module.
*/
typedef BOOL (*tCtrlkExecuteCmdCb)(tCtrlCmdType cmd_p, UINT16* pRet_p, UINT16* pStatus_p, BOOL* pfExit_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError ctrlk_init(tCtrlkExecuteCmdCb pfnExecuteCmdCb_p);
void       ctrlk_exit(void);
BOOL       ctrlk_process(void);
tOplkError ctrlk_executeCmd(tCtrlCmdType cmd,
                            UINT16* pRet_p,
                            UINT16* pStatus_p,
                            BOOL* pfExit_p);
void       ctrlk_updateHeartbeat(void);
UINT16     ctrlk_getHeartbeat(void);
tOplkError ctrlk_readFileChunk(tOplkApiFileChunkDesc* pDesc_p,
                               size_t size_p,
                               void* pBuffer_p);
size_t     ctrlk_getMaxFileChunkSize(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_ctrlk_H_ */
