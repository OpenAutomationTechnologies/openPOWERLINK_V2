/**
********************************************************************************
\file   kernel/pdok.h

\brief  Include file for kernel PDO module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#ifndef _INC_kernel_pdok_H_
#define _INC_kernel_pdok_H_

//---------------------------------------------------------------------------
// includes
//---------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/pdo.h>
#include <oplk/frame.h>

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError pdok_processRxPdo(const tPlkFrame* pFrame_p, UINT frameSize_p) SECTION_PDOK_PROCESS_RPDO;
tOplkError pdok_init(void);
tOplkError pdok_exit(void);
tOplkError pdok_deAllocChannelMem(void);
tOplkError pdok_allocChannelMem(const tPdoAllocationParam* pAllocationParam_p);
tOplkError pdok_configureChannel(const tPdoChannelConf* pChannelConf_p);
tOplkError pdok_setupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p);

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_pdok_H_ */
