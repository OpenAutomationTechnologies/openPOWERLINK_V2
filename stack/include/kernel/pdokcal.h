/**
********************************************************************************
\file   kernel/pdokcal.h

\brief  Include file for kernel PDO CAL module

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
#ifndef _INC_kernel_pdokcal_H_
#define _INC_kernel_pdokcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/pdo.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError pdokcal_init(void);
tOplkError pdokcal_exit(void);
tOplkError pdokcal_process(const tEvent* pEvent_p) SECTION_PDOKCAL_PROCESS;

// PDO memory functions
tOplkError pdokcal_openMem(void);
tOplkError pdokcal_closeMem(void);
tOplkError pdokcal_allocateMem(size_t memSize_p, void** pPdoMem_p);
tOplkError pdokcal_freeMem(void* pMem_p, size_t memSize_p);

// PDO buffer functions
tOplkError pdokcal_initPdoMem(const tPdoChannelSetup* pPdoChannels,
                              size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p);
void       pdokcal_cleanupPdoMem(void);
tOplkError pdokcal_getPdoMemRegion(void** ppPdoMemBase, size_t* pPdoMemSize_p);
tOplkError pdokcal_writeRxPdo(UINT8 channelId_p,
                              const void* pPayload_p,
                              UINT16 pdoSize_p)
                              SECTION_PDOKCAL_WRITE_RPDO;
tOplkError pdokcal_readTxPdo(UINT8 channelId_p,
                             void* pPayload_p,
                             UINT16 pdoSize_p)
                             SECTION_PDOKCAL_READ_TPDO;

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_pdokcal_H_ */
