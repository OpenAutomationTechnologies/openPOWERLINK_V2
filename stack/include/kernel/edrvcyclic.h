/**
********************************************************************************
\file   kernel/edrvcyclic.h

\brief  Definitions for cyclic Ethernet driver module

This file contains definitions for the cyclic Ethernet driver module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
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
#ifndef _INC_kernel_edrvcyclic_H_
#define _INC_kernel_edrvcyclic_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/edrv.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef CONFIG_EDRV_CYCLIC_USE_DIAGNOSTICS
#define CONFIG_EDRV_CYCLIC_USE_DIAGNOSTICS      FALSE
#endif

#ifndef EDRV_CYCLIC_SAMPLE_NUM
#define EDRV_CYCLIC_SAMPLE_NUM                  501
#endif

//------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------

/// Callback function pointer for Edrv cyclic sync
typedef tOplkError (*tEdrvCyclicCbSync)(void);

/// Callback function pointer for Edrv cyclic error
typedef tOplkError (*tEdrvCyclicCbError)(tOplkError errorCode_p, const tEdrvTxBuffer* pTxBuffer_p);


#if (CONFIG_EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE)
/**
\brief Structure for cyclic Ethernet driver diagnostics

This structure is used to provide diagnostics of the cyclic Ethernet driver.
*/
typedef struct
{
    // continuous min/max/avg measurement
    ULONGLONG   cycleCount;                                 ///< Cycle counter
    UINT32      cycleTimeMin;                               ///< Minimum measured cycle time
    UINT32      cycleTimeMax;                               ///< Maximum measured cycle time
    ULONGLONG   cycleTimeMeanSum;                           ///< Sum of the mean measured cycle times
    UINT32      usedCycleTimeMin;                           ///< Minimum utilized cycle time
    UINT32      usedCycleTimeMax;                           ///< Maximum utilized cycle time
    ULONGLONG   usedCycleTimeMeanSum;                       ///< Sum of the mean utilized cycle times
    UINT32      spareCycleTimeMin;                          ///< Minimum spare cycle time
    UINT32      spareCycleTimeMax;                          ///< Maximum spare cycle time
    ULONGLONG   spareCycleTimeMeanSum;                      ///< Sum of the mean spare cycle times
    // sampling of runaway cycles
    UINT        sampleNum;                                  ///< Sample number
    UINT        sampleBufferedNum;                          ///< Buffered sample number
    ULONGLONG   aSampleTimeStamp[EDRV_CYCLIC_SAMPLE_NUM];   ///< Array of sampled timestamps (SoC send)
    UINT32      aCycleTime[EDRV_CYCLIC_SAMPLE_NUM];         ///< Array of cycle time values (until next SoC send)
    UINT32      aUsedCycleTime[EDRV_CYCLIC_SAMPLE_NUM];     ///< Array of used cycle time values
    UINT32      aSpareCycleTime[EDRV_CYCLIC_SAMPLE_NUM];    ///< Array of spare cycle time values
} tEdrvCyclicDiagnostics;
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError edrvcyclic_init(void);
tOplkError edrvcyclic_exit(void);
tOplkError edrvcyclic_setMaxTxBufferListSize(UINT maxListSize_p);
tOplkError edrvcyclic_setNextTxBufferList(tEdrvTxBuffer* const* ppTxBuffer_p,
                                          UINT txBufferCount_p) SECTION_EDRVCYC_SET_NEXT_TX;
tOplkError edrvcyclic_setCycleTime(UINT32 cycleTimeUs_p, UINT32 minSyncTime_p);
tOplkError edrvcyclic_startCycle(BOOL fContinuousMode_p);
tOplkError edrvcyclic_stopCycle(BOOL fKeepCycle_p);
tOplkError edrvcyclic_regSyncHandler(tEdrvCyclicCbSync pfnEdrvCyclicCbSync_p);
tOplkError edrvcyclic_regErrorHandler(tEdrvCyclicCbError pfnEdrvCyclicCbError_p);

#if (CONFIG_EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE)
tOplkError edrvcyclic_getDiagnostics(const tEdrvCyclicDiagnostics** ppDiagnostics_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_edrvcyclic_H_ */
