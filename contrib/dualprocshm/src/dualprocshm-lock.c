/**
********************************************************************************
\file   dualprocshm-lock.c

\brief  Dual processor library - Target lock

This file provides specific function definitions for the target lock.

\ingroup module_dualprocshm
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <dualprocshm-target.h>
#include <trace/trace.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(acquire)

This routine implements a target specific locking mechanism using the shared
memory between two processors/processes. The caller needs to pass the base
address and processor instance of the calling processor.

The locking is achieved using Peterson's algorithm
\ref https://en.wikipedia.org/wiki/Peterson's_algorithm

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetAcquireLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    tDualprocLock*      pLock = pBase_p;
    tDualProcInstance   otherProcInstance;

    if (pLock == NULL)
        return;

    switch (procInstance_p)
    {
        case kDualProcFirst:
            otherProcInstance = kDualProcSecond;
            break;

        case kDualProcSecond:
            otherProcInstance = kDualProcFirst;
            break;

        default:
            TRACE("Invalid processor instance\n");
            return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pLock, sizeof(tDualprocLock));

    DPSHM_WRITE8(&pLock->afFlag[procInstance_p], 1);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->afFlag[procInstance_p],
                                   sizeof(pLock->afFlag[procInstance_p]));

    DPSHM_WRITE8(&pLock->turn, otherProcInstance);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->turn, sizeof(pLock->turn));

    DPSHM_DMB();

    do
    {
        DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pLock, sizeof(tDualprocLock));
    } while (DPSHM_READ8(&pLock->afFlag[otherProcInstance]) &&
             (DPSHM_READ8(&pLock->turn) == otherProcInstance));
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory unlock routine (release)

This routine is used to release a lock acquired at a specified address.

\param[in,out]  pBase_p             Base address of the lock memory
\param[in]      procInstance_p      Processor instance of the calling processor

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReleaseLock(tDualprocLock* pBase_p,
                                   tDualProcInstance procInstance_p)
{
    tDualprocLock*  pLock = pBase_p;

    if (pLock == NULL)
        return;

    DPSHM_WRITE8(&pLock->afFlag[procInstance_p], 0);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&pLock->afFlag[procInstance_p],
                                   sizeof(pLock->afFlag[procInstance_p]));
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
