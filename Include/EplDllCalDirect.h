/**
********************************************************************************
\file   EplDllCalDirect.h

\brief  header file for DLL CAL direct call module

This DLL CAL queue implementation applies a single buffer for each queue
instance.

Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Kalycito Infotech Private Ltd.
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
*******************************************************************************/

#ifndef _INC_EPLDLLCALDIRECT_H_
#define _INC_EPLDLLCALDIRECT_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "EplDll.h"
#include "EplDllCal.h"

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
extern "C" {
#endif

tEplKernel EplDllCalDirectAddInstance (tEplDllCalQueueInstance *ppDllCalQueue_p,
        tEplDllCalQueue DllCalQueue_p);

tEplKernel EplDllCalDirectDelInstance (tEplDllCalQueueInstance pDllCalQueue_p);

tEplKernel EplDllCalDirectInsertDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize);

tEplKernel EplDllCalDirectGetDataBlock (tEplDllCalQueueInstance pDllCalQueue_p,
        BYTE *pData_p, unsigned int *puiDataSize);

tEplKernel EplDllCalDirectGetDataBlockCount (
        tEplDllCalQueueInstance pDllCalQueue_p,
        unsigned long *pulDataBlockCount);

tEplKernel EplDllCalDirectResetDataBlockQueue (
        tEplDllCalQueueInstance pDllCalQueue_p);


#ifdef __cplusplus
}
#endif

#endif /* _INC_EPLDLLCALDIRECT_H_ */
