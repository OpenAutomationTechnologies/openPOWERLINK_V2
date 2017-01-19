/**
********************************************************************************
\file   drv_kernelmod_zynq/zynq.h

\brief  openPOWERLINK zynq driver header file

openPOWERLINK Kernel driver for zynq/FPGA - Header file

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Kalycito Infotech Private Limited
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
#ifndef _INC_zynqdrv_H_
#define _INC_zynqdrv_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
*  \brief Platform device IO memory regions
*
*  The enumeration defines the platform device IO memory regions
*/
typedef enum
{
    kIoMemReg1,
    kIoMemReg2,
    kIoMemRegCount
} eIOMemReg;

/**
\brief Platform device IO memory regions data type

Data type for the enumerator \ref eIOMemReg.
*/
typedef UINT32 tIoMemReg;

typedef tOplkError (*tIrqCallback)(void);   ///< Function signature of ISR callback for upper layer

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError  zynqdrv_init(void);
tOplkError  zynqdrv_exit(void);

void*       zynqdrv_getMemRegionAddr(UINT8 memId_p);
void*       zynqdrv_getMemPhyAddr(UINT8 memId_p);
tOplkError  zynqdrv_regSyncHandler(tIrqCallback cbSync_p);
tOplkError  zynqdrv_enableSync(BOOL fEnable_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_zynqdrv_H_ */
