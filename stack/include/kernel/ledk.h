/**
********************************************************************************
\file   kernel/ledk.h

\brief  Definitions for kernel LED module

This file contains definitions and declarations of the kernel LED module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited.
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
#ifndef _INC_kernel_ledk_H_
#define _INC_kernel_ledk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/nmt.h>
#include <oplk/event.h>
#include <common/led.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define LEDK_DURATION_FLICKERING    50      // [ms]
#define LEDK_DURATION_BLINKING      200     // [ms]
#define LEDK_DURATION_FLASH_ON      200     // [ms]
#define LEDK_DURATION_FLASH_OFF     1000    // [ms]

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

//ledk functions
tOplkError ledk_init(void);
tOplkError ledk_exit(void);
tOplkError ledk_handleNmtStateChange(const tEventNmtStateChange* pNmtStateChange_p);
tOplkError ledk_process(void);

//ledktimer functions
tOplkError ledk_timerInit(void);
tOplkError ledk_timerExit(void);
tOplkError ledk_updateLedState(void);
tOplkError ledk_setLedMode(tLedType ledType_p, tLedMode newMode_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_ledk_H_ */
