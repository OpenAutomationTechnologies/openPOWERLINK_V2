/**
********************************************************************************
\file   common/target.h

\brief  Definitions for target module

This file contains the definitions for the target modules.

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
#ifndef _INC_common_target_H_
#define _INC_common_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/led.h>

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

tOplkError target_init(void);
tOplkError target_cleanup(void);
void       target_msleep(UINT32 milliSeconds_p);
tOplkError target_setIpAdrs(const char* ifName_p,
                            UINT32 ipAddress_p,
                            UINT32 subnetMask_p,
                            UINT16 mtu_p);
tOplkError target_setDefaultGateway(UINT32 defaultGateway_p);
void       target_enableGlobalInterrupt(BOOL fEnable_p) SECTION_TARGET_GLOBAL_INT;
void       target_setInterruptContextFlag(BOOL fEnable_p) SECTION_TARGET_SET_INTCONT;
BOOL       target_getInterruptContextFlag(void) SECTION_TARGET_GET_INTCONT;
UINT32     target_getTickCount(void);
ULONGLONG  target_getCurrentTimestamp(void);

/* functions for mutex implementation */
tOplkError target_createMutex(const char* mutexName_p,
                              OPLK_MUTEX_T* pMutex_p);
tOplkError target_lockMutex(OPLK_MUTEX_T mutexId_p);
void       target_unlockMutex(OPLK_MUTEX_T mutexId_p);
void       target_destroyMutex(OPLK_MUTEX_T mutexId_p);

/* functions for lock implementation */
int        target_initLock(OPLK_LOCK_T* pSlock_p);
int        target_lock(void);
int        target_unlock(void);

/* function for LED */
tOplkError target_setLed(tLedType ledType_p,
                         BOOL fLedOn_p);

/* function to enumerate interfaces */
tOplkError target_enumerateNetworkInterfaces(tNetIfId* pInterfaces_p,
                                             size_t* pNoInterfaces_p);
/* function to get current system timestamp */
#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
tOplkError target_getSystemTime(tNetTime* pNetTime_p, BOOL* pValidSystemTime_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_common_target_H_ */
