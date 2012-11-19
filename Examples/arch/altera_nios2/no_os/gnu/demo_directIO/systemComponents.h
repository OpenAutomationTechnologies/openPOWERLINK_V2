/**
********************************************************************************
\file       systemComponents.h

\brief      Module which contains of processor specific functions
            (nios II version)

Provides all functions which are platform dependent for the application of the
directIO example.

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

#ifndef SYSTEMCOMPONENTS_H
#define SYSTEMCOMPONENTS_H

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "global.h"
#include "system.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifdef POWERLINK_0_SMP_BASE
#define LATCHED_IOPORT_BASE   (void*) POWERLINK_0_SMP_BASE
#define LATCHED_IOPORT_CFG    (void*) (LATCHED_IOPORT_BASE + 4)
#endif

#ifdef POWERLINK_0_PDI_PCP_BASE
    #define PDI_DPRAM_BASE_PCP    POWERLINK_0_PDI_PCP_BASE
    #ifdef STATUS_LED_PIO_BASE
      #warning Deprecated! Deactivate STATUS_LED_PIO_BASE in SOPC-Builder!
    #endif // STATUS_LED_PIO_BASE
#endif //POWERLINK_0_PDI_PCP_BASE

#if defined(POWERLINK_0_PDI_PCP_PDITPDOBUFSIZE0) &&  defined(POWERLINK_0_PDI_PCP_PDIRPDOBUFSIZE0)
#define PDI_TPDO_BUF_SIZE   POWERLINK_0_PDI_PCP_PDITPDOBUFSIZE0
#define PDI_RPDO_BUF_SIZE   POWERLINK_0_PDI_PCP_PDIRPDOBUFSIZE0
#define PDI_TPDO_MAX_PAYLOAD   (POWERLINK_0_PDI_PCP_PDITPDOBUFSIZE0 - 0)    ///< TPDO PDI buffer does not have any header
#define PDI_RPDO_MAX_PAYLOAD   (POWERLINK_0_PDI_PCP_PDIRPDOBUFSIZE0 - 16)   ///< withdraw RPDO PDI buffer header (16 bytes)
#endif

#if defined(POWERLINK_0_PDI_PCP_PDITPDOS) && defined(POWERLINK_0_PDI_PCP_PDIRPDOS)
#define TPDO_CHANNELS_MAX     POWERLINK_0_PDI_PCP_PDITPDOS ///< Max Number of TxPDO's of this CN
#define RPDO_CHANNELS_MAX     POWERLINK_0_PDI_PCP_PDIRPDOS ///< Max Number of RxPDO's of this CN
#endif

#ifdef STATUS_LED_PIO_BASE
#define STATUS_LEDS_BASE STATUS_LED_PIO_BASE
#endif

#ifdef NODE_SWITCH_PIO_BASE
#define NODE_SWITCH_BASE NODE_SWITCH_PIO_BASE
#endif //NODE_SWITCH_PIO_BASE

#ifdef NODE_SWITCH_SPI_BASE
#define NODE_SWITCH_BASE NODE_SWITCH_SPI_BASE
#endif //NODE_SWITCH_SPI_BASE

#ifndef NODE_SWITCH_BASE
 #warning No Node ID module present in SOPC. Node ID can only be set by SW!
#endif //NODE_SWITCH_BASE

#if POWERLINK_0_MAC_CMP_TIMESYNCHW != FALSE
#define TIMESYNC_HW
#endif //POWERLINK_0_MAC_CMP_TIMESYNCHW

#ifdef POWERLINK_0_PDI_PCP_ASYNCBUFCOUNT
#define PDI_ASYNC_CHANNELS_MAX POWERLINK_0_PDI_PCP_ASYNCBUFCOUNT
#endif //POWERLINK_0_PDI_PCP_ASYNCBUFCOUNT


#ifdef SYSID_ID
#define FPGA_SYSTEM_ID        SYSID_ID
#endif //SYSID_ID

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
void SysComp_initPeripheral(void);
void SysComp_freeProcessorCache(void);
void SysComp_enableInterrupts(void);

BYTE SysComp_getNodeId(void);
void SysComp_setPowerlinkStatus(BYTE bBitNum_p);
void SysComp_resetPowerlinkStatus(BYTE bBitNum_p);

#endif /* _INC_SYSTEMCOMPONENTS_H_ */

