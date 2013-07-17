/**
********************************************************************************
\file   nmtu.h

\brief  Definitions for nmtu module

This file contains the definitions for the nmtu module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_nmtu_H_
#define _INC_nmtu_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "EplNmt.h"
#include "user/eventu.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* \brief    NMT commands
*
* This enumeration defines all valid NMT commands.
*/
typedef enum
{
    // requestable ASnd ServiceIds    0x01..0x1F
    kNmtCmdIdentResponse             = 0x01,
    kNmtCmdStatusResponse            = 0x02,
    // plain NMT state commands       0x20..0x3F
    kNmtCmdStartNode                 = 0x21,
    kNmtCmdStopNode                  = 0x22,
    kNmtCmdEnterPreOperational2      = 0x23,
    kNmtCmdEnableReadyToOperate      = 0x24,
    kNmtCmdResetNode                 = 0x28,
    kNmtCmdResetCommunication        = 0x29,
    kNmtCmdResetConfiguration        = 0x2A,
    kNmtCmdSwReset                   = 0x2B,
    // extended NMT state commands    0x40..0x5F
    kNmtCmdStartNodeEx               = 0x41,
    kNmtCmdStopNodeEx                = 0x42,
    kNmtCmdEnterPreOperational2Ex    = 0x43,
    kNmtCmdEnableReadyToOperateEx    = 0x44,
    kNmtCmdResetNodeEx               = 0x48,
    kNmtCmdResetCommunicationEx      = 0x49,
    kNmtCmdResetConfigurationEx      = 0x4A,
    kNmtCmdSwResetEx                 = 0x4B,
    // NMT managing commands          0x60..0x7F
    kNmtCmdNetHostNameSet            = 0x62,
    kNmtCmdFlushArpEntry             = 0x63,
    // NMT info services              0x80..0xBF
    kNmtCmdPublishConfiguredCN       = 0x80,
    kNmtCmdPublishActiveCN           = 0x90,
    kNmtCmdPublishPreOperational1    = 0x91,
    kNmtCmdPublishPreOperational2    = 0x92,
    kNmtCmdPublishReadyToOperate     = 0x93,
    kNmtCmdPublishOperational        = 0x94,
    kNmtCmdPublishStopped            = 0x95,
    kNmtCmdPublishEmergencyNew       = 0xA0,
    kNmtCmdPublishTime               = 0xB0,

    kNmtCmdInvalidService            = 0xFF
} tNmtCommand;

typedef tEplKernel (*tNmtuStateChangeCallback) (tEplEventNmtStateChange  NmtStateChange_p);
typedef tEplKernel (*tNmtuCheckEventCallback) (tEplNmtEvent  NmtEvent_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#if defined(CONFIG_INCLUDE_NMTU)

#ifdef __cplusplus
extern "C" {
#endif

tEplKernel      nmtu_init(void);
tEplKernel      nmtu_addInstance(void);
tEplKernel      nmtu_delInstance(void);
tEplKernel      nmtu_postNmtEvent(tEplNmtEvent nmtEvent_p);
tEplNmtState    nmtu_getNmtState(void);
tEplKernel      nmtu_processEvent(tEplEvent* pEvent_p);
tEplKernel      nmtu_registerStateChangeCb(tNmtuStateChangeCallback pfnNmtStateChangeCb_p);

#ifdef __cplusplus
}
#endif

#endif // #if defined(CONFIG_INCLUDE_NMTU)

#endif  // #ifndef _INC_nmtu_H_


