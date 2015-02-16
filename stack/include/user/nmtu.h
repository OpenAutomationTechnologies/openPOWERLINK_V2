/**
********************************************************************************
\file   nmtu.h

\brief  Definitions for nmtu module

This file contains the definitions for the nmtu module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/oplkinc.h>
#include <oplk/nmt.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NMT_PLAIN_COMMAND_START     0x20        // Start of plain NMT command range
#define NMT_PLAIN_COMMAND_END       0x3F        // End of plain NMT command range

#define NMT_EXT_COMMAND_START       0x40        // Start of extended NMT command range
#define NMT_EXT_COMMAND_END         0x5F        // End of extended NMT command range

#define C_MAX_NMT_CMD_DATA_SIZE (C_DLL_MAX_PAYL_OFFSET - 6)

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
    kNmtCmdIdentResponse             = 0x01,            ///< IdentResponse command
    kNmtCmdStatusResponse            = 0x02,            ///< StatusResponse command
    // plain NMT state commands       0x20..0x3F
    kNmtCmdStartNode                 = 0x21,            ///< NMTStartNode command
    kNmtCmdStopNode                  = 0x22,            ///< NMTStopNode command
    kNmtCmdEnterPreOperational2      = 0x23,            ///< NMTEnterPreOperational2 command
    kNmtCmdEnableReadyToOperate      = 0x24,            ///< NMTEnableReadyToOperate command
    kNmtCmdResetNode                 = 0x28,            ///< NMTResetNode command
    kNmtCmdResetCommunication        = 0x29,            ///< NMTResetCommunication command
    kNmtCmdResetConfiguration        = 0x2A,            ///< NMTResetConfiguration command
    kNmtCmdSwReset                   = 0x2B,            ///< NMTSwReset command
    // extended NMT state commands    0x40..0x5F
    kNmtCmdStartNodeEx               = 0x41,            ///< NMTStartNodeEx command
    kNmtCmdStopNodeEx                = 0x42,            ///< NMTStopNodeEx command
    kNmtCmdEnterPreOperational2Ex    = 0x43,            ///< NMTEnterPreOperational2Ex command
    kNmtCmdEnableReadyToOperateEx    = 0x44,            ///< NMTEnableReadyToOperateEx command
    kNmtCmdResetNodeEx               = 0x48,            ///< NMTResetNodeEx command
    kNmtCmdResetCommunicationEx      = 0x49,            ///< NMTResetCommunicationEx command
    kNmtCmdResetConfigurationEx      = 0x4A,            ///< NMTResetConfigurationEx command
    kNmtCmdSwResetEx                 = 0x4B,            ///< NMTSwResetEx command
    // NMT managing commands          0x60..0x7F
    kNmtCmdNetHostNameSet            = 0x62,            ///< NMTNetHostNameSet command
    kNmtCmdFlushArpEntry             = 0x63,            ///< NMTFlushArpEntry command
    // NMT info services              0x80..0xBF
    kNmtCmdPublishConfiguredCN       = 0x80,            ///< NMTPublishConfiguredNodes command
    kNmtCmdPublishActiveCN           = 0x90,            ///< NMTPublishActiveNodes command
    kNmtCmdPublishPreOperational1    = 0x91,            ///< NMTPublishPreOperational1 command
    kNmtCmdPublishPreOperational2    = 0x92,            ///< NMTPublishPreOperational2 command
    kNmtCmdPublishReadyToOperate     = 0x93,            ///< NMTPublishReadyToOperate command
    kNmtCmdPublishOperational        = 0x94,            ///< NMTPublishOperational command
    kNmtCmdPublishStopped            = 0x95,            ///< NMTPublishStopped command
    kNmtCmdPublishNodeStates         = 0x96,            ///< NMTPublishNodeStates command
    kNmtCmdPublishEmergencyNew       = 0xA0,            ///< NMTPublishEmergencyNew command
    kNmtCmdPublishTime               = 0xB0,            ///< NMTPublishTime command

    kNmtCmdInvalidService            = 0xFF             ///< NMTInvalidService command
} tNmtCommand;

typedef tOplkError (*tNmtuStateChangeCallback)(tEventNmtStateChange NmtStateChange_p);
typedef tOplkError (*tNmtuCheckEventCallback)(tNmtEvent NmtEvent_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tOplkError      nmtu_init(void);
tOplkError      nmtu_exit(void);
tOplkError      nmtu_postNmtEvent(tNmtEvent nmtEvent_p);
tNmtState       nmtu_getNmtState(void);
tOplkError      nmtu_processEvent(tEvent* pEvent_p);
tOplkError      nmtu_registerStateChangeCb(tNmtuStateChangeCallback pfnNmtStateChangeCb_p);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_nmtu_H_
