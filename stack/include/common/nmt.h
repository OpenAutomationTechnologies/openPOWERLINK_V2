/**
********************************************************************************
\file   common/nmt.h

\brief  Common include file for NMT modules

This file is the common include file for all NMT modules
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_common_nmt_H_
#define _INC_common_nmt_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

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
    kNmtCmdGoToStandby               = 0x2C,            ///< NMTGoToStandby command (RMN)
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
} eNmtCommand;

/**
\brief NMT command data type

Data type for the enumerator \ref eNmtCommand.
*/
typedef UINT32 tNmtCommand;

#endif /* _INC_common_nmt_H_ */
