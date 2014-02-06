/**
********************************************************************************
\file   nmt.h

\brief  Global include file for NMT modules

This file is the global include file for all NMT modules
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_nmt_H_
#define _INC_nmt_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// define super-states and masks to identify a super-state
#define NMT_GS_POWERED              0x0008  // super state
#define NMT_GS_INITIALISATION       0x0009  // super state
#define EPL_NMT_GS_COMMUNICATING    0x000C  // super state
#define NMT_CS_PLKMODE              0x000D  // super state
#define NMT_MS_PLKMODE              0x000D  // super state

#define NMT_SUPERSTATE_MASK         0x000F  // mask to select state

#define NMT_TYPE_UNDEFINED          0x0000  // type of NMT state is still undefined
#define NMT_TYPE_CS                 0x0100  // CS type of NMT state
#define NMT_TYPE_MS                 0x0200  // MS type of NMT state
#define NMT_TYPE_MASK               0x0300  // mask to select type of NMT state (i.e. CS or MS)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* \brief NMT states
*
* The enumeration lists all valid NMT states. The lower Byte of the NMT-State
* is encoded like the values in the POWERLINK standard. The higher byte is used
* to encode MN (Bit 1 of the higher byte = 1) or CN (Bit 0 of the
* higher byte  = 1). The super-states are not mentioned in this enum because
* they are no real states --> there are masks defined to identify the
* super-states.
*
* The order of the states is important as it is used in the source code to
* determine several things:
*
* state > kMntGsResetConfiguration:  No reset state
* state >= kNmtMsNotActive:          Node is running as MN
* state < kNmtMsNotActive:           Node is running as CN
*/
typedef enum
{
    kNmtGsOff                       = 0x0000,
    kNmtGsInitialising              = 0x0019,
    kNmtGsResetApplication          = 0x0029,
    kNmtGsResetCommunication        = 0x0039,
    kNmtGsResetConfiguration        = 0x0079,
    kNmtCsNotActive                 = 0x011C,
    kNmtCsPreOperational1           = 0x011D,
    kNmtCsStopped                   = 0x014D,
    kNmtCsPreOperational2           = 0x015D,
    kNmtCsReadyToOperate            = 0x016D,
    kNmtCsOperational               = 0x01FD,
    kNmtCsBasicEthernet             = 0x011E,
    kNmtMsNotActive                 = 0x021C,
    kNmtMsPreOperational1           = 0x021D,
    kNmtMsPreOperational2           = 0x025D,
    kNmtMsReadyToOperate            = 0x026D,
    kNmtMsOperational               = 0x02FD,
    kNmtMsBasicEthernet             = 0x021E
} tNmtState;

/**
* \brief NMT events
*
* This enumeration lists all valid NMT events.
*/
typedef enum
{
    // Events from DLL
    // Events defined by EPL V2 specification
    kNmtEventNoEvent                =   0x00,
    // kNmtEventDllMePres           =   0x01,
    kNmtEventDllMePresTimeout       =   0x02,
    // kNmtEventDllMeAsnd           =   0x03,
    kNmtEventDllMeAsndTimeout       =   0x03,
    // kNmtEventDllMeSoaSent        =   0x04,
    kNmtEventDllMeSocTrig           =   0x05,
    kNmtEventDllMeSoaTrig           =   0x06,
    kNmtEventDllCeSoc               =   0x07,
    kNmtEventDllCePreq              =   0x08,
    kNmtEventDllCePres              =   0x09,
    kNmtEventDllCeSoa               =   0x0A,
    kNmtEventDllCeAInv              =   0x0B,
    kNmtEventDllCeAsnd              =   0x0C,
    kNmtEventDllCeFrameTimeout      =   0x0D,

    // Events triggered by NMT-Commands
    kNmtEventSwReset                =   0x10,   ///< NMT_GT1, NMT_GT2, NMT_GT8
    kNmtEventResetNode              =   0x11,
    kNmtEventResetCom               =   0x12,
    kNmtEventResetConfig            =   0x13,
    kNmtEventEnterPreOperational2   =   0x14,
    kNmtEventEnableReadyToOperate   =   0x15,
    kNmtEventStartNode              =   0x16,   ///< NMT_CT7
    kNmtEventStopNode               =   0x17,

    // Events triggered by higher layer
    kNmtEventEnterResetApp          =   0x20,
    kNmtEventEnterResetCom          =   0x21,
    kNmtEventInternComError         =   0x22,   ///< NMT_GT6, internal communication error -> enter ResetCommunication
    kNmtEventEnterResetConfig       =   0x23,
    kNmtEventEnterCsNotActive       =   0x24,
    kNmtEventEnterMsNotActive       =   0x25,
    kNmtEventTimerBasicEthernet     =   0x26,   ///< NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    kNmtEventTimerMsPreOp1          =   0x27,   ///< enter PreOp1 on MN (NotActive -> MsPreOp1)
    kNmtEventNmtCycleError          =   0x28,   ///< NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    kNmtEventTimerMsPreOp2          =   0x29,   ///< enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if kNmtEventAllMandatoryCNIdent)
    kNmtEventAllMandatoryCNIdent    =   0x2A,   ///< enter PreOp2 on MN if kNmtEventTimerMsPreOp2
    kNmtEventEnterReadyToOperate    =   0x2B,   ///< application ready for the state ReadyToOp
    kNmtEventEnterMsOperational     =   0x2C,   ///< enter Operational on MN
    kNmtEventSwitchOff              =   0x2D,   ///< enter state Off
    kNmtEventCriticalError          =   0x2E,   ///< enter state Off because of critical error
} tNmtEvent;

/**
* \brief NMT state change event
*
* This structure defines the NMT state change event.
*/
typedef struct
{
    tNmtState               newNmtState;        ///< New NMT state
    tNmtState               oldNmtState;        ///< Old NMT state
    tNmtEvent               nmtEvent;           ///< NMT event
} tEventNmtStateChange;

/**
* \brief Heartbeat event
*
* This structure defines the heartbeat event.
*/
typedef struct
{
    UINT                    nodeId;             ///< Node ID.
    tNmtState               nmtState;           ///< NMT state (remember distinguish between MN / CN)
    UINT16                  errorCode;          ///< POWERLINK error code in case of NMT state NotActive
} tHeartbeatEvent;


/**
* \brief Node events
*
* The enumeration lists the valid node events.
*/
typedef enum
{
    kNmtNodeEventFound              = 0x00,
    kNmtNodeEventUpdateSw           = 0x01,     ///< application shall update software on CN
    kNmtNodeEventCheckConf          = 0x02,     ///< application / Configuration Manager shall check and update configuration on CN
    kNmtNodeEventUpdateConf         = 0x03,     ///< application / Configuration Manager shall update configuration on CN (check was done by NmtMn module)
    kNmtNodeEventVerifyConf         = 0x04,     ///< application / Configuration Manager shall verify configuration of CN
    kNmtNodeEventReadyToStart       = 0x05,     ///< issued if EPL_NMTST_NO_STARTNODE set, application must call EplNmtMnuSendNmtCommand(kEplNmtCmdStartNode) manually
    kNmtNodeEventNmtState           = 0x06,
    kNmtNodeEventError              = 0x07,     ///< NMT error of CN
} tNmtNodeEvent;

/**
* \brief NMT node commands
*
* This enumeration lists all valid NMT node commands.
*/
typedef enum
{
    kNmtNodeCommandBoot             = 0x01,     ///< if EPL_NODEASSIGN_START_CN not set it must be issued after kNmtNodeEventFound
    kNmtNodeCommandSwOk             = 0x02,     ///< application updated software on CN successfully
    kNmtNodeCommandSwUpdated        = 0x03,     ///< application updated software on CN successfully
    kNmtNodeCommandConfOk           = 0x04,     ///< application / Configuration Manager has updated configuration on CN successfully
    kNmtNodeCommandConfRestored     = 0x05,     ///< application / Configuration Manager has restored the original CN configuration and CN need ResetNode to complete the restore process, afterwards the new configuration can be downloaded
    kNmtNodeCommandConfReset        = 0x06,     ///< application / Configuration Manager has updated configuration on CN successfully and CN needs ResetConf so that the configuration gets activated
    kNmtNodeCommandConfErr          = 0x07,     ///< application / Configuration Manager failed on updating configuration on CN
    kNmtNodeCommandStart            = 0x08,     ///< if EPL_NMTST_NO_STARTNODE set it must be issued after kNmtNodeEventReadyToStart
} tNmtNodeCommand;

/**
* \brief NMT boot events
*
* This enumeration lists all valid NMT boot events.
*/
typedef enum
{
    kNmtBootEventBootStep1Finish    = 0x00,     ///< PreOp2 is possible
    kNmtBootEventBootStep2Finish    = 0x01,     ///< ReadyToOp is possible for MN
    kNmtBootEventEnableReadyToOp    = 0x02,     ///< ReadyToOP is possible for CN
    kNmtBootEventCheckComFinish     = 0x03,     ///< Operational is possible
    kNmtBootEventOperational        = 0x04,     ///< all mandatory CNs are Operational
    kNmtBootEventError              = 0x05,     ///< boot process halted because of an error
} tNmtBootEvent;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif  // #ifndef _INC_nmt_H_


