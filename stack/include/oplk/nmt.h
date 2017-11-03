/**
********************************************************************************
\file   oplk/nmt.h

\brief  Global include file for NMT modules

This file is the global include file for all NMT modules
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
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
#ifndef _INC_oplk_nmt_H_
#define _INC_oplk_nmt_H_

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
#define NMT_GS_COMMUNICATING        0x000C  // super state
#define NMT_CS_PLKMODE              0x000D  // super state
#define NMT_MS_PLKMODE              0x000D  // super state

#define NMT_SUPERSTATE_MASK         0x000F  // mask to select state

#define NMT_TYPE_UNDEFINED          0x0000  // type of NMT state is still undefined
#define NMT_TYPE_CS                 0x0100  // CS type of NMT state
#define NMT_TYPE_MS                 0x0200  // MS type of NMT state
#define NMT_TYPE_RMS                0x0300  // RMS type of NMT state
#define NMT_TYPE_MASK               0x0300  // mask to select type of NMT state (i.e. CS or MS)

#define NMT_STATE_GS_INITIALISING           0x0019
#define NMT_STATE_GS_RESET_APPLICATION      0x0029
#define NMT_STATE_GS_RESET_COMMUNICATION    0x0039
#define NMT_STATE_GS_RESET_CONFIGURATION    0x0079

#define NMT_STATE_XX_NOT_ACTIVE             0x001C
#define NMT_STATE_XX_PRE_OPERATIONAL_1      0x001D
#define NMT_STATE_XX_PRE_OPERATIONAL_2      0x005D
#define NMT_STATE_XX_READY_TO_OPERATE       0x006D
#define NMT_STATE_XX_OPERATIONAL            0x00FD
#define NMT_STATE_XX_STOPPED                0x004D
#define NMT_STATE_XX_BASIC_ETHERNET         0x001E
#define NMT_STATE_XX_MASK                   0x00FF

#define NMT_IF_CN_OR_RMN(nmtState)  ((nmtState) & NMT_TYPE_CS)
#define NMT_IF_CN(nmtState)         (((nmtState) & NMT_TYPE_MASK) == NMT_TYPE_CS)
#define NMT_IF_ACTIVE_CN(nmtState)  (((nmtState) & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) \
                                     == (NMT_TYPE_CS | NMT_CS_PLKMODE))
#define NMT_IF_MN(nmtState)         (((nmtState) & NMT_TYPE_MASK) == NMT_TYPE_MS)
#define NMT_IF_MN_OR_RMN(nmtState)  ((nmtState) & NMT_TYPE_MS)
#define NMT_IF_ACTIVE_MN(nmtState)  (((nmtState) & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) \
                                     == (NMT_TYPE_MS | NMT_MS_PLKMODE))
#define NMT_IF_ACTIVE(nmtState)     (((nmtState) & NMT_SUPERSTATE_MASK) \
                                     == NMT_CS_PLKMODE)

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
    kNmtGsOff                       = 0x0000,   ///< NMT_GS_OFF
    kNmtGsInitialising              = 0x0019,   ///< NMT_GS_INITIALIZING
    kNmtGsResetApplication          = 0x0029,   ///< NMT_GS_RESET_APPLICATION
    kNmtGsResetCommunication        = 0x0039,   ///< NMT_GS_RESET_COMMUNICATION
    kNmtGsResetConfiguration        = 0x0079,   ///< NMT_GS_RESET_CONFIGURATION
    kNmtCsNotActive                 = 0x011C,   ///< NMT_CS_NOT_ACTIVE
    kNmtCsPreOperational1           = 0x011D,   ///< NMT_CS_PRE_OPERATIONAL_1
    kNmtCsStopped                   = 0x014D,   ///< NMT_CS_STOPPED
    kNmtCsPreOperational2           = 0x015D,   ///< NMT_CS_PRE_OPERATIONAL_2
    kNmtCsReadyToOperate            = 0x016D,   ///< NMT_CS_READY_TO_OPERATE
    kNmtCsOperational               = 0x01FD,   ///< NMT_CS_OPERATIONAL
    kNmtCsBasicEthernet             = 0x011E,   ///< NMT_CS_BASIC_ETHERNET
    kNmtMsNotActive                 = 0x021C,   ///< NMT_MS_NOT_ACTIVE
    kNmtMsPreOperational1           = 0x021D,   ///< NMT_MS_PRE_OPERATIONAL_1
    kNmtMsPreOperational2           = 0x025D,   ///< NMT_MS_PRE_OPERATIONAL_2
    kNmtMsReadyToOperate            = 0x026D,   ///< NMT_MS_READY_TO_OPERATE
    kNmtMsOperational               = 0x02FD,   ///< NMT_MS_OPERATIONAL
    kNmtMsBasicEthernet             = 0x021E,   ///< NMT_MS_BASIC_ETHERNET
    kNmtRmsNotActive                = 0x031C,   ///< NMT_RMS_NOT_ACTIVE
    kNmtStateInvalid                = 0xFFFF    ///< Dummy state to detect invalid states (garbage)
} eNmtState;

/**
\brief NMT state data type

Data type for the enumerator \ref eNmtState.
*/
typedef UINT16 tNmtState;

/**
* \brief NMT events
*
* This enumeration lists all valid NMT events.
*/
typedef enum
{
    // Events from DLL
    // Events defined by the POWERLINK specification
    kNmtEventNoEvent                =   0x00,   ///< No event has occurred
    // kNmtEventDllMePres           =   0x01,
    kNmtEventDllMePresTimeout       =   0x02,   ///< A PRes timeout event has occurred on the MN
    // kNmtEventDllMeAsnd           =   0x03,
    kNmtEventDllMeAsndTimeout       =   0x03,   ///< An ASnd timeout event has occurred on the MN
    // kNmtEventDllMeSoaSent        =   0x04,
    kNmtEventDllMeSocTrig           =   0x05,   ///< An SoC trigger event has occurred on the MN
    kNmtEventDllMeSoaTrig           =   0x06,   ///< An SoA trigger event has occurred on the MN
    kNmtEventDllCeSoc               =   0x07,   ///< An SoC event has occurred on the CN
    kNmtEventDllCePreq              =   0x08,   ///< A PReq event has occurred on the CN
    kNmtEventDllCePres              =   0x09,   ///< A PRes event has occurred on the CN
    kNmtEventDllCeSoa               =   0x0A,   ///< An SoA event has occurred on the CN
    kNmtEventDllCeAInv              =   0x0B,   ///< An AInv event has occurred on the CN
    kNmtEventDllCeAsnd              =   0x0C,   ///< An ASnd event has occurred on the CN
    kNmtEventDllCeFrameTimeout      =   0x0D,   ///< A frame timeout has occurred on the CN
    kNmtEventDllReAmni              =   0x0E,   ///< NMT_RMT5, NMT_RMT7
    kNmtEventDllReSwitchOverTimeout =   0x0F,   ///< NMT_RMT4, NMT_RMT6

    // Events triggered by NMT-Commands
    kNmtEventSwReset                =   0x10,   ///< A SwReset event has occurred (NMT_GT1, NMT_GT2, NMT_GT8)
    kNmtEventResetNode              =   0x11,   ///< A ResetNode event has occurred
    kNmtEventResetCom               =   0x12,   ///< A ResetCommunication event has occurred
    kNmtEventResetConfig            =   0x13,   ///< A ResetConfiguration event has occurred
    kNmtEventEnterPreOperational2   =   0x14,   ///< An EnterPreOperational2 event has occurred
    kNmtEventEnableReadyToOperate   =   0x15,   ///< An EnableReadyToOperate event has occurred
    kNmtEventStartNode              =   0x16,   ///< A StartNode event has occurred (NMT_CT7)
    kNmtEventStopNode               =   0x17,   ///< A StopNode event has occurred

    // Events triggered by NMT-Requests or application
    kNmtEventGoToStandby            =   0x18,   ///< GoToStandby request received (NMT_RMT5, NMT_RMT7)
    kNmtEventGoToStandbyDelayed     =   0x19,   ///< GoToStandby request (with DF flag) received (NMT_RMT5, NMT_RMT7)

    // Events triggered by higher layer
    kNmtEventEnterResetApp          =   0x20,   ///< An EnterResetApplication event has occurred
    kNmtEventEnterResetCom          =   0x21,   ///< An EnterResetCommunication event has occurred
    kNmtEventInternComError         =   0x22,   ///< An internal communication error has occurred (NMT_GT6 -> enter ResetCommunication)
    kNmtEventEnterResetConfig       =   0x23,   ///< An EnterResetConfiguration event has occurred
    kNmtEventEnterCsNotActive       =   0x24,   ///< An EnterCsNotActive event has occurred
    kNmtEventEnterMsNotActive       =   0x25,   ///< An EnterMsNotActive event has occurred
    kNmtEventTimerBasicEthernet     =   0x26,   ///< NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    kNmtEventTimerMsPreOp1          =   0x27,   ///< enter PreOp1 on MN (NotActive -> MsPreOp1)
    kNmtEventNmtCycleError          =   0x28,   ///< NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    kNmtEventTimerMsPreOp2          =   0x29,   ///< enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if kNmtEventAllMandatoryCNIdent)
    kNmtEventAllMandatoryCNIdent    =   0x2A,   ///< enter PreOp2 on MN if kNmtEventTimerMsPreOp2
    kNmtEventEnterReadyToOperate    =   0x2B,   ///< application ready for the state ReadyToOp
    kNmtEventEnterMsOperational     =   0x2C,   ///< enter Operational on MN
    kNmtEventSwitchOff              =   0x2D,   ///< enter state Off
    kNmtEventCriticalError          =   0x2E,   ///< enter state Off because of critical error
    kNmtEventEnterRmsNotActive      =   0x2F,   ///< enter RMS NotActive state, because Bit 14 in object 0x1F80
} eNmtEvent;

/**
\brief NMT event data type

Data type for the enumerator \ref eNmtEvent.
*/
typedef UINT8 tNmtEvent;

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
    UINT8                   aPadding[3];        ///< Padding for 32 bit-alignment
} tEventNmtStateChange;

/**
* \brief Heartbeat event
*
* This structure defines the heartbeat event.
*/
typedef struct
{
    UINT                    nodeId;             ///< Node ID
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
    kNmtNodeEventFound              = 0x00,     ///< A configured node has been found.
    kNmtNodeEventUpdateSw           = 0x01,     ///< The application shall update the software on the CN.
    kNmtNodeEventCheckConf          = 0x02,     ///< The application / Configuration Manager shall check and update the configuration on the CN.
    kNmtNodeEventUpdateConf         = 0x03,     ///< The application / Configuration Manager shall update the configuration on the CN (check was done by NmtMn module).
    kNmtNodeEventVerifyConf         = 0x04,     ///< The application / Configuration Manager shall verify the configuration of the CN.
    kNmtNodeEventReadyToStart       = 0x05,     ///< Issued if NMT_STARTUP_NO_STARTNODE set, application must call oplk_execNmtCommand(kErrorNmtCmdStartNode) manually.
    kNmtNodeEventNmtState           = 0x06,     ///< Issued if the NMT state of the CN has changed.
    kNmtNodeEventError              = 0x07,     ///< NMT error of the CN.
    kNmtNodeEventAmniReceived       = 0x08,     ///< An AMNI frame was received.
    kNmtNodeEventConfDone           = 0x09,     ///< Configuration done.
} eNmtNodeEvent;

/**
\brief Node event data type

Data type for the enumerator \ref eNmtNodeEvent.
*/
typedef UINT32 tNmtNodeEvent;

/**
* \brief NMT node commands
*
* This enumeration lists all valid NMT node commands.
*/
typedef enum
{
    kNmtNodeCommandBoot             = 0x01,     ///< If NMT_NODEASSIGN_START_CN is not set, this command must be issued after kNmtNodeEventFound.
    kNmtNodeCommandSwOk             = 0x02,     ///< The application has verified the software on the CN to be fine.
    kNmtNodeCommandSwUpdated        = 0x03,     ///< The application has successfully updated the software on the CN.
    kNmtNodeCommandConfOk           = 0x04,     ///< The application / Configuration Manager has successfully updated the configuration on the CN.
    kNmtNodeCommandConfRestored     = 0x05,     ///< The application / Configuration Manager has restored the original CN configuration, and the CN needs a ResetNode to complete the restore process. Afterwards, the new configuration can be downloaded.
    kNmtNodeCommandConfReset        = 0x06,     ///< The application / Configuration Manager has successfully updated the configuration on the CN, and the CN needs ResetConf so that the configuration gets activated.
    kNmtNodeCommandConfErr          = 0x07,     ///< The application / Configuration Manager failed on updating configuration on the CN.
    kNmtNodeCommandStart            = 0x08,     ///< If NMT_STARTUP_NO_STARTNODE is set, this command must be issued after kNmtNodeEventReadyToStart.
    kNmtNodeCommandSwErr            = 0x09,     ///< The application failed on updating the software on the CN.
} eNmtNodeCommand;

/**
\brief Node command data type

Data type for the enumerator \ref eNmtNodeCommand.
*/
typedef UINT32 tNmtNodeCommand;

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
} eNmtBootEvent;

/**
\brief NMT boot event data type

Data type for the enumerator \ref eNmtBootEvent.
*/
typedef UINT32 tNmtBootEvent;

#endif  /* _INC_oplk_nmt_H_ */
