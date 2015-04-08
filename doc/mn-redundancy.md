Managing Node Redundancy {#page_mnredundancy}
========================

[TOC]

# RMN Introduction {#sect_rmn_introduction}

The Managing Node Redundancy extension according to EPSG 302-A Section 3
provides the ability to have more than one Managing Node in the POWERLINK network
to increase the availability. Those nodes are called Redundant Managing Node
(RMN). Only one of them at a time is running as master. It is called
Active Managing Node (AMN) then. The other RMNs run like ordinary slaves and are
called Standby Managing Node (SMN). They can switch over to AMN function as soon
as the current AMN fails.

# Application requirements {#sect_rmn_app_req}

There application of the node which shall support the Managing Node Redundancy
feature and the configuration of the POWERLINK network have to fulfil several
requirements for proper function.

1. Macro CONFIG_INCLUDE_NMT_RMN is defined in oplkcfg.h of the POWERLINK library.
2. There is no ordinary MN with node-ID 240 without RMN support in the network.
3. Node-ID of RMN is in range 241 to 250 or RMN priority in object 0x1F89 sub-indexes 0xA-0xC is configured properly
4. Configuration requirements (RMN object dictionary). Some values must be
   written to the local object dictionary manually during ResetConfiguration by
   application, because current version of openCONFIGURATOR does not provide
   means for configuration.
   + Object 0x1F80/0 NMT_StartUp_U32: Bit 14 is 1 (Enable RMN)
   + Object 0x1F81 NMT_NodeAssignment_AU32 must include all the RMNs,
     i.e. Bits 0 and 1 are set for RMN node-IDs.
   + Object 0x1F92 NMT_MNCNPResTimeout_AU32 shall contain appropriate values for
     RMN node-IDs.
   + Object 0x1F89/2 MNWaitNotAct_U32 shall be set different on each RMN to get
     distinct RMN priorities in NMT state Pre-Operational1.
   + PDO configuration is up to the application's requirements.
5. The RMN runs as CN (SMN) and MN (AMN). So the target platform for industrial
   applications is required to fulfil both CN and MN performance and real-time
   requirements. For CN it is a very low and stable PRes latency (i.e. fast
   Ethernet controller interrupt response).
   For MN it is a precise high-resolution timer implementation.
   Especially for the SMN the determinism and precision of the high-res timer is
   important. There the switch-over timer determines which SMN in the network
   wins the election process and becomes AMN, when the current AMN fails. It may
   lead to unexpected situations if two SMN decide on the very same time to
   become AMN, because of the indeterminism of the high-res timer.


# Supported features {#sect_rmn_supported}

The following features are currently supported by the Managing Node Redundancy
extension.

- If RMN is enabled, the node-ID 240 is only used as source node-ID for SoC, PReq
  and SoA. All the other POWERLINK frames use the actual node-ID as source
  node-ID.
- The SoC relative time is synchronized between RMNs.
- NMT request NMTGoToStandby with optional delay flag.
- Node event [kNmtNodeEventAmniRecvd](\ref kNmtNodeEventAmniRecvd) for indication
  of AMNI reception to application. Element tOplkApiEventNode#nodeId contains the
  node-ID of the new AMN.
- The [Configuration Manager](\ref module_cfmu) supports the transfer of the CN
  CDCs in object 0x1F22 to the other RMNs in the network
  (i.e. distribution of network configuration data).
- Calculate object 0x1F89/A MNSwitchOverPriority_U32 per default with formula
  (node-ID - 240). If the result is negative, the default value is 10 (very low
  priority).


# Not supported features {#sect_rmn_notsupported}

The following features are currently not supported in conjunction with Managing
Node Redundancy extension.

- Targets with openMAC (macro CONFIG_EDRV_AUTO_RESPONSE is TRUE)
- Ethernet driver for Intel i210 (untested)
- POWERLINK extension PollResponse Chaining (the specifications EPSG 302-C and 302-A are incompatible)
- NMT command NMTFlushArpEntry is not transmitted after switch-over as
  recommanded by the specification. Currently, this command isn't implemented at
  all.
- Possibility of application to reject NMT request GoToStandby. Currently, there
  is no API to ask application about NMT requests at all.



