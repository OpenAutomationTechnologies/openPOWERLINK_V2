Status-/Control-Registers Documentation {#hostif_sc}
========================================

[TOC]

# Overview {#sec-overview}

The status-/control registers are seen from Pcp and Host side.
However, some of the registers can only be read from Pcp side and written by the Host - or vice versa.

Memory range    | Buffer
--------------- | ----------------------
0x0000 - 0x00FC | INFORMATION
0x0100 - 0x01FC | RESERVED
0x0200 - 0x02FC | CONTROL
0x0300 - 0x03FC | SYNCHRONIZATION
0x0400 - 0x04FC | DYNAMIC_BUFFER
0x0500 - 0x05FC | RESERVED
0x0600 - 0x06FC | RESERVED
0x0700 - 0x07FC | RESERVED


# Host Side {#sec-host_side}

Addr    |   31..24  |   23..16  |   15..8   |   7..0
------- | --------- | --------- | --------- | ---------
0x0000  | [MAGIC[3]](#SC_MAGIC) (RO) | [MAGIC[2]](#SC_MAGIC) (RO) | [MAGIC[1]](#SC_MAGIC) (RO) | [MAGIC[0]](#SC_MAGIC) (RO)
0x0004  | [FW_VERSION[3]](#SC_FW_VERSION) (RO) | [FW_VERSION[2]](#SC_FW_VERSION) (RO) | [FW_VERSION[1]](#SC_FW_VERSION) (RO) | [FW_VERSION[0]](#SC_FW_VERSION) (RO)
0x0008  | [BOOT_BASE[3]](#SC_BOOT_BASE) (RO) | [BOOT_BASE[2]](#SC_BOOT_BASE) (RO) | [BOOT_BASE[1]](#SC_BOOT_BASE) (RO) | [BOOT_BASE[0]](#SC_BOOT_BASE) (RO)
0x000C  | [INIT_BASE[3]](#SC_INIT_BASE) (RO) | [INIT_BASE[2]](#SC_INIT_BASE) (RO) | [INIT_BASE[1]](#SC_INIT_BASE) (RO) | [INIT_BASE[0]](#SC_INIT_BASE) (RO)
0x0010  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x01FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0200  | RESERVED  | RESERVED  | [BRIDGE_ENABLE[1]](#SC_BRIDGE_ENABLE) (RO) | [BRIDGE_ENABLE[0]](#SC_BRIDGE_ENABLE) (RO)
0x0204  | [STATE[1]](#SC_STATE) (RO) | [STATE[0]](#SC_STATE) (RO) | [COMMAND[1]](#SC_COMMAND) (RW) | [COMMAND[0]](#SC_COMMAND) (RW)
0x0208  | RESERVED  | RESERVED  | [RETURN[1]](#SC_RETURN) (RO) | [RETURN[0]](#SC_RETURN) (RO)
0x020C  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0210  | RESERVED  | RESERVED  | [LED_CONTROL[1]](#SC_LED_CONTROL) (RW) | [LED_CONTROL[0]](#SC_LED_CONTROL) (RW)
0x0214  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x02FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0300  | [IRQ_PENDING[1]](#SC_IRQ_PENDING) (RO) | [IRQ_PENDING[0]](#SC_IRQ_PENDING) (RO) | [IRQ_ENABLE_HOST[1]](#SC_IRQ_ENABLE_HOST) (RW) | [IRQ_ENABLE_HOST[0]](#SC_IRQ_ENABLE_HOST) (RW)
0x0304  | [IRQ_ACK[1]](#SC_IRQ_ACK) (SC) | [IRQ_ACK[0]](#SC_IRQ_ACK) (SC) | [IRQ_MASTER_ENABLE[1]](#SC_IRQ_MASTER_ENABLE) (RW) | [IRQ_MASTER_ENABLE[0]](#SC_IRQ_MASTER_ENABLE) (RW)
0x0308  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x030C  | RESERVED  | RESERVED  | [SYNC_CONFIG[1]](#SC_SYNC_CONFIG) (RW) | [SYNC_CONFIG[0]](#SC_SYNC_CONFIG) (RW)
0x0310  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x03FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0400  | [DYN_BUF_0_MMU_ADDR[3]](#SC_DYN_BUF0_MMU_ADDR) (RW) | [DYN_BUF_0_MMU_ADDR[2]](#SC_DYN_BUF0_MMU_ADDR) (RW) | [DYN_BUF_0_MMU_ADDR[1]](#SC_DYN_BUF0_MMU_ADDR) (RW) | [DYN_BUF_0_MMU_ADDR[0]](#SC_DYN_BUF0_MMU_ADDR) (RW)
0x0404  | [DYN_BUF_1_MMU_ADDR[3]](#SC_DYN_BUF1_MMU_ADDR) (RW) | [DYN_BUF_1_MMU_ADDR[2]](#SC_DYN_BUF1_MMU_ADDR) (RW) | [DYN_BUF_1_MMU_ADDR[1]](#SC_DYN_BUF1_MMU_ADDR) (RW) | [DYN_BUF_1_MMU_ADDR[0]](#SC_DYN_BUF1_MMU_ADDR) (RW)
0x0408  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x04FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0500  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x05FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0600  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x06FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0700  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x07FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED

# Pcp Side  {#sec-pcp_side}

Addr    |   31..24  |   23..16  |   15..8   |   7..0
------- | --------- | --------- | --------- | ---------
0x0000  | [MAGIC[3]](#SC_MAGIC) (RO) | [MAGIC[2]](#SC_MAGIC) (RO) | [MAGIC[1]](#SC_MAGIC) (RO) | [MAGIC[0]](#SC_MAGIC) (RO)
0x0004  | [FW_VERSION[3]](#SC_FW_VERSION) (RO) | [FW_VERSION[2]](#SC_FW_VERSION) (RO) | [FW_VERSION[1]](#SC_FW_VERSION) (RO) | [FW_VERSION[0]](#SC_FW_VERSION) (RO)
0x0008  | [BOOT_BASE[3]](#SC_BOOT_BASE) (RW) | [BOOT_BASE[2]](#SC_BOOT_BASE) (RW) | [BOOT_BASE[1]](#SC_BOOT_BASE) (RW) | [BOOT_BASE[0]](#SC_BOOT_BASE) (RW)
0x000C  | [INIT_BASE[3]](#SC_INIT_BASE) (RW) | [INIT_BASE[2]](#SC_INIT_BASE) (RW) | [INIT_BASE[1]](#SC_INIT_BASE) (RW) | [INIT_BASE[0]](#SC_INIT_BASE) (RW)
0x0010  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x01FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0200  | RESERVED  | RESERVED  | [BRIDGE_ENABLE[1]](#SC_BRIDGE_ENABLE) (RW) | [BRIDGE_ENABLE[0]](#SC_BRIDGE_ENABLE) (RW)
0x0204  | [STATE[1]](#SC_STATE) (RW) | [STATE[0]](#SC_STATE) (RW) | [COMMAND[1]](#SC_COMMAND) (RW) | [COMMAND[0]](#SC_COMMAND) (RW)
0x0208  | RESERVED  | RESERVED  | [RETURN[1]](#SC_RETURN) (RW) | [RETURN[0]](#SC_RETURN) (RW)
0x020C  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0210  | RESERVED  | RESERVED  | [LED_CONTROL[1]](#SC_LED_CONTROL) (RO) | [LED_CONTROL[0]](#SC_LED_CONTROL) (RO)
0x0214  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x02FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0300  | [IRQ_PENDING[1]](#SC_IRQ_PENDING) (RO) | [IRQ_PENDING[0]](#SC_IRQ_PENDING) (RO) | [IRQ_ENABLE_PCP[1]](#SC_IRQ_ENABLE_PCP) (RW) | [IRQ_ENABLE_PCP[0]](#SC_IRQ_ENABLE_PCP) (RW)
0x0304  | [IRQ_SET[1]](#SC_IRQ_SET) (SC) | [IRQ_SET[0]](#SC_IRQ_SET) (SC) | [IRQ_MASTER_ENABLE[1]](#SC_IRQ_MASTER_ENABLE) (RO) | [IRQ_MASTER_ENABLE[0]](#SC_IRQ_MASTER_ENABLE) (RO)
0x0308  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x030C  | RESERVED  | RESERVED  | [SYNC_CONFIG[1]](#SC_SYNC_CONFIG) (RO) | [SYNC_CONFIG[0]](#SC_SYNC_CONFIG) (RO)
0x0310  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x03FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0400  | [ERROR-COUNTER_MMU_ADDR[3]](#SC_ERROR-COUNTER_MMU_ADDR) (RW) | [ERROR-COUNTER_MMU_ADDR[2]](#SC_ERROR-COUNTER_MMU_ADDR) (RW) | [ERROR-COUNTER_MMU_ADDR[1]](#SC_ERROR-COUNTER_MMU_ADDR) (RW) | [ERROR-COUNTER_MMU_ADDR[0]](#SC_ERROR-COUNTER_MMU_ADDR) (RW)
0x0404  | [TX_NMT_Q_MMU_ADDR[3]](#SC_TX_NMT_Q_MMU_ADDR) (RW) | [TX_NMT_Q_MMU_ADDR[2]](#SC_TX_NMT_Q_MMU_ADDR) (RW) | [TX_NMT_Q_MMU_ADDR[1]](#SC_TX_NMT_Q_MMU_ADDR) (RW) | [TX_NMT_Q_MMU_ADDR[0]](#SC_TX_NMT_Q_MMU_ADDR) (RW)
0x0408  | [TX_GEN_Q_MMU_ADDR[3]](#SC_TX_GEN_Q_MMU_ADDR) (RW) | [TX_GEN_Q_MMU_ADDR[2]](#SC_TX_GEN_Q_MMU_ADDR) (RW) | [TX_GEN_Q_MMU_ADDR[1]](#SC_TX_GEN_Q_MMU_ADDR) (RW) | [TX_GEN_Q_MMU_ADDR[0]](#SC_TX_GEN_Q_MMU_ADDR) (RW)
0x040C  | [TX_SYNC_Q_MMU_ADDR[3]](#SC_TX_SYNC_Q_MMU_ADDR) (RW) | [TX_SYNC_Q_MMU_ADDR[2]](#SC_TX_SYNC_Q_MMU_ADDR) (RW) | [TX_SYNC_Q_MMU_ADDR[1]](#SC_TX_SYNC_Q_MMU_ADDR) (RW) | [TX_SYNC_Q_MMU_ADDR[0]](#SC_TX_SYNC_Q_MMU_ADDR) (RW)
0x0410  | [TX_VETH_Q_MMU_ADDR[3]](#SC_TX_VETH_Q_MMU_ADDR) (RW) | [TX_VETH_Q_MMU_ADDR[2]](#SC_TX_VETH_Q_MMU_ADDR) (RW) | [TX_VETH_Q_MMU_ADDR[1]](#SC_TX_VETH_Q_MMU_ADDR) (RW) | [TX_VETH_Q_MMU_ADDR[0]](#SC_TX_VETH_Q_MMU_ADDR) (RW)
0x0414  | [RX_VETH_Q_MMU_ADDR[3]](#SC_RX_VETH_Q_MMU_ADDR) (RW) | [RX_VETH_Q_MMU_ADDR[2]](#SC_RX_VETH_Q_MMU_ADDR) (RW) | [RX_VETH_Q_MMU_ADDR[1]](#SC_RX_VETH_Q_MMU_ADDR) (RW) | [RX_VETH_Q_MMU_ADDR[0]](#SC_RX_VETH_Q_MMU_ADDR) (RW)
0x0418  | [KERNEL-TO-USER_Q_MMU_ADDR[3]](#SC_KERNEL-TO-USER_Q_MMU_ADDR) (RW) | [KERNEL-TO-USER_Q_MMU_ADDR[2]](#SC_KERNEL-TO-USER_Q_MMU_ADDR) (RW) | [KERNEL-TO-USER_Q_MMU_ADDR[1]](#SC_KERNEL-TO-USER_Q_MMU_ADDR) (RW) | [KERNEL-TO-USER_Q_MMU_ADDR[0]](#SC_KERNEL-TO-USER_Q_MMU_ADDR) (RW)
0x041C  | [USER-TO-KERNEL_Q_MMU_ADDR[3]](#SC_USER-TO-KERNEL_Q_MMU_ADDR) (RW) | [USER-TO-KERNEL_Q_MMU_ADDR[2]](#SC_USER-TO-KERNEL_Q_MMU_ADDR) (RW) | [USER-TO-KERNEL_Q_MMU_ADDR[1]](#SC_USER-TO-KERNEL_Q_MMU_ADDR) (RW) | [USER-TO-KERNEL_Q_MMU_ADDR[0]](#SC_USER-TO-KERNEL_Q_MMU_ADDR) (RW)
0x0420  | [TPDO_MMU_ADDR[3]](#SC_TPDO_MMU_ADDR) (RW) | [TPDO_MMU_ADDR[2]](#SC_TPDO_MMU_ADDR) (RW) | [TPDO_MMU_ADDR[1]](#SC_TPDO_MMU_ADDR) (RW) | [TPDO_MMU_ADDR[0]](#SC_TPDO_MMU_ADDR) (RW)
0x0424  | [RPDO_MMU_ADDR[3]](#SC_RPDO_MMU_ADDR) (RW) | [RPDO_MMU_ADDR[2]](#SC_RPDO_MMU_ADDR) (RW) | [RPDO_MMU_ADDR[1]](#SC_RPDO_MMU_ADDR) (RW) | [RPDO_MMU_ADDR[0]](#SC_RPDO_MMU_ADDR) (RW)
0x0428  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x04FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0500  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x05FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0600  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x06FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x0700  | RESERVED  | RESERVED  | RESERVED  | RESERVED
...     | RESERVED  | RESERVED  | RESERVED  | RESERVED
0x07FC  | RESERVED  | RESERVED  | RESERVED  | RESERVED

# Register Specification {#sec_reg-spec}

## MAGIC (0x0000, RO) {#SC_MAGIC}

|  31.. 0    |
| ---------- |
| 0x504C4B00 |

The magic word gives a unique pattern that has to be read by the host to identify a valid host interface,
respectively a valid status/control register.

## FW_VERSION (0x0004, RO) {#SC_FW_VERSION}

31..24  | 23..16    | 15.. 8    |  7.. 0
------- | --------- | --------- | ---------
MAJOR   | MINOR     | REVISION  | CNT

The firmware version field enables to find HW/SW mismatches. \n
The first three fields MAJOR, MINOR and REVISION have to be read in order to identify a version mismatch of software and IP-Core.
The pattern CNT enables to find a mismatch of the generated IP-Core and the FPGA system header file (system.h or xparameters.h) available at the host.
The CNT value is incremented by the system-level-tool (Qsys or XPS) when the user regenerates the PCP system.

## BOOT_BASE (0x0008, Host = RO / Pcp = RW) {#SC_BOOT_BASE}

| 31.. 0    |
| --------- |
| BOOT_BASE |

This register is set by the PCP after reset to the boot loader memory location. The host has to write this
value to a dynamic buffer to access the space in the PCP memory map.

**Note that a value of 0x00000000 (NULL) is invalid and must not be accessed through the dynamic buffers!**

## INIT_BASE (0x000C, Host = RO / Pcp = RW) {#SC_INIT_BASE}

| 31.. 0    |
| --------- |
| INIT_BASE |

This register is set by the PCP after successfully booting.
The host has to write this value to a dynamic buffer to access the space in the PCP memory map.

**Note that a value of 0x00000000 (NULL) is invalid and must not be accessed through the dynamic buffers!**

## BRIDGE_ENABLE (0x0200, Host = RO / Pcp = RW) {#SC_BRIDGE_ENABLE}

| 15.. 0    |
| --------- |
| BRIDGE_ENABLE |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..1 | /     | 0         | RESERVED
0   | BEN   | 0         | Enables the bridge logic for the dynamic buffers.

**Note that if `BEN` is disabled the host cannot access the shared memory.
Read data is invalid and writes are ignored.**

## COMMAND (0x0204, Host = RW / Pcp = RW) {#SC_COMMAND}

| 15.. 0    |
| --------- |
| COMMAND |

## STATE (0x0206, Host = RO / Pcp = RW) {#SC_STATE}

| 15.. 0    |
| --------- |
| STATE |

## RETURN (0x0208, Host = RO / Pcp = RW) {#SC_RETURN}

| 15.. 0    |
| --------- |
| RETURN |

## LED_CONTROL (0x0210, Host = RW / Pcp = RO) {#SC_LED_CONTROL}

| 15.. 0    |
| --------- |
| LED_CONTROL |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..2 | /     | 0         | RESERVED
1   | PLER  | 0         | Control POWERLINK Error LED pin (hostInterface.oPlkLedError)
0   | PLST  | 0         | Control POWERLINK Status LED pin (hostInterface.oPlkLedStatus)

## IRQ_PENDING (0x0302, RO) {#SC_IRQ_PENDING}

| 15.. 0    |
| --------- |
| IRQ_PENDING |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..4 | /     | 0         | RESERVED
3   | PNAR  | 0         | Pending request at interrupt source Asynchronous RX (high active)
2   | PNAT  | 0         | Pending request at interrupt source Asynchronous TX (high active)
1   | PNEV  | 0         | Pending request at interrupt source Event (high active)
0   | PNSY  | 0         | Pending request at interrupt source SYNC (high active)

## IRQ_MASTER_ENABLE (0x0304, Host = RW / Pcp = RO) {#SC_IRQ_MASTER_ENABLE}

| 15.. 0    |
| --------- |
| IRQ_MASTER_ENABLE |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..1 | /     | 0         | RESERVED
0   | MEN   | 0         | Master enable to forward any enabled interrupt request to output Irq (hostInterface.oIrq)

## IRQ_ACK (0x0306, Host = RW/SC) {#SC_IRQ_ACK}

| 15.. 0    |
| --------- |
| IRQ_ACK |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..4 | /     | 0         | RESERVED
3   | ACAR  | 0         | Acknowledge request at interrupt source Asynchronous RX (high active)
2   | ACAT  | 0         | Acknowledge request at interrupt source Asynchronous TX (high active)
1   | ACEV  | 0         | Acknowledge request at interrupt source Event (high active)
0   | ACSY  | 0         | Acknowledge request at interrupt source SYNC (high active)

## IRQ_SET (0x0306, Pcp = RW/SC) {#SC_IRQ_SET}

| 15.. 0    |
| --------- |
| IRQ_SET |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..4 | /     | 0         | RESERVED
3   | STAR  | 0         | Set interrupt source Asynchronous RX (high active)
2   | STAT  | 0         | Set interrupt source Asynchronous TX (high active)
1   | STEV  | 0         | Set interrupt source Event (high active)
0   | -     | 0         | IGNORED

## SYNC_CONFIG (0x0318, Host = RW / Pcp = RO) {#SC_SYNC_CONFIG}

| 15.. 0    |
| --------- |
| SYNC_CONFIG |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..3 | /     | 0         | RESERVED
2..1 | XSED  | 0         | SYNC pulse configuration
0   | ENXS  | 0         | Enable external synchronization source (high active)

Value   | Name  | Description
------- | ----- | -----------
3       | XSED_RSI | SYNC pulse is generated with the rising edge of EX_SYNC input.
2       | XSED_FAL | SYNC pulse is generated with the falling edge of EX_SYNC input.
1       | XSED_ANY | SYNC pulse is generated with any edge of EX_SYNC input.

## Host-only {#SC_HOST_ONLY}

The following registers can be accessed from host side only.

### IRQ_ENABLE_HOST (0x0300, Host = RW) {#SC_IRQ_ENABLE_HOST}

**Note that the interrupt sources have to be enabled on both sides (PCP and Host).
If only one side enables the source, the interrupt source can not trigger the interrupt!**

| 15.. 0    |
| --------- |
| IRQ_ENABLE_HOST |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..4 | /     | 0         | RESERVED
3   | ENAR  | 0         | Enable interrupt source Asynchronous RX (high active)
2   | ENAT  | 0         | Enable interrupt source Asynchronous TX (high active)
1   | ENEV  | 0         | Enable interrupt source Event (high active)
0   | ENSY  | 0         | Enable interrupt source SYNC (high active)

### DYN_BUF0_MMU_ADDR (0x04000, Host = RW) {#SC_DYN_BUF0_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer the host wants to access at the
PCP's memory environment. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### DYN_BUF1_MMU_ADDR (0x04004, Host = RW) {#SC_DYN_BUF1_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer the host wants to access at the
PCP's memory environment. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

## Pcp-only {#SC_PCP_ONLY}

The following registers can be accessed from Pcp side only.

### IRQ_ENABLE_PCP (0x0300, Pcp = RW) {#SC_IRQ_ENABLE_PCP}
**Note that the interrupt sources have to be enabled on both sides (PCP and Host).
If only one side enables the source, the interrupt source can not trigger the interrupt!**

| 15.. 0    |
| --------- |
| IRQ_ENABLE_PCP |

Bit | Name  | Default   | Description
--- | ----- | --------- | -----------
15..4 | /     | 0         | RESERVED
3   | ENAR  | 0         | Enable interrupt source Asynchronous RX (high active)
2   | ENAT  | 0         | Enable interrupt source Asynchronous TX (high active)
1   | ENEV  | 0         | Enable interrupt source Event (high active)
0   | ENSY  | 0         | Enable interrupt source SYNC (high active)

### ERROR-COUNTER_MMU_ADDR (0x04000, Pcp = RW) {#SC_ERROR-COUNTER_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### TX_NMT_Q_MMU_ADDR (0x04004, Pcp = RW) {#SC_TX_NMT_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### TX_GEN_Q_MMU_ADDR (0x04008, Pcp = RW) {#SC_TX_GEN_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### TX_SYNC_Q_MMU_ADDR (0x0400C, Pcp = RW) {#SC_TX_SYNC_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### TX_VETH_Q_MMU_ADDR (0x04010, Pcp = RW) {#SC_TX_VETH_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### RX_VETH_Q_MMU_ADDR (0x04014, Pcp = RW) {#SC_RX_VETH_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### KERNEL-TO-USER_Q_MMU_ADDR (0x04018, Pcp = RW) {#SC_KERNEL-TO-USER_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### USER-TO-KERNEL_Q_MMU_ADDR (0x0401C, Pcp = RW) {#SC_USER-TO-KERNEL_Q_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### TPDO_MMU_ADDR (0x04020, Pcp = RW) {#SC_TPDO_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**

### RPDO_MMU_ADDR (0x04024, Pcp = RW) {#SC_RPDO_MMU_ADDR}

| 31.. 0    |
| --------- |
| MMU_ADDR |

The `MMU_ADDR` field has to be set to the base address of the buffer in the PCP's memory range. \n
**Note that the [Bridge Enable](#SC_BRIDGE_ENABLE) bit has to be activated by the PCP to enable access to the given address.**
