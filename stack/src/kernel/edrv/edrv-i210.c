/**
********************************************************************************
\file   edrv-i210.c

\brief  Implementation of Ethernet driver for Intel I210

This file contains the implementation of the Ethernet driver for
Intel I210 Gigabit Ethernet Controller and compatible devices.

The driver is based on edrv-82573.c and igb_main.c

Interrupt Handling Routines

The I210 uses the MSI-X interrupt mechanism for interrupt handling.
The driver handles interrupts via two MSI-X vectors: one takes care of the
Tx-Rx queue (e.g. descriptor write back, queue errors etc.), the second one
handles timer interrupts and therefore enables the I210 hardware timer as a
source for the high-resolution timer module.


\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2017, B&R Industrial Automation GmbH
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/bufalloc.h>
#include <kernel/edrv.h>
#include <kernel/hrestimer.h>           // using definition of tHresCallback

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
#error "Linux Kernel versions older than 2.6.19 are not supported by this driver!"
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Intel's I210 Ethernet controller specific defines
//------------------------------------------------------------------------------
#define CONFIG_BAR              0
#define DRIVER_NAME             "plk"

#define EDRV_MAX_RX_QUEUES       2
#define EDRV_MAX_TX_QUEUES       2
#define EDRV_MAX_QUEUE_VECTOR    2
#define EDRV_SR_QUEUE            0
#define EDRV_MAX_TIMER_COUNT     2

#define INTERRUPT_STRING_SIZE    25
#define SEC_TO_NSEC              1000000000

//------------------------------------------------------------------------------
// Device Register offsets
//------------------------------------------------------------------------------
#define EDRV_CTRL_REG            0x00000         // Device Control Register
#define EDRV_CTRL_EXTN_REG       0x00018         // Extended Device Control Register
#define EDRV_STATUS_REG          0x00008         // Device Status Register
#define EDRV_MDIC_REG            0x00020         // MDI Control Register
#define EDRV_INTR_READ_REG       0x01500         // Interrupt Cause Read
#define EDRV_INTR_SET_REG        0x01504         // Interrupt Cause Set
#define EDRV_INTR_MASK_SET_READ  0x01508         // Interrupt Mask Set/Read
#define EDRV_INTR_MASK_CLEAR     0x0150C         // Interrupt Mask Clear
#define EDRV_INTR_ACK_AUTO_MASK  0x01510         // Interrupt Acknowledge Auto Mask
#define EDRV_INTR_EIAC           0x0152C         // Extended Interrupt Auto Clear
#define EDRV_INTR_GPIE_REG       0x01514         // General Purpose Interrupt Enable
#define EDRV_IVAR0_REG           0x01700         // Interrupt Vector Allocation Registers (Tx-Rx queues)
#define EDRV_IVAR_MISC           0x01740         // IVAR for other causes
#define EDRV_EXT_INTR_REG        0X01580         // Extended Interrupt Cause Set
#define EDRV_EXT_INTR_MASK_CLEAR 0x01528         // Extended Interrupt Mask Clear
#define EDRV_EXT_INTR_MASK_SET   0x01524         // Extended Interrupt Mask Set/Read
#define EDRV_MDICNFG_REG         0x00E04         // MDC/MDIO Configuration Register
#define EDRV_SWSM_REG            0x05B50         // Software Semaphore
#define EDRV_TIPG_REG            0x00410         // Transmit Inter Packet Gap
#define EDRV_DCA_CTRL_REG        0x05B74         // Direct Cache Access control register
#define EDRV_EECD_REG            0x00010         // EEPROM-Mode Control Register
#define EDRV_SW_FW_SYNC          0x05B5C         // Software-Firmware Synchronization
#define EDRV_SYSTIMR_REG         0x0B6F8         // System time register Residue
#define EDRV_SYSTIML_REG         0x0B600         // System time register Low
#define EDRV_SYSTIMH_REG         0x0B604         // System time register High
#define EDRV_STAT_TPT            0x040D4         // Total Packets Transmitted
#define EDRV_MRQC_REG            0x05818         // Multiple Receive Queues Command
#define EDRV_EEER_REG            0x00E30         // Energy Efficient Ethernet (EEE) Register
// Semaphore defines used for Software/Firmware synchronization
#define EDRV_SWSM_SMBI           0x00000001      // Software
#define EDRV_SWSM_SWESMBI        0x00000002      // Software EEPROM Semaphore Bit
#define EDRV_EECD_AUTO_RD        0x00000200
#define EDRV_SWFW_PHY0_SM        0x02            // Software Phy Semaphore Bit

//------------------------------------------------------------------------------
// Transmit Register offset and defines
//------------------------------------------------------------------------------
#define EDRV_TCTL_REG            0x00400         // Tx Control
#define EDRV_TCTL_EXT_REG        0x00404         // Tx Control Extended
#define EDRV_DTXCTL_REG          0x03590         // DMA Tx Control
#define EDRV_DTX_MAX_PKTSZ_REG   0x0355C         // DMA Tx Max Allowable Packet Size
#define EDRV_TXPBSIZE_REG        0x03404         // Transmit Packet Buffer Size
#define EDRV_TQAVCTRL_REG        0x03570         // Transmit Qav Control

#define EDRV_TDBAL(n)            ((n < 4) ? (0x0E000 + 0x40 * n) :\
                                 (0x0E000 + 0x40 * (n - 4)))            // Tx Descriptor Base Low ( n: 0-3 )
#define EDRV_TDBAH(n)            ((n < 4) ? (0x0E004 + 0x40 * n) :\
                                 (0x0E004 + 0x40 * (n - 4)))            // Tx Descriptor Base High ( n: 0-3 )
#define EDRV_TDLEN(n)            ((n < 4) ? (0x0E008 + 0x40 * n) :\
                                 (0x0E008 + 0x40 * (n - 4)))            // Tx Descriptor Ring Length ( n: 0-3 )
#define EDRV_TDHEAD(n)           ((n < 4) ? (0x0E010 + 0x40 * n) :\
                                 (0x0E010 + 0x40 * (n - 4)))            // Tx Descriptor Head ( n: 0-3 )
#define EDRV_TDTAIL(n)           ((n < 4) ? (0x0E018 + 0x40 * n) :\
                                 (0x0E018 + 0x40 * (n - 4)))            // Tx Descriptor Tail ( n: 0-3 )
#define EDRV_TXDCTL(n)           ((n < 4) ? (0x0E028 + 0x40 * n) :\
                                 (0x0E028 + 0x40 * (n - 4)))            // Tx Descriptor Control Queue ( n: 0-3 )
#define EDRV_TQAVCC(n)           ((n < 2) ? (0x03004 + 0x40 * n) :\
                                 (0x03004 + 0x40 * (n - 2)))            // Tx Qav Credit Control ( n: 0-1 )
#define EDRV_TQAVHC(n)           ((n < 2) ? (0x0300C + 0x40 * n) :\
                                 (0x0300C + 0x40 * (n - 2)))            // Tx Qav High Credits ( n: 0-1 )
#define EDRV_TDWBAL(n)           ((n < 4) ? (0x0E038 + 0x40 * n) :\
                                 (0x0E038 + 0x40 * (n - 1)))            // Tx Descriptor WB Address Low Queue ( n: 0-3 )

#define EDRV_TIPG_DEF            0x00702008      // default according to Intel PCIe GbE Controllers Open Source Software Developer's Manual
#define EDRV_TXPBSIZE_DEF        0x04104208      // 8 Kb TQ0, 8Kb TQ1, 4 Kb TQ2, 4Kb TQ3 and 4 Kb Os2Bmc
#define EDRV_MAX_TX_DESCRIPTOR   256             // Max no of Desc in mem
#define EDRV_MAX_TX_DESC_LEN     (EDRV_MAX_TX_DESCRIPTOR - 1)
// One slot to diff full
#define EDRV_MAX_TTX_DESC_LEN    ((EDRV_MAX_TX_DESCRIPTOR >> 1) -1)
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS      256             // Max no of Buffers
#endif

#define EDRV_MAX_FRAME_SIZE      0x600           // 1536
#define EDRV_TX_BUFFER_SIZE      (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE)
#define EDRV_TX_DESCS_SIZE       (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvAdvTxDesc))

#define EDRV_TDESC_CMD_DEXT              (1 << 29)         // Descriptor type
#define EDRV_TDESC_CMD_RS                (1 << 27)         // Report Status
#define EDRV_TDESC_CMD_IC                (1 << 26)         // Insert Checksum
#define EDRV_TDESC_CMD_IFCS              (1 << 25)         // Insert FCS
#define EDRV_TDESC_CMD_EOP               (1 << 24)         // End of Packet
#define EDRV_TDESC_STATUS_DD             (1 << 0)          // Descriptor done
#define EDRV_TDESC_DTYP_CTXT             (2 << 20)         // Context Descriptor
#define EDRV_TDESC_DTYP_ADV              (3 << 20)         // Advance Tx Descriptor
#define EDRV_TCTL_CT                     0x000000F0        // Collision Threshold
#define EDRV_TCTL_CLEAR_CT               0x00000FF0        // Clear Collision Threshold bits
#define EDRV_TCTL_EN                     0x00000002        // Transmit Enable
#define EDRV_TCTL_PSP                    0x00000008        // Pad short packets
#define EDRV_TCTL_RTLC                   0x01000000        // Re-Transmit on Late collision
#define EDRV_TCTL_EXT_COLD_CLEAR         0x000FFC00        // Clear Collision threshold
#define EDRV_TCTL_EXT_COLD               0x0003F000        // default value as per 802.3 spec
#define EDRV_TXDCTL_PTHRESH              0                 // Prefetch threshold
#define EDRV_TXDCTL_HTHRESH              0                 // Host threshold
#define EDRV_TXDCTL_WTHRESH              0                 // Write-Back threshold
#define EDRV_TXDCTL_QUEUE_EN             0x02000000        // Transmit Queue Enable
#define EDRV_TXDCTL_SWFLSH               (1 << 26)         // Software Flush of queue
#define EDRV_TXDCTL_PRIORITY             (1 << 27)         // Transmit Queue Priority
#define EDRV_TQAVCTRL_TXMODE             (1 << 0)          // Transmit mode configuration
#define EDRV_TQAVCTRL_FETCH_ARB          (1 << 4)          // Data Fetch arbitration
#define EDRV_TQAVCTRL_DTRANSARB          (1 << 8)          // Data Transmit arbitration
#define EDRV_TQAVCTRL_TRANSTIM           (1 << 9)          // Data Launch time valid
#define EDRV_TQAVCTRL_SP_WAIT_SR         (1 << 10)         // SP queues wait for SR queue to guarantee SR launch time
#define EDRV_TQAVCTRL_1588_STAT_EN       (1 << 2)          // Report DMA transmit time in WB descriptors
#define EDRV_TQAVCC_QUEUE_MODE_SR        (1 << 31)         // Queue mode Strict Reservation (Launch time based)
#define EDRV_TQAVCTRL_FETCH_TM_SHIFT     16                // Fetch time shift
#define EDRV_LAUNCH_OSO_SHIFT            5                 // Launch time offset shift (Launch time = Launch time + Off)
#define EDRV_TQAVARBCTRL_TXQPRIO(_q, _n) (((_n) & 0x3) << (_q << 2))
#define EDRV_TXPBSIZE_TX1PB_SHIFT        6                 // Tx packet buffer size shift for 2 queue
#define EDRV_TXPBSIZE_TX2PB_SHIFT        12                // Tx packet buffer size shift for 3 queue
#define EDRV_TXPBSIZE_TX3PB_SHIFT        18                // Tx packet buffer size shift for 4 queue

//------------------------------------------------------------------------------
// Receive Register offset and defines
//------------------------------------------------------------------------------
#define EDRV_RCTL_REG            0x00100                   // Rx Control
#define EDRV_RXPBSIZE_REG        0x02404                   // Rx Packet Buffer Size
#define EDRV_SRRCTL(n)           ((n < 4) ? (0x0C00C + 0x40 * n) :\
                                 (0x0C00C + 0x40 * (n - 4)))            // Split and Replication Receive Control Register Queue
#define EDRV_RDBAL(n)            ((n < 4) ? (0x0C000 + 0x40 * n) :\
                                 (0x0C000 + 0x40 * (n -4)))             // Rx Descriptor Base Low Queue
#define EDRV_RDBAH(n)            ((n < 4) ? (0x0C004 + 0x40 * n) :\
                                 (0x0C004 + 0x40 * (n - 4)))            // Rx Descriptor Base High Queue
#define EDRV_RDLEN(n)            ((n < 4) ? (0x0C008 + 0x40 * n) :\
                                 (0x0C008 + 0x40 * (n - 4)))            // Rx Descriptor Ring Length Queue
#define EDRV_RDHEAD(n)           ((n < 4) ? (0x0C010 + 0x40 * n) :\
                                 (0x0C010 + 0x40 * (n - 4)))            // Rx Descriptor Head Queue
#define EDRV_RDTAIL(n)           ((n < 4) ? (0x0C018 + 0x40 * n) :\
                                 (0x0C018 + 0x40 * (n - 4)))            // Rx Descriptor Tail Queue
#define EDRV_RXDCTL(n)           ((n < 4) ? (0x0C028 + 0x40 * n) :\
                                 (0x0C028 + 0x40 * (n - 4)))            // Receive Descriptor Control Queue
#define EDRV_RAL(n)              (0x05400 + 8 * n)         // Receive Address Low (15:0)
#define EDRV_RAH(n)              (0x05404 + 8 * n)         // Receive Address High (15:0)
#define EDRV_MTA(n)              (0x05200 + 4 * n)         // Multicast Table Array
#define EDRV_MAX_RX_DESCRIPTOR   256                       // Max no of Desc in mem
#define EDRV_MAX_RX_DESC_LEN     (EDRV_MAX_RX_DESCRIPTOR - 1)
// Used for buffer handling
#define EDRV_MAX_RX_BUFFERS      256             // Max no of Buffers
#define EDRV_RX_BUFFER_SIZE      (EDRV_MAX_RX_BUFFERS * EDRV_MAX_FRAME_SIZE)
// Rx buffer size
#define EDRV_RX_DESCS_SIZE       (EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvAdvRxDesc))

#define EDRV_RDESC_STATUS_EOP    (1 << 1)        // End of Packet
#define EDRV_RDESC_STATUS_DD     (1 << 0)        // Descriptor done
#define EDRV_RDESC_ERRORS_RXE    (1 << 31)       // RX data Error
#define EDRV_RCTL_EN             (1 << 1)        // Receive Enable
#define EDRV_RCTL_SBP            (1 << 2)        // Store Bad Packets
#define EDRV_RCTL_UPE            (1 << 3)        // Unicast Enable
#define EDRV_RCTL_MPE            (1 << 4)        // Multicast Enable
#define EDRV_RCTL_LPE            (1 << 5)        // Long packet Reception enable
#define EDRV_RCTL_LBM_MAC        (1 << 6)        // Loop back mode
#define EDRV_RCTL_LBM_CLEAR      (3 << 6)        // Clear loop back
#define EDRV_RCTL_MO_SHIFT       12
#define EDRV_RCTL_MO_36_47       (0 << 12)       // Multicast Offset bit[47:36]
#define EDRV_RCTL_MO_35_46       (1 << 12)       // Multicast Offset bit[46:35]
#define EDRV_RCTL_MO_34_45       (1 << 13)       // Multicast Offset bit[45:34]
#define EDRV_RCTL_MO_32_43       (3 << 12)       // Multicast Offset bit[43:32]
#define EDRV_RCTL_BAM            (1 << 15)       // Accept Broadcast packets
#define EDRV_RCTL_BSIZE_OFFSET   16              // Buffer Size Offset (00:2048, 01:1024, 10:512, 11:256)
#define EDRV_RCTL_VFE            (1 << 16)       // Enable Vlan Filter
#define EDRV_RCTL_PSP            (1 << 21)       // Pad Small packets
#define EDRV_RCTL_SECRC          (1 << 26)       // Strip CRC
#define EDRV_SRRCTL_DESCTYPE_ADV (1 << 25)       // Advance Rx Descriptor one buffer
#define EDRV_SRRCTL_TIMESTAMP    (1 << 30)       // Time stamp received packets
#define EDRV_SRRCTL_DROP_EN      (1 << 31)       // drop packets if no descriptors available
#define EDRV_RXDCTL_PTHRESH      0               // Prefetch Threshold
#define EDRV_RXDCTL_HTHRESH      0               // Host Threshold
#define EDRV_RXDCTL_WTHRESH      1               // Write-back threshold
#define EDRV_RXDCTL_QUEUE_EN     (1 << 25)       // Receive Queue Enable
#define EDRV_RXDCTL_SWFLUSH      (1 << 26)       // Software flush
#define EDRV_RXPBSIZE_CLEAR      0x3F            // Clear Packet buffer size
#define EDRV_RXPBSIZE_DEF        0x8000001E      // default configuration
#define EDRV_RAH_AV              (1 << 31)       // Address Valid

//------------------------------------------------------------------------------
// Time Sync Register offset and defines
//------------------------------------------------------------------------------
#define EDRV_LAUNCH_OSO                  0x03578           // Launch Time Offset Register 0
#define EDRV_FREQOUT(n)                  (0x0B654 +\
                                         ((n) * 4))        // Frequency Out Control Register
#define EDRV_TSIM                        0x0B674           // Time Sync Interrupt Mask Register
#define EDRV_TSICR                       0x0B66C           // Time Sync Interrupt Cause Register
#define EDRV_TSAUXC                      0x0B640           // Auxiliary Control Register
#define EDRV_AUXSTMPL0                   0x0B65C           // Auxiliary Time Stamp 0 Reg - Low
#define EDRV_AUXSTMPH0                   0x0B660           // Auxiliary Time Stamp 0 Reg - High
#define EDRV_TRGTTIML(n)                 (0x0B644 +\
                                         ((n) * 8))        // Target Time Register (n) Low
#define EDRV_TRGTTIMH(n)                 (0x0B648 +\
                                         ((n) * 8))        // Target Time Register (n) High
#define EDRV_TSSDP                       0x0003C           // Time Sync SDP Configuration Register
#define	EDRV_TSIM_TT(n)                  (1 << (3 + (n)))  // Target time (n) Trigger Mask.
#define EDRV_TSAUXC_SAMP_AUTO            0x00000008        // Sample SYSTIM into AUXSTMP0 register
#define EDRV_TSAUXC_EN_TT(n)             (1 << (n))        // Enable target time n
#define EDRV_TSAUXC_EN_CLK(n)            (5 << ((3 * (n)) + 2))   // Enable Configurable Frequency Clock 0
#define EDRV_TSSDP_TS_EN_SDP(n)          (1 << (8 +\
                                         (3 * (n))))       // SDP(n) is assigned to Tsync
#define EDRV_TSSDP_TS_SEL_SDP(n, m)      ((m) << (6 +\
                                         (3 * (n))))       // SDP(n) allocation to Tsync(m) event
#define SET_DIR_OUT_SDP(n)               (1 << (22 + (n))) // Set direction Out for SDP0 and SDP1
#define EDRV_TSICR_TT0                   (1 << 3)          // Target time 0 interrupt cause
#define EDRV_TSICR_TT1                   (1 << 4)          // Target time 1 interrupt cause
#define EDRV_TSICR_SYSTIM                (1 << 0)          // Wrap around interrupt cause

//------------------------------------------------------------------------------
// MDIC specific defines
//------------------------------------------------------------------------------

#define EDRV_MDIC_DATA_MASK      0x0000FFFF      // Data mask
#define EDRV_MDIC_OP_READ        0x08000000      // Read command for PHY register
#define EDRV_MDIC_OP_WRITE       0x04000000      // Write command for PHY register
#define EDRV_MDIC_RD             0x10000000      // Ready Bit to indicate completion of Read or Write
#define EDRV_MDIC_INTR_DIS       0x20000000      // Interrupt Enable at the end of operation specified by opcode
#define EDRV_MDIC_ERR            0x40000000      // Error in read
#define EDRV_MDIC_REGADD_MASK    0x001F0000      // PHY register address mask
//------------------------------------------------------------------------------
//                          Interrupt defines
//------------------------------------------------------------------------------

#define EDRV_EIMC_CLEAR_ALL      0xC000001F      // Clear all interrupts
#define EDRV_INTR_GPIE_NSICR     (1 << 0)        // Non Selective Interrupt Clear on Read
#define EDRV_INTR_GPIE_MULT_MSIX (1 << 4)        // Multiple MSIX
#define EDRV_INTR_GPIE_PBA       (1 << 31)       // PBA Support
#define EDRV_INTR_ICR_TXDW       0x00000001      // Transmit descriptor write back
#define EDRV_INTR_ICR_RXDW       (1 << 7)        // Receive descriptor write back
#define EDRV_INTR_ICR_TIME_SYNC  (1 << 19)       // Time Sync interrupt
#define	EDRV_INTR_ICR_RXDMT0     (1 << 4)        // Receive Descriptor Minimum Threshold Reached
#define EDRV_INTR_ICR_RXMISS     (1 << 6)        // Missed packet interrupt
#define EDRV_INTR_ICR_FER        (1 << 22)       // Fatal Error
#define EDRV_EIMC_OTHR_EN        (1 << 31)       // Other Interrupt Cause Active
#define EDRV_EICS_OTHER          (1 << 0)        // Vector for Other Interrupt Cause in MSI-X mode
#define EDRV_EICS_QUEUE          0x0000001E      // All queue interrupts
#define EDRV_EICS_TXRXQUEUE1     (1 << 1)        // Vector for TX-RX queue 0
#define EDRV_EICS_TXRXQUEUE2     (1 << 2)        // Vector for TX-RX queue 1
#define EDRV_EICS_TXRXQUEUE3     (1 << 3)        // Vector for TX-RX queue 2
#define EDRV_EICS_TXRXQUEUE4     (1 << 4)        // Vector for TX-RX queue 3
#define EDRV_IVAR_VALID          0x80            // Interrupt vector valid bit
#define EDRV_INTR_ICR_MASK_DEF   (EDRV_INTR_ICR_TXDW           /* Transmit descriptor write back */\
                                  | EDRV_INTR_ICR_RXDW         /* Receive descriptor write back */\
                                  | EDRV_INTR_ICR_RXDMT0       /* Receive Descriptor Minimum Threshold Reached */\
                                  | EDRV_INTR_ICR_RXMISS       /* Missed packet interrupt */\
                                  | EDRV_INTR_ICR_FER          /* Fatal Error */\
                                  | EDRV_INTR_ICR_TIME_SYNC)   /* Time Sync interrupt */

//------------------------------------------------------------------------------
// Phy register offsets and defines
//------------------------------------------------------------------------------
#define PHY_CONTROL_REG_OFFSET   0x00            // Phy Control register
#define PHY_STATUS_REG_OFFSET    0x01            // Phy Status Register
#define PHY_LINK_SPEED_100       0x2000          // 100 MB/s Link speed
#define PHY_RESET                0x8000          // Phy reset
#define PHY_MODE_FD              0x0100          // Full Duplex
#define PHY_CONTROL_POWER_DOWN   0x0800          // Power down Phy
#define PHY_I210_COPPER_SPEC     0x0010          // I210 specific phy register
#define PHY_I210_CS_POWER_DOWN   0x0002          // Power down phy

//------------------------------------------------------------------------------
// Control/Control Extension register defines
//------------------------------------------------------------------------------

#define EDRV_CTRL_FD             0x00000001      // Full Duplex
#define EDRV_CTRL_MASTER_DIS     (1 << 2)        // GIO Master Disable
#define EDRV_CTRL_SLU            (1 << 6)        // Set Link Up
#define EDRV_CTRL_ILOS           (1 << 7)        // Invert Loss-of-Signal (LOS/LINK) Signal
#define EDRV_CTRL_RST            (1 << 26)       // Port Software Reset
#define EDRV_CTRL_RFCE           (1 << 27)       // Receive Flow Control Enable
#define EDRV_CTRL_TFCE           (1 << 28)       // Transmit Flow Control Enable
#define EDRV_CTRL_DEV_RST        (1 << 29)       // Device Reset
#define EDRV_CTRL_PHY_RST        (1 << 31)       // PHY Reset
#define EDRV_CTRL_EXTN_DRV_LOAD  (1 << 28)       // Software driver loaded. Used to take control from Firmware

//------------------------------------------------------------------------------
// Status register defines
//------------------------------------------------------------------------------
#define EDRV_STATUS_MASTER_EN            (1 << 19)         // GIO Master Enable
#define EDRV_STATUS_PF_RST_DONE          (1 << 21)         // Software reset done
#define EDRV_STATUS_LU                   (1 << 1)          // Link Up
#define EDRV_STATUS_DEV_RST_SET          (1 << 20)         // Device reset done
#define EDRV_SW_RST_DONE_TIMEOUT         10   // ms        // Software rest timeout
#define EDRV_MASTER_DIS_TIMEOUT          90   // ms        // Master disable timeout
#define EDRV_LINK_UP_TIMEOUT             6000 // ms        // Link up timeout
#define EDRV_AUTO_READ_DONE_TIMEOUT      10                // Auto register read done timeout
#define EDRV_PHY_SEMPHORE_TIMEOUT        100
#define EDRV_POLL_TIMEOUT                3                 // Tx-Rx Enable bit poll timeout

//------------------------------------------------------------------------------
// Statistic registers
//------------------------------------------------------------------------------

#define EDRV_STAT_TPR            0x040D0         // Total Packets Received
#define EDRV_STAT_GPRC           0x04074         // Good Packets Received Count
#define EDRV_STAT_BPRC           0x04078         // Broadcast Packets Received Count
#define EDRV_STAT_MPRC           0x0407C         // Multicast Packets Received Count
#define EDRV_PQGPRC(n)           ((n < 4) ? (0x10010 + 0x100 * n) :\
                                 (0x10010 + 0x100  * (n - 4)))       // Per Queue Good Packets Received Count
// Utility Macros
#define EDRV_GET_RX_DESC(pQueue, index)   (&(((tEdrvAdvRxDesc*)pQueue->pDescVirt)[index]))
#define EDRV_GET_TX_DESC(pQueue, index)   (&(((tEdrvAdvTxDesc*)pQueue->pDescVirt)[index]))
#define EDRV_GET_CTXT_DESC(pQueue, index) (&(((tEdrvContextDesc*)pQueue->pDescVirt)[index]))
#define EDRV_GET_TTX_DESC(pQueue, index) (&(((tEdrvTtxDesc*)pQueue->pDescVirt)[index]))

#define EDRV_REGDW_READ(reg)            readl((UINT8*)edrvInstance_l.pIoAddr + reg)
#define EDRV_REGDW_WRITE(reg, val)      writel(val, (UINT8*)edrvInstance_l.pIoAddr + reg)
#define EDRV_REGB_READ(reg)             readb((UINT8*)edrvInstance_l.pIoAddr + reg)

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void target_signalTracePoint(UINT8 tracePointNumber_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   target_signalTracePoint(p)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#define EDRV_COUNT_SEND                 TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT              TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR              TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX                   TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX                   TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION        TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL            TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN               TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST              TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC           TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT          TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ           TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER         TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN              TGT_DBG_SIGNAL_TRACE_POINT(18)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Advanced transmit data descriptor

This union points to buffers used to pass data to the controller.
*/
typedef union
{
    struct
    {
        UINT64      bufferAddrLe;                           ///< Address of descriptor's data buffer (little-endian)
        UINT32      cmdTypeLen;                             ///< Command, descriptor type, data length
        UINT32      statusIdxPaylen;                        ///< Status, index, payload length
    } sRead;

    struct
    {
        UINT64      timeStampLe;                            ///< Reserved or DMA timestamp (if enabled) (little-endian)
        UINT32      rsvdLe;                                 ///< Reserved (little-endian)
        UINT32      statusLe;                               ///< Status (little-endian)
    } sWb;
} tEdrvAdvTxDesc;

/**
 \brief Advanced context descriptor structure

This structure specifies a launch time for a packet stored in succeeding
advanced data descriptors.
 */
typedef struct
{
    UINT32          ipMaclenVlan;                           ///< IPv4 address / MAC length / VLAN
    UINT32          launchTime;                             ///< Launch time for packet [32:56]
    UINT32          tucmdType;                              ///< Command type
    UINT32          idxL4lenMss;                            ///< Index, L4 MSS (maximum segment size)
} tEdrvContextDesc;

/**
\brief Advanced receive data descriptor

This union points to buffers used to pass data to the device driver from the
controller.
*/
typedef union
{
    struct
    {
        UINT64      bufferAddrLe;                           ///< Data buffer address (little-endian)
        UINT64      headerAddrLe;                           ///< Head buffer address (little-endian)
    } sRead;

    struct
    {
        UINT16      rssPktType;                             ///< RSS type or packet Type
        UINT16      headerLen;                              ///< Header length
        UINT32      rssHash;                                ///< RSS hash
        UINT32      extStatusError;                         ///< Extended error and status
        UINT32      lenVlanTag;                             ///< Data length or VLAN tag
    } sWb;
} tEdrvAdvRxDesc;

/**
\brief Time-triggered send descriptor

This structure is used to refer to a pair of context descriptor and advanced
TX data descriptor pointing to a single data packet with its respective launch
time.

It simplifies the queue handling logic in time-triggered send mode and makes it
consistent with older configurations.
*/
typedef struct
{
    tEdrvContextDesc    ctxtDesc;                           ///< Context Descriptor
    tEdrvAdvTxDesc      advDesc;                            ///< Data Descriptor
} tEdrvTtxDesc;

/**
\brief Structure for queue vector

This structure is used as reference in the ISR to identify the corresponding
TX-RX queue pair.
*/
typedef struct
{
    UINT                queueIdx;                           ///< Queue index
    UINT                vector;                             ///< Vector index
    char                strName[INTERRUPT_STRING_SIZE];     ///< Name to be registered for the vector
} tEdrvQVector;

/**
\brief Structure describing a packet buffer.

This structure describes a packet buffer of the Ethernet driver. It is used for
bookkeeping the DMA address and length.
*/
typedef struct
{
    dma_addr_t          dmaAddr;                            ///< DMA address
    void*               pVirtAddr;                          ///< Virtual address
    UINT                len;                                ///< Length of the buffer
} tEdrvPktBuff;

/**
\brief Structure describing an Edrv queue

This structure describes an Ethernet driver queue.
*/
typedef struct
{
    tEdrvQVector*       pQvector;                           ///< Pointer to the vector structure for this queue
    int                 index;                              ///< Queue index
    dma_addr_t          descDma;                            ///< DMA address for descriptor queue
    void*               pDescVirt;                          ///< Virtual address for descriptor queue
    tEdrvTxBuffer*      apTxBuffer[EDRV_MAX_TX_DESCRIPTOR]; ///< TX buffer array
    void __iomem*       pBuf;                               ///< Pointer to buffer for the queue
    tEdrvPktBuff*       pPktBuff;                           ///< Bookkeeping structure
    int                 nextDesc;                           ///< Next descriptor to be used
    int                 nextWb;                             ///< Next descriptor to be written back
} tEdrvQueue;

/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                          ///< Init parameters
    struct pci_dev*     pPciDev;                            ///< Pointer to the PCI device structure
    void*               pIoAddr;                            ///< Pointer to the register space of the Ethernet controller

    tEdrvQueue*         pTxQueue[EDRV_MAX_TX_QUEUES];       ///< Array of TX queues
    tEdrvQueue*         pRxQueue[EDRV_MAX_RX_QUEUES];       ///< Array of RX queues
    tEdrvQVector*       pQvector[EDRV_MAX_QUEUE_VECTOR];    ///< Array of vector queues
    UINT                txMaxQueue;                         ///< Max number of TX queues
    UINT                rxMaxQueue;                         ///< Max number of RX queues
    UINT                numQVectors;                        ///< Number of queue vectors (Total = numQvectors + 1)

    void*               pTxBuf;                             ///< Pointer to the TX buffer
    BOOL                afTxBufUsed[EDRV_MAX_TX_BUFFERS];   ///< Array indicating the use of a specific TX buffer

    struct msix_entry*  pMsixEntry;                         ///< Pointer to the MSI-X structure

    // Timer related members
    tTimerHdl           aTimerHdl[EDRV_MAX_TIMER_COUNT];    ///< Array for timer handle
    tHresCallback       hresTimerCb;                        ///< Timer callback
    BOOL                fInitialized;                       ///< Flag determines if module is initialized
    struct timespec     aTimeOut[EDRV_MAX_TIMER_COUNT];     ///< Array to hold the timeout values for a cyclic timer
} tEdrvInstance;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static irqreturn_t edrvTimerInterrupt(int irqNum_p, void* ppDevInstData_p);
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p);
static UINT32 readSystimRegister(struct timespec* psTime_p);
static void writeSystimRegister(const struct timespec* psTime_p);
static void releaseHwSemaphoreGeneric(void);
static tOplkError acquireHwSemaphore(void);
static void releaseHwSemaphore(void);
static tOplkError acquireSwFwSync(UINT16 mask_p);
static void releaseSwFwSync(UINT16 mask_p);
static void writeMdioPhyReg(UINT phyreg_p, UINT16 value_p);
static UINT16 readMdioPhyReg(UINT phyreg_p);
static void freeTxBuffersOfQueue(const tEdrvQueue* pTxQueue_p);
static void freeTxBuffers(void);
static void freeTxQueues(void);
static tOplkError initTxQueue(tEdrvQueue* pTxQueue_p);
static void configureTxQueue(const tEdrvQueue* pTxQueue_p);
static void freeRxQueues(void);
static void freeRxBuffersOfQueue(const tEdrvQueue* pRxQueue_p);
static void freeRxBuffers(void);
static tOplkError initRxQueue(tEdrvQueue* pRxQueue_p);
static tOplkError allocateRxBuffer(tEdrvQueue* pRxQueue_p);
static void configureRxQueue(const tEdrvQueue* pRxQueue_p);
static void initQavMode(void);
static void writeIvarRegister(int vector_p, int index_p, int offset_p);
static int requestMsixIrq(void);
static int initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p);
static void removeOnePciDev(struct pci_dev* pPciDev_p);
static tOplkError sendNormalBuffer(tEdrvTxBuffer* pBuffer_p);

#if (EDRV_USE_TTTX != FALSE)
static tOplkError sendTimeTrigBuffer(tEdrvTxBuffer* pBuffer_p);
#endif

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
static struct pci_device_id aEdrvPciTbl[] =
{
    { 0x8086, 0x1533, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },   //I210 (copper only)
    { 0x8086, 0x157B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },   //I210 (Flash-less, copper only)
    { 0, }
};

MODULE_DEVICE_TABLE(pci, aEdrvPciTbl);

tEdrvInstance edrvInstance_l;
static tBufAlloc* pBufAlloc_l = NULL;
static struct pci_driver edrvDriver_l =
{
     .name      = DRIVER_NAME,
     .id_table  = aEdrvPciTbl,
     .probe     = initOnePciDev,
     .remove    = removeOnePciDev,
};

//===========================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(const tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError      ret = kErrorOk;
    int             result;
    int             index;
    tBufData        bufData;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // clear driver structure
    OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    edrvDriver_l.name         = DRIVER_NAME;
    edrvDriver_l.id_table     = aEdrvPciTbl;
    edrvDriver_l.probe        = initOnePciDev;
    edrvDriver_l.remove       = removeOnePciDev;

    printk("Registering Driver.....");
    result = pci_register_driver(&edrvDriver_l);
    if (result != 0)
    {
        printk("%s pci_register_driver failed with %d\n", __func__, result);
        ret = kErrorNoResource;
        goto Exit;
    }

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev=NULL\n", __func__);
        edrv_exit();
        ret = kErrorNoResource;
        goto Exit;
    }

    // init and fill buffer allocation instance
    if ((pBufAlloc_l = bufalloc_init(EDRV_MAX_TX_BUFFERS)) == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    for (index = 0; index < EDRV_MAX_TX_BUFFERS; index++)
    {
        bufData.bufferNumber = index;
        bufData.pBuffer = (UINT8*)edrvInstance_l.pTxBuf + (index * EDRV_MAX_FRAME_SIZE);

        bufalloc_addBuffer(pBufAlloc_l, &bufData);
    }

    // local MAC address might have been changed in EdrvInitOne
    printk("%s local MAC = ", __func__);
    for (index = 0; index < 6; index++)
    {
        printk("%02X ", (UINT)edrvInstance_l.initParam.aMacAddr[index]);
    }
    printk("\n");

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down Ethernet driver

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_exit(void)
{
    if (edrvDriver_l.name != NULL)
    {
        printk("%s calling pci_unregister_driver()\n", __func__);
        pci_unregister_driver (&edrvDriver_l);
        // clear buffer allocation
        bufalloc_exit(pBufAlloc_l);
        pBufAlloc_l = NULL;
        // clear driver structure
        OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    }
    else
    {
        printk("%s PCI driver for openPOWERLINK already unregistered\n", __func__);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address

This function returns the MAC address of the Ethernet controller

\return The function returns a pointer to the MAC address.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
const UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    tOplkError      ret = kErrorOk;
    UINT32          data;
    int             index;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    // entry 0 is used for local MAC address
    for (index = 1; index < 16; index++)
    {
        data = EDRV_REGDW_READ(EDRV_RAH(index));
        if ((data & EDRV_RAH_AV) == 0)
        {   // free MAC address entry
            break;
        }
    }

    if (index == 16)
    {   // no free entry found
        printk("%s Implementation of Multicast Table Array support required\n", __func__);
        ret = kErrorEdrvInit;
        goto Exit;
    }
    else
    {
        // write MAC address to free entry
        data = 0;
        data |= pMacAddr_p[0] << 0;
        data |= pMacAddr_p[1] << 8;
        data |= pMacAddr_p[2] << 16;
        data |= pMacAddr_p[3] << 24;
        EDRV_REGDW_WRITE(EDRV_RAL(index), data);

        data = 0;
        data |= pMacAddr_p[4] << 0;
        data |= pMacAddr_p[5] << 8;
        data |= EDRV_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_RAH(index), data);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    tOplkError          ret = kErrorOk;
    UINT32              data;
    int                 index;
    UINT32              addrLow;
    UINT32              addrHigh;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    addrLow = 0;
    addrLow |= pMacAddr_p[0] << 0;
    addrLow |= pMacAddr_p[1] << 8;
    addrLow |= pMacAddr_p[2] << 16;
    addrLow |= pMacAddr_p[3] << 24;

    addrHigh = 0;
    addrHigh |= pMacAddr_p[4] << 0;
    addrHigh |= pMacAddr_p[5] << 8;
    addrHigh |= EDRV_RAH_AV;

    for (index = 1; index < 16; index++)
    {
        data = EDRV_REGDW_READ(EDRV_RAH(index));
        if (addrHigh == (data & (EDRV_RAH_AV | 0xFFFF)))
        {
            data = EDRV_REGDW_READ(EDRV_RAL(index));
            if (data == addrLow)
            {   // set address valid bit to invalid
                EDRV_REGDW_WRITE(EDRV_RAH(index), 0);
                break;
            }
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                               UINT count_p,
                               UINT entryChanged_p,
                               UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError          ret = kErrorOk;
    tBufData            bufData;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (edrvInstance_l.pTxBuf == NULL)
    {
        printk("%s Tx buffers currently not allocated\n", __func__);
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // get a free Tx buffer from the allocation instance
    ret = bufalloc_getBuffer(pBufAlloc_l, &bufData);
    if (ret != kErrorOk)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->pBuffer = bufData.pBuffer;
    pBuffer_p->txBufferNumber.value = bufData.bufferNumber;
    pBuffer_p->maxBufferSize = EDRV_MAX_FRAME_SIZE;
    edrvInstance_l.afTxBufUsed[bufData.bufferNumber] = TRUE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError  ret;
    tBufData    bufData;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufData.pBuffer = pBuffer_p->pBuffer;
    bufData.bufferNumber = pBuffer_p->txBufferNumber.value;

    edrvInstance_l.afTxBufUsed[pBuffer_p->txBufferNumber.value] = FALSE;
    ret = bufalloc_releaseBuffer(pBufAlloc_l, &bufData);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError      ret = kErrorOk;
    UINT            bufferNumber;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if ((bufferNumber >= EDRV_MAX_TX_BUFFERS) || (edrvInstance_l.afTxBufUsed[bufferNumber] == FALSE))
    {
        ret = kErrorEdrvBufNotExisting;
        goto Exit;
    }

#if (EDRV_USE_TTTX != FALSE)
    if (pBuffer_p->fLaunchTimeValid)
        ret = sendTimeTrigBuffer(pBuffer_p);
    else
#endif
        ret = sendNormalBuffer(pBuffer_p);

Exit:
    return ret;
}

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
//------------------------------------------------------------------------------
/**
\brief  Release Rx buffer

This function releases a late release Rx buffer.

\param[in,out]  pRxBuffer_p         Rx buffer to be released

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_releaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
    UNUSED_PARAMETER(pRxBuffer_p);

    return kErrorOk;
}
#endif

//------------------------------------------------------------------------------
// Timer helper functions
//------------------------------------------------------------------------------
#if (EDRV_USE_TTTX != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Retrieve current MAC time

The function retrieves the current MAC time.

\param[out]     pCurtime_p          Pointer to store the current MAC time.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError edrv_getMacTime(UINT64* pCurtime_p)
{
    UINT32      timh;
    UINT32      timl;
    UINT32      reg;

    if (pCurtime_p == NULL)
        return kErrorNoResource;

    // Sample the current SYSTIM time in Auxiliary registers
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg |= EDRV_TSAUXC_SAMP_AUTO;
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    timl = EDRV_REGDW_READ(EDRV_AUXSTMPL0);
    timh = EDRV_REGDW_READ(EDRV_AUXSTMPH0);

    *pCurtime_p = (UINT64)timh * SEC_TO_NSEC + (UINT64)timl;

    return kErrorOk;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Start timer

The function starts the timer in the I210.

\param[in]      pTimerHdl_p         Timer handle of the timer to start.
\param[in]      index_p             Index of the timer.
\param[in]      frequency_p         Cycle time (frequency).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError edrv_startTimer(const tTimerHdl* pTimerHdl_p, UINT32 index_p, UINT64 frequency_p)
{
    UINT32              reg;
    struct timespec     ts;
    struct timespec*    pTimeout = NULL;
    UINT64              seconds, nanoseconds, rem;

    if (!edrvInstance_l.fInitialized)
        return kErrorOk;

    if ((pTimerHdl_p == NULL) || (index_p >= EDRV_MAX_TIMER_COUNT))
    {
        printk("%s() Invalid parameters for the timer\n", __func__);
        return kErrorTimerInvalidHandle;
    }

    pTimeout = &edrvInstance_l.aTimeOut[index_p];

    edrvInstance_l.aTimerHdl[index_p] = *pTimerHdl_p; //TODO: Handle already started timer

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSSDP);
    reg |= (EDRV_TSSDP_TS_SEL_SDP(index_p, index_p) | EDRV_TSSDP_TS_EN_SDP(index_p));

    EDRV_REGDW_WRITE(EDRV_TSSDP, reg);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg &= ~(EDRV_TSAUXC_EN_TT(index_p));
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    readSystimRegister(&ts);

    // Calculate the timeouts considering the roll over
    seconds = ts.tv_nsec + frequency_p;
    rem = do_div(seconds, 1000000000);
    ts.tv_sec += seconds;

    nanoseconds = (ts.tv_nsec + frequency_p);
    rem = do_div(nanoseconds, 1000000000);
    ts.tv_nsec = rem;

    EDRV_REGDW_WRITE(EDRV_TRGTTIML(index_p), ts.tv_nsec);
    EDRV_REGDW_WRITE(EDRV_TRGTTIMH(index_p), ts.tv_sec);

    // Store the last time out values to be used for calculating
    // the restart timeouts.
    pTimeout->tv_sec = ts.tv_sec;
    pTimeout->tv_nsec = ts.tv_nsec;

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg |= SET_DIR_OUT_SDP(index_p);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg |= EDRV_TSAUXC_EN_TT(index_p);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSIM);
    reg |= (EDRV_TSIM_TT(index_p) | EDRV_TSICR_SYSTIM);
    EDRV_REGDW_WRITE(EDRV_TSIM, reg);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Stop timer

The function stop the timer in the I210.

\param[in]      pTimerHdl_p         Handle of the timer to stop.
\param[in]      index_p             Index of the timer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError edrv_stopTimer(const tTimerHdl* pTimerHdl_p, UINT32 index_p)
{
    UINT32      reg;

    // Check parameter validity
    ASSERT(pTimerHdl_p != NULL);

    if (!edrvInstance_l.fInitialized)
        return kErrorOk;

    if (index_p >= EDRV_MAX_TIMER_COUNT)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p != edrvInstance_l.aTimerHdl[index_p])
        return kErrorTimerInvalidHandle;

    edrvInstance_l.aTimerHdl[index_p] = 0;

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSIM);
    reg &= ~(EDRV_TSIM_TT(index_p));
    EDRV_REGDW_WRITE(EDRV_TSIM, reg);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg &= ~(EDRV_TSAUXC_EN_TT(index_p));
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Restart the timer on the I210

The function restarts the timer on the I210

\param[in]      pTimerHdl_p         Handle of the timer to restart.
\param[in]      index_p             Index of the timer.
\param[in]      frequency_p         Cycle time (frequency of the repetition).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError edrv_restartTimer(const tTimerHdl* pTimerHdl_p, UINT32 index_p, UINT64 frequency_p)
{
    UINT32              reg;
    struct timespec     ts;
    struct timespec*    pTimeout = NULL;
    UINT64              seconds, nanoseconds, rem;

    if (!edrvInstance_l.fInitialized)
        return kErrorOk;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    pTimeout = &edrvInstance_l.aTimeOut[index_p];
    edrvInstance_l.aTimerHdl[index_p] = *pTimerHdl_p;

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg &= ~(EDRV_TSAUXC_EN_TT(index_p));
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    // Update the timeouts considering the roll over
    seconds = pTimeout->tv_nsec + frequency_p;
    rem = do_div(seconds, 1000000000);
    ts.tv_sec = pTimeout->tv_sec + seconds;

    nanoseconds = (pTimeout->tv_nsec + frequency_p);
    rem = do_div(nanoseconds, 1000000000);
    ts.tv_nsec = rem;

    EDRV_REGDW_WRITE(EDRV_TRGTTIML(index_p), ts.tv_nsec);
    EDRV_REGDW_WRITE(EDRV_TRGTTIMH(index_p), ts.tv_sec);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg |= (EDRV_TSAUXC_EN_TT(index_p));
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    // Store the timeout for the next call
    pTimeout->tv_sec = ts.tv_sec;
    pTimeout->tv_nsec = ts.tv_nsec;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Register the timer callback function

The function is used to register a timer callback function for the I210
timer.

\param[in]      pfnHighResCb_p      Pointer to the callback routine to register

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError edrv_registerHresCallback(tHresCallback pfnHighResCb_p)
{
    if (pfnHighResCb_p == NULL)
        return kErrorIllegalInstance;

    edrvInstance_l.hresTimerCb = pfnHighResCb_p;
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief     edrvTimerInterrupt()

The function implements the interrupt service routine for the timer
interrupt. It calls the timer callback function of the high-resolution
timer module.

\param[in]      irqNum_p            IRQ number
\param[in,out]  ppDevInstData_p     Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t edrvTimerInterrupt(int irqNum_p, void* ppDevInstData_p)
{
    UINT32          status;
    UINT32          reg;
    irqreturn_t     handled = IRQ_HANDLED;
    UINT32          index;

    UNUSED_PARAMETER(irqNum_p);

    if (ppDevInstData_p != edrvInstance_l.pPciDev)
    {
        handled = IRQ_NONE;
        goto Exit;
    }

    status = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    if ((status & EDRV_INTR_ICR_TIME_SYNC) == 0)
    {
        handled = IRQ_NONE;
        EDRV_COUNT_PCI_ERR;
        goto Exit;
    }

    status &= ~EDRV_INTR_ICR_TIME_SYNC;
    EDRV_REGDW_WRITE(EDRV_INTR_SET_REG, status);
    reg = EDRV_REGDW_READ(EDRV_TSICR);

    if (reg & EDRV_TSICR_SYSTIM)
    {
        return handled;
    }

    if (reg & EDRV_TSICR_TT0)
    {
        // Timer 0
        index = 0;
#if (EDRV_USE_TTTX != FALSE)
    // Call timer CallBack
    if ((edrvInstance_l.hresTimerCb != NULL) && (edrvInstance_l.aTimerHdl[index] != 0))
    {
        edrvInstance_l.hresTimerCb(&edrvInstance_l.aTimerHdl[index]);
    }
#endif
    }

    if (reg & EDRV_TSICR_TT1)
    {
        // Timer 1
        index = 1;
#if (EDRV_USE_TTTX != FALSE)
    // Call timer CallBack
    if ((edrvInstance_l.hresTimerCb != NULL) && (edrvInstance_l.aTimerHdl[index] != 0))
    {
        edrvInstance_l.hresTimerCb(&edrvInstance_l.aTimerHdl[index]);
    }
#endif
    }

Exit:
    return handled;
}

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver interrupt handler

This function is the interrupt service routine for the Ethernet driver.

\param[in]      irqNum_p            IRQ number
\param[in,out]  pDevInstData_p      Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p)
{
    UINT32          reg;
    irqreturn_t     handled = IRQ_HANDLED;
    int             index;
    tEdrvQVector*   pQVector = (tEdrvQVector*)pDevInstData_p;
    tEdrvQueue*     pTxQueue;
    tEdrvQueue*     pRxQueue;

    UNUSED_PARAMETER(irqNum_p);

    if (edrvInstance_l.pQvector[pQVector->queueIdx] != pQVector)
    {
        handled = IRQ_NONE;
        EDRV_COUNT_PCI_ERR;
        goto Exit;
    }

    // Retrieve the queue structure for the vector
    pTxQueue = edrvInstance_l.pTxQueue[pQVector->queueIdx];
    pRxQueue = edrvInstance_l.pRxQueue[pQVector->queueIdx];

    // Driver only supports 1 Rx queue, so avoid handling receive to second queue
    if (pQVector->queueIdx != EDRV_SR_QUEUE)
        pRxQueue = NULL;

    // Read the interrupt status
    reg = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    // Process Rx with priority over Tx
    if ((pRxQueue != NULL) && (reg & EDRV_INTR_ICR_RXDW))
    {
        tEdrvAdvRxDesc* pAdvRxDesc;
        tEdrvRxBuffer   rxBuffer;

        index = pRxQueue->nextWb;

        // Clear the Rx write-back bit
        reg &= ~EDRV_INTR_ICR_RXDW;

        pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue, index);

        while ((pAdvRxDesc->sWb.extStatusError & EDRV_RDESC_STATUS_DD) &&
              (pAdvRxDesc->sWb.extStatusError & EDRV_RDESC_STATUS_EOP))
        {
            UINT rcvLen;

            if (pAdvRxDesc->sWb.extStatusError & EDRV_RDESC_ERRORS_RXE)
            {
                EDRV_COUNT_RX_ERR_CRC;
            }
            else
            {
                // good packet
                rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
                rcvLen = (0x0000FFFF & pAdvRxDesc->sWb.lenVlanTag);
                rxBuffer.rxFrameSize = rcvLen;
                rxBuffer.pBuffer = pRxQueue->pPktBuff[index].pVirtAddr;

                EDRV_COUNT_RX;

                dma_sync_single_for_cpu(&edrvInstance_l.pPciDev->dev,
                                        pRxQueue->pPktBuff[index].dmaAddr,
                                        rcvLen,
                                        DMA_FROM_DEVICE);

                // Forward the Rcv packet to DLL
                if (edrvInstance_l.initParam.pfnRxHandler != NULL)
                {
                    edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
                }

            }

            // Prepare descriptor for next Rcv
            pAdvRxDesc->sRead.bufferAddrLe = cpu_to_le64(pRxQueue->pPktBuff[index].dmaAddr);
            pAdvRxDesc->sRead.headerAddrLe = 0;

            // Handle the processed descriptor to Hw
            EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue->index), index);

            // Increment the pointer
            index = ((index + 1) & EDRV_MAX_RX_DESC_LEN);
            pRxQueue->nextWb = index;
            pAdvRxDesc = EDRV_GET_RX_DESC(pRxQueue, index);
        }
    }

    // Process Tx
    if ((pTxQueue != NULL) && (reg & EDRV_INTR_ICR_TXDW))
    {
        tEdrvTtxDesc*   pTtxDesc;
        tEdrvAdvTxDesc* pAdvTxDesc;

        // Clear Tx write-Back bit
        reg &= ~EDRV_INTR_ICR_TXDW;

        if (pQVector->queueIdx == EDRV_SR_QUEUE)
        {
            index = 0;
            index = pTxQueue->nextWb;

            pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue, index);
            pAdvTxDesc = &(pTtxDesc->advDesc);

            do
            {
                if (pAdvTxDesc->sWb.statusLe & EDRV_TDESC_STATUS_DD)
                {
                    // Process the send packet
                    tEdrvTxBuffer*  pTxBuffer;
                    UINT32          txStatus;

                    txStatus = pAdvTxDesc->sWb.statusLe;
                    pAdvTxDesc->sWb.statusLe = 0;
                    pTxBuffer = pTxQueue->apTxBuffer[index];
                    pTxQueue->apTxBuffer[index] = NULL;

                    dma_unmap_single(&edrvInstance_l.pPciDev->dev,
                                     pTxQueue->pPktBuff[index].dmaAddr,
                                     pTxQueue->pPktBuff[index].len, DMA_TO_DEVICE);
                    EDRV_COUNT_TX;
                    if (pTxBuffer != NULL)
                    {
                        // Call Tx handler of Data link layer
                        if (pTxBuffer->pfnTxHandler != NULL)
                        {
                            pTxBuffer->pfnTxHandler(pTxBuffer);
                        }
                    }
                    else
                    {
                        EDRV_COUNT_TX_FUN;
                    }

                    index = ((index + 1) & EDRV_MAX_TTX_DESC_LEN);

                    pTxQueue->nextWb = index;
                    pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue, index);
                    pAdvTxDesc = &(pTtxDesc->advDesc);
                }
                else
                {
                    break;      // no completed packet to process
                }
            } while (pAdvTxDesc->sWb.statusLe & EDRV_TDESC_STATUS_DD);
        }
        else
        {
            index = 0;
            index = pTxQueue->nextWb;
            pAdvTxDesc = EDRV_GET_TX_DESC(pTxQueue, index);

            do
            {
                if (pAdvTxDesc->sWb.statusLe & EDRV_TDESC_STATUS_DD)
                {
                    // Process the send packet
                    tEdrvTxBuffer*  pTxBuffer;
                    UINT32          txStatus;

                    txStatus = pAdvTxDesc->sWb.statusLe;
                    pAdvTxDesc->sWb.statusLe = 0;
                    pTxBuffer = pTxQueue->apTxBuffer[index];
                    pTxQueue->apTxBuffer[index] = NULL;

                    dma_unmap_single(&edrvInstance_l.pPciDev->dev,
                                     pTxQueue->pPktBuff[index].dmaAddr,
                                     pTxQueue->pPktBuff[index].len, DMA_TO_DEVICE);
                    EDRV_COUNT_TX;
                    if (pTxBuffer != NULL)
                    {
                        // Call Tx handler of Data link layer
                        if (pTxBuffer->pfnTxHandler != NULL)
                        {
                            pTxBuffer->pfnTxHandler(pTxBuffer);
                        }
                    }
                    else
                    {
                        EDRV_COUNT_TX_FUN;
                    }

                    index = ((index + 1) & EDRV_MAX_TX_DESC_LEN);

                    pTxQueue->nextWb = index;
                    pAdvTxDesc = EDRV_GET_TX_DESC(pTxQueue, index);
                }
                else
                {
                    break;      // no completed packet to process
                }
            } while (pAdvTxDesc->sWb.statusLe & EDRV_TDESC_STATUS_DD);
        }
    }

    // Set the values in ICR again which were not processed here so that they are available
    // for processing in other ISR
    EDRV_REGDW_WRITE(EDRV_INTR_SET_REG, reg);

Exit:
    return handled;
}

//------------------------------------------------------------------------------
/**
\brief  Read SYSTIM register of I210

The function reads the current time from the SYSTIM register in
struct timespec format.

\param[out]     psTime_p            Pointer to timespec struct where the current
                                    time will be stored.

\retval    The function returns the nanoseconds part of the current time.
*/
//------------------------------------------------------------------------------
static UINT32 readSystimRegister(struct timespec* psTime_p)
{
    UINT32  sec;
    UINT32  nsec;
    UINT32  psec;

    psec = EDRV_REGDW_READ(EDRV_SYSTIMR_REG);
    nsec = EDRV_REGDW_READ(EDRV_SYSTIML_REG);
    sec = EDRV_REGDW_READ(EDRV_SYSTIMH_REG);

    psTime_p->tv_sec = sec;
    psTime_p->tv_nsec = nsec;

    return nsec;
}

//------------------------------------------------------------------------------
/**
\brief  Write SYSTIM register of I210

The function writes the SYSTIM register with the specified time.

\param[in]      psTime_p            Pointer to the time to write into the SYSTIM register.
*/
//------------------------------------------------------------------------------
static void writeSystimRegister(const struct timespec* psTime_p)
{
    EDRV_REGDW_WRITE(EDRV_SYSTIML_REG, psTime_p->tv_nsec);
    EDRV_REGDW_WRITE(EDRV_SYSTIMH_REG, psTime_p->tv_sec);
}

//------------------------------------------------------------------------------
/**
\brief  Release a HW semaphore

The function releases an acquired HW semaphore.
*/
//------------------------------------------------------------------------------
static void releaseHwSemaphoreGeneric(void)
{
    UINT32      swsm;

    swsm = EDRV_REGDW_READ(EDRV_SWSM_REG);
    swsm &= ~(EDRV_SWSM_SMBI | EDRV_SWSM_SWESMBI);
    EDRV_REGDW_WRITE(EDRV_SWSM_REG, swsm);
}

//------------------------------------------------------------------------------
/**
\brief  Acquire a HW semaphore

The function acquires the Hardware semaphore to access the PHY registers.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError acquireHwSemaphore(void)
{
    UINT32          swsm;
    tOplkError      result = kErrorOk;
    int             timeout;
    int             index = 0;

    timeout = EDRV_PHY_SEMPHORE_TIMEOUT;
    // Get the FW semaphore.
    for (index = 0; index < timeout; index++)
    {
        swsm = EDRV_REGDW_READ(EDRV_SWSM_REG);
        EDRV_REGDW_WRITE(EDRV_SWSM_REG, swsm | EDRV_SWSM_SWESMBI);
        // Semaphore acquired if bit latched
        if (EDRV_REGDW_READ(EDRV_SWSM_REG) & EDRV_SWSM_SWESMBI)
            break;
        udelay(50);
    }

    if (index == timeout)
    {
        releaseHwSemaphoreGeneric();
        return kErrorNoResource;
    }
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Release a HW semaphore

The function releases the acquired HW semaphore used to access the PHY.
*/
//------------------------------------------------------------------------------
static void releaseHwSemaphore(void)
{
    UINT32       swsm;

    swsm = EDRV_REGDW_READ(EDRV_SWSM_REG);
    swsm &= ~EDRV_SWSM_SWESMBI;
    EDRV_REGDW_WRITE(EDRV_SWSM_REG, swsm);
}

//------------------------------------------------------------------------------
/**
\brief  Acquire SW/FW sync

The function acquires a Software/Firmware sync semaphore to access the PHY.

\param[in]      mask_p              Specifies which semaphore to acquire.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError acquireSwFwSync(UINT16 mask_p)
{
    UINT32      swfwSync;
    UINT32      swmask = mask_p;
    UINT32      fwmask = mask_p << 16;
    tOplkError  result = kErrorOk;
    int         index = 0;
    int         timeout = 200;

    while (index < timeout)
    {
        if (acquireHwSemaphore())
            return kErrorNoResource;

        swfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
        if (!(swfwSync & fwmask))
            break;

        // Firmware currently using resource (fwmask)
        releaseHwSemaphore();
        mdelay(5);
        index++;
    }

    if (index == timeout)
    {
        printk("Driver can't access resource, SW_FW_SYNC timeout.\n");
        return kErrorNoResource;
    }

    swfwSync |= swmask;
    EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, swfwSync);

    releaseHwSemaphore();
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Release SW/FW sync

The function releases an acquired software/firmware semaphore used to access
the PHY.

\param[in]      mask_p              Specifies which semaphore to release.
*/
//------------------------------------------------------------------------------
static void releaseSwFwSync(UINT16 mask_p)
{
    UINT32      swfwSync;

    while (acquireHwSemaphore() != 0)
        ;

    swfwSync = EDRV_REGDW_READ(EDRV_SW_FW_SYNC);
    swfwSync &= ~mask_p;
    EDRV_REGDW_WRITE(EDRV_SW_FW_SYNC, swfwSync);
    releaseHwSemaphore();
}

//------------------------------------------------------------------------------
/**
\brief  Write PHY register via MDIO

The function write into a PHY register via the MDIO interface.

\param[in]      phyreg_p            PHY Register to write.
\param[in]      value_p             Value to write into register.
*/
//------------------------------------------------------------------------------
static void writeMdioPhyReg(UINT phyreg_p, UINT16 value_p)
{
    UINT32      regVal = 0;
    UINT32      mdicRd;

    regVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
    regVal |= value_p;
    regVal |= (phyreg_p << 16);
    regVal |= EDRV_MDIC_OP_WRITE;
    regVal |= EDRV_MDIC_INTR_DIS;
    regVal &= ~EDRV_MDIC_RD;

    EDRV_REGDW_WRITE(EDRV_MDIC_REG, regVal);
    // wait for completion of transfer
    do
    {
        udelay(50);
        mdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);
    } while ((mdicRd & EDRV_MDIC_RD) == 0);          //wait PHYIDLE is set 1
}

//------------------------------------------------------------------------------
/**
\brief  Read PHY register via MDIO

The function reads from a PHY register via the MDIO interface.

\param[in]      phyreg_p            PHY register to read.

\retval The function returns the read register value.
*/
//------------------------------------------------------------------------------
static UINT16 readMdioPhyReg(UINT phyreg_p)
{
    UINT32      regVal = 0;
    UINT16      value = 0;
    UINT32      mdicRd;

    regVal &= ~(EDRV_MDIC_DATA_MASK | EDRV_MDIC_REGADD_MASK);
    regVal |= (phyreg_p << 16);
    regVal |= EDRV_MDIC_OP_READ;
    regVal |= EDRV_MDIC_INTR_DIS;
    regVal &= ~EDRV_MDIC_RD;

    EDRV_REGDW_WRITE(EDRV_MDIC_REG, regVal);

    // wait until the value is read
    do
    {
        cpu_relax();
        mdicRd = EDRV_REGDW_READ(EDRV_MDIC_REG);
    } while ((mdicRd & EDRV_MDIC_RD) == 0);          //wait RD is set 1

    value = (mdicRd & EDRV_MDIC_DATA_MASK);
    return value;
}

//------------------------------------------------------------------------------
/**
\brief  Free TX buffer queue

The function frees the buffers of a TX queue.

\param[in]      pTxQueue_p      TX queue to be processed.
*/
//------------------------------------------------------------------------------
static void freeTxBuffersOfQueue(const tEdrvQueue* pTxQueue_p)
{
    int         index;

    for (index = 0; index < EDRV_MAX_RX_DESCRIPTOR; index++)
    {
        // set report status for all descriptor
        if (pTxQueue_p->pPktBuff[index].dmaAddr)
        {
            dma_unmap_single(&edrvInstance_l.pPciDev->dev,
                             pTxQueue_p->pPktBuff[index].dmaAddr,
                             pTxQueue_p->pPktBuff[index].len, DMA_TO_DEVICE);
        }

    }
}

//------------------------------------------------------------------------------
/**
\brief  Free TX buffers

The function frees all TX buffers.
*/
//------------------------------------------------------------------------------
static void freeTxBuffers(void)
{
    int         index;

    for (index = 0; index < edrvInstance_l.txMaxQueue; index++)
    {
        freeTxBuffersOfQueue(edrvInstance_l.pTxQueue[index]);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Free memory of TX queues

The function frees the memory allocated for the Tx queues.
*/
//------------------------------------------------------------------------------
static void freeTxQueues(void)
{
    tEdrvQueue*     pTxQueue;
    UINT            index;

    for (index = 0; index < edrvInstance_l.txMaxQueue; index++)
    {
        pTxQueue = edrvInstance_l.pTxQueue[index];

        if (pTxQueue == NULL)
            break;

        if (pTxQueue->pBuf != NULL)
            kfree(pTxQueue->pBuf);

        if (pTxQueue->pPktBuff != NULL)
            kfree(pTxQueue->pPktBuff);

        if (pTxQueue->pDescVirt != NULL)
        {
            dma_free_coherent(&edrvInstance_l.pPciDev->dev,
                              EDRV_TX_DESCS_SIZE, pTxQueue->pDescVirt,
                              pTxQueue->descDma);
            kfree(pTxQueue);
            edrvInstance_l.pTxQueue[index] = NULL;
        }
    }
    edrvInstance_l.txMaxQueue = 0;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize TX queue

The function initialize the TX queue. It allocates and sets up the memory for
the TX descriptors.

\param[in,out]  pTxQueue_p          Pointer to the TX queue structure to initialize.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initTxQueue(tEdrvQueue* pTxQueue_p)
{
    int         descSize;

    // Allocate Tx Descriptor
    printk("{%s}:Allocating Tx Desc %p\n", __func__, pTxQueue_p);
    descSize = ALIGN(EDRV_TX_DESCS_SIZE, 4096);
    pTxQueue_p->pDescVirt = dma_alloc_coherent(&edrvInstance_l.pPciDev->dev,
                                               descSize, &pTxQueue_p->descDma, GFP_KERNEL);
    if (pTxQueue_p->pDescVirt == NULL)
        return kErrorEdrvInit;

    printk("... Done\n");

    // Clear the descriptor memory
    memset(pTxQueue_p->pDescVirt, 0, EDRV_TX_DESCS_SIZE);

    pTxQueue_p->pPktBuff = kzalloc((EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvPktBuff)), GFP_KERNEL);
    if (pTxQueue_p->pPktBuff == NULL)
        return kErrorEdrvInit;

    pTxQueue_p->nextDesc = 0;
    pTxQueue_p->nextWb = 0;

    // Map queue to its vector;
    pTxQueue_p->pQvector = edrvInstance_l.pQvector[pTxQueue_p->index];

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Configure a TX queue

The function configure a TX queue.

\param[in]      pTxQueue_p          Pointer to the queue structure to be configured.
*/
//------------------------------------------------------------------------------
static void configureTxQueue(const tEdrvQueue* pTxQueue_p)
{
    UINT64      txDescDma;
    UINT32      reg;
    int         queue = pTxQueue_p->index;
    int         index;

    printk("Configure Tx Queue %d", queue);
    EDRV_REGDW_WRITE(EDRV_TXDCTL(queue), 0);        // Disable the queue
    EDRV_REGDW_READ(EDRV_STATUS_REG);               // Program the hw with queue values
    mdelay(10);

    txDescDma = pTxQueue_p->descDma;

    // Initialize the queue parameters in controller
    EDRV_REGDW_WRITE(EDRV_TDLEN(queue), EDRV_TX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_TDBAL(queue), (txDescDma & 0x00000000ffffffffULL));
    EDRV_REGDW_WRITE(EDRV_TDBAH(queue), (txDescDma >> 32));
    EDRV_REGDW_WRITE(EDRV_TDHEAD(queue), 0);
    EDRV_REGDW_WRITE(EDRV_TDTAIL(queue), 0);

    // Setup the threshold values for queue and enable
    reg = 0;
    reg |= (EDRV_TXDCTL_PTHRESH | (EDRV_TXDCTL_HTHRESH << 8) |
           (EDRV_TXDCTL_WTHRESH << 16));

    // Highest priority to Queue0
    reg |= EDRV_TXDCTL_PRIORITY;
    reg |= EDRV_TXDCTL_QUEUE_EN;

    EDRV_REGDW_WRITE(EDRV_TXDCTL(queue), reg);
    // Poll until queue gets enabled
    printk("Poll TXDCTL");
    for (index = 0; index < EDRV_POLL_TIMEOUT; index++)
    {
        if ((EDRV_REGDW_READ(EDRV_TXDCTL(queue)) & EDRV_TXDCTL_QUEUE_EN))
            break;

        msleep(1);
    }

    if (index == EDRV_POLL_TIMEOUT)
        printk("...Fail\n");
    else
        printk("....Done\n");
}

//------------------------------------------------------------------------------
/**
\brief  Free memory of RX queues

The function frees the memory allocated for the RX queues.
*/
//------------------------------------------------------------------------------
static void freeRxQueues(void)
{
    tEdrvQueue* pRxQueue = NULL;
    UINT        index;

    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        pRxQueue = edrvInstance_l.pRxQueue[index];

        if (pRxQueue == NULL)
            break;

        if (pRxQueue->pBuf != NULL)
            kfree(pRxQueue->pBuf);

        if (pRxQueue->pPktBuff != NULL)
            kfree(pRxQueue->pPktBuff);

        if (pRxQueue->pDescVirt != NULL)
        {
            dma_free_coherent(&edrvInstance_l.pPciDev->dev,
                              EDRV_TX_DESCS_SIZE, pRxQueue->pDescVirt,
                              pRxQueue->descDma);
            kfree(pRxQueue);
            edrvInstance_l.pRxQueue[index] = NULL;
        }
    }
    edrvInstance_l.rxMaxQueue = 0;
}

//------------------------------------------------------------------------------
/**
\brief  Free RX buffer queue

The function frees the buffers of a RX queue.

\param[in]      pRxQueue_p          RX Queue to be processed.
*/
//------------------------------------------------------------------------------
static void freeRxBuffersOfQueue(const tEdrvQueue* pRxQueue_p)
{
    int index;

    for (index = 0; index < EDRV_MAX_RX_DESCRIPTOR; index++)
    {
        // set report status for all descriptor
        dma_unmap_single(&edrvInstance_l.pPciDev->dev,
                         pRxQueue_p->pPktBuff[index].dmaAddr, EDRV_MAX_FRAME_SIZE,
                         DMA_FROM_DEVICE);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Free RX buffers

The function frees all RX buffers.
*/
//------------------------------------------------------------------------------
static void freeRxBuffers(void)
{
    int     index;

    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        freeRxBuffersOfQueue(edrvInstance_l.pRxQueue[index]);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the receive queue

The function initializes the Rx queue with descriptors and memory resources.

\param[in,out]  pRxQueue_p          Pointer to the queue structure to initialize.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initRxQueue(tEdrvQueue* pRxQueue_p)
{
    tOplkError      ret = kErrorOk;
    int             descSize;

    // Allocate Tx Descriptor
    printk("{%s}:Allocating Rx Desc %p\n", __func__, pRxQueue_p);
    descSize = ALIGN(EDRV_RX_DESCS_SIZE, 4096);
    pRxQueue_p->pDescVirt = dma_alloc_coherent(&edrvInstance_l.pPciDev->dev,
                                               descSize, &pRxQueue_p->descDma, GFP_KERNEL);
    if (pRxQueue_p->pDescVirt == NULL)
        return kErrorEdrvInit;

    printk("... Done\n");

    // Clear the descriptor memory
    memset(pRxQueue_p->pDescVirt, 0, EDRV_RX_DESCS_SIZE);

    pRxQueue_p->pPktBuff = kzalloc(EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvPktBuff), GFP_KERNEL);
    if (pRxQueue_p->pPktBuff == NULL)
        return kErrorEdrvInit;

    pRxQueue_p->pBuf = kzalloc(EDRV_RX_BUFFER_SIZE, GFP_KERNEL);

    if (pRxQueue_p->pBuf == NULL)
        return kErrorEdrvInit;

    // Initialize head & tail
    pRxQueue_p->nextDesc = 0;
    pRxQueue_p->nextWb = 0;

    // Map queue to its vector;
    pRxQueue_p->pQvector = edrvInstance_l.pQvector[pRxQueue_p->index];

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate receive buffers

The function allocates receive buffers and associates it with the corresponding
descriptors.

\param[in,out]  pRxQueue_p          Pointer to the queue structure to allocate buffers

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError allocateRxBuffer(tEdrvQueue* pRxQueue_p)
{
    int                 index;
    dma_addr_t          rxDma;
    tOplkError          ret = kErrorOk;
    tEdrvAdvRxDesc*     rxDesc;

    for (index = 0; index < EDRV_MAX_RX_DESCRIPTOR; index++)
    {
        rxDesc = EDRV_GET_RX_DESC(pRxQueue_p, index);

        rxDma = dma_map_single(&edrvInstance_l.pPciDev->dev,
                               (pRxQueue_p->pBuf + (index * EDRV_MAX_FRAME_SIZE)),
                               EDRV_MAX_FRAME_SIZE, DMA_FROM_DEVICE);

        if (dma_mapping_error(&edrvInstance_l.pPciDev->dev, rxDma))
            return kErrorEdrvInit;

        pRxQueue_p->pPktBuff[index].dmaAddr = rxDma;
        pRxQueue_p->pPktBuff[index].pVirtAddr = (pRxQueue_p->pBuf + (index * EDRV_MAX_FRAME_SIZE));
        rxDesc->sRead.bufferAddrLe = cpu_to_le64(rxDma);

        EDRV_REGDW_WRITE(EDRV_RDTAIL(pRxQueue_p->index), index);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure the RX queue

The function configures the receive queue of the I210.

\param[in]      pRxQueue_p          Pointer to the queue structure to be configured.
*/
//------------------------------------------------------------------------------
static void configureRxQueue(const tEdrvQueue* pRxQueue_p)
{
    UINT64      rxDescDma;
    UINT32      reg;
    int         queue = pRxQueue_p->index;
    int         index;

    printk("Configure Rx Queue %d", queue);
    // Disable the Queue
    EDRV_REGDW_WRITE(EDRV_RXDCTL(queue), 0);
    EDRV_REGDW_READ(EDRV_STATUS_REG);
    mdelay(10);

    // Program the hw with queue values
    rxDescDma = pRxQueue_p->descDma;
    EDRV_REGDW_WRITE(EDRV_RDLEN(queue), EDRV_RX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_RDBAL(queue), (rxDescDma & 0x00000000ffffffffULL));
    EDRV_REGDW_WRITE(EDRV_RDBAH(queue), (rxDescDma >> 32));

    EDRV_REGDW_WRITE(EDRV_RDHEAD(queue), 0);
    EDRV_REGDW_WRITE(EDRV_RDTAIL(queue), 0);

    // Configure split-receive register for queue
    reg = EDRV_REGDW_READ(EDRV_SRRCTL(queue));
    //reg |= EDRV_SRRCTL_DROP_EN;
    reg |= EDRV_SRRCTL_DESCTYPE_ADV;          // Advance mode with no packet split
    EDRV_REGDW_WRITE(EDRV_SRRCTL(queue), reg);

    // Enable the rx queue
    reg = 0;
    reg = (EDRV_RXDCTL_PTHRESH | (EDRV_RXDCTL_HTHRESH << 8) |
           (EDRV_RXDCTL_WTHRESH << 16));
    reg |= EDRV_RXDCTL_QUEUE_EN;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(queue), reg);

    printk("Poll RXDCTL");
    for (index = 0; index < EDRV_POLL_TIMEOUT; index++)
    {
        reg = EDRV_REGDW_READ(EDRV_RXDCTL(queue));
        if ((reg & EDRV_RXDCTL_QUEUE_EN))
            break;
        EDRV_REGDW_WRITE(EDRV_RXDCTL(queue), reg);
        msleep(1);
    }

    if (index == EDRV_POLL_TIMEOUT)
        printk("...Fail\n");
    else
        printk("....Done\n");
}

//------------------------------------------------------------------------------
/**
\brief  Initialize Qav mode

The function configures the device for Qav mode using one Tx queue as SR queue
with launch time logic.
*/
//------------------------------------------------------------------------------
static void initQavMode(void)
{
    UINT32 tqavctrl = 0;
    UINT32 launchOff;
    UINT32 tqavcc0 = 0;
    UINT32 txpbsize;
    UINT32 rxpbsize;

    // reconfigure the rx packet buffer allocation to 30k
    rxpbsize = EDRV_REGDW_READ(EDRV_RXPBSIZE_REG);
    rxpbsize &= ~EDRV_RXPBSIZE_CLEAR;
    rxpbsize |= EDRV_RXPBSIZE_DEF;
    EDRV_REGDW_WRITE(EDRV_RXPBSIZE_REG, rxpbsize);

    txpbsize = (10);
    txpbsize |= (10) << EDRV_TXPBSIZE_TX1PB_SHIFT;
    EDRV_REGDW_WRITE(EDRV_TXPBSIZE_REG, txpbsize);


#if (EDRV_USE_TTTX != FALSE)
    tqavctrl = (EDRV_TQAVCTRL_TRANSTIM | EDRV_TQAVCTRL_SP_WAIT_SR);
#endif

    // Q0 is always SR queue
    tqavcc0 = EDRV_TQAVCC_QUEUE_MODE_SR; /* no idle slope */
    EDRV_REGDW_WRITE(EDRV_TQAVCC(0), tqavcc0);

    tqavctrl |= (EDRV_TQAVCTRL_TXMODE | EDRV_TQAVCTRL_FETCH_ARB |
                 EDRV_TQAVCTRL_1588_STAT_EN);
    // default to a 10 usec prefetch delta from launch time
    tqavctrl |= (10 << 5) << EDRV_TQAVCTRL_FETCH_TM_SHIFT;

    EDRV_REGDW_WRITE(EDRV_TQAVCTRL_REG, tqavctrl);

    launchOff = 0;
    launchOff |= (4 << 5) << EDRV_LAUNCH_OSO_SHIFT;

    EDRV_REGDW_WRITE(EDRV_LAUNCH_OSO, launchOff);
}

//------------------------------------------------------------------------------
/**
\brief  Write the IVAR register

The function writes the IVAR register of the I210 for mapping interrupt vectors
of Tx-Rx causes.

\param[in]      vector_p            Vector number to be assigned.
\param[in]      index_p             Index to determine the queue.
\param[in]      offset_p            Offset for the queue.
 */
//------------------------------------------------------------------------------
static void writeIvarRegister(int vector_p, int index_p, int offset_p)
{
    UINT32 ivar = EDRV_REGDW_READ((EDRV_IVAR0_REG + (index_p << 2)));

    // clear any bits that are currently set
    ivar &= ~(0x000000FFUL << offset_p);

    // write vector and valid bit
    ivar |= (vector_p | EDRV_IVAR_VALID) << offset_p;

    EDRV_REGDW_WRITE((EDRV_IVAR0_REG + (index_p << 2)), ivar);
}

//------------------------------------------------------------------------------
/**
\brief  Request interrupts

The function requests the interrupts for the timer and the queue interrupts.
It enables MSI-X and sets up the interrupt vectors.

\return The function returns the error code returned by request_irq().
*/
//------------------------------------------------------------------------------
static int requestMsixIrq(void)
{
    UINT            vector = 0;
    UINT            index = 0;
    UINT32          reg;
    int             ret = 0;
    UINT            txQueue;
    UINT            rxQueue;
    tEdrvQVector*   pQvector;

    // request timer interrupt
    ret = request_irq(edrvInstance_l.pMsixEntry[vector].vector,
                      edrvTimerInterrupt, 0, DRIVER_NAME, edrvInstance_l.pPciDev);
    if (ret != 0)
        return ret;

    vector++;
    // request queue interrupts
    for (index = 0; index < edrvInstance_l.numQVectors; index++, vector++)
    {
        pQvector = edrvInstance_l.pQvector[index];
        ret = request_irq(edrvInstance_l.pMsixEntry[vector].vector,
                          edrvIrqHandler, 0, pQvector->strName, pQvector);
        if (ret != 0)
            return ret;
    }

    // Configure MSI-X
    reg = 0;
    reg = (EDRV_INTR_GPIE_MULT_MSIX | EDRV_INTR_GPIE_PBA | EDRV_INTR_GPIE_NSICR);
    EDRV_REGDW_WRITE(EDRV_INTR_GPIE_REG, reg);
    vector = 0;

    // Enable Msi-x for Other Cause;
    reg = EDRV_REGDW_READ(EDRV_IVAR_MISC);
    reg |= ((vector | EDRV_IVAR_VALID) << 8);
    EDRV_REGDW_WRITE(EDRV_IVAR_MISC, reg);

    vector++;
    // Map Tx0 and Rx0 Interrupt cause
    for (index = 0; index < edrvInstance_l.numQVectors; index++, vector++)
    {
        txQueue = edrvInstance_l.pTxQueue[index]->index;
        writeIvarRegister(vector, txQueue >> 1, ((txQueue & 0x1) << 4) + 8);
        rxQueue = edrvInstance_l.pRxQueue[index]->index;
        writeIvarRegister(vector, rxQueue >> 1, ((rxQueue & 0x1) << 4));
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send a buffer using normal transmission queue

This function queues the frame to be transmitted using normal transmission.
Queue 1 is used for transmitting frames in normal mode.

\param[in,out]  pBuffer_p           Pointer to the Tx buffer to transmit.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendNormalBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError          ret = kErrorOk;
    tEdrvAdvTxDesc*     pTxDesc;
    tEdrvQueue*         pTxQueue;
    int                 queue = 1;
    int                 index = 0;
    dma_addr_t          txDma;

    pTxQueue = edrvInstance_l.pTxQueue[queue];
    index = pTxQueue->nextDesc;

    if (((index + 1) & EDRV_MAX_TX_DESC_LEN) == pTxQueue->nextWb)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    pTxDesc = EDRV_GET_TX_DESC(pTxQueue, index);

    txDma = dma_map_single(&edrvInstance_l.pPciDev->dev, pBuffer_p->pBuffer,
                           pBuffer_p->txFrameSize, DMA_TO_DEVICE);

    if (dma_mapping_error(&edrvInstance_l.pPciDev->dev, txDma))
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // Store TxBuffer for reference in ISR
    pTxQueue->apTxBuffer[index] = pBuffer_p;

    EDRV_COUNT_SEND;
    // Store DMA address, length and virtual address for reference
    pTxQueue->pPktBuff[index].dmaAddr = txDma;
    pTxQueue->pPktBuff[index].pVirtAddr = pBuffer_p->pBuffer;
    pTxQueue->pPktBuff[index].len = pBuffer_p->txFrameSize;

    pTxDesc->sRead.bufferAddrLe = cpu_to_le64(txDma);
    pTxDesc->sRead.cmdTypeLen = (UINT)pBuffer_p->txFrameSize;
    pTxDesc->sRead.cmdTypeLen |= (EDRV_TDESC_CMD_DEXT | EDRV_TDESC_DTYP_ADV |
                                  EDRV_TDESC_CMD_EOP | EDRV_TDESC_CMD_IFCS |
                                  EDRV_TDESC_CMD_RS);

    pTxDesc->sRead.statusIdxPaylen = (pBuffer_p->txFrameSize << 14);

    index = ((index + 1) & EDRV_MAX_TX_DESC_LEN);

    // increment Tx descriptor queue tail pointer
    pTxQueue->nextDesc = index;
    // Handle the frame to Hw
    EDRV_REGDW_WRITE(EDRV_TDTAIL(queue), index);

Exit:
    return ret;
}

#if (EDRV_USE_TTTX != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Send a buffer using time triggered send

This function queues the frame to be transmitted using valid launch time based
transmission. Queue 0 is used to transmit time triggered frames.

\param[in,out]  pBuffer_p           Pointer to the Tx buffer to transmit.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendTimeTrigBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError      ret = kErrorOk;
    tEdrvTtxDesc*   pTtxDesc;
    UINT64          launchTime;
    UINT64          curTime;
    tEdrvQueue*     pTxQueue;
    int             queue = 0;
    int             index = 0;
    dma_addr_t      txDma;

    pTxQueue = edrvInstance_l.pTxQueue[queue];
    index = pTxQueue->nextDesc;

    if (((index + 1) & EDRV_MAX_TTX_DESC_LEN) == pTxQueue->nextWb)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    pTtxDesc = EDRV_GET_TTX_DESC(pTxQueue, index);

    pTtxDesc->ctxtDesc.idxL4lenMss = 0;
    pTtxDesc->ctxtDesc.ipMaclenVlan = 0;
    launchTime = pBuffer_p->launchTime.nanoseconds;

    // Scale the launch time to 32 nsecs unit
    do_div(launchTime, SEC_TO_NSEC);
    curTime = pBuffer_p->launchTime.nanoseconds - (launchTime * SEC_TO_NSEC);
    do_div(curTime, 32);

    pTtxDesc->ctxtDesc.launchTime = curTime;

    // Always reset the launch time
    pBuffer_p->launchTime.nanoseconds = 0;

    // Set descriptor type
    pTtxDesc->ctxtDesc.tucmdType = (EDRV_TDESC_CMD_DEXT | EDRV_TDESC_DTYP_CTXT);

    txDma = dma_map_single(&edrvInstance_l.pPciDev->dev, pBuffer_p->pBuffer,
                           pBuffer_p->txFrameSize, DMA_TO_DEVICE);

    if (dma_mapping_error(&edrvInstance_l.pPciDev->dev, txDma))
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // Store TxBuffer for reference in ISR
    pTxQueue->apTxBuffer[index] = pBuffer_p;

    EDRV_COUNT_SEND;
    // Store DMA address, length and virtual address for reference
    pTxQueue->pPktBuff[index].dmaAddr = txDma;
    pTxQueue->pPktBuff[index].pVirtAddr = pBuffer_p->pBuffer;
    pTxQueue->pPktBuff[index].len = pBuffer_p->txFrameSize;

    pTtxDesc->advDesc.sRead.bufferAddrLe = cpu_to_le64(txDma);
    pTtxDesc->advDesc.sRead.cmdTypeLen = (UINT)pBuffer_p->txFrameSize;
    pTtxDesc->advDesc.sRead.cmdTypeLen |= (EDRV_TDESC_CMD_DEXT | EDRV_TDESC_DTYP_ADV |
                                           EDRV_TDESC_CMD_EOP | EDRV_TDESC_CMD_IFCS |
                                           EDRV_TDESC_CMD_RS);

    pTtxDesc->advDesc.sRead.statusIdxPaylen = (pBuffer_p->txFrameSize << 14);

    index = ((index + 1) & EDRV_MAX_TTX_DESC_LEN);

    // increment Tx descriptor queue tail pointer
    pTxQueue->nextDesc = index;
    // Handle the frame to Hw
    EDRV_REGDW_WRITE(EDRV_TDTAIL(queue), (index * 2));

Exit:
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Initialize one PCI device

This function initializes one PCI device.

\param[in,out]  pPciDev_p           Pointer to corresponding PCI device structure
\param[in]      pId_p               PCI device ID

\return The function returns an integer error code.
\retval 0           Successful
\retval Otherwise   Error
*/
//------------------------------------------------------------------------------
static int initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p)
{
    int             result = 0;
    tEdrvQueue*     pQueue;
    tEdrvQVector*   pVector;
    UINT32          reg;
    int             index;
    int             numVectors;
    struct timespec sysTime;

    if (edrvInstance_l.pPciDev != NULL)
    {
        // Perform a sanity check to avoid insertion of module again
        printk("%s device %s discarded\n", __func__, pci_name(pPciDev_p));
        result = -ENODEV;
        goto Exit;
    }

    // Enable device
    result = pci_enable_device(pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    edrvInstance_l.pPciDev = pPciDev_p;

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev==NULL\n", __func__);
        result = -1;
        goto Exit;
    }

    result = dma_set_mask(&edrvInstance_l.pPciDev->dev, DMA_BIT_MASK(64));
    if (result == 0)
    {
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 33))
        dma_set_coherent_mask(&edrvInstance_l.pPciDev->dev, DMA_BIT_MASK(64));
#else
        pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(64));
#endif
    }
    else
    {
        printk(" Using 32 bit Mask\n");
        result = dma_set_mask(&pPciDev_p->dev, DMA_BIT_MASK(32));
        if (result == 0)
        {
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 33))
            result = dma_set_coherent_mask(&edrvInstance_l.pPciDev->dev,
                                           DMA_BIT_MASK(32));
#else
            result = pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(32));
#endif
            if (result != 0)
            {
                printk("[EPLi210]: No usable DMA configuration available\n");
                goto ExitFail;
            }
        }
    }

    result = pci_request_regions(pPciDev_p, DRIVER_NAME);

    if (result != 0)
    {
        printk("pci_request_regions....Failed\n");
        goto ExitFail;
    }
    pci_set_master(pPciDev_p);

    edrvInstance_l.pIoAddr = ioremap(pci_resource_start(pPciDev_p, 0),
                                     pci_resource_len(pPciDev_p, 0));
    if (edrvInstance_l.pIoAddr == NULL)
    {
        result = -EIO;
        goto ExitFail;
    }
    printk("edrvInstance_l.pIoAddr :0X%p\n", edrvInstance_l.pIoAddr);

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    // Disable the Master
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg |= (EDRV_CTRL_MASTER_DIS);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);

    for (index = EDRV_MASTER_DIS_TIMEOUT; index > 0; index--)
    {
        if ((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_MASTER_EN)== 0)
            break;

        msleep(1);
    }

    if (index == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    EDRV_REGDW_READ(EDRV_STATUS_REG);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg |= (EDRV_CTRL_DEV_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);
    msleep(5);

    index = 0;
    while (index < EDRV_AUTO_READ_DONE_TIMEOUT)
    {
        if (EDRV_REGDW_READ(EDRV_EECD_REG) & EDRV_EECD_AUTO_RD)
            break;

        msleep(1);
        index++;
    }

    if (index == EDRV_AUTO_READ_DONE_TIMEOUT)
    {
        printk("Auto read by HW from NVM has not completed.\n");
        goto ExitFail;
    }

    EDRV_REGDW_WRITE(EDRV_STATUS_REG, EDRV_STATUS_DEV_RST_SET);

    // disable the interrupts
    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);

    // Set device configuration
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg &= ~(EDRV_CTRL_FD | EDRV_CTRL_ILOS | EDRV_CTRL_TFCE | EDRV_CTRL_RFCE);
    reg |= EDRV_CTRL_SLU;
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);

    // Reset the PHY
    //1. Acquire PHY Sw semaphore for PHY
    acquireSwFwSync(EDRV_SWFW_PHY0_SM);
    //2. Set PHY reset bit in CTRL register
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg |= (EDRV_CTRL_PHY_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);
    //3. Wait for 1 ms to complete the effect
    msleep(1);
    //4. Clear the bit
    reg = EDRV_REGDW_READ(EDRV_CTRL_REG);
    reg &= ~(EDRV_CTRL_PHY_RST);
    EDRV_REGDW_WRITE(EDRV_CTRL_REG, reg);
    msleep(10);
    //5. Release semaphore
    releaseSwFwSync(EDRV_SWFW_PHY0_SM);

    // Get Control from hardware
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
    reg |= EDRV_CTRL_EXTN_DRV_LOAD;
    EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG, reg);

    // Set the queue parameters
    edrvInstance_l.txMaxQueue = EDRV_MAX_TX_QUEUES;
    edrvInstance_l.rxMaxQueue = EDRV_MAX_RX_QUEUES;
    edrvInstance_l.numQVectors = EDRV_MAX_QUEUE_VECTOR;

    sysTime = ktime_to_timespec(ktime_get_real());
    writeSystimRegister(&sysTime);

    //Initialize the SYSTIM timer with current system time
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg &= ~(1 << 31);
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    // Clear the statistic register
    reg = EDRV_REGDW_READ(EDRV_STAT_TPT);
    reg = EDRV_REGDW_READ(EDRV_STAT_TPR);
    reg = EDRV_REGDW_READ(EDRV_STAT_GPRC);
    reg = EDRV_REGDW_READ(EDRV_STAT_BPRC);
    reg = EDRV_REGDW_READ(EDRV_STAT_MPRC);

    EDRV_REGDW_WRITE(EDRV_TIPG_REG, EDRV_TIPG_DEF);

    // Setup interrupt capability
    printk("Register MSI-X");
    numVectors = edrvInstance_l.numQVectors + 1;
    edrvInstance_l.pMsixEntry = kcalloc(numVectors, sizeof(struct msix_entry), GFP_KERNEL);
    if (edrvInstance_l.pMsixEntry)
    {
        for (index = 0; index < numVectors; index++)
        {
            edrvInstance_l.pMsixEntry[index].entry = index;
        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
        result = pci_enable_msix_exact(pPciDev_p, edrvInstance_l.pMsixEntry, numVectors);
#else
        result = pci_enable_msix(pPciDev_p, edrvInstance_l.pMsixEntry, numVectors);
#endif
        if (result != 0)
        {
            printk("...Failed\n");
            goto ExitFail;
        }
    }
    else
    {
        goto ExitFail;
    }
    printk("...Done\n");

    // Allocate queue vectors
    for (index = 0; index < edrvInstance_l.numQVectors; index++)
    {
        pVector = kzalloc(sizeof(tEdrvQVector), GFP_KERNEL);
        pVector->queueIdx = index;
        pVector->vector = edrvInstance_l.pMsixEntry[index + 1].vector;
        snprintf(pVector->strName, (sizeof(pVector->strName) - 1),
                 "%s-TxRxQ-%u", DRIVER_NAME, pVector->queueIdx);
        edrvInstance_l.pQvector[index] = pVector;
    }

    // Allocate Tx buffer memory
    edrvInstance_l.pTxBuf = kzalloc(EDRV_TX_BUFFER_SIZE, GFP_KERNEL);
    if (edrvInstance_l.pTxBuf == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    for (index = 0; index < edrvInstance_l.txMaxQueue; index++)
    {
        pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
        if (pQueue == NULL)
        {
            goto ExitFail;
        }
        pQueue->index = index;
        edrvInstance_l.pTxQueue[index] = pQueue;
    }

    for (index = 0; index < edrvInstance_l.txMaxQueue; index++)
    {
        result = initTxQueue(edrvInstance_l.pTxQueue[index]);
        if (result != kErrorOk)
        {
            goto ExitFail;
        }
    }

    // check if user specified a MAC address
    if ((edrvInstance_l.initParam.aMacAddr[0] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[1] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[2] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[3] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[4] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[5] != 0))
    {
        // write specified MAC address to controller
        reg = 0;
        EDRV_REGDW_WRITE(EDRV_RAH(0), reg);          // disable Entry
        reg |= edrvInstance_l.initParam.aMacAddr[0] << 0;
        reg |= edrvInstance_l.initParam.aMacAddr[1] << 8;
        reg |= edrvInstance_l.initParam.aMacAddr[2] << 16;
        reg |= edrvInstance_l.initParam.aMacAddr[3] << 24;
        EDRV_REGDW_WRITE(EDRV_RAL(0), reg);
        reg = 0;
        reg |= edrvInstance_l.initParam.aMacAddr[4] << 0;
        reg |= edrvInstance_l.initParam.aMacAddr[5] << 8;
        reg |= EDRV_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_RAH(0), reg);
    }
    else
    {   // read MAC address from controller
        reg = EDRV_REGDW_READ(EDRV_RAL(0));
        edrvInstance_l.initParam.aMacAddr[0] = (reg >> 0) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[1] = (reg >> 8) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[2] = (reg >> 16) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[3] = (reg >> 24) & 0xFF;
        reg = EDRV_REGDW_READ(EDRV_RAH(0));
        edrvInstance_l.initParam.aMacAddr[4] = (reg >> 0) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[5] = (reg >> 8) & 0xFF;
    }

    // initialize Multicast Table Array to 0
    for (index = 0; index < 128; index++)
    {
        EDRV_REGDW_WRITE(EDRV_MTA(index), 0);
    }

    // Alloc Rx queue here
    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        pQueue = kzalloc(sizeof(tEdrvQueue), GFP_KERNEL);
        if (pQueue == NULL)
        {
            goto ExitFail;
        }
        pQueue->index = index;
        edrvInstance_l.pRxQueue[index] = pQueue;
    }

    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        result = initRxQueue(edrvInstance_l.pRxQueue[index]);
        if (result != kErrorOk)
        {
            goto ExitFail;
        }
    }

    // Configure device for Qav mode
    initQavMode();

    // DMA configuration
    EDRV_REGDW_WRITE(EDRV_DTX_MAX_PKTSZ_REG, EDRV_MAX_FRAME_SIZE / 64);

    // Setup Tx Configuration
    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), 0);          // Disable Q0 before proceeding
    EDRV_REGDW_WRITE(EDRV_TXDCTL(1), 0);          // Disable Q1 before proceeding
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    reg &= ~(EDRV_TCTL_CLEAR_CT | EDRV_TCTL_RTLC);
    reg |= (EDRV_TCTL_PSP);
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, reg);

    // Set the default collision threshold
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TCTL_EXT_REG);
    reg &= ~EDRV_TCTL_EXT_COLD_CLEAR;
    reg |= EDRV_TCTL_EXT_COLD;
    EDRV_REGDW_WRITE(EDRV_TCTL_EXT_REG, reg);

    // Enable the Tx
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    reg |= EDRV_TCTL_EN;
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, reg);

    for (index = 0; index < edrvInstance_l.txMaxQueue; index++)
    {
        configureTxQueue(edrvInstance_l.pTxQueue[index]);
    }

    // Disable 1st Rx Queue
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), 0);

    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_RCTL_REG);
    reg &= ~(3 << EDRV_RCTL_MO_SHIFT);
    reg &= ~(3 << EDRV_RCTL_BSIZE_OFFSET);
    reg &= ~EDRV_RCTL_LBM_CLEAR;
    reg |= (EDRV_RCTL_EN | EDRV_RCTL_BAM | EDRV_RCTL_SECRC | EDRV_RCTL_LPE);
    // Receive all packets
#ifdef PROMISCUOUS_MODE
    reg |= (EDRV_RCTL_UPE | EDRV_RCTL_MPE);
#endif

    // Enable receive
    EDRV_REGDW_WRITE(EDRV_RCTL_REG, reg);

    // configure Rx queues
    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        configureRxQueue(edrvInstance_l.pRxQueue[index]);
    }

    // Allocate Rx buffers
    for (index = 0; index < edrvInstance_l.rxMaxQueue; index++)
    {
        result = allocateRxBuffer(edrvInstance_l.pRxQueue[index]);
        if (result != kErrorOk)
        {
            goto ExitFail;
        }
    }

    printk("Requesting Interrupt");
    //Request MSI-X
    result = requestMsixIrq();
    if (result != 0)
    {
        printk("... Failed\n");
        goto ExitFail;
    }
    printk("...Done\n");

    // enable interrupts
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_SET_READ, (EDRV_INTR_ICR_TIME_SYNC));

    reg = EDRV_REGDW_READ(EDRV_EXT_INTR_MASK_SET);
    reg |= (EDRV_EICS_TXRXQUEUE1 | EDRV_EICS_TXRXQUEUE2 | EDRV_EICS_OTHER);

    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_SET, reg);

    reg = EDRV_REGDW_READ(EDRV_INTR_EIAC);
    reg |= (EDRV_EICS_TXRXQUEUE1 | EDRV_EICS_TXRXQUEUE2 | EDRV_EICS_OTHER);

    EDRV_REGDW_WRITE(EDRV_INTR_EIAC, reg);

    printk("%s waiting for link up...", __func__);
    for (index = EDRV_LINK_UP_TIMEOUT; index > 0; index -= 100)
    {
        if ((EDRV_REGDW_READ(EDRV_STATUS_REG) & EDRV_STATUS_LU))
        {
            printk("Link Up\n");
            reg = EDRV_REGDW_READ(EDRV_STATUS_REG);
            break;
        }
        msleep(100);
    }

    if (index == 0)
    {
        printk("Link Down\n");
        result = -EIO;
        goto ExitFail;
    }

    edrvInstance_l.fInitialized = TRUE;
    goto Exit;


ExitFail:
    removeOnePciDev(pPciDev_p);

Exit:
    printk("%s finished with %d\n", __func__, result);
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Remove one PCI device

This function removes one PCI device.

\param[in,out]  pPciDev_p           Pointer to corresponding PCI device structure
*/
//------------------------------------------------------------------------------
static void removeOnePciDev(struct pci_dev* pPciDev_p)
{
    UINT32          reg;
    int             index;
    UINT16          wReg;
    int             vector;
    tEdrvQVector*   pVector;

    edrvInstance_l.fInitialized = FALSE;

    if (pPciDev_p != edrvInstance_l.pPciDev)
    {
        BUG_ON(edrvInstance_l.pPciDev != pPciDev_p);
        goto Exit;
    }

    EDRV_REGDW_WRITE(EDRV_EXT_INTR_MASK_CLEAR, EDRV_EIMC_CLEAR_ALL);
    EDRV_REGDW_WRITE(EDRV_INTR_MASK_CLEAR, ~0);
    EDRV_REGDW_WRITE(EDRV_INTR_ACK_AUTO_MASK, 0);
    reg = EDRV_REGDW_READ(EDRV_INTR_READ_REG);

    // Stop the timers
    reg = 0;
    reg = EDRV_REGDW_READ(EDRV_TSAUXC);
    reg &= ~(EDRV_TSAUXC_EN_TT(0) | EDRV_TSAUXC_EN_TT(1));
    EDRV_REGDW_WRITE(EDRV_TSAUXC, reg);

    if (edrvInstance_l.pMsixEntry)
    {
        vector = 0;
        free_irq(edrvInstance_l.pMsixEntry[vector].vector, pPciDev_p);
        vector++;
        for (index = 0; index < edrvInstance_l.numQVectors; index++)
        {
            free_irq(edrvInstance_l.pMsixEntry[vector].vector,
                     edrvInstance_l.pQvector[index]);
            vector++;
        }
        pci_disable_msix(pPciDev_p);
        kfree(edrvInstance_l.pMsixEntry);
    }

    reg = EDRV_REGDW_READ(EDRV_TXDCTL(0));
    reg |= EDRV_TXDCTL_SWFLSH;
    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), reg);

    //Disable Rx
    reg = EDRV_REGDW_READ(EDRV_RXDCTL(0));
    reg |= EDRV_RXDCTL_SWFLUSH;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), reg);

    EDRV_REGDW_READ(EDRV_STATUS_REG);
    msleep(10);

    reg = EDRV_REGDW_READ(EDRV_TXDCTL(0));
    reg &= ~EDRV_TXDCTL_SWFLSH;
    EDRV_REGDW_WRITE(EDRV_TXDCTL(0), reg);

    reg = EDRV_REGDW_READ(EDRV_RXDCTL(0));
    reg &= ~EDRV_RXDCTL_SWFLUSH;
    EDRV_REGDW_WRITE(EDRV_RXDCTL(0), reg);

    // Disable Tx
    reg = EDRV_REGDW_READ(EDRV_TCTL_REG);
    reg &= ~EDRV_TCTL_EN;
    EDRV_REGDW_WRITE(EDRV_TCTL_REG, reg);

    //Disable Rx
    reg = EDRV_REGDW_READ(EDRV_RCTL_REG);
    reg &= ~EDRV_RCTL_EN;
    EDRV_REGDW_WRITE(EDRV_RCTL_REG, reg);

    freeTxBuffers();
    freeRxBuffers();

    freeTxQueues();
    freeRxQueues();

    if (edrvInstance_l.pTxBuf != NULL)
    {
        kfree(edrvInstance_l.pTxBuf);
    }

    // Power down PHY
    wReg = readMdioPhyReg(PHY_I210_COPPER_SPEC);
    wReg |= PHY_I210_CS_POWER_DOWN;
    writeMdioPhyReg(PHY_I210_COPPER_SPEC, wReg);

    wReg = readMdioPhyReg(PHY_CONTROL_REG_OFFSET);
    wReg |= PHY_CONTROL_POWER_DOWN;
    writeMdioPhyReg(PHY_CONTROL_REG_OFFSET, wReg);
    msleep(1);

    // Release the control to hardware
    reg = EDRV_REGDW_READ(EDRV_CTRL_EXTN_REG);
    reg &= ~EDRV_CTRL_EXTN_DRV_LOAD;
    EDRV_REGDW_WRITE(EDRV_CTRL_EXTN_REG, reg);

    // Free queue vectors
    for (index = 0; index < edrvInstance_l.numQVectors; index++)
    {
        pVector = edrvInstance_l.pQvector[index];
        kfree(pVector);
        edrvInstance_l.pQvector[index] = NULL;
    }
    edrvInstance_l.numQVectors = 0;

    // unmap controller's register space
    if (edrvInstance_l.pIoAddr != NULL)
    {
        iounmap(edrvInstance_l.pIoAddr);
        edrvInstance_l.pIoAddr = NULL;
    }

    // Release memory regions
    pci_release_regions(pPciDev_p);

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    edrvInstance_l.pPciDev = NULL;

Exit:
    return;
}

/// \}
