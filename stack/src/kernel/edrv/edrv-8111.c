/**
********************************************************************************
\file   edrv-8111.c

\brief  Implementation of Ethernet driver module

This file contains the implementation of the Ethernet driver module for
Realtek 8111/8168.

\ingroup module_edrv
*******************************************************************************/

/* ------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
------------------------------------------------------------------------------ */

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/ami.h>
#include <common/bufalloc.h>
#include <kernel/edrv.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gfp.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26))
#include <linux/semaphore.h>
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
#error "Linux Kernel versions older 2.6.19 are not supported by this driver!"
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define DRV_NAME                                        "plk"                                       // driver name to be used by linux

// Buffer and descriptor configurations
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS                             256                                         // max no. of tx buffers to be allocated
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS                               64                                          // max no. of tx descriptors to be used
#define EDRV_TX_DESC_MASK                               (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS                             256                                         // max no. of rx buffers to be allocated
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS                               16                                          // max no. of rx descriptors to be used
#define EDRV_RX_DESC_MASK                               (EDRV_MAX_RX_DESCS - 1)
#endif

#if EDRV_MAX_RX_DESCS > EDRV_MAX_RX_BUFFERS
#error "Number of Rx Buffers should be more than the number of descriptors used!!"
#endif

#define EDRV_MAX_FRAME_SIZE                             0x600                                       // max size of a single frame used for tx/rx in bytes - 1536
#define EDRV_TX_BUFFER_SIZE                             (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define EDRV_TX_DESCS_SIZE                              (EDRV_MAX_TX_DESCS * sizeof(tEdrvTxDesc))
#define EDRV_RX_BUFFER_SIZE_SHIFT                       11                                          // rx buffer size to be 2048 Byte
#define EDRV_RX_BUFFER_SIZE                             (1 << EDRV_RX_BUFFER_SIZE_SHIFT)
#define EDRV_RX_DESCS_SIZE                              (EDRV_MAX_RX_DESCS * sizeof(tEdrvRxDesc))

// time-outs and sample counts etc.
#define EDRV_HASH_BITS                                  6                                           // used bits in hash
#define EDRV_CRC32_POLY                                 0x04C11DB6                                  // hash generation polynomial
#define EDRV_DIAG_HISTORY_COUNT                         14                                          // diagnostics history count
#define EDRV_SAMPLE_NUM                                 10000                                       // no. of samples used by diagnostics for mean calculation
#define EDRV_AUTO_READ_DONE_TIMEOUT                     10                                          // in ms, max wait time for Phy read and write
#define EDRV_MASTER_DISABLE_TIMEOUT                     90                                          // in ms, max wait time for MAC reset
#define EDRV_LINK_UP_TIMEOUT                            3000                                        // in ms, max wait time for Phy reset
#define EDRV_DIAG_DUMP_TIMEOUT                          10                                          // in ms, max wait time for dump tally counter to dump data
#define EDRV_PHY_UP_WAIT_TIME                           500                                         // in ms, max wait time for Phy to be up
#define EDRV_PHY_LINK_CHANGE_WAIT_TIME                  20                                          // in ms, wait time for Phy state change

// Register read write functions
#define EDRV_REGDW_WRITE(reg, val)                      writel((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGW_WRITE(reg, val)                       writew((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGB_WRITE(reg, val)                       writeb((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGDW_READ(reg)                            readl((UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGW_READ(reg)                             readw((UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGB_READ(reg)                             readb((UINT8*)edrvInstance_l.pIoAddr + (reg))

#define DW_LO(qw)                                       ((UINT64)qw & 0xFFFFFFFF)   // higher 32 bits of 64 bit
#define DW_HI(qw)                                       ((UINT64)qw >> 32)          // lower 32 bits of 64 bit

// Tx Descriptor frame configuration offsets
#define TX_DESC_FRAME_LENGTH_MASK                       0x0000FFFF      // frame length indication bit-mask in Tx Descriptor
#define TX_DESC_FLAG_TCPCS                              0x00010000      // TCP checksum offload enable
#define TX_DESC_FLAG_UDPCS                              0x00020000      // UDP checksum offload enable
#define TX_DESC_FLAG_IPCS                               0x00040000      // IP checksum offload enable
// Rsvd 8 bits - 0x07F80000
#define TX_DESC_FLAG_LGSEN                              0x08000000      // TCP/IP large-send option enable
#define TX_DESC_FLAG_LS                                 0x10000000      // flag for last-segment of the tx packet
#define TX_DESC_FLAG_FS                                 0x20000000      // flag for first-segment of the tx packet
#define TX_DESC_FLAG_EOR                                0x40000000      // flag for end-of-descriptor-ring
#define TX_DESC_FLAG_OWN                                0x80000000      // flag for indicating owner of the descriptor (1- MAC, 0- CPU)

/* Rx Descriptor frame configuration offsets */
#define RX_DESC_FRAME_LENGTH_MASK                       0x00003FFF      // frame length indication bit mask in Rx Descriptor
#define RX_DESC_FLAG_TCPF                               0x00004000      // TCP checksum failure
#define RX_DESC_FLAG_UDPF                               0x00008000      // UDP checksum failure
#define RX_DESC_FLAG_IPF                                0x00010000      // IP checksum failure
#define RX_DESC_FLAG_PIDO                               0x00020000      // protocol ID field 0  ( 00 - Non-IP, 01-TCP/IP
#define RX_DESC_FLAG_PIDI                               0x00040000      // protocol ID field 1  ( 10 - UDP/IP, 11 - IP
#define RX_DESC_FLAG_CRC                                0x00080000      // CRC error
#define RX_DESC_FLAG_RUNT                               0x00100000      // packet received is of length < 64 bytes
#define RX_DESC_FLAG_RES                                0x00200000      // receive error summary flag- 1 if any rx error bit is set
#define RX_DESC_FLAG_RWT                                0x00400000      // receive watch-dog timer expire- packet length is > 8192 bytes
// Rsvd 2bits - 0x01800000
#define RX_DESC_FLAG_BAR                                0x02000000      // flags when a Broadcast address is received
#define RX_DESC_FLAG_PAM                                0x04000000      // flags when a matched physical address is received
#define RX_DESC_FLAG_MAR                                0x08000000      // flags when a multicast address is received
#define RX_DESC_FLAG_LS                                 0x10000000      // flag for the last segment of the rx packet
#define RX_DESC_FLAG_FS                                 0x20000000      // flag for the first segment of the rx packet
#define RX_DESC_FLAG_EOR                                0x40000000      // flag for indicating end-of-descriptor-ring
#define RX_DESC_FLAG_OWN                                0x80000000      // flag for indicating owner of the descriptor (1- MAC, 0- CPU)

/* Tx and Rx Descriptor VLAN_TAG configuration bit offsets */
#define DESC_VLAN_TAG                                   0x0000FFFF
#define DESC_VLAN_TAG_VIDH                              0x0000000F
#define DESC_VLAN_TAG_CFI                               0x00000010
#define DESC_VLAN_TAG_PRIO                              0x000000E0
#define DESC_VLAN_TAG_VIDL                              0x0000FF00

/* 8168B/ 8111B specific Configurations */
#define R8111_REGS_SIZE                                 256             // IO mapped register block sizes

/* 8168B/ 8111B MAC/Phy register offsets and configuration offsets */

// MAC Address Registers
#define RW_REGDW_MAC_ADDR_BANK_LO                       0x00            // lower 4-byte address offset of MAC-ID registers
#define RW_REGDW_MAC_ADDR_BANK_HI                       0x04            // higher 4-byte address offset of MAC-ID registers
#define RW_REGDW_MAC_ADDR_BANK_0                        0x00            // MAC ID register 1
#define RW_REGDW_MAC_ADDR_BANK_1                        0x01            // MAC ID register 2
#define RW_REGDW_MAC_ADDR_BANK_2                        0x02            // MAC ID register 3
#define RW_REGDW_MAC_ADDR_BANK_3                        0x03            // MAC ID register 4
#define RW_REGDW_MAC_ADDR_BANK_4                        0x04            // MAC ID register 5
#define RW_REGDW_MAC_ADDR_BANK_5                        0x05            // MAC ID register 6

// Multicast Address Registers
#define RW_REGDW_MULTICAST_ADDR_BANK_LO                 0x08            // lower 4-byte address offset of multicast-address registers
#define RW_REGDW_MULTICAST_ADDR_BANK_HI                 0x0C            // higher 4-byte address offset of multicast-address registers
#define RW_REGDW_MULTICAST_ADDR_BANK_0                  0x0F
#define RW_REGDW_MULTICAST_ADDR_BANK_1                  0x0E
#define RW_REGDW_MULTICAST_ADDR_BANK_2                  0x0D
#define RW_REGDW_MULTICAST_ADDR_BANK_3                  0x0C
#define RW_REGDW_MULTICAST_ADDR_BANK_4                  0x0B
#define RW_REGDW_MULTICAST_ADDR_BANK_5                  0x0A
#define RW_REGDW_MULTICAST_ADDR_BANK_6                  0x09
#define RW_REGDW_MULTICAST_ADDR_BANK_7                  0x08

// C+ Command Register
#define RW_REGW_CPLUS_COMMAND                           0x00E0          // Cplus command register
#define RW_REGW_CPLUS_COMMAND_RX_CHKSUM_OFLD_EN         0x20            // rx checksum offload enable (1- MAC handles cheksum offload)
#define RW_REGW_CPLUS_COMMAND_RX_VLAN_DETAG_EN          0x40            // rx vlan-tag detag enable (1- MAC handles the detagging)

// Command Register
#define RW_REGB_COMMAND                                 0x0037          // MAC command register
#define RW_REGB_COMMAND_RST                             0x10            // MAC reset auto-clear bit(1- resets the mac, auto-clears after done)
#define RW_REGB_COMMAND_RX_EN                           0x08            // rx enable bit (has to be enabled before configuration
#define RW_REGB_COMMAND_TX_EN                           0x04            // tx enable bit (in the RCR or TCR registers

// 93C46 (93C56) Command
#define RW_REGB_93C46_CMD                               0x50            // 9346CR: 93C46 (93C56) command register address offset
#define RW_REGB_93C46_CMD_OP_MODE_CFG_MASK              0xC0            // MAC operation mode selection bit-mask(00 - normal mode)

// default configurations
#define RW_REGB_93C46_CMD_OP_MODE_NORM                  0x00            // normal mode of MAC operation
#define RW_REGB_93C46_CMD_OP_MODE_AUTO_LD               0x40            // Auto-load mode of MAC operation
#define RW_REGB_93C46_CMD_OP_MODE_PGMM                  0x80            // 93C46 (93C56) programming mode - enable direct access to 93C46 (93C56)
#define RW_REGB_93C46_CMD_OP_MODE_CONFIG                0xC0            // Config register write enable mode - unlock the configuration registers

// Maximum Tx Packet Size
#define RW_REGB_TX_MAX_PKT_SIZE                         0x00EC          // max-tx-packet-length configuration register address offset
#define RW_REGB_TX_MAX_PKT_SIZE_MASK                    0x003F          // size indication mask-bits
#define RW_REGB_TX_MAX_PKT_SIZE_UNIT                    128             // unit(granularity) of size in the above field, i.e. the mltiplican factor

// Transmit Configuration Register
#define RW_REGDW_TCR                                    0x00000040      // transmit configuration register address offset
#define RW_REGDW_TCR_MAX_DMA_BURST_SIZE                 0x00000700      // maximum dma burst size per tx dma burst
#define RW_REGDW_TCR_TX_CRC_DIS                         0x00010000      // bit to disable CRC append by MAC (0 - enable)
#define RW_REGDW_TCR_LOOP_BACK_MODE                     0x00060000      // enable tx loop-back in MAC(not dependent on Phy)
#define RW_REGDW_TCR_IFG_TIME_LO                        0x03000000      // inter-frame-gap time config bits 1 & 0 (011- default 960ns
#define RW_REGDW_TCR_IFG_TIME_HI                        0x00080000      // inter-frame-gap time config bit 2      (@ 100MHz
#define R_REGDW_TCR_HW_VER_ID0                          0x30000000      // MAC hardware version ID flag 0
#define R_REGDW_TCR_HW_VER_ID0_FIELD_LO                 0x10000000      // MAC hardware version ID flag 0 bit 0
#define R_REGDW_TCR_HW_VER_ID0_FIELD_HI                 0x20000000      // MAC hardware version ID flag 0 bit 1
#define R_REGDW_TCR_HW_VER_ID1                          0x04000000      // MAC hardware version ID bit 1
#define R_REGDW_TCR_HW_VER_ID2                          0x00800000      // MAC hardware version ID bit 2

// default configurations
#define REGDW_TCR_VER_MASK                              (R_REGDW_TCR_HW_VER_ID0 | \
                                                        R_REGDW_TCR_HW_VER_ID1 | \
                                                        R_REGDW_TCR_HW_VER_ID2)

#define REGDW_TCR_VER_8111B_8168B                       (R_REGDW_TCR_HW_VER_ID0 & \
                                                        ~R_REGDW_TCR_HW_VER_ID1 & \
                                                        ~R_REGDW_TCR_HW_VER_ID2)

#define REGDW_TCR_VER_8100E                             ((R_REGDW_TCR_HW_VER_ID0 & \
                                                         ~R_REGDW_TCR_HW_VER_ID1) | \
                                                        R_REGDW_TCR_HW_VER_ID2)

#define REGDW_TCR_VER_8101E                             (R_REGDW_TCR_HW_VER_ID0 | \
                                                        (R_REGDW_TCR_HW_VER_ID1 & \
                                                         ~R_REGDW_TCR_HW_VER_ID2))

#define REGDW_TCR_VER_8111C                             (~R_REGDW_TCR_HW_VER_ID0 & \
                                                        R_REGDW_TCR_HW_VER_ID1 & \
                                                        ~R_REGDW_TCR_HW_VER_ID2)

#define REGDW_TCR_VER_8111D                             (R_REGDW_TCR_HW_VER_ID0_FIELD_HI & \
                                                        ~R_REGDW_TCR_HW_VER_ID0_FIELD_LO & \
                                                        ~R_REGDW_TCR_HW_VER_ID1 & \
                                                        ~R_REGDW_TCR_HW_VER_ID2)

#define REGDW_TCR_IFG_960                               (~RW_REGDW_TCR_IFG_TIME_HI & \
                                                        RW_REGDW_TCR_IFG_TIME_LO)

#define RW_REGDW_TCR_MAX_DMA_BURST_SIZE_UNLMTD          RW_REGDW_TCR_MAX_DMA_BURST_SIZE

#define RW_REGDW_TCR_DEF                                ((RW_REGDW_TCR_MAX_DMA_BURST_SIZE_UNLMTD | \
                                                         REGDW_TCR_IFG_960) & \
                                                        ~RW_REGDW_TCR_TX_CRC_DIS)

// Transmit Priority Polling
#define RW_REGB_TX_PRIO_POLL                            0x0038          // tx priority polling register address offset
#define RW_REGB_TX_PRIO_POLL_FORCE_SW_INT               0x01            // force a sw interrupt on MAC (dep.- RW_REGW_INT_MASK_SW)
#define RW_REGB_TX_PRIO_POLL_NO_PRIO_SEND               0x40            // force MAC to poll for a pending tx in normal prio tx queue
#define RW_REGB_TX_PRIO_POLL_HI_PRIO_SEND               0x80            // force MAC to poll for a pending tx in high prio tx queue

// Transmit-High_Priority Descriptor Start Address Register
#define RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_HI          0x002C
#define RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_LO          0x0028

// Transmit-Normal_Priority Descriptor Start Address Register
#define RW_REGDW_TX_NO_PRIO_DESC_START_ADDR_HI          0x0024
#define RW_REGDW_TX_NO_PRIO_DESC_START_ADDR_LO          0x0020

// Rx Maximum packet size
#define RW_REGW_RX_MAX_PKT_SIZE                         0x00DA          // rx-max-packet-size register address offset
#define RW_REGW_RX_MAX_PKT_SIZE_MASK                    0x3FFF          // rx-max-packet-size indication bit-mask

// Receive Descriptor Start Address Register
#define RW_REGDW_RX_DESC_START_ADDR_HI                  0x00E8
#define RW_REGDW_RX_DESC_START_ADDR_LO                  0x00E4

// Receive Configuration Register
#define RW_REGDW_RCR                                    0x00000044      // receive-config register address offset (dep.- RW_REGB_COMMAND_RX_EN)
#define RW_REGDW_RCR_ACC_ALL_PKTS                       0x01            // promiscuous mode- accept all packets from the Phy
#define RW_REGDW_RCR_ACC_UNICAST_PKTS                   0x02            // accept only the packets whose destination address matches
#define RW_REGDW_RCR_ACC_MULTICAST_PKTS                 0x04            // accept multicast packets whose dest. group addr matches
#define RW_REGDW_RCR_ACC_BROADCAST_PKTS                 0x08            // accept Broadcast packets
#define RW_REGDW_RCR_ACC_RUNT                           0x10            // accept packets having size 8B <> 64B
#define RW_REGDW_RCR_ACC_ERR_PKTS                       0x20            // accept corrupted packets
#define R_REGDW_RCR_EEPROM_SEL                          0x40            // reflects the eeprom used(readonly: 0- 9346, 1- 9356)
#define RW_REGDW_RCR_MAX_DMA_BURST_SIZE                 0x0700          // max rx dma burst size per rx dma burst
#define RW_REGDW_RCR_RX_FIFO_THRSHLD                    0xE000          // bit-mask for the rx FIFO threshold

// default configurations
#define RW_REGDW_RCR_RX_FIFO_THRSHOLD_UNLMTD            RW_REGDW_RCR_RX_FIFO_THRSHLD
#define RW_REGDW_RCR_MAX_DMA_BURST_SIZE_UNLMTD          RW_REGDW_RCR_MAX_DMA_BURST_SIZE
#define RW_REGDW_RCR_DEF                                (RW_REGDW_RCR_RX_FIFO_THRSHOLD_UNLMTD |   \
                                                         RW_REGDW_RCR_MAX_DMA_BURST_SIZE_UNLMTD | \
                                                         RW_REGDW_RCR_ACC_UNICAST_PKTS |          \
                                                         RW_REGDW_RCR_ACC_MULTICAST_PKTS |        \
                                                         RW_REGDW_RCR_ACC_BROADCAST_PKTS /* for arp */)

// Dump tally Counter command (DTCCR)
#define RW_REGQW_DUMP_TALLY_COUNT                       0x10                // dump-tally-counter-command register address offset
#define RW_REGQW_DUMP_TALLY_COUNT_CMD                   0x08                // command bit for the dump-tally-counter (1- dump counter data to the addr)
#define RW_REGQW_DUMP_TALLY_COUNT_START_ADDR_MASK       0xFFFFFFFFFFFFFFC0  // 64 byte alignment, 64-byte long address
#define RW_REGQW_DUMP_TALLY_COUNT_START_ADDR_MASK_LO    0x00000000FFFFFFC0  // bit-mask for lower 32-bit of dump-tally-counter start address
#define RW_REGQW_DUMP_TALLY_COUNT_TX_OK_OFSET           0x00                // tx-ok-counter offset from the start address
#define RW_REGQW_DUMP_TALLY_COUNT_RX_OK_OFSET           0x08
#define RW_REGQW_DUMP_TALLY_COUNT_TX_ERR                0x16
#define RW_REGQW_DUMP_TALLY_COUNT_RX_ERR                0x24
#define RW_REGQW_DUMP_TALLY_COUNT_MISS_PKT              0x28
#define RW_REGQW_DUMP_TALLY_COUNT_ALIGN_ERR             0x30
#define RW_REGQW_DUMP_TALLY_COUNT_TX_1_COL              0x32
#define RW_REGQW_DUMP_TALLY_COUNT_RX_1_COL              0x36
#define RW_REGQW_DUMP_TALLY_COUNT_RX_OK_UNICAST         0x40
#define RW_REGQW_DUMP_TALLY_COUNT_RX_OK_BROADCAST       0x48
#define RW_REGQW_DUMP_TALLY_COUNT_RX_OK_MULTICAST       0x56
#define RW_REGQW_DUMP_TALLY_COUNT_TX_ABORT              0x60
#define RW_REGQW_DUMP_TALLY_COUNT_TX_UNDER_RUN          0x62
#define RW_REGQW_DUMP_TALLY_COUNT_LO                    0x0010          // DTCCR address offset for lower 32 bit
#define RW_REGQW_DUMP_TALLY_COUNT_HI                    0x0014          // DTCCR address offset for higher 32 bit

// Interrupt Mask Register
#define RW_REGW_INT_MASK                                0x003C          // interrupt mask register address offset
#define RW_REGW_INT_MASK_RX_OK                          0x0001          // enable rx ok interrupt
#define RW_REGW_INT_MASK_RX_ERR                         0x0002          // enable error in rx interrupt
#define RW_REGW_INT_MASK_TX_OK                          0x0004          // enable tx ok interrupt
#define RW_REGW_INT_MASK_TX_ERR                         0x0008          // enable error in tx interrupt
#define RW_REGW_INT_MASK_RX_DESC_UNAVLBL                0x0010          // enable rx descriptor unavailable interrupt
#define RW_REGW_INT_MASK_LINK_CHNG                      0x0020          // enable Phy link status change interrupt
#define RW_REGW_INT_MASK_RX_FIFO_OV                     0x0040          // enable rx FIFO overflow interrupt
#define RW_REGW_INT_MASK_TX_DESC_UNAVLBL                0x0080          // enable tx descriptor unavailable interrupt
#define RW_REGW_INT_MASK_SW                             0x0100          // enable software interrupt
#define RW_REGW_INT_MASK_RX_FIFO_EMPTY                  0x0200          // enable interrupt for rx FIFO empty after a rx FIFO full status indication
#define RW_REGW_INT_MASK_TIMEOUT                        0x4000          // enable interrupt for timeout in TCTR

// default configurations
#define RW_REGW_INT_MASK_DEF                            (RW_REGW_INT_MASK_TX_OK |      \
                                                         RW_REGW_INT_MASK_TX_ERR |     \
                                                         RW_REGW_INT_MASK_LINK_CHNG |  \
                                                         RW_REGW_INT_MASK_RX_OK |      \
                                                         RW_REGW_INT_MASK_RX_ERR |     \
                                                         RW_REGW_INT_MASK_RX_FIFO_OV | \
                                                         RW_REGW_INT_MASK_RX_DESC_UNAVLBL)

// Interrupt Status Register
#define RW_REGW_INT_STATUS                              0x003E          // interrupt status register address offset
#define RW_REGW_INT_STATUS_RX_OK                        0x0001          // flag for rx ok
#define RW_REGW_INT_STATUS_RX_ERR                       0x0002          // flag for error in rx
#define RW_REGW_INT_STATUS_TX_OK                        0x0004          // flag for tx ok
#define RW_REGW_INT_STATUS_TX_ERR                       0x0008          // flag for error in tx
#define RW_REGW_INT_STATUS_RX_DESC_UNAVLBL              0x0010          // flag for rx descriptor unavailable
#define RW_REGW_INT_STATUS_LINK_CHNG                    0x0020          // flag for Phy link status change
#define RW_REGW_INT_STATUS_RX_FIFO_OV                   0x0040          // flag for rx FIFO overflow
#define RW_REGW_INT_STATUS_TX_DESC_UNAVLBL              0x0080          // flag for tx descriptor unavailable
#define RW_REGW_INT_STATUS_SW                           0x0100          // flag for s\w int. (can be forced via RW_REGB_TX_PRIO_POLL_FORCE_SW_INT)
#define RW_REGW_INT_STATUS_RX_FIFO_EMPTY                0x0200          // flag for rx FIFO empty after a rx FIFO full status indication
#define RW_REGW_INT_STATUS_TIMEOUT                      0x4000          // flag for timeout of TCTR (TCTR value == Timer Interrupt register value)

// Timer Count Register
#define RW_REGDW_TIMER_COUNTER                          0x48            // timer count register address offset      (writing anything to this
#define RW_REGDW_TIMER_COUNTER_UNIT                     8               // granularity(in ns) of the timer counter  (resets the register to 0

// Timer Interrupt Register
#define RW_REGDW_TIMER_COMPARE                          0x58            // timer interrupt register address offset. Acts as a compare register for
                                                                        // RW_REGDW_TIMER_COUNTER. generates RW_REGW_INT_STATUS_TIMEOUT

// PHY Access Register
#define RW_REGDW_PHY_ACCESS                             0x0060          // Phy-access register address offset
#define RW_REGDW_PHY_ACCESS_PHY_REG_DATA_OFFSET         0x00000000      // start bit-offset for the PHY-read/write data field in Phy-AR
#define RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_OFFSET         16              // start bit-offset for the Phy-source/destination address field in Phy-AR
#define RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG          0x80000000      // 1- write data to address, 0- read data from address
#define RW_REGDW_PHY_ACCESS_PHY_REG_DATA_MASK           0xFFFF          // Phy-read/write data field bit-mask in Phy-AR
#define RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_MASK           0x001F0000      // Phy-source/destination address field bit-mask in Phy-AR

// PHY Status Register
#define R_REGB_PHY_STATUS                               0x006C          // MAC register for indicating Phy status (update cycle: 300us)
#define R_REGB_PHY_STATUS_FULL_DUPLEX                   0x0001          // flag for full duplex mode of operation of Phy
#define R_REGB_PHY_STATUS_LINK_STATUS                   0x0002          // flag for link status of Phy (1- link up)
#define R_REGB_PHY_STATUS_LINK_SPEED_10                 0x0004          // flag for link speed of Phy is 10 Mbps
#define R_REGB_PHY_STATUS_LINK_SPEED_100                0x0008          // flag for link speed of Phy is 100 Mbps
#define R_REGB_PHY_STATUS_FULL_DUPLEX_1000              0x0010          // flag for link speed of Phy is 1000 Mbps in full duplex mode
#define R_REGB_PHY_STATUS_RX_FLOW_CTRL_ON               0x0020          // flag for Phy tx flow control on
#define R_REGB_PHY_STATUS_TX_FLOW_CTRL_ON               0x0040          // flag for Phy rx flow control on

// required configuration
#define R_REGB_PHY_STATUS_CFG_MASK                      (R_REGB_PHY_STATUS_FULL_DUPLEX |      \
                                                         R_REGB_PHY_STATUS_LINK_STATUS |      \
                                                         R_REGB_PHY_STATUS_LINK_SPEED_10 |    \
                                                         R_REGB_PHY_STATUS_LINK_SPEED_100 |   \
                                                         R_REGB_PHY_STATUS_FULL_DUPLEX_1000 | \
                                                         R_REGB_PHY_STATUS_RX_FLOW_CTRL_ON |  \
                                                         R_REGB_PHY_STATUS_TX_FLOW_CTRL_ON)
#define R_REGB_PHY_STATUS_REQ_CFG                       (R_REGB_PHY_STATUS_LINK_SPEED_100 | \
                                                         R_REGB_PHY_STATUS_LINK_STATUS)
// PHY Basic Mode Control Register
#define RW_REGW_PHY_BASIC_MOD_CTRL                      0x0000          // Phy basic-mode-control-register address offset
#define RW_REGW_PHY_BASIC_MOD_CTRL_SPEED_1              0x0040          // Phy link speed selection bit 1 (00- 10 Mbps, 01- 100 Mbps)
#define RW_REGW_PHY_BASIC_MOD_CTRL_SPEED_0              0x2000          // Phy link speed selection bit 1 (10- 1000 Mbps, 11- rsvd)
#define RW_REGW_PHY_BASIC_MOD_CTRL_FULL_DUPLEX_EN       0x0100          // enable Phy full duplex mode of operation
#define RW_REGW_PHY_BASIC_MOD_CTRL_RENEGOTIATE          0x0200          // restart the auto-negotiation process
#define RW_REGW_PHY_BASIC_MOD_CTRL_ISOLATE              0x0400          // isolate the port from MII except the management interface
#define RW_REGW_PHY_BASIC_MOD_CTRL_PWR_DWN              0x0800          // power down Phy (link status down)
#define RW_REGW_PHY_BASIC_MOD_CTRL_AUTONEGOTIATE_EN     0x1000          // enable auto-negotiation
#define RW_REGW_PHY_BASIC_MOD_CTRL_LOOP_BACK_EN         0x4000          // enable Phy loopback (MII or analog loopback)
#define RW_REGW_PHY_BASIC_MOD_CTRL_RESET                0x8000          // reset Phy (register and configurations)

// default configurations
#define RW_REGW_PHY_BASIC_MOD_CTRL_DEF                  (~RW_REGW_PHY_BASIC_MOD_CTRL_SPEED_1 &        \
                                                         RW_REGW_PHY_BASIC_MOD_CTRL_SPEED_0 &         \
                                                         ~RW_REGW_PHY_BASIC_MOD_CTRL_FULL_DUPLEX_EN & \
                                                         ~RW_REGW_PHY_BASIC_MOD_CTRL_ISOLATE &        \
                                                         ~RW_REGW_PHY_BASIC_MOD_CTRL_PWR_DWN &        \
                                                         ~RW_REGW_PHY_BASIC_MOD_CTRL_AUTONEGOTIATE_EN)

// PHY Basic Mode Status Register
#define RW_REGW_PHY_BASIC_MOD_STATUS                    0x0001

// PHY Auto-negotiation Advertising Register
#define RW_REGW_PHY_AUTO_NEG_ADV                        0x0004
#define RW_REGW_PHY_AUTO_NEG_ADV_100BASE_TX_HALF        0x0080          // advertise 100 Mbps half-duplex only

// PHY 1000Base-T Control Register
#define RW_REGW_PHY_GBCR                                0x0009
#define RW_REGW_PHY_GBCR_TEST_MODE_MASK                 0xE000          // test mode selection bits (Normal: 000, tx test mode- 1..4: 001..100)
#define RW_REGW_PHY_GBCR_OP_MODE_SEL                    0x1000          // flag to enable manual master/slave configuration (automatic: 0, manual:1)

// default configuration
#define RW_REGW_PHY_GBCR_DEF_CFG                        0x0000          // disable 1000Base-Tx-Full advertisement, enable normal mode

// PHY 1000Base-T Status Register
#define RW_REGW_PHY_GBSR                                0x000A

#ifdef _DBG_TRACE_POINTS_
void target_signalTracePoint(UINT8 tracePointNumber_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   target_signalTracePoint(p)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#define EDRV_COUNT_SEND                                 TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT                              TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_SW_INT                               TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX                                   TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX                                   TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_RX_ERR_RUNT                          TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL                               TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_DESC_UNAVLBL                      TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_RX_ERR_SEQ                           TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC                           TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_DESC_UNAVLBL                      TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_OTHER                         TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_FOVW_EMPTY                        TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_FOVW                              TGT_DBG_SIGNAL_TRACE_POINT(18)
#define EDRV_COUNT_RX_FAE                               TGT_DBG_SIGNAL_TRACE_POINT(19)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

// Tx descriptor structure -ref. Tx descriptor configuration bits
typedef struct
{
    UINT32              frameConfig_le;                 // tx frame configuration ref Descriptor configuration constants
    UINT32              vLanTag_le;                     // vlan tag configuration ref Descriptor configuration constants
    UINT64              txBufferAddr_le;                // tx frame buffer address
} tEdrvTxDesc;

// Rx descriptor structure -ref. Rx descriptor configuration bits
typedef struct
{
    UINT32              frameConfig_le;                 // rx frame configuration ref. Descriptor configuration constants
    UINT32              vLanTag_le;                     // vlan tag configuration ref. Descriptor configuration constants
    UINT64              rxBufferAddr_le;                // rx frame buffer address
} tEdrvRxDesc;

// Dump-tally-counter data
typedef struct
{
    UINT64              countTxOk;                      // tx okay count
    UINT64              countRxOk;                      // rx okay count
    UINT64              countTxErr;                     // tx error count
    UINT32              countRxErr;                     // rx error count
    UINT16              countRxMissPkt;                 // rx missed packets count because of rx FIFO full
    UINT16              countFrameAlignErr;             // frame alignment error count
    UINT32              countTxOkWith01Col;             // tx okay count for frames with 1 collision
    UINT32              countTxOkWith16Col;             // tx okay count for frames with less than 16 collisions
    UINT64              countRxOkWithUnicastAddrMatch;  // rx okay count for matching unicast destination address frames
    UINT64              countRxOkWithBroadcastAddr;     // rx okay count for broadcast frames
    UINT32              countRxOkWithMulticastAddr;     // rx okay count for multicast frames
    UINT16              countTxAbort;                   // tx abort count due to high collision
    UINT16              countTxUnderRun;                // tx underrun or frame discard count
} tEdrvDmpTalyCnt;

// EDRV instance structure
typedef struct
{
    tEdrvInitParam      initParam;                      // initialization parameters for the EDRV passed from upper layer
    struct pci_dev*     pPciDev;                        // pointer to PCI device structure
    void*               pIoAddr;                        // pointer to register space of Ethernet controller
    spinlock_t          spinLockRxBufRelease;           // rxBuffer list access spinlock
    dma_addr_t          pTxDescDma;                     // dma start address for tx descriptor ring
    dma_addr_t          pTxBufDma;                      // dma start address for tx buffers stack
    dma_addr_t          pRxDescDma;                     // dma pointer to rx descriptors
    dma_addr_t          pRxBufDma;                      // dma pointer to the rx buffers stack
    tEdrvTxDesc*        pTxDesc;                        // pointer to tx descriptors
    tEdrvRxDesc*        pRxDesc;                        // pointer to rx descriptors
    void*               pTxBuf;                         // pointer to tx buffer
    void*               apRxBufInDesc[EDRV_MAX_RX_DESCS];
    void*               apRxBufFree[EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS + 1]; // Stack of free rx buffers; +1 additional place if ReleaseRxBuffer is called before return of RxHandler (multi processor)
    int                 rxBufFreeTop;                   // index to the top of the rx buffers stack
    int                 pageAllocations;                // number of physical pages allocated for rxBuffers
    UINT                headRxDesc;                     // pointer to the next rx descriptor to be processed
    UINT                headTxDesc;                     // pointer to the next tx descriptor to be processed
    UINT                tailTxDesc;                     // pointer to the next free descriptor in the ring
    tEdrvTxBuffer*      apTxBufferInDesc[EDRV_MAX_TX_DESCS];    // array of tx buffers currently used by the respective tx descriptors(index)
    BOOL                afTxBufUsed[EDRV_MAX_TX_BUFFERS];       // status of all tx buffers(index) on whether its in use or free

    dma_addr_t          pDtcrDma;                       // dma pointer to the dump-tally-counter data
    tEdrvDmpTalyCnt*    pDtcr;                          // pointer to the dump-tally-counter data

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    UINT64              interruptCount;
    int                 rxBufFreeMin;
    UINT                rxCount[EDRV_SAMPLE_NUM];
    UINT                txCount[EDRV_SAMPLE_NUM];
    UINT                pos;
#endif
} tEdrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int          initOnePciDev(struct pci_dev* pPciDev_p,
                                  const struct pci_device_id* pId_p);
static void         removeOnePciDev(struct pci_dev* pPciDev_p);
static irqreturn_t  edrvIrqHandler(int irqNum_p, void* pDevInstData_p);
static tOplkError   mdioWrite(UINT8 phyRegOffset_p, UINT16 writeValue_p);
static tOplkError   mdioRead(UINT8 phyRegOffset_p, UINT16* pReadValue_p);
static UINT8        calcHash(const UINT8* pMacAddr_p);
static void         reinitRx(void);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static struct pci_device_id     aEdrvPciTbl_l[] =
{
    {0x10ec, 0x8111, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 8111B
    {0x10ec, 0x8168, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 8168B
    {0, }
};

MODULE_DEVICE_TABLE(pci, aEdrvPciTbl_l);

static tEdrvInstance            edrvInstance_l;
static tBufAlloc*               pBufAlloc_l = NULL;
static struct pci_driver        edrvDriver_l =
{
    .name         = DRV_NAME,
    .id_table     = aEdrvPciTbl_l,
    .probe        = initOnePciDev,
    .remove       = removeOnePciDev,
};

//============================================================================//
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
    UINT            index;
    tBufData        bufData;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // clear driver structure
    OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    edrvDriver_l.name         = DRV_NAME,
    edrvDriver_l.id_table     = aEdrvPciTbl_l,
    edrvDriver_l.probe        = initOnePciDev,
    edrvDriver_l.remove       = removeOnePciDev,

    // register PCI driver
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

    // read MAC address from controller
    printk("%s local MAC = ", __func__);
    for (index = 0; index < 6; index++)
    {
        edrvInstance_l.initParam.aMacAddr[index] = EDRV_REGB_READ(RW_REGDW_MAC_ADDR_BANK_0 + index);
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
        // unregister PCI driver
        printk("%s calling pci_unregister_driver()\n", __func__);
        pci_unregister_driver(&edrvDriver_l);
        // clear buffer allocation
        bufalloc_exit(pBufAlloc_l);
        pBufAlloc_l = NULL;
        // clear driver structure
        OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    }
    else
    {
        printk("%s pci driver for openPOWERLINK already unregistered\n", __func__);
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
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError      ret = kErrorOk;
    tBufData        bufData;

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
    tEdrvTxDesc*    pTxDesc;
    UINT32          frameConfig_le;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    // workaround for Rx-FIFO overflow
    if (EDRV_REGW_READ(RW_REGW_INT_STATUS) & RW_REGW_INT_STATUS_RX_FIFO_OV)
    {
        EDRV_REGW_WRITE(RW_REGW_INT_STATUS, RW_REGW_INT_STATUS_RX_FIFO_OV);
    }

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if ((bufferNumber >= EDRV_MAX_TX_BUFFERS) ||
        (edrvInstance_l.afTxBufUsed[bufferNumber] == FALSE))
    {
        ret = kErrorEdrvBufNotExisting;
        goto Exit;
    }

    // check if the next tx descriptor is free
    if (((edrvInstance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK) == edrvInstance_l.headTxDesc)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    // signal trace point
    EDRV_COUNT_SEND;

    // Save pointer to buffer structure for TxHandler
    edrvInstance_l.apTxBufferInDesc[edrvInstance_l.tailTxDesc] = pBuffer_p;

    // get the corresponding Tx Descriptor
    pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.tailTxDesc];

    // set the txbuffer address to the txDescriptor
    ami_setUint64Le(&pTxDesc->txBufferAddr_le,
                    (UINT64)edrvInstance_l.pTxBufDma + (bufferNumber * EDRV_MAX_FRAME_SIZE));

    // configure the TxDescriptor
    pTxDesc->vLanTag_le = 0;
    ami_setUint32Le(&frameConfig_le, (((UINT32)pBuffer_p->txFrameSize) & TX_DESC_FRAME_LENGTH_MASK) |
                                      TX_DESC_FLAG_FS | TX_DESC_FLAG_LS);
    pTxDesc->frameConfig_le |= frameConfig_le;

    wmb();

    // increment Tx descriptor queue tail pointer
    edrvInstance_l.tailTxDesc = (edrvInstance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK;

    // set the owner flag in the TxDescriptor for MAC
    ami_setUint32Le(&frameConfig_le, TX_DESC_FLAG_OWN);
    pTxDesc->frameConfig_le |= frameConfig_le;

    wmb();

    // 8168 fix: TxPoll requests are lost when the Tx packets are too close.
    while ((EDRV_REGB_READ(RW_REGB_TX_PRIO_POLL) & RW_REGB_TX_PRIO_POLL_HI_PRIO_SEND) != 0);

    // Notify the MAC about the pending Tx
    EDRV_REGB_WRITE(RW_REGB_TX_PRIO_POLL, RW_REGB_TX_PRIO_POLL_HI_PRIO_SEND);

Exit:
    return ret;
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
    UINT8           hash;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    hash = calcHash(pMacAddr_p);

    if (hash > 31)
    {
        data = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_LO);
        data |= swab32(1 << (hash - 32));
        EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_LO, data);
    }
    else
    {
        data = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_HI);
        data |= swab32(1 << hash);
        EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_HI, data);
    }

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
    tOplkError      ret = kErrorOk;
    UINT32          data;
    UINT8           hash;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    hash = calcHash(pMacAddr_p);

    if (hash > 31)
    {
        data = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_LO);
        data &= ~swab32(1 << (hash - 32));
        EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_LO, data);
    }
    else
    {
        data = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_HI);
        data &= ~swab32(1 << hash);
        EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_HI, data);
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
    tOplkError    ret = kErrorEdrvInvalidRxBuf;

    // Check parameter validity
    ASSERT(pRxBuffer_p != NULL);

    if (edrvInstance_l.rxBufFreeTop < (EDRV_MAX_RX_BUFFERS - 1))
    {
        ULONG    flag;

        spin_lock_irqsave(&edrvInstance_l.spinLockRxBufRelease, flag);
        edrvInstance_l.apRxBufFree[++edrvInstance_l.rxBufFreeTop] = pRxBuffer_p->pBuffer;
        spin_unlock_irqrestore(&edrvInstance_l.spinLockRxBufRelease, flag);

        ret = kErrorOk;
    }

    return ret;
}
#endif

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Get Edrv module diagnostics

This function returns the Edrv diagnostics to a provided buffer.

\param[out]     pBuffer_p           Pointer to buffer filled with diagnostics.
\param[in]      size_p              Size of buffer

\return The function returns the size of the diagnostics information.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
int edrv_getDiagnostics(char* pBuffer_p, size_t size_p)
{
    const tEdrvTxDesc*  pTxDesc;
    UINT32              txStatus;
    size_t              usedSize = 0;
    UINT                index;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "\nEdrv Diagnostic Information\n");

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Head: %u (%lu)",
                         edrvInstance_l.headTxDesc,
                         (ULONG)(edrvInstance_l.pTxDescDma +
                                 edrvInstance_l.headTxDesc * EDRV_MAX_FRAME_SIZE));

    pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.headTxDesc];
    txStatus = pTxDesc->frameConfig_le;

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "      Headstatus: %lX\n", (ULONG)txStatus);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Tail: %u (%lu)\n",
                         edrvInstance_l.tailTxDesc,
                         (ULONG)(edrvInstance_l.pTxDescDma +
                                 edrvInstance_l.tailTxDesc * EDRV_MAX_FRAME_SIZE));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Free RxBuffers: %d (Min: %d)\n",
                         edrvInstance_l.rxBufFreeTop + 1,
                         edrvInstance_l.rxBufFreeMin);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Status Register:                     0x%08X\n",
                         (UINT)EDRV_REGW_READ(RW_REGW_INT_STATUS));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Interrupt Mask Set/Read Register:    0x%08X\n",
                         (UINT)EDRV_REGW_READ(RW_REGW_INT_MASK));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Config Register:            0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(RW_REGDW_RCR));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Addr Register: 0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(RW_REGDW_RX_DESC_START_ADDR_LO));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Max Buf Size Register:  0x%08X\n",
                         (UINT16)(RW_REGW_RX_MAX_PKT_SIZE_MASK |
                                  EDRV_REGW_READ(RW_REGW_RX_MAX_PKT_SIZE)));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Head Register:    0x%08lX (%u)\n",
                         (ULONG)(edrvInstance_l.pRxDescDma +
                                 edrvInstance_l.headRxDesc * EDRV_MAX_FRAME_SIZE),
                         edrvInstance_l.headRxDesc);

    EDRV_REGDW_WRITE(RW_REGQW_DUMP_TALLY_COUNT_HI, DW_HI(edrvInstance_l.pDtcrDma));

    EDRV_REGDW_WRITE(RW_REGQW_DUMP_TALLY_COUNT_LO,
                     (DW_LO(edrvInstance_l.pDtcrDma) &
                      RW_REGQW_DUMP_TALLY_COUNT_START_ADDR_MASK_LO) |
                     RW_REGQW_DUMP_TALLY_COUNT_CMD);

    // wait until reset has finished
    for (index = EDRV_DIAG_DUMP_TIMEOUT; index > 0; index--)
    {
        if ((EDRV_REGDW_READ(RW_REGQW_DUMP_TALLY_COUNT_LO) & RW_REGQW_DUMP_TALLY_COUNT_CMD) == 0)
        {
            break;
        }

        msleep_interruptible(1);
    }

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "\nFrom Error Counters:\n");
    if (index > 1)
    {
        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Frame Alignment Error:              %u\n",
                             (UINT)edrvInstance_l.pDtcr->countFrameAlignErr);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Error Count:                     %lu\n",
                             (ULONG)edrvInstance_l.pDtcr->countRxErr);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Missed packets due to FIFO Full: %u\n",
                             (UINT)edrvInstance_l.pDtcr->countRxMissPkt);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Okay Count:                      %llu\n",
                             (ULONGLONG)edrvInstance_l.pDtcr->countRxOk);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Ok for broadcast frames:         %llu\n",
                             (ULONGLONG)edrvInstance_l.pDtcr->countRxOkWithBroadcastAddr);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Ok for multicast frames:         %lu\n",
                             (ULONG)edrvInstance_l.pDtcr->countRxOkWithMulticastAddr);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx Ok for matching unicast frames:  %llu\n",
                             (ULONGLONG)edrvInstance_l.pDtcr->countRxOkWithUnicastAddrMatch);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Abort Count:                     %u\n",
                             (UINT)edrvInstance_l.pDtcr->countTxAbort);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Total Error Count:               %llu\n",
                             (ULONGLONG)edrvInstance_l.pDtcr->countTxErr);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Ok Count:                        %llu\n",
                             (ULONGLONG)edrvInstance_l.pDtcr->countTxOk);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Ok Count after 1 Collision:      %lu\n",
                             (ULONG)edrvInstance_l.pDtcr->countTxOkWith01Col);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Ok Count after less than 16 Col: %lu\n",
                             (ULONG)edrvInstance_l.pDtcr->countTxOkWith16Col);

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Tx Packet UnderRun Count:           %u\n",
                             (UINT)edrvInstance_l.pDtcr->countTxUnderRun);
    }
    else
    {
        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "DATA UNAVAILABLE\n");
    }

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "END OF COUNTERS\n");

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Edrv Interrupts:                     %llu\n",
                         edrvInstance_l.interruptCount);

    {
        UINT    rxCountMean = 0;
        UINT    txCountMean = 0;
        UINT    aHistoryRx[EDRV_DIAG_HISTORY_COUNT];
        UINT    aHistoryTx[EDRV_DIAG_HISTORY_COUNT];
        UINT    historyRxMax;
        UINT    historyTxMax;
        int     index;

        for (index = 0; index < EDRV_SAMPLE_NUM; index++)
        {
            rxCountMean += edrvInstance_l.rxCount[index] * 100;
            txCountMean += edrvInstance_l.txCount[index] * 100;
        }

        rxCountMean /= EDRV_SAMPLE_NUM;
        txCountMean /= EDRV_SAMPLE_NUM;

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx/Int %u.%02u / Tx/Int %u.%02u\n",
                             rxCountMean / 100, rxCountMean % 100,
                             txCountMean / 100, txCountMean % 100);

        historyRxMax = 0;
        historyTxMax = 0;
        for (index = 0; index < EDRV_DIAG_HISTORY_COUNT; index++)
        {
            aHistoryRx[index] = 0;
            aHistoryTx[index] = 0;
        }

        for (index = 0; index < EDRV_SAMPLE_NUM; index++)
        {
            if (edrvInstance_l.rxCount[index] < EDRV_DIAG_HISTORY_COUNT)
            {
                aHistoryRx[edrvInstance_l.rxCount[index]]++;
            }
            else if (edrvInstance_l.rxCount[index] > historyRxMax)
            {
                historyRxMax = edrvInstance_l.rxCount[index];
            }

            if (edrvInstance_l.txCount[index] < EDRV_DIAG_HISTORY_COUNT)
            {
                aHistoryTx[edrvInstance_l.txCount[index]]++;
            }
            else if (edrvInstance_l.txCount[index] > historyTxMax)
            {
                historyTxMax = edrvInstance_l.txCount[index];
            }
        }

        if (historyRxMax > 0)
        {
            usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                 "MaxRx %3u\n", historyRxMax);
        }

        if (historyTxMax > 0)
        {
            usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                 "MaxTx %3u\n", historyTxMax);
        }

        for (index = EDRV_DIAG_HISTORY_COUNT - 1; index >= 0; index--)
        {
            if ((aHistoryRx[index] != 0) || (aHistoryTx[index] != 0))
            {
                usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                     "Hist  %3u  %5u  %5u\n",
                                     index, aHistoryRx[index], aHistoryTx[index]);
            }
        }
    }

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize, "\n");

    return usedSize;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

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
    UINT16      status;
    irqreturn_t handled = IRQ_HANDLED;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(pDevInstData_p);

    // read the interrupt status
    status = EDRV_REGW_READ(RW_REGW_INT_STATUS);
    // acknowledge the interrupts
    EDRV_REGW_WRITE(RW_REGW_INT_STATUS, status);

    if (status == 0)
    {
        handled = IRQ_NONE;
        goto Exit;
    }

    // Check for the interrupts that require descriptor handling
    if ((status & (RW_REGW_INT_STATUS_RX_OK | RW_REGW_INT_STATUS_TX_OK |
                   RW_REGW_INT_STATUS_RX_ERR | RW_REGW_INT_STATUS_TX_ERR)) != 0)
    {
        UINT    headRxDescOrg;

        if (edrvInstance_l.pTxBuf == NULL)
        {
            printk("%s Tx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        headRxDescOrg = edrvInstance_l.headRxDesc;

        do
        {
            tEdrvRxDesc*    pRxDesc;
            tEdrvTxDesc*    pTxDesc;

            // Process receive descriptors
            pRxDesc = &edrvInstance_l.pRxDesc[edrvInstance_l.headRxDesc];

            while ((ami_getUint32Le(&pRxDesc->frameConfig_le) & RX_DESC_FLAG_OWN) == 0)
            {
                // Rx frame available
                tEdrvRxBuffer           rxBuffer;
                tEdrvReleaseRxBuffer    retReleaseRxBuffer;
                UINT32                  frameConfig;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                edrvInstance_l.rxCount[edrvInstance_l.pos]++;
#endif

                frameConfig = ami_getUint32Le(&pRxDesc->frameConfig_le);

                if (!((frameConfig & RX_DESC_FLAG_FS) && (frameConfig & RX_DESC_FLAG_LS)))
                {
                    // Multiple descriptors used for one packet
                    EDRV_COUNT_RX_ERR_SEQ;
                }
                else if (((frameConfig & RX_DESC_FLAG_RES) && (frameConfig & RX_DESC_FLAG_LS)) != 0)
                {
                    // Error present
                    if ((frameConfig & RX_DESC_FLAG_CRC) != 0)
                    {
                        // CRC error
                        EDRV_COUNT_RX_ERR_CRC;
                    }
                    else if ((frameConfig & RX_DESC_FLAG_RWT) != 0)
                    {
                        // Packet size exceeds 8192 bytes
                        EDRV_COUNT_RX_ERR_SEQ;
                    }
                    else if ((frameConfig & RX_DESC_FLAG_RUNT) != 0)
                    {
                        // received a runt
                        EDRV_COUNT_RX_ERR_RUNT;
                    }
                    else
                    {
                        // FAE(frame alignment) error
                        EDRV_COUNT_RX_FAE;
                    }
                }
                else
                {
                    // Packet is OK
                    void**  ppRxBufInDesc;

                    EDRV_COUNT_RX;

                    rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

                    // Get length of received packet
                    // size does not contain CRC as RW_REGW_CPLUS_COMMAND_RX_CHKSUM_OFLD_EN is set
                    rxBuffer.rxFrameSize = (frameConfig & RX_DESC_FRAME_LENGTH_MASK) - EDRV_ETH_CRC_SIZE;

                    ppRxBufInDesc = &edrvInstance_l.apRxBufInDesc[edrvInstance_l.headRxDesc];
                    rxBuffer.pBuffer = *ppRxBufInDesc;

                    pci_dma_sync_single_for_cpu(edrvInstance_l.pPciDev,
                                                (dma_addr_t)ami_getUint64Le(&pRxDesc->rxBufferAddr_le),
                                                EDRV_RX_BUFFER_SIZE,
                                                PCI_DMA_FROMDEVICE);

                    // Call Rx handler of Data link layer
                    retReleaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
                    if (retReleaseRxBuffer == kEdrvReleaseRxBufferLater)
                    {
                        if (edrvInstance_l.rxBufFreeTop >= 0)
                        {
                            dma_addr_t      dmaAddr;
                            void*           pRxBufInDescPrev;
                            ULONG           flags;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                            if (edrvInstance_l.rxBufFreeTop < edrvInstance_l.rxBufFreeMin)
                            {
                                edrvInstance_l.rxBufFreeMin = edrvInstance_l.rxBufFreeTop;
                            }
#endif
                            pRxBufInDescPrev = *ppRxBufInDesc;

                            spin_lock_irqsave(&edrvInstance_l.spinLockRxBufRelease, flags);
                            *ppRxBufInDesc = edrvInstance_l.apRxBufFree[edrvInstance_l.rxBufFreeTop--];
                            spin_unlock_irqrestore(&edrvInstance_l.spinLockRxBufRelease, flags);

                            if (*ppRxBufInDesc != pRxBufInDescPrev)
                            {
                                pci_unmap_single(edrvInstance_l.pPciDev,
                                                 (dma_addr_t)ami_getUint64Le(&pRxDesc->rxBufferAddr_le),
                                                 EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);

                                dmaAddr = pci_map_single(edrvInstance_l.pPciDev,
                                                         *ppRxBufInDesc,
                                                         EDRV_RX_BUFFER_SIZE,
                                                         PCI_DMA_FROMDEVICE);
                                if (pci_dma_mapping_error(edrvInstance_l.pPciDev, dmaAddr))
                                {
                                    printk("%s DMA mapping error\n", __func__);
                                    // Signal dma mapping error
                                }

                                ami_setUint64Le(&pRxDesc->rxBufferAddr_le, (UINT64)dmaAddr);
                            }
                        }
                        else
                        {
                            // Signal no free RxBuffers left
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                            edrvInstance_l.rxBufFreeMin = -1;
#endif
                        }
                    }
                }

                ami_setUint32Le(&pRxDesc->frameConfig_le, (frameConfig & RX_DESC_FLAG_EOR) |
                                                            (EDRV_RX_BUFFER_SIZE & RX_DESC_FRAME_LENGTH_MASK) |
                                                            RX_DESC_FLAG_OWN);

                edrvInstance_l.headRxDesc = (edrvInstance_l.headRxDesc + 1) & EDRV_RX_DESC_MASK;
                wmb();

                pRxDesc = &edrvInstance_l.pRxDesc[edrvInstance_l.headRxDesc];
            }

            // Acknowledge the IRQ again, to avoid a blank irq
            EDRV_REGW_WRITE(RW_REGW_INT_STATUS, RW_REGW_INT_STATUS_RX_OK | RW_REGW_INT_STATUS_RX_ERR);

            // Process one transmit descriptor
            pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.headTxDesc];

            if ((pTxDesc->frameConfig_le & TX_DESC_FLAG_OWN) == 0)
            {
                // Transmit finished
                tEdrvTxBuffer*   pTxBuffer;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                edrvInstance_l.txCount[edrvInstance_l.pos]++;
#endif

                pTxBuffer = edrvInstance_l.apTxBufferInDesc[edrvInstance_l.headTxDesc];

                if (pTxBuffer != NULL)
                {
                    // Call Tx handler of Data link layer
                    edrvInstance_l.apTxBufferInDesc[edrvInstance_l.headTxDesc] = NULL;
                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }

                    // Increment Tx descriptor queue head pointer
                    edrvInstance_l.headTxDesc = (edrvInstance_l.headTxDesc + 1) & EDRV_TX_DESC_MASK;
                }
                else
                {
                    // No Tx pending- but this should not come for the second time
                    break;
                }
            }
            else
            {
                // MAC holds the descriptor, i.e. Tx pending
                break;
            }
        } while (edrvInstance_l.headTxDesc != edrvInstance_l.tailTxDesc);

        // Acknowledge the IRQ again, to avoid a blank irq
        EDRV_REGW_WRITE(RW_REGW_INT_STATUS, RW_REGW_INT_STATUS_TX_OK | RW_REGW_INT_STATUS_TX_ERR);
    }

    // Check for Tx Errors
    if ((status & RW_REGW_INT_STATUS_TX_ERR) != 0)
    {
        // excessive collision- pkt could not be transmitted
        EDRV_COUNT_TX_COL;
    }

    if ((status & RW_REGW_INT_STATUS_TX_OK) != 0)
    {
        // Tx Ok
        EDRV_COUNT_TX;
    }
    else if ((status & RW_REGW_INT_STATUS_TX_DESC_UNAVLBL) != 0)
    {
        // RW_REGW_INT_STATUS_TX_DESC_UNAVLBL with RW_REGW_INT_STATUS_TX_OK is normal
        EDRV_COUNT_TX_DESC_UNAVLBL;
    }

    // Set of Rx errors
    if ((status & (RW_REGW_INT_STATUS_RX_ERR |
                   RW_REGW_INT_STATUS_RX_FIFO_OV |
                   RW_REGW_INT_STATUS_RX_FIFO_EMPTY |
                   RW_REGW_INT_STATUS_RX_DESC_UNAVLBL)) != 0)
    {
        // receive error interrupt
        EDRV_REGW_WRITE(RW_REGW_INT_STATUS, RW_REGW_INT_STATUS_RX_ERR |
                        RW_REGW_INT_STATUS_RX_FIFO_OV |
                        RW_REGW_INT_STATUS_RX_FIFO_EMPTY |
                        RW_REGW_INT_STATUS_RX_DESC_UNAVLBL);

        if ((status & RW_REGW_INT_STATUS_RX_FIFO_EMPTY) != 0)
        {
            EDRV_COUNT_RX_FOVW_EMPTY;
        }
        else if ((status & RW_REGW_INT_STATUS_RX_FIFO_OV) != 0)
        {
            EDRV_COUNT_RX_FOVW;

            // Reinitialize Rx process
            reinitRx();
        }

        if ((status & RW_REGW_INT_STATUS_RX_ERR) != 0)
        {
            // Check the descriptor for the exact error
            EDRV_COUNT_RX_ERR_OTHER;
        }

        if ((status & RW_REGW_INT_STATUS_RX_DESC_UNAVLBL) != 0)
        {
            EDRV_COUNT_RX_DESC_UNAVLBL;

            // Reinitialize Rx process
            reinitRx();
        }
    }

    if ((status & RW_REGW_INT_STATUS_LINK_CHNG) != 0)
    {
        printk("Mac Link Status has changed\n");
    }

    if ((status & RW_REGW_INT_STATUS_SW) != 0)
    {
        EDRV_COUNT_SW_INT;
    }

    if ((status & RW_REGW_INT_STATUS_TIMEOUT) != 0)
    {
        // Timeout
        EDRV_COUNT_TIMEOUT;
    }

Exit:
    return handled;
}

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
static int initOnePciDev(struct pci_dev* pPciDev_p,
                         const struct pci_device_id* pId_p)
{
    UINT        index;
    UINT32      temp;
    int         result = 0;
    UINT16      regValue;
    UINT64      descAddr;
    UINT        order;
    UINT        rxBuffersInAllocation;
    UINT        rxBufferCount;
    UINT32      flags_le;

    if (edrvInstance_l.pPciDev != NULL)
    {
        // Edrv is already connected to a PCI device
        printk("%s device %s discarded\n", __func__, pci_name(pPciDev_p));
        result = -ENODEV;
        goto Exit;
    }

    edrvInstance_l.pPciDev = pPciDev_p;

    // enable device
    printk("%s enable device\n", __func__);
    result = pci_enable_device(pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    // pci base address 1 is MMIO
    if ((pci_resource_flags(pPciDev_p, 2) & IORESOURCE_MEM) == 0)
    {
        result = -ENODEV;
        goto Exit;
    }

    // check for weird broken IO regions
    if (pci_resource_len(pPciDev_p, 2) < R8111_REGS_SIZE)
    {
        result = -ENODEV;
        goto Exit;
    }

    printk("%s request regions\n", __func__);
    result = pci_request_regions(pPciDev_p, DRV_NAME);
    if (result != 0)
    {
        goto ExitFail;
    }

    printk("%s ioremap\n", __func__);
    edrvInstance_l.pIoAddr = ioremap(pci_resource_start(pPciDev_p, 2),
                                     pci_resource_len(pPciDev_p, 2));
    if (edrvInstance_l.pIoAddr == NULL)
    {
        // remap of controller's register space failed
        result = -EIO;
        goto ExitFail;
    }

    // Enable PCI busmaster
    printk("%s enable busmaster\n", __func__);
    pci_set_master(pPciDev_p);

    // Enable checksum offloading in the card
    printk("%s enable checksum offloading\n", __func__);
    EDRV_REGW_WRITE(RW_REGW_CPLUS_COMMAND, RW_REGW_CPLUS_COMMAND_RX_CHKSUM_OFLD_EN);

    // Reset controller
    printk("%s reset controller\n", __func__);
    EDRV_REGB_WRITE(RW_REGB_COMMAND, RW_REGB_COMMAND_RST);

    // Wait until reset has finished
    for (index = EDRV_MASTER_DISABLE_TIMEOUT; index > 0; index--)
    {
        if ((EDRV_REGB_READ(RW_REGB_COMMAND) & RW_REGB_COMMAND_RST) == 0)
        {
            break;
        }

        msleep_interruptible(1);
    }

    if (index == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    // Normal mode of operation
    EDRV_REGB_WRITE(RW_REGB_93C46_CMD, RW_REGB_93C46_CMD_OP_MODE_NORM & RW_REGB_93C46_CMD_OP_MODE_CFG_MASK);

    // Check hardware version, i.e. chip ID
    temp = EDRV_REGDW_READ(RW_REGDW_TCR);
    if (((temp & REGDW_TCR_VER_MASK) != REGDW_TCR_VER_8111B_8168B) &&
        ((temp & REGDW_TCR_VER_MASK) != REGDW_TCR_VER_8100E) &&
        ((temp & REGDW_TCR_VER_MASK) != REGDW_TCR_VER_8101E) &&
        ((temp & REGDW_TCR_VER_MASK) != REGDW_TCR_VER_8111D) &&
        ((temp & REGDW_TCR_VER_MASK) != REGDW_TCR_VER_8111C))
    {
        // unsupported chip
        printk("%s Unsupported chip! TCR = 0x%08lX\n", __func__, (ULONG)temp);
        result = -ENODEV;
        goto ExitFail;
    }

    // initialize the spinlock for rx buffers stack
    spin_lock_init(&edrvInstance_l.spinLockRxBufRelease);

    // disable interrupts
    printk("%s disable interrupts\n", __func__);
    EDRV_REGW_WRITE(RW_REGW_INT_MASK, 0);

    // acknowledge all pending interrupts
    EDRV_REGW_WRITE(RW_REGW_INT_STATUS, EDRV_REGW_READ(RW_REGW_INT_STATUS));

    // Enable msi
    printk("Enable MSI\n");
    result = pci_enable_msi(pPciDev_p);
    if (result != 0)
    {
        printk("%s Could not enable MSI\n", __func__);
    }

    // install interrupt handler
    printk("%s install interrupt handler\n", __func__);
    result = request_irq(pPciDev_p->irq,
                         edrvIrqHandler,
                         IRQF_SHARED,
                         DRV_NAME, /* pPciDev_p->dev.name */
                         pPciDev_p);
    if (result != 0)
    {
        goto ExitFail;
    }

    // Set Dma mask size to 32 bits
    printk("enable DMA- size: %lu\n", (ULONG)sizeof(dma_addr_t));
    result = pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(32));
    if (result != 0)
    {
        printk(KERN_WARNING "edrv-8111: No suitable DMA available.\n");
        goto ExitFail;
    }

    // allocate memory for error counters
    edrvInstance_l.pDtcr = pci_alloc_consistent(pPciDev_p,
                                                sizeof(tEdrvDmpTalyCnt),
                                                &edrvInstance_l.pDtcrDma);
    if (edrvInstance_l.pDtcr == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    OPLK_MEMSET(edrvInstance_l.pDtcr, 0, sizeof(tEdrvDmpTalyCnt));

    // allocate and map large (PAGE_SIZE or so) consistent DMA regions
    // allocate tx-descriptors
    edrvInstance_l.pTxDesc = pci_alloc_consistent(pPciDev_p,
                                                  EDRV_TX_DESCS_SIZE,
                                                  &edrvInstance_l.pTxDescDma);
    if (edrvInstance_l.pTxDesc == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    OPLK_MEMSET(edrvInstance_l.pTxDesc, 0, EDRV_TX_DESCS_SIZE);

    // allocate rx-descriptors
    edrvInstance_l.pRxDesc = pci_alloc_consistent(pPciDev_p,
                                                  EDRV_RX_DESCS_SIZE,
                                                  &edrvInstance_l.pRxDescDma);
    if (edrvInstance_l.pRxDesc == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    OPLK_MEMSET(edrvInstance_l.pRxDesc, 0, EDRV_RX_DESCS_SIZE);

    printk("%s allocate buffers\n", __func__);
    edrvInstance_l.pTxBuf = pci_alloc_consistent(pPciDev_p,
                                                 EDRV_TX_BUFFER_SIZE,
                                                 &edrvInstance_l.pTxBufDma);

    if (edrvInstance_l.pTxBuf == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    // Allocate Rx Buffers
    if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
    {
        // rx-buffer is larger than or equal to page size
        order = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
        rxBuffersInAllocation = 1;
    }
    else
    {
        // Multiple rx-buffers fit into one page
        order = 0;
        rxBuffersInAllocation = 1 << (PAGE_SHIFT - EDRV_RX_BUFFER_SIZE_SHIFT);
    }

    for (rxBufferCount = 0; rxBufferCount < EDRV_MAX_RX_BUFFERS;)
    {
        ULONG       bufferPointer;
        UINT        nInAlloc;

        bufferPointer = __get_free_pages(GFP_KERNEL, order);
        if (bufferPointer == 0)
        {
            result = -ENOMEM;
            goto ExitFail;
        }

        edrvInstance_l.pageAllocations++;

        for (nInAlloc = 0; nInAlloc < rxBuffersInAllocation; nInAlloc++)
        {
            if (rxBufferCount < EDRV_MAX_RX_DESCS)
            {
                // Insert rx-buffer in rx-descriptor
                edrvInstance_l.apRxBufInDesc[rxBufferCount] = (void*)bufferPointer;
            }
            else
            {
                // Insert rx-buffer in free-rx-buffer stack
                edrvInstance_l.apRxBufFree[rxBufferCount - EDRV_MAX_RX_DESCS] = (void*)bufferPointer;
            }

            rxBufferCount++;
            bufferPointer += EDRV_RX_BUFFER_SIZE;
        }
    }

    edrvInstance_l.rxBufFreeTop = EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS - 1;

    // initialize Rx descriptors
    printk("%s initialize Rx descriptors\n", __func__);
    for (index = 0; index < EDRV_MAX_RX_DESCS; index++)
    {
        dma_addr_t    dmaAddr;

        // get dma streaming
        dmaAddr = pci_map_single(edrvInstance_l.pPciDev, edrvInstance_l.apRxBufInDesc[index],
                                 EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);
        if (pci_dma_mapping_error(edrvInstance_l.pPciDev, dmaAddr))
        {
            result = -ENOMEM;
            goto ExitFail;
        }

        ami_setUint64Le(&edrvInstance_l.pRxDesc[index].rxBufferAddr_le, (UINT64)dmaAddr);
        ami_setUint32Le(&edrvInstance_l.pRxDesc[index].frameConfig_le,
                        (EDRV_RX_BUFFER_SIZE & RX_DESC_FRAME_LENGTH_MASK) | RX_DESC_FLAG_OWN);
    }

    // Mark the end of descriptor ring
    ami_setUint32Le(&flags_le, TX_DESC_FLAG_EOR);
    edrvInstance_l.pTxDesc[EDRV_MAX_TX_DESCS - 1].frameConfig_le |= flags_le;
    ami_setUint32Le(&flags_le, RX_DESC_FLAG_EOR);
    edrvInstance_l.pRxDesc[EDRV_MAX_RX_DESCS - 1].frameConfig_le |= flags_le;

    // Rx descriptor typ is set to legacy by default
    descAddr = edrvInstance_l.pRxDescDma;
    EDRV_REGDW_WRITE(RW_REGDW_RX_DESC_START_ADDR_LO, DW_LO(descAddr));
    EDRV_REGDW_WRITE(RW_REGDW_RX_DESC_START_ADDR_HI, DW_HI(descAddr));
    edrvInstance_l.headRxDesc = 0;

    // initialize Tx descriptors
    printk("%s initialize Tx descriptors\n", __func__);
    OPLK_MEMSET(edrvInstance_l.apTxBufferInDesc, 0, sizeof(edrvInstance_l.apTxBufferInDesc));
    descAddr = edrvInstance_l.pTxDescDma;
    EDRV_REGDW_WRITE(RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_LO, DW_LO(descAddr));
    EDRV_REGDW_WRITE(RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_HI, DW_HI(descAddr));
    edrvInstance_l.headTxDesc = 0;
    edrvInstance_l.tailTxDesc = 0;
    printk("Descriptor Address in pci; HPA: 0x%08X, LPA: 0x%08X\n",
          EDRV_REGDW_READ(RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_HI),
          EDRV_REGDW_READ(RW_REGDW_TX_HI_PRIO_DESC_START_ADDR_LO));

    // enable interrupts
    printk("%s enable interrupts\n", __func__);
    EDRV_REGW_WRITE(RW_REGW_INT_STATUS, EDRV_REGW_READ(RW_REGW_INT_STATUS));
    EDRV_REGW_WRITE(RW_REGW_INT_MASK, RW_REGW_INT_MASK_LINK_CHNG);

    // Reset Phy
    result = mdioWrite((UINT8)RW_REGW_PHY_BASIC_MOD_CTRL, (UINT16)RW_REGW_PHY_BASIC_MOD_CTRL_PWR_DWN);

    if (result != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }

    msleep_interruptible(EDRV_PHY_LINK_CHANGE_WAIT_TIME);

    result = mdioWrite((UINT8)RW_REGW_PHY_BASIC_MOD_CTRL,
                       (UINT16)RW_REGW_PHY_BASIC_MOD_CTRL_RESET | RW_REGW_PHY_BASIC_MOD_CTRL_DEF);

    if (result != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }

    // Wait for Phy reset to complete
    for (index = EDRV_LINK_UP_TIMEOUT; index > 0; index--)
    {
        regValue = 0xFFFF;
        result = mdioRead((UINT8)RW_REGW_PHY_BASIC_MOD_CTRL, (UINT16*)&regValue);

        if (((regValue & RW_REGW_PHY_BASIC_MOD_CTRL_RESET) == 0x0) && (result == kErrorOk))
        {
            printk("Phy reset done!\n");
            break;
        }
    }

    if (index == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    // Phy Configuration
    if ((mdioWrite((UINT8)0x0E, (UINT16)0x0000)) != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }
    else if ((mdioWrite((UINT8)RW_REGW_PHY_AUTO_NEG_ADV, (UINT16)RW_REGW_PHY_AUTO_NEG_ADV_100BASE_TX_HALF)) != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }
    else if ((mdioWrite((UINT8)RW_REGW_PHY_GBCR, (UINT16)RW_REGW_PHY_GBCR_DEF_CFG)) != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }

    result = mdioWrite((UINT8)RW_REGW_PHY_BASIC_MOD_CTRL, (UINT16)RW_REGW_PHY_BASIC_MOD_CTRL_PWR_DWN);

    if (result != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }

    msleep_interruptible(EDRV_PHY_LINK_CHANGE_WAIT_TIME);

    result = mdioWrite((UINT8)RW_REGW_PHY_BASIC_MOD_CTRL, (UINT16)RW_REGW_PHY_BASIC_MOD_CTRL_DEF);

    if (result != kErrorOk)
    {
        result = -EIO;
        goto ExitFail;
    }

    // wait for phy link up (updated in MAC)
    for (index = EDRV_PHY_UP_WAIT_TIME; index > 0; index--)
    {
        if ((EDRV_REGB_READ(R_REGB_PHY_STATUS) & R_REGB_PHY_STATUS_CFG_MASK) == R_REGB_PHY_STATUS_REQ_CFG)
        {
            break;
        }

        msleep_interruptible(1);
    }

    if (index == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    // Set the frame size for Tx
    EDRV_REGB_WRITE(RW_REGB_TX_MAX_PKT_SIZE,
                    (EDRV_MAX_FRAME_SIZE / RW_REGB_TX_MAX_PKT_SIZE_UNIT) & RW_REGB_TX_MAX_PKT_SIZE_MASK);

    EDRV_REGW_WRITE(RW_REGW_RX_MAX_PKT_SIZE, EDRV_MAX_FRAME_SIZE | RW_REGW_RX_MAX_PKT_SIZE_MASK);

    // enable transmitter and receiver
    printk("%s enable Tx and Rx", __func__);
    EDRV_REGB_WRITE(RW_REGB_COMMAND, (RW_REGB_COMMAND_RX_EN | RW_REGB_COMMAND_TX_EN));

    // set transmit configuration register
    printk("%s set Tx conf register", __func__);
    EDRV_REGDW_WRITE(RW_REGDW_TCR, RW_REGDW_TCR_DEF);
    printk(" = 0x%08X\n", EDRV_REGDW_READ(RW_REGDW_TCR));

    // set receive configuration register
    printk("%s set Rx conf register", __func__);
    EDRV_REGDW_WRITE(RW_REGDW_RCR, RW_REGDW_RCR_DEF);
    printk(" = 0x%08X\n", EDRV_REGDW_READ(RW_REGDW_RCR));

    // reset multicast MAC address filter
    EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_LO, 0);
    temp = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_LO);
    EDRV_REGDW_WRITE(RW_REGDW_MULTICAST_ADDR_BANK_HI, 0);
    temp = EDRV_REGDW_READ(RW_REGDW_MULTICAST_ADDR_BANK_HI);

    EDRV_REGW_WRITE(RW_REGW_INT_MASK, RW_REGW_INT_MASK_DEF);

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
    UINT32      temp;
    UINT        order;
    ULONG       bufferPointer;
    UINT        rxBufferCount;

    if (edrvInstance_l.pPciDev != pPciDev_p)
    {
        // trying to remove unknown device
        BUG_ON(edrvInstance_l.pPciDev != pPciDev_p);
        goto Exit;
    }

    // disable transmitter and receiver
    EDRV_REGB_WRITE(RW_REGB_COMMAND, 0);

    if (edrvInstance_l.pIoAddr != NULL)
    {
        // disable interrupts
        EDRV_REGW_WRITE(RW_REGW_INT_MASK, 0);
        temp = EDRV_REGDW_READ(RW_REGW_INT_MASK);

        // disable transmitter and receiver
        EDRV_REGDW_WRITE(RW_REGDW_TCR, 0);
    }

    // remove interrupt handler
    free_irq(pPciDev_p->irq, pPciDev_p);

    // Disable Message Signaled Interrupt
    printk("%s Disable MSI\n", __func__);
    pci_disable_msi(pPciDev_p);

    // free buffers
    if (edrvInstance_l.pDtcr != NULL)
    {
        pci_free_consistent(pPciDev_p,
                            sizeof(tEdrvDmpTalyCnt),
                            edrvInstance_l.pDtcr,
                            edrvInstance_l.pDtcrDma);
        edrvInstance_l.pDtcr = NULL;
    }

    if (edrvInstance_l.pTxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p,
                            EDRV_TX_BUFFER_SIZE,
                            edrvInstance_l.pTxBuf,
                            edrvInstance_l.pTxBufDma);
        edrvInstance_l.pTxBuf = NULL;
    }

    if (edrvInstance_l.pTxDesc != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_DESCS_SIZE,
                            edrvInstance_l.pTxDesc, edrvInstance_l.pTxDescDma);
        edrvInstance_l.pTxDesc = NULL;
    }

    if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
    {
        // rx-buffer is larger than or equal to page size
        order = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
    }
    else
    {
        // Multiple rx-buffers fit into one page
        order = 0;
    }

    for (rxBufferCount = 0; rxBufferCount < EDRV_MAX_RX_DESCS; rxBufferCount++)
    {
        pci_unmap_single(edrvInstance_l.pPciDev,
                         (dma_addr_t)ami_getUint64Le(&edrvInstance_l.pRxDesc[rxBufferCount].rxBufferAddr_le),
                         EDRV_RX_BUFFER_SIZE,
                         PCI_DMA_FROMDEVICE);

        bufferPointer = (ULONG)edrvInstance_l.apRxBufInDesc[rxBufferCount];

        if ((order == 0) && ((bufferPointer & ((1UL << PAGE_SHIFT) - 1)) == 0))
        {
            free_pages(bufferPointer, order);
            edrvInstance_l.pageAllocations--;
        }
    }

    if (edrvInstance_l.rxBufFreeTop < EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS - 1)
    {
        printk("%s %d rx-buffers were lost\n", __func__, edrvInstance_l.rxBufFreeTop);
    }

    for (; edrvInstance_l.rxBufFreeTop >= 0; edrvInstance_l.rxBufFreeTop--)
    {
        bufferPointer = (ULONG)edrvInstance_l.apRxBufFree[edrvInstance_l.rxBufFreeTop];

        if ((order == 0) && ((bufferPointer & ((1UL << PAGE_SHIFT) - 1)) == 0))
        {
            free_pages(bufferPointer, order);
            edrvInstance_l.pageAllocations--;
        }
    }

    if (edrvInstance_l.pageAllocations > 0)
    {
        printk("%s Less pages freed than allocated (%d)\n", __func__, edrvInstance_l.pageAllocations);
    }
    else if (edrvInstance_l.pageAllocations < 0)
    {
        printk("%s Attempted to free more pages than allocated (%d)\n", __func__, (edrvInstance_l.pageAllocations * -1));
    }

    if (edrvInstance_l.pRxDesc != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_RX_DESCS_SIZE,
                            edrvInstance_l.pRxDesc, edrvInstance_l.pRxDescDma);
        edrvInstance_l.pRxDesc = NULL;
    }

    // unmap controller's register space
    if (edrvInstance_l.pIoAddr != NULL)
    {
        iounmap(edrvInstance_l.pIoAddr);
        edrvInstance_l.pIoAddr = NULL;
    }

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    // release memory regions
    pci_release_regions(pPciDev_p);

    edrvInstance_l.pPciDev = NULL;

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Writes from the phy register

This function sets the 16 bit data to the phy register.

\param[in]      phyRegOffset_p      5-bit phy Register offset address
\param[in]      writeValue_p        The 16-bit data to be written to the phy register

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError mdioWrite(UINT8 phyRegOffset_p, UINT16 writeValue_p)
{
    UINT16          waitCount;
    tOplkError      ret  = kErrorRetry;

    EDRV_REGDW_WRITE(RW_REGDW_PHY_ACCESS,
                     (((phyRegOffset_p << RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_OFFSET) &
                       RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_MASK) |
                      ((writeValue_p << RW_REGDW_PHY_ACCESS_PHY_REG_DATA_OFFSET) &
                       RW_REGDW_PHY_ACCESS_PHY_REG_DATA_MASK) |
                      RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG));

    for (waitCount = EDRV_AUTO_READ_DONE_TIMEOUT; waitCount > 0; waitCount--)
    {
        if ((EDRV_REGDW_READ(RW_REGDW_PHY_ACCESS) & RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG) == 0)
        {
            ret = kErrorOk;
            break;
        }

        msleep_interruptible(1);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reads from the phy register

This function gets the 16 bit data from the phy register.

\param[in]      phyRegOffset_p      5-bit phy Register offset address
\param[out]     pReadValue_p        Pointer to the address where the 16-bit read
                                    value will be stored

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError mdioRead(UINT8 phyRegOffset_p, UINT16* pReadValue_p)
{
    UINT16          waitCount;
    tOplkError      ret  = kErrorRetry;

    EDRV_REGDW_WRITE(RW_REGDW_PHY_ACCESS,
                     (((phyRegOffset_p << RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_OFFSET) &
                          RW_REGDW_PHY_ACCESS_PHY_REG_ADDR_MASK) &
                         ~RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG));

    for (waitCount = EDRV_AUTO_READ_DONE_TIMEOUT; waitCount > 0; waitCount--)
    {
        if ((EDRV_REGDW_READ(RW_REGDW_PHY_ACCESS) & RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG) ==
            RW_REGDW_PHY_ACCESS_PHY_REG_RD_WR_FLAG)
        {
            *pReadValue_p = (EDRV_REGDW_READ(RW_REGDW_PHY_ACCESS) & RW_REGDW_PHY_ACCESS_PHY_REG_DATA_MASK);
            ret = kErrorOk;
            break;
        }

        msleep_interruptible(1);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate MAC address hash

This function calculates the entry for the hash-table from MAC address.

\param  pMacAddr_p  Pointer to MAC address

\return The function returns the calculated hash table.
*/
//------------------------------------------------------------------------------
static UINT8 calcHash(const UINT8* pMacAddr_p)
{
    UINT32       byteCounter;
    UINT32       bitCounter;
    UINT32       data;
    UINT32       crc;
    UINT32       carry;
    const UINT8* pData;
    UINT8        hash;

    pData = pMacAddr_p;

    // calculate crc32 value of mac address
    crc = 0xFFFFFFFF;

    for (byteCounter = 0; byteCounter < 6; byteCounter++)
    {
        data = *pData;
        pData++;
        for (bitCounter = 0; bitCounter < 8; bitCounter++, data >>= 1)
        {
            carry = (((crc >> 31) ^ data) & 1);
            crc = crc << 1;
            if (carry != 0)
            {
                crc = (crc ^ EDRV_CRC32_POLY) | carry;
            }
        }
    }

    // only upper 6 bits (HASH_BITS) are used
    // which point to specific bit in the hash registers
    hash = (UINT8)((crc >> (32 - EDRV_HASH_BITS)) & 0x3f);

    return hash;
}

//------------------------------------------------------------------------------
/**
\brief  Reinitialize Rx process

This function reinitializes the Rx process because of an error.
*/
//------------------------------------------------------------------------------
static void reinitRx(void)
{
    UINT8    cmd;

    // Disable the Receiver and re-enable it
    cmd = EDRV_REGB_READ(RW_REGB_COMMAND);
    EDRV_REGB_WRITE(RW_REGB_COMMAND, cmd & ~RW_REGB_COMMAND_RX_EN);
    EDRV_REGB_WRITE(RW_REGB_COMMAND, cmd);

    // Set receive configuration register
    EDRV_REGDW_WRITE(RW_REGDW_RCR, RW_REGDW_RCR_DEF);
}

/// \}
