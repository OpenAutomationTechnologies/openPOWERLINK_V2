/*********************************************************************
 *
 * Copyright (C) 2004  FREESCALE, Inc.
 *  FREESCALE, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use
 *  the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of FREESCALE, Inc. This
 *  software is provided on an "AS IS" basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, FREESCALE
 *  DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED, INCLUDING
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
 *  PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH REGARD TO THE
 *  SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF) AND ANY
 *  ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL FREESCALE BE LIABLE FOR ANY DAMAGES WHATSOEVER (INCLUDING
 *  WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS
 *  INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY
 *  LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  FREESCALE assumes no responsibility for the maintenance and support
 *  of this software
 ********************************************************************/

/*
 * File:        MCD_dma.h
 * Purpose:     Main header file for multi-channel DMA API.
 *
 * Notes:
 *
 * Modifications:
 */
#ifndef _MCD_API_H
#define _MCD_API_H

/*
 * Turn Execution Unit tasks ON (#define) or OFF (#undef)
 */
#undef MCD_INCLUDE_EU

/*
 * Number of DMA channels
 */
#define NCHANNELS 16

/*
 * Total number of variants
 */
#ifdef MCD_INCLUDE_EU
#define NUMOFVARIANTS   6
#else
#define NUMOFVARIANTS   4
#endif

/*
 * Defines for the debug control register's functions
 * TBD - These may be redefined
 */
#define DEBUGAUTOARM        (0x8000) /* automatically rearm the comparators */
#define DEBUGENABLECOMPARES (0x4000) /* enable "internal" debug activity    */
#define DEBUGBREAKONCOMPARE (0x0002) /* breakpoint on comparator match      */
#define DEBUGCOMPARE1TASK   (0x2000) /* have comparator 1 look for a task # */
#define DEBUGCRENABLEVALUE  (DEBUGAUTOARM        | \
                             DEBUGENABLECOMPARES | \
                             DEBUGBREAKONCOMPARE | \
                             DEBUGCOMPARE1TASK)
#define DEBUGCRDISABLEVALUE (DEBUGAUTOARM        | \
                             DEBUGBREAKONCOMPARE | \
                             DEBUGCOMPARE1TASK)
#define DEBUGKILLALLSTATUS  (0xFFFFFFFF)
#define REGINITIATOR        (0x01)

/*
 * Offset to context save area where progress is stored
 */
#define CSAVE_OFFSET        10

/*
 * Define sizes of the various tables
 */
#define TASK_TABLE_SIZE     (NCHANNELS*32)
#define VAR_TAB_SIZE        (128)
#define CONTEXT_SAVE_SIZE   (128)
#define FUNCDESC_TAB_SIZE   (256)

#ifdef MCD_INCLUDE_EU
#define FUNCDESC_TAB_NUM    16
#else
#define FUNCDESC_TAB_NUM    1
#endif


#ifndef DEFINESONLY

/*
 * Portability typedefs
 */
typedef int s32;
typedef unsigned int u32;
typedef short s16;
typedef unsigned short u16;
typedef char s8;
typedef unsigned char u8;

/*
 * These structures represent the internal registers of the
 * multi-channel DMA
 */
struct ptd_ivr_st {
   u8  int_vector1;
   u8  int_vector2;
   u16 ptd_cntl;
};/*TBD will be removed */
struct dmaRegs_s {
   u32 taskbar;         /* task table base address register */
   u32 currPtr;
   u32 endPtr;
   u32 varTablePtr;
#if 1 /* will be replaced with code below */
   struct ptd_ivr_st ptd_ivr;
#else
   u16 dma_rsvd0;
   u16 ptdCntl;
#endif
   u32 intPending;      /* interrupt pending register */
   u32 intMask;         /* interrupt mask register */
   u16 taskControl[16]; /* task control registers */
   u8  priority[32];    /* priority registers */
   u32 initiatorMux;    /* initiator mux control */
   u32 taskSize0;       /* task size control register 0. */
   u32 taskSize1;       /* task size control register 1. */
   u32 dma_rsvd1;       /* reserved */
   u32 dma_rsvd2;       /* reserved */
   u32 debugComp1;      /* debug comparator 1 */
   u32 debugComp2;      /* debug comparator 2 */
   u32 debugControl;    /* debug control */
   u32 debugStatus;     /* debug status */
   u32 ptdDebug;        /* priority task decode debug */
   u32 eu0_reserved[7]; /* punt these TBD and add u32 dma_rsvd3[31] */
   u32 eu1_reserved[8];
   u32 eu2_reserved[8];
   u32 eu3_reserved[8];
};
typedef volatile struct dmaRegs_s dmaRegs;

#endif

/*
 * PTD contrl reg bits
 */
#define PTD_CNTL_TSK_PRI            0x8000
#define PTD_CNTL_COMM_PREFETCH      0x0001

/*
 * Task Control reg bits and field masks
 */
#define TASK_CONTROL_EN             0x8000
#define TASK_CONTROL_VALID          0x4000
#define TASK_CONTROL_ALWAYS         0x2000
#define TASK_CONTROL_INIT_MASK      0x1f00
#define TASK_CONTROL_ASTRT          0x0080
#define TASK_CONTROL_HIPRITSKEN     0x0040
#define TASK_CONTROL_HLDINITNUM     0x0020
#define TASK_CONTROL_ASTSKNUM_MASK  0x000f

/*
 * Priority reg bits and field masks
 */
#define PRIORITY_HLD                0x80
#define PRIORITY_PRI_MASK           0x07

/*
 * Debug Control reg bits and field masks
 */
#define DBG_CTL_BLOCK_TASKS_MASK    0xffff0000
#define DBG_CTL_AUTO_ARM            0x00008000
#define DBG_CTL_BREAK               0x00004000
#define DBG_CTL_COMP1_TYP_MASK      0x00003800
#define DBG_CTL_COMP2_TYP_MASK      0x00000070
#define DBG_CTL_EXT_BREAK           0x00000004
#define DBG_CTL_INT_BREAK           0x00000002

/*
 * Defines for general intuitiveness
 */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/*
 * General return values
 */
#define MCD_OK                  0
#define MCD_ERROR               -1
#define MCD_TABLE_UNALIGNED     -2
#define MCD_CHANNEL_INVALID     -3

/*
 * MCD_initDma input flags
 */
#define MCD_RELOC_TASKS         0x00000001
#define MCD_NO_RELOC_TASKS      0x00000000
#define MCD_COMM_PREFETCH_EN    0x00000002  /* Commbus Prefetching - MCF547x/548x ONLY */

/*
 * MCD_dmaStatus Status Values for each channel
 */
#define MCD_NO_DMA  1 /* No DMA has been requested since reset */
#define MCD_IDLE    2 /* DMA active, but the initiator is currently inactive */
#define MCD_RUNNING 3 /* DMA in progress and transfers are happening */
#define MCD_PAUSED  4 /* DMA active but it is currently paused */
#define MCD_HALTED  5 /* the most recent DMA has been killed with MCD_killTask() */
#define MCD_DONE    6 /* the most recent DMA has completed. */


/*
 * MCD_startDma parameter defines
 */

/* Constants for the funcDesc parameter: */

/* Byte swapping: */
#define MCD_NO_BYTE_SWAP    0x00045670  /* to disable byte swapping. */
#define MCD_BYTE_REVERSE    0x00076540  /* to reverse the bytes of each u32 of the DMAed data. */
#define MCD_U16_REVERSE     0x00067450
 /* to reverse the 16-bit halves of each 32-bit data value being DMAed.*/
#define MCD_U16_BYTE_REVERSE    0x00054760
 /* to reverse the byte halves of each 16-bit half of each 32-bit data value DMAed */
#define MCD_NO_BIT_REV  0x00000000  /* do not reverse the bits of each byte DMAed. */
#define MCD_BIT_REV     0x00088880  /* reverse the bits of each byte DMAed */
/* CRCing: */
#define MCD_CRC16       0xc0100000  /* to perform CRC-16 on DMAed data. */
#define MCD_CRCCCITT    0xc0200000  /* to perform CRC-CCITT on DMAed data. */
#define MCD_CRC32       0xc0300000  /* to perform CRC-32 on DMAed data. */
#define MCD_CSUMINET    0xc0400000  /* to perform internet checksums on DMAed data.*/
#define MCD_NO_CSUM     0xa0000000  /* to perform no checksumming. */

/* Constants for the flags parameter: */
/* to perform a single-buffer DMA as defined by the call parameters */
#define MCD_TT_FLAGS_RL   0x00000001 /* Read line */
#define MCD_TT_FLAGS_CW   0x00000002 /* Combine Writes */
#define MCD_TT_FLAGS_SP   0x00000004 /* Speculative prefetch(XLB) MCF547x/548x ONLY  */
#define MCD_TT_FLAGS_MASK 0x000000ff

#define MCD_TT_FLAGS_DEF  (MCD_TT_FLAGS_RL | MCD_TT_FLAGS_CW)

#define MCD_SINGLE_DMA  0x00000100 /* Unchained DMA */
#define MCD_CHAIN_DMA              /* TBD */
#define MCD_EU_DMA                 /* TBD */
#define MCD_FECTX_DMA   0x00001000 /* FEC TX ring DMA */
#define MCD_FECRX_DMA   0x00002000 /* FEC RX ring DMA */


/* these flags are valid for MCD_startDma and the chained buffer descriptors */
#define MCD_BUF_READY   0x80000000 /* indicates that this buffer is now under the DMA's control */
#define MCD_LOOP        0x40000000 /* to repeat the stated DMA indefinitely. TBD*/
#define MCD_WRAP        0x20000000 /* to tell the FEC Dmas to wrap to the first BD */
#define MCD_INTERRUPT   0x10000000 /* to generate an interrupt after completion of the DMA. */
#define MCD_END_FRAME   0x08000000 /* tell the DMA to end the frame when transferring
                                        last byte of data in buffer */

/* Defines for the FEC buffer descriptor control/status word*/
#define MCD_FEC_BUF_READY   0x8000
#define MCD_FEC_WRAP        0x2000
#define MCD_FEC_INTERRUPT   0x1000
#define MCD_FEC_END_FRAME   0x0800

/* TBD - NEEDS to /WILL be redefined when EU stuff is fully implemented*/
#define MCD_CRC_RESTART 0x00000002
/* to empty out the accumulated checksum prior to performing the DMA. */

/*
 * Defines for Byte Swapping
 */
#define MCD_BYTE_SWAP_KILLER    0xFFF8888F
#define MCD_NO_BYTE_SWAP_ATALL  0x00040000

/* 
 * Execution Unit Identifiers
 */
#define MAC 0
#define LUAC 1
#define CRC 2
#define LURC 3

/*
 * Task Identifiers
 */
#define TASK_CHAINNOEU  0
#define TASK_SINGLENOEU 1
#ifdef MCD_INCLUDE_EU
#define TASK_CHAINEU    2
#define TASK_SINGLEEU   3
#define TASK_FECRX      4
#define TASK_FECTX      5
#else
#define TASK_CHAINEU    0
#define TASK_SINGLEEU   1
#define TASK_FECRX      2
#define TASK_FECTX      3
#endif

#define NOEU1 (MCD_NO_BYTE_SWAP | MCD_NO_BIT_REV | MCD_NO_CSUM)
#define NOEU2 (MCD_NO_BYTE_SWAP | MCD_NO_CSUM)

/* 
 * Three different cases for destination and source. 
 */
#define MINUS1          -1
#define ZERO            0
#define PLUS1           1

#ifndef DEFINESONLY



/* Task Table Entry struct*/
typedef struct {
    u32 TDTstart;   /* task descriptor table start */
    u32 TDTend;     /* task descriptor table end */
    u32 varTab;     /* variable table start */
    u32 FDTandFlags;    /* function descriptor table start and flags */
    volatile u32 descAddrAndStatus;
    volatile u32 modifiedVarTab;
    u32 contextSaveSpace;   /* context save space start */
    u32 literalBases;
} TaskTableEntry;


/* Chained buffer descriptor */
typedef volatile struct MCD_bufDesc_struct MCD_bufDesc;
struct MCD_bufDesc_struct {
   u32 flags;         /* flags describing the DMA */
   u32 csumResult;    /* checksum from checksumming performed since last checksum reset */
   s8  *srcAddr;      /* the address to move data from */
   s8  *destAddr;     /* the address to move data to */
   s8  *lastDestAddr; /* the last address written to */
   u32 dmaSize;       /* the number of bytes to transfer independent of the transfer size */
   MCD_bufDesc *next; /* next buffer descriptor in chain */
   u32 info;          /* private information about this descriptor;  DMA does not affect it */
};

/* Progress Query struct */
typedef volatile struct MCD_XferProg_struct {
   s8 *lastSrcAddr;         /* the most-recent or last, post-increment source address */
   s8 *lastDestAddr;        /* the most-recent or last, post-increment destination address */
   u32  dmaSize;              /* the amount of data transferred for the current buffer */
   MCD_bufDesc *currBufDesc;  /* pointer to the current buffer descriptor being DMAed */
} MCD_XferProg;


/* FEC buffer descriptor */
typedef volatile struct MCD_bufDescFec_struct {
    u16 statCtrl;
    u16 length;
    u32 dataPointer;
} MCD_bufDescFec;

/* 
 * Structure to remember which variant is on which channel 
 * TBD- need this?
 */
typedef struct MCD_remVariants_struct MCD_remVariant;
struct MCD_remVariants_struct 
{
   int remDestRsdIncr[NCHANNELS];  /* -1,0,1 */
   int remSrcRsdIncr[NCHANNELS];   /* -1,0,1 */
   s16 remDestIncr[NCHANNELS];     /* DestIncr */
   s16 remSrcIncr[NCHANNELS];      /* srcIncr */
   u32 remXferSize[NCHANNELS];     /* xferSize */
};



/***** Function Prototypes *************************************************/

int MCD_startDma (
   int channel,   /* the channel on which to run the DMA */
   s8  *srcAddr,  /* the address to move data from, or buffer-descriptor address */
   s16 srcIncr,   /* the amount to increment the source address per transfer */
   s8  *destAddr, /* the address to move data to */
   s16 destIncr,  /* the amount to increment the destination address per transfer */
   u32 dmaSize,   /* the number of bytes to transfer independent of the transfer size */
   u32 xferSize,  /* the number bytes in of each data movement (1, 2, or 4) */
   u32 initiator, /* what device initiates the DMA */
   int priority,  /* priority of the DMA */
   u32 flags,     /* flags describing the DMA */
   u32 funcDesc   /* a description of byte swapping, bit swapping, and CRC actions */
);


/* MCD_initDma() initializes the DMA API and its associated registers inside
   DMA controller.  It also moves the DMA algorithms the on-chip SRAM.
*/
int MCD_initDma (dmaRegs *sDmaBarAddr, void *taskTableDest, u32 flags);

/* MCD_dmaStatus() returns the status of the DMA on the requested channel.
*/
int MCD_dmaStatus (int channel);


/* MCD_XferProgrQuery() upon completing or after aborting a DMA, or while the DMA is in
   progress, this function returns the first DMA-destination address not (or not yet) used
   in the DMA.
*/
int MCD_XferProgrQuery (int channel, MCD_XferProg *progRep);


/* MCD_killDma() terminates the DMA task running on the channel specified by the parameter.
   A DMA may be killed from any state, including paused state, and it always goes to the
   MCD_HALTED state even if it is killed while in the MCD_NO_DMA or MCD_IDLE states.
*/
int MCD_killDma (int channel);


/* MCD_continDma() continues a DMA that has already been started but may have stopped.  A
   common use of this call is in adding buffers to a buffer ring.  A common sequence of
   steps is as follows:
   1.  Mark the buffer directly before the buffer you wish to manipulate as not ready,
   2.  Add or remove buffers, or do whatever else to the chain,
   3.  Mark that buffer as ready again, then
   4.  Call MCD_continDma() to make the DMA continue running if it stopped from encountering
       the momentarily unready buffer descriptor.
*/
int MCD_continDma (int channel);


/* MCD_pauseDma() and MCD_resumeDma() pause or resume a DMA in progress.
   It's extremely important to not pause more than one DMA channel at a time.
*/
int MCD_pauseDma (int channel);

int MCD_resumeDma (int channel);


/* returns the checksum value after performing a non-chained DMA. */
int MCD_csumQuery (int channel, u32 *csum);

int MCD_getCodeSize(void);

int MCD_getVersion(char **longVersion);


/* These definitions are used internally by the API code: */

/* from dmaApi.h - some of this can probably go away TBD */
/*#define MAX_VAR_TABLE_SIZE    48
#define VAR_TABLE_SIZE      MAX_VAR_TABLE_SIZE
#define SC_VARTAB_SIZE      VAR_TABLE_SIZE*/




#define MCD_SET_VAR(taskTab,idx,value) ((u32 *)(taskTab)->varTab)[idx] = value
   /* Note that MCD_SET_VAR() is invoked many times in firing up a DMA function,
      so I'm avoiding surrounding it with "do {} while(0)" */

/*typedef void (*ptr2Func)( int*, short, short, int, int, int, int, int, short, int *,
                          volatile TaskTableEntry *, int);*/



#endif  /* DEFINESONLY */

#endif /* _MCD_API_H */
