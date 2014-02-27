/*
 * Copyright (c) 2010 Avnet.  All rights reserved.
 *
 * Avnet (HK)
 * AVNET IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, AVNET IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION
 * AVNET EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef __SPI_H__
#define __SPI_H__

/***************************** Include Files *********************************/
#include "xparameters.h"
#include "xstatus.h"
#include "xbasic_types.h"
#include "xio.h"

#if SPI_FLASH == 1

/************************** Constant Definitions *****************************/
#define SPI_BASEADDRESS XPAR_SPI_FLASH_BASEADDR

/**************************** Type Definitions *******************************/
typedef struct {
    u32 control;
} SPI_INST_t;


/************************** Constant Definitions *****************************/

/*
 * Register offset
 */
#define DGIER_OFFSET       0x1C             /**< Global Intr Enable Reg */
#define IISR_OFFSET        0x20             /**< Interrupt status Reg */
#define IIER_OFFSET        0x28             /**< Interrupt Enable Reg */
#define SRR_OFFSET         0x40             /**< Software Reset register */

#if XPAR_MICROBLAZE_ENDIANNESS == 0     /* MICROBLAZE is big endian */
  #define CR_OFFSET         (0x60 + 0x2)    /**< 16-bit Control */
  #define SR_OFFSET         (0x64 + 3)        /**< 8 Bit Status Register */
  #define DTR_OFFSET        (0x68 + 3)        /**< Data transmit */
  #define DRR_OFFSET        (0x6C + 3)        /**< Data receive */
  #define SSR_OFFSET        0x70            /**< 32-bit slave select */
  #define TFO_OFFSET        (0x74 + 3)        /**< 8 bit Tx FIFO occupancy */
  #define RFO_OFFSET        (0x78 + 3)        /**< 8 bit Rx FIFO occupancy */
#else                               /* MICROBLAZE is little endian */
  #define CR_OFFSET       (0x60)      /**< 16-bit Control */
  #define SR_OFFSET       (0x64)      /**< 8 Bit Status Register */
  #define DTR_OFFSET      (0x68)      /**< Data transmit */
  #define DRR_OFFSET      (0x6C)      /**< Data receive */
  #define SSR_OFFSET      (0x70)      /**< 32-bit slave select */
  #define TFO_OFFSET      (0x74)      /**< 8 bit Tx FIFO occupancy */
  #define RFO_OFFSET      (0x78)      /**< 8 bit Rx FIFO occupancy */
#endif

/*
 * Reset register bit definition
 */
#define SRR_RESET_MASK            0xA


/*
 * Control register bit definition
 */
#define CR_LOOPBACK_MASK          0x1   /**< Local loopback mode */
#define CR_ENABLE_MASK            0x2   /**< System enable */
#define CR_MASTER_MODE_MASK       0x4   /**< Enable master mode */
#define CR_CLK_POLARITY_MASK      0x8   /**< Clock polarity high or low */
#define CR_CLK_PHASE_MASK         0x10  /**< Clock phase 0 or 1 */
#define CR_TXFIFO_RESET_MASK      0x20  /**< Reset transmit FIFO */
#define CR_RXFIFO_RESET_MASK      0x40  /**< Reset receive FIFO */
#define CR_MANUAL_SS_MASK         0x80  /**< Manual slave sel assert */
#define CR_TRANS_INHIBIT_MASK     0x100 /**< Master transaction inhibit */
#define CR_LSB_MSB_FIRST_MASK     0x200


/*
 * Status register bit definition
 *
 */
#define SR_RX_EMPTY_MASK           0x1  /**< Receive Reg/FIFO is empty */
#define SR_RX_FULL_MASK            0x2  /**< Receive Reg/FIFO is full */
#define SR_TX_EMPTY_MASK           0x4  /**< Transmit Reg/FIFO is empty */
#define SR_TX_FULL_MASK            0x8  /**< Transmit Reg/FIFO is full */
#define SR_MODE_FAULT_MASK         0x10 /**< Mode fault error */

/*
 * Tx fifo
 */
#define TFO_MASK        0x1F

/*
 * Rx fifo
 */
#define RFO_MASK        0x1F


/***************** Macros (Inline Functions) Definitions *********************/
#define Spi_mReset() \
    XIo_Out32(SPI_BASEADDRESS + SRR_OFFSET,     \
        SRR_RESET_MASK)

#define Spi_mIntrGlobalEnable()            \
    XIo_Out32(SPI_BASEADDRESS + DGIER_OFFSET, \
        GINTR_ENABLE_MASK)

#define Spi_mIntrGlobalDisable()                 \
    XIo_Out32(SPI_BASEADDRESS + DGIER_OFFSET, 0)

#define Spi_mSetControlReg(Mask) \
    XIo_Out16(SPI_BASEADDRESS + CR_OFFSET, (Mask))

#define Spi_mGetControlReg() \
    XIo_In16(SPI_BASEADDRESS + CR_OFFSET)

#define Spi_mGetStatusReg() \
    XIo_In8(SPI_BASEADDRESS + SR_OFFSET)

#define Spi_mSetSlaveSelectReg(Mask) \
    XIo_Out32(SPI_BASEADDRESS + SSR_OFFSET, (Mask))

#define Spi_mGetSlaveSelectReg() \
    XIo_In32(SPI_BASEADDRESS + SSR_OFFSET)

#define Spi_mEnable() \
{ \
    u16 Control; \
    Control = Spi_mGetControlReg(); \
    Control |= CR_ENABLE_MASK; \
    Control &= ~CR_TRANS_INHIBIT_MASK; \
    Spi_mSetControlReg(Control); \
}

#define Spi_mDisable() \
    Spi_mSetControlReg( Spi_mGetControlReg() & ~CR_ENABLE_MASK)

#define Spi_mGetTxFiFoOcy() \
    XIo_In8(SPI_BASEADDRESS + TFO_OFFSET);

#define Spi_mGetRxFiFoOcy() \
    XIo_In8(SPI_BASEADDRESS + RFO_OFFSET);

#define Spi_mWriteTxData(data) \
    XIo_Out8(SPI_BASEADDRESS + DTR_OFFSET, (data));

#define Spi_mReadRxData() \
    XIo_In8(SPI_BASEADDRESS + DRR_OFFSET);


#define XSP_SR_RESET_STATE       0x5       /* Default to Tx/Rx reg empty */
#define XSP_CR_RESET_STATE       0x180
#define SPI_TX_FIFO_LEN          16
#define SPI_RX_FIFO_LEN          16

#define RDID         0x9f
#define RDSR         0x05
#define FAST_READ    0x0B

#define FAST_READ_DUMMY_NUM     5


/************************** Function Prototypes ******************************/
void spi_init(SPI_INST_t *spi_inst);
int spi_fast_read(SPI_INST_t *spi_inst, u32 address, u8 *RxBuf, u32 Len);
u8 BitReOrder8(u8 data);

#endif // SPI_FLASH == 1

#endif // __SPI_H__







