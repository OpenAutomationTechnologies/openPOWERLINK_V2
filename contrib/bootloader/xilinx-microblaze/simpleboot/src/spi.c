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
#include "global.h"
#include "spi.h"
#include "util.h"
#include "stdio.h"

#if SPI_FLASH == 1

void spi_init(SPI_INST_t *spi_inst)
{
    /*
     * Reset the SPI device to leave it in a known good state
     */
    Spi_mReset();

    Spi_mSetControlReg(spi_inst->control);

    #if DEBUG_SPI
    print("Control   reg 0x"); putnum(Spi_mGetControlReg()); print("\r\n");
    print("Status    reg 0x"); putnum(Spi_mGetStatusReg()); print("\r\n");
    print("Slave Sel reg 0x"); putnum(Spi_mGetSlaveSelectReg()); print("\r\n");
    #endif

    /*
     * Don't need interrupt
     */
    Spi_mIntrGlobalDisable();

}


int spi_fast_read(SPI_INST_t *spi_inst, u32 Address, u8 *RxBuf, u32 Len)
{
    volatile u32 Reg32;
    u32 Count = Len + FAST_READ_DUMMY_NUM;
    u8 *pRx = RxBuf;
    u32 ReadCount = 0;
    u8 TxBuf[4];
    u8 TxCount = 0;
#if DEBUG_PRINT_FLASH_DATA
    u32 PrintCount;
#endif

    (void)spi_inst;

#if BIT_SWAP_ON
        TxBuf[0] = BitReOrder8(FAST_READ);
        TxBuf[1] = BitReOrder8((u8)(Address >> 16));
        TxBuf[2] = BitReOrder8((u8)(Address >> 8));
        TxBuf[3] = BitReOrder8((u8)Address);
#else
        TxBuf[0] = FAST_READ;
        TxBuf[1] = (u8)(Address >> 16);
        TxBuf[2] = (u8)(Address >> 8);
        TxBuf[3] = (u8)Address;
#endif

    /*
     * Set the slave select register to select the device on the SPI before
     * starting the transfer of data.
     */
    Spi_mSetSlaveSelectReg(0);

    do{
        /*
         * Fill up tx fifo
         */
        while(1){
            Reg32 = Spi_mGetStatusReg();
            if( ((Reg32 & SR_TX_FULL_MASK) == 0) && Count>0 ){
                if(TxCount < 4){
                    Spi_mWriteTxData(TxBuf[TxCount]);
                }
                else{
                    Spi_mWriteTxData(0xff);
                }
                Count--;
                TxCount++;
            }
            else{
                break;
            }
        }

        /*
         * Start the transfer by not inhibiting the transmitter and
         * enabling the device
         */
        Reg32 = Spi_mGetControlReg() & (~CR_TRANS_INHIBIT_MASK);
        Spi_mSetControlReg(Reg32 | CR_ENABLE_MASK);

        /*
         * Wait for the transfer to be done by polling the transmit
         * empty status bit
         */
        do {
            Reg32 = Spi_mGetStatusReg();
        } while ((Reg32 & SR_TX_EMPTY_MASK) == 0);

        /*
         * Read data from the fifo
         */
        while(1)
        {
            Reg32 = Spi_mGetStatusReg();
            if((Reg32 & SR_RX_EMPTY_MASK) == 0)
            {
                Reg32 = Spi_mReadRxData();
                if(ReadCount >= FAST_READ_DUMMY_NUM){ // bypass dummy bytes
                    *pRx++ = Reg32;
                }
                ReadCount++;
                #if DEBUG_PRINT_FLASH_DATA
                if(PrintCount%8==0 && PrintCount>=8){
                    print("\r\n");
                }
                putnum(Reg32);
                print(" ");
                PrintCount++;
                #endif
            }
            else{
                break;
            }
        }
        #if DEBUG_PRINT_FLASH_DATA
        print("\r\n");
        #endif

        /*
         * Stop the transfer (hold off automatic sending) by inhibiting
         * the transmitter.
         */
        Reg32 = Spi_mGetControlReg();
        Spi_mSetControlReg(Reg32 | CR_TRANS_INHIBIT_MASK);

    }while(Count > 0);


    /*
     * Select the slave on the SPI bus
     */
    Spi_mSetSlaveSelectReg(1);

    /*
     * Disable SPI master
     */
    Reg32 = Spi_mGetControlReg();
    Spi_mSetControlReg(Reg32 & ~CR_ENABLE_MASK);

#if DEBUG_SPI
    if(ReadCount != Len+FAST_READ_DUMMY_NUM)
    {
        print("length not match ");
        putnum(ReadCount);
        print(" ");
        putnum(Len+FAST_READ_DUMMY_NUM);
        print("\r\n");

        return XST_FAILURE;
    }
#endif

    return XST_SUCCESS;

}

#endif //SPI_FLASH == 1





