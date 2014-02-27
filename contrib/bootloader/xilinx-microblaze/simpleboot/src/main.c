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
#include "xparameters.h"
#include <stdio.h>
#include "global.h"
#include "spi.h"
#include "util.h"
#include "mb_interface.h"
#include "xil_cache.h"

/************************** Constant Definitions *****************************/

/*
 * Target memory defines
 */
#ifdef XPAR_MCB_DDR2_MPMC_BASEADDR
#define MEM_BASE_ADDR       XPAR_MCB_DDR2_MPMC_BASEADDR
#elif XPAR_MCB_DDR2_S0_AXI_BASEADDR
#define MEM_BASE_ADDR       XPAR_MCB_DDR2_S0_AXI_BASEADDR
#elif defined(XPAR_MCB_DDR3_MPMC_BASEADDR)
#define MEM_BASE_ADDR       XPAR_MCB_DDR3_MPMC_BASEADDR
#elif defined(XPAR_MCB_DDR3_S0_AXI_BASEADDR)
#define MEM_BASE_ADDR       XPAR_MCB_DDR3_S0_AXI_BASEADDR
#elif defined(XPAR_MCB3_LPDDR_MPMC_BASEADDR)
#define MEM_BASE_ADDR       XPAR_MCB3_LPDDR_MPMC_BASEADDR
#elif defined(XPAR_MCB_DDR3_PIM0_BASEADDR)
#define MEM_BASE_ADDR       XPAR_MCB_DDR3_PIM0_BASEADDR
#elif defined(XPAR_MCB3_LPDDR_S0_AXI_BASEADDR)
#define MEM_BASE_ADDR       XPAR_MCB3_LPDDR_S0_AXI_BASEADDR
#elif defined(XPAR_SRAM256KX32_MEM0_BASEADDR)
#define MEM_BASE_ADDR       XPAR_SRAM256KX32_MEM0_BASEADDR
#elif defined(XPAR_SRAM256KX32_S_AXI_MEM0_BASEADDR)
#define MEM_BASE_ADDR       XPAR_SRAM256KX32_S_AXI_MEM0_BASEADDR
#endif

#define PROGRAM_START_ADDRESS     MEM_BASE_ADDR

/*
 * Flash defines
 */
#if SPI_FLASH == 1
    //#define FLASH_END_ADDRESS     0xfffff // for M25P80
    #define FLASH_START_ADDRESS     0x0
    #define FLASH_END_ADDRESS       0x7fffff // for W25Q64BV
#else
  #ifdef XPAR_LINEAR_FLASH_S_AXI_MEM0_BASEADDR
    #define FLASH_START_ADDRESS     XPAR_LINEAR_FLASH_S_AXI_MEM0_BASEADDR
    #define FLASH_END_ADDRESS       XPAR_LINEAR_FLASH_S_AXI_MEM0_HIGHADDR
  #elif defined(XPAR_LINEAR_FLASH_MEM0_BASEADDR)
    #define FLASH_START_ADDRESS     XPAR_LINEAR_FLASH_MEM0_BASEADDR
    #define FLASH_END_ADDRESS       XPAR_LINEAR_FLASH_MEM0_HIGHADDR
  #endif
#endif



#if SPI_FLASH == 1
  /*
   *  For SPI bitstream and application are on same flash
   *  Enter the size of  the bitstream to speed up the syn search for the app code
   */
  #define MCS_OFFSET (FLASH_START_ADDRESS + 0x50000)    // offset for lx9
#else
  #define MCS_OFFSET (FLASH_START_ADDRESS + 0x0)     // parallel flash is used! (No offset needed)
#endif


/*
 * bit swap means bit[7:0] swapped to bit[0:7] before programmed to the flash
 */
#if BIT_SWAP_ON == 1
  #define SW_SYN_BYTE1 0xF9
  #define SW_SYN_BYTE2 0xF1
  #define SW_SYN_BYTE3 0xF5
  #define SW_SYN_BYTE4 0xFD
#else
  #define SW_SYN_BYTE1 0x9F
  #define SW_SYN_BYTE2 0x8F
  #define SW_SYN_BYTE3 0xAF
  #define SW_SYN_BYTE4 0xBF
#endif

#if SPI_FLASH == 1
#define SYNC_RX_LEN     SPI_RX_FIFO_LEN
#define SYNC_TX_LEN     SPI_TX_FIFO_LEN
#else
#define SYNC_RX_LEN     16
#define SYNC_TX_LEN     16
#endif

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
static Xuint32 DetectSync(void);
Xuint32 load_section(Xuint32 SourceAddress, Xuint32 *SectStrtAddr, u8 first_section);
#if SPI_FLASH == 0
  void* mycpy(void* dest, const void* src, size_t count);
#endif


/************************** Variable Definitions *****************************/
#if SPI_FLASH == 1
SPI_INST_t spi_inst;
#endif //SPI_FLASH == 1

u8 TxBuf[SYNC_TX_LEN];
u8 RxBuf[SYNC_RX_LEN+3] = { 0 };


//static Xuint32 ProgramStartAddress;
void (*app) ();


/****************************** Program **************************************/
int main(void){

    u32 SrcAddress;
    static u32 SectionAddress;
    u8 first_section = 1;

    microblaze_disable_interrupts();
    Xil_DCacheDisable();
    Xil_ICacheDisable();

#if DEBUG
    print("\r\nSystem booting, please wait\r\n");
#endif

#if SPI_FLASH == 1
    #if BIT_SWAP_ON == 1
    spi_inst.control = (Spi_mGetControlReg() | CR_LSB_MSB_FIRST_MASK | CR_MASTER_MODE_MASK);
    #else
    spi_inst.control = (Spi_mGetControlReg() | CR_MASTER_MODE_MASK);
    #endif

    /* init SPI */
    spi_init(&spi_inst);
#endif

    /* Detect Sync */
#if DEBUG
    print("Detect SYNC\r\n");
#endif

    SrcAddress = DetectSync();
    if(!SrcAddress)
    {
#if DEBUG
        print("No SYNC detected\r\n");
#endif
        goto Exit;
    }

#if DEBUG
    print("New source address after SYNC 0x");
    putnum(SrcAddress);
    print("\r\n");
#endif

    do{

        SrcAddress = load_section(SrcAddress, &SectionAddress, first_section);

        if(SrcAddress == 0 && first_section == 0){
            break;
        }
        first_section = 0;
        #if DEBUG
        print("New source address 0x");
        putnum(SrcAddress);
        print("\r\n");
        #endif
    } while(1);


    app = (void (*) ())PROGRAM_START_ADDRESS;
#if DEBUG
    print("Run application\r\n");
#endif
    app(); // jump to application, no return

    // should not reach this code here!

Exit:
#if DEBUG
    print("FATAL: boot failed\r\n");
#endif

    return -1;
}

static Xuint32 DetectSync(void){

    u32 address = MCS_OFFSET; // start search address
    u32 syn_address = 0;
    u32 cmp_address;
    u32 read_len;
    u32 i;


    /* Search for SYNC - 0xF9F1F5FD (0x9F8FAFBF before bit swap)
     *
     * Note:
     *        1.     16bit readback from platform flash will swap hi-low byte,
     *               so the SYNC readback should be 0xF1F9 & 0xFDF5
     *        2.     Assume SYNC words are 16bit aligned
     *        3.     If spi flash, can ignore (1) & (2)
     *
     */

    while(address < FLASH_END_ADDRESS)
    {
        if((address+SYNC_RX_LEN) > FLASH_END_ADDRESS)
        {
            read_len = FLASH_END_ADDRESS - address;
        }
        else {
            read_len = SYNC_RX_LEN;
        }

#if SPI_FLASH == 1
        spi_fast_read(&spi_inst, address, RxBuf+3, read_len);
#else
        mycpy(RxBuf+3, (void *)address, read_len);
#endif

        cmp_address = address-3;
        for(i=0;i<read_len;i++)
        {
            if((RxBuf[i] == SW_SYN_BYTE1) && (RxBuf[i+1] == SW_SYN_BYTE2) \
                && (RxBuf[i+2] == SW_SYN_BYTE3) && (RxBuf[i+3] == SW_SYN_BYTE4) )
            {
                syn_address = cmp_address + i + 4;    // point to the address just after
                                                    // syn word
                address = FLASH_END_ADDRESS; // force to end address
                break;
            }
        }

        RxBuf[0] = RxBuf[SYNC_RX_LEN];        // copy the last byte to buffer head
        RxBuf[1] = RxBuf[SYNC_RX_LEN+1];      // for next cycle comparison
        RxBuf[2] = RxBuf[SYNC_RX_LEN+2];
        address += read_len;
    }

    return syn_address;
}



/* Load program from flash to sdram
 *
 * Note:     1.     16bit readback from platform flash will swap hi-low byte,
 *                  so need byte re-order
 *           2.     Each byte is bit-swapped, so need bit re-order
 *           3.     Assume program section all start at 16bit boundary
 *           4.     Assume program size is even
 *           5.     If spi flash, ignore (1) & (4)
 */

u32 load_section(u32 SourceAddress, u32 *SectStrtAddr, u8 first_section){

    u32 SectionStartAddress;
    u32 SectionSizeByte;
#if SPI_FLASH == 1 && BIT_SWAP_ON == 1
    //volatile u16 *dptr16;
    volatile u8 *dptr8;
    volatile u8 spi_data;
#endif
    volatile u16 data[4];
    u32 address;

    /* Get program start address */
    address = SourceAddress;

#if SPI_FLASH == 1
    spi_fast_read(&spi_inst, address, (u8 *)&data[0], 8);
#else
    mycpy(&data, (void *)address, 8);
#endif

    address += 4;


    if((data[0] == 0) && (data[1] == 0) && (data[2] == 0x0000) &&
            (data[3] == 0x0000) && (first_section == 0))
    {
        #if DEBUG
            print("Section end detected\r\n");
        #endif

        return 0; // no more valid section
    }

    SectionStartAddress = BitReOrder16(data[0]);
    SectionStartAddress = (SectionStartAddress << 16) | BitReOrder16(data[1]);
    *SectStrtAddr = SectionStartAddress;

#if DEBUG
    print("Section start address 0x");
    putnum(SectionStartAddress);
    print("\r\n");

    /* Get program size */
    print("size hi: 0x");
#endif

#if SPI_FLASH == 1
    spi_fast_read(&spi_inst, address, (u8 *)&data[0], 4);
#else
    mycpy(&data, (void *)address, 4);
#endif
    address += 4;

#if DEBUG
    putnum(data[0]);
    print(" size lo: 0x");
    putnum(data[1]);
    print("\r\n");
#endif

    SectionSizeByte = BitReOrder16(data[0]);
    SectionSizeByte = (SectionSizeByte << 16) | BitReOrder16(data[1]);

#if DEBUG
    print("Section size 0x");
    putnum(SectionSizeByte);
    print("\r\n");

    if((address + SectionSizeByte) > FLASH_END_ADDRESS)
    {
        print("Failed: Invalid section size\r\n");

        return 0;
    }
#endif

    /* Load program to sdram */
#if SPI_FLASH == 1
    #if BIT_SWAP_ON == 0
    spi_fast_read(&spi_inst, address, (u8 *)SectionStartAddress, SectionSizeByte);
    address += SectionSizeByte;
    #else
    dptr8 = (u8 *)SectionStartAddress;
    do{
        spi_fast_read(&spi_inst, address, (u8 *)&spi_data, 1);
        address += 1;
        *dptr8++ = BitReOrder8(spi_data);
    } while(--SectionSizeByte > 0);
    #endif
#else
    mycpy((void *)SectionStartAddress, (void *)address, SectionSizeByte);
    address += SectionSizeByte;
#endif


    return address;

}

#if SPI_FLASH == 0
/**
 * simple memcpy which copies size data from src to dst
 */
void* mycpy(void* dest, const void* src, size_t count)
{
        char* dst8 = (char*)dest;
        char* src8 = (char*)src;

        while (count--) {
#if BIT_SWAP_ON == 0
            *dst8++ = *src8++;
#else
            *dst8++ = BitReOrder8(*src8++);
#endif
        }

        return dest;
}
#endif


