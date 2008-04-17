/*
 *  m5485.h -- Motorola Coldfire 5485 CPU Support.
 *
 *  (C) Copyright 1999, Greg Ungerer (gerg@moreton.com.au)
 *  Modified from m5307.h by Kendrick Hamilton (hamilton@sedsystems.ca)
 */

/****************************************************************************/
#ifndef m5485_h
#define m5485_h
#define M5485
//#define M5407
/****************************************************************************/

/*
 *  Define the base address of the UARTS within the MBAR address
 *  space.
 */
#define MCFUART_BASE1       0x8600           /* Base address of UART1 */
#define MCFUART_BASE2       0x8700           /* Base address of UART2 */

/*
 *  Define master clock frequency of our 5485.
 */
#ifdef CONFIG_VERDI
#define MCF_BUSCLK     132000000
#else
#define MCF_BUSCLK     100000000
#endif

/*
 *  The board memory map is set up as follows:
 *
 *  0x00000000  --  SDRAM (operational memory - 4 or 16 Mb)
 *  0x10000000  --  MBAR (5307 SIM module peripherals)
 *  0x20000000  --  RAMBAR (5307 internal SRAM - 4k)
 *  0x30400000  --  LED bank (CS2)
 *  0x30600000  --  LAN (smc91c96 ethernet (CS3)
 *  0x50000000  --  PCI interface (CS1)
 *  0xf0000000  --  FLASH (1Mb) (CS0)
 */
#define MCF_MBAR        0x80000000

#define SDRAM_ADDRESS   0x00000000
#define FLASH_ADDRESS   0xFE000000


//---------------------------------------------------------------------------
// 2005/09/19 -rs
#ifdef CONFIG_VERDI
    #define FLASH_CONFIG_DATA_ADDRESS  0xFEA00000

#elif CONFIG_SYSTEC_PLCCORE5484
    #define FLASH_CONFIG_DATA_ADDRESS  0xFE0A0000   // adaption to PLCcore-MCF548x

#else
    // 2005/09/19 -rs
    // #define FLASH_CONFIG_DATA_ADDRESS  0xF0020000
    #define FLASH_CONFIG_DATA_ADDRESS  0xFE080000   // adaption to phyCORE-MCF548x
#endif
//---------------------------------------------------------------------------


/*
 * Size of internal RAM (on 5485 it is 4K)
 */

#define INT_RAM_SIZE 4096

/*
 *  Define the 5307 SIM register set addresses.
 */
#define MCFSIM_SIMR     0x03        /* SIM Config reg (r/w) */
#define MCFSIM_ICR0     0x4c        /* Intr Ctrl reg 0 (r/w) */
#define MCFSIM_ICR1     0x4d        /* Intr Ctrl reg 1 (r/w) */
#define MCFSIM_ICR2     0x4e        /* Intr Ctrl reg 2 (r/w) */
#define MCFSIM_ICR3     0x4f        /* Intr Ctrl reg 3 (r/w) */
#define MCFSIM_ICR4     0x50        /* Intr Ctrl reg 4 (r/w) */
#define MCFSIM_ICR5     0x51        /* Intr Ctrl reg 5 (r/w) */
#define MCFSIM_ICR6     0x52        /* Intr Ctrl reg 6 (r/w) */
#define MCFSIM_ICR7     0x53        /* Intr Ctrl reg 7 (r/w) */
#define MCFSIM_ICR8     0x54        /* Intr Ctrl reg 8 (r/w) */
#define MCFSIM_ICR9     0x55        /* Intr Ctrl reg 9 (r/w) */
#define MCFSIM_ICR10        0x56        /* Intr Ctrl reg 10 (r/w) */
#define MCFSIM_ICR11        0x57        /* Intr Ctrl reg 11 (r/w) */

#define MCFSIM_IPR      0x40        /* Interrupt Pend reg (r/w) */
#define MCFSIM_IMR      0x44        /* Interrupt Mask reg (r/w) */

#define MCFSIM_RSR      0x00        /* Reset Status reg (r/w) */
#define MCFSIM_SYPCR        0x01        /* System Protection reg (r/w)*/
#define MCFSIM_SWIVR        0x02        /* SW Watchdog intr reg (r/w) */
#define MCFSIM_SWSR     0x03        /* SW Watchdog service (r/w) */
#define MCFSIM_PAR      0x04        /* Pin Assignment reg (r/w) */

#define MCFSIM_PADDR        0x244       /* Parallel Direction (r/w) */
#define MCFSIM_PADAT        0x248       /* Parallel Port Value (r/w) */

#define MCFSIM_CSAR0        0x80        /* CS 0 Address 0 reg (r/w) */
#define MCFSIM_CSMR0        0x84        /* CS 0 Mask 0 reg (r/w) */
#define MCFSIM_CSCR0        0x8a        /* CS 0 Control reg (r/w) */
#define MCFSIM_CSAR1        0x8c        /* CS 1 Address reg (r/w) */
#define MCFSIM_CSMR1        0x90        /* CS 1 Mask reg (r/w) */
#define MCFSIM_CSCR1        0x96        /* CS 1 Control reg (r/w) */

#define MCFSIM_CSMR2        0x9e        /* CS 2 Mask reg (r/w) */
#define MCFSIM_CSCR2        0xa2        /* CS 2 Control reg (r/w) */
#define MCFSIM_CSMR3        0xaa        /* CS 3 Mask reg (r/w) */
#define MCFSIM_CSCR3        0xae        /* CS 3 Control reg (r/w) */
#define MCFSIM_CSMR4        0xb6        /* CS 4 Mask reg (r/w) */
#define MCFSIM_CSCR4        0xba        /* CS 4 Control reg (r/w) */
#define MCFSIM_CSMR5        0xc2        /* CS 5 Mask reg (r/w) */
#define MCFSIM_CSCR5        0xc6        /* CS 5 Control reg (r/w) */
#define MCFSIM_CSMR6        0xce        /* CS 6 Mask reg (r/w) */
#define MCFSIM_CSCR6        0xd2        /* CS 6 Control reg (r/w) */
#define MCFSIM_CSMR7        0xda        /* CS 7 Mask reg (r/w) */
#define MCFSIM_CSCR7        0xde        /* CS 7 Control reg (r/w) */

#define MCFSIM_DCR      0x100       /* DRAM Control reg (r/w) */
#define MCFSIM_DACR0        0x108       /* DRAM 0 Addr and Ctrl (r/w) */
#define MCFSIM_DMR0     0x10c       /* DRAM 0 Mask reg (r/w) */
#define MCFSIM_DACR1        0x110       /* DRAM 1 Addr and Ctrl (r/w) */
#define MCFSIM_DMR1     0x114       /* DRAM 1 Mask reg (r/w) */


/*
 *  Some symbol defines for the above...
 */
#define MCFSIM_SWDICR       MCFSIM_ICR0 /* Watchdog timer ICR */
#define MCFSIM_TIMER1ICR    MCFSIM_ICR1 /* Timer 1 ICR */
#define MCFSIM_TIMER2ICR    MCFSIM_ICR2 /* Timer 2 ICR */
#define MCFSIM_UART1ICR     MCFSIM_ICR4 /* UART 1 ICR */
#define MCFSIM_UART2ICR     MCFSIM_ICR5 /* UART 2 ICR */


/*
 *      GPIO
 */
#define MCF_GPIO_PAR_PSC0           (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A4F))
#define MCF_GPIO_PAR_PSC1           (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A4E))

/* Bit definitions and macros for MCF_GPIO_PAR_PSC1 */
#define MCF_GPIO_PAR_PSC1_PAR_TXD1                  (0x04)
#define MCF_GPIO_PAR_PSC1_PAR_RXD1                  (0x08)
#define MCF_GPIO_PAR_PSC1_PAR_RTS1(x)               (((x)&0x03)<<4)
#define MCF_GPIO_PAR_PSC1_PAR_CTS1(x)               (((x)&0x03)<<6)
#define MCF_GPIO_PAR_PSC1_PAR_CTS1_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC1_PAR_CTS1_BCLK             (0x80)
#define MCF_GPIO_PAR_PSC1_PAR_CTS1_CTS              (0xC0)
#define MCF_GPIO_PAR_PSC1_PAR_RTS1_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC1_PAR_RTS1_FSYNC            (0x20)
#define MCF_GPIO_PAR_PSC1_PAR_RTS1_RTS              (0x30)

/* Bit definitions and macros for MCF_GPIO_PAR_PSC0 */
#define MCF_GPIO_PAR_PSC0_PAR_TXD0                  (0x04)
#define MCF_GPIO_PAR_PSC0_PAR_RXD0                  (0x08)
#define MCF_GPIO_PAR_PSC0_PAR_RTS0(x)               (((x)&0x03)<<4)
#define MCF_GPIO_PAR_PSC0_PAR_CTS0(x)               (((x)&0x03)<<6)
#define MCF_GPIO_PAR_PSC0_PAR_CTS0_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC0_PAR_CTS0_BCLK             (0x80)
#define MCF_GPIO_PAR_PSC0_PAR_CTS0_CTS              (0xC0)
#define MCF_GPIO_PAR_PSC0_PAR_RTS0_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC0_PAR_RTS0_FSYNC            (0x20)
#define MCF_GPIO_PAR_PSC0_PAR_RTS0_RTS              (0x30)

/*********************************************************************
*
* FlexBus Chip Selects (FBCS)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_FBCS_CSAR0      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000500))
#define MCF_FBCS_CSMR0      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000504))
#define MCF_FBCS_CSCR0      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000508))
#define MCF_FBCS_CSAR1      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00050C))
#define MCF_FBCS_CSMR1      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000510))
#define MCF_FBCS_CSCR1      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000514))
#define MCF_FBCS_CSAR2      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000518))
#define MCF_FBCS_CSMR2      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00051C))
#define MCF_FBCS_CSCR2      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000520))
#define MCF_FBCS_CSAR3      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000524))
#define MCF_FBCS_CSMR3      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000528))
#define MCF_FBCS_CSCR3      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00052C))
#define MCF_FBCS_CSAR4      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000530))
#define MCF_FBCS_CSMR4      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000534))
#define MCF_FBCS_CSCR4      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000538))
#define MCF_FBCS_CSAR5      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00053C))
#define MCF_FBCS_CSMR5      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000540))
#define MCF_FBCS_CSCR5      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000544))
#define MCF_FBCS_CSAR(x)    (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000500+((x)*0x00C)))
#define MCF_FBCS_CSMR(x)    (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000504+((x)*0x00C)))
#define MCF_FBCS_CSCR(x)    (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000508+((x)*0x00C)))

/* Bit definitions and macros for MCF_FBCS_CSAR */
#define MCF_FBCS_CSAR_BA(x)        ((x)&0xFFFF0000)

/* Bit definitions and macros for MCF_FBCS_CSMR */
#define MCF_FBCS_CSMR_V            (0x00000001)
#define MCF_FBCS_CSMR_WP           (0x00000100)
#define MCF_FBCS_CSMR_BAM(x)       (((x)&0x0000FFFF)<<16)
#define MCF_FBCS_CSMR_BAM_4G       (0xFFFF0000)
#define MCF_FBCS_CSMR_BAM_2G       (0x7FFF0000)
#define MCF_FBCS_CSMR_BAM_1G       (0x3FFF0000)
#define MCF_FBCS_CSMR_BAM_1024M    (0x3FFF0000)
#define MCF_FBCS_CSMR_BAM_512M     (0x1FFF0000)
#define MCF_FBCS_CSMR_BAM_256M     (0x0FFF0000)
#define MCF_FBCS_CSMR_BAM_128M     (0x07FF0000)
#define MCF_FBCS_CSMR_BAM_64M      (0x03FF0000)
#define MCF_FBCS_CSMR_BAM_32M      (0x01FF0000)
#define MCF_FBCS_CSMR_BAM_16M      (0x00FF0000)
#define MCF_FBCS_CSMR_BAM_8M       (0x007F0000)
#define MCF_FBCS_CSMR_BAM_4M       (0x003F0000)
#define MCF_FBCS_CSMR_BAM_2M       (0x001F0000)
#define MCF_FBCS_CSMR_BAM_1M       (0x000F0000)
#define MCF_FBCS_CSMR_BAM_1024K    (0x000F0000)
#define MCF_FBCS_CSMR_BAM_512K     (0x00070000)
#define MCF_FBCS_CSMR_BAM_256K     (0x00030000)
#define MCF_FBCS_CSMR_BAM_128K     (0x00010000)
#define MCF_FBCS_CSMR_BAM_64K      (0x00000000)

/* Bit definitions and macros for MCF_FBCS_CSCR */
#define MCF_FBCS_CSCR_BSTW         (0x00000008)
#define MCF_FBCS_CSCR_BSTR         (0x00000010)
#define MCF_FBCS_CSCR_PS(x)        (((x)&0x00000003)<<6)
#define MCF_FBCS_CSCR_AA           (0x00000100)
#define MCF_FBCS_CSCR_WS(x)        (((x)&0x0000003F)<<10)
#define MCF_FBCS_CSCR_WRAH(x)      (((x)&0x00000003)<<16)
#define MCF_FBCS_CSCR_RDAH(x)      (((x)&0x00000003)<<18)
#define MCF_FBCS_CSCR_ASET(x)      (((x)&0x00000003)<<20)
#define MCF_FBCS_CSCR_SWSEN        (0x00800000)
#define MCF_FBCS_CSCR_SWS(x)       (((x)&0x0000003F)<<26)
#define MCF_FBCS_CSCR_PS_8         (0x0040)
#define MCF_FBCS_CSCR_PS_16        (0x0080)
#define MCF_FBCS_CSCR_PS_32        (0x0000)

/********************************************************************/

/*********************************************************************
*
* SDRAM Controller (SDRAMC)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_SDRAMC_SDRAMDS      (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000004))
#define MCF_SDRAMC_CS0CFG       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000020))
#define MCF_SDRAMC_CS1CFG       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000024))
#define MCF_SDRAMC_CS2CFG       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000028))
#define MCF_SDRAMC_CS3CFG       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00002C))
#define MCF_SDRAMC_CSnCFG(x)    (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000020+((x)*0x004)))
#define MCF_SDRAMC_SDMR         (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000100))
#define MCF_SDRAMC_SDCR         (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000104))
#define MCF_SDRAMC_SDCFG1       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x000108))
#define MCF_SDRAMC_SDCFG2       (*(volatile unsigned int*)(void*)(MCF_MBAR+0x00010C))

/* Bit definitions and macros for MCF_SDRAMC_SDRAMDS */
#define MCF_SDRAMC_SDRAMDS_SB_D(x)         (((x)&0x00000003)<<0)
#define MCF_SDRAMC_SDRAMDS_SB_S(x)         (((x)&0x00000003)<<2)
#define MCF_SDRAMC_SDRAMDS_SB_A(x)         (((x)&0x00000003)<<4)
#define MCF_SDRAMC_SDRAMDS_SB_C(x)         (((x)&0x00000003)<<6)
#define MCF_SDRAMC_SDRAMDS_SB_E(x)         (((x)&0x00000003)<<8)
#define MCF_SDRAMC_SDRAMDS_DRIVE_8MA       (0x10)
#define MCF_SDRAMC_SDRAMDS_DRIVE_16MA      (0x01)
#define MCF_SDRAMC_SDRAMDS_DRIVE_24MA      (0x00)
#define MCF_SDRAMC_SDRAMDS_DRIVE_NONE      (0x03)

/* Bit definitions and macros for MCF_SDRAMC_CSnCFG */
#define MCF_SDRAMC_CSnCFG_CSSZ(x)          (((x)&0x0000001F)<<0)
#define MCF_SDRAMC_CSnCFG_CSBA(x)          (((x)&0x00000FFF)<<20)
#define MCF_SDRAMC_CSnCFG_CSSZ_DIABLE      (0x00000000)
#define MCF_SDRAMC_CSnCFG_CSSZ_1MBYTE      (0x00000013)
#define MCF_SDRAMC_CSnCFG_CSSZ_2MBYTE      (0x00000014)
#define MCF_SDRAMC_CSnCFG_CSSZ_4MBYTE      (0x00000015)
#define MCF_SDRAMC_CSnCFG_CSSZ_8MBYTE      (0x00000016)
#define MCF_SDRAMC_CSnCFG_CSSZ_16MBYTE     (0x00000017)
#define MCF_SDRAMC_CSnCFG_CSSZ_32MBYTE     (0x00000018)
#define MCF_SDRAMC_CSnCFG_CSSZ_64MBYTE     (0x00000019)
#define MCF_SDRAMC_CSnCFG_CSSZ_128MBYTE    (0x0000001A)
#define MCF_SDRAMC_CSnCFG_CSSZ_256MBYTE    (0x0000001B)
#define MCF_SDRAMC_CSnCFG_CSSZ_512MBYTE    (0x0000001C)
#define MCF_SDRAMC_CSnCFG_CSSZ_1GBYTE      (0x0000001D)
#define MCF_SDRAMC_CSnCFG_CSSZ_2GBYTE      (0x0000001E)
#define MCF_SDRAMC_CSnCFG_CSSZ_4GBYTE      (0x0000001F)

/* Bit definitions and macros for MCF_SDRAMC_SDMR */
#define MCF_SDRAMC_SDMR_CMD                (0x00010000)
#define MCF_SDRAMC_SDMR_AD(x)              (((x)&0x00000FFF)<<18)
#define MCF_SDRAMC_SDMR_BNKAD(x)           (((x)&0x00000003)<<30)
#define MCF_SDRAMC_SDMR_BNKAD_LMR          (0x00000000)
#define MCF_SDRAMC_SDMR_BNKAD_LEMR         (0x40000000)

/* Bit definitions and macros for MCF_SDRAMC_SDCR */
#define MCF_SDRAMC_SDCR_IPALL              (0x00000002)
#define MCF_SDRAMC_SDCR_IREF               (0x00000004)
#define MCF_SDRAMC_SDCR_BUFF               (0x00000010)
#define MCF_SDRAMC_SDCR_DQS_OE(x)          (((x)&0x0000000F)<<8)
#define MCF_SDRAMC_SDCR_RCNT(x)            (((x)&0x0000003F)<<16)
#define MCF_SDRAMC_SDCR_DRIVE              (0x00400000)
#define MCF_SDRAMC_SDCR_AP                 (0x00800000)
#define MCF_SDRAMC_SDCR_MUX(x)             (((x)&0x00000003)<<24)
#define MCF_SDRAMC_SDCR_REF                (0x10000000)
#define MCF_SDRAMC_SDCR_DDR                (0x20000000)
#define MCF_SDRAMC_SDCR_CKE                (0x40000000)
#define MCF_SDRAMC_SDCR_MODE_EN            (0x80000000)

/* Bit definitions and macros for MCF_SDRAMC_SDCFG1 */
#define MCF_SDRAMC_SDCFG1_WTLAT(x)         (((x)&0x00000007)<<4)
#define MCF_SDRAMC_SDCFG1_REF2ACT(x)       (((x)&0x0000000F)<<8)
#define MCF_SDRAMC_SDCFG1_PRE2ACT(x)       (((x)&0x00000007)<<12)
#define MCF_SDRAMC_SDCFG1_ACT2RW(x)        (((x)&0x00000007)<<16)
#define MCF_SDRAMC_SDCFG1_RDLAT(x)         (((x)&0x0000000F)<<20)
#define MCF_SDRAMC_SDCFG1_SWT2RD(x)        (((x)&0x00000007)<<24)
#define MCF_SDRAMC_SDCFG1_SRD2RW(x)        (((x)&0x0000000F)<<28)

/* Bit definitions and macros for MCF_SDRAMC_SDCFG2 */
#define MCF_SDRAMC_SDCFG2_BL(x)            (((x)&0x0000000F)<<16)
#define MCF_SDRAMC_SDCFG2_BRD2WT(x)        (((x)&0x0000000F)<<20)
#define MCF_SDRAMC_SDCFG2_BWT2RW(x)        (((x)&0x0000000F)<<24)
#define MCF_SDRAMC_SDCFG2_BRD2PRE(x)       (((x)&0x0000000F)<<28)

/********************************************************************/



/*********************************************************************
*
* General purpose I/O (extension)
*
* 2006/01/30 -rs
*
*********************************************************************/

#define MCF_GPIO_PAR_DMA        (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A43))
#define MCF_GPIO_PPDSDR_DMA     (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A22))
#define MCF_GPIO_PAR_PSC2       (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A4D))
#define MCF_GPIO_PAR_PSC3       (*(volatile unsigned char *)(void*)(MCF_MBAR + 0x000A4C))

/* Bit definitions and macros for MCF_GPIO_PAR_PSC3 */
#define MCF_GPIO_PAR_PSC3_PAR_TXD3                  (0x04)
#define MCF_GPIO_PAR_PSC3_PAR_RXD3                  (0x08)
#define MCF_GPIO_PAR_PSC3_PAR_RTS3(x)               (((x)&0x03)<<4)
#define MCF_GPIO_PAR_PSC3_PAR_CTS3(x)               (((x)&0x03)<<6)
#define MCF_GPIO_PAR_PSC3_PAR_CTS3_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC3_PAR_CTS3_BCLK             (0x80)
#define MCF_GPIO_PAR_PSC3_PAR_CTS3_CTS              (0xC0)
#define MCF_GPIO_PAR_PSC3_PAR_RTS3_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC3_PAR_RTS3_FSYNC            (0x20)
#define MCF_GPIO_PAR_PSC3_PAR_RTS3_RTS              (0x30)
#define MCF_GPIO_PAR_PSC3_PAR_CTS2_CANRX            (0x40)

/* Bit definitions and macros for MCF_GPIO_PAR_PSC2 */
#define MCF_GPIO_PAR_PSC2_PAR_TXD2                  (0x04)
#define MCF_GPIO_PAR_PSC2_PAR_RXD2                  (0x08)
#define MCF_GPIO_PAR_PSC2_PAR_RTS2(x)               (((x)&0x03)<<4)
#define MCF_GPIO_PAR_PSC2_PAR_CTS2(x)               (((x)&0x03)<<6)
#define MCF_GPIO_PAR_PSC2_PAR_CTS2_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC2_PAR_CTS2_BCLK             (0x80)
#define MCF_GPIO_PAR_PSC2_PAR_CTS2_CTS              (0xC0)
#define MCF_GPIO_PAR_PSC2_PAR_RTS2_GPIO             (0x00)
#define MCF_GPIO_PAR_PSC2_PAR_RTS2_CANTX            (0x10)
#define MCF_GPIO_PAR_PSC2_PAR_RTS2_FSYNC            (0x20)
#define MCF_GPIO_PAR_PSC2_PAR_RTS2_RTS              (0x30)

/* GPIO / LED pins */
#define MCF_GPIO_PODR_PCIBG         (*(volatile unsigned char *)(void*)(&__MBAR[0x000A09]))
#define MCF_GPIO_PODR_PCIBR         (*(volatile unsigned char *)(void*)(&__MBAR[0x000A0A]))
#define MCF_GPIO_PDDR_PCIBG         (*(volatile unsigned char *)(void*)(&__MBAR[0x000A19]))
#define MCF_GPIO_PDDR_PCIBR         (*(volatile unsigned char *)(void*)(&__MBAR[0x000A1A]))

/********************************************************************/

#define MCF_FEC_FECRFRP0            (*(volatile unsigned int*)(void*)(&__MBAR[0x00919C]))
#define MCF_FEC_FECRFWP0            (*(volatile unsigned int*)(void*)(&__MBAR[0x0091A0]))
#define MCF_FEC_FECRLRFP0           (*(volatile unsigned int*)(void*)(&__MBAR[0x009190]))
#define MCF_FEC_FECRLWFP0           (*(volatile unsigned int*)(void*)(&__MBAR[0x009194]))


#define MCF_ICR_IP(x)           (((x)&0x07)<<0)
#define MCF_ICR_IL(x)           (((x)&0x07)<<3)


/*
 *  Macro to set IMR register. It is 32 bits on the 5485.
 */

#define mcf_getimr()        \
    *((volatile unsigned long *) (MCF_MBAR + MCFSIM_IMR))

#define mcf_setimr(imr)     \
    *((volatile unsigned long *) (MCF_MBAR + MCFSIM_IMR)) = (imr);

/****************************************************************************/
#endif  /* m5485_h */
