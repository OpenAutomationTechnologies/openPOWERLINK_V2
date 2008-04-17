/*
 * File:		sysinit.c
 * Purpose:		PLCcore Reset Configuration
 *
 * Notes:
 *
 */

#include "../include/common.h"

/********************************************************************/

void fbcs_init(void);
void sdramc_init(void);
void mcf548x_init (void);

//---------------------------------------------------------------------------
//
// Function:     atexit
//
// Description:  This function will be called at the end of ctr0.s previous
//               calling main-function. The function is called from crt0.s
//               and gets as parameter the address of __FINI_SECTION__. The
//               function is called previouse calling __INIT_SECTION__.
//
// Parameters:   pointer to __FINI_SECTION__
//
// 
// Returns:      none
//
//
// State:        
//
//---------------------------------------------------------------------------

void atexit(void * p) 
{
}

//---------------------------------------------------------------------------
//
// Function:     __main
//
// Description:  This function is called previous application main function
//               parameter initializing (after atexit). The function enables
//               the "Global interrupt state" and sets the supervisor mode.
//
// Parameters:   none
//
// 
// Returns:      none
//
//
// State:        
//
//---------------------------------------------------------------------------

void __main(void) 
{
     //mcf5xxx_wr_sr(0x3000);

     // setup initialized and unintialized memory
     // setup interrupt vector table
    // startup();
    mcf548x_init ();

}

//---------------------------------------------------------------------------
//
// Function:     exit
//
// Description:  This function is called after main-function was finished. 
//
// Parameters:   none
//
// 
// Returns:      none
//
//
// State:        
//
//---------------------------------------------------------------------------

void exit(void) 
{ 
     while(1); 
}

/********************************************************************/
void
mcf548x_init (void)
{
	extern char __BSS_START[];
	extern char __BSS_END[];
	extern uint32 VECTOR_TABLE[];
	extern uint32 __VECTOR_RAM[];

	uint32 n;
	uint8 *dp, *sp;
	
	// Set the MBAR to the default value of 0x80000000.
	// The system SRAM is located relating to the MBAR.
	
	mcf5xxx_wr_rambar0(0x20000021);
	mcf5xxx_wr_rambar1(0x20001021);

	fbcs_init();
	sdramc_init();

	/* Copy the vector table to RAM */
	if (__VECTOR_RAM != VECTOR_TABLE)
	{
		for (n = 0; n < 256; n++)
			__VECTOR_RAM[n] = VECTOR_TABLE[n];
	}
    mcf5xxx_wr_vbr((uint32)__VECTOR_RAM);

	/* Move initialized data from ROM to RAM. */
	#if (INIT_VARS > 0)
	{
	extern char __DATA_ROM[];
	extern char __DATA_RAM[];
	extern char __DATA_END[];
	uint32 n;
	uint8 *dp, *sp;
	
        if (__DATA_ROM != __DATA_RAM)
	    {
		   dp = (uint8 *)__DATA_RAM;
		   sp = (uint8 *)__DATA_ROM;
		   n = __DATA_END - __DATA_RAM;
		   while (n--)
		       *dp++ = *sp++;
         }
	}
    #endif
 
	/* Zero uninitialized data */
	if (__BSS_START != __BSS_END)
	{
		sp = (uint8 *)__BSS_START;
		n = __BSS_END - __BSS_START;
		while (n--)
			*sp++ = 0;
	}

    // Initiate invalidation of all caches.
    mcf5xxx_wr_cacr (0x01040100);

    // mask MBAR memory region from caching
    mcf5xxx_wr_acr0 (MCF_MBAR | 0xc040);

    // mask DMA memory region from caching
    mcf5xxx_wr_acr1 (0x0100c060);

//    mcf5xxx_wr_acr1 (0x0200c020);
//    mcf5xxx_wr_acr2 (0x0200c020);
    // Enable the Instruction Cache
    // Initiate invalidation of instruction cache.
//    mcf5xxx_wr_cacr (0x00008000);
//    mcf5xxx_wr_cacr (0xA30C8100);
    mcf5xxx_wr_cacr (0x86008000);
    /*
     * Enable the Instruction Cache
     */
/*    mcf5xxx_wr_cacr(0	| MCF5XXX_CACR_CENB
                        | MCF5XXX_CACR_CINV
                        | MCF5XXX_CACR_DISD
                        | MCF5XXX_CACR_CEIB
                        | MCF5XXX_CACR_CLNF_00);
    mcf5xxx_wr_cacr(0
        | MCF5XXX_CACR_IEC
        | MCF5XXX_CACR_ICINVA);*/
}
/********************************************************************/
void
fbcs_init (void)
{
	/* Flash */
	MCF_FBCS_CSAR0 = MCF_FBCS_CSAR_BA(FLASH_ADDRESS);
	
	MCF_FBCS_CSCR0 = (MCF_FBCS_CSCR_PS_16
					| MCF_FBCS_CSCR_AA
					| MCF_FBCS_CSCR_WS(7));
					
	MCF_FBCS_CSMR0 = (MCF_FBCS_CSMR_BAM_32M 
					| MCF_FBCS_CSMR_V);
}  
/********************************************************************/
void
sdramc_init (void)
{
	/*
	 * Check to see if the SDRAM has already been initialized
	 * by a run control tool
	 */
	if (!(MCF_SDRAMC_SDCR & MCF_SDRAMC_SDCR_REF))
	{
		/* Basic configuration and initialization */
		
		MCF_SDRAMC_SDRAMDS = (0
			| MCF_SDRAMC_SDRAMDS_SB_E(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
			| MCF_SDRAMC_SDRAMDS_SB_C(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
			| MCF_SDRAMC_SDRAMDS_SB_A(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
			| MCF_SDRAMC_SDRAMDS_SB_S(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
			| MCF_SDRAMC_SDRAMDS_SB_D(MCF_SDRAMC_SDRAMDS_DRIVE_8MA) ); // 0x0000_02AA
				
		MCF_SDRAMC_CS0CFG = (0
			| MCF_SDRAMC_CSnCFG_CSBA(SDRAM_ADDRESS)
			| MCF_SDRAMC_CSnCFG_CSSZ(MCF_SDRAMC_CSnCFG_CSSZ_128MBYTE) ); // 0x0000_001A
				
		MCF_SDRAMC_SDCFG1 = (0
			| MCF_SDRAMC_SDCFG1_SRD2RW(5)
			| MCF_SDRAMC_SDCFG1_SWT2RD(3)
			| MCF_SDRAMC_SDCFG1_RDLAT(7)
			| MCF_SDRAMC_SDCFG1_ACT2RW(2)
			| MCF_SDRAMC_SDCFG1_PRE2ACT(2)
			| MCF_SDRAMC_SDCFG1_SRD2RW(5)
			| MCF_SDRAMC_SDCFG1_REF2ACT(9)
			| MCF_SDRAMC_SDCFG1_WTLAT(3)   ); // 0x5372_2930
			
		MCF_SDRAMC_SDCFG2 = (0
			| MCF_SDRAMC_SDCFG2_BRD2PRE(2)
			| MCF_SDRAMC_SDCFG2_BWT2RW(4)
			| MCF_SDRAMC_SDCFG2_BRD2WT(3)
//			| MCF_SDRAMC_SDCFG2_BL(7)      ); // 0x2433_0000
			| MCF_SDRAMC_SDCFG2_BL(3)      ); // 0x2433_0000

		/* Precharge and enable write to SDMR */
		MCF_SDRAMC_SDCR = (0
			| MCF_SDRAMC_SDCR_MODE_EN
			| MCF_SDRAMC_SDCR_CKE
			| MCF_SDRAMC_SDCR_DDR
			| MCF_SDRAMC_SDCR_MUX(0x01)
			| MCF_SDRAMC_SDCR_RCNT(0xF)
			| MCF_SDRAMC_SDCR_IPALL	     ); // 0xE10F_0002
		
		/* Write extended mode register */
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LEMR
			| MCF_SDRAMC_SDMR_AD(0x0)
			| MCF_SDRAMC_SDMR_CMD        ); // 0x4001_0000
		
		/* Write mode register and reset DLL */ 	
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LMR
			| MCF_SDRAMC_SDMR_AD(0x588)
			| MCF_SDRAMC_SDMR_CMD        ); // 0x0589_0000
				
		/* Execute a PALL command */
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR
			| MCF_SDRAMC_SDCR_IPALL      ); // 0xE10F_0002
		
		/* Perform two REF cycles */
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR		 /*perform first refresh*/
			| MCF_SDRAMC_SDCR_IREF       ); // 0xE10F_0004
		
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR		 /*perform second refresh*/
			| MCF_SDRAMC_SDCR_IREF       ); // 0xE10F_0004
			
		/* Write mode register and clear reset DLL */
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LMR
			| MCF_SDRAMC_SDMR_AD(0x188)
			| MCF_SDRAMC_SDMR_CMD        ); // 0x0189_0000
				
		/* Enable auto refresh and lock SDMR */
		MCF_SDRAMC_SDCR = (0 | MCF_SDRAMC_SDCR
			& ~MCF_SDRAMC_SDCR_MODE_EN
			| MCF_SDRAMC_SDCR_REF
			| MCF_SDRAMC_SDCR_DQS_OE(0xF) ); // 0x710F_0006			
	}
}
/********************************************************************/
