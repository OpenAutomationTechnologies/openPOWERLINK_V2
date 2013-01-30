/*
 * File:		sysinit.c
 * Purpose:		Verdi Reset Configuration
 *
 * Notes:
 *
 */

#include "common.h"
#include "psc_uart.h"
#include "mcf548x_fbcs.h"
#include "mcf548x_sdramc.h"



/********************************************************************/

void fbcs_init(void);
void sdramc_init(void);

/********************************************************************/
void
mcf548x_init (void)
{
	extern uint32 _vectors[];
	uint32* __VECTOR_RAM;
	__VECTOR_RAM = (void*)0x0;

	uint32 n;
	uint8 *dp, *sp;
	
	psc_uart_init(TERMINAL_PORT, TERMINAL_BAUD);
#ifdef CONFIG_VERDI	
	fbcs_init();
	sdramc_init();
#else
	// TODO
	// initialization for FireEngine
	sdramc_init();
#endif

#if 0

	/* Copy the vector table to RAM */
		for (n = 0; n < 256; n++)
			__VECTOR_RAM[n] = _vectors[n];

	/* Move initialized data from ROM to RAM. */
	if (__DATA_ROM != __DATA_RAM)
	{
		dp = (uint8 *)__DATA_RAM;
		sp = (uint8 *)__DATA_ROM;
		n = __DATA_END - __DATA_RAM;
		while (n--)
			*dp++ = *sp++;
	}
 
	/* Zero uninitialized data */
	if (__BSS_START != __BSS_END)
	{
		sp = (uint8 *)__BSS_START;
		n = __BSS_END - __BSS_START;
		while (n--)
			*sp++ = 0;
	}
#endif	
}
/********************************************************************/
void
fbcs_init (void)
{
	/* External SRAM */
	MCF_FBCS_CSAR1 = MCF_FBCS_CSAR_BA(EXT_SRAM_ADDRESS);
	MCF_FBCS_CSCR1 = (MCF_FBCS_CSCR_PS_32
					| MCF_FBCS_CSCR_AA
					| MCF_FBCS_CSCR_WS(1));
	MCF_FBCS_CSMR1 = (MCF_FBCS_CSMR_BAM_512K 
					| MCF_FBCS_CSMR_V);

	/* Flash */
	/*
#define FLASH_ADDRESS 0xFE000000	
	MCF_FBCS_CSAR0 = MCF_FBCS_CSAR_BA(FLASH_ADDRESS);
	MCF_FBCS_CSCR0 = (MCF_FBCS_CSCR_PS_32
					| MCF_FBCS_CSCR_AA
					| MCF_FBCS_CSCR_WS(6));
	MCF_FBCS_CSMR0 = (MCF_FBCS_CSMR_BAM_32M 
					| MCF_FBCS_CSMR_V);
	*/
}  
/********************************************************************/
void
sdramc_init (void)
{
	/* Initialize DDR DIMMs on the Verdi board */
	
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
			| MCF_SDRAMC_SDRAMDS_SB_D(MCF_SDRAMC_SDRAMDS_DRIVE_8MA) );
			
		MCF_SDRAMC_CS0CFG = (0
			| MCF_SDRAMC_CSnCFG_CSBA(SDRAM_ADDRESS)
			| MCF_SDRAMC_CSnCFG_CSSZ(MCF_SDRAMC_CSnCFG_CSSZ_32MBYTE) ); 
		
			
		MCF_SDRAMC_SDCFG1 = (0
			| MCF_SDRAMC_SDCFG1_SRD2RW(5)
			| MCF_SDRAMC_SDCFG1_SWT2RD(3)
			| MCF_SDRAMC_SDCFG1_RDLAT(7)
			| MCF_SDRAMC_SDCFG1_ACT2RW(2)
			| MCF_SDRAMC_SDCFG1_PRE2ACT(2)
			| MCF_SDRAMC_SDCFG1_SRD2RW(5)
			| MCF_SDRAMC_SDCFG1_REF2ACT(9)
			| MCF_SDRAMC_SDCFG1_WTLAT(3)   );
			
		MCF_SDRAMC_SDCFG2 = (0
			| MCF_SDRAMC_SDCFG2_BRD2PRE(2)
			| MCF_SDRAMC_SDCFG2_BWT2RW(4)
			| MCF_SDRAMC_SDCFG2_BRD2WT(3)
			| MCF_SDRAMC_SDCFG2_BL(3)      );
				
		/* Precharge and enable write to SDMR */
		MCF_SDRAMC_SDCR = (0
			| MCF_SDRAMC_SDCR_MODE_EN
			| MCF_SDRAMC_SDCR_CKE
			| MCF_SDRAMC_SDCR_DDR
			| MCF_SDRAMC_SDCR_MUX(0)
			| MCF_SDRAMC_SDCR_RCNT(0xF)
			| MCF_SDRAMC_SDCR_IPALL	     );
		
		/* Write extended mode register */
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LEMR
			| MCF_SDRAMC_SDMR_AD(0x0)
			| MCF_SDRAMC_SDMR_CMD        );
		
		/* Write mode register and reset DLL */ 	
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LMR
			| MCF_SDRAMC_SDMR_AD(0x588)
			| MCF_SDRAMC_SDMR_CMD        );
				
		/* Execute a PALL command */
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR
			| MCF_SDRAMC_SDCR_IPALL      );
		
		/* Perform two REF cycles */
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR		 /*perform first refresh*/
			| MCF_SDRAMC_SDCR_IREF       );
		
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR		 /*perform second refresh*/
			| MCF_SDRAMC_SDCR_IREF       );
			
		/* Write mode register and clear reset DLL */
		MCF_SDRAMC_SDMR = (0
			| MCF_SDRAMC_SDMR_BNKAD_LMR
			| MCF_SDRAMC_SDMR_AD(0x188)
			| MCF_SDRAMC_SDMR_CMD        );
				
		/* Enable auto refresh and lock SDMR */
		MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR
			& ~MCF_SDRAMC_SDCR_MODE_EN
			| MCF_SDRAMC_SDCR_REF
			| MCF_SDRAMC_SDCR_DQS_OE(0xF) );
	}
}
/********************************************************************/
