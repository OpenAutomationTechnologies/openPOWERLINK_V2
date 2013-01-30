/*
 * File:        verdi.h
 * Purpose:     Validation board definitions and memory map information
 *
 * Notes:
 */

#ifndef __VERDI_H__
#define __VERDI_H__
#include "mcf5xxx.h"

#include "arch.h"

#include "sys_sram.h"

/********************************************************************/

/*
 * Debug prints ON (#define) or OFF (#undef)
 */
#define DEBUG


/*
 * System Bus Clock Info
 */

#define SYSTEM_CLOCK        100     /* system bus frequency in MHz */
#define PERIOD              10.     /* system bus period in ns */


/*
 * Flash Info
 */
#define AMD_FLASH_DEVICES   2
#define AMD_FLASH_AM29LV128M_16BIT

/*
 * Serial Port Info
 */
#define TERMINAL_PORT       0       /* PSC channel used as terminal */
#define TERMINAL_BAUD       19200   /* 115200 */
#undef  HARDWARE_FLOW_CONTROL       /* Flow control ON or OFF */

/*
 * Ethernet Port Info
 */
#define FEC_PHY0            (0x01)
#define FEC_PHY1            (0x01)
#define FEC_PHY(x)          ((x == 0) ? FEC_PHY0 : FEC_PHY1)



#define EXT_SRAM_ADDRESS    0x40000000
#define EXT_SRAM_SIZE       0x00080000
#define SDRAM_ADDRESS       0x00000000
#define SDRAM_SIZE          0x01000000
#define SYS_SRAM_ADDRESS    0x80010000
#define SYS_SRAM_SIZE       0x00008000




/*
 * Memory map definitions from linker command files
 */
 
extern uint8 __MBAR[];
extern uint8 __SDRAM[];
extern uint8 __SDRAM_SIZE[];
extern uint8 __SYS_SRAM[];
extern uint8 __SYS_SRAM_SIZE[];
extern uint8 __CORE_SRAM0[];
extern uint8 __CORE_SRAM0_SIZE[];
extern uint8 __CORE_SRAM1[];
extern uint8 __CORE_SRAM1_SIZE[];
extern uint8 __EXT_SRAM[];
extern uint8 __EXT_SRAM_SIZE[];
extern uint8 __FLASH[];
extern uint8 __FLASH_SIZE[];

/*
 * Memory Map Info
 */
/*
#define MBAR_ADDRESS        (uint32)__MBAR

#define SDRAM_ADDRESS       (uint32)__SDRAM
#define SDRAM_SIZE          (uint32)__SDRAM_SIZE

#define SYS_SRAM_ADDRESS    (uint32)__SYS_SRAM
#define SYS_SRAM_SIZE       (uint32)__SYS_SRAM_SIZE

#define CORE_SRAM0_ADDRESS  (uint32)__CORE_SRAM0
#define CORE_SRAM0_SIZE     (uint32)__CORE_SRAM0_SIZE

#define CORE_SRAM1_ADDRESS  (uint32)__CORE_SRAM1
#define CORE_SRAM1_SIZE     (uint32)__CORE_SRAM1_SIZE

#define EXT_SRAM_ADDRESS    (uint32)__EXT_SRAM
#define EXT_SRAM_SIZE       (uint32)__EXT_SRAM_SIZE

#define FLASH_ADDRESS       (uint32)__FLASH
#define FLASH_SIZE          (uint32)__FLASH_SIZE
*/
/********************************************************************/

/*
 * Interrupt Priorities
 */
#define DMA_INTC_LVL        7
#define DMA_INTC_PRI        3
#define FEC0_INTC_LVL       6
#define FEC0_INTC_PRI       1
#define FEC1_INTC_LVL       6
#define FEC1_INTC_PRI       0
#define FEC_INTC_LVL(x)     ((x == 0) ? FEC0_INTC_LVL : FEC1_INTC_LVL)
#define FEC_INTC_PRI(x)     ((x == 0) ? FEC0_INTC_PRI : FEC1_INTC_PRI)

/*
 * DMA Task Priorities
 */
#define FEC0RX_DMA_PRI        7
#define FEC1RX_DMA_PRI        7
#define FECRX_DMA_PRI(x)      ((x == 0) ? FEC0RX_DMA_PRI : FEC1RX_DMA_PRI)
#define FEC0TX_DMA_PRI        6
#define FEC1TX_DMA_PRI        6
#define FECTX_DMA_PRI(x)      ((x == 0) ? FEC0TX_DMA_PRI : FEC1TX_DMA_PRI)

#endif /* __VERDI_H__ */
