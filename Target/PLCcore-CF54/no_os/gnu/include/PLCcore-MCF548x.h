/*
 * File:        PLCcore-MCF548x.h
 * Purpose:     Validation board definitions and memory map information
 *
 * Notes:
 */

#ifndef __PLCCOREMCF548x_H__
#define __PLCCOREMCF548x_H__

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
//#define SPANSION_FLASH_DEVICES   1
//#define SPANSION_FLASH_S29GL128N_16BIT

/*
 * Serial Port Info
 */
#define TERMINAL_PORT       0       /* PSC channel used as terminal */
#define TERMINAL_BAUD       9600    /* 9600 Baud */
#undef  HARDWARE_FLOW_CONTROL       /* Flow control ON or OFF */

/*
 * Ethernet Port Info
 */
#define FEC_PHY0            (0x01)
#define FEC_PHY1            (0x01)
#define FEC_PHY(x)          ((x == 0) ? FEC_PHY0 : FEC_PHY1)

/*
 * Memory map definitions from linker command files
 */
extern uint8  __MBAR[];
extern uint8  __SDRAM[];
extern uint32 __SDRAM_SIZE;
extern uint8  __SYS_SRAM[];
extern uint32 __SYS_SRAM_SIZE;
extern uint8  __CORE_SRAM0[];
extern uint32 __CORE_SRAM0_SIZE;
extern uint8  __CORE_SRAM1[];
extern uint32 __CORE_SRAM1_SIZE;
extern uint8  __FLASH[];
extern uint32 __FLASH_SIZE;

/*
 * Memory Map Info
 */
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

/********************************************************************/

#endif /* __PLCCOREMCF548x_H__ */
