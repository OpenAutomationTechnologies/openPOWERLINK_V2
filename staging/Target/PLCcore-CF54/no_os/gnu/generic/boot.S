/*****************************************************************************/

/*
 *	boot.S -- startup code for colilo
 *
 *      Based on NETtel code: (C) Copyright 1999, Moreton Bay Ventures Pty Ltd.
 *      Modifications: (C) Copyright 1999, Rob Scott
 *      Modifications: (C) Copyright 2003, Jeremy Andrus
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*****************************************************************************/


#include "arch.h"

/*
 * MEM_BUILTIN - address to relocate on chip SRAM to. (Usually 8KB).
 *
 * NOTE: The 5249 has 32k + 64k of internal SRAM. ~Jeremy
 * NOTE: The 5485 has 4k + 4k of internal SRAM
 */
#define MEM_BUILTIN 0x20010000

#ifdef __MCF5249__
#define MEM_BUILTIN2 0x20000000
#endif

/*****************************************************************************/

.global _start
.global _vectors
.global _version
.global _fault
.global nop
.global startimage
.global disableCache


.extern start_main
.extern recover
.extern xfer_addr
.extern __SP_INIT
.extern mcf548x_init
.extern dma_interrupt_handler
.extern timer_handler

/*****************************************************************************/

.text

/*
 *	Vector table. This is used for initial platform startup.
 *	These vectors are to catch any un-intended traps.
 */

_vectors:

/* The initial program counter is 0x04. The start address needs to be there */
INITSP:		.long	__SP_INIT				/* Initial SP			*/
INITPC:		.long	_start					/* Initial PC			*/
vector02:	.long	_fault	/* Access Error			*/
vector03:	.long	_fault	/* Address Error		*/
vector04:	.long	_fault	/* Illegal Instruction	*/
vector05:	.long	_fault	/* Reserved				*/
vector06:	.long	_fault	/* Reserved				*/
vector07:	.long	_fault	/* Reserved				*/
vector08:	.long	_fault	/* Privilege Violation	*/
vector09:	.long	_fault	/* Trace				*/
vector0A:	.long	_fault	/* Unimplemented A-Line	*/
vector0B:	.long	_fault	/* Unimplemented F-Line	*/
vector0C:	.long	_fault	/* Debug Interrupt		*/
vector0D:	.long	_fault	/* Reserved				*/
vector0E:	.long	_fault	/* Format Error			*/
vector0F:	.long	_fault	/* Unitialized Int.		*/
vector10:	.long	_fault	/* Reserved				*/
vector11:	.long	_fault	/* Reserved				*/
vector12:	.long	_fault	/* Reserved				*/
vector13:	.long	_fault	/* Reserved				*/
vector14:	.long	_fault	/* Reserved				*/
vector15:	.long	_fault	/* Reserved				*/
vector16:	.long	_fault	/* Reserved				*/
vector17:	.long	_fault	/* Reserved				*/
vector18:	.long	_fault	/* Spurious Interrupt	*/
vector19:	.long	_fault				/* Autovector Level 1	*/
vector1A:	.long	_fault				/* Autovector Level 2	*/
vector1B:	.long	_fault				/* Autovector Level 3	*/
vector1C:	.long	_fault				/* Autovector Level 4	*/
vector1D:	.long	_fault				/* Autovector Level 5	*/
vector1E:	.long	_fault				/* Autovector Level 6	*/
vector1F:	.long	_fault				/* Autovector Level 7	*/
vector20:	.long	_fault	/* TRAP #0				*/
vector21:	.long	_fault	/* TRAP #1				*/
vector22:	.long	_fault	/* TRAP #2				*/
vector23:	.long	_fault	/* TRAP #3				*/
vector24:	.long	_fault	/* TRAP #4				*/
vector25:	.long	_fault	/* TRAP #5				*/
vector26:	.long	_fault	/* TRAP #6				*/
vector27:	.long	_fault	/* TRAP #7				*/
vector28:	.long	_fault	/* TRAP #8				*/
vector29:	.long	_fault	/* TRAP #9				*/
vector2A:	.long	_fault	/* TRAP #10				*/
vector2B:	.long	_fault	/* TRAP #11				*/
vector2C:	.long	_fault	/* TRAP #12				*/
vector2D:	.long	_fault	/* TRAP #13				*/
vector2E:	.long	_fault	/* TRAP #14				*/
vector2F:	.long	_fault	/* TRAP #15				*/
vector30:	.long	_fault	/* Reserved				*/
vector31:	.long	_fault	/* Reserved				*/
vector32:	.long	_fault	/* Reserved				*/
vector33:	.long	_fault	/* Reserved				*/
vector34:	.long	_fault	/* Reserved				*/
vector35:	.long	_fault	/* Reserved				*/
vector36:	.long	_fault	/* Reserved				*/
vector37:	.long	_fault	/* Reserved				*/
vector38:	.long	_fault
vector39:	.long	_fault	/* Reserved				*/
vector3A:	.long	_fault	/* Reserved				*/
vector3B:	.long	_fault	/* Reserved				*/
vector3C:	.long	_fault	/* Reserved				*/
vector3D:	.long	_fault	/* Reserved				*/
vector3E:	.long	_fault	/* Reserved				*/
vector3F:	.long	_fault	/* Reserved				*/
vector40:	.long	_fault
vector41:	.long	_fault
vector42:	.long	_fault
vector43:	.long	_fault
vector44:	.long	_fault
vector45:	.long	_fault
vector46:	.long	_fault
vector47:	.long	_fault
vector48:	.long	_fault
vector49:	.long	_fault
vector4A:	.long	_fault
vector4B:	.long	_fault
vector4C:	.long	_fault
vector4D:	.long	_fault
vector4E:	.long	_fault
vector4F:	.long	_fault
vector50:	.long	_fault
vector51:	.long	_fault
vector52:	.long	_fault
vector53:	.long	_fault
vector54:	.long	_fault
vector55:	.long	_fault
vector56:	.long	_fault
vector57:	.long	_fault
vector58:	.long	_fault
vector59:	.long	_fault
vector5A:	.long	_fault
vector5B:	.long	_fault
vector5C:	.long	_fault
vector5D:	.long	_fault
vector5E:	.long	_fault
vector5F:	.long	_fault
vector60:	.long	_fault
vector61:	.long	_fault
vector62:	.long	_fault
vector63:	.long	_fault
vector64:	.long	_fault
vector65:	.long	_fault
vector66:	.long	_fault
vector67:	.long	_fault
vector68:	.long	_fault
vector69:	.long	_fault
vector6A:	.long	_fault
vector6B:	.long	_fault
vector6C:	.long	_fault
vector6D:	.long	_fault
vector6E:	.long	_fault
vector6F:	.long	_fault
vector70:	.long	dma_interrupt_handler
vector71:	.long	_fault
vector72:	.long	_fault
vector73:	.long	_fault
vector74:	.long	_fault
vector75:	.long	_fault
vector76:	.long	_fault /*timer_handler*/
vector77:	.long	_fault
vector78:	.long	_fault
vector79:	.long	_fault
vector7A:	.long	_fault
vector7B:	.long	_fault
vector7C:	.long	_fault
vector7D:	.long	_fault
vector7E:	.long	_fault
vector7F:	.long	_fault
vector80:	.long	_fault
vector81:	.long	_fault
vector82:	.long	_fault
vector83:	.long	_fault
vector84:	.long	_fault
vector85:	.long	_fault
vector86:	.long	_fault
vector87:	.long	_fault
vector88:	.long	_fault
vector89:	.long	_fault
vector8A:	.long	_fault
vector8B:	.long	_fault
vector8C:	.long	_fault
vector8D:	.long	_fault
vector8E:	.long	_fault
vector8F:	.long	_fault
vector90:	.long	_fault
vector91:	.long	_fault
vector92:	.long	_fault
vector93:	.long	_fault
vector94:	.long	_fault
vector95:	.long	_fault
vector96:	.long	_fault
vector97:	.long	_fault
vector98:	.long	_fault
vector99:	.long	_fault
vector9A:	.long	_fault
vector9B:	.long	_fault
vector9C:	.long	_fault
vector9D:	.long	_fault
vector9E:	.long	_fault
vector9F:	.long	_fault
vectorA0:	.long	_fault
vectorA1:	.long	_fault
vectorA2:	.long	_fault
vectorA3:	.long	_fault
vectorA4:	.long	_fault
vectorA5:	.long	_fault
vectorA6:	.long	_fault
vectorA7:	.long	_fault
vectorA8:	.long	_fault
vectorA9:	.long	_fault
vectorAA:	.long	_fault
vectorAB:	.long	_fault
vectorAC:	.long	_fault
vectorAD:	.long	_fault
vectorAE:	.long	_fault
vectorAF:	.long	_fault
vectorB0:	.long	_fault
vectorB1:	.long	_fault
vectorB2:	.long	_fault
vectorB3:	.long	_fault
vectorB4:	.long	_fault
vectorB5:	.long	_fault
vectorB6:	.long	_fault
vectorB7:	.long	_fault
vectorB8:	.long	_fault
vectorB9:	.long	_fault
vectorBA:	.long	_fault
vectorBB:	.long	_fault
vectorBC:	.long	_fault
vectorBD:	.long	_fault
vectorBE:	.long	_fault
vectorBF:	.long	_fault
vectorC0:	.long	_fault
vectorC1:	.long	_fault
vectorC2:	.long	_fault
vectorC3:	.long	_fault
vectorC4:	.long	_fault
vectorC5:	.long	_fault
vectorC6:	.long	_fault
vectorC7:	.long	_fault
vectorC8:	.long	_fault
vectorC9:	.long	_fault
vectorCA:	.long	_fault
vectorCB:	.long	_fault
vectorCC:	.long	_fault
vectorCD:	.long	_fault
vectorCE:	.long	_fault
vectorCF:	.long	_fault
vectorD0:	.long	_fault
vectorD1:	.long	_fault
vectorD2:	.long	_fault
vectorD3:	.long	_fault
vectorD4:	.long	_fault
vectorD5:	.long	_fault
vectorD6:	.long	_fault
vectorD7:	.long	_fault
vectorD8:	.long	_fault
vectorD9:	.long	_fault
vectorDA:	.long	_fault
vectorDB:	.long	_fault
vectorDC:	.long	_fault
vectorDD:	.long	_fault
vectorDE:	.long	_fault
vectorDF:	.long	_fault
vectorE0:	.long	_fault
vectorE1:	.long	_fault
vectorE2:	.long	_fault
vectorE3:	.long	_fault
vectorE4:	.long	_fault
vectorE5:	.long	_fault
vectorE6:	.long	_fault
vectorE7:	.long	_fault
vectorE8:	.long	_fault
vectorE9:	.long	_fault
vectorEA:	.long	_fault
vectorEB:	.long	_fault
vectorEC:	.long	_fault
vectorED:	.long	_fault
vectorEE:	.long	_fault
vectorEF:	.long	_fault
vectorF0:	.long	_fault
vectorF1:	.long	_fault
vectorF2:	.long	_fault
vectorF3:	.long	_fault
vectorF4:	.long	_fault
vectorF5:	.long	_fault
vectorF6:	.long	_fault
vectorF7:	.long	_fault
vectorF8:	.long	_fault
vectorF9:	.long	_fault
vectorFA:	.long	_fault
vectorFB:	.long	_fault
vectorFC:	.long	_fault
vectorFD:	.long	_fault
vectorFE:	.long	_fault
#vectorFF:	.long	_fault

/*
 *	The very last vector position is stolen to use as a version
 *	number for this boot code :-)
 */

_version:
.long	0x03100001

/*****************************************************************************/


/*
 *	This is the initial start up entry point.
 */

_start:
	nop				/* Filler */
	move.w	#0x2700, %sr		/* All interrupts off */


#ifndef CONFIG_VERDI
    #ifdef CONFIG_FLASH

        /* copy from flash to RAM */
        #ifdef CONFIG_SYSTEC_PLCCORE5484
        // #error "CONFIG_SYSTEC_PLCCORE5484"

        	move.l	#0xFE060000, %a0        /* source adress in flash */
        	move.l	#0x01000000, %a1        /* destination adress in RAM */
        	move.l	#0xFFFC, %d0            /* copy size: 256 Kb */

        #else
        /*---------------------------------------------------------------------------
        	2005/09/26  -rs
        	move.l	#0xE0000000, %a0
        	move.l	#0x1000000, %a1
        	move.l	#0x7FFC, %d0 // 128 Kb //
        ---------------------------------------------------------------------------*/
        	move.l	#0xFE100000, %a0        /* source adress in flash */
        	move.l	#0x01000000, %a1        /* destination adress in RAM */
        	move.l	#0xFFFC, %d0            /* copy size: 256 Kb */
        /*-------------------------------------------------------------------------*/
        #endif

        loop_copy:
        	move.l	%a0@+,%d1
        	move.l	%d1,%a1@+
        	subq.l	#1,%d0
        	bne.s	loop_copy

        	move.l	#real_start,%a0
        	jmp	(%a0)	/* Jump to colilo code start address in memory*/

    #endif  // #ifdef CONFIG_FLASH
#endif      // #ifndef CONFIG_VERDI


real_start:
	/*
	 *	Setup VBR, MBAR and RAMBAR first...
	 */
#if     defined(M5485) || defined(M5475)
	move.l  #0, %d0
#else
	move.l	#_vectors, %d0  	/* Set VBR address (into FLASH) */
#endif
	movec	%d0, %VBR

#if	defined(M5485) || defined(M5475)
	move.l	#(MCF_MBAR), %d0	/* Get MBAR address */
#else
	move.l	#(MCF_MBAR+1), %d0	/* Get MBAR address */
#endif
	movec	%d0, %MBAR		/* Set MBAR */

	/*** The 5249 has MBAR2 as well ***/
#ifdef MCF_MBAR2
	move.l	#(MCF_MBAR2+1), %d0	/* Get MBAR2 address */
	movec	%d0, #0xc0e		/* Set MBAR2 */
#endif

	move.l	#MEM_BUILTIN+0x1,%a0	/* Set SRAM base addr */
	movec	%a0, %RAMBAR0		/* Note: bit 0 is Validate */
#if     defined(M5485) || defined(M5475)
	move.l	#MEM_BUILTIN, %d0
	add.l	#INT_RAM_SIZE, %d0
	addq.l	#0x01, %d0
	move.c	%d0, %RAMBAR1		/* initialize second SRAM */
	move.l	#MEM_BUILTIN, %d0
	add.l	#INT_RAM_SIZE, %d0
	add.l	#INT_RAM_SIZE, %d0
	move.l	%d0, %a7
#else
#ifdef M5407
	move.l	#MEM_BUILTIN, %d0
	add.l	#INT_RAM_SIZE, %d0
	addq.l	#0x01, %d0
	move.c	%d0, %RAMBAR1
	move.l	#MEM_BUILTIN, %d0
	add.l	#INT_RAM_SIZE, %d0
	add.l	#INT_RAM_SIZE, %d0
	move.l	%d0, %a7
#else
	move.l	#MEM_BUILTIN+INT_RAM_SIZE, %a7 /* Stack in internal SRAM */
#endif
#endif

	/* The 5249 has two banks of built-in SRAM */
#ifdef MEM_BUILTIN2
	move.l	#MEM_BUILTIN2+0x1,%a0	/* Set SRAM base addr */
	movec	%a0, %RAMBAR1		/* Note: bit 0 is Validate */
	move.l	#MEM_BUILTIN2+INT_RAM_SIZE2, %a7 /* Move stack to larger SRAM bank */
#endif

	/* Disable BDM, this reduces EMI */
#if 0
	move.l  #MEM_BUILTIN,	%a0
	move.w  #0x2c80,	%a0@+
	move.w  #0x0002,	%a0@+
	move.w  #0x0000,	%a0@+
	move.w  #0x0000,	%a0@+
	move.l  #MEM_BUILTIN,	%a0
	wdebug  %a0@
#endif

	/*
	 *	Disable the cache. Don't want it on just yet.
	 */
	jsr	disableCache

	jsr mcf548x_init
	move.l	#__SP_INIT, %SP

	/*
	 *	Most of the setup work is done in main().
	 */
	jsr	start_main

	/*
	 *	Should never get here!
	 */
forever:
	jmp	forever


/****************************************************************************/

nop:
	nop				/* Small delay */
	rts

/****************************************************************************/

/*
 *	Jump into the image copied into RAM.
 */

startimage:
/*	move.w	#0x2000, %sr	*/	/* Enable All Interrupts */
	move.l	xfer_addr,%a0
	jmp	(%a0)			/* Jump to code start address */

/****************************************************************************/

/****************************************************************************/

/*
 *	Local fault routine. Shouldn't never get here. If we ever do then
 *	the boot up process has gone badly wrong. Enter the serial port
 *	download state (blinking LEDs so operator knows something is up).
 *
 *	It is possible that the stack pointer is bad, so we will set it
 *	back to a sane value here too.
 */

_fault:
    halt
	move.l	#MEM_BUILTIN+INT_RAM_SIZE, %a7	/* Reset stack to local SRAM */
	jsr	recover				/* Let recover clean up */
	jmp	_start				/* Should never get here */

/****************************************************************************/
#ifndef M5407
#ifndef M5485
#ifndef M5475
disableCache:
	move.l	#0x00000400,	%d0	/* Disable and invalidate cache */
	move.c	%d0, %CACR

	/* This takes 31 machine cycles */
	move.l	#0x00008020,	%d0	/* Completely disable cache */
	move.c	%d0,		%ACR0
	move.c	%d0,		%ACR1
	rts
#endif
#endif
#endif
