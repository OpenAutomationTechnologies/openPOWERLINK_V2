/*
 * File:	mcf548x_lo.s
 * Purpose:	Lowest level routines for mcf548x.
 *
 * Notes:	
 *
 */

#ifdef _UNDERSCORE_
#define __MBAR				___MBAR
#define __CORE_SRAM0		___CORE_SRAM0
#define __CORE_SRAM1		___CORE_SRAM1
#define __CORE_SRAM0_SIZE	___CORE_SRAM0_SIZE
#define __CORE_SRAM1_SIZE	___CORE_SRAM1_SIZE
#define __SP_INIT			___SP_INIT
#define mcf548x_init		_mcf548x_init
#define main				_main
#define	asm_set_ipl			_asm_set_ipl
#endif
 
	.extern __MBAR
	.extern __CORE_SRAM0
	.extern __CORE_SRAM1
	.extern __CORE_SRAM0_SIZE
	.extern __CORE_SRAM1_SIZE
    .extern __SP_INIT
    .extern _mcf548x_init
    .extern _main
	.extern asm_set_ipl
 
/*    .global asm_startmeup
    .global _asm_startmeup
*/
    .global cpu_cache_flush
    .global _cpu_cache_flush
	.global cpu_cache_disable
	.global	_cpu_cache_disable

	.text

/********************************************************************
 * This is the main entry point upon hard reset.
 */
/* ... */


/********************************************************************
 *	Routine to cleanly flush the cache, pushing all lines and 
 *	invalidating them.  This must be done to change the cache when 
 *	we have been operating in copyback mode (i.e. writes to a copyback 
 *	region are probably resident in the cache and not in the main store).
 */
_cpu_cache_flush:
cpu_cache_flush:
	nop					/* synchronize - flush store buffer */
	moveq.l	#0,%D0		/* init way counter */
	moveq.l #0,%D1		/* init set counter */
	move.l	%D0,%A0		/* init An */

flushloop:

	.word	0xF4E8		/* cpushl bc,(A0) -- push cache line */

	add.l	#0x0010,%A0	/* increment setindex by 1 */
	addq.l	#1,%D1		/* increment set counter */
	cmpi.l	#512,%D1		/* are sets for this line done? */
	bne		flushloop

	moveq.l	#0,%D1		/* set counter to zero again */
	addq.l	#1,%D0		/* increment to next line */
	move.l	%D0,%A0		/* set 0, line d0 into a0 as per cpushl */
	cmpi.l	#4,%D0
	bne		flushloop

	rts

/********************************************************************
 *	Routine to disable the cache completely
 */
_cpu_cache_disable:
cpu_cache_disable:

	move.l  #0x7,-(%SP)		/* Disable interrupts (set IPL = 7) */		
	jsr		asm_set_ipl
	lea.l	4(%SP),%SP
	move.l	%D0,%D1

	jsr		_cpu_cache_flush	/* flush the cache completely */	

	clr.l	%D0
	/* ACR0 off */
	.long   0x4e7b0004      /* movec d0,ACR0 */
    nop
    
	/* ACR1 off */
	.long   0x4e7b0005      /* movec d0,ACR1 */
    nop
    
	/* ACR2 off */
	.long   0x4e7b0006      /* movec d0,ACR2 */
    nop
    
	/* ACR3 off */
	.long   0x4e7b0007      /* movec d0,ACR3 */
    nop

	move.l	#0x01000100,%D0	/* Invalidate and disable cache */
	.long   0x4e7b0002      /* movec d0,cacr */
    nop
    	
	move.l  %D1,-(%SP)		/* Restore interrupt level */		
	jsr		asm_set_ipl
	lea.l	4(%SP),%SP

	rts

/********************************************************************/
	.end
