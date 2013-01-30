.global enableCache
.global disableCache
.text

enableCache:
	/*
	 *	Enable CPU internal cache.
	 */
#ifdef _UNUSED_
/*	move.l	#0x01040100, %d0*/		/* Invalidate whole cache */
/*	movec	%d0,%CACR*/
/*	nop
	
/*	move.l  #0x0000e400, %d0*/                /* setup ACR0..ACR3 */
/*	movec    %d0, %ITT0*/                     /* ACR0             */
/*	movec    %d0, %DTT0*/                     /* ACR2             */
/*	move.l   #0, %d0*/
/*	movec    %d0, %ITT1*/                     /* ACR1             */
/*	movec    %d0, %DTT1*/                     /* ACR3             */

	/* Enable cache */
/*	move.l	#0xa4098400, %d0*/		/* Write buffer, dflt precise */
/*	movec	%d0,%CACR*/
#endif
	nop
	rts

disableCache:
#ifdef _UNUSED_
/*	move.l  #0x01040100, %d0*/
/*	move.c  %d0,%CACR*/
/*	nop
/*	move.l  #0x00000000, %d0*/
/*	move.c  %d0,%CACR*/
/*	movec    %d0, %ITT0*/                     /* ACR0             */
/*	movec    %d0, %DTT0*/                     /* ACR2             */
/*	move.l   #0, %d0*/
/*	movec    %d0, %ITT1*/                     /* ACR1             */
/*	movec    %d0, %DTT1*/                     /* ACR3             */
#endif
	rts

